#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D1 — Auditoría integral de entrega del Avance 2 AMR logístico.

SOLO LECTURA de evidencias, CNN y Git; no ROS/Gazebo, no git add/commit/push/tag.
Escribe únicamente una carpeta NUEVA en ~/AMR_A2_entrega_local/.

Ubuntu / WSL:
    python3 /mnt/c/Users/Henry/Downloads/A2_D1_auditoria_integral_entrega_v02.py
    python3 A2_D1_auditoria_integral_entrega_v02.py --repo ~/AMR_Logistico_Fase_0

No supone que las llegadas impliquen navegación exclusivamente difusa ni que
la ausencia de una alarma geométrica demuestre cero contactos físicos Gazebo.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import zipfile
from collections import Counter
from pathlib import Path

BASE_B = 'A2_B2_rescate_geometria_astar/20261108_171723'
B1_BASE = 'A2_B1_pasillo_1m/20261108_100813/B1_3_bateria_5x'
IDS = {
    'B2': ('B2_rescate_integrado', ('20261108_173825', '20261108_174121',
           '20261108_174221', '20261108_174318', '20261108_174416')),
    'B3': ('B3_obstaculo_sorpresa_v02', ('20261108_195247', '20261108_195643',
           '20261108_195749', '20261108_195855', '20261108_195957')),
    'B4': ('B4_pasillo_angosto_v03', ('20261108_203551', '20261108_203955',
           '20261108_204054', '20261108_204150', '20261108_204247')),
}
C_EXPECT = {
    'C1': ('C1_auditoria_*_UTC', 'dictamen_C1.json', 'DEBILIDAD_IDENTIFICADA_Y_CUANTIFICADA'),
    'C2': ('C2_linea_base_*_UTC', 'dictamen_C2.json', 'LINEA_BASE_OPERACIONAL_DEFINIDA_CON_RESERVAS'),
    'C3': ('C3_evaluacion_*_UTC', 'dictamen_C3.json', 'REDISENO_HIBRIDO_ASISTIDO_OBSERVADO_CON_RESERVAS'),
    'C4': ('C4_cierre_*_UTC', 'dictamen_C4.json', 'CIERRE_DESCRIPTIVO_HIBRIDO_ASISTIDO_CON_RESERVAS'),
}
EXPECTED_ZIP_SHA = '9ac794dc4615254c9373f55c17969b495a581b1fb594552892c36d3beeaf7434'
SCHEMA = ('seccion', 'id', 'estado', 'criterio', 'detalle', 'archivo')


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for blob in iter(lambda: f.read(1024 * 1024), b''):
            h.update(blob)
    return h.hexdigest()


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def float_or_none(value):
    try:
        n = float(value)
        return n if math.isfinite(n) else None
    except (ValueError, TypeError):
        return None


def result_status(text):
    """Reconoce dictámenes masculinos/femeninos sin acreditar rechazos.

    B1 utiliza literalmente APROBADA_B1_NAVEGACION_GEOMETRIA y NO_APROBADA.
    El auditor v01 solo reconocía APROBADO, por eso reportaba B1=0/5.
    """
    t = str(text or '').upper()
    if re.search(r'NO[_\s-]*APROBAD[OA]|FALLO|ERROR|RECHAZAD[OA]|INVALID|FALSE', t):
        return False
    return bool(re.search(r'APROBAD[OA]|VALIDAD|EXITO|LLEGADA|COMPLETAD[OA]|\bOK\b|TRUE', t))


def run_git(repo, *argv):
    try:
        p = subprocess.run(['git', '-C', str(repo), *argv], capture_output=True,
                           text=True, timeout=12, check=False)
        if p.returncode != 0:
            return None, (p.stderr or p.stdout).strip()[:300]
        return p.stdout.strip(), None
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, str(exc)


class Auditor:
    def __init__(self, repo, entrega, cnn_zip=None):
        self.repo, self.entrega = repo, entrega
        self.cnn_zip_override = cnn_zip
        self.rows = []
        self.sources = {}
        self.blocks = []
        self.notes = []
        self.counts = Counter()
        self.cdirs = {}
        self.b1_acc = []
        self.bags = 0

    def add(self, section, item, status, criterion, detail, file=None):
        path = str(file) if file is not None else ''
        self.rows.append(dict(zip(SCHEMA, (section, item, status, criterion, str(detail), path))))
        if status == 'BLOQUEO':
            self.blocks.append(f'{section}/{item}: {detail}')
        elif status == 'REVISAR':
            self.notes.append(f'{section}/{item}: {detail}')

    def source(self, p, sha=True):
        if p.is_file():
            self.sources[str(p.resolve())] = (p.stat().st_size, digest(p) if sha else '')
            return True
        return False

    def bag_files(self, run):
        files = [p for p in run.rglob('*.db3')
                 if p.is_file() and p.stat().st_size > 4096]
        if files:
            self.bags += 1
        return files

    def audit_b1(self):
        base = self.repo/'resultados'/B1_BASE
        if not base.is_dir():
            self.add('B1', 'base', 'BLOQUEO', 'B1 pasillo 1m',
                     'No se localizó carpeta original B1; no imputar 5 éxitos', base)
            return
        summaries = [base/'B1_3_resumen_5_repeticiones.json']
        summaries += list(base.glob('*resumen*5*repeticiones*.json'))
        summaries += list(base.parent.glob('*resumen*5*repeticiones*.json'))
        summary = next((p for p in summaries if p.is_file()), None)
        if summary:
            self.source(summary)
            try:
                global_result = read_json(summary)
                self.add('B1', 'consolidado', 'OK', 'JSON global disponible',
                         'Resumen global encontrado; se valida además cada carpeta individual', summary)
            except (OSError, json.JSONDecodeError) as exc:
                global_result = None
                self.add('B1','consolidado','BLOQUEO','JSON global legible',str(exc),summary)
        else:
            global_result = None
            self.add('B1','consolidado','REVISAR','Resumen global',
                     'No hallado el consolidado B1; revisar nombre/carpeta',base)
        csvs = list(base.glob('*resultados*5*repeticiones*.csv'))
        csvs += list(base.parent.glob('*resultados*5*repeticiones*.csv'))
        for p in csvs:
            self.source(p)
        if not csvs:
            self.add('B1','tabla_global','REVISAR','CSV de 5 repeticiones',
                     'No se halló CSV global B1',base)
        # Incluye carpetas con intentos fallidos: JAMAS se acreditan por el nombre.
        candidates = sorted(p for p in base.rglob('B1_3_repeticion_*') if p.is_dir()
                            and not any(x.name.startswith('B1_3_repeticion_') for x in p.parents))
        if not candidates:
            self.add('B1','repeticiones','BLOQUEO','Cinco carpetas individuales',
                     'No aparecen carpetas B1_3_repeticion_*',base)
            return
        review = []
        for p in candidates:
            summaryfile = p/'resumen.json'
            metfile = p/'B1_3_metricas.json'
            extra_metrics = sorted(p.glob('*metricas*.json'))
            jf = (summaryfile if summaryfile.is_file() else
                  metfile if metfile.is_file() else
                  extra_metrics[0] if extra_metrics else None)
            if not jf:
                self.add('B1',p.name,'EXCLUIDO','Intento no acreditado',
                         'Sin resumen.json o B1_3_metricas.json: NO se cuenta',p)
                continue
            self.source(jf)
            try:
                data = read_json(jf)
                if not isinstance(data, dict):
                    raise ValueError('JSON individual no es un objeto')
            except (OSError, ValueError) as exc:
                self.add('B1',p.name,'EXCLUIDO','JSON individual legible',str(exc),jf)
                continue
            bags = self.bag_files(p)
            nested = [data]+[data[k] for k in ('metricas','resumen','resultado','validacion')
                              if isinstance(data.get(k),dict)]
            aliases = ('distancia_final_meta_m','error_final_m','error_meta_m',
                       'error_final','distancia_a_meta_m','error_llegada_m',
                       'distancia_final_m','error_posicion_m')
            error = next((float_or_none(obj.get(k)) for obj in nested for k in aliases
                          if obj.get(k) is not None),None)
            mode = ' '.join(str(obj.get(k,'')) for obj in nested for k in
                            ('estado','resultado','dictamen','evaluacion','aprobado',
                             'llegada','llego_meta','llegada_meta','exito','cumple'))
            identified = result_status(mode)
            adverse = bool(re.search(r'NO[_\s-]*APROBAD[OA]|FALLO|ERROR|FALSE|RECHAZAD[OA]', mode.upper()))
            if adverse:
                status='EXCLUIDO';why='Dictamen individual de rechazo, no se acredita'
            elif not bags:
                status='REVISAR';why='Sin rosbag DB3 no vacío localizable'
            elif identified and error is not None and error<=0.15001:
                status='OK';why='Resumen con marca positiva y rosbag presente'
                self.b1_acc.append(p)
            else:
                status='REVISAR';why='No se puede acreditar inequívocamente mediante campos del JSON; revisar dictamen individual'
            self.add('B1',p.name,status,'Llegada individual + rosbag',
                     f'{why}; error_final={error}; DB3={len(bags)}; estado={mode[:130]}',jf)
            if status == 'REVISAR':
                review.append(p.name)
        count = len(self.b1_acc)
        self.counts['B1'] = min(5,count)
        if count == 5:
            self.add('B1','resultado','OK','5 repeticiones acreditadas',
                     f'{count}/5 con resumen positivo y rosbag local; intentos no acreditados excluidos',base)
        else:
            self.add('B1','resultado','REVISAR','5 repeticiones acreditadas',
                     f'{count}/5 acreditadas automáticamente; verificar estructura de {review} y resumen global',base)
        if global_result and isinstance(global_result,dict):
            # La cifra global nunca sustituye la auditoría individual.
            self.add('B1','alcance','INFO','Evidencias individuales',
                     'La existencia del resumen global no acredita por sí sola las cinco repeticiones',summary)

    def audit_b234(self):
        base = self.repo/'resultados'/BASE_B
        for stage,(dirname,ids) in IDS.items():
            good=0
            for stamp in ids:
                root = base/dirname/stamp
                sf = root/'resumen.json'
                label=f'{stage}-{stamp[-6:]}'
                if not sf.is_file():
                    self.add(stage,label,'BLOQUEO','Resumen individual',
                             'No existe resumen.json de la repetición acreditada',sf)
                    continue
                self.source(sf)
                try:
                    data=read_json(sf)
                    if not isinstance(data,dict): raise ValueError('JSON inesperado')
                except (ValueError,OSError) as exc:
                    self.add(stage,label,'BLOQUEO','JSON válido',str(exc),sf)
                    continue
                for x in ('control.csv','odometria.csv'):
                    pp = root/x
                    if not pp.is_file() or pp.stat().st_size<=60:
                        self.add(stage,label+'-'+x,'BLOQUEO','Control/odometría',
                                 'Archivo ausente o vacío',pp)
                    else:
                        self.source(pp)
                bags=self.bag_files(root)
                state=str(data.get('estado',''))
                error=float_or_none(data.get('distancia_final_meta_m'))
                clear=float_or_none(data.get('distancia_minima_a_colisiones_sdf_m'))
                clean=(data.get('fallos') in ([],None))
                ros=(data.get('rosbag_valido') is True and bool(bags))
                expected={'B2':'LLEGADA_Y_ROSBAG_VALIDADOS_CONTACTOS_NO_MEDIDOS',
                          'B3':'B3_PILOTO_HIBRIDO_ASISTIDO_APROBADO_CONTACTOS_NO_MEDIDOS',
                          'B4':'B4_LLEGADA_Y_ROSBAG_VALIDOS_CONTACTOS_NO_MEDIDOS'}[stage]
                extra=True
                if stage=='B3':
                    extra = (data.get('obstaculo_dinamico_insertado') is True
                             and data.get('evasion_hibrida_asistida_demostrada') is True
                             and data.get('bypass_completado') is True)
                if stage=='B4':
                    extra = (data.get('pasillo_angosto_atravesado') is True
                             and float_or_none(data.get('ancho_pasillo_m')) is not None
                             and abs(float_or_none(data.get('ancho_pasillo_m'))-0.6)<1e-3)
                success=(state==expected and clean and ros and error is not None
                         and error<=0.15001 and clear is not None and clear>0.255 and extra
                         and (root/'control.csv').is_file() and (root/'control.csv').stat().st_size>60
                         and (root/'odometria.csv').is_file() and (root/'odometria.csv').stat().st_size>60)
                status='OK' if success else 'BLOQUEO'
                if success: good+=1
                self.add(stage,label,status,'Llegada + márgenes + ROSBAG',
                         f'estado={state}; error={error}; despeje={clear}; '+
                         f'rosbag_valido={data.get("rosbag_valido")}; db3={len(bags)}; controles={extra}',sf)
            self.counts[stage]=good
            self.add(stage,'resultado','OK' if good==5 else 'BLOQUEO',
                     'Cinco ejecuciones acreditadas', f'{good}/5 verificadas',base/dirname)
        self.add('B3','reserva','INFO','Tipo de control',
                 'Evasión híbrida asistida: no demuestra mejora aislada de Mamdani')
        self.add('B4','reserva','INFO','Tipo de control',
                 'Centrado geométrico asistido, con A3/A4/FIS en modo sombra')
        self.add('B1-B4','contactos','REVISAR','Contactos físicos en Gazebo',
                 'Los ensayos reportan despeje geométrico; no hay auditoría directa de contactos')

    def audit_c(self):
        for stage,(pat,file,expected) in C_EXPECT.items():
            folders = sorted((p for p in self.entrega.glob(pat)
                              if p.is_dir() and (p/file).is_file()),
                             key=lambda p:p.stat().st_mtime)
            if not folders:
                self.add(stage,'carpeta','BLOQUEO','Dictamen existente',
                         f'No se halló {pat} / {file}',self.entrega)
                continue
            latest=folders[-1];self.cdirs[stage]=latest
            try:
                d=read_json(latest/file)
            except (ValueError,OSError) as exc:
                self.add(stage,'dictamen','BLOQUEO','JSON válido',str(exc),latest/file)
                continue
            value=(d.get('dictamen_C1') if stage=='C1' else
                   d.get('dictamen') if stage=='C2' else
                   d.get('resultado',{}).get('resultado') if stage=='C3' else d.get('dictamen'))
            if value!=expected:
                self.add(stage,'dictamen','BLOQUEO','Dictamen esperado',
                         f'Recibido {value!r}; esperado {expected!r}',latest/file)
            else:
                self.add(stage,'dictamen','OK','Dictamen con reservas',value,latest/file)
            self.source(latest/file)
            required=['informe_'+stage+'.md','tabla_'+stage+'.csv','manifiesto_SHA256.csv']
            if stage=='C4': required+=['reporte_visual_C4.html']
            for n in required:
                p=latest/n
                if not p.is_file() or p.stat().st_size==0:
                    self.add(stage,n,'BLOQUEO','Documento obligatorio',
                             'Archivo ausente o vacío',p)
                else: self.source(p)
            manifest=latest/'manifiesto_SHA256.csv'
            if manifest.is_file():
                try:
                    with manifest.open(newline='',encoding='utf-8-sig') as fh:
                        rd=csv.DictReader(fh)
                        records=list(rd)
                    checked=0;missing=[];wrong=[];paths=set()
                    for row in records:
                        name=row.get('archivo_fuente') or row.get('archivo')
                        expected_hash=row.get('sha256')
                        if not name or not expected_hash:
                            wrong.append('fila incompleta');continue
                        src=Path(name).expanduser()
                        if str(src) in paths:
                            wrong.append('duplicado '+str(src));continue
                        paths.add(str(src))
                        if not src.is_file():
                            missing.append(str(src));continue
                        if digest(src).lower()!=expected_hash.lower():
                            wrong.append(str(src));continue
                        checked+=1
                    if missing or wrong or checked==0:
                        self.add(stage,'hashes','BLOQUEO','Integridad SHA256',
                                 f'{checked}/{len(records)} correctos; ausentes={missing[:2]}; errores={wrong[:2]}',manifest)
                    else:
                        self.add(stage,'hashes','OK','Integridad SHA256',
                                 f'{checked}/{len(records)} referencias verificadas',manifest)
                except (OSError,ValueError,KeyError) as exc:
                    self.add(stage,'hashes','BLOQUEO','Manifiesto legible',str(exc),manifest)
            if value==expected and any(r['seccion']==stage and r['id']=='hashes' and r['estado']=='OK' for r in self.rows) and not any(r['seccion']==stage and r['estado']=='BLOQUEO' for r in self.rows):
                self.counts[stage]=1

    def find_cnn_zip(self):
        """Localiza el paquete por ruta explícita o en carpetas plausibles.

        No hace exploración recursiva indiscriminada del disco, y verifica siempre
        SHA256 y contenido antes de acreditar el paquete.
        """
        if self.cnn_zip_override is not None:
            return self.cnn_zip_override.expanduser().resolve()
        exact = 'A2_dataset_600_y_CNN_entrenada.zip'
        preferred = self.entrega/exact
        if preferred.is_file():
            return preferred
        roots = [self.entrega, Path.home(), Path.home()/'Downloads',
                 self.repo.parent, Path('/mnt/c/Users/Henry/Downloads')]
        # En WSL el perfil Windows no siempre se llama Henry.
        windows_root = Path('/mnt/c/Users')
        if windows_root.is_dir():
            roots += [p/'Downloads' for p in windows_root.iterdir() if p.is_dir()
                      and p.name not in ('Default','Public','Default User','All Users')]
        patterns = (exact, 'A2_dataset*.zip', '*CNN*entren*.zip',
                    '*/'+exact, '*/A2_dataset*.zip')
        seen = set(); found = []
        for root in roots:
            if not root.is_dir():
                continue
            for pattern in patterns:
                for candidate in root.glob(pattern):
                    if not candidate.is_file() or candidate in seen:
                        continue
                    seen.add(candidate); found.append(candidate)
        if not found:
            return preferred
        # Si existe un duplicado con el SHA esperado, se prefiere a una
        # copia truncada/renombrada. Se limitan los hashes a 25 candidatos.
        for candidate in found[:25]:
            try:
                if digest(candidate).lower() == EXPECTED_ZIP_SHA:
                    return candidate
            except OSError:
                continue
        return found[0]

    def audit_cnn(self):
        evidence=self.repo/'docs/evidencias_A2/R22_R23'
        req=['resumen_dataset.json','manifiesto.csv','configuracion_optica.json',
             'entrenamiento/reporte_entrenamiento.json','entrenamiento/historial.csv',
             'entrenamiento/matriz_confusion.csv','entrenamiento/mosaico_control_calidad.png']
        for name in req:
            path=evidence/name
            if self.source(path):
                self.add('CNN',name,'OK','Evidencia ligera', 'Archivo existente y SHA256 calculado',path)
            else:
                self.add('CNN',name,'BLOQUEO','Evidencia ligera', 'Archivo ausente en repo local',path)
        summary=evidence/'resumen_dataset.json'
        if summary.is_file():
            try:
                d=read_json(summary)
                self.add('CNN','dataset_metadatos','INFO','Resumen de captura',
                         'Datos de origen (sin extrapolar): '+str(d)[:350],summary)
            except (OSError,ValueError) as exc:
                self.add('CNN','dataset_metadatos','REVISAR','JSON dataset',str(exc),summary)
        cnnroot=self.repo/'resultados/A2_R22_R23_CNN_integral_v6_contexto/20261108_004923'
        if not cnnroot.is_dir():
            self.add('CNN','archivos_origen','REVISAR','600 PNG originales',
                     'No existe directorio local de captura esperado; comprobar ZIP externo',cnnroot)
        else:
            png=[p for p in cnnroot.rglob('*.png') if p.is_file()]
            # Puede haber mosaicos PNG de informes; no se cuentan todos como dataset.
            self.add('CNN','directorio_origen','INFO','Captura existente',
                     f'Directorio presente; {len(png)} PNG encontrados en total (incluye posibles mosaicos)',cnnroot)
        zip_path=self.find_cnn_zip()
        if zip_path.is_file():
            try:
                h=digest(zip_path)
                with zipfile.ZipFile(zip_path) as z:
                    bad=z.testzip()
                    names=z.namelist()
                    png=sum(x.lower().endswith('.png') for x in names)
                    keras=sum(x.lower().endswith('.keras') for x in names)
                matched = h==EXPECTED_ZIP_SHA and bad is None and png>=600 and keras>=1
                self.add('CNN','paquete_completo','OK' if matched else 'BLOQUEO',
                         'ZIP con dataset + modelo',
                         f'SHA256={h}; png={png}; modelos_keras={keras}; integridad_ZIP={bad or "OK"}; '+
                         f'SHA_esperado={h==EXPECTED_ZIP_SHA}',zip_path)
                self.source(zip_path,sha=False)  # Hash ya calculado; evitar repetirlo.
                self.sources[str(zip_path.resolve())]=(zip_path.stat().st_size,h)
            except (OSError,ValueError,zipfile.BadZipFile) as exc:
                self.add('CNN','paquete_completo','BLOQUEO','ZIP íntegro',str(exc),zip_path)
        else:
            self.add('CNN','paquete_completo','BLOQUEO','ZIP disponible',
                     'Paquete CNN no localizado en rutas plausibles; usar --cnn-zip RUTA si se movió. '
                     'Sin un ZIP íntegro y accesible al docente no se cierra D2.',zip_path)
        self.add('CNN','revision_visual','REVISAR','Control visual por persona',
                 'Abrir mosaico_control_calidad.png; no se acredita inspección visual automáticamente',
                 evidence/'entrenamiento/mosaico_control_calidad.png')
        self.add('CNN','distribucion','REVISAR','Acceso para docente',
                 'ZIP local no implica que el docente pueda descargarlo; adjuntar o publicar enlace verificable',zip_path)

    def audit_git(self):
        repo=self.repo
        branch,err=run_git(repo,'rev-parse','--abbrev-ref','HEAD')
        if err:
            self.add('GIT','repositorio','BLOQUEO','Repositorio Git',err,repo);return
        self.add('GIT','rama','OK' if branch=='desarrollo-avance2' else 'BLOQUEO',
                 'Rama de trabajo correcta',str(branch),repo)
        head,_=run_git(repo,'rev-parse','HEAD')
        self.add('GIT','HEAD','INFO','Commit local',head or 'desconocido')
        status,err=run_git(repo,'status','--porcelain=v1','--untracked-files=normal')
        if err:
            self.add('GIT','estado_local','REVISAR','Estado Git',err)
        elif status:
            self.add('GIT','estado_local','REVISAR','Cambios sin publicar',
                     'Cambios locales detectados: '+str(len(status.splitlines()))+' entradas; revisar antes de commit')
        else:
            self.add('GIT','estado_local','OK','Árbol de trabajo',
                     'Sin cambios locales por registrar; no confirma publicación de evidencias externas')
        tag,_=run_git(repo,'tag','--list','avance2')
        self.add('GIT','tag_local','INFO' if tag else 'REVISAR','Tag local avance2',
                 'Existe localmente' if tag else 'No existe tag local avance2')
        remote,_=run_git(repo,'remote','get-url','origin')
        self.add('GIT','remoto','INFO','Origin',remote or 'No configurado')
        self.add('GIT','scripts_de_prueba','REVISAR','Incorporación a repo',
                 'Comprobar que ejecutores B2–B4 y scripts C1–C4 usados realmente estén añadidos al repositorio, no solo en Descargas')
        self.add('GIT','readme','REVISAR','Coherencia README',
                 'Reconciliar descripciones antiguas de CNN pendiente y resultados B1–B4 con la evidencia actual',repo/'README.md')
        self.add('GIT','push_y_tag_remoto','REVISAR','Verificación remota',
                 'No se hacen conexiones ni pushes: D5/D6 requieren verificación posterior en GitHub')
        self.add('ENTREGA','informe_2_paginas','REVISAR','Informe final',
                 'D4: verificar que exista y tenga exactamente dos páginas y se corresponda con las evidencias')
        self.add('ENTREGA','blackboard','REVISAR','Confirmación de envío',
                 'D7: pendiente de cargar archivos y comprobar acuse Blackboard')

    def write(self, out):
        out.mkdir(parents=True,exist_ok=False)
        tab=out/'matriz_D1.csv'
        with tab.open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=SCHEMA);w.writeheader();w.writerows(self.rows)
        with (out/'manifiesto_SHA256.csv').open('w',newline='',encoding='utf-8') as f:
            w=csv.writer(f);w.writerow(('archivo_fuente','bytes','sha256'))
            for path,(n,h) in sorted(self.sources.items()):w.writerow((path,n,h))
        all_b=sum(self.counts.get(b,0) for b in ('B1','B2','B3','B4'))
        d={
            'etapa':'D1','generado_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
            'repo':str(self.repo),'entrega':str(self.entrega),
            'ensayos_b1_b4_individualmente_acreditados':all_b,
            'ensayos_previstos':20,
            'por_escenario':{b:self.counts.get(b,0) for b in ('B1','B2','B3','B4')},
            'cierres_C':{c:bool(self.counts.get(c,0)) for c in C_EXPECT},
            'rosbags_db3_presentes_numero_ejecuciones':self.bags,
            'rosbags_reproducidos':False,'contactos_gazebo_medidos':False,
            'nota_rosbags':'Incluye el intento B1 NO_APROBADA, no equivale a 21 éxitos',
            'comparacion_B3_pareada':False,'b3_b4_control_exclusivamente_difuso':False,
            'fuentes_hash_generado_n':len(self.sources),
            'bloqueos':self.blocks,'revisiones_manuales':self.notes,
            'dictamen':'D1_EVIDENCIA_VERIFICADA_CON_RESERVAS' if not self.blocks and all_b==20 and
                       all(self.counts.get(c,0) for c in C_EXPECT) else 'D1_PENDIENTES_O_INDETERMINACIONES',
            'alcance':'Auditoría local estática: NO confirma push/tag remoto, entrega Blackboard, pruebas de contactos ni replay de rosbag.'
        }
        (out/'dictamen_D1.json').write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
        md=['# D1 — Auditoría integral de entrega | AMR logístico', '',
            '**Dictamen:** `'+d['dictamen']+'`', '',
            '## Batería oficial de escenarios', '',
            '| Escenario | Repeticiones acreditadas sobre 5 | Observación |',
            '|---|---:|---|',
            '| B1 | %d/5 | Pasillo 1 m; excluir intento fallido sin evidencia |'%self.counts.get('B1',0),
            '| B2 | %d/5 | A* y desvío estático preplanificado |'%self.counts.get('B2',0),
            '| B3 | %d/5 | Evasión híbrida con subobjetivos, NO Mamdani aislado |'%self.counts.get('B3',0),
            '| B4 | %d/5 | Centrado geométrico asistido, A3/A4/FIS en sombra |'%self.counts.get('B4',0),
            '**Total acreditado automáticamente: %d/20.**'%all_b,'',
            '## Rediseño', '',
            'C1–C4: '+', '.join('%s=%s'%(k,'localizado' if self.counts.get(k,0) else 'pendiente')
                             for k in C_EXPECT)+'.', '',
            'El antes/después B3 es descriptivo (un fallo original y cinco llegadas híbridas),\n'
            'no es comparación A/B pareada, y no demuestra mejora aislada de Mamdani.', '',
            '## Reservas metodológicas', '',
            '- Rosbags: se verificó presencia de SQLite `.db3`, NO su reproducción.',
            '- Contactos físicos Gazebo: no medidos directamente.',
            '- El ZIP CNN requiere distribución accesible para el docente y revisión visual del mosaico.',
            '- `git status` local no confirma publicación remota ni creación del tag remoto.', '',
            '## Bloqueos y faltantes', '']
        md += ['- '+x for x in self.blocks] or ['- Ningún bloqueo automático.']
        md += ['', '## Revisiones y entregables pendientes','']
        md += ['- '+x for x in self.notes] or ['- Ninguna revisión automática.']
        md += ['', '## Próximos pasos','',
              '1. D2: abrir mosaico CNN, comprobar paquete completo y acceso externo.',
              '2. D3: integrar B1–B4 y dictámenes C1–C4 al cierre.',
              '3. D4: preparar y validar informe académico de dos páginas.',
              '4. D5: hacer commit/push controlado y verificar GitHub Actions.',
              '5. D6: crear `avance2` tras verificar el commit y confirmar tag remoto.',
              '6. D7: preparar Blackboard y conservar acuse de entrega.', '',
              'Ver detalles por ensayo y evidencia en `matriz_D1.csv`; fuentes y SHA en `manifiesto_SHA256.csv`.']
        (out/'informe_D1.md').write_text('\n'.join(md)+'\n',encoding='utf-8')
        checklist=['# D1 — Lista de preparación para el Avance 2','',
                   '- [ ] B1–B4: comprobar 20/20 registros individuales y sus rosbags',
                   '- [ ] C1–C4: validar dictámenes y manifiestos SHA256',
                   '- [ ] Revisar visualmente mosaico CNN',
                   '- [ ] Subir el ZIP completo de las 600 imágenes y CNN',
                   '- [ ] Integrar resultados y limitaciones del rediseño',
                   '- [ ] Validar el informe de dos páginas',
                   '- [ ] Revisar git diff, commit, push y GitHub Actions',
                   '- [ ] Crear/verificar tag remoto `avance2`',
                   '- [ ] Blackboard: subir entrega y obtener comprobante','']
        (out/'checklist_entrega_D1.md').write_text('\n'.join(checklist),encoding='utf-8')
        return d


def main(argv=None):
    p=argparse.ArgumentParser(description='D1: auditoría estática, conservadora y sin modificaciones de fuentes')
    p.add_argument('--repo',type=Path,default=Path.home()/'AMR_Logistico_Fase_0')
    p.add_argument('--entrega',type=Path,default=Path.home()/'AMR_A2_entrega_local')
    p.add_argument('--salida',type=Path)
    p.add_argument('--cnn-zip',type=Path,
                   help='Ruta explícita al ZIP original de 600 imágenes y modelo CNN')
    a=p.parse_args(argv)
    repo=a.repo.expanduser().resolve(); entrega=a.entrega.expanduser().resolve()
    if not repo.is_dir():
        p.error('Repositorio no existe: '+str(repo))
    if not entrega.is_dir():
        p.error('No existe carpeta de entregas C1–C4: '+str(entrega))
    if a.salida:
        out=a.salida.expanduser().resolve()
    else:
        stamp=dt.datetime.now(dt.timezone.utc).strftime('%Y%m%d_%H%M%S_%f_UTC')
        out=entrega/('D1_auditoria_integral_'+stamp)
    if out.exists():
        p.error('La salida ya existe; no se sobrescribe: '+str(out))
    if repo==out or repo in out.parents:
        p.error('No escribir resultados dentro del repositorio')
    au=Auditor(repo,entrega,cnn_zip=a.cnn_zip)
    au.audit_b1();au.audit_b234();au.audit_c();au.audit_cnn();au.audit_git()
    d=au.write(out)
    print('=== D1 — AUDITORIA INTEGRAL SOLO LECTURA ===')
    print('B1–B4 verificados por escenario:',', '.join(f'{k}={d["por_escenario"][k]}/5' for k in ('B1','B2','B3','B4')))
    print('TOTAL:',d['ensayos_b1_b4_individualmente_acreditados'],'/20')
    print('C1–C4:',', '.join(f'{k}={"OK" if v else "REVISAR"}' for k,v in d['cierres_C'].items()))
    print('ROSBAG: archivos DB3 presentes en',d['rosbags_db3_presentes_numero_ejecuciones'],
          'intentos (incluye B1 no aprobado); no reproducidos')
    print('DICTAMEN:',d['dictamen'])
    print('BLOQUEOS:',len(d['bloqueos']),'| REVISIONES MANUALES:',len(d['revisiones_manuales']))
    for x in d['bloqueos'][:8]:print('[BLOQUEO]',x)
    for x in d['revisiones_manuales'][:8]:print('[REVISAR]',x)
    print('SALIDA:',out)
    print('ARCHIVOS: informe_D1.md, matriz_D1.csv, dictamen_D1.json, manifiesto_SHA256.csv, checklist_entrega_D1.md')
    print('NOTA: No publica Git ni valida Blackboard; no demuestra contactos ni replay de rosbags.')
    return 0


if __name__=='__main__':
    try:raise SystemExit(main())
    except (OSError,ValueError,KeyError,zipfile.BadZipFile) as ex:
        print('[ERROR AUDITOR D1]',ex,file=sys.stderr)
        raise SystemExit(2)
