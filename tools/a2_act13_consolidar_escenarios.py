#!/usr/bin/env python3
"""Auditoría documental A2 ACT13 de B1-B4, usando exclusivamente datos archivados.

--write: genera solo nuevos artefactos de ACT13 (no modifica las fuentes).
--check: verifica que los artefactos reproduzcan exactamente las fuentes y
         que los archivos fuente sean idénticos al tag inmutable 'final' si hay Git.
Sin dependencias externas; no vuelve a ejecutar ROS/Gazebo ni inferencia Mamdani.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import statistics as st
import subprocess
import sys
from xml.sax.saxutils import escape

ROOT = Path(__file__).resolve().parent.parent
SRC = Path('docs/evidencias_A2')
BASE = SRC / 'ESCENARIOS_B1_B4'
A17 = SRC / 'A17_cierre_20261109'
OUT = Path('docs')
FIG = OUT / 'evidencias_A2/ACT13'
FIELDS_MAIN = ['escenario', 'repeticion', 'origen_fuente', 'estado', 'llegada',
               'contactos_pares', 'contactos_episodios', 'cobertura',
               'error_final_m', 'recorrido_m', 'despeje_sdf_min_m',
               'desviacion_max_m', 'tiempo_llegada_s', 'oscilaciones_omega',
               'comandos_difuso_activos', 'modo_real', 'fuente_relativa',
               'observaciones']
FIELDS_SUPP = ['escenario','repeticion','serie','metodo_tiempo','duracion_s',
               'recorrido_m','desplazamiento_neto_m','error_final_m','despeje_geometrico_m',
               'desviacion_max_m','variacion_absoluta_omega_rad_s',
               'inversiones_signo_omega_umbral_0p05','oscilaciones_omega_reportadas','eventos_contactos_directos',
               'observacion_contactos','modo_real','fuente_resumen','fuente_control']


def readcsv(p):
    with (ROOT / p).open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def readjson(p):
    return json.loads((ROOT / p).read_text(encoding='utf-8-sig'))


def number(x, default=''):
    if x is None or x == '':
        return default
    return float(x)


def fmt(x, decimals=4):
    return 'N/D' if x in ('', None, 'N/D') else f'{float(x):.{decimals}f}'


def avg(rows, field):
    v = [float(r[field]) for r in rows if r.get(field, '') not in ('', None)]
    return st.mean(v) if v else None


def csvbytes(rows, fields):
    s = io.StringIO(newline='')
    w = csv.DictWriter(s, fieldnames=fields, extrasaction='ignore', lineterminator='\n')
    w.writeheader()
    w.writerows(rows)
    return s.getvalue().encode('utf-8')


def sha_bytes(b):
    return hashlib.sha256(b).hexdigest()


def git_blob_sha(b):
    return hashlib.sha1(b'blob ' + str(len(b)).encode() + b'\0' + b).hexdigest()


def collect():
    paths=set([str(A17 / 'acta_cierre.md'), str(BASE / 'indice_publicacion.csv')])
    main=[];supp=[]; historical_notes=[]
    for b in ('B1','B2','B3','B4'):
        path = A17 / f'tabla_{b}_fuente.csv'
        paths.add(str(path))
        rows=readcsv(path)
        if len(rows)!=5: raise RuntimeError(f'{b}: A17 no contiene 5 ensayos')
        for r in rows:
            n=int(r['intento'])
            if n not in range(1,6): raise RuntimeError(f'{b}: índice fuera de rango')
            if r['aprobado'].lower()!='true' or r['cobertura']!='VALIDA':
                raise RuntimeError(f'{b}-{n}: ensayo no acreditado o cobertura inválida')
            if int(r['contactos_pares'])!=0 or int(r['contactos_episodios'])!=0:
                raise RuntimeError(f'{b}-{n}: contacto distinto de cero')
            if b in ('B2','B3','B4') and r['rosbag_valido'].lower()!='true':
                raise RuntimeError(f'{b}-{n}: rosbag no marcado válido')
            mode = {'B1':'RUTA A* (sin evidencia de mando DIFUSO en A17)',
                    'B2':'A* y desvío geométrico (DIFUSO no actuó)',
                    'B3':'HÍBRIDO ASISTIDO (Mamdani activo; no puro)',
                    'B4':'CENTRADO GEOMÉTRICO (FIS en sombra)'}[b]
            if b=='B2' and (r['actuacion_DIFUSO_observada']!='False' or int(r['comandos_reales_modo_DIFUSO'])!=0):
                raise RuntimeError('B2: el modo difuso no coincide con la fuente')
            if b=='B3' and (r['actuacion_DIFUSO_observada']!='True' or int(r['comandos_reales_modo_DIFUSO'])<=0):
                raise RuntimeError('B3: falta actuación DIFUSO registrada')
            if b=='B4' and 'SOMBRA' not in r['modo_control']:
                raise RuntimeError('B4: FIS sombra no identificado')
            main.append(dict(
                escenario=b,repeticion=n,origen_fuente='A12/A17 (corte acreditado)',
                estado=r['estado'],llegada='1',contactos_pares=r['contactos_pares'],
                contactos_episodios=r['contactos_episodios'],cobertura=r['cobertura'],
                error_final_m=r['error_meta_m'],recorrido_m=r.get('trayectoria_m',''),
                despeje_sdf_min_m=r.get('min_despeje_sdf_m',''),
                desviacion_max_m=r.get('max_desviacion_m',''),
                tiempo_llegada_s='',oscilaciones_omega='',
                comandos_difuso_activos=r.get('comandos_reales_modo_DIFUSO',''),
                modo_real=mode,fuente_relativa=str(path),
                observaciones= 'Sin tiempos ni oscilaciones publicados en A17; no trasvasar desde otras corridas' if b!='B4' else 'Despeje/desviación publicados; sin tiempo/oscilaciones A17'
            ))
    main.sort(key=lambda x:(x['escenario'],int(x['repeticion'])))

    # Segunda serie B1 independiente (5 repeticiones) — no son los cinco pilotos A17.
    b1base=BASE/'B1'
    b1_files=sorted(p for p in (ROOT/b1base).glob('0[1-5]_*/resumen.json'))
    if len(b1_files)!=5: raise RuntimeError('B1 histórico: no están los cinco archivos')
    for p in b1_files:
        path=p.relative_to(ROOT);paths.add(str(path));d=readjson(path)
        m=d['metricas'];n=int(d['repeticion'])
        if not d['estado'].startswith('APROBADA'):raise RuntimeError('B1 histórico sin aprobación')
        supp.append(dict(escenario='B1',repeticion=n,
             serie='B1_3 (histórica distinta del B1 A17)',
             metodo_tiempo='tiempo_llegada_s_reloj_real en resumen B1_3',
             duracion_s=fmt(m['tiempo_llegada_s_reloj_real'],6),
             recorrido_m='',desplazamiento_neto_m=fmt(d['distancia_neta_m'],6),
             error_final_m=fmt(m['error_final_m'],6),
             despeje_geometrico_m=fmt(m.get('distancia_minima_robot_obstaculo_m'),6),
             desviacion_max_m=fmt(m.get('desviacion_maxima_m'),6),
             variacion_absoluta_omega_rad_s=fmt(m.get('variacion_total_omega_rad_s'),6),
             inversiones_signo_omega_umbral_0p05='',
             oscilaciones_omega_reportadas=str(m.get('oscilaciones_omega','')),
             eventos_contactos_directos='',observacion_contactos='En B1_3 no se midieron contactos Gazebo en estos JSON; estimación geométrica solamente',
             modo_real='Seguimiento de ruta A*',fuente_resumen=str(path),fuente_control=''))

    # Los JSON/CSV B2-B4 son corridas preliminares diferentes de las cinco filas A17.
    for b in ('B2','B3','B4'):
        rbase=ROOT/BASE/b
        dirs=sorted(p for p in rbase.iterdir() if p.is_dir() and (p/'resumen.json').exists())
        if len(dirs)!=5:raise RuntimeError(f'{b}: se esperaban cinco corridas preliminares')
        for rep,p in enumerate(dirs,1):
            res=p/'resumen.json';ctl=p/'control.csv';odo=p/'odometria.csv'
            for f in (res,ctl,odo):paths.add(str(f.relative_to(ROOT)))
            d=readjson(res.relative_to(ROOT))
            c=readcsv(ctl.relative_to(ROOT))
            o=readcsv(odo.relative_to(ROOT))
            if len(c)<5 or len(o)<5:raise RuntimeError(f'{b}-{rep}: traza corta')
            ts=[float(v['t_s']) for v in c]
            ws=[float(v['omega_radps']) for v in c]
            if any(t2<t1 for t1,t2 in zip(ts,ts[1:])):raise RuntimeError('Marcas temporales no monótonas')
            variation=sum(abs(v2-v1) for v1,v2 in zip(ws,ws[1:]))
            sign=[1 if w>0.05 else -1 if w<-0.05 else 0 for w in ws]
            nonzero=[s for s in sign if s]
            inversions=sum(a!=z for a,z in zip(nonzero,nonzero[1:]))
            if not d.get('rosbag_valido',False): raise RuntimeError(f'{b}-{rep}: rosbag inválido')
            if d.get('contactos_gazebo_verificados') is not False:
                raise RuntimeError('La reserva original de ausencia de contactos Gazebo cambió')
            if b=='B2':mode='A* con desvío; DIFUSO no actuó'
            elif b=='B3':mode='Híbrido asistido; DIFUSO activo; subobjetivos geométricos'
            else:mode='Centrado geométrico; FIS sombra'
            supp.append(dict(escenario=b,repeticion=rep,serie=f'{b} preliminar distinta de A17',
                  metodo_tiempo='duración entre primer y último registro control.csv t_s',
                  duracion_s=fmt(ts[-1]-ts[0],6),recorrido_m=fmt(d['distancia_recorrida_m'],6),desplazamiento_neto_m='',
                  error_final_m=fmt(d['distancia_final_meta_m'],6),
                  despeje_geometrico_m=fmt(d['distancia_minima_a_colisiones_sdf_m'],6),
                  desviacion_max_m=fmt(d['desviacion_max_a_ruta_m'],6),
                  variacion_absoluta_omega_rad_s=fmt(variation,6),
                  inversiones_signo_omega_umbral_0p05=str(inversions),
                  oscilaciones_omega_reportadas='',
                  eventos_contactos_directos='',
                  observacion_contactos='Sin sensor Gazebo de contacto en esta traza; despeje SDF es geométrico',
                  modo_real=mode,fuente_resumen=str(res.relative_to(ROOT)),
                  fuente_control=str(ctl.relative_to(ROOT))))
    supp.sort(key=lambda x:(x['escenario'],int(x['repeticion'])))
    if len(main)!=20 or len(supp)!=20:raise RuntimeError('La selección no contiene 20 y 20 ensayos')
    return main,supp,sorted(paths)


def svg_bars(title, subtitle, labels, values, xlabel, ymax=None, color='#2563eb'):
    n=len(labels);W,H=880,440; left=82;right=30;top=94;bottom=88
    mx=max(values) if values else 1
    maxv= ymax or (mx*1.15 if mx else 1)
    plotH=H-top-bottom;plotW=W-left-right
    lines=['<svg xmlns="http://www.w3.org/2000/svg" width="880" height="440" viewBox="0 0 880 440">',
           '<rect width="100%" height="100%" fill="white"/>',
           f'<text x="44" y="42" font-family="Arial" font-size="24" font-weight="bold" fill="#0f172a">{escape(title)}</text>',
           f'<text x="44" y="65" font-family="Arial" font-size="13" fill="#475569">{escape(subtitle)}</text>']
    for i in range(5):
        v=maxv*i/4; y=top+plotH*(1-i/4)
        lines.append(f'<line x1="{left}" y1="{y:.1f}" x2="{W-right}" y2="{y:.1f}" stroke="#e2e8f0"/>')
        lines.append(f'<text x="{left-12}" y="{y+5:.1f}" text-anchor="end" font-family="Arial" font-size="12" fill="#475569">{v:.2f}</text>')
    w=plotW/n; barw=min(95,w*.48)
    for i,(lab,val) in enumerate(zip(labels,values)):
        x=left+i*w+w/2; bh=(val/maxv)*plotH; y=top+plotH-bh
        lines.append(f'<rect x="{x-barw/2:.1f}" y="{y:.1f}" width="{barw:.1f}" height="{max(0,bh):.1f}" rx="4" fill="{color}"/>')
        lines.append(f'<text x="{x:.1f}" y="{y-9:.1f}" text-anchor="middle" font-family="Arial" font-size="14" fill="#0f172a">{val:.3f}</text>')
        lines.append(f'<text x="{x:.1f}" y="{H-bottom+23}" text-anchor="middle" font-family="Arial" font-size="13" fill="#334155">{escape(lab)}</text>')
    lines.append(f'<text x="{W/2:.1f}" y="{H-22}" text-anchor="middle" font-family="Arial" font-size="14" fill="#475569">{escape(xlabel)}</text>')
    lines.append('</svg>')
    return ('\n'.join(lines)+'\n').encode()


def build():
    main,supp,paths=collect()
    srcmanifest=[]
    for x in paths:
        b=(ROOT/x).read_bytes()
        srcmanifest.append(dict(ruta=x,bytes=len(b),sha256=sha_bytes(b),git_blob_sha1=git_blob_sha(b)))
    summ=[];suppsumm=[]
    for b in ('B1','B2','B3','B4'):
        m=[r for r in main if r['escenario']==b]
        sm=[r for r in supp if r['escenario']==b]
        summ.append(dict(escenario=b, ensayos=len(m),llegadas=sum(int(x['llegada']) for x in m),
           contactos_episodios=sum(int(x['contactos_episodios']) for x in m),
           error_final_m_media=fmt(avg(m,'error_final_m'),6),
           recorrido_m_media=fmt(avg(m,'recorrido_m'),6) if avg(m,'recorrido_m') is not None else '',
           despeje_sdf_m_min=fmt(min((float(x['despeje_sdf_min_m']) for x in m if x['despeje_sdf_min_m']), default=None),6),
           desviacion_max_m_mayor=fmt(max((float(x['desviacion_max_m']) for x in m if x['desviacion_max_m']), default=None),6),
           tiempos_A17='NO_PUBLICADOS',oscilaciones_A17='NO_PUBLICADAS',
           comandos_DIFUSO_total=str(sum(int(x['comandos_difuso_activos']) for x in m if x['comandos_difuso_activos']!='')) if b in ('B2','B3') else '',
           modo_real=m[0]['modo_real']))
        suppsumm.append(dict(escenario=b, ensayos=len(sm),
             duracion_media_s=fmt(avg(sm,'duracion_s'),3),
             metodo_tiempo=sm[0]['metodo_tiempo'],
             recorrido_m_media=fmt(avg(sm,'recorrido_m'),4),
             despeje_geom_min_m=fmt(min(float(x['despeje_geometrico_m']) for x in sm),4),
             desviacion_max_m=fmt(max(float(x['desviacion_max_m']) for x in sm),4),
             variacion_omega_media=fmt(avg(sm,'variacion_absoluta_omega_rad_s'),4),
             inversiones_omega_media=fmt(avg(sm,'inversiones_signo_omega_umbral_0p05'),2) if b!='B1' else '',
             contacto_directo='NO MEDIDO',
             serie='Histórica independiente de A17'))

    svgs={
        str(FIG/'A17_error_final_medio.svg'):svg_bars('A12/A17: error final medio',
           'Veinte corridas acreditadas, cinco por escenario (metros)',
           ['B1 libre','B2 fijo','B3 sorpresa','B4 0,60 m'],
           [float(s['error_final_m_media']) for s in summ], 'Error final medio a meta (m)',color='#155e75'),
        str(FIG/'historicas_duracion_control_B2_B4.svg'):svg_bars('Trazas históricas: duración de control',
           'B2–B4: intervalos t_s de control.csv; NO son corridas A17',
           ['B2 inicial','B3 híbrido','B4 centrado'],
           [float(s['duracion_media_s']) for s in suppsumm if s['escenario']!='B1'],
           'Duración media de la traza de control (s)',color='#7c3aed'),
        str(FIG/'historicas_desviacion_B2_B4.svg'):svg_bars('Trazas históricas: máxima desviación',
           'Máximo entre cinco ejecuciones preliminares; no sustituye A17',
           ['B2 inicial','B3 híbrido','B4 centrado'],
           [float(s['desviacion_max_m']) for s in suppsumm if s['escenario']!='B1'],
           'Desviación máxima frente a ruta (m)',color='#ca8a04'),
        str(FIG/'historicas_variacion_omega_B2_B4.svg'):svg_bars('Trazas históricas: variación de ω aplicada',
           'Suma de |Δω| de control.csv; proxy, no prueba de oscilación física',
           ['B2 inicial','B3 híbrido','B4 centrado'],
           [float(s['variacion_omega_media']) for s in suppsumm if s['escenario']!='B1'],
           'Variación angular media acumulada (rad/s)',color='#15803d')
    }
    report=[]
    report.append('# A2 · Actividad 13 — Consolidación reproducible B1–B4')
    report.append('')
    report.append('**Propósito:** documentar las cinco repeticiones de cada escenario de la rúbrica U2S7, con trazabilidad y sin editar código ni resultados experimentales. **Fuentes:** corte A12/A17 e historiales independientes B1_3/B2/B3/B4. **No se reejecutó Gazebo ni ROS 2.**')
    report.append('')
    report.append('## 1. Criterio normativo')
    report.append('La rúbrica pide para B1 (pasillo libre), B2 (obstáculo fijo), B3 (obstáculo aparecido a 1 m) y B4 (pasillo de 0,6 m) cinco repeticiones con: colisiones, distancia mínima al obstáculo, tiempo de llegada, desviación máxima de A* y oscilaciones de ω. La evaluación de esos indicadores depende de conservar la identidad de cada ejecución. Esta actividad **no fusiona resultados de corridas distintas**.')
    report.append('')
    report.append('## 2. Matriz A12/A17: veinte ensayos acreditados')
    report.append('')
    report.append('| Escenario | Llegadas | Contactos registrados | Error medio (m) | Recorrido medio (m) | Despeje SDF mínimo (m) | Desv. máxima (m) | Modo demostrado |')
    report.append('|---|---:|---:|---:|---:|---:|---:|---|')
    for s in summ:
        def fld(k,n=4):return fmt(s[k],n)
        report.append(f"| {s['escenario']} | {s['llegadas']}/{s['ensayos']} | {s['contactos_episodios']} | {fld('error_final_m_media')} | {fld('recorrido_m_media')} | {fld('despeje_sdf_m_min')} | {fld('desviacion_max_m_mayor')} | {s['modo_real']} |")
    report.extend(['', '**Interpretación:** el cierre A17 informa cobertura válida, 20/20 llegadas y cero contactos medidos en sus tablas. En los JSON preliminares B2–B4 constaba que todavía NO se había medido el contacto directamente; se trata de versiones y auditorías diferentes. **Los rosbags brutos no están todos en el repositorio publicado**: la auditoría presente valida las fuentes archivadas y sus hashes, no reproduce la medición original.', '',
       '**Datos ausentes de A17:** B1–B4 no publican tiempo de llegada ni oscilaciones de ω para esas mismas corridas. B1–B3 tampoco incluyen la misma columna de despeje y desviación completa. Se conserva N/D en las tablas en lugar de imputar datos.', '',
       '**Responsabilidad del control:** B1 sigue A*, B2 completa desvío A* pero **no** demuestra actuación Mamdani, B3 combina Mamdani activo y subobjetivos geométricos (no Mamdani puro; 162 comandos activos DIFUSO en total), B4 completa centrado geométrico con FIS en sombra. Estas condiciones impiden atribuir los 20 éxitos a evasión difusa exclusiva.', '',
       '## 3. Métricas adicionales de series históricas independientes', '',
       '| Serie | n | Duración media (s) | Recorrido medio (m) | Despeje mínimo geom. (m) | Desv. máxima (m) | Σ|Δω| media (rad/s) | Inversiones signo ω medias |',
       '|---|---:|---:|---:|---:|---:|---:|---:|'])
    for s in suppsumm:
        report.append(f"| {s['escenario']} temprana | {s['ensayos']} | {s['duracion_media_s']} | {s['recorrido_m_media']} | {s['despeje_geom_min_m']} | {s['desviacion_max_m']} | {s['variacion_omega_media']} | {s['inversiones_omega_media'] or 'N/D'} |")
    report.extend(['', '**B1_3 registra 0 oscilaciones de ω en las cinco repeticiones**, según su JSON. Su campo `distancia_neta_m` es desplazamiento neto, no una trayectoria integrada; por ello el recorrido de esa serie aparece como N/D y su desplazamiento se conserva en una columna aparte.', '',
       '**Método temporal:** B1_3 usa `tiempo_llegada_s_reloj_real` consignado en su `resumen.json`; B2–B4 usan `t_s(último)-t_s(primero)` de `control.csv`. Por sus orígenes distintos, NO son una comparación de rendimiento temporal normalizada ni tiempos válidos para sustituir los de A17.', '',
       '**Oscilaciones:** para B1_3 se retiene el indicador histórico `oscilaciones_omega`; para B2–B4 se calculó la **variación angular acumulada** `Σ|ωᵢ-ωᵢ₋₁|` y las **inversiones de signo** ignorando valores `|ω|≤0,05 rad/s`. Son indicadores derivados de COMANDOS, no observaciones directas de la velocidad angular física del robot; el umbral es una decisión metodológica explícita. La suma no debe interpretarse como giro acumulado en radianes.', '',
       '**Distancia mínima:** B1_3 utiliza distancia robot/obstáculo geométrica, aunque no se inserta una caja en ese pasillo libre; B2–B4 reportan distancia mínima a superficies SDF. No son medidas homogéneas de distancia a una misma clase de obstáculo.', '',
       '**Contactos en las trazas tempranas:** en los JSON B2–B4 aparece `contactos_gazebo_verificados: false`; en B1_3 tampoco consta sensor de contacto medido. Es incorrecto afirmar cero CONTACTOS FÍSICOS a partir de esas fuentes. El cero que figura en A17 corresponde a la auditoría posterior de cobertura.', '',
       '## 4. Figuras y archivos reproducibles', '',
       '- `docs/A2_ACT13_A17_20_REPETICIONES.csv`: las 20 filas A17, sin datos imputados.',
       '- `docs/A2_ACT13_A17_RESUMEN.csv`: resultados agregados exclusivos de A17.',
       '- `docs/A2_ACT13_HISTORICOS_20_REPETICIONES.csv`: 20 pruebas más tempranas, independientes.',
       '- `docs/A2_ACT13_HISTORICOS_RESUMEN.csv`: promedios de ensayos tempranos (no comparable temporalmente con A17).',
       '- `docs/A2_ACT13_FUENTES_SHA256.csv`: manifiesto de fuentes originales sin modificaciones.',
       '- `docs/evidencias_A2/ACT13/*.svg`: cuatro gráficos construidos directamente de las tablas anteriores.',
       '- `tools/a2_act13_consolidar_escenarios.py`: regeneración y verificación deterministas (Python estándar).', '',
       '## 5. Limitaciones y próximos pasos', '',
       '1. Completar una captura uniforme con tiempo simulado de llegada, despeje, error respecto a A*, señales ω reales y contacto Gazebo para cinco repeticiones de **cada** escenario bajo la misma versión de control.',
       '2. Repetir B2 y B4 demostrando actuación directa del difuso si se pretende acreditar evasión Mamdani pura; B3 ya demuestra actuación pero está asistido.',
       '3. Utilizar los resultados A13 (experimento pareado v03/v04) exclusivamente como estudio de rediseño separado; no incorporarlos como repeticiones de A12.',
       '4. Preservar los registros y variantes originales. No reescribir `avance2` ni `final`.', '',
       '## 6. Procedimiento de reproducción', '',
       'Desde la raíz del repositorio en Ubuntu:', '',
       '```bash',
       'python3 tools/a2_act13_consolidar_escenarios.py --check',
       '# Solo si necesita regenerar los derivados del ACT13:',
       'python3 tools/a2_act13_consolidar_escenarios.py --write',
       '```', '',
       '**Criterio de cierre:** la actividad 13 documenta honestamente toda la información publicada y sus ausencias. No constituye la validación integral pendiente de los cuatro escenarios, que sigue requiriendo pruebas uniformes.', ''])
    files={
     str(OUT/'A2_ACT13_A17_20_REPETICIONES.csv'):csvbytes(main,FIELDS_MAIN),
     str(OUT/'A2_ACT13_A17_RESUMEN.csv'):csvbytes(summ,list(summ[0])),
     str(OUT/'A2_ACT13_HISTORICOS_20_REPETICIONES.csv'):csvbytes(supp,FIELDS_SUPP),
     str(OUT/'A2_ACT13_HISTORICOS_RESUMEN.csv'):csvbytes(suppsumm,list(suppsumm[0])),
     str(OUT/'A2_ACT13_FUENTES_SHA256.csv'):csvbytes(srcmanifest,['ruta','bytes','sha256','git_blob_sha1']),
     str(OUT/'A2_ACT13_CONSOLIDACION_B1_B4.md'):('\n'.join(report)).encode(),
     **svgs
    }
    return files,paths,summ,suppsumm


def check_git(paths):
    if not (ROOT/'.git').exists():
        print('[AVISO] Repositorio de prueba sin .git; comprobación contra tag final diferida a Ubuntu')
        return
    proc=subprocess.run(['git','-C',str(ROOT),'ls-tree','-r','final'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    if proc.returncode:
        raise RuntimeError('No está disponible el tag final; ejecute git fetch origin --tags')
    expected={}
    for line in proc.stdout.splitlines():
        left,rel=line.split('\t',1)
        typ,sha=left.split()[1:3]
        if typ=='blob':expected[rel]=sha
    for p in paths:
        if p not in expected:raise RuntimeError(f'El tag final no incluye {p}')
        actual=git_blob_sha((ROOT/p).read_bytes())
        if expected[p]!=actual:raise RuntimeError(f'Diferencia respecto a tag final: {p}')
    print(f'[OK] Fuentes coinciden con Git tag final: {len(paths)}/{len(paths)}')


def main():
    pa=argparse.ArgumentParser(description=__doc__)
    group=pa.add_mutually_exclusive_group(required=True)
    group.add_argument('--write',action='store_true',help='Genera únicamente los nuevos archivos ACT13')
    group.add_argument('--check',action='store_true',help='Verifica sin modificar los archivos')
    args=pa.parse_args()
    files,paths,sums,hists=build()
    print('=== A2 / ACT13: CONSOLIDACIÓN B1–B4 ===')
    print('[OK] A17: 20/20 llegadas; 0 episodios contacto en tablas publicadas')
    print('[OK] Separación: cinco A17 y cinco históricos por escenario, sin fusionar corridas')
    print('[OK] B3: 162 comandos reales DIFUSO en A17; B2 sin actuación; B4 FIS en sombra')
    print('[OK] Campos sin fuente A17 permanecen vacíos, NO inferidos')
    if args.check:
        check_git(paths)
        for rel,content in files.items():
            file=ROOT/rel
            if not file.exists() or file.read_bytes()!=content:
                raise RuntimeError(f'Archivo derivado ausente o diferente: {rel}; revise las fuentes')
            print('[OK] Reproducible:',rel)
        print('[RESULTADO] AUDITORIA A2-ACT13 APROBADA: documental, sin ejecutar Gazebo')
    else:
        for rel,content in files.items():
            file=ROOT/rel
            file.parent.mkdir(parents=True,exist_ok=True)
            file.write_bytes(content)
            print('[GENERADO]',rel)
        print('[RESULTADO] DERIVADOS A2-ACT13 GENERADOS; ejecute --check para validarlos')

if __name__=='__main__':
    try:main()
    except Exception as exc:
        print('[ERROR]',exc,file=sys.stderr)
        raise SystemExit(1)
