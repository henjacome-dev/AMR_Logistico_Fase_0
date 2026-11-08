#!/usr/bin/env python3
"""Auditoría CONSOLIDADA, SOLO LECTURA, R19-R23 (Avance 2).

Ubuntu WSL2:
  python3 /mnt/c/Users/Henry/Downloads/A2_auditoria_integral_cierre_R19_R23.py

Inspecciona resultados reales existentes, sin iniciar Gazebo ni repetir ensayos.
No modifica archivos Git, no hace commit/push, no elimina nada.
Genera un informe Markdown, un resumen JSON, y un indice de evidencias CSV
bajo resultados/A2_auditoria_integral_cierre/. Rosbags solo se constatan,
no se reinterpretan ni se copian. Las validaciones visuales siguen requiriendo
inspeccion humana. Umbrales R19 y definicion R20 requieren cotejo documental.
"""
from __future__ import annotations
import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import re
import struct
import zlib
import subprocess
import sys
from collections import Counter,defaultdict
from pathlib import Path

REPO=Path.home()/'AMR_Logistico_Fase_0'
BASE='82ba1d98a4201b01e7eaaa42146c163808dcffec'
FIS_BLOB='6c7dfdd083728d49c64dd1c90a59b19d63222063'
ROOT=REPO/'resultados'
PATHS={
 'R19':ROOT/'A2_R19_integral_gazebo',
 'R20':ROOT/'A2_R20_integral_4x5',
 'R21':ROOT/'A2_R21_comparacion_Mamdani_AB',
 'R22_R23':ROOT/'A2_R22_R23_CNN_integral_v6_contexto',
}
CLASSES={'RECEPCION','ALMACENAMIENTO','EMPAQUE','DESPACHO'}

class Audit:
    def __init__(self):
        self.checks=[];self.findings=[];self.inventory=[];self.directories={}
    def check(self,ok,msg,critical=True):
        ok=bool(ok);self.checks.append({'ok':ok,'mensaje':msg,'critico':critical})
        print(('[OK] ' if ok else ('[ERROR] ' if critical else '[ATENCION] '))+msg,flush=True)
        return ok
    def observation(self,msg):
        self.findings.append(msg);print('[OBSERVACION] '+msg,flush=True)
    def file(self,p,sha=True):
        p=Path(p)
        if not p.is_file():return False
        item={'ruta':str(p.relative_to(REPO)) if p.is_relative_to(REPO) else str(p),
              'bytes':p.stat().st_size,'sha256':digest(p) if sha else '',
              'metodo':'SHA256' if sha else 'presencia_y_tamano'}
        self.inventory.append(item);return True

def digest(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def git_blob(p):
    b=Path(p).read_bytes()
    return hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()

def data(p):return json.loads(Path(p).read_text(encoding='utf8'))
def table(p):
    with Path(p).open(encoding='utf8',newline='') as f:return list(csv.DictReader(f))

def find_run(root,required):
    if not root.is_dir():return None
    options=[p for p in root.iterdir() if p.is_dir() and all((p/f).is_file() for f in required)]
    return max(options,key=lambda p:p.name) if options else None

def preflight(a):
    print('=== AUDITORIA UNICA R19-R23 | NO SE EJECUTAN ENSAYOS ===',flush=True)
    a.check((REPO/'.git').is_dir(),'Repositorio Git disponible')
    if not (REPO/'.git').is_dir():return
    r=subprocess.run(['git','rev-parse','HEAD'],cwd=REPO,capture_output=True,text=True)
    a.check(r.returncode==0 and r.stdout.strip()==BASE,'HEAD es la base auditada E2.3.6A')
    r=subprocess.run(['git','branch','--show-current'],cwd=REPO,capture_output=True,text=True)
    a.check(r.returncode==0 and r.stdout.strip()=='desarrollo-avance2','Rama desarrollo-avance2')
    f=REPO/'matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis'
    a.check(f.is_file() and git_blob(f)==FIS_BLOB,'FIS oficial v01 mantiene integridad de Git')
    a.file(f)
    r=subprocess.run(['git','status','--porcelain','--untracked-files=no'],cwd=REPO,capture_output=True,text=True)
    a.check(r.returncode==0 and not r.stdout.strip(),'Sin cambios PREEXISTENTES en archivos rastreados (no se modifica ninguno)',critical=False)

def audit_r19(a,p):
    print('\n=== R19: SUPERVISOR ===',flush=True)
    if not a.check(p is not None,'Evidencia completa R19 localizada'):return
    s=data(p/'resumen.json');a.file(p/'resumen.json');a.file(p/'observaciones.csv')
    a.check(s.get('estado')=='APROBADA_PREVALIDACION_EXPERIMENTAL','Estado R19: prevalidacion experimental aprobada')
    a.check(len(s.get('etapas',[]))==7,'Siete transiciones/etapas R19 observadas')
    a.check(not s.get('fallos',[]),'Sin fallos de seguridad declarados por R19')
    a.check(s.get('r19_oficial_acreditado') is False,'R19 conserva advertencia: no acreditado formalmente')
    a.observation('R19: umbrales 0.60/0.80 m provisionales; contrastar el enunciado y formalizar el lanzamiento operativo.')

def audit_r20(a,p):
    print('\n=== R20: CUATRO RUTAS POR CINCO ENSAYOS ===',flush=True)
    if not a.check(p is not None,'Evidencia completa R20 localizada'):return
    s=data(p/'resumen.json');rows=table(p/'resumen.csv')
    for f in ('resumen.json','resumen.csv','agregado_por_ruta.csv'):a.file(p/f)
    a.check(s.get('estado')=='APROBADA_EXPERIMENTAL' and s.get('exitos')==20 and s.get('n_ensayos')==20,
            'R20: resultado experimental 20/20')
    a.check(len(rows)==20 and all(r.get('estado')=='OK' for r in rows),'20 filas CSV con estado OK')
    ids=Counter((r['ruta'],r['repeticion']) for r in rows)
    a.check(len(ids)==20 and all(n==1 for n in ids.values()) and
            {r['ruta'] for r in rows}=={'1','2','3','4'} and
            {r['repeticion'] for r in rows}=={'1','2','3','4','5'},
            'Matriz de rutas y repeticiones completa y sin duplicados')
    a.check(all(float(r.get('error_final_m','999'))<=.1000001 for r in rows),
            '20 errores de meta <=0.10m')
    a.check(all(int(r.get(k,'0'))>0 for r in rows for k in ('rosbag_cmd','rosbag_odom','rosbag_scan')),
            'Conteos positivos por ensayo de /cmd_vel, /odom y /scan')
    bags=[]
    for r in rows:
        local=p/f"ruta_{int(r['ruta']):02d}_rep_{int(r['repeticion']):02d}"/'rosbag2'
        meta=local/'metadata.yaml'
        dbs=list(local.glob('*.db3')) if local.exists() else []
        ok=meta.is_file() and bool(dbs) and all(d.stat().st_size>0 for d in dbs)
        bags.append(ok)
        if ok:
            a.file(meta)
            for db in dbs:a.file(db,sha=False)
    a.check(sum(bags)==20,f'20 carpetas rosbag2 con metadatos y bases SQLite presentes ({sum(bags)}/20)')
    a.observation('R20: la auditoria constata archivos, conteos del CSV y tamaños; NO reproduce los rosbags ni verifica internamente cada evento.')
    a.observation('R20: 4 rutas A* empleadas como escenarios; verificar equivalencia con la rubrica oficial. DIFUSO=0 en 20 ensayos.')
    a.check(all(int(r.get('cmd_difuso','0'))==0 for r in rows),'R20 se ejecutó sin comandos DIFUSO; no prueba mejoras de evasión')

def audit_r21(a,p):
    print('\n=== R21: REDISENO A/B ===',flush=True)
    if not a.check(p is not None,'Evidencia completa R21 localizada'):return
    s=data(p/'resumen_R21.json');rows=table(p/'comparacion_20_ensayos.csv')
    for f in ('resumen_R21.json','comparacion_20_ensayos.csv','resumen_4_escenarios.csv',
              'observaciones_AB.csv','panel_R21.html','informe_R21.md',
              'AMR_Mamdani_A2_v02_experimental.fis'):a.check(a.file(p/f),f'R21: {f} presente')
    a.check(s.get('parejas')==20 and s.get('inferencias_total')==1440 and len(rows)==20,
            '20 pares, 1.440 inferencias y tabla de 20 comparaciones')
    a.check(s.get('blob_original')==FIS_BLOB,'Referencia del FIS v01 coincide')
    v02=p/'AMR_Mamdani_A2_v02_experimental.fis'
    a.check(v02.is_file() and digest(v02)==s.get('sha256_v02'),'SHA256 del FIS v02 experimental conforme')
    pair=Counter((r['escenario'],r['repeticion']) for r in rows)
    a.check(len(pair)==20 and all(v==1 for v in pair.values()),'20 pares únicos por escenario y repetición')
    global_m=s.get('metricas',{})
    if 'A' in global_m and 'B' in global_m:
        A=global_m['A'];B=global_m['B']
        a.check(all(A.get(k)==0 and B.get(k)==0 for k in ('violaciones_parada','violaciones_limites')),
                'A/B sin violaciones numéricas de parada ni límites')
        for label,key in (('giro_medio','omega_abs_medio_rad_s'),('variacion','variacion_omega_rad_s')):
            x,y=A.get(key,0),B.get(key,0)
            if x:a.observation(f'R21 {label}: A={x:.5f}, B={y:.5f}, cambio={100*(y/x-1):+.2f}%.')
    by={r['escenario']:r for r in s.get('comparacion_por_escenario',[])}
    lateral=[by.get('OBSTACULO_IZQUIERDO'),by.get('OBSTACULO_DERECHO')]
    if all(lateral):
        a.check(all(r['cambio_variacion_pct']>0 for r in lateral),
                'R21: aumento de variación angular lateral reconocido (resultado mixto)')
    a.observation('R21: cambios positivos y negativos documentados; no existe prueba de mejora de navegación real en Gazebo.')

def png_size(path):
    """Valida PNG completo: firma, chunks/CRC, zlib y longitud de raster."""
    with Path(path).open('rb') as f:
        sig=f.read(8)
        if sig!=b'\x89PNG\r\n\x1a\n':raise ValueError('Firma PNG invalida')
        width=height=None;pixel_data=[];iend=False
        while True:
            h=f.read(8)
            if len(h)!=8:raise ValueError('Chunk PNG truncado')
            length,tag=struct.unpack('>I4s',h)
            if length>100*1024*1024:raise ValueError('Chunk PNG demasiado grande')
            payload=f.read(length);crc=f.read(4)
            if len(payload)!=length or len(crc)!=4:raise ValueError('Datos PNG truncados')
            actual=zlib.crc32(tag+payload)&0xffffffff
            if struct.unpack('>I',crc)[0]!=actual:raise ValueError('CRC PNG no valido')
            if tag==b'IHDR':
                if length!=13 or width is not None:raise ValueError('IHDR PNG incorrecto')
                width,height,depth,ctype,compression,filtermethod,interlace=struct.unpack('>IIBBBBB',payload)
                if (depth,ctype,compression,filtermethod,interlace)!=(8,2,0,0,0):
                    raise ValueError('PNG no es RGB8 sin entrelazado')
            if tag==b'IDAT':pixel_data.append(payload)
            if tag==b'IEND':iend=True;break
        if f.read(1):raise ValueError('Bytes sobrantes tras IEND')
    if not iend or not pixel_data or (width,height)!=(640,360):raise ValueError('Dimensiones o datos PNG invalidos')
    try:raw=zlib.decompress(b''.join(pixel_data))
    except zlib.error as exc:raise ValueError('Raster comprimido PNG invalido') from exc
    row_bytes=width*3+1
    if len(raw)!=height*row_bytes:raise ValueError('Longitud raster PNG invalida')
    if any(raw[y*row_bytes]>4 for y in range(height)):
        raise ValueError('Filtro PNG no valido')
    return width,height

def audit_cnn(a,p):
    print('\n=== R22/R23: DATASET Y CNN ===',flush=True)
    if not a.check(p is not None,'Evidencia completa R22/R23 v6 localizada'):return
    summary=data(p/'resumen_dataset.json')
    for f in ('resumen_dataset.json','manifiesto.csv','configuracion_optica.json'):
        a.check(a.file(p/f),f'Dataset: {f} presente')
    rows=table(p/'manifiesto.csv')
    a.check(len(rows)==600 and summary.get('total')==600,'600 registros dataset y resumen de adquisición')
    classes=Counter(r.get('clase') for r in rows)
    a.check(set(classes)==CLASSES and all(v==150 for v in classes.values()),'4 clases x 150 imágenes')
    parts=Counter(r.get('particion') for r in rows)
    a.check(parts=={'train':420,'val':90,'test':90},'Partición 420/90/90')
    groups=defaultdict(set)
    for r in rows:groups[r.get('grupo')].add(r.get('particion'))
    a.check(len(groups)==100 and all(len(x)==1 for x in groups.values()),'100 grupos; ninguna pose compartida entre particiones')
    a.check(len({r.get('id') for r in rows})==600,'600 IDs únicos')
    checksums={};bad=[];mismatch=[];n_png=0
    for r in rows:
        rel=Path(r.get('archivo',''))
        if rel.is_absolute() or '..' in rel.parts:bad.append(r.get('id'));continue
        f=p/rel
        try:
            if not f.is_file() or png_size(f)!=(640,360):bad.append(r.get('id'));continue
            sha=digest(f)
            if sha!=r.get('sha256'):mismatch.append(r.get('id'))
            checksums.setdefault(sha,[]).append(r.get('id'))
            a.inventory.append({'ruta':str(f.relative_to(REPO)),'bytes':f.stat().st_size,
                                'sha256':sha,'metodo':'SHA256 + PNG IHDR'})
            n_png+=1
        except (OSError,ValueError):bad.append(r.get('id'))
    a.check(n_png==600 and not bad,f'600 PNG íntegros 640x360 (raster y CRC) ({n_png}/600)')
    a.check(not mismatch and not [v for v in checksums.values() if len(v)>1],
            'SHA256 por imagen correcto y sin duplicados exactos')
    a.check(summary.get('sha256_manifiesto')==digest(p/'manifiesto.csv'),'Hash del manifiesto coincide con resumen')
    optical=p/'configuracion_optica.json'
    a.check(summary.get('configuracion_optica_sha256')==digest(optical),'Hash de configuracion optica verificado')
    train=p/'entrenamiento'
    metrics=train/'reporte_entrenamiento.json';model=train/'modelo_cnn_a2.keras'
    for f in (metrics,model,train/'reporte_visual.html',train/'matriz_confusion.csv',
              train/'historial.csv',train/'mosaico_control_calidad.png'):
        a.check(a.file(f),f'R23: {f.name} presente')
    if metrics.is_file():
        m=data(metrics)
        a.check(m.get('estado')=='R23_PRIMER_ENTRENAMIENTO_EJECUTADO','CNN preliminar entrenada')
        a.check(m.get('muestras')=={'train':420,'val':90,'test':90},'Entrenamiento usó 420/90/90')
        a.check(m.get('epocas',0)>=1,'Al menos una época completada')
        a.check(model.is_file() and digest(model)==m.get('sha256_modelo')==summary.get('modelo_sha256'),
                'Modelo .keras y hashes de reportes coinciden')
        mat=m.get('matriz_confusion',[])
        a.check(len(mat)==4 and all(len(row)==4 for row in mat) and
                sum(sum(row) for row in mat)==90,'Matriz de confusión 4x4, 90 muestras test')
        if len(mat)==4 and all(len(row)==4 for row in mat):
            diag=sum(mat[i][i] for i in range(4))
            a.observation(f'R23: aciertos test={diag}/90, accuracy declarada={m.get("accuracy_test",0):.2%}; solo dominio sintético.')
            a.check(abs(diag/90-m.get('accuracy_test',float('nan')))<1e-5,'Exactitud test consistente con matriz de confusión')
    a.observation('R22/R23: auditar visualmente mosaico_control_calidad.png; partición por pose no equivale a validación en otro mundo/cámara.')


def create_report(a):
    output=ROOT/'A2_auditoria_integral_cierre'/dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    output.mkdir(parents=True,exist_ok=False)
    failures=[x for x in a.checks if not x['ok'] and x['critico']]
    warn=[x for x in a.checks if not x['ok'] and not x['critico']]
    result={'estado':'AUDITORIA_TECNICA_FAVORABLE_CON_RESERVAS' if not failures else 'AUDITORIA_CON_FALLOS',
            'fecha':dt.datetime.now().isoformat(timespec='seconds'),'repo_base':BASE,
            'resumen':{'verificaciones':len(a.checks),'fallos_criticos':len(failures),'alertas':len(warn),
                       'evidencias_indexadas':len(a.inventory)},
            'sesiones':a.directories,'verificaciones':a.checks,'observaciones':a.findings,
            'alcance':'Inspeccion de archivos locales; no se repite Gazebo, no se publica Git, no revisa video manualmente.'}
    (output/'dictamen_integral.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    with (output/'indice_evidencias.csv').open('w',newline='',encoding='utf8') as f:
        w=csv.DictWriter(f,fieldnames=['ruta','bytes','sha256','metodo']);w.writeheader();w.writerows(a.inventory)
    lines=['# Auditoría consolidada — Avance 2 / R19–R23','',
           '**Dictamen técnico:** '+result['estado'].replace('_',' '),
           '',f"Verificaciones: **{len(a.checks)}**, fallos críticos: **{len(failures)}**, alertas: **{len(warn)}**.",
           '', '**Importante:** esta auditoría lee las evidencias locales, no equivale a entrega académica oficial.',
           '', '## Carpetas auditadas','']
    lines +=[f'- **{k}:** `{v}`' for k,v in a.directories.items()]
    lines+=['','## Verificaciones','']
    lines +=[('- [x] ' if x['ok'] else '- [ ] ')+x['mensaje'] for x in a.checks]
    lines+=['','## Reservas y pendientes','']
    lines +=['- '+s for s in a.findings]
    lines+=['','## Acciones de cierre','',
            '1. Revisar visualmente el mosaico de test CNN y las capturas de Gazebo.',
            '2. Confirmar umbrales R19 y definición de escenarios R20 con rúbrica/enunciado.',
            '3. Redactar R21 como comparación numérica con resultado mixto; no afirmar mejora de navegación.',
            '4. Preparar una única publicación GitHub con informe y evidencias ligeras, conservando localmente dataset completo y rosbags.',
            '5. Confirmar requisitos de entrega Blackboard y registrar acuse de entrega.',
            '', 'Los rosbags no fueron copiados ni reanalizados. El índice registra tamaño de sus bases de datos sin calcular SHA256 completo.',
            '']
    (output/'informe_cierre_R19_R23.md').write_text('\n'.join(lines),encoding='utf8')
    print('\n=== DICTAMEN: '+result['estado']+' ===')
    print('[EVIDENCIA] '+str(output/'informe_cierre_R19_R23.md'))
    print('[EVIDENCIA] '+str(output/'dictamen_integral.json'))
    print('[EVIDENCIA] '+str(output/'indice_evidencias.csv'))
    print('[NOTA] Sin cambios ni commits Git. La auditoria no publica resultados.')
    return 0 if not failures else 2


def run():
    a=Audit();preflight(a)
    lookups={'R19':['resumen.json','observaciones.csv'],
             'R20':['resumen.json','resumen.csv','agregado_por_ruta.csv'],
             'R21':['resumen_R21.json','comparacion_20_ensayos.csv'],
             'R22_R23':['resumen_dataset.json','manifiesto.csv','entrenamiento/reporte_entrenamiento.json']}
    directories={k:find_run(PATHS[k],req) for k,req in lookups.items()}
    a.directories={k:(str(v) if v else 'NO LOCALIZADO') for k,v in directories.items()}
    audit_r19(a,directories['R19'])
    audit_r20(a,directories['R20'])
    audit_r21(a,directories['R21'])
    audit_cnn(a,directories['R22_R23'])
    return create_report(a)

if __name__=='__main__':
    try:sys.exit(run())
    except Exception as e:
        print('[ERROR AUDITORIA] '+str(e),file=sys.stderr,flush=True)
        sys.exit(2)
