#!/usr/bin/env python3
"""Verificador reproducible de evidencias A2 R19-R23 publicadas en Git.

Ejecuta solo biblioteca estándar Python y NO exige ROS/Gazebo ni datos externos.
Los 20 rosbags y los 600 PNG originales permanecen fuera de Git; únicamente
verifica las referencias, manifiestos y resultados seleccionados disponibles.
"""
import csv
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] if Path(__file__).parent.name == 'a2_bancos' else Path.cwd()
EVID = ROOT / 'docs/evidencias_A2'
TOOLS = ROOT / 'tools/a2_bancos'
FIS = ROOT / 'matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis'
FIS_BLOB = '6c7dfdd083728d49c64dd1c90a59b19d63222063'
EXPECTED = {
 'R19': {'resumen.json','observaciones.csv'},
 'R20': {'resumen.json','resumen.csv','agregado_por_ruta.csv'},
 'R21': {'resumen_R21.json','resumen_4_escenarios.csv','comparacion_20_ensayos.csv','observaciones_AB.csv','informe_R21.md','panel_R21.html','AMR_Mamdani_A2_v02_experimental.fis'},
 'R22_R23': {'resumen_dataset.json','manifiesto.csv','configuracion_optica.json','entrenamiento/reporte_entrenamiento.json','entrenamiento/matriz_confusion.csv','entrenamiento/historial.csv','entrenamiento/reporte_visual.html','entrenamiento/mosaico_control_calidad.png'},
 'AUDITORIA': {'informe_cierre_R19_R23.md','dictamen_integral.json','indice_evidencias.csv'},
}
SCRIPTS = {'A2_R19_banco_integral_gazebo.py','A2_R20_bateria_integral_4x5_gazebo_v2_teleport.py','A2_R21_comparacion_mamdani_AB.py','A2_R22_R23_bateria_integral_CNN_v6_camara_contextual.py','A2_auditoria_integral_cierre_R19_R23.py'}


def check(p, note):
 if not p:raise AssertionError(note)
 print('[OK] '+note, flush=True)


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
 with path.open(encoding='utf-8',newline='') as f:return list(csv.DictReader(f))


def load(path):return json.loads(path.read_text(encoding='utf-8'))


def main():
 check(FIS.is_file(), 'FIS original presente')
 data=FIS.read_bytes()
 blob=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
 check(blob==FIS_BLOB,'FIS v01 exactamente inalterado')
 manifest=rows(EVID/'manifiesto_publicacion.csv')
 expected={'docs/evidencias_A2/'+k+'/'+v for k,files in EXPECTED.items() for v in files}
 expected|={'tools/a2_bancos/'+s for s in SCRIPTS}
 check(len(manifest)==28 and {r['archivo'] for r in manifest}==expected,
       'Manifiesto exacto: 28 evidencias y scripts de R19-R23')
 for r in manifest:
  rel=Path(r['archivo'])
  check(not rel.is_absolute() and '..' not in rel.parts,'Ruta de manifiesto segura: '+str(rel))
  src=ROOT/rel
  check(src.is_file() and src.stat().st_size==int(r['tamano_bytes']) and digest(src)==r['sha256'],
        'Integridad: '+r['archivo'])
 actual={str(p.relative_to(ROOT)) for t in (EVID,TOOLS) for p in t.rglob('*') if p.is_file()}
 check(actual==expected|{'docs/evidencias_A2/manifiesto_publicacion.csv','tools/a2_bancos/A2_validar_publicacion_R19_R23.py'},
       'Sin archivos inesperados dentro del paquete publicado')
 r19=load(EVID/'R19/resumen.json')
 check(r19.get('estado')=='APROBADA_PREVALIDACION_EXPERIMENTAL' and len(r19.get('etapas',[]))==7 and not r19.get('fallos',[]) and r19.get('r19_oficial_acreditado') is False,
       'R19: 7 etapas, sin fallos declarados y reserva de acreditación preservada')
 r20=load(EVID/'R20/resumen.json');a=rows(EVID/'R20/resumen.csv')
 check(r20.get('exitos')==20 and r20.get('n_ensayos')==20 and r20.get('estado')=='APROBADA_EXPERIMENTAL',
       'R20: 20/20 declarados')
 pairs=Counter((r['ruta'],r['repeticion']) for r in a)
 check(len(a)==20 and all(r['estado']=='OK' and float(r['error_final_m'])<=0.1000001 for r in a) and
       len(pairs)==20 and set(pairs)=={(str(i),str(j)) for i in range(1,5) for j in range(1,6)},
       'R20: matriz 4x5, 20 metas correctas, sin pares repetidos')
 check(all(int(r[k])>0 for r in a for k in ('rosbag_cmd','rosbag_odom','rosbag_scan')),
       'R20: datos positivos de tres tópicos registrados por ensayo')
 check(all(int(r.get('cmd_difuso','0'))==0 for r in a),'R20: ausencia de modo difuso reconocida')
 r21=load(EVID/'R21/resumen_R21.json');b=rows(EVID/'R21/comparacion_20_ensayos.csv')
 check(r21.get('parejas')==20 and r21.get('inferencias_total')==1440 and len(b)==20 and
       r21.get('blob_original')==FIS_BLOB,'R21: 20 pares, 1440 inferencias, FIS A intacto')
 check(digest(EVID/'R21/AMR_Mamdani_A2_v02_experimental.fis')==r21.get('sha256_v02'),
       'R21: FIS B coincide con hash del rediseño')
 classes={'RECEPCION','ALMACENAMIENTO','EMPAQUE','DESPACHO'}
 data=rows(EVID/'R22_R23/manifiesto.csv');s=load(EVID/'R22_R23/resumen_dataset.json')
 check(len(data)==600 and s.get('total')==600 and len({x['id'] for x in data})==600,
       'R22: 600 metadatos e IDs únicos')
 check(Counter(r['clase'] for r in data)=={k:150 for k in classes},'R22: cuatro estaciones, 150 cada una')
 check(Counter(r['particion'] for r in data)=={'train':420,'val':90,'test':90},
       'R22: partición 420/90/90')
 groups=defaultdict(set)
 for r in data:groups[r['grupo']].add(r['particion'])
 check(len(groups)==100 and all(len(v)==1 for v in groups.values()),
       'R22: 100 poses, sin particiones cruzadas')
 check(s.get('sha256_manifiesto')==digest(EVID/'R22_R23/manifiesto.csv') and
       s.get('configuracion_optica_sha256')==digest(EVID/'R22_R23/configuracion_optica.json'),
       'R22: hashes de manifiesto y cámara coherentes')
 c=load(EVID/'R22_R23/entrenamiento/reporte_entrenamiento.json')
 matrix=c.get('matriz_confusion',[])
 check(c.get('estado')=='R23_PRIMER_ENTRENAMIENTO_EJECUTADO' and c.get('epocas',0)>=1 and
       c.get('muestras')=={'train':420,'val':90,'test':90},
       'R23: primer entrenamiento ejecutado')
 check(len(matrix)==4 and all(len(row)==4 for row in matrix) and sum(map(sum,matrix))==90 and
       abs(sum(matrix[i][i] for i in range(4))/90-c.get('accuracy_test',-1))<1e-5,
       'R23: test con matriz 4x4 y 90 muestras')
 audit=load(EVID/'AUDITORIA/dictamen_integral.json')
 check(audit.get('estado')=='AUDITORIA_TECNICA_FAVORABLE_CON_RESERVAS',
       'Dictamen favorable con reservas conservado')
 docs=(ROOT/'docs/A2_cierre_experimental_20261108.md').read_text(encoding='utf-8')
 check(all(x in docs for x in ['R19','R20','R21','R22','R23','reservas','rosbags']),
       'Acta de cierre conserva los alcances y reservas')
 print('=== CIERRE A2 R19-R23: EVIDENCIAS REMOTAS COHERENTES; DATOS EXTERNOS NO AUDITADOS POR CI ===',flush=True)


if __name__=='__main__':main()
