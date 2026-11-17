#!/usr/bin/env python3
"""A1 ACT12: verificacion estatica retrospectiva, sin ROS ni cambios en fuentes.
Ejemplo:
 python3 tools/a1_act12_verificar_evidencias.py --zip-original /mnt/c/Users/Henry/Downloads/AMR_Logistico_Fase_1-main.zip --check
"""
import argparse
import csv
import hashlib
import io
import json
import re
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_ZIP_SHA = "398835963816b9cf429ab68283b27340d6481fffa5006b115b6cad4cfc21502b"
EXPECTED_MANIFEST_SHA = "c0a8c5fcb2bfb97fbbf90800d3b45e3a30bd43eb742400b158effca0752b7d09"
CLASSES = {"RECEPCION", "ALMACENAMIENTO", "EMPAQUE", "DESPACHO"}
errors = []

def sha(data): return hashlib.sha256(data).hexdigest()
def check(cond, msg):
    print(("[OK] " if cond else "[ERROR] ") + msg)
    if not cond: errors.append(msg)
def read_bytes(path): return (ROOT / path).read_bytes()
def read(path): return (ROOT / path).read_text(encoding="utf-8-sig")

def check_source_manifest():
    file = ROOT / "docs/A1_ACT12_MANIFIESTO_SHA256.csv"
    rows = list(csv.DictReader(file.open(encoding="utf-8", newline="")))
    paths = [r['ruta'] for r in rows]
    check(len(rows) >= 35 and len(paths) == len(set(paths)), f"Manifiesto de fuentes: {len(rows)} rutas únicas")
    wrong=[]; missing=[]
    for r in rows:
        path=ROOT / r['ruta']
        if not path.is_file(): missing.append(r['ruta']); continue
        b=path.read_bytes()
        if len(b) != int(r['bytes']) or sha(b) != r['sha256']:wrong.append(r['ruta'])
    check(not missing and not wrong, f"SHA-256 y bytes de fuentes: {len(rows)-len(missing)-len(wrong)}/{len(rows)} coincidentes")
    if missing:print('  AUSENTES:',missing[:10])
    if wrong:print('  DIFERENTES:',wrong[:10])

def check_original(zipfile):
    path=Path(zipfile)
    check(path.is_file(), "ZIP original A1 accesible")
    if not path.is_file():return
    check(sha(path.read_bytes()) == EXPECTED_ZIP_SHA, "SHA-256 de ZIP A1 entregado")
    with ZipFile(path) as z:
        check(z.testzip() is None, 'ZIP original CRC íntegro')
        files=[x.filename for x in z.infolist() if not x.is_dir()]
        check(len(files)==76, 'ZIP original A1: 76 archivos')
        root='AMR_Logistico_Fase_0-main/'
        check(all(x.startswith(root) for x in files), 'ZIP original con raíz prevista')
        names={x[len(root):] for x in files if x.startswith(root)}
        check('modelos/almacen_f1.world' in names and 'docs/F0_arquitectura.md' in names, 'World y arquitectura presentes en A1')
        check('tools/emulador_sensores_f1.py' not in names and 'datos/cnn/plan_adquisicion.csv' not in names, 'Emulador y plan CSV ausentes del ZIP A1 histórico')
        if 'modelos/almacen_f1.world' in names:
            old=z.read(root+'modelos/almacen_f1.world').decode('utf-8')
            old_names={x.attrib['name'] for x in ET.fromstring(old).iter('model')}
            check(not any(x.startswith('panel_') for x in old_names), 'Mundo original sin paneles verticales posteriores')

def check_world():
    w=ET.fromstring(read('modelos/almacen_f1.world'))
    names={x.attrib['name'] for x in w.iter('model')}
    stations={'estacion_'+x.lower() for x in CLASSES}
    panels={'panel_'+x.lower() for x in CLASSES}
    check(stations.issubset(names) and panels.issubset(names), 'Mundo posterior: cuatro estaciones y cuatro paneles verticales')
    yaml=read('config/escenario_f1.yaml')
    check('size_m: [10.0, 8.0]' in yaml and 'resolution_m: 0.05' in yaml and 'inflation_radius_m: 0.21' in yaml, 'Dimensiones/resolución/inflado declarados')
    materials=read('modelos/senales_almacen/materials/scripts/estaciones.material')
    for name in CLASSES:
        path=f"modelos/senales_almacen/materials/textures/{name.lower()}.png"
        check((ROOT/path).is_file() and f"material AMR/{name}" in materials and f"AMR/{name}" in read('modelos/almacen_f1.world'), f"Marcador texturizado {name}")

def check_sensors():
    code=read('tools/emulador_sensores_f1.py')
    interfaces=read('config/interfaces.yaml')
    topics=['/rango_frontal','/rango_izq','/rango_der']
    check(all(x in code and x in interfaces for x in topics) and '"/scan"' in code, 'Emulador: LaserScan → tres Range')
    check('math.radians(15.0)' in code and 'ALCANCE_MAX_M = 2.0' in code and 'SIGMA_M = 0.01' in code, 'Sectores/alcance/ruido implementados')
    data=list(csv.DictReader(io.StringIO(read('resultados/R0708_validacion/resumen.csv'))))
    check(len(data)==3 and set(d['sensor'] for d in data)==set(topics) and all(int(d['muestras'])==600 and 0.008 <= float(d['sigma_muestral_m']) <= 0.012 for d in data), 'R07-R08: resultados estadísticos de 3 × 600 muestras sintéticas')
    wall=read('resultados/F1_A_cierre_20261106/actividad_B_pared_oeste.txt')
    check('0.852 m' in wall and '0.842 m' in wall, 'Ensayo posterior: acercamiento a pared frente a Range')

def check_architecture():
    arch=read('docs/F0_arquitectura.md')
    nodes=['emulador_sensores','seguidor_ruta','controlador_difuso','reconocedor_estacion','supervisor']
    check(all(n in arch for n in nodes), 'Arquitectura: cinco nodos especificados')
    topics=['/scan','/odom','/camera/image_raw','/cmd_vel','/rango_frontal','/rango_izq','/rango_der']
    check(all(t in read('config/interfaces.yaml') for t in topics), 'Contratos ROS 2: interfaces esenciales documentadas')
    check('Range aun no alimentan' in read('docs/F1_B5_integracion_navegacion.md'), 'Limitación B5 preservada: Range no gobernaba movimiento en ese corte')
    check('map -> odom' in read('docs/F1_R35_marcos_tf.md'), 'Informe TF posterior disponible')

def check_dataset(full_hash=True):
    planned=list(csv.DictReader(io.StringIO(read('datos/cnn/plan_adquisicion.csv'))))
    check(len(planned)==600 and all(row['estado']=='PLANIFICADA' for row in planned), 'Plan: 600 filas PLANIFICADA, no imágenes')
    check(Counter(row['estacion'] for row in planned)==Counter({c:150 for c in CLASSES}), 'Plan: cuatro clases × 150')
    check({row['distancia_m'] for row in planned}=={'0.30','0.60','0.90','1.20','1.50'} and {int(row['angulo_deg']) for row in planned}=={-30,-15,0,15,30},'Plan: 5 distancias y 5 ángulos')
    actual_bytes=read_bytes('datos/u3s7/manifiesto.csv')
    check(sha(actual_bytes)==EXPECTED_MANIFEST_SHA, 'Manifiesto capturado SHA-256 original')
    actual=list(csv.DictReader(io.StringIO(actual_bytes.decode('utf-8-sig'))))
    check(len(actual)==600 and Counter(row['clase'] for row in actual)==Counter({c:150 for c in CLASSES}), 'Captura: 600 imágenes, 150 por clase')
    check(Counter(row['particion'] for row in actual)==Counter({'train':420,'val':90,'test':90}), 'Captura: 420/90/90')
    split_groups=defaultdict(set)
    for row in actual:split_groups[row['grupo']].add(row['particion'])
    check(len(split_groups)==100 and all(len(s)==1 for s in split_groups.values()), 'Captura: 100 poses sin filtración entre particiones')
    bad=[];missing=[]
    for row in actual:
        rel=Path('datos/u3s7')/row['archivo']
        if rel.is_absolute() or '..' in rel.parts or not str(rel).startswith('datos/u3s7/imagenes/'):
            bad.append(str(rel));continue
        path=ROOT/rel
        if not path.is_file(): missing.append(str(rel));continue
        if full_hash and sha(path.read_bytes())!=row['sha256']:bad.append(str(rel))
    check(not missing and not bad, f"Imágenes: {len(actual)-len(missing)-len(bad)}/600 {'SHA-256' if full_hash else 'rutas'} verificados")
    if bad: print('  HASH INCORRECTO:', bad[:6])
    if missing: print('  NO ENCONTRADO:',missing[:6])
    summary=json.loads(read('datos/u3s7/resumen_dataset.json'))
    check(summary['total']==600 and summary['particiones']=={'train':420,'val':90,'test':90} and summary['sha256_manifiesto']==EXPECTED_MANIFEST_SHA, 'Resumen de captura coherente con manifiesto')
    c1=Counter((x['estacion'],x['particion']) for x in planned)
    c2=Counter((x['clase'],x['particion']) for x in actual)
    differences={k:(c1[k],c2[k]) for k in set(c1)|set(c2) if c1[k]!=c2[k]}
    check(bool(differences), 'Distribución por clase del dataset difiere del plan: registrada como diferencia, no error')
    print('  Diferencias clase/partición:', sorted(differences.items()))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zip-original', default=None, help='ZIP histórico entregado en Blackboard')
    parser.add_argument('--check',action='store_true',help='Verificar hashes de las 600 imágenes')
    args=parser.parse_args()
    print('=== A1 / ACT12: UNIFICACION ESTATICA DE EVIDENCIAS ===')
    print('Raíz:',ROOT)
    try:
        check_source_manifest()
        if args.zip_original:check_original(args.zip_original)
        else: print('[AVISO] Comparación ZIP 4/11 omitida; usar --zip-original para comprobarla')
        check_world();check_sensors();check_architecture();check_dataset(full_hash=args.check)
    except Exception as e:
        errors.append(str(e));print('[ERROR] Excepción:',type(e).__name__,str(e))
    if errors:
        print('[RESULTADO] AUDITORÍA A1-ACT12 FALLIDA:', len(errors),'errores');sys.exit(1)
    print('[RESULTADO] AUDITORIA A1-ACT12 APROBADA (estática, no reejecuta ROS/Gazebo)')
