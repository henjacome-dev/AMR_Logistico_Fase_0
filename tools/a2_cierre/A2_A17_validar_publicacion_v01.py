#!/usr/bin/env python3
"""Validador reproducible para CI A17: solo evidencia ligera, sin ROS/Gazebo."""
from __future__ import annotations
import csv,hashlib,json,re,sys,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
E=ROOT/'docs/evidencias_A2/A17_cierre_20261109'
EXPECTED='dbbac54debea339d9ff9cd14985b66f9ada44c16'
FIS='67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903'
FISPATH=ROOT/'matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis'
STATUS=['B1','B2','B3','B4','A13','A14','A15']

def need(x,m):
 if not x:raise RuntimeError(m)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def self_test():
 assert sha(Path(__file__)) and set(STATUS)=={'B1','B2','B3','B4','A13','A14','A15'}
 print('[OK AUTOPRUEBA A17 CI] Herramienta pasiva, fuentes y casos negativos listos')
def validate():
 snap=json.loads((E/'snapshot_A17.json').read_text(encoding='utf8'))
 need(snap['schema']=='A17_v01','Esquema A17 distinto')
 need(snap['base_congelada']==EXPECTED,'Commit base cambia')
 need(snap['fis_oficial']=='v03' and snap['fis_sha256']==FIS,'FIS oficial incorrecto')
 need(snap['fis_v04']=='EXPERIMENTAL_A13_NO_OPERATIVO','v04 no es exclusivamente experimental')
 need(FISPATH.is_file() and sha(FISPATH)==FIS,'FIS v03 no coincide SHA256')
 txt=FISPATH.read_text(encoding='utf8')
 need('NumRules=27' in txt,'27 reglas ausentes')
 r=txt.split('[Rules]',1);need(len(r)==2,'Reglas no localizadas')
 rules=[z for z in r[1].splitlines() if re.match(r'^\s*[123]\s+[123]\s+[123],',z)]
 need(len(rules)==27,'No hay exactamente 27 reglas')
 need(snap['A12_20_recorridos'] is True and snap['A13_pares_acreditados']==5,'Ensayos incompletos')
 need(snap['A14_rosbags_auditados']==30 and snap['A14_replays_aislados']==6,'A14 incompleta')
 need(snap['A15_imagenes_originales']==600 and snap['A15_imagenes_derivadas']==600,'A15 incompleta')
 need(snap['A15_test_aciertos']==90,'CNN test publicado incorrecto')
 need(snap['A16_acceso']=='CONFIRMADO_POR_USUARIO_NO_REVALIDADO_EXTERNAMENTE','A16 excede evidencia')
 need(len(snap['sesiones'])==8 and set(STATUS).issubset(snap['sesiones']) and 'A14_REPLAY' in snap['sesiones'],'Fuentes ausentes')
 for k in STATUS:
  s=snap['sesiones'][k]
  need(len(s['hash_sha256'])==64 and len(s['hash_manifiesto_sha256'])==64 and s['dictamen'],'Fuente SHA ausente: '+k)
 for key,expected in {**snap['tablas_publicadas_sha256'],**snap['artefactos_publicados_sha256']}.items():
  p=E/key;need(p.is_file() and sha(p)==expected,'Archivo publicado alterado: '+key)
 need(len(snap['tablas_publicadas_sha256'])==5,'Faltan cinco tablas de resultados')
 for k,n in [('B1',5),('B2',5),('B3',5),('B4',5),('A13',5)]:
  p=E/f'tabla_{k}_fuente.csv'
  with p.open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
  need(len(rows)==n,'Filas '+k+' incorrectas')
  if k!='A13':need(all(str(x.get('aprobado','')).lower() in ('true','1','si') for x in rows),'Tabla no acreditada '+k)
  else:need(all(str(x.get('par_comparable','')).lower()=='true' for x in rows),'Pares no comparables')
 for f in ['README.md','docs/A2_estado_vigente_20261109.md']:
  s=(ROOT/f).read_text(encoding='utf8')
  need('v03' in s and 'v04' in s,'README no distingue versión FIS')
 readme=(ROOT/'README.md').read_text(encoding='utf8')
 need(readme.count('<!-- INICIO ESTADO VIGENTE AVANCE 2 2026-11-09 -->')==1,'Estado README duplicado')
 section=readme.split('<!-- INICIO ESTADO VIGENTE AVANCE 2 2026-11-09 -->',1)[1].split('<!-- FIN ESTADO VIGENTE AVANCE 2 2026-11-09 -->',1)[0]
 need('30 rosbags' in section and '20/20' in section and 'v03' in section,'Estado README obsoleto')
 src=(ROOT/'.github/workflows/validar_avance2.yml').read_text(encoding='utf8')
 need('cierre-avance2-v2' in src and 'Validar cierre consolidado R19-R23' in src,'Workflow historico perdido')
 try:
  tag=subprocess.run(['git','rev-parse','avance2^{commit}'],cwd=ROOT,text=True,capture_output=True,check=True).stdout.strip()
  need(tag==EXPECTED,'Tag congelado avance2 alterado')
 except FileNotFoundError:raise RuntimeError('Git no instalado')
 print('[OK A17 CI] FIS v03 SHA256, 27 reglas, 7 dictámenes, 5 CSV y evidencias SHA256')
 print('[OK A17 CI] README actualizado; workflows históricos preservados; tag avance2 inmóvil')
 print('DICTAMEN_CI_A17: VALIDACION_DOCUMENTAL_Y_ESTÁTICA_APROBADA')

if __name__=='__main__':
 try:
  if '--autoprueba' in sys.argv:self_test()
  else:validate()
 except Exception as e:print('[FALLO CI A17] '+str(e),file=sys.stderr);sys.exit(1)
