#!/usr/bin/env python3
"""Verifica capturas E2.3.4-E2.3.5 hechas en Ubuntu: NO ejecuta Gazebo/ROS."""
from pathlib import Path
import ast
import csv
import hashlib
import json
import math
import sys

ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
    'tools/A2_E234_Gazebo_sombra_seguro.py': 'dafa7fe42b58580bca9caacee21e1c5af43174eba0033e911fc514ed282696f9',
    'tools/A2_E235_Gazebo_transiciones_sombra_v3.py': '42bd98c561014db90c1a3eb129745aa1d5c9215c3f5ad067212a537d09184984',
}
TOPICS = {'scan','odom','df','di','dd','fuzzy','fuzzy_state','candidate','candidate_state'}
BASE = 'resultados/'

def require(value, message):
    if not value:raise ValueError(message)

def check_csv(folder, expected):
    path=ROOT/folder/'observaciones.csv'
    require(path.is_file(),str(path))
    with path.open(encoding='utf-8',newline='') as f:
        reader=csv.DictReader(f)
        require(set(expected).issubset(set(reader.fieldnames or [])),'CSV: cabecera inesperada '+folder)
        rows=list(reader)
    require(len(rows)>=100,'CSV insuficiente '+folder)
    return rows

def check_logs(folder):
    for name in ('monitor.log','gazebo_lanzamiento.log'):
        p=ROOT/folder/name
        require(p.is_file() and 100<p.stat().st_size<5_000_000,'Registro ausente/inválido '+str(p))

def check():
    for path, sha in SOURCES.items():
        target=ROOT/path
        require(target.is_file(),'Falta código: '+path)
        require(hashlib.sha256(target.read_bytes()).hexdigest()==sha,'Código cambiado: '+path)
        ast.parse(target.read_text(encoding='utf-8'),filename=path)
    path234='resultados/A2_E234_gazebo_sombra/evidencia_publicada'
    j=json.loads((ROOT/path234/'resumen.json').read_text(encoding='utf-8'))
    require(j.get('etapa')=='E2.3.4' and j.get('estado')=='APROBADA_SOMBRA','E2.3.4 no aprobada')
    require(j.get('domain_id')=='223' and j.get('solo_loopback') is True,'E2.3.4 dominio')
    require(j.get('gazebo') is True and j.get('publication_cmd_vel') is False,'E2.3.4 seguridad')
    require(not j.get('errors'),'E2.3.4 contiene errores')
    c=j.get('topic_counts',{})
    require(set(c)==TOPICS and all(isinstance(x,int) and x>0 for x in c.values()),'E2.3.4 nueve tópicos')
    require('ACTIVO' in j.get('initial_states',{}).get('fuzzy',[]),'E2.3.4 difuso no activo')
    check_csv(path234,{'t_sec','topic','detail'})
    check_logs(path234)
    print('[OK] E2.3.4: Gazebo, 9 tópicos y aislamiento verificados')

    path235='resultados/A2_E235_gazebo_transiciones_sombra/evidencia_publicada'
    j=json.loads((ROOT/path235/'resumen.json').read_text(encoding='utf-8'))
    require(j.get('etapa')=='E2.3.5' and j.get('estado')=='APROBADA_SOMBRA','E2.3.5 no aprobada')
    require(j.get('domain_id')=='224' and j.get('publicacion_cmd_vel') is False,'E2.3.5 aislamiento')
    require(j.get('comandos_ruta_todos_cero') is True and j.get('route_zero_count',0)>100,'E2.3.5 ruta cero')
    require(j.get('umbrales_provisionales') is True and j.get('umbral_ruta_difuso_m')==[0.60,0.80], 'E2.3.5 umbrales')
    c=j.get('topic_counts',{})
    require(set(c)==TOPICS and all(isinstance(x,int) and x>0 for x in c.values()),'E2.3.5 nueve tópicos')
    transitions=j.get('transiciones',[])
    expected=('RUTA','DIFUSO','DETENIDO','DIFUSO','RUTA')
    require(len(transitions)==5,'E2.3.5: no hay cinco transiciones')
    for i,(t,mode) in enumerate(zip(transitions,expected),1):
        require(t.get('estado','').startswith(mode+'|'),'Modo incorrecto, paso '+str(i))
        v,w=t.get('v'),t.get('omega')
        require(all(isinstance(x,(int,float)) and math.isfinite(x) for x in (v,w)),'Twist no finito')
        require(0<=v<=.120001 and abs(w)<=.700001,'Saturación excedida')
        if mode in ('RUTA','DETENIDO'):
            require(abs(v)<1e-6 and abs(w)<1e-6,'Parada/ruta no cero')
        else:require(v>0,'Propuesta difusa no positiva')
        d=t.get('scan_m')
        require(isinstance(d,(int,float)) and math.isfinite(d),'Falta LiDAR')
        if i in (1,5):require(d>=.85,'Pasillo libre no comprobado')
        elif i in (2,4):require(.35<d<=.60,'Obstáculo intermedio fuera del rango')
        else:require(d<=.35 and 'PARADA_' in t['estado'],'Parada LiDAR no demostrada')
    check_csv(path235,{'seg','topico','valor'})
    check_logs(path235)
    print('[OK] E2.3.5: RUTA>DIFUSO>DETENIDO>DIFUSO>RUTA, parada estricta y /cmd_vel aislado')
    print('=== E2.3.4-E2.3.5: EVIDENCIAS GAZEBO EN SOMBRA APROBADAS ===')
    print('[PENDIENTE] R19 operativo; umbrales 0.60/0.80 experimentales. CI no ejecutó Gazebo.')

if __name__ == '__main__':
    try:check()
    except Exception as exc:
        print('[ERROR GAZEBO SOMBRA]',str(exc),flush=True)
        sys.exit(1)
