#!/usr/bin/env python3
"""Valida evidencia capturada en Ubuntu E2.3.6A; no ejecuta ROS ni Gazebo."""
from pathlib import Path
import ast
import csv
import hashlib
import json
import math

ROOT=Path(__file__).resolve().parents[1]
SCRIPT='tools/A2_E236A_Gazebo_canal_cmd_vel_cero_v5_headless.py'
SCRIPT_SHA='9589b7f32cdfe4c3c81b5d33accd378ac8395aa13104ff28ec91c1805bdd1869'
DIR=ROOT/'resultados/A2_E236A_gazebo_headless_canal_cero/evidencia_publicada'
TOPICS={'scan','odom','df','di','dd','fuzzy','fuzzy_state','candidate','candidate_state','cmd_vel'}

def require(cond,what):
    if not cond:raise RuntimeError(what)

def validate():
    target=ROOT/SCRIPT
    require(target.is_file() and hashlib.sha256(target.read_bytes()).hexdigest()==SCRIPT_SHA,
            'Fuente E2.3.6A ausente o distinta de SHA256 del ensayo')
    ast.parse(target.read_text(encoding='utf-8'),filename=SCRIPT)
    require(DIR.is_dir(),'Directorio de evidencias E2.3.6A ausente')
    for name in ('resumen.json','observaciones.csv','factory_probe.log','gzserver_factory.log',
                 'gazebo_lanzamiento.log','monitor.log'):
        p=DIR/name
        require(p.is_file() and 20<p.stat().st_size<5_000_000,'Evidencia ausente/tamano invalido: '+str(p))
    data=json.loads((DIR/'resumen.json').read_text(encoding='utf-8'))
    require(data.get('etapa')=='E2.3.6A' and data.get('estado')=='APROBADO_CANAL_CERO',
            'Etapa/estado no aprobado')
    require(str(data.get('domain_id'))=='224','Dominio inesperado')
    require(data.get('umbrales_provisionales') is True and
            data.get('umbral_ruta_difuso_m')==[.60,.80],
            'Umbrales experimentales no declarados correctamente')
    require(data.get('comandos_ruta_todos_cero') is True and
            data.get('route_zero_count',0)>=110,
            'Propuesta de ruta no acreditada como cero')
    require(data.get('publicacion_cmd_vel') is True and
            data.get('cmd_vel_siempre_cero') is True and
            data.get('cmd_vel_observados_todos_cero') is True,
            'Canal /cmd_vel no verificado como cero')
    require(data.get('cmd_vel_publicadores_esperados')==1,
            'No coincide exclusividad del publicador')
    sent=data.get('cmd_vel_zero_sent')
    count=data.get('topic_counts',{})
    require(set(count)==TOPICS and all(type(v)==int and v>0 for v in count.values()),
            'Diez topicos o contadores invalidos')
    require(type(sent)==int and sent>=110 and 100<count['cmd_vel']<=sent+10,
            'Numero de comandos /cmd_vel no coincide')
    require(sent==data.get('route_zero_count'),'Ruta y comando cero deben coincidir')
    require(data.get('cmd_vel_zero_period_s')==.1,'Periodo de envio distinto del aprobado')
    hz=data.get('cmd_vel_frecuencia_hz_medida')
    require(isinstance(hz,(int,float)) and math.isfinite(hz) and 8<=hz<=12,
            'Frecuencia /cmd_vel fuera de 8-12 Hz')
    displacement=data.get('desplazamiento_odom_m')
    require(isinstance(displacement,(int,float)) and math.isfinite(displacement) and
            0<=displacement<.05,'Desplazamiento excede 5 cm o ausente')
    modes=('RUTA','DIFUSO','DETENIDO','DIFUSO','RUTA')
    stages=data.get('transiciones',[])
    require(len(stages)==5,'Se esperan cinco transiciones')
    for i,(row,mode) in enumerate(zip(stages,modes),1):
        require(row.get('estado','').startswith(mode+'|'),'Modo incorrecto paso '+str(i))
        d=row.get('scan_m'); rng=row.get('range_m')
        v=row.get('v');w=row.get('omega')
        require(all(isinstance(z,(int,float)) and math.isfinite(z)
                    for z in (d,rng,v,w)),'Valores no finitos en etapa '+str(i))
        require(0<=v<=.120001 and abs(w)<=.700001,
                'Propuesta excede limite '+str(i))
        if mode in ('RUTA','DETENIDO'):
            require(abs(v)<1e-6 and abs(w)<1e-6,'Parada o ruta no cero '+str(i))
        else:
            require(.35<d<.60 and 0<v<=.12,'DIFUSO incoherente '+str(i))
        if mode=='DETENIDO':
            require(d<=.35,'Obstaculo cercano no detenido')
        if mode=='RUTA':
            require(d>=.8,'Ruta en distancia de peligro')
    with (DIR/'observaciones.csv').open(encoding='utf-8',newline='') as fh:
        reader=csv.DictReader(fh)
        require(reader.fieldnames==['seg','topico','valor'],'Encabezado observaciones invalido')
        records=list(reader)
    require(len(records)>200,'Insuficientes observaciones CSV')
    seen={r['topico'] for r in records}
    require(TOPICS.issubset(seen),'CSV no registra los diez topicos')
    require(all(r['valor']=='CERO' for r in records if r['topico']=='cmd_vel'),
            'CSV /cmd_vel contiene valores distintos de CERO')
    require(sum(r['topico']=='cmd_vel' for r in records)>100,'CSV menos de 101 cmd_vel')
    factory=(DIR/'factory_probe.log').read_text(encoding='utf8',errors='replace')
    launch=(DIR/'gazebo_lanzamiento.log').read_text(encoding='utf8',errors='replace')
    monitor=(DIR/'monitor.log').read_text(encoding='utf8',errors='replace')
    require('Factory disponible antes de insertar el robot' in factory,'Factory no confirmada')
    require('Successfully spawned entity [waffle_pi]' in launch,'Robot no insertado')
    require('UNICO /cmd_vel CERO Y ARBITRAJE SOMBRA APROBADOS' in monitor,
            'Monitor no confirmo aprobacion')
    require('no_cero=False' in monitor,'Monitor no confirmo comandos cero')
    print('[OK] E2.3.6A: Robot insertado con ROS Factory headless')
    print('[OK] E2.3.6A: 10 topicos; cinco transiciones y parada LiDAR')
    print('[OK] E2.3.6A: un publicador /cmd_vel, >100 mensajes solo cero a 8-12 Hz')
    print('[OK] E2.3.6A: inmovilidad <0,05 m; fuente autenticada SHA256')
    print('=== E2.3.6A: EVIDENCIA GAZEBO CANAL CERO APROBADA; R19 PENDIENTE ===')

if __name__=='__main__':
    validate()
