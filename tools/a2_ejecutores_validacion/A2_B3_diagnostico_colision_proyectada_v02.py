#!/usr/bin/env python3
"""Diagnostico SOLO LECTURA del ultimo intento B3. NO inicia Gazebo ni ROS.

Uso WSL: python3 /mnt/c/Users/Henry/Downloads/A2_B3_diagnostico_colision_proyectada_v01.py
Opcional: python3 SCRIPT.py --carpeta /ruta/B3_obstaculo_sorpresa/20261108_192430
"""
from __future__ import annotations
import argparse
import csv
import json
import math
import sqlite3
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = (Path.home()/'AMR_Logistico_Fase_0'/'resultados'/'A2_B2_rescate_geometria_astar')
BOX_HALF = (0.10, 0.10)
HORIZON = 0.35
SAFETY = 0.255


def load_csv(path):
    with path.open(newline='', encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))


def point_rect_dist(pos, rect):
    _, a, b, c, d = rect
    x, y = pos
    return math.hypot(max(a-x, 0., x-b), max(c-y, 0., y-d))


def world_rects(path):
    result = []
    root = ET.parse(path).getroot()
    for model in root.findall('./world/model'):
        name = model.get('name', '')
        pose = [float(s) for s in model.findtext('pose', '0 0 0 0 0 0').split()]
        if len(pose) != 6:
            continue
        x, y, z, roll, pitch, yaw = pose
        for link in model.findall('link'):
            for col in link.findall('collision'):
                size = col.findtext('./geometry/box/size')
                if not size or link.find('pose') is not None or col.find('pose') is not None or any(abs(a) > 1e-8 for a in (roll,pitch,yaw)):
                    continue
                sx, sy, sz = map(float, size.split())
                if z + sz/2 < .02:
                    continue
                result.append((name,x-sx/2,x+sx/2,y-sy/2,y+sy/2))
    return result


def project(x,y,yaw,v,w,dt=HORIZON):
    if abs(w) < 1e-6:
        return (x+v*dt*math.cos(yaw), y+v*dt*math.sin(yaw))
    return (x+(v/w)*(math.sin(yaw+w*dt)-math.sin(yaw)),
            y-(v/w)*(math.cos(yaw+w*dt)-math.cos(yaw)))


def asfloat(row,key,default=None):
    try: return float(row[key])
    except (KeyError,TypeError,ValueError): return default


def read_rosbag_last_cmds(directory):
    # Twist CDR: encapsulacion de 4 bytes + 6 float64 (linear.xyz angular.xyz).
    out=[]
    for path in (directory/'rosbag2').glob('*.db3'):
        try:
            with sqlite3.connect('file:'+str(path)+'?mode=ro', uri=True) as con:
                ids={k:int(v) for v,k in con.execute('SELECT id,name FROM topics')}
                tid=ids.get('/cmd_vel_a3_prueba')
                if tid is None: continue
                for ts,raw in con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp DESC LIMIT 12',(tid,)):
                    if len(raw) < 52:continue
                    marker=raw[:2]
                    endian = '<' if marker == b'\x00\x01' else ('>' if marker == b'\x00\x00' else None)
                    if endian is None: continue
                    x,y,z,ax,ay,az=struct.unpack_from(endian+'6d',raw,4)
                    if all(math.isfinite(q) for q in (x,az)) and abs(x)<2 and abs(az)<5:
                        out.append((ts,x,az))
        except (sqlite3.Error,OSError,ValueError):
            continue
    out.sort()
    return out[-12:]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--carpeta',type=Path,default=None)
    args=parser.parse_args()
    if args.carpeta:
        run=args.carpeta.expanduser().resolve()
    else:
        folders=sorted(BASE.glob('*/B3_obstaculo_sorpresa/*/resumen.json'))
        if not folders:
            parser.error('No se encontro resumen.json B3; usa --carpeta')
        run=folders[-1].parent
    summary=run/'resumen.json'
    if not summary.is_file():parser.error('Falta '+str(summary))
    data=json.loads(summary.read_text(encoding='utf-8'))
    print('\n=== DIAGNOSTICO B3 | SOLO LECTURA ===')
    print('CARPETA:',run)
    for label,key in [('ESTADO','estado'),('FALLOS','fallos'),('BARRERAS','barreras'),
                      ('OBSTACULO INSERTADO','obstaculo_dinamico_insertado'),
                      ('DISTANCIA MINIMA','distancia_minima_a_colisiones_sdf_m'),
                      ('DESVIACION MAXIMA','desviacion_max_a_ruta_m'),
                      ('RECORRIDO','distancia_recorrida_m'),
                      ('DISTANCIA RESTANTE','distancia_final_meta_m'),
                      ('COMANDOS DIFUSOS ACTIVOS','comandos_difuso_activos_despues_spawn'),
                      ('RETORNO A RUTA','comandos_ruta_despues_difuso')]:
        print(label+':',data.get(key,'NO REGISTRADO'))
    print('EVENTO CAJA:',json.dumps(data.get('obstaculo_evento',{}),ensure_ascii=False))
    print('MODOS A3:',json.dumps(data.get('modos_a3',[])[-12:],ensure_ascii=False))
    print('EVENTOS A4:',json.dumps(data.get('eventos_a4',[])[-8:],ensure_ascii=False))
    odom=load_csv(run/'odometria.csv')
    controls=load_csv(run/'control.csv')
    print('\n=== ULTIMOS 14 CONTROLES (VALORES REALMENTE ENVIADOS) ===')
    for r in controls[-14:]:
        print(f"t={r.get('t_s','?'):>8}s modo={r.get('modo_a3','?'):<6} v={r.get('v_mps','?'):>7} "
              f"w={r.get('omega_radps','?'):>7} df={r.get('df_m','?'):>6} "
              f"scan={r.get('scan_frontal_m','?'):>6} motivo={r.get('razon','?')}")
    print('\n=== ULTIMAS 10 POSES ===')
    rects=[]
    world=data.get('mundo')
    if world and Path(world).is_file():rects=world_rects(Path(world))
    event=data.get('obstaculo_evento') or {}
    pose=event.get('pose_caja')
    if isinstance(pose,list) and len(pose)>=2:
        x,y=float(pose[0]),float(pose[1]); hx,hy=BOX_HALF
        rects.append(('CAJA_DINAMICA_B3',x-hx,x+hx,y-hy,y+hy))
    for r in odom[-10:]:
        x=asfloat(r,'x_m');y=asfloat(r,'y_m');yaw=asfloat(r,'yaw_rad')
        if x is None or y is None:continue
        d=sorted(((point_rect_dist((x,y),box),box[0]) for box in rects))
        close=(f'{d[0][1]}={d[0][0]:.4f}m' if d else 'sin geometria')
        print(f"t={r.get('t_s','?'):>8}s xy=({x: .4f},{y: .4f}) yaw={yaw} "
              f"meta={r.get('distancia_meta_m','?')} despeje={r.get('despeje_geometrico_m','?')} "
              f"cercano={close}")
    if odom and controls and rects:
        # Ultimo Twist NO-CERO registrado y pose mas cercana al mismo instante.
        active=[r for r in controls if abs(asfloat(r,'v_mps',0))>.0001 or abs(asfloat(r,'omega_radps',0))>.0001]
        if active:
            cmd=active[-1]
            ct=asfloat(cmd,'t_s',0.)
            nearest=min(odom,key=lambda r:abs(asfloat(r,'t_s',0.)-ct))
            vals=[asfloat(nearest,k) for k in ('x_m','y_m','yaw_rad')]
            if all(v is not None for v in vals):
                v,w=asfloat(cmd,'v_mps'),asfloat(cmd,'omega_radps')
                candidate=project(*vals,v,w)
                ranking=sorted((point_rect_dist(candidate,rect),rect[0]) for rect in rects)
                print('\n=== PROYECCION DEL ULTIMO COMANDO EFECTIVO (APROXIMACION) ===')
                print(f't_comando={ct:.3f}s; cmd v={v:.5f}m/s omega={w:.5f}rad/s')
                print(f'pose_asociada=({vals[0]:.4f}, {vals[1]:.4f}, yaw={vals[2]:.4f})')
                print(f'proyeccion_0.35s=({candidate[0]:.4f},{candidate[1]:.4f})')
                for dist,name in ranking[:4]:
                    print(f'  {name}: {dist:.4f}m (umbral de seguridad {SAFETY:.3f}m)')
                print('NOTA: esta proyeccion usa el ultimo comando *publicado*, no el comando A3 rechazado.')
    ros=read_rosbag_last_cmds(run)
    print('\n=== ULTIMAS PROPUESTAS A3 EN ROSBAG ===')
    if ros:
        for ts,v,w in ros[-12:]: print(f'  bag_ns={ts} v_prop={v:.5f}m/s omega_prop={w:.5f}rad/s')
    else:print('No disponibles en rosbag2; no inferir velocidad de propuesta rechazada')
    print('\nCONCLUSION: no repetir Gazebo. Entregar esta salida para distinguir giro tardio,')
    print('avance hacia caja o efecto de pared y definir UNA correccion dirigida.')
    return 0

if __name__=='__main__':
    raise SystemExit(main())
