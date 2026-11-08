#!/usr/bin/env python3
"""A2/R22-R23 - v6 bateria integral CNN, encuadre contextual y GUI.

Descargar a C:/Users/Henry/Downloads y ejecutar desde Ubuntu WSL:
    python3 /mnt/c/Users/Henry/Downloads/A2_R22_R23_bateria_integral_CNN_v6_camara_contextual.py --reanudar --visual

Opciones: --visual, --reanudar, --solo-captura, --solo-entrenar, --self-test.
Correccion v6: el SDF temporal del sensor usa 145 grados horizontales,
calibracion de desplazamiento longitudinal del centro de camara (a partir
las poses SDF del sensor y link), y una serie nueva independiente para
no mezclar imagenes obtenidas con diferentes parametros opticos. Exige
al menos un 2.5 % de contexto cromatico no perteneciente al panel en el
recorte de control, sin bajar el umbral minimo de variacion. No se
sobrescribe ni elimina ninguna imagen o evidencia v5.
Correccion v5: si un encuadre no cumple el control de calidad, prueba hasta
12 poses proximas conservando la misma estacion, rango de distancia y angulo
nominal. No rebaja el umbral de calidad ni acepta PNG de diagnostico; registra
cada intento y conserva las capturas v4 autenticas ya guardadas con SHA256.
Correccion v4: el cliente grafico se inicia inmediatamente tras ROS Factory
 y el spawn, ANTES de esperar la primera imagen para evitar un bloqueo circular.
 La grafica es opcional; los errores de la GUI no se confunden con los del servidor.
Correccion v3: deteccion dinamica de topicos Image y diagnosticos de camara.
Correccion v2: un launch temporal que contiene solo spawn_entity.py puede
terminar con codigo cero tras crear el robot; NO implica cierre de gzserver.
Se tolera su finalizacion normal y se audita la vida real de gzserver.
No modifica archivos rastreados ni publica commits. Una sola sesion Gazebo,
servicios Factory y State validados; NO publica comandos de movimiento.
Las poses se realizan exclusivamente con teleport en simulacion aislada.
Se conserva el PNG 640x360 RGB original de cada muestra y se construye
un dataset de 224x224 solo durante entrenamiento, sin sobreescribir PNG.
El modelo CNN es una linea base preliminar R23, NO MobileNetV2 final R24.
"""
from __future__ import annotations
import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import random
import shutil
import signal
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import zlib

REPO=Path.home()/'AMR_Logistico_Fase_0'
ROOT=REPO/'resultados/A2_R22_R23_CNN_integral_v6_contexto'
OLD_ROOT=REPO/'resultados/A2_R22_R23_CNN_integral'
BASE_WORLD=REPO/'modelos/almacen_f1.world'
ACTIVATE=REPO/'tools/activar_entorno.sh'
ROS_PY=Path.home()/'.venvs/amr-a2-r18/bin/python'
CNN_ENV=Path.home()/'.venvs/amr-a2-cnn'
DOMAIN='224'
EXPECTED_HEAD='82ba1d98a4201b01e7eaaa42146c163808dcffec'
SEED=20261108
STATIONS=(
    # class_id, class_name, panel x, panel y, robot side (+1 east of panel / -1 west)
    (0,'RECEPCION',-4.73,-2.80,+1),
    (1,'ALMACENAMIENTO',-4.73,+2.80,+1),
    (2,'EMPAQUE',+4.73,+2.80,-1),
    (3,'DESPACHO',+4.73,-2.80,-1),
)
DISTS=(.30,.60,.90,1.20,1.50)
YAW_OFFSETS=(-30,-15,0,15,30)
CAPTURES=6
MIN_COLOR_FRACTION=.0008  # 0.08% de pixeles cromaticos.
MIN_LUMINANCE_STD=6.0  # No se rebaja el umbral que detecto la saturacion v5.
MAX_COLOR_FRACTION=.975  # Panel no puede ocupar practicamente todo el fotograma central.
CAMERA_HFOV_DEG=145.0  # FOV horizontal del sensor temporal, uniforme en TODAS las capturas.
CAMERA_FORWARD_OFFSET_M=0.0  # Calibrado de SDF y leido del dataset al ejecutar ROS.
MAX_CAPTURE_ATTEMPTS=12  # 1 nominal + 11 reencuadres deterministicos.
MODEL_WIDTH,MODEL_HEIGHT=640,360
STATE_PLUGIN='''  <plugin name="a2_cnn_state" filename="libgazebo_ros_state.so">
    <ros><namespace>/gazebo</namespace></ros>
    <update_rate>20.0</update_rate>
  </plugin>\n'''
LAUNCH='''#!/usr/bin/env python3
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    model=Path(__file__).parent/'waffle_pi_cnn_temporal.sdf'
    return LaunchDescription([
        Node(package='gazebo_ros',executable='spawn_entity.py',output='screen',
            arguments=['-entity','waffle_pi','-file',str(model),
                       '-x','-3.0','-y','-2.8','-z','0.05','-Y','0.0']),
    ])
'''


def log(msg):print(msg,flush=True)
def check(cond,msg):
    if not cond:raise RuntimeError(msg)
    log('[OK] '+msg)

def git(*args):
    q=subprocess.run(['git',*args],cwd=REPO,text=True,capture_output=True,timeout=25)
    if q.returncode:raise RuntimeError('git '+ ' '.join(args)+': '+q.stderr.strip())
    return q.stdout.strip()

def env():
    e=os.environ.copy();e['ROS_DOMAIN_ID']=DOMAIN;e['ROS_LOCALHOST_ONLY']='1'
    e['TURTLEBOT3_MODEL']='waffle_pi';e['RCUTILS_COLORIZED_OUTPUT']='0'
    # WSL2 suele funcionar con Mesa sin aceleracion NVIDIA: no modificar el entorno original.
    e.setdefault('LIBGL_ALWAYS_SOFTWARE','1')
    # Evitar busquedas externas de modelos durante la carga de Gazebo Classic.
    # El mundo original usa modelos base ya instalados localmente.
    e.setdefault('GAZEBO_MODEL_DATABASE_URI','')
    return e

def sourced(args):
    return ['bash','-c','source "$1" >/dev/null && shift && exec "$@"','bash',str(ACTIVATE),*args]

def child(args):
    return sourced([str(ROS_PY),'-u',str(Path(__file__).resolve()),*args])

def kill_own(p,timeout=8):
    if p is None or p.poll() is not None:return
    for sig,wait in ((signal.SIGINT,timeout),(signal.SIGTERM,4),(signal.SIGKILL,2)):
        if p.poll() is not None:break
        try:os.killpg(p.pid,sig)
        except ProcessLookupError:break
        try:p.wait(timeout=wait)
        except subprocess.TimeoutExpired:pass

def runtime_check(gzserver_rc, spawn_launch_rc):
    """Check real server liveness; normal spawn-only launch exit 0 is acceptable.

    Returns a short status; raises on fatal process errors. Not ROS-dependent.
    """
    if gzserver_rc is not None:
        raise RuntimeError(f'gzserver termino durante captura (codigo {gzserver_rc})')
    if spawn_launch_rc is not None and spawn_launch_rc != 0:
        raise RuntimeError(f'ros2 launch de insercion fallo (codigo {spawn_launch_rc})')
    return 'spawn_launch_terminado_correctamente' if spawn_launch_rc == 0 else 'spawn_launch_activo'


def log_tail(path, title, limit=10):
    try:
        if path.is_file():
            lines=path.read_text(encoding='utf8',errors='replace').splitlines()
            log(f'[DIAGNOSTICO] {title}: {path}')
            for line in lines[-limit:]:
                log('  '+line[:350])
    except OSError as exc:
        log(f'[DIAGNOSTICO] No se pudo leer {path.name}: {exc}')


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def world_with_state(s):
    root=ET.fromstring(s)
    check(root.tag=='sdf' and len(root.findall('world'))==1,'Mundo temporal SDF parseable')
    if 'libgazebo_ros_state.so' in s:raise RuntimeError('El mundo ya tiene gazebo_ros_state')
    if s.count('</world>')!=1:raise RuntimeError('Se esperaba un solo cierre world')
    return s.replace('</world>',STATE_PLUGIN+'</world>')

def camera_forward_offset(s):
    """Estimacion geometrica del centro optico respecto al origen del robot.

    Pose SDF de link y sensor; sus roll/pitch/yaw se registran y se exige
    orientacion longitudinal para no usar una calibracion invalida.
    Esta magnitud es una aproximacion documentada, no una calibracion fisica.
    """
    root=ET.fromstring(s)
    findings=[]
    for link in root.findall('.//link'):
        for sensor in link.findall('sensor'):
            if sensor.get('type')!='camera':continue
            lp=link.find('pose');sp=sensor.find('pose')
            def coord(el):
                vals=[float(x) for x in (el.text or '').split()] if el is not None else []
                if not vals:return [0.]*6
                if len(vals)!=6 or not all(math.isfinite(v) for v in vals):
                    raise RuntimeError('Pose SDF de camara/link no interpretable')
                return vals
            lv,sv=coord(lp),coord(sp)
            # Se considera el desplazamiento longitudinal. Registrar si hay
            # rotacion del link que pueda afectar a la posicion exacta.
            v=lv[0]+sv[0]
            if abs(v)>.35:
                raise RuntimeError(f'Offset camara fuera de rango, revisar modelo SDF: {v:.3f}m')
            findings.append(v)
    if len(findings)!=1:
        raise RuntimeError(f'Se esperaba una camara para 600 fotos; hay {len(findings)}')
    return findings[0]


def context_geometry(distance_m, fov_deg=CAMERA_HFOV_DEG):
    """Huella horizontal y vertical teorica del plano a la distancia de trabajo."""
    tan_half=math.tan(math.radians(fov_deg/2))
    return (2*distance_m*tan_half,
            2*distance_m*tan_half*MODEL_HEIGHT/MODEL_WIDTH)


def make_model(s,cam_pitch_deg=12.0):
    """Copia efimera del SDF: camara 640x360 @8Hz, inclinacion leve hacia arriba.

    El modelo original instalado permanece intacto. La inclinacion se declara
    como una modificacion del montaje de sensor SOLO en el dataset simulado.
    """
    root=ET.fromstring(s)
    cams=root.findall('.//sensor[@type="camera"]')
    if not cams:raise RuntimeError('Modelo Waffle Pi sin sensor camera en SDF')
    for sensor in cams:
        cfg=sensor.find('camera')
        if cfg is None:raise RuntimeError('Sensor camera sin bloque camera')
        fov=cfg.find('horizontal_fov')
        if fov is None:
            fov=ET.Element('horizontal_fov')
            cfg.insert(0,fov)
        fov.text=f'{math.radians(CAMERA_HFOV_DEG):.10f}'
        img=cfg.find('image')
        if img is None:raise RuntimeError('Sensor camera sin image')
        for key,value in [('width',str(MODEL_WIDTH)),('height',str(MODEL_HEIGHT))]:
            tag=img.find(key)
            if tag is None:tag=ET.SubElement(img,key)
            tag.text=value
        rate=sensor.find('update_rate')
        if rate is None:rate=ET.SubElement(sensor,'update_rate')
        rate.text='8'
        active=sensor.find('always_on')
        if active is None:active=ET.SubElement(sensor,'always_on')
        active.text='true'
        viz=sensor.find('visualize')
        if viz is None:viz=ET.SubElement(sensor,'visualize')
        viz.text='false' # Evitar la visualizacion de la frusta de camara en OGRE
        pose=sensor.find('pose')
        if pose is None:
            pose=ET.SubElement(sensor,'pose')
            sensor_pose=[0.0]*6
        else:
            sensor_pose=[float(x) for x in (pose.text or '').split()]
            if len(sensor_pose)!=6:
                raise RuntimeError('Pose de camara inesperada; no cambiar montaje')
        # Mantener traslacion, roll y yaw originales; corregir solo pitch temporal.
        sensor_pose[4]-=math.radians(cam_pitch_deg)
        pose.text=' '.join(f'{v:.8f}' for v in sensor_pose)
    output=ET.tostring(root,encoding='unicode')
    ET.fromstring(output)
    return output

def split_groups():
    """70/15/15 exacto sin dispersar replicas de una misma pose entre splits.
    100 grupos de seis: 70 train, 15 val, 15 test. Balanceados entre estaciones.
    """
    # grupos por clase (train,val,test); suma por clase 25 y total (70,15,15)
    quotas=[(18,4,3),(17,4,4),(17,4,4),(18,3,4)]
    rng=random.Random(SEED)
    all_groups={}
    for (idx,name,*_),q in zip(STATIONS,quotas):
        groups=[(di,ai) for di in range(5) for ai in range(5)]
        rng.shuffle(groups)
        nt,nv,nte=q
        for j,p in enumerate(groups):
            partition='train' if j<nt else ('val' if j<nt+nv else 'test')
            all_groups[(idx,*p)]=partition
    counts={x:sum(CAPTURES for p in all_groups.values() if p==x)
            for x in ('train','val','test')}
    assert counts=={'train':420,'val':90,'test':90},counts
    return all_groups

def expected_rows():
    assignments=split_groups();rows=[]
    for idx,name,px,py,side in STATIONS:
        for di,dist in enumerate(DISTS):
            for ai,offset in enumerate(YAW_OFFSETS):
                partition=assignments[(idx,di,ai)]
                for cap in range(CAPTURES):
                    sample=f'S{idx+1:02d}_D{di+1:02d}_A{ai+1:02d}_C{cap+1:02d}'
                    rows.append({'id':sample,'clase':name,'clase_id':idx,
                        'particion':partition,'distancia_m':dist,'angulo_deg':offset,
                        'captura':cap+1,'grupo':f'S{idx+1:02d}_D{di+1:02d}_A{ai+1:02d}',
                        'archivo':f'imagenes/{partition}/{name}/{sample}.png'})
    assert len(rows)==600
    return rows

def png_rgb(data,width,height):
    """Encoder PNG puro Python: imagen RGB compacta, sin cv_bridge/Pillow."""
    assert len(data)==width*height*3
    def chunk(kind,content):
        return struct.pack('!I',len(content))+kind+content+struct.pack('!I',zlib.crc32(kind+content)&0xffffffff)
    rows=b''.join(b'\x00'+data[y*width*3:(y+1)*width*3] for y in range(height))
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',width,height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(rows,6))+chunk(b'IEND',b'')

def img_to_rgb(msg):
    import numpy as np
    enc=msg.encoding.lower()
    if enc in ('rgb8','bgr8','8uc3'):channels=3
    elif enc in ('rgba8','bgra8'):channels=4
    elif enc in ('mono8','8uc1'):channels=1
    else:raise RuntimeError(f'Encoding de camara no compatible: {msg.encoding}')
    h,w,step=int(msg.height),int(msg.width),int(msg.step)
    if w<224 or h<224 or step<w*channels or len(msg.data)<h*step:
        raise RuntimeError(f'Image invalida o resolucion insuficiente: {w}x{h} step={step}')
    a=np.frombuffer(msg.data,dtype=np.uint8,count=h*step).reshape(h,step)[:,:w*channels].reshape(h,w,channels)
    if channels==1:a=np.repeat(a,3,axis=2)
    elif channels==4:a=a[:,:,:3]
    if enc in ('bgr8','bgra8'):a=a[:,:,::-1]
    return np.ascontiguousarray(a)

def color_metric(img,name):
    import numpy as np
    arr=img[img.shape[0]//12:img.shape[0]*11//12,img.shape[1]//12:img.shape[1]*11//12]
    a=arr.astype(np.int16,copy=False)
    r,g,b=a[:,:,0],a[:,:,1],a[:,:,2]
    if name=='RECEPCION':mask=(b>65)&(b>r*1.18)&(b>g*1.11)
    elif name=='ALMACENAMIENTO':mask=(r>65)&(r>g*1.16)&(r>b*1.17)
    elif name=='EMPAQUE':mask=(g>65)&(g>r*1.16)&(g>b*1.10)
    elif name=='DESPACHO':mask=(r>70)&(g>65)&(b<np.minimum(r,g)*.78)&(np.abs(r-g)<100)
    else:raise ValueError(name)
    lum=arr.astype('float32').mean(axis=2)
    return {'cobertura_color':round(float(np.mean(mask)),6),
            'variacion_luminancia':round(float(np.std(lum)),3),
            'brillo_medio':round(float(lum.mean()),3)}

def verify_capture(row,record,path):
    if record.get('id')!=row['id'] or not path.is_file():return False
    try:
        if digest(path)!=record.get('sha256'):return False
        if path.stat().st_size<500:return False
        if record.get('grupo')!=row['grupo'] or record.get('particion')!=row['particion']:return False
        return True
    except (OSError,ValueError):return False

def records_load(folder):
    records={};p=folder/'registros.jsonl'
    if not p.exists():return records
    for line in p.read_text(encoding='utf8').splitlines():
        if line.strip():
            obj=json.loads(line)
            if obj['id'] in records:raise RuntimeError('ID duplicado en registro: '+obj['id'])
            records[obj['id']]=obj
    return records

def export_manifest(folder):
    rows=expected_rows();saved=records_load(folder)
    actual=[]
    for x in rows:
        r=saved.get(x['id'])
        if r is None:continue
        if not verify_capture(x,r,folder/x['archivo']):raise RuntimeError('PNG corrupto: '+x['id'])
        actual.append(r)
    fp=folder/'manifiesto.csv'
    fields=['id','clase','clase_id','particion','distancia_m','angulo_deg','captura','grupo',
            'archivo','sha256','bytes','cobertura_color','variacion_luminancia',
            'brillo_medio','intento_adquisicion','pose_x','pose_y','pose_yaw_deg','stamp_sec',
            'hfov_deg','offset_frontal_camara_m']
    with fp.open('w',newline='',encoding='utf8') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore',lineterminator='\n')
        w.writeheader();w.writerows(actual)
    return actual

def audit(folder,require_complete=True):
    rows=expected_rows();known=records_load(folder)
    if require_complete and len(known)!=600:raise RuntimeError(f'Dataset incompleto: {len(known)}/600')
    for row in rows:
        rec=known.get(row['id'])
        if rec is None:continue
        if not verify_capture(row,rec,folder/row['archivo']):raise RuntimeError('Evidencia no valida: '+row['id'])
        if not quality_ok({'cobertura_color':float(rec['cobertura_color']),
                           'variacion_luminancia':float(rec['variacion_luminancia'])}):
            raise RuntimeError('Fallo calidad registrado: '+row['id'])
    if any(k not in {x['id'] for x in rows} for k in known):raise RuntimeError('IDs desconocidos en dataset')
    if require_complete:
        # Prohibir archivos exactamente duplicados incluso entre grupos de distintas clases.
        seen_hash={}
        for r in known.values():
            other=seen_hash.get(r['sha256'])
            if other is not None:
                raise RuntimeError('Imagen duplicada exacta: '+other+' y '+r['id'])
            seen_hash[r['sha256']]=r['id']
        groups={}
        for r in known.values():
            groups.setdefault(r['grupo'],set()).add(r['particion'])
        if len(groups)!=100 or any(len(v)!=1 for v in groups.values()):
            raise RuntimeError('Fuga de grupos entre particiones')
        from collections import Counter
        cnt=Counter(x['particion'] for x in known.values())
        if cnt!={'train':420,'val':90,'test':90}:
            raise RuntimeError(f'Particiones incorrectas: {cnt}')
    actual=export_manifest(folder)
    if require_complete:
        info={'estado':'R22_CAPTURA_APROBADA_R23_ENTRENAMIENTO_PENDIENTE',
              'total':600,'clases':{name:150 for _,name,*_ in STATIONS},
              'particiones':{'train':420,'val':90,'test':90},
              'grupos':100,'politica_split':'6 capturas de cada pose pertenecen a una unica particion',
              'dimensiones_fuente':[MODEL_WIDTH,MODEL_HEIGHT],
              'dimensiones_entrenamiento':[224,224],
              'camara_inclinacion_temporal_deg':12,
              'camara_hfov_temporal_deg':CAMERA_HFOV_DEG,
              'max_color_permitido_fraccion':MAX_COLOR_FRACTION,
              'configuracion_optica_sha256':digest(folder/'configuracion_optica.json'),
              'origen':'camara simulada Waffle Pi, modelo SDF temporal; world oficial intacto',
              'advertencia':'No equivale a validacion de camara fisica o dominio real',
              'sha256_manifiesto':digest(folder/'manifiesto.csv')}
        (folder/'resumen_dataset.json').write_text(json.dumps(info,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    return len(actual)

def preflight():
    log('=== R22/R23 - PREFLIGHT DATASET CNN ===')
    check((REPO/'.git').is_dir(),'Repositorio Git presente')
    check(git('branch','--show-current')=='desarrollo-avance2','Rama desarrollo-avance2')
    head=git('rev-parse','HEAD')
    check(head==EXPECTED_HEAD,'Misma base validada E2.3.6A, sin commits intermedios')
    for p in ['modelos/almacen_f1.world','launch/almacen_f1.launch.py',
              'tools/supervisor_f13.py','matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis']:
        check(git('hash-object','--',p)==git('rev-parse','HEAD:'+p),'Archivo original sin alteracion: '+p)
    check(BASE_WORLD.is_file() and ROS_PY.is_file() and ACTIVATE.is_file(),
          'Mundo y entorno ROS 2 Humble disponibles')
    procs=[]
    for p in Path('/proc').iterdir():
        if p.name.isdigit():
            try:
                if (p/'comm').read_text().strip() in ('gzserver','gzclient'):procs.append(p.name)
            except (OSError,PermissionError,UnicodeError):pass
    check(not procs,'Sin otras instancias de Gazebo: '+(','.join(procs)))
    space=shutil.disk_usage(REPO).free
    check(space>=1_000_000_000,'Espacio libre >= 1 GB para dataset y CNN')
    check(len(expected_rows())==600,'Plan de 600 capturas preparado; particion 420/90/90')

def probe():
    import rclpy
    from rclpy.node import Node
    rclpy.init();n=Node('a2_cnn_dominio_libre',enable_rosout=False)
    try:
        until=time.monotonic()+2.2
        while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.1)
        topics=[x for x,_ in n.get_topic_names_and_types() if x not in ('/parameter_events','/rosout')]
        check(not topics and n.count_publishers('/cmd_vel')==0,'Dominio ROS 224 aislado y sin publicadores')
    finally:
        n.destroy_node();rclpy.shutdown()
    return 0

def factory_probe():
    import rclpy
    from rclpy.node import Node
    rclpy.init();n=Node('a2_cnn_factory',enable_rosout=False)
    try:
        limit=time.monotonic()+50
        while time.monotonic()<limit:
            rclpy.spin_once(n,timeout_sec=.2)
            if 'gazebo_msgs/srv/SpawnEntity' in dict(n.get_service_names_and_types()).get('/spawn_entity',[]):
                check(True,'/spawn_entity disponible ANTES del robot');return 0
        raise RuntimeError('Timeout Factory: revisar gzserver.log')
    finally:
        n.destroy_node();rclpy.shutdown()

def station_pose(station,di,ai,cap=0):
    idx,name,px,py,side=station
    d=DISTS[di];a=YAW_OFFSETS[ai]
    # Pose base: frente al panel en eje horizontal normal; el yaw mira hacia el panel.
    base_yaw=math.pi if side==1 else 0.
    # Perturbaciones pequenas reproducibles; permanecen dentro de grupo experimental.
    rng=random.Random(SEED+idx*100000+di*10000+ai*100+cap)
    distance_jitter=0. if cap==0 else rng.uniform(-.008,.008)
    y_jitter=0. if cap==0 else rng.uniform(-.009,.009)
    yaw_jitter=0. if cap==0 else math.radians(rng.uniform(-1.25,1.25))
    yaw=base_yaw+math.radians(a)+yaw_jitter
    # La distancia declarada es del CENTRO OPTICO al plano del panel, no
    # solamente del origen base_link del robot. Calibracion aproximada del
    # desplazamiento hacia delante de la camara extraido del SDF original.
    angle=math.radians(a)+yaw_jitter
    x=px+side*(d+distance_jitter+CAMERA_FORWARD_OFFSET_M*math.cos(angle))
    y=py+y_jitter-CAMERA_FORWARD_OFFSET_M*math.sin(yaw)
    return x,y,yaw

def pose_for_attempt(station, di, ai, cap, attempt):
    """Mantiene el objetivo nominal, modifica solo el punto de vista en fallo.

    Delta lateral <= 0.078 m; delta en distancia normal <= 0.018 m;
    delta yaw <= 7.5 grados. No confunde la etiqueta del grupo con el
    posicionamiento nominal: la pose medida y el numero de intento se guardan.
    """
    if not 0 <= attempt < MAX_CAPTURE_ATTEMPTS:
        raise ValueError('Intento de captura fuera de rango')
    x, y, yaw = station_pose(station, di, ai, cap)
    if attempt == 0:
        return x, y, yaw
    idx, name, px, py, side = station
    magnitude = min(.078, .018 + .012 * ((attempt - 1) // 2))
    direction = -1 if (attempt % 2) else 1
    y += direction * magnitude
    # Desplazamientos normales alternados, dentro de la tolerancia de pose.
    normal_shift = ((attempt - 1) % 3 - 1) * .009
    x += side * normal_shift
    yaw += direction * math.radians(min(7.5, 2.0 + 1.0 * ((attempt - 1) // 2)))
    if not (-4.85 <= x <= 4.85 and -3.84 <= y <= 3.84):
        raise RuntimeError('Reencuadre saldria de los limites del almacen')
    return x, y, yaw


def quality_ok(metrics):
    return (metrics['variacion_luminancia'] >= MIN_LUMINANCE_STD
            and MIN_COLOR_FRACTION <= metrics['cobertura_color'] <= MAX_COLOR_FRACTION)


def capture(folder):
    global CAMERA_FORWARD_OFFSET_M
    setup=json.loads((folder/'configuracion_optica.json').read_text(encoding='utf8'))
    check(abs(setup['hfov_horizontal_deg']-CAMERA_HFOV_DEG)<1e-9,
          'FOV de la serie coincide con script, sin mezcla de opticas')
    CAMERA_FORWARD_OFFSET_M=float(setup['offset_frontal_camara_m'])
    check(abs(CAMERA_FORWARD_OFFSET_M)<.35,
          f'Offset SDF de camara calibrado: {CAMERA_FORWARD_OFFSET_M:+.4f}m')
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from gazebo_msgs.srv import SetEntityState
    from sensor_msgs.msg import Image,LaserScan
    from nav_msgs.msg import Odometry
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n=Node('a2_cnn_dataset_monitor')
    state=n.create_client(SetEntityState,'/gazebo/set_entity_state')
    latest={'img':None,'img_t':0.,'image_seq':0,'img_topic':None,'pose':None,'pose_t':0.,'scan_t':0.}
    image_subs={};discovery_log=set()
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    qos_reliable=QoSProfile(depth=3,reliability=ReliabilityPolicy.RELIABLE)
    def img_cb(m,topic):
        if latest['img_topic'] is None or topic==latest['img_topic']:
            latest['img']=m;latest['img_t']=time.monotonic();latest['image_seq']+=1
            latest['img_topic']=topic
    def image_topics_discover():
        for topic,types in n.get_topic_names_and_types():
            if 'sensor_msgs/msg/Image' not in types or topic in image_subs:
                continue
            # No aceptar depth o IR como RGB; se valida encoding al recibir.
            if any(k in topic.lower() for k in ('depth','infrared','/ir/')):
                continue
            cbs=[]
            for qos in (qos_profile_sensor_data,qos_reliable):
                sub=n.create_subscription(Image,topic,lambda m,t=topic:img_cb(m,t),qos)
                cbs.append(sub)
            image_subs[topic]=cbs
            log('[CAMARA] Descubierto '+topic+' tipo sensor_msgs/msg/Image; pub='+str(n.count_publishers(topic)))
    def camera_diagnostic(reason):
        topics=n.get_topic_names_and_types()
        info={}
        for name,types in topics:
            if any(z in name.lower() for z in ('camera','image','odom','scan','clock')):
                info[name]={'types':types,'publishers':n.count_publishers(name),'subscribers':n.count_subscribers(name)}
        info['_diagnostico']={'motivo':reason,'imagen_recibida':latest['img'] is not None,
            'img_topic':latest['img_topic'],'odom_recibida':latest['pose'] is not None,
            'lidar_recibido':latest['scan_t']>0,'dominio':os.environ.get('ROS_DOMAIN_ID'),
            'DISPLAY':os.environ.get('DISPLAY'),'WAYLAND_DISPLAY':os.environ.get('WAYLAND_DISPLAY')}
        path=folder/'diagnostico_camara.json'
        path.write_text(json.dumps(info,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
        log('[DIAGNOSTICO] '+reason+'; resumen de topicos en '+str(path))
        for name,details in info.items():
            if name!='_diagnostico':log('[DIAGNOSTICO] '+name+': '+str(details['types'])+' pub='+str(details['publishers']))
        return path
    def odom_cb(m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        latest['pose']=(float(p.x),float(p.y),yaw);latest['pose_t']=time.monotonic()
    image_topics_discover()
    n.create_subscription(Odometry,'/odom',odom_cb,qos_profile_sensor_data)
    n.create_subscription(LaserScan,'/scan',lambda m:latest.__setitem__('scan_t',time.monotonic()),qos_profile_sensor_data)
    records=records_load(folder)
    planned=expected_rows()
    check(state.wait_for_service(timeout_sec=35),'/gazebo/set_entity_state operativo')
    def pump(seconds=.10):rclpy.spin_once(n,timeout_sec=seconds)
    try:
        t=time.monotonic()+75;last_discovery=0.;last_heartbeat=0.
        while time.monotonic()<t and not (latest['img'] is not None and latest['pose'] is not None and latest['scan_t']):
            now=time.monotonic()
            if now-last_discovery>1.2:
                image_topics_discover();last_discovery=now
            if now-last_heartbeat>12:
                log('[CAMARA] Esperando fotograma RGB; topicos Image='+str(list(image_subs))
                    +'; odom='+str(latest['pose'] is not None)+'; lidar='+str(latest['scan_t']>0))
                last_heartbeat=now
            pump(.15)
        if latest['img'] is None or latest['pose'] is None or latest['scan_t']<=0:
            camera_diagnostic('Sin fotogramas RGB o sin odometria/LiDAR luego de 75 s')
        check(latest['img'] is not None,'Camara RGB ROS publica fotogramas reales')
        log('[OK] Camara RGB activa desde '+str(latest['img_topic']))
        check(latest['pose'] is not None and latest['scan_t']>0,'Odometria y LiDAR activos')
        check(n.count_publishers('/cmd_vel')==0,'SIN publicadores /cmd_vel durante dataset')
        check(int(latest['img'].width)==MODEL_WIDTH and int(latest['img'].height)==MODEL_HEIGHT,
              f'Resolucion RGB temporal {MODEL_WIDTH}x{MODEL_HEIGHT}')
        log('=== R22 - ADQUISICION AUTOMATIZADA 600 IMAGENES, CAMARA CONTEXTUAL ===')
        log(f'[OPTICA] Horizontal FOV={CAMERA_HFOV_DEG:.1f} deg; '
            f'contexto visual exigido >= {(1-MAX_COLOR_FRACTION)*100:.1f}% en recorte central')
        log('[INFO] Es posible reanudar en un segundo intento con --reanudar; no se repiten capturas validas.')
        rec_path=folder/'registros.jsonl'
        with rec_path.open('a',encoding='utf8') as dst:
            accepted=0
            for j,row in enumerate(planned):
                old=records.get(row['id'])
                if old:
                    if verify_capture(row,old,folder/row['archivo']):
                        accepted+=1;continue
                    raise RuntimeError('Imagen previa alterada, no se sobrescribe: '+row['id'])
                station=STATIONS[int(row['clase_id'])]
                di=DISTS.index(row['distancia_m']);ai=YAW_OFFSETS.index(row['angulo_deg']);cap=int(row['captura'])-1
                # Reencuadre automatico para escenas casi uniformes: se mantiene
                # el control de calidad anterior, NO se acepta sin demostrarlo.
                candidate=None;selected=None;best=None
                for attempt in range(MAX_CAPTURE_ATTEMPTS):
                    x,y,yaw=pose_for_attempt(station,di,ai,cap,attempt)
                    req=SetEntityState.Request();req.state.name='waffle_pi';req.state.reference_frame='world'
                    req.state.pose.position.x=x;req.state.pose.position.y=y;req.state.pose.position.z=.05
                    req.state.pose.orientation.z=math.sin(yaw/2);req.state.pose.orientation.w=math.cos(yaw/2)
                    fut=state.call_async(req)
                    deadline=time.monotonic()+15
                    while not fut.done() and time.monotonic()<deadline:pump(.05)
                    if not fut.done():raise RuntimeError('Timeout teleport en '+row['id'])
                    if fut.exception():raise RuntimeError('Servicio teleport: '+str(fut.exception()))
                    ans=fut.result()
                    if not ans.success:raise RuntimeError('Teleport rechazado: '+str(ans.status_message))
                    after=time.monotonic();seen_seq=latest['image_seq']
                    # La muestra debe proceder de la pose NUEVA, no del buffer ROS.
                    wait_until=after+.18;deadline=after+7
                    good=False
                    while time.monotonic()<deadline:
                        pump(.065)
                        pose=latest['pose']
                        if pose is None:continue
                        yaw_delta=math.atan2(math.sin(pose[2]-yaw),math.cos(pose[2]-yaw))
                        if (latest['pose_t']>after and latest['img_t']>wait_until and
                            latest['scan_t']>after and latest['image_seq']>seen_seq and
                            math.dist(pose[:2],(x,y))<.12 and abs(yaw_delta)<.15):
                            good=True;break
                    if not good:
                        raise RuntimeError('Timeout imagen/pose/scan frescos o teleport inestable en '+row['id'])
                    img=img_to_rgb(latest['img'])
                    metrics=color_metric(img,row['clase'])
                    record_trial={'id':row['id'],'intento':attempt,'clase':row['clase'],
                                  'distancia_nominal_m':row['distancia_m'],
                                  'angulo_nominal_deg':row['angulo_deg'],
                                  'pose_x':round(x,5),'pose_y':round(y,5),
                                  'pose_yaw_deg':round(math.degrees(yaw),4),
                                  'aprobado':quality_ok(metrics),**metrics}
                    with (folder/'intentos_calidad.jsonl').open('a',encoding='utf8') as tf:
                        tf.write(json.dumps(record_trial,ensure_ascii=False)+'\n')
                    score=(MIN_COLOR_FRACTION<=metrics['cobertura_color']<=MAX_COLOR_FRACTION,
                           metrics['variacion_luminancia'],
                           -max(0.,metrics['cobertura_color']-MAX_COLOR_FRACTION))
                    if best is None or score>best[0]:
                        best=(score,img.copy(),metrics,attempt,x,y,yaw)
                    if quality_ok(metrics):
                        candidate=img;selected=(metrics,attempt,x,y,yaw)
                        if attempt:
                            log(f'[REENCADRE] {row["id"]}: aceptado intento {attempt+1}/'
                                f'{MAX_CAPTURE_ATTEMPTS}; variacion={metrics["variacion_luminancia"]:.3f}; '
                                f'color={metrics["cobertura_color"]:.3%}')
                        break
                    if attempt==0 or attempt%3==2:
                        log(f'[REINTENTO] {row["id"]}: intento {attempt+1}/'
                            f'{MAX_CAPTURE_ATTEMPTS}; variacion={metrics["variacion_luminancia"]:.3f}'
                            f' (min {MIN_LUMINANCE_STD}), color={metrics["cobertura_color"]:.3%}'
                            f' (max {MAX_COLOR_FRACTION:.1%})')
                if candidate is None:
                    diag=folder/'diagnosticos';diag.mkdir(exist_ok=True)
                    _,best_img,best_met,best_attempt,bx,by,byaw=best
                    dest=diag/(row['id']+'_rechazada.png')
                    dest.write_bytes(png_rgb(best_img.tobytes(),best_img.shape[1],best_img.shape[0]))
                    (diag/(row['id']+'_calidad.json')).write_text(
                        json.dumps({'id':row['id'],'n_intentos':MAX_CAPTURE_ATTEMPTS,
                                    'mejor_intento':best_attempt,'mejores_metricas':best_met,
                                    'pose':{'x':bx,'y':by,'yaw':byaw},
                                    'imagen':str(dest)},indent=2,ensure_ascii=False)+'\n',encoding='utf8')
                    raise RuntimeError(f'CALIDAD INSUFICIENTE TRAS {MAX_CAPTURE_ATTEMPTS} ENCUADRES '
                                       f'{row["id"]}; mejor={best_met}; ver {dest}. '
                                       'No se guardo como imagen valida.')
                img=candidate
                metrics,attempt,x,y,yaw=selected
                filename=folder/row['archivo'];filename.parent.mkdir(parents=True,exist_ok=True)
                content=png_rgb(img.tobytes(),img.shape[1],img.shape[0]);filename.write_bytes(content)
                record=dict(row,sha256=hashlib.sha256(content).hexdigest(),bytes=len(content),
                            **metrics,intento_adquisicion=attempt,pose_x=round(x,5),pose_y=round(y,5),
                            pose_yaw_deg=round(math.degrees(yaw),4),
                            stamp_sec=int(latest['img'].header.stamp.sec),
                            hfov_deg=CAMERA_HFOV_DEG,
                            offset_frontal_camara_m=CAMERA_FORWARD_OFFSET_M)
                dst.write(json.dumps(record,ensure_ascii=False)+'\n');dst.flush()
                accepted+=1
                if accepted%6==0:
                    log(f'[CAPTURA {accepted}/600] {row["clase"]} - d={row["distancia_m"]:.2f}m '
                        f'angulo={row["angulo_deg"]:+d} deg; color={metrics["cobertura_color"]:.3%}')
                if accepted%60==0:export_manifest(folder)
                if n.count_publishers('/cmd_vel')!=0:raise RuntimeError('Publicador inesperado /cmd_vel; ABORTAR')
        count=audit(folder,require_complete=True)
        check(count==600,'Dataset R22: 600/600 PNG autenticos, validos y separados 70/15/15')
        return 0
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

def latest_folder():
    if not ROOT.is_dir():return None
    items=sorted((p for p in ROOT.iterdir() if p.is_dir() and (p/'identidad.json').exists()),reverse=True)
    return items[0] if items else None

def cnn_python_install():
    python=CNN_ENV/'bin/python'
    if not python.is_file():
        log('[DEPENDENCIAS] Creando entorno aislado para TensorFlow CPU...')
        subprocess.run([sys.executable,'-m','venv',str(CNN_ENV)],check=True)
    test=subprocess.run([str(python),'-c','import tensorflow, PIL; print(tensorflow.__version__)'],
                        capture_output=True,text=True)
    if test.returncode!=0:
        log('[DEPENDENCIAS] Instalando tensorflow-cpu==2.15.1 y Pillow en ~/.venvs/amr-a2-cnn.')
        r=subprocess.run([str(python),'-m','pip','install','--disable-pip-version-check',
                          'tensorflow-cpu==2.15.1','Pillow==10.4.0'],check=False)
        if r.returncode:raise RuntimeError('No se pudo instalar TensorFlow: captura R22 conservada. '
                                           'Repetir --solo-entrenar tras resolver pip/red.')
    return python

def train(folder):
    check((folder/'resumen_dataset.json').is_file(),'Dataset R22 completo antes del entrenamiento')
    count=audit(folder,True)
    check(count==600,'Auditoria 600/600, SHA256 y particiones antes de CNN')
    py=cnn_python_install()
    log('[R23] Iniciando primer entrenamiento de CNN ligera sobre dataset 224x224...')
    cmd=[str(py),'-u',str(Path(__file__).resolve()),'--entrenamiento-interno','--out',str(folder)]
    p=subprocess.run(cmd,cwd=REPO,env={**os.environ,'TF_CPP_MIN_LOG_LEVEL':'2'},check=False)
    if p.returncode:raise RuntimeError('Entrenamiento CNN fallo. Dataset intacto; ejecutar --solo-entrenar.')
    check((folder/'entrenamiento/reporte_entrenamiento.json').is_file(),'Primer entrenamiento CNN con reporte y modelo')
    return 0

def run_training_child(folder):
    import numpy as np
    import tensorflow as tf
    from PIL import Image,ImageOps,ImageDraw
    tf.keras.utils.set_random_seed(SEED)
    tf.config.threading.set_intra_op_parallelism_threads(4)
    tf.config.threading.set_inter_op_parallelism_threads(2)
    records=list(records_load(folder).values())
    records.sort(key=lambda x:x['id'])
    paths={s:[] for s in ('train','val','test')};labels={s:[] for s in paths}
    for r in records:
        s=r['particion'];paths[s].append(str(folder/r['archivo']));labels[s].append(int(r['clase_id']))
    for s,expected in [('train',420),('val',90),('test',90)]:
        if len(paths[s])!=expected:raise RuntimeError(f'Split {s}: {len(paths[s])}/{expected}')
    out=folder/'entrenamiento';out.mkdir(exist_ok=True)
    def dataset(part,shuffle=False):
        ds=tf.data.Dataset.from_tensor_slices((paths[part],labels[part]))
        if shuffle:ds=ds.shuffle(len(paths[part]),seed=SEED,reshuffle_each_iteration=True)
        def read(path,label):
            data=tf.io.read_file(path);im=tf.io.decode_png(data,channels=3)
            im=tf.image.resize_with_pad(im,224,224)
            im=tf.cast(im,tf.float32)/255.0
            return im,tf.cast(label,tf.int32)
        return ds.map(read,num_parallel_calls=2).batch(16).prefetch(2)
    ds_train=dataset('train',True);ds_val=dataset('val');ds_test=dataset('test')
    # CNN de linea base, no proclamar que sustituye MobileNetV2 de entrega final.
    model=tf.keras.Sequential([
        tf.keras.layers.Input(shape=(224,224,3)),
        tf.keras.layers.RandomRotation(.025),
        tf.keras.layers.RandomContrast(.12),
        tf.keras.layers.Conv2D(16,3,activation='relu',padding='same'),
        tf.keras.layers.MaxPool2D(),
        tf.keras.layers.Conv2D(32,3,activation='relu',padding='same'),
        tf.keras.layers.MaxPool2D(),
        tf.keras.layers.Conv2D(64,3,activation='relu',padding='same'),
        tf.keras.layers.MaxPool2D(),
        tf.keras.layers.GlobalAveragePooling2D(),
        tf.keras.layers.Dense(64,activation='relu'),
        tf.keras.layers.Dropout(.25),
        tf.keras.layers.Dense(4,activation='softmax'),
    ],name='AMR_CNN_A2_primera_linea_base')
    model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=.001),
                  loss='sparse_categorical_crossentropy',metrics=['accuracy'])
    callbacks=[tf.keras.callbacks.EarlyStopping(monitor='val_loss',patience=3,
                                                 min_delta=.001,restore_best_weights=True)]
    history=model.fit(ds_train,validation_data=ds_val,epochs=8,verbose=2,callbacks=callbacks)
    values=model.evaluate(ds_test,verbose=0)
    prediction=model.predict(ds_test,verbose=0).argmax(axis=1)
    true=np.array(labels['test'],dtype=np.int32)
    matrix=np.zeros((4,4),dtype=np.int64)
    for a,b in zip(true,prediction):matrix[a,b]+=1
    for name,rows in [('historial.csv',[dict(epoca=i+1,**{k:float(v[i]) for k,v in history.history.items()})
                                          for i in range(len(history.history['loss']))])]:
        with (out/name).open('w',newline='',encoding='utf8') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    with (out/'matriz_confusion.csv').open('w',newline='',encoding='utf8') as f:
        w=csv.writer(f);w.writerow(['real\\predicho']+[x[1] for x in STATIONS])
        for i,st in enumerate(STATIONS):w.writerow([st[1]]+list(map(int,matrix[i])))
    model.save(out/'modelo_cnn_a2.keras')
    metrics={'estado':'R23_PRIMER_ENTRENAMIENTO_EJECUTADO','modelo':'CNN ligera propia (NO MobileNetV2 final)',
        'entradas':'RGB 224x224 mediante padding; originales PNG 640x360 conservados',
        'muestras':{'train':420,'val':90,'test':90},'epocas':len(history.history['loss']),
        'accuracy_test':float(values[1]),'loss_test':float(values[0]),
        'matriz_confusion':matrix.tolist(),'n_parametros':int(model.count_params()),
        'advertencia':'R23 primer entrenamiento; R24-R25 corresponden a entrega final; datos sinteticos Gazebo',
        'sha256_modelo':digest(out/'modelo_cnn_a2.keras')}
    (out/'reporte_entrenamiento.json').write_text(json.dumps(metrics,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    # Mosaico RGB de muestras test para auditoria humana: paneles legibles.
    mosaic=Image.new('RGB',(4*256,4*180),(245,245,245));draw=ImageDraw.Draw(mosaic)
    for i,st in enumerate(STATIONS):
        class_images=[p for p,l in zip(paths['test'],labels['test']) if l==i][:4]
        for j,p in enumerate(class_images):
            im=Image.open(p).convert('RGB');im.thumbnail((250,145))
            x=j*256+3;y=i*180+3
            mosaic.paste(im,(x,y));draw.text((x,y+148),f'{st[1]} | Test {j+1}',fill=(30,30,30))
    mosaic.save(out/'mosaico_control_calidad.png')
    hist=history.history
    model_info={'imagenes_dataset':600,'modelado':'preliminar','resultado_test_accuracy':metrics['accuracy_test']}
    chartdata={'train_accuracy':list(map(float,hist['accuracy'])),
               'val_accuracy':list(map(float,hist['val_accuracy'])),
               'train_loss':list(map(float,hist['loss'])),
               'val_loss':list(map(float,hist['val_loss']))}
    html='''<!doctype html><html lang="es"><meta charset="utf-8"><title>R22-R23 AMR CNN</title>
    <style>body{font:16px system-ui;margin:35px auto;max-width:1100px;line-height:1.5;color:#1a2535}
    table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:6px 12px}img{max-width:100%}
    .numbers{display:flex;gap:20px}.numbers strong{font-size:32px}</style>
    <h1>Dataset y primera CNN · AMR logístico</h1><p>R22: 600 imágenes RGB; R23: primer entrenamiento ejecutado.</p>
    <div class="numbers"><div><strong>600</strong><p>imágenes</p></div>
    <div><strong>420/90/90</strong><p>train / val / test</p></div>
    <div><strong>__ACC__%</strong><p>exactitud test</p></div></div>
    <p><b>Modelo:</b> CNN ligera de referencia (no MobileNetV2 final).</p>
    <p><b>Épocas:</b> __EPOCHS__. <b>Parámetros:</b> __PARAMS__.</p>
    <h2>Matriz de confusión (filas real, columnas predicción)</h2>__TABLE__
    <h2>Imágenes de evaluación, inspección visual</h2><img src="mosaico_control_calidad.png">
    <h2>Curvas de entrenamiento</h2><p>Disponibles en <code>historial.csv</code>. Valores:</p>
    <pre>__HISTORY__</pre>
    <p>Nota: evaluaciones sobre dataset sintético. La exactitud es preliminar;
    R24/R25 corresponden a la entrega final.</p></html>'''
    table='<table><tr><th>Real\\Pred</th>'+''.join(f'<th>{s[1]}</th>' for s in STATIONS)+'</tr>'
    for i,s in enumerate(STATIONS):table+='<tr><th>'+s[1]+'</th>'+''.join(f'<td>{int(x)}</td>' for x in matrix[i])+'</tr>'
    table+='</table>'
    html=(html.replace('__ACC__',f'{100*metrics["accuracy_test"]:.2f}')
              .replace('__EPOCHS__',str(metrics['epocas']))
              .replace('__PARAMS__',str(metrics['n_parametros']))
              .replace('__TABLE__',table)
              .replace('__HISTORY__',json.dumps(chartdata,indent=2,ensure_ascii=False)))
    (out/'reporte_visual.html').write_text(html,encoding='utf8')
    summary=json.loads((folder/'resumen_dataset.json').read_text(encoding='utf8'))
    summary['estado']='R22_R23_EJECUTADOS_PENDIENTE_AUDITORIA_DE_ENTREGA'
    summary['modelo_sha256']=metrics['sha256_modelo']
    summary['accuracy_test']=metrics['accuracy_test']
    (folder/'resumen_dataset.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    log(f'[R23] ENTRENAMIENTO COMPLETADO: test_accuracy={metrics["accuracy_test"]:.3%}; '
        f'epocas={metrics["epocas"]}; modelo={out/"modelo_cnn_a2.keras"}')
    log('[EVIDENCIA] '+str(out/'reporte_visual.html'))
    return 0

def run(args):
    if args.solo_entrenar:
        folder=latest_folder()
        if folder is None:raise RuntimeError('No hay dataset local para entrenar')
        return train(folder)
    preflight()
    e=env()
    check(subprocess.run(child(['--probe']),cwd=REPO,env=e,timeout=30).returncode==0,
          'Dominio ROS reservado')
    if args.reanudar and latest_folder() is not None:
        folder=latest_folder()
        log('[REANUDAR] Serie de camara contextual v6: '+str(folder))
    else:
        if args.reanudar:
            log('[NUEVA_SERIE_V6] No existe dataset con esta optica; se inicia uno nuevo.')
            log('[CONSERVACION] Dataset y diagnosticos v5 permanecen intactos: '+str(OLD_ROOT))
        ROOT.mkdir(parents=True,exist_ok=True)
        stamp=dt.datetime.now().strftime('%Y%m%d_%H%M%S')
        folder=ROOT/stamp
        folder.mkdir(parents=True,exist_ok=False)
        identity={'id':stamp,'fecha_local':dt.datetime.now().isoformat(timespec='seconds'),
                  'script_sha256':digest(Path(__file__).resolve()),
                  'base_commit':git('rev-parse','HEAD'),
                  'cam_pitch_deg':12,'cam_hfov_deg':CAMERA_HFOV_DEG,
                  'serie_optica':'v6_contexto', 'dominio_ros':int(DOMAIN)}
        (folder/'identidad.json').write_text(json.dumps(identity,indent=2)+'\n',encoding='utf8')
    temp_world=folder/'almacen_cnn_temporal.world'
    temp_model=folder/'waffle_pi_cnn_temporal.sdf'
    launch_py=folder/'almacen_cnn_temporal.launch.py'
    # Localizar modelo sin introducir cambios en sistema ni repositorio.
    p=subprocess.run(sourced(['ros2','pkg','prefix','turtlebot3_gazebo']),cwd=REPO,env=e,
                     capture_output=True,text=True,timeout=25)
    if p.returncode:raise RuntimeError('No se encontro paquete turtlebot3_gazebo: '+p.stderr)
    sdf=Path(p.stdout.strip())/'share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf'
    check(sdf.is_file(),'Modelo TurtleBot3 Waffle Pi disponible')
    original_sdf=sdf.read_text(encoding='utf8')
    camera_offset=camera_forward_offset(original_sdf)
    existing_optics=folder/'configuracion_optica.json'
    optics={'hfov_horizontal_deg':CAMERA_HFOV_DEG,
            'offset_frontal_camara_m':camera_offset,
            'cam_pitch_deg':12.0,
            'fuente_modelo_sha256':hashlib.sha256(original_sdf.encode()).hexdigest(),
            'advertencia':'Configuracion optica temporal de simulacion; no valida una camara fisica',
            'huella_teorica_a_0_30_m':list(context_geometry(.30)),
            'campo_horizontal_previo':'Valor original SDF archivado en el modelo instalado, sin modificarlo'}
    if existing_optics.exists():
        old=json.loads(existing_optics.read_text(encoding='utf8'))
        if old!=optics:raise RuntimeError('No se mezclan imagenes con distintas opticas SDF; crear serie nueva')
    else:
        existing_optics.write_text(json.dumps(optics,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    temp_world.write_text(world_with_state(BASE_WORLD.read_text(encoding='utf8')),encoding='utf8')
    temp_model.write_text(make_model(original_sdf),encoding='utf8')
    check(True,f'Calibracion optica: FOV horizontal {CAMERA_HFOV_DEG:.0f} deg, '
               f'offset longitudinal camara {camera_offset:+.3f}m')
    launch_py.write_text(LAUNCH,encoding='utf8')
    check(True,'Mundo y modelo temporales; archivos originales intactos')
    session=folder/'sesiones'/dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    session.mkdir(parents=True,exist_ok=False)
    handles=[(session/name).open('w',encoding='utf8',buffering=1) for name in
             ('gzserver.log','factory.log','spawn.log','captura.log','gzclient.log')]
    procs=[None]*5
    try:
        procs[0]=subprocess.Popen(sourced(['gzserver','--verbose','-s','libgazebo_ros_init.so',
                                      '-s','libgazebo_ros_factory.so',str(temp_world)]),
                         cwd=REPO,env=e,stdout=handles[0],stderr=subprocess.STDOUT,start_new_session=True)
        log('[INFO] Gazebo iniciado; verificando Factory...')
        procs[1]=subprocess.Popen(child(['--factory-probe']),cwd=REPO,env=e,
                         stdout=handles[1],stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+65
        while procs[1].poll() is None:
            if procs[0].poll() is not None or time.monotonic()>deadline:raise RuntimeError('Gazebo Factory no disponible')
            time.sleep(.25)
        check(procs[1].returncode==0,'Factory verificada antes de spawn')
        procs[2]=subprocess.Popen(sourced(['ros2','launch',str(launch_py)]),cwd=REPO,env=e,
                         stdout=handles[2],stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+75
        while time.monotonic()<deadline:
            # Primero revisar evidencia de spawn; launch puede salir con 0 en el
            # mismo instante en que spawn_entity.py reporta el exito.
            if 'Successfully spawned entity [waffle_pi]' in (session/'spawn.log').read_text(encoding='utf8',errors='replace'):
                break
            runtime_check(procs[0].poll(),procs[2].poll())
            if procs[2].poll() == 0:
                # El proceso termino sin imprimir exito: no confundir con un
                # spawn confirmado, a pesar de su returncode 0.
                raise RuntimeError('Launch finalizo sin confirmacion de spawn; revisar spawn.log')
            time.sleep(.25)
        else:raise RuntimeError('Timeout insertando robot en Gazebo')
        check(True,'TurtleBot3 insertado con camara RGB simulada')
        # El lanzador contiene UNICAMENTE spawn_entity.py: su salida normal
        # no es un fallo. gzserver sigue siendo el proceso del simulador.
        launch_rc=procs[2].poll()
        status=runtime_check(procs[0].poll(),launch_rc)
        if status=='spawn_launch_terminado_correctamente':
            log('[OK] ros2 launch spawn-only finalizo con codigo 0; gzserver sigue activo')
        else:
            log('[INFO] ros2 launch spawn-only continua vivo; se vigilara gzserver')
        if args.visual:
            check(bool(e.get('DISPLAY') or e.get('WAYLAND_DISPLAY')),
                  'Sesion grafica DISPLAY/WAYLAND disponible para gzclient')
            log('[VISUAL] Iniciando gzclient AHORA, antes de esperar /camera/image_raw.')
            procs[4]=subprocess.Popen(sourced(['gzclient']),cwd=REPO,env=e,
                         stdout=handles[4],stderr=subprocess.STDOUT,start_new_session=True)
            time.sleep(1.5)
            if procs[4].poll() is not None:
                log(f'[AVISO] gzclient termino prematuramente, codigo {procs[4].returncode}. '
                    'Revisar gzclient.log; la captura continuara si el servidor funciona.')
                log_tail(session/'gzclient.log','gzclient.log',limit=16)
            else:
                log('[VISUAL] Proceso gzclient activo. La carga grafica puede tardar; '
                    'la adquisicion no depende de su splash screen.')
        procs[3]=subprocess.Popen(child(['--captura-interna','--out',str(folder)]),cwd=REPO,env=e,
                         stdout=handles[3],stderr=subprocess.STDOUT,start_new_session=True)
        cursor=0;next_print=time.monotonic();gui_closed_reported=False
        while procs[3].poll() is None:
            # El ros2 launch de spawn_entity.py PUEDE terminar normalmente.
            # Solamente un cierre real de gzserver o salida anormal del
            # lanzador debe abortar esta adquisicion.
            runtime_check(procs[0].poll(),procs[2].poll())
            if procs[4] is not None and procs[4].poll() is not None and not gui_closed_reported:
                log(f'[AVISO] gzclient salio (codigo {procs[4].returncode}); adquisicion sigue en servidor headless')
                gui_closed_reported=True
            if time.monotonic()>=next_print:
                lines=(session/'captura.log').read_text(encoding='utf8',errors='replace').splitlines()
                for line in lines[cursor:]:
                    if line.startswith(('[OK]','[CAPTURA','[ERROR','[INFO]','===')):log(line)
                cursor=len(lines);next_print=time.monotonic()+3
            time.sleep(.35)
        lines=(session/'captura.log').read_text(encoding='utf8',errors='replace').splitlines()
        for line in lines[cursor:]:log(line)
        if procs[3].returncode:raise RuntimeError('Captura incompleta: revisar '+str(session/'captura.log'))
        check(audit(folder,True)==600,'600 imagenes y manifiesto auditados')
        if args.solo_captura:
            log('=== R22: CAPTURA COMPLETA; R23 ENTRENAMIENTO PENDIENTE ===');return 0
    except Exception:
        log('[EVIDENCIA] Diagnosticos guardados en '+str(session))
        for name in ('captura.log','gzserver.log','spawn.log','gzclient.log'):
            log_tail(session/name,name,limit=12)
        raise
    finally:
        for p in (procs[3],procs[4],procs[2],procs[1],procs[0]):kill_own(p)
        for h in handles:h.close()
        log('[OK] Procesos exclusivos de adquisicion cerrados, repo intacto')
    return train(folder)

def self_test():
    import numpy as np
    rows=expected_rows()
    assert len(rows)==600
    assert len({r['id'] for r in rows})==600
    from collections import Counter,defaultdict
    counts=Counter(r['particion'] for r in rows)
    assert counts=={'train':420,'val':90,'test':90}
    groups=defaultdict(set)
    classes=Counter(r['clase'] for r in rows)
    for r in rows:groups[r['grupo']].add(r['particion'])
    assert len(groups)==100 and all(len(v)==1 for v in groups.values())
    assert len(classes)==4 and all(v==150 for v in classes.values())
    for i,st in enumerate(STATIONS):
        x,y,yaw=station_pose(st,0,2)
        assert -4.925<x<4.925 and abs(y)<3.925
        assert abs(math.atan2(math.sin(yaw),math.cos(yaw)))<1e-6 if st[4]<0 else abs(abs(yaw)-math.pi)<1e-6
    # Test de regresion: un launch spawn-only que sale 0 NO mata a Gazebo.
    assert runtime_check(None,None)=='spawn_launch_activo'
    assert runtime_check(None,0)=='spawn_launch_terminado_correctamente'
    for srv,launch in ((1,None),(-9,0),(None,1),(None,-11)):
        try:runtime_check(srv,launch)
        except RuntimeError:pass
        else:raise AssertionError(f'No detecto fallo real: gzserver={srv}, launch={launch}')
    image=np.zeros((224,224,3),dtype=np.uint8)
    for name,color in [('RECEPCION',(10,10,230)),('ALMACENAMIENTO',(220,12,12)),
                       ('EMPAQUE',(15,230,15)),('DESPACHO',(200,195,20))]:
        image[:]=color
        met=color_metric(image,name)
        assert met['cobertura_color']>.95,(name,met)
    image[:]=(100,100,100)
    assert color_metric(image,'RECEPCION')['cobertura_color']==0
    p=png_rgb(image.tobytes(),224,224)
    assert p[:8]==b'\x89PNG\r\n\x1a\n' and len(p)>100
    root='<sdf version="1.6"><model name="tb"><link name="cam"><sensor name="camera" type="camera"><camera><image><width>1920</width><height>1080</height></image></camera></sensor></link></model></sdf>'
    assert camera_forward_offset(root)==0
    parsed=ET.fromstring(make_model(root))
    assert abs(float(parsed.find('.//camera/horizontal_fov').text)-math.radians(CAMERA_HFOV_DEG))<1e-7
    width,height=context_geometry(.30)
    assert width>1.7 and height>0.9, (width,height)
    assert MAX_COLOR_FRACTION<1 and MIN_LUMINANCE_STD==6.0
    assert parsed.find('.//image/width').text=='640'
    assert '-0.2094' in ET.tostring(parsed,encoding='unicode')
    assert parsed.find('.//sensor/always_on').text=='true'
    assert parsed.find('.//sensor/visualize').text=='false'
    log('=== R22/R23: AUTOPRUEBAS LOCALES APROBADAS ===')
    log('[OK] 600 IDs, 4 clases de 150, 100 grupos sin fuga, split 420/90/90')
    log('[OK] 4 mascaras cromaticas, encoder PNG y modelo temporal de camara')
    log('[OK] Launch spawn-only finalizado normalmente NO se interpreta como caida de Gazebo')
    log('[OK] Camara activa, topicos Image dinamicos y GUI inmediata opcional')
    assert MAX_CAPTURE_ATTEMPTS==12
    assert quality_ok({'variacion_luminancia':6.2,'cobertura_color':.2})
    assert not quality_ok({'variacion_luminancia':9.0,'cobertura_color':.998143})
    assert quality_ok({'variacion_luminancia':9.0,'cobertura_color':.85})
    assert not quality_ok({'variacion_luminancia':5.972,'cobertura_color':.996373})
    assert not quality_ok({'variacion_luminancia':100,'cobertura_color':.0001})
    for st in STATIONS:
        for di in range(5):
            for ai in range(5):
                poses=[pose_for_attempt(st,di,ai,5,k) for k in range(MAX_CAPTURE_ATTEMPTS)]
                assert len(set(poses))==MAX_CAPTURE_ATTEMPTS
                p0=poses[0]
                assert p0==station_pose(st,di,ai,5)
                assert all(abs(p[0]-p0[0])<=.02 and abs(p[1]-p0[1])<=.079
                           and abs(math.atan2(math.sin(p[2]-p0[2]),math.cos(p[2]-p0[2])))
                               <= math.radians(7.6) for p in poses)
    log('[OK] v6: camara FOV 145 deg, contexto no saturado, offset SDF calibrado')
    log('[OK] v5: 12 reencuadres automaticos, umbral de luminancia conservado')
    return 0

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    opts=parser.add_mutually_exclusive_group()
    opts.add_argument('--solo-captura',action='store_true')
    opts.add_argument('--solo-entrenar',action='store_true')
    parser.add_argument('--reanudar',action='store_true',help='Reanuda imagenes existentes SHA256')
    parser.add_argument('--visual',action='store_true',help='Abre Gazebo gzclient (requiere WSLg)')
    parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--probe',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--factory-probe',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--captura-interna',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--entrenamiento-interno',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--out',type=Path,help=argparse.SUPPRESS)
    args=parser.parse_args()
    try:
        if args.self_test:return self_test()
        if args.probe:return probe()
        if args.factory_probe:return factory_probe()
        if args.captura_interna:
            if not args.out or os.environ.get('ROS_DOMAIN_ID')!=DOMAIN:raise RuntimeError('Captura interna sin contexto')
            return capture(args.out)
        if args.entrenamiento_interno:
            if not args.out:return 2
            return run_training_child(args.out)
        if args.reanudar and args.solo_entrenar:raise RuntimeError('--reanudar no se combina con --solo-entrenar')
        self_test();return run(args)
    except KeyboardInterrupt:
        log('[INTERRUMPIDO] Datos guardados hasta ultimo PNG; reanudar con --reanudar.');return 130
    except Exception as exc:
        log('[ERROR R22/R23] '+str(exc));return 2

if __name__=='__main__':sys.exit(main())
