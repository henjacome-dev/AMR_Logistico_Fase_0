#!/usr/bin/env python3
"""AMR Logistico / E2.3.6A: canal operativo /cmd_vel en cero + arbitraje Gazebo en sombra.

Windows Downloads -> Ubuntu WSL:
 python3 /mnt/c/Users/Henry/Downloads/A2_E236A_Gazebo_canal_cmd_vel_cero_v5_headless.py

Primero arranca gzserver HEADLESS con ROS Init/Factory explicitos y confirma
/spawn_entity ANTES de cargar el robot. No abre interfaz grafica Gazebo.
Un unico portero de salida publica Twist() CERO a 10 Hz en /cmd_vel para probar
el canal ROS real de Gazebo, NUNCA transmite propuestas no nulas.
El arbitraje se mantiene en /cmd_vel_integrado_prueba y /estado_integrado_prueba.
Una propuesta de ruta CERO se publica continuamente en /cmd_vel_ruta.
No lanza seguidor de rutas ni supervisor F1.3; no hay navegacion autonoma. Dos cajas estaticas de ensayo se crean mediante el servicio
SpawnEntity en Gazebo y se eliminan por DeleteEntity durante la observacion.
Todo el ensayo sucede en ROS_DOMAIN_ID=224 y ROS_LOCALHOST_ONLY=1.
No usa gzclient; comprueba /spawn_entity antes de insertar el robot.
Los umbrales de conmutacion .60/.80 m NO se acreditan como oficiales R19.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

REPO = Path.home() / 'AMR_Logistico_Fase_0'
VENV = Path.home() / '.venvs' / 'amr-a2-r18' / 'bin' / 'python'
ACTIVATE = REPO / 'tools' / 'activar_entorno.sh'
EXPECTED = '8c893f13710713be5ea35ef226952bf10a91d6d9'
DOMAIN = '224'
START_X = -3.0
START_Y = -2.8
DISCOVER_S = 135.0
STAGE_S = 22.0
FACTORY_WAIT_S = 40.0
WORLD = REPO / 'modelos/almacen_f1.world'
TRACKED = [
    'matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis',
    'tools/supervisor_f13.py',
    'launch/almacen_f1.launch.py',
    'launch/almacen_operativo_f1.launch.py',
    'tools/emulador_sensores_f1.py',
    'tools/A2_E22_nodo_difuso.py',
    'tools/A2_E23_arbitraje_sombra.py',
    'tools/A2_E233_supervisor_integrado_aislado.py',
]

LAUNCH = '''#!/usr/bin/env python3
"""E2.3.6A: robot/nodos en servidor Gazebo ROS Factory headless prevalidado."""
import sys
import subprocess
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    root = Path.home() / 'AMR_Logistico_Fase_0'
    py = Path.home() / '.venvs/amr-a2-r18/bin/python'
    modelo = Path(get_package_share_directory('turtlebot3_gazebo')) / 'models/turtlebot3_waffle_pi/model.sdf'
    urdf = Path(get_package_share_directory('turtlebot3_description')) / 'urdf/turtlebot3_waffle_pi.urdf'
    robot_description = subprocess.check_output(['xacro',str(urdf)],text=True)
    pub = Node(package='robot_state_publisher',executable='robot_state_publisher',
        name='robot_state_publisher',output='screen',
        parameters=[{'robot_description':robot_description,'use_sim_time':True}])
    robot = Node(package='gazebo_ros',executable='spawn_entity.py',output='screen',
        arguments=['-entity','waffle_pi','-file',str(modelo),
                   '-x','-3.0','-y','-2.8','-z','0.05','-Y','0.0'])
    sensor = ExecuteProcess(cmd=[sys.executable,'-u',str(root/'tools/emulador_sensores_f1.py'),
        '--ros-args','-p','use_sim_time:=true'],cwd=str(root),output='screen')
    fuzzy = ExecuteProcess(cmd=[str(py),'-u',str(root/'tools/A2_E22_nodo_difuso.py'),'--ros',
        '--ros-args','-p','use_sim_time:=true'],cwd=str(root),output='screen')
    candidate = ExecuteProcess(cmd=[str(py),'-u',str(root/'tools/A2_E233_supervisor_integrado_aislado.py'),
        '--nodo-aislado'],cwd=str(root),output='screen',additional_env={'_A2_E233_CHILD':'1'})
    return LaunchDescription([pub,robot,sensor,fuzzy,candidate])
'''


def say(txt):
    print(txt,flush=True)


def require(cond,txt):
    if not cond: raise RuntimeError(txt)
    say('[OK] '+txt)


def git(*args):
    r=subprocess.run(['git',*args],cwd=REPO,text=True,capture_output=True,timeout=22)
    if r.returncode: raise RuntimeError('Git: '+(r.stderr or r.stdout).strip())
    return r.stdout.strip()


def preflight():
    say('=== E2.3.6A - PREFLIGHT GAZEBO HEADLESS CON RUTA CERO ===')
    require((REPO/'.git').is_dir(),'Repositorio presente')
    require(git('branch','--show-current')=='desarrollo-avance2','Rama de desarrollo correcta')
    require(git('rev-parse','HEAD')==EXPECTED,'Commit E2.3 publicado presente')
    for rel in TRACKED:
        require((REPO/rel).is_file() and git('hash-object','--',rel)==git('rev-parse','HEAD:'+rel),
                'Sin modificaciones: '+rel)
    require(VENV.is_file() and ACTIVATE.is_file(),'Entorno ROS 2 y Python disponibles')
    code=(REPO/'tools/A2_E233_supervisor_integrado_aislado.py').read_text(encoding='utf-8')
    require("OUTPUT = '/cmd_vel_integrado_prueba'" in code,
            'Supervisor candidato limitado a /cmd_vel_integrado_prueba')
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            if (p/'comm').read_text(encoding='utf-8').strip() in ('gzserver','gzclient'):
                raise RuntimeError('Ya existe Gazebo: PID '+p.name+'. No iniciar segundo Gazebo.')
        except (FileNotFoundError,PermissionError,UnicodeError):pass
    say('[OK] No hay gzserver/gzclient anteriores')
    say('[PENDIENTE] R19 operativo y umbrales oficiales siguen sin acreditarse.')


def env():
    e=os.environ.copy();e['ROS_DOMAIN_ID']=DOMAIN;e['ROS_LOCALHOST_ONLY']='1';return e


def selfcmd(extra):
    return ['bash','-c','source "$1" >/dev/null && exec "$2" "$3" "${@:4}"',
            'bash',str(ACTIVATE),str(VENV),str(Path(__file__).resolve()),*extra]


def probe():
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node=Node('a2_e236a_preflight',enable_rosout=False)
    try:
        t=time.monotonic()+2.7
        while time.monotonic()<t:rclpy.spin_once(node,timeout_sec=.12)
        other=[x for x,_ in node.get_topic_names_and_types() if x not in ('/rosout','/parameter_events')]
        require(not other and node.count_publishers('/cmd_vel')==0,
                'Dominio 224 libre, sin publicaciones /cmd_vel')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def factory_probe():
    """Check ROS Factory service before launching TurtleBot3, without calling it."""
    import rclpy
    from rclpy.node import Node
    from gazebo_msgs.srv import SpawnEntity
    require(os.environ.get('ROS_DOMAIN_ID')==DOMAIN and
            os.environ.get('ROS_LOCALHOST_ONLY')=='1',
            'Comprobacion Factory en dominio ROS aislado')
    rclpy.init()
    node=Node('a2_e236a_factory_pre_spawn',enable_rosout=False)
    try:
        until=time.monotonic()+FACTORY_WAIT_S
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.2)
            services=dict(node.get_service_names_and_types())
            for name in ('/spawn_entity','/gazebo/spawn_entity'):
                if (name in services and
                        'gazebo_msgs/srv/SpawnEntity' in services[name]):
                    say('[OK] Factory disponible antes de insertar el robot: '+name)
                    return 0
        say('[ERROR] Servicio /spawn_entity ausente tras el tiempo de comprobacion.')
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def box_xml(name):
    # Caja fija y conspicua de 0.18 x 0.20 x 0.40 m. No obstruye por si sola
    # una pared antes de que la detecte el lidar; no se guarda en el mundo.
    return f'''<?xml version="1.0"?>
<sdf version="1.6"><model name="{name}"><static>true</static>
<link name="box"><collision name="collision"><geometry><box><size>0.18 0.20 0.40</size></box></geometry></collision>
<visual name="visual"><geometry><box><size>0.18 0.20 0.40</size></box></geometry>
<material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material></visual></link>
</model></sdf>'''


def monitor(out):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, Range
    from std_msgs.msg import String
    from gazebo_msgs.srv import SpawnEntity, DeleteEntity

    require(os.environ.get('ROS_DOMAIN_ID')==DOMAIN and
            os.environ.get('ROS_LOCALHOST_ONLY')=='1',
            'Sonda en dominio 224 aislado')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Node('a2_e236a_monitoreo_sombra')
    counts={k:0 for k in ('scan','odom','df','di','dd','fuzzy','fuzzy_state','candidate','candidate_state','cmd_vel')}
    states=[];events=[];stage_log=[];created=[]
    last={'df':None,'scan':None,'odom':None,'state':None,'candidate':None,'fuzzy':None,
          'state_at':0.,'candidate_at':0.}
    bad_twist=False
    cmd_vel_nonzero=False
    odom_initial=None
    started=time.monotonic()

    def record(k,v):
        counts[k]+=1
        if len(events)<20000:
            events.append({'seg':round(time.monotonic()-started,3),'topico':k,'valor':str(v)[:120]})

    def scan_cb(m):
        values=[]
        for i,r in enumerate(m.ranges):
            angle=math.atan2(math.sin(m.angle_min+i*m.angle_increment),
                             math.cos(m.angle_min+i*m.angle_increment))
            if abs(angle)>math.radians(25):continue
            if math.isfinite(r) and m.range_min<=r<=m.range_max:values.append(r)
        last['scan']=min(values) if values else None
        record('scan',round(last['scan'],3) if last['scan'] is not None else 'sin_rayos')

    def range_cb(k,m):
        if k=='df':last['df']=float(m.range)
        record(k,round(m.range,4))

    def twist_cb(k,m):
        nonlocal bad_twist
        vals=[m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z]
        if not all(math.isfinite(float(v)) for v in vals):bad_twist=True
        last[k]=(float(m.linear.x),float(m.angular.z))
        if k=='candidate':last['candidate_at']=time.monotonic()
        record(k,'v=%.5f w=%+.5f'%(m.linear.x,m.angular.z))

    def state_cb(k,m):
        if k=='candidate_state':
            last['state']=m.data
            last['state_at']=time.monotonic()
            if not states or states[-1]!=m.data:states.append(m.data)
        record(k,m.data)

    node.create_subscription(LaserScan,'/scan',scan_cb,qos_profile_sensor_data)
    def odom_cb(m):
        nonlocal odom_initial
        xyz=(float(m.pose.pose.position.x),float(m.pose.pose.position.y))
        if odom_initial is None and all(math.isfinite(z) for z in xyz):
            odom_initial=xyz
        last['odom']=xyz
        record('odom',tuple(round(z,3) for z in xyz))
    node.create_subscription(Odometry,'/odom',odom_cb,qos_profile_sensor_data)
    for k,topic in [('df','/rango_frontal'),('di','/rango_izq'),('dd','/rango_der')]:
        node.create_subscription(Range,topic,lambda m,k=k:range_cb(k,m),10)
    node.create_subscription(Twist,'/cmd_vel_difuso',lambda m:twist_cb('fuzzy',m),10)
    node.create_subscription(String,'/estado_difuso',lambda m:state_cb('fuzzy_state',m),10)
    node.create_subscription(Twist,'/cmd_vel_integrado_prueba',lambda m:twist_cb('candidate',m),10)
    node.create_subscription(String,'/estado_integrado_prueba',lambda m:state_cb('candidate_state',m),10)

    route_pub=node.create_publisher(Twist,'/cmd_vel_ruta',10)
    # Barrera operativa E2.3.6A: unico publicador de /cmd_vel, SIEMPRE cero.
    # La propuesta no nula de DIFUSO se limita al topico de prueba.
    # La misma pasarela observa la exclusividad del publicador y aborta si hay otro.
    zero_pub=node.create_publisher(Twist,'/cmd_vel',10)
    def cmd_vel_cb(m):
        nonlocal cmd_vel_nonzero
        components=(m.linear.x,m.linear.y,m.linear.z,
                    m.angular.x,m.angular.y,m.angular.z)
        if any(not math.isfinite(v) or abs(v)>1e-9 for v in components):
            cmd_vel_nonzero=True
        record('cmd_vel', 'CERO' if not cmd_vel_nonzero else 'NO_CERO')
    node.create_subscription(Twist,'/cmd_vel',cmd_vel_cb,10)
    # La ruta propuesta es siempre Twist() de cero.

    route_count=0
    zero_count=0
    zero_first_s=None
    zero_last_s=None
    spawn_client=None;delete_client=None
    last_progress=0

    def publicar_ceros():
        # Frecuencia limitada a 10 Hz mediante temporizador ROS
        # (este nodo no emplea use_sim_time).
        # CRÍTICO: nunca publicar desde pump(), porque la suscripción al mismo
        # /cmd_vel despierta spin_once() inmediatamente y causa un bucle sin límite.
        nonlocal route_count,zero_count,zero_first_s,zero_last_s
        ahora = time.monotonic()
        zero_pub.publish(Twist())
        route_pub.publish(Twist())
        if zero_first_s is None:
            zero_first_s=ahora
        zero_last_s=ahora
        zero_count+=1
        route_count+=1

    timer_ceros=node.create_timer(.10,publicar_ceros)

    def pump():
        nonlocal last_progress
        if node.count_publishers('/cmd_vel')!=1:
            raise RuntimeError('SEGURIDAD: /cmd_vel no tiene exactamente un publicador.')
        if cmd_vel_nonzero:
            raise RuntimeError('SEGURIDAD: se recibio movimiento NO CERO en /cmd_vel.')
        if bad_twist:raise RuntimeError('Se observó un Twist no finito')
        rclpy.spin_once(node,timeout_sec=.06)
        # Se cede tiempo al supervisor candidato, Gazebo y los demás nodos.
        time.sleep(.005)
        if time.monotonic()-last_progress>18:
            say('[INFO] Observando Gazebo/sensores: '+', '.join(k+'='+str(v) for k,v in counts.items()))
            last_progress=time.monotonic()

    def wait_for_mode(label,expected,timeout=STAGE_S,reason_prefix=None):
        begin=time.monotonic()
        while time.monotonic()-begin<timeout:
            pump()
            s=last['state'] or ''
            if not (s.startswith(expected+'|') and
                    last['state_at']>=begin and last['candidate_at']>=begin):
                continue
            if reason_prefix is not None and not s.split('|',1)[1].startswith(reason_prefix):
                continue
            if last['candidate'] is None:continue
            v,w=last['candidate']
            if expected in ('RUTA','DETENIDO'):
                valid=abs(v)<1e-6 and abs(w)<1e-6
            else:
                # El candidato publica velocidades NO NULAS solo en el topico
                # de PRUEBA. No hay orden de marcha en /cmd_vel.
                valid=(0.0<v<=.120001 and abs(w)<=.700001)
            if valid:
                stage_log.append({'etapa':label,'estado':s,'scan_m':last['scan'],
                    'range_m':last['df'],'v':v,'omega':w})
                say(f'[OK] {label}: {s}; v={v:.4f}, w={w:+.4f}; scan={last["scan"]}; Range={last["df"]}')
                return
        raise RuntimeError(f'No se observo {expected} en {label}; ultimo={last["state"]}, '
            f'scan={last["scan"]}, Range={last["df"]}. Revisar geometria del obstaculo.')

    def wait_client(kind):
        cls=SpawnEntity if kind=='spawn' else DeleteEntity
        possibilities=['/spawn_entity','/gazebo/spawn_entity'] if kind=='spawn' else ['/delete_entity','/gazebo/delete_entity']
        expiry=time.monotonic()+25
        while time.monotonic()<expiry:
            available={name for name,_ in node.get_service_names_and_types()}
            for name in possibilities:
                if name in available:
                    return node.create_client(cls,name)
            pump()
        raise RuntimeError('Servicio Gazebo no disponible: '+kind)

    def ros_future(client,request,what):
        future=client.call_async(request)
        limit=time.monotonic()+18
        while not future.done() and time.monotonic()<limit:pump()
        if not future.done():raise RuntimeError('Tiempo agotado al '+what)
        result=future.result()
        if result is None or not result.success:
            raise RuntimeError(what+': '+str(getattr(result,'status_message','sin respuesta')))
        say('[OK] Servicio Gazebo: '+what)

    def spawn(name,offset):
        req=SpawnEntity.Request();req.name=name;req.xml=box_xml(name)
        req.robot_namespace='';req.reference_frame='world'
        req.initial_pose.position.x=START_X+offset
        req.initial_pose.position.y=START_Y
        req.initial_pose.position.z=0.30
        req.initial_pose.orientation.w=1.0
        ros_future(spawn_client,req,'spawn '+name)
        created.append(name)

    def delete(name):
        req=DeleteEntity.Request();req.name=name
        ros_future(delete_client,req,'delete '+name)
        created.remove(name)

    try:
        say('=== E2.3.6A - GAZEBO: RUTA -> DIFUSO -> DETENIDO -> RUTA (SOMBRA) ===')
        say('[SEGURIDAD] /cmd_vel tiene un solo publicador, restringido a Twist() cero.')
        say('[SEGURIDAD] Las propuestas del candidato NO se reenvian al robot.')
        # El servidor Factory fue comprobado previamente. Sin /scan y /odom no
        # hay robot; se evita esperar el plazo largo sin signos de insercion.
        until=time.monotonic()+DISCOVER_S
        while time.monotonic()<until and not all(counts.values()):pump()
        require(all(counts.values()),'10 topicos activos con Gazebo, incluido /cmd_vel')
        require(last['odom'] is not None,'Odometria presente')
        require(last['scan'] is not None and last['scan']>=0.85,
                'Entorno inicial libre por LiDAR (>=0.85 m)')
        wait_for_mode('Pasillo libre, ruta cero','RUTA')
        spawn_client=wait_client('spawn')
        delete_client=wait_client('delete')
        spawn('a2_e236a_caja_media',0.56)
        wait_for_mode('Obstaculo medio: difuso','DIFUSO')
        spawn('a2_e236a_caja_cerca',0.34)
        wait_for_mode('Obstaculo cercano: parada','DETENIDO',reason_prefix='PARADA_')
        delete('a2_e236a_caja_cerca')
        wait_for_mode('Obstaculo medio: recupera difuso','DIFUSO')
        delete('a2_e236a_caja_media')
        wait_for_mode('Pasillo despejado: retorna ruta','RUTA')
        require(node.count_publishers('/cmd_vel')==1,
                'Exactamente un publicador autorizado de /cmd_vel')
        # Las cinco transiciones suelen completar en pocos segundos. A 10 Hz,
        # no pueden contabilizarse 100 mensajes tan pronto. Observar sin
        # modificar el temporizador ni rebajar el umbral de aceptación.
        say('[INFO] Cinco transiciones correctas; completando observacion de /cmd_vel a 10 Hz...')
        limite_ceros=time.monotonic()+22.0
        while time.monotonic()<limite_ceros and not (
                counts['cmd_vel']>=110 and zero_count>=110):
            pump()
        duracion_ceros=(zero_last_s-zero_first_s
                        if zero_first_s is not None and zero_last_s is not None
                        else 0.0)
        frecuencia_ceros=((zero_count-1)/duracion_ceros
                          if zero_count>1 and duracion_ceros>0 else 0.0)
        say(f'[MEDICION] /cmd_vel enviados={zero_count}; '
            f'recibidos={counts["cmd_vel"]}; frecuencia={frecuencia_ceros:.2f} Hz; '
            f'lapso={duracion_ceros:.2f} s; no_cero={cmd_vel_nonzero}')
        require(100<counts['cmd_vel']<=zero_count+10 and not cmd_vel_nonzero,
                'Mas de 100 mensajes /cmd_vel recibidos, todos cero y sin duplicacion')
        require(8.0<=frecuencia_ceros<=12.0,
                'Frecuencia de emision /cmd_vel medida dentro de 8-12 Hz (nominal 10 Hz)')
        require(zero_count>=110 and route_count==zero_count,
                'Temporizador 10 Hz: ruta y /cmd_vel recibieron los mismos envios cero')
        require(odom_initial is not None and last['odom'] is not None and
                math.dist(odom_initial,last['odom'])<.05,
                'Robot sin desplazamiento apreciable (menos de 5 cm en odom)')
        require(all(abs(x)<1e-6 for x in last['candidate']),'Propuesta final es cero')
        require(route_count>100,'Ruta cero publicada de forma continua')
        # Nota: estado RUTA/DIFUSO se ha observado con la propuesta de ruta cero;
        # no demuestra seguimiento de trayectoria ni comportamiento bajo movimiento.
        status='APROBADO_CANAL_CERO'
        return_code=0
        say('=== E2.3.6A: UNICO /cmd_vel CERO Y ARBITRAJE SOMBRA APROBADOS; R19 PENDIENTE ===')
    except Exception as e:
        status='NO_APROBADA';return_code=2
        say('[ERROR E2.3.6A] '+str(e))
    finally:
        # Limpiar las entidades propias si Gazebo continúa disponible.
        if delete_client is not None:
            for name in list(reversed(created)):
                try:delete(name)
                except Exception as exc:say('[WARN] No se pudo eliminar '+name+': '+str(exc))
        out.mkdir(parents=True,exist_ok=True)
        payload={'etapa':'E2.3.6A','estado':status,'domain_id':DOMAIN,
          'umbral_ruta_difuso_m':[0.60,0.80],'umbrales_provisionales':True,
          'comandos_ruta_todos_cero':True,
          'publicacion_cmd_vel':bool(counts['cmd_vel']>0),
          'cmd_vel_siempre_cero':bool(counts['cmd_vel']>0 and not cmd_vel_nonzero),
          'cmd_vel_publicadores_esperados':1,
          'desplazamiento_odom_m':(math.dist(odom_initial,last['odom'])
                if odom_initial is not None and last['odom'] is not None else None),
          'topic_counts':counts,'route_zero_count':route_count,'cmd_vel_zero_sent':zero_count,
          'cmd_vel_zero_period_s':0.1,
          'cmd_vel_frecuencia_hz_medida':((zero_count-1)/(zero_last_s-zero_first_s)
              if zero_count>1 and zero_first_s is not None and zero_last_s is not None
              and zero_last_s>zero_first_s else None),
          'cmd_vel_observados_todos_cero':bool(counts['cmd_vel']>0 and not cmd_vel_nonzero),
          'transiciones':stage_log,
          'estados_vistos':states,'nota':'Gazebo con cajas temporales y propuesta RUTA cero; '
          'canal real solo a cero, sin autorizacion de movimiento ni cierre R19.'}
        (out/'resumen.json').write_text(json.dumps(payload,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
        with (out/'observaciones.csv').open('w',encoding='utf-8',newline='') as f:
            w=csv.DictWriter(f,fieldnames=['seg','topico','valor'],lineterminator='\n')
            w.writeheader();w.writerows(events)
        say('[EVIDENCIA] '+str(out/'resumen.json'))
        say('[EVIDENCIA] '+str(out/'observaciones.csv'))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return return_code


def stop_own(proc):
    if proc is None or proc.poll() is not None:return
    for sign,wait in [(signal.SIGINT,14),(signal.SIGTERM,8),(signal.SIGKILL,3)]:
        if proc.poll() is not None:break
        try:os.killpg(proc.pid,sign)
        except ProcessLookupError:break
        try:proc.wait(timeout=wait)
        except subprocess.TimeoutExpired:continue


def run():
    preflight()
    require(WORLD.is_file(), 'Mundo original del almacen presente')
    e=env()
    rc=subprocess.run(selfcmd(['--probe']),cwd=REPO,env=e,timeout=45).returncode
    require(rc==0,'Dominio 224 disponible')
    folder=REPO/'resultados/A2_E236A_gazebo_headless_canal_cero'/dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    folder.mkdir(parents=True,exist_ok=False)
    launch=folder/'almacen_e236a_headless.launch.py'
    compile(LAUNCH,str(launch),'exec')
    launch.write_text(LAUNCH,encoding='utf-8',newline='\n')
    say('[INFO] Lanzamiento temporal sin GUI: '+str(launch))
    logserver=(folder/'gzserver_factory.log').open('w',encoding='utf-8')
    logfactory=(folder/'factory_probe.log').open('w',encoding='utf-8')
    loglaunch=(folder/'gazebo_lanzamiento.log').open('w',encoding='utf-8')
    logmonitor=(folder/'monitor.log').open('w',encoding='utf-8')
    server=None;factory=None;p=None;mon=None
    try:
        # Mismo arranque explicitamente comprobado por el diagnostico Factory:
        # Gazebo Classic headless, ROS init y ROS factory.
        server_cmd=['bash','-c',
          'source "$1" >/dev/null && exec gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$2"',
          'bash',str(ACTIVATE),str(WORLD)]
        server=subprocess.Popen(server_cmd,cwd=REPO,env=e,stdout=logserver,
              stderr=subprocess.STDOUT,start_new_session=True)
        say('[INFO] gzserver headless iniciado; comprobando /spawn_entity ANTES del robot...')
        factory=subprocess.Popen(selfcmd(['--factory-probe']),cwd=REPO,env=e,
              stdout=logfactory,stderr=subprocess.STDOUT,start_new_session=True)
        limit=time.monotonic()+FACTORY_WAIT_S+8
        while factory.poll() is None:
            if server.poll() is not None:
                raise RuntimeError('gzserver terminó antes de ofrecer /spawn_entity.')
            if time.monotonic()>limit:
                raise RuntimeError('Timeout comprobando Factory; no se insertará robot.')
            time.sleep(.3)
        factory_log=(folder/'factory_probe.log').read_text(encoding='utf-8',errors='replace')
        for line in factory_log.splitlines():say(line)
        require(factory.returncode==0 and server.poll() is None,
                'ROS Factory listo, servidor activo; insercion del robot autorizada')
        # Ahora se lanza solo el robot y los nodos ROS. NUNCA otro gzserver/gzclient.
        cmd=['bash','-c','source "$1" >/dev/null && exec ros2 launch "$2"',
             'bash',str(ACTIVATE),str(launch)]
        p=subprocess.Popen(cmd,cwd=REPO,env=e,stdout=loglaunch,
              stderr=subprocess.STDOUT,start_new_session=True)
        # Segunda barrera: no empezar observacion hasta que Gazebo confirme la
        # insercion real de waffle_pi. De lo contrario fallar pronto.
        spawned=False
        timeout_spawn=time.monotonic()+55
        while time.monotonic()<timeout_spawn:
            if server.poll() is not None:
                raise RuntimeError('gzserver termino antes de insertar el robot.')
            if p.poll() is not None:
                raise RuntimeError('ros2 launch termino antes de insertar el robot.')
            captured=(folder/'gazebo_lanzamiento.log').read_text(
                encoding='utf-8',errors='replace')
            if 'Successfully spawned entity [waffle_pi]' in captured:
                spawned=True
                break
            if ('Spawn service failed' in captured or
                    'Service /spawn_entity unavailable' in captured):
                raise RuntimeError('Falló spawn_entity; abortando antes del monitor.')
            time.sleep(.4)
        require(spawned,'TurtleBot3 insertado correctamente antes de observar /cmd_vel')
        mon=subprocess.Popen(selfcmd(['--monitor','--out',str(folder)]),cwd=REPO,env=e,
              stdout=logmonitor,stderr=subprocess.STDOUT,start_new_session=True)
        last_msg=0
        cutoff=time.monotonic()+DISCOVER_S+5*STAGE_S+65
        while mon.poll() is None:
            if server.poll() is not None:
                raise RuntimeError('gzserver terminó prematuramente durante el ensayo')
            if p.poll() is not None:
                raise RuntimeError('El lanzamiento de robot/nodos terminó antes del monitor')
            if time.monotonic()>cutoff:
                raise RuntimeError('Tiempo global del ensayo agotado')
            if time.monotonic()-last_msg>18:
                say('[INFO] Gazebo headless y monitor en dominio aislado; logs: '+str(folder))
                last_msg=time.monotonic()
            time.sleep(.5)
        output=(folder/'monitor.log').read_text(encoding='utf-8',errors='replace')
        for line in output.strip().splitlines()[-45:]:say(line)
        if mon.returncode:
            raise RuntimeError('Ensayo ROS no aprobado; monitor codigo '+str(mon.returncode))
        launch_output=(folder/'gazebo_lanzamiento.log').read_text(encoding='utf-8',errors='replace')
        require('Successfully spawned entity [waffle_pi]' in launch_output,
                'Gazebo confirmó la insercion exitosa de waffle_pi')
        say('[LOG] '+str(folder/'gzserver_factory.log'))
        say('[LOG] '+str(folder/'gazebo_lanzamiento.log'))
        return 0
    except Exception:
        logserver.flush();loglaunch.flush();logfactory.flush();logmonitor.flush()
        for file in (folder/'factory_probe.log',folder/'gazebo_lanzamiento.log',folder/'gzserver_factory.log'):
            if file.exists():
                for line in file.read_text(encoding='utf-8',errors='replace').splitlines()[-9:]:
                    say('[LOG '+file.name+'] '+line)
        raise
    finally:
        if mon is not None and mon.poll() is None:
            mon.terminate()
            try:mon.wait(timeout=5)
            except subprocess.TimeoutExpired:mon.kill();mon.wait(timeout=5)
        # Detener exclusivamente los grupos de procesos iniciados por esta prueba.
        stop_own(p)
        stop_own(factory)
        stop_own(server)
        logserver.close();logfactory.close();loglaunch.close();logmonitor.close()
        say('[OK] Cierre del ensayo; sin intervenir procesos externos.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--factory-probe',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--monitor',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--out',help=argparse.SUPPRESS)
    args=parser.parse_args()
    try:
        if args.probe:return probe()
        if args.factory_probe:return factory_probe()
        if args.monitor:
            if not args.out:raise RuntimeError('Falta --out')
            return monitor(Path(args.out))
        return run()
    except KeyboardInterrupt:
        say('[CANCELADO] Ensayo interrumpido; no quedan comandos de ruta activos.')
        return 130
    except Exception as exc:
        say('[ERROR/PENDIENTE] '+str(exc))
        say('[NOTA] /cmd_vel solo publico cero; sin cambios al FIS, supervisor o launch original. Sin Git add/commit/push.')
        return 1


if __name__=='__main__':
    sys.exit(main())
