#!/usr/bin/env python3
"""A2 / R20 - Bateria unica de navegacion: cuatro rutas A* x cinco repeticiones.

Descargar en Windows y ejecutar en Ubuntu WSL:
 python3 /mnt/c/Users/Henry/Downloads/A2_R20_bateria_integral_4x5_gazebo_v2_teleport.py

SOLO Gazebo Classic headless + ROS 2 Humble, dominio 224 aislado.
Una sola sesion Gazebo, un publicador /cmd_vel, supervisor Mamdani integrado.
La geometria del mundo se conserva: se añade un plugin SOLO a una copia temporal.
No hacer spawn/delete repetido de waffle_pi: prueba v1 fallo tras el primer reset.
Usa el seguidor existente sobre cada ruta A* completa. Reposiciona waffle_pi
con gazebo_ros_state (sin borrarlo ni reiniciar sus plugins); registra rosbag2, CSV, JSON,
resumen.csv y agregado_por_ruta.csv. No realiza commits ni cambia archivos
versionados. Los umbrales 0.60/0.80 m NO son valores oficiales acreditados.

IMPORTANTE: el programa puede tardar bastante porque recorre 4 rutas completas
cinco veces. Ante violaciones de seguridad aborta la bateria y emite ceros.
Una llegada de ruta fallida queda registrada y permite pasar al siguiente
ensayo si no hubo violacion critica. No se aprueba R20 con 20 fallos.
"""
from __future__ import annotations

import argparse
import csv
from collections import deque, Counter
import datetime as dt
import json
import math
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import sys
import time

REPO = Path.home() / 'AMR_Logistico_Fase_0'
VENV = Path.home() / '.venvs/amr-a2-r18/bin/python'
ACTIVATE = REPO / 'tools/activar_entorno.sh'
WORLD = REPO / 'modelos/almacen_f1.world'
EXPECTED = '82ba1d98a4201b01e7eaaa42146c163808dcffec'
DOMAIN = '224'
REPETITIONS = 5
ROUTES = tuple(range(1, 5))
PER_TRIAL_TIMEOUT_S = 260.0
FACTORY_TIMEOUT_S = 45.0
DISCOVERY_TIMEOUT_S = 65.0
MAX_V = .10
MAX_W = .50
FRONT_SCAN_STOP = .43
FRONT_RANGE_STOP = .40
SIDE_STOP = .30
POSE_TOL = .20
GOAL_TOL = .10
MAX_DISK_REQUIRED_GB = 1.2
TRACKED = (
    'matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis',
    'tools/supervisor_f13.py',
    'launch/almacen_f1.launch.py',
    'launch/almacen_operativo_f1.launch.py',
    'tools/emulador_sensores_f1.py',
    'tools/A2_E22_nodo_difuso.py',
    'tools/A2_E23_arbitraje_sombra.py',
    'tools/A2_E233_supervisor_integrado_aislado.py',
    'tools/seguidor_ruta_f13.py',
    *(f'resultados/F12_ruta_{n}.csv' for n in ROUTES),
)
BAG_TOPICS = ('/clock', '/odom', '/scan', '/rango_frontal', '/rango_izq',
              '/rango_der', '/cmd_vel_ruta', '/cmd_vel_difuso',
              '/cmd_vel_integrado_prueba', '/estado_integrado_prueba', '/cmd_vel')

STATE_PLUGIN = '''  <plugin name="a2_r20_state" filename="libgazebo_ros_state.so">
    <ros><namespace>/gazebo</namespace></ros>
    <update_rate>20.0</update_rate>
  </plugin>
'''


def world_with_state_plugin(original: str) -> str:
    """Retorna la copia del mundo con plugin ROS State (sin alterar el original)."""
    import xml.etree.ElementTree as ET
    root=ET.fromstring(original)
    worlds=root.findall('world')
    if len(worlds)!=1 or root.tag!='sdf':
        raise RuntimeError('El mundo SDF original no tiene un unico elemento world')
    if any('libgazebo_ros_state.so' in p.attrib.get('filename','')
           for p in worlds[0].findall('plugin')):
        raise RuntimeError('El mundo ya incluye ros_state: revisar para no duplicar')
    if original.count('</world>')!=1:
        raise RuntimeError('No se puede insertar plugin de forma segura')
    updated=original.replace('</world>',STATE_PLUGIN+'  </world>')
    check=ET.fromstring(updated)
    assert len(check.find('world').findall('plugin'))==len(worlds[0].findall('plugin'))+1
    return updated

LAUNCH = '''#!/usr/bin/env python3
"""R20: Gazebo servidor ya iniciado y Factory confirmada."""
import subprocess,sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    root=Path.home()/'AMR_Logistico_Fase_0'
    py=Path.home()/'.venvs/amr-a2-r18/bin/python'
    modelo=Path(get_package_share_directory('turtlebot3_gazebo'))/'models/turtlebot3_waffle_pi/model.sdf'
    urdf=Path(get_package_share_directory('turtlebot3_description'))/'urdf/turtlebot3_waffle_pi.urdf'
    desc=subprocess.check_output(['xacro',str(urdf)],text=True)
    pub=Node(package='robot_state_publisher',executable='robot_state_publisher',
        output='screen',parameters=[{'robot_description':desc,'use_sim_time':True}])
    robot=Node(package='gazebo_ros',executable='spawn_entity.py',output='screen',
        arguments=['-entity','waffle_pi','-file',str(modelo),
            '-x','-3.0','-y','-2.8','-z','0.05','-Y','0.0'])
    sensors=ExecuteProcess(cmd=[sys.executable,'-u',str(root/'tools/emulador_sensores_f1.py'),
        '--ros-args','-p','use_sim_time:=true'],cwd=str(root),output='screen')
    fuzzy=ExecuteProcess(cmd=[str(py),'-u',str(root/'tools/A2_E22_nodo_difuso.py'),
        '--ros','--ros-args','-p','use_sim_time:=true'],cwd=str(root),output='screen')
    arb=ExecuteProcess(cmd=[str(py),'-u',str(root/'tools/A2_E233_supervisor_integrado_aislado.py'),
        '--nodo-aislado'],cwd=str(root),output='screen',additional_env={'_A2_E233_CHILD':'1'})
    return LaunchDescription([pub,robot,sensors,fuzzy,arb])
'''


def say(s):
    print(str(s), flush=True)


def require(cond, message):
    if not cond: raise RuntimeError(message)
    say('[OK] '+message)


def git(*args):
    r=subprocess.run(['git',*args], cwd=REPO, capture_output=True, text=True, timeout=25)
    if r.returncode: raise RuntimeError('Git: '+(r.stderr or r.stdout).strip())
    return r.stdout.strip()


def read_route(n):
    path=REPO/f'resultados/F12_ruta_{n}.csv'
    with path.open(newline='',encoding='utf-8-sig') as f:
        rdr=csv.DictReader(f)
        if not {'x_m','y_m'}.issubset(set(rdr.fieldnames or [])):
            raise ValueError('Ruta sin columnas x_m,y_m: '+str(path))
        pts=[(float(row['x_m']),float(row['y_m'])) for row in rdr]
    require(len(pts)>=2 and all(math.isfinite(x) and math.isfinite(y) for x,y in pts),
            f'Ruta {n}: coordenadas validas')
    lengths=[math.dist(a,b) for a,b in zip(pts,pts[1:])]
    require(max(lengths)<=.15+1e-6,f'Ruta {n}: continuidad <=0.15 m')
    return pts,sum(lengths)


def env():
    e=os.environ.copy()
    e['ROS_DOMAIN_ID']=DOMAIN
    e['ROS_LOCALHOST_ONLY']='1'
    return e


def child_cmd(extra):
    return ['bash','-c','source "$1" >/dev/null && exec "$2" "$3" "${@:4}"',
            'bash',str(ACTIVATE),str(VENV),str(Path(__file__).resolve()),*extra]


def sourced_cmd(command):
    return ['bash','-c','source "$1" >/dev/null && shift && exec "$@"',
            'bash',str(ACTIVATE),*command]


def stop_own(proc,timeout=12):
    if proc is None or proc.poll() is not None:return
    for sig,wait in ((signal.SIGINT,timeout),(signal.SIGTERM,5),(signal.SIGKILL,2)):
        if proc.poll() is not None:break
        try:os.killpg(proc.pid,sig)
        except ProcessLookupError:break
        try:proc.wait(timeout=wait)
        except subprocess.TimeoutExpired:pass


def preflight():
    say('=== R20 v2 - PREFLIGHT INTEGRAL 4 RUTAS x 5 REPETICIONES ===')
    require((REPO/'.git').is_dir(),'Repositorio presente')
    require(git('branch','--show-current')=='desarrollo-avance2','Rama desarrollo-avance2')
    require(git('rev-parse','HEAD')==EXPECTED,'Base E2.3.6A publicada sin commits adicionales')
    for p in TRACKED:
        require((REPO/p).is_file() and git('hash-object','--',p)==git('rev-parse','HEAD:'+p),
                'Integridad conservada: '+p)
    require(ACTIVATE.is_file() and VENV.is_file() and WORLD.is_file(),
            'Entorno ROS, Python y mundo original disponibles')
    t=(REPO/'tools/A2_E233_supervisor_integrado_aislado.py').read_text(encoding='utf8')
    require("OUTPUT = '/cmd_vel_integrado_prueba'" in t,
            'Supervisor candidato sin publicar directamente /cmd_vel')
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            if (p/'comm').read_text(encoding='utf8').strip() in ('gzserver','gzclient'):
                raise RuntimeError('Gazebo ya esta en marcha: PID '+p.name+'. No se interviene.')
        except (FileNotFoundError,PermissionError,UnicodeError):pass
    say('[OK] Ningun Gazebo previo')
    routes={n:read_route(n) for n in ROUTES}
    say('[RUTAS] '+', '.join(f'{n}={routes[n][1]:.3f} m' for n in ROUTES))
    disk=shutil.disk_usage(REPO)
    require(disk.free>=int(MAX_DISK_REQUIRED_GB*1_000_000_000),
            'Espacio libre >=1.2 GB para 20 rosbags')
    bag=subprocess.run(sourced_cmd(['ros2','bag','record','--help']),cwd=REPO,env=env(),
                       stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True,timeout=25)
    require(bag.returncode==0,'rosbag2 disponible para grabar 20 ensayos')
    say('[ADVERTENCIA] Cuatro rutas A* son la definicion experimental de escenarios R20.')
    say('[ADVERTENCIA] Los umbrales .60/.80 m NO estan confirmados por el enunciado.')
    return routes


def probe():
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    n=Node('a2_r20_dominio_probe',enable_rosout=False)
    try:
        until=time.monotonic()+2.8
        while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.15)
        listed=[x for x,_ in n.get_topic_names_and_types() if x not in ('/parameter_events','/rosout')]
        require(not listed and n.count_publishers('/cmd_vel')==0,
                'Dominio 224 libre y /cmd_vel sin publicadores')
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0


def factory_probe():
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    n=Node('a2_r20_factory_probe',enable_rosout=False)
    try:
        until=time.monotonic()+FACTORY_TIMEOUT_S
        while time.monotonic()<until:
            rclpy.spin_once(n,timeout_sec=.20)
            if 'gazebo_msgs/srv/SpawnEntity' in dict(n.get_service_names_and_types()).get('/spawn_entity',[]):
                say('[OK] /spawn_entity operativo antes del robot')
                return 0
        raise RuntimeError('Gazebo Factory ausente. No se autoriza movimiento.')
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def angle(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def polyline_dist(x,y,pts):
    nearest=float('inf')
    for (ax,ay),(bx,by) in zip(pts,pts[1:]):
        dx,dy=bx-ax,by-ay
        den=dx*dx+dy*dy
        t=max(0.,min(1.,((x-ax)*dx+(y-ay)*dy)/den)) if den else 0.
        nearest=min(nearest,math.hypot(x-ax-t*dx,y-ay-t*dy))
    return nearest


def gate(now,enabled,stamps,measure,producers):
    """Arbitraje ya decidido por E2.3.3; pasarela de seguridad adicional.
    Devuelve v,w,modo. Ante fallo, SIEMPRE ceros, sin latched motion.
    """
    def zero(why):return (0.,0.,'CERO',why)
    if producers!=1:return zero('NO_EXCLUSIVIDAD')
    if not enabled:return zero('NO_ARMADO')
    for k,limit in (('scan',.75),('df',.75),('di',.75),('dd',.75),
                    ('odom',.80),('candidate',.38),('state',.38)):
        t=stamps.get(k)
        if t is None or not math.isfinite(t) or not 0<=now-t<=limit:
            return zero('DATO_INVALIDO_O_CADUCADO_'+k)
    vals=[measure.get(k) for k in ('scan','df','di','dd')]
    if any(v is None or not math.isfinite(v) for v in vals):return zero('RANGO_INVALIDO')
    scan,df,di,dd=vals
    if any(not 0<=v<=2 for v in (df,di,dd)):return zero('RANGO_RANGE_FUERA_DOMINIO')
    if scan<=FRONT_SCAN_STOP or df<=FRONT_RANGE_STOP or di<=SIDE_STOP or dd<=SIDE_STOP:
        return zero('BARRERA_INDEPENDIENTE')
    pose=measure.get('odom')
    if not isinstance(pose,tuple) or len(pose)!=3 or not all(math.isfinite(x) for x in pose):
        return zero('ODOMETRIA_INVALIDA')
    candidate=measure.get('candidate')
    if not isinstance(candidate,tuple) or len(candidate)!=2 or not all(math.isfinite(v) for v in candidate):
        return zero('CANDIDATO_INVALIDO')
    v,w=candidate
    if not 0<=v<=.120001 or abs(w)>.700001:
        return zero('CANDIDATO_FUERA_CONTRATO')
    state=measure.get('state')
    if state=='RUTA|PROPUESTA_RUTA_VALIDA':
        if scan<=.60 or df<=.60:return zero('RUTA_CON_OBSTACULO')
        return min(v,MAX_V),max(-MAX_W,min(MAX_W,w)),'RUTA','VALIDO'
    if state=='DIFUSO|PROPUESTA_DIFUSO_VALIDA':
        return min(v,.06),max(-MAX_W,min(MAX_W,w)),'DIFUSO','VALIDO'
    return zero('SUPERVISOR_NO_AUTORIZA')


def self_test():
    now=100.
    st={k:now for k in ('scan','df','di','dd','odom','candidate','state')}
    m={'scan':1.5,'df':1.5,'di':1.0,'dd':1.0,'odom':(0.,0.,0.),
       'candidate':(.12,.7),'state':'RUTA|PROPUESTA_RUTA_VALIDA'}
    assert gate(now,True,st,m,1)[:3]==(.10,.50,'RUTA')
    assert gate(now,False,st,m,1)[:3]==(0.,0.,'CERO')
    assert gate(now,True,st,m,2)[:3]==(0.,0.,'CERO')
    d=dict(m,state='DIFUSO|PROPUESTA_DIFUSO_VALIDA')
    assert gate(now,True,st,d,1)[:3]==(.06,.50,'DIFUSO')
    for k in ('scan','df','di','dd','odom','candidate','state'):
        old=dict(st);old[k]=now-3.
        assert gate(now,True,old,m,1)[:3]==(0.,0.,'CERO')
    for k,val in [('scan',.3),('df',.3),('di',.2),('dd',.2),('scan',math.nan),
                  ('df',math.nan),('df',2.5),('candidate',(math.inf,0.)),
                  ('candidate',(-.1,0.)),('odom',(math.nan,0.,0.)),
                  ('state','DETENIDO|PARADA_SCAN_ESTRICTA')]:
        old=dict(m);old[k]=val
        assert gate(now,True,st,old,1)[:3]==(0.,0.,'CERO'),(k,val)
    assert abs(polyline_dist(1,1,[(0,0),(2,0)])-1)<1e-8
    assert abs(polyline_dist(0,0,[(0,0),(2,0)]))<1e-8
    original='<sdf version="1.6"><world name="w"><physics type="ode"/></world></sdf>'
    changed=world_with_state_plugin(original)
    assert 'libgazebo_ros_state.so' in changed and original not in changed
    assert world_with_state_plugin(original)==changed
    assert changed.count('<plugin ')==1
    assert original.count('<plugin ')==0
    say('[OK] 28 autopruebas: autoridad, limites, paro, caducidad, geometria y world temporal')
    return 0


def bag_statistics(folder):
    dbs=list(folder.glob('*.db3'))
    metadata=folder/'metadata.yaml'
    if not dbs or not metadata.is_file():return None
    counts={}
    for db in dbs:
        conn=sqlite3.connect(f'file:{db}?mode=ro',uri=True,timeout=8)
        try:
            rows=conn.execute('SELECT t.name, COUNT(*) FROM messages m JOIN topics t ON m.topic_id=t.id GROUP BY t.name').fetchall()
            for name,n in rows:counts[name]=counts.get(name,0)+int(n)
        finally:conn.close()
    return counts


def emit_csv(path,rows,columns=None):
    if columns is None:columns=list(rows[0].keys()) if rows else ['vacio']
    with path.open('w',newline='',encoding='utf8') as f:
        w=csv.DictWriter(f,fieldnames=columns,lineterminator='\n',extrasaction='ignore')
        w.writeheader();w.writerows(rows)


def run_monitor(out):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from gazebo_msgs.srv import SetEntityState
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan,Range
    from std_msgs.msg import String
    require(os.environ.get('ROS_DOMAIN_ID')==DOMAIN and
            os.environ.get('ROS_LOCALHOST_ONLY')=='1','Monitor en dominio aislado')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Node('a2_r20_banco_integral')
    states={'scan':None,'df':None,'di':None,'dd':None,'odom':None,
            'candidate':None,'state':None}
    stamps={};publisher=node.create_publisher(Twist,'/cmd_vel',10)
    pending=deque();faults=[];current=None;trial_logs=[];trial_events=[]
    current_pose=None;last_odom_at=0.;last_scan_at=0.
    recording=False;count_sent=Counter();count_recv=Counter();enabled=False
    trials=[];active_recorder=None;active_follower=None;recorder_log=None;follower_log=None
    teleport_client=node.create_client(SetEntityState,'/gazebo/set_entity_state')
    done=0
    start_mono=time.monotonic()

    def record_sample():
        if not recording:return
        pos=current_pose
        if pos is None:return
        trial_logs.append({'t_s':round(time.monotonic()-trial_t0,3),
            'x_m':round(pos[0],5),'y_m':round(pos[1],5),'yaw_rad':round(pos[2],5),
            'scan_frontal_m':states['scan'],'range_frontal_m':states['df'],
            'modo':states['state'],'v_cmd_mps':last_command[0],'w_cmd_rps':last_command[1],
            'error_meta_m':round(math.dist(pos[:2],route_points[-1]),5) if route_points else None,
            'desviacion_ruta_m':round(polyline_dist(pos[0],pos[1],route_points),5) if route_points else None})

    def odom_cb(m):
        nonlocal current_pose,last_odom_at
        p=m.pose.pose.position;q=m.pose.pose.orientation
        coords=(float(p.x),float(p.y),float(angle(q)))
        current_pose=coords if all(math.isfinite(x) for x in coords) else None
        states['odom']=current_pose;stamps['odom']=time.monotonic()
        last_odom_at=stamps['odom']
        record_sample()

    def scan_cb(m):
        nonlocal last_scan_at
        values=[]
        if math.isfinite(m.angle_min) and math.isfinite(m.angle_increment):
            for i,r in enumerate(m.ranges):
                a=math.atan2(math.sin(m.angle_min+i*m.angle_increment),
                             math.cos(m.angle_min+i*m.angle_increment))
                if abs(a)<=math.radians(25):
                    if r==math.inf:r=m.range_max
                    if math.isfinite(r) and m.range_min<=r<=m.range_max:values.append(float(r))
        states['scan']=min(values) if values else None
        stamps['scan']=time.monotonic();last_scan_at=stamps['scan']

    def range_cb(k,m):
        states[k]=float(m.range);stamps[k]=time.monotonic()

    def candidate_cb(m):
        if any(abs(float(v))>1e-7 or not math.isfinite(float(v)) for v in
               (m.linear.y,m.linear.z,m.angular.x,m.angular.y)):
            faults.append('PROPUESTA_CON_EJES_NO_AUTORIZADOS')
        states['candidate']=(float(m.linear.x),float(m.angular.z))
        stamps['candidate']=time.monotonic()

    def state_cb(m):
        states['state']=m.data
        stamps['state']=time.monotonic()
        if recording:trial_events.append({'t_s':round(time.monotonic()-trial_t0,3),'evento':str(m.data)})

    last_command=(0.,0.)
    def cmd_cb(m):
        vals=(m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z)
        if any(not math.isfinite(float(x)) for x in vals):faults.append('CMD_NO_FINITO');return
        if (m.linear.x< -1e-6 or m.linear.x>MAX_V+1e-6 or
            abs(m.angular.z)>MAX_W+1e-6 or
            any(abs(float(x))>1e-6 for x in (m.linear.y,m.linear.z,m.angular.x,m.angular.y))):
            faults.append('CMD_FUERA_LIMITES')
        if not pending:faults.append('CMD_SIN_ENVIO');return
        exp=pending.popleft()
        if (abs(m.linear.x-exp[0])>1e-5 or abs(m.angular.z-exp[1])>1e-5):
            faults.append('CMD_DIFERENTE_ENVIO')
        if recording:count_recv[exp[2]]+=1

    node.create_subscription(Odometry,'/odom',odom_cb,qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/scan',scan_cb,qos_profile_sensor_data)
    for k,t in (('df','/rango_frontal'),('di','/rango_izq'),('dd','/rango_der')):
        node.create_subscription(Range,t,lambda m,k=k:range_cb(k,m),10)
    node.create_subscription(Twist,'/cmd_vel_integrado_prueba',candidate_cb,10)
    node.create_subscription(String,'/estado_integrado_prueba',state_cb,10)
    node.create_subscription(Twist,'/cmd_vel',cmd_cb,10)

    def tick():
        nonlocal last_command
        pubs=node.count_publishers('/cmd_vel')
        if pubs!=1:faults.append('SEGUNDO_PUBLICADOR_CMD_VEL')
        v,w,mode,reason=gate(time.monotonic(),enabled,stamps,states,pubs)
        if faults:v=w=0.;mode='CERO'
        msg=Twist();msg.linear.x=v;msg.angular.z=w
        pending.append((v,w,mode))
        if len(pending)>15:faults.append('COLA_SATURADA')
        publisher.publish(msg)
        last_command=(v,w)
        if recording:count_sent[mode]+=1
    timer=node.create_timer(.10,tick)

    def pump():
        rclpy.spin_once(node,timeout_sec=.06)
        time.sleep(.004)
        if faults:raise RuntimeError('SEGURIDAD: '+', '.join(faults[-4:]))
        if node.count_publishers('/cmd_vel')!=1:
            raise RuntimeError('SEGURIDAD: /cmd_vel sin autoridad exclusiva')

    def wait_until(pred,timeout,label):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            pump()
            if pred():return
        raise RuntimeError('TIMEOUT '+label)

    def service(client,request,label):
        wait_until(lambda:client.service_is_ready(),15.,'servicio '+label)
        fut=client.call_async(request)
        wait_until(lambda:fut.done(),30.,label)
        r=fut.result()
        require(r is not None and r.success,label+': '+str(getattr(r,'status_message','')))

    def reset_robot(points):
        """R20: reubica el MISMO modelo; nunca lo borra ni recarga plugins.

        La comprobacion de /odom y /scan se ancla a la llamada al servicio
        para impedir aceptar lecturas viejas. El actuador queda sin armar.
        """
        nonlocal enabled
        enabled=False
        zero_start=time.monotonic()
        while time.monotonic()-zero_start<.70:pump()
        x,y=points[0]
        yaw=math.atan2(points[1][1]-y,points[1][0]-x)
        pose_before=current_pose
        old_odom=last_odom_at;old_scan=last_scan_at
        request=SetEntityState.Request()
        request.state.name='waffle_pi'
        request.state.reference_frame='world'
        request.state.pose.position.x=float(x)
        request.state.pose.position.y=float(y)
        request.state.pose.position.z=.05
        request.state.pose.orientation.z=math.sin(yaw/2)
        request.state.pose.orientation.w=math.cos(yaw/2)
        # El twist del servicio queda cero en los 6 ejes por construccion.
        require(all(abs(getattr(getattr(request.state.twist,axis),coord))<1e-12
                    for axis in ('linear','angular') for coord in ('x','y','z')),
                'Reposicionamiento con Twist de seis ejes a cero')
        service(teleport_client,request,'teleport waffle_pi')
        start=time.monotonic()
        def nueva_telemetria():
            if current_pose is None or not (last_odom_at>old_odom and last_scan_at>old_scan):
                return False
            dist=math.dist(current_pose[:2],(x,y))
            dyaw=math.atan2(math.sin(current_pose[2]-yaw),math.cos(current_pose[2]-yaw))
            return dist<POSE_TOL and abs(dyaw)<.40
        try:
            wait_until(nueva_telemetria,18.,'odom y scan tras teleport')
        except RuntimeError:
            say('[DIAGNOSTICO] Pose previo='+str(pose_before)+'; actual='+str(current_pose))
            say('[DIAGNOSTICO] Objetivo='+str((x,y,yaw))+'; t_odom='+str(last_odom_at)
                +'; t_scan='+str(last_scan_at)+'; t_anteriores='+str((old_odom,old_scan)))
            raise
        say(f'[OK] Teleport verificado en /odom y /scan: ({current_pose[0]:.3f},'
            f' {current_pose[1]:.3f}), yaw={current_pose[2]:+.3f}; objetivo={x:.3f},{y:.3f}')
        # Periodo sin movimiento con sensores nuevos (no aceptar datos viejos).
        settle=time.monotonic()+.75
        while time.monotonic()<settle:pump()
        require(last_odom_at>=start and last_scan_at>=start,
                'Sensores posteriores al teleport, sin lecturas obsoletas')

    def aggregate():
        groups=[]
        for n in ROUTES:
            xs=[t for t in trials if t['ruta']==n]
            ok=sum(t['estado']=='OK' for t in xs)
            errs=[t['error_final_m'] for t in xs if t['error_final_m'] is not None]
            groups.append({'ruta':n,'ensayos':len(xs),'exitos':ok,
                'tasa_exito_pct':round(100*ok/len(xs),1) if xs else 0.,
                'error_final_prom_m':round(sum(errs)/len(errs),4) if errs else None,
                'tiempo_prom_s':round(sum(t['duracion_s'] for t in xs)/len(xs),3) if xs else None})
        return groups

    def checkpoint():
        emit_csv(out/'resumen.csv',trials,columns=[
            'ruta','repeticion','estado','motivo','duracion_s','longitud_planificada_m',
            'recorrido_real_m','error_final_m','desviacion_rms_m','desviacion_max_m',
            'min_scan_frontal_m','max_v_mps','max_abs_omega_rps','cmd_ruta','cmd_difuso',
            'cmd_cero','rosbag_cmd','rosbag_odom','rosbag_scan','directorio'])
        emit_csv(out/'agregado_por_ruta.csv',aggregate())
        (out/'avance.json').write_text(json.dumps({
            'estado':'EN_PROGRESO' if len(trials)<20 else 'FINALIZADO',
            'n':len(trials),'correctas':sum(t['estado']=='OK' for t in trials),
            'resultados':trials,'umbral_oficial_confirmado':False},ensure_ascii=False,indent=2)+'\n',encoding='utf8')

    route_points=None;trial_t0=0.
    try:
        say('=== R20: 4 RUTAS A* x 5 REPETICIONES EN UNA SESION GAZEBO ===')
        wait_until(lambda:all(k in stamps for k in ('scan','df','di','dd','odom','candidate','state')),
                   DISCOVERY_TIMEOUT_S,'sensores y supervisor')
        require(node.count_publishers('/cmd_vel')==1,'Unico /cmd_vel')
        wait_until(lambda:teleport_client.service_is_ready(),18.,
                   'servicio /gazebo/set_entity_state')
        require(True,'Servicio Gazebo /gazebo/set_entity_state activo; no se borra waffle_pi')
        for repetition in range(1,REPETITIONS+1):
            for route in ROUTES:
                label=f'ruta_{route:02d}_rep_{repetition:02d}'
                folder=out/label
                folder.mkdir(parents=True,exist_ok=False)
                route_points,route_length=read_route(route)
                say(f'\n[ENSAYO {len(trials)+1}/20] {label}: {route_length:.3f} m')
                reset_robot(route_points)
                require(states['scan'] is not None and states['scan']>=.70 and
                        states['df'] is not None and states['df']>=.70,
                        'Condicion inicial despejada '+label)
                trial_logs=[];trial_events=[];count_sent.clear();count_recv.clear()
                # Bag por ensayo, sin alterar archivos historicos de F1.
                bagdir=folder/'rosbag2'
                recorder_cmd=sourced_cmd(['ros2','bag','record','--storage','sqlite3',
                                           '-o',str(bagdir),*BAG_TOPICS])
                recorder_log=(folder/'rosbag.log').open('w',encoding='utf8')
                active_recorder=subprocess.Popen(recorder_cmd,cwd=folder,env=env(),
                    stdout=recorder_log,stderr=subprocess.STDOUT,start_new_session=True)
                trial_t0=time.monotonic()
                time_limit=trial_t0+PER_TRIAL_TIMEOUT_S
                end=time.monotonic()+1.6
                while time.monotonic()<end:
                    pump()
                    if active_recorder.poll() is not None:
                        raise RuntimeError('ros2 bag record termino antes de iniciar el seguimiento')
                follower_cmd=sourced_cmd(['python3',str(REPO/'tools/seguidor_ruta_f13.py'),
                    '--ruta',str(REPO/f'resultados/F12_ruta_{route}.csv'),
                    '--ros-args','-p','use_sim_time:=true'])
                follower_log=(folder/'seguidor.log').open('w',encoding='utf8')
                active_follower=subprocess.Popen(follower_cmd,cwd=folder,env=env(),
                    stdout=follower_log,stderr=subprocess.STDOUT,start_new_session=True)
                recording=True;enabled=True
                outcome='FALLO';reason='';rc=None
                try:
                    while active_follower.poll() is None:
                        pump()
                        if time.monotonic()>time_limit:
                            outcome='FALLO';reason='Timeout ruta '+label
                            break
                        if active_recorder.poll() is not None:
                            raise RuntimeError('rosbag2 dejo de grabar durante '+label)
                    else:
                        rc=active_follower.returncode
                    if not reason:
                        if rc==0:outcome='OK';reason='Seguidor completo la meta'
                        else:outcome='FALLO';reason='Seguidor retorno codigo '+str(rc)
                finally:
                    enabled=False
                    stop_own(active_follower,4)
                    active_follower=None
                    if follower_log:follower_log.close();follower_log=None
                    hold_until=time.monotonic()+1.3
                    while time.monotonic()<hold_until:pump()
                    recording=False
                    stop_own(active_recorder,15)
                    active_recorder=None
                    if recorder_log:recorder_log.close();recorder_log=None
                bagstats=bag_statistics(bagdir)
                if not bagstats:
                    raise RuntimeError('rosbag2 sin SQLite o metadata en '+label)
                require(all(bagstats.get(topic,0)>=10 for topic in ('/odom','/scan','/cmd_vel')),
                        f'Rosbag {label}: /odom, /scan y /cmd_vel presentes')
                require(len(trial_logs)>=10,f'Odometria suficiente {label}')
                require(count_recv['RUTA']+count_recv['DIFUSO']+count_recv['CERO']>=10,
                        'Mensajes /cmd_vel observados '+label)
                xys=[(float(s['x_m']),float(s['y_m'])) for s in trial_logs]
                path=sum(math.dist(a,b) for a,b in zip(xys,xys[1:]))
                dev=[float(s['desviacion_ruta_m']) for s in trial_logs]
                err=float(trial_logs[-1]['error_meta_m'])
                scan=[float(s['scan_frontal_m']) for s in trial_logs
                      if s['scan_frontal_m'] is not None and math.isfinite(s['scan_frontal_m'])]
                vs=[float(s['v_cmd_mps']) for s in trial_logs]
                ws=[float(s['w_cmd_rps']) for s in trial_logs]
                if outcome=='OK' and err>GOAL_TOL:
                    outcome='FALLO';reason='Error meta odometrico mayor de .10m'
                trial={'ruta':route,'repeticion':repetition,'estado':outcome,'motivo':reason,
                    'duracion_s':round(time.monotonic()-trial_t0,3),
                    'longitud_planificada_m':round(route_length,4),
                    'recorrido_real_m':round(path,4),'error_final_m':round(err,4),
                    'desviacion_rms_m':round(math.sqrt(sum(x*x for x in dev)/len(dev)),4),
                    'desviacion_max_m':round(max(dev),4),
                    'min_scan_frontal_m':round(min(scan),4) if scan else None,
                    'max_v_mps':round(max(vs),4),'max_abs_omega_rps':round(max(abs(w) for w in ws),4),
                    'cmd_ruta':count_recv['RUTA'],'cmd_difuso':count_recv['DIFUSO'],
                    'cmd_cero':count_recv['CERO'],
                    'rosbag_cmd':bagstats.get('/cmd_vel',0),'rosbag_odom':bagstats.get('/odom',0),
                    'rosbag_scan':bagstats.get('/scan',0),
                    'directorio':str(folder.relative_to(REPO))}
                emit_csv(folder/'odometria_metricas.csv',trial_logs)
                emit_csv(folder/'estados.csv',trial_events,columns=['t_s','evento'])
                (folder/'resultado.json').write_text(json.dumps(trial,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
                trials.append(trial)
                checkpoint()
                say(f'[{outcome}] {label}: error={err:.3f}m; recorrido={path:.3f}m; '
                    f'RUTA={count_recv["RUTA"]}, DIFUSO={count_recv["DIFUSO"]}; bag /odom={bagstats.get("/odom",0)}')
                done+=1
        success=sum(x['estado']=='OK' for x in trials)
        summary={'estado':'APROBADA_EXPERIMENTAL' if success==20 else 'PARCIAL',
            'n_ensayos':len(trials),'exitos':success,'repeticiones_por_ruta':REPETITIONS,
            'r19_umbrales_oficiales_confirmados':False,
            'nota':'R20 evaluado sobre cuatro rutas A* definidas en F1.2, no escenarios numerados en enunciado.',
            'rutas':aggregate(),'resultados':trials}
        (out/'resumen.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
        say(f'=== R20: {success}/20 ENSAYOS CON META COMPLETADA ===')
        say('[EVIDENCIA] '+str(out/'resumen.csv'))
        say('[EVIDENCIA] '+str(out/'agregado_por_ruta.csv'))
        return 0 if success==20 else 2
    finally:
        enabled=False
        if rclpy.ok():
            for _ in range(12):
                try:pump()
                except Exception:pass
        stop_own(active_follower,4)
        stop_own(active_recorder,15)
        if follower_log:follower_log.close()
        if recorder_log:recorder_log.close()
        if trials:checkpoint()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def run():
    self_test();preflight()
    e=env()
    require(subprocess.run(child_cmd(['--probe']),cwd=REPO,env=e,timeout=35).returncode==0,
            'Aislamiento ROS disponible')
    stamp=dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    out=REPO/'resultados/A2_R20_integral_4x5'/stamp
    out.mkdir(parents=True,exist_ok=False)
    # Copia SOLO dentro de resultados: el world oficial versionado no cambia.
    temporary_world=out/'almacen_r20_con_plugin_state.world'
    temporary_world.write_text(world_with_state_plugin(WORLD.read_text(encoding='utf8')),
                               encoding='utf8',newline='\n')
    say('[OK] Mundo temporal con gazebo_ros_state; original intacto')
    launchfile=out/'almacen_r20_integral.launch.py'
    compile(LAUNCH,str(launchfile),'exec')
    launchfile.write_text(LAUNCH,encoding='utf8',newline='\n')
    file_handles=[(out/x).open('w',encoding='utf8') for x in
                  ('gzserver.log','factory_probe.log','gazebo_launch.log','monitor.log')]
    server=None;factory=None;launch=None;monitor=None
    try:
        server=subprocess.Popen(sourced_cmd(['gzserver','--verbose',
            '-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so',str(temporary_world)]),
            cwd=REPO,env=e,stdout=file_handles[0],stderr=subprocess.STDOUT,start_new_session=True)
        say('[INFO] Gazebo headless iniciado, esperando ROS Factory...')
        factory=subprocess.Popen(child_cmd(['--factory-probe']),cwd=REPO,env=e,
            stdout=file_handles[1],stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+FACTORY_TIMEOUT_S+8
        while factory.poll() is None:
            if server.poll() is not None:raise RuntimeError('gzserver termino durante Factory')
            if time.monotonic()>deadline:raise RuntimeError('Timeout Factory')
            time.sleep(.35)
        file_handles[1].flush()
        for line in (out/'factory_probe.log').read_text(encoding='utf8').splitlines():say(line)
        require(factory.returncode==0 and server.poll() is None,'Factory y servidor Gazebo operativos')
        launch=subprocess.Popen(sourced_cmd(['ros2','launch',str(launchfile)]),cwd=REPO,
            env=e,stdout=file_handles[2],stderr=subprocess.STDOUT,start_new_session=True)
        end=time.monotonic()+56
        while time.monotonic()<end:
            if server.poll() is not None or launch.poll() is not None:raise RuntimeError('Gazebo o launch termino')
            file_handles[2].flush()
            log=(out/'gazebo_launch.log').read_text(encoding='utf8',errors='replace')
            if 'Successfully spawned entity [waffle_pi]' in log:break
            if 'Spawn service failed' in log:raise RuntimeError('Fallo spawn waffle_pi')
            time.sleep(.3)
        else:raise RuntimeError('Timeout spawn waffle_pi inicial')
        require(True,'TurtleBot3 insertado y Gazebo estable')
        monitor=subprocess.Popen(child_cmd(['--monitor','--out',str(out)]),cwd=REPO,
            env=e,stdout=file_handles[3],stderr=subprocess.STDOUT,start_new_session=True)
        last=0.
        last_printed=0
        while monitor.poll() is None:
            if server.poll() is not None or launch.poll() is not None:
                raise RuntimeError('Gazebo/launch termino durante bateria')
            if time.monotonic()-last>16:
                file_handles[3].flush()
                log=(out/'monitor.log').read_text(encoding='utf8',errors='replace')
                all_lines=log.splitlines()
                for line in all_lines[last_printed:]:
                    if line.startswith(('[ENSAYO','[OK]','[FALLO]','[ERROR]','[RESULTADO]')):say(line)
                last_printed=len(all_lines)
                last=time.monotonic()
            time.sleep(.6)
        file_handles[3].flush()
        for line in (out/'monitor.log').read_text(encoding='utf8',errors='replace').splitlines()[-38:]:say(line)
        if monitor.returncode!=0:
            raise RuntimeError('R20 monitor no aprobado (codigo '+str(monitor.returncode)+
                               '). Revisar log y evidencias, no se declara 20/20')
        say('[OK] R20: 20/20 rutas completas; 20 rosbags y metricas')
        return 0
    finally:
        stop_own(monitor,6);stop_own(launch);stop_own(factory);stop_own(server)
        for h in file_handles:h.close()
        say('[OK] Procesos creados por la bateria cerrados; sin modificar Git.')
        say('[EVIDENCIA] '+str(out))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--self-test',action='store_true',help='Sin ROS ni Gazebo')
    p.add_argument('--preflight',action='store_true',help='Sin Gazebo ni movimiento')
    p.add_argument('--probe',action='store_true',help=argparse.SUPPRESS)
    p.add_argument('--factory-probe',action='store_true',help=argparse.SUPPRESS)
    p.add_argument('--monitor',action='store_true',help=argparse.SUPPRESS)
    p.add_argument('--out',help=argparse.SUPPRESS)
    args=p.parse_args()
    try:
        if args.self_test:return self_test()
        if args.preflight:self_test();preflight();return 0
        if args.probe:return probe()
        if args.factory_probe:return factory_probe()
        if args.monitor:
            require(bool(args.out),'Directorio de salida requerido')
            return run_monitor(Path(args.out))
        return run()
    except KeyboardInterrupt:
        say('[INTERRUMPIDO] Ensayo detenido; salida solo cero en cierre controlado.');return 130
    except Exception as exc:
        say('[ERROR R20 INTEGRAL] '+str(exc))
        say('[NOTA] Evidencias parciales conservadas; R20 no aprobado.')
        return 2

if __name__=='__main__':sys.exit(main())
