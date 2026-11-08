#!/usr/bin/env python3
"""AMR Logistico — banco INTEGRAL de prevalidacion operativa R19 en Gazebo.

Desde Windows Descargas, en Ubuntu WSL2:
    python3 /mnt/c/Users/Henry/Downloads/A2_R19_banco_integral_gazebo.py

Una sola sesion headless de Gazebo Classic y una sola autoridad /cmd_vel:
 RUTA en marcha -> DIFUSO en marcha -> PARADA LiDAR -> recuperacion DIFUSO
 -> retorno a RUTA -> fallo de propuesta de ruta -> recuperacion y frenado.

Las velocidades efectivas pasan por una barrera independiente del arbitraje;
se retransmiten propuestas REALES del supervisor E2.3.3, no velocidades
fabricadas a partir del estado. Maximo 0.04 m/s RUTA, 0.025 m/s DIFUSO y
0.20 rad/s; sin giro de RUTA. Abortos/faltas generan velocidad CERO.

SOLO SIMULACION, ROS_DOMAIN_ID=224, ROS_LOCALHOST_ONLY=1. No se altera Git,
FIS, modelos o lanzadores originales. NO acredita umbrales oficiales de R19:
entrada 0.60 m, salida 0.80 m son parametros de ingenieria experimentales.
"""
from __future__ import annotations

import argparse
import csv
from collections import deque
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
VENV = Path.home() / '.venvs/amr-a2-r18/bin/python'
ACTIVATE = REPO / 'tools/activar_entorno.sh'
EXPECTED = '82ba1d98a4201b01e7eaaa42146c163808dcffec'
WORLD = REPO / 'modelos/almacen_f1.world'
DOMAIN = '224'
DISCOVERY_S = 65.0
FACTORY_WAIT_S = 42.0
STAGE_S = 14.0
MAX_GLOBAL_S = 75.0
TRACKED = (
    'matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis',
    'tools/supervisor_f13.py',
    'launch/almacen_f1.launch.py',
    'launch/almacen_operativo_f1.launch.py',
    'tools/emulador_sensores_f1.py',
    'tools/A2_E22_nodo_difuso.py',
    'tools/A2_E23_arbitraje_sombra.py',
    'tools/A2_E233_supervisor_integrado_aislado.py',
)
LAUNCH = '''#!/usr/bin/env python3
"""R19: solo robot y nodos; gzserver ya lanzado headless."""
import sys
import subprocess
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
    description=subprocess.check_output(['xacro',str(urdf)],text=True)
    pub=Node(package='robot_state_publisher',executable='robot_state_publisher',
       name='robot_state_publisher',output='screen',
       parameters=[{'robot_description':description,'use_sim_time':True}])
    robot=Node(package='gazebo_ros',executable='spawn_entity.py',output='screen',
       arguments=['-entity','waffle_pi','-file',str(modelo),'-x','-3.0',
                  '-y','-2.8','-z','0.05','-Y','0.0'])
    sensor=ExecuteProcess(cmd=[sys.executable,'-u',str(root/'tools/emulador_sensores_f1.py'),
       '--ros-args','-p','use_sim_time:=true'],cwd=str(root),output='screen')
    fuzzy=ExecuteProcess(cmd=[str(py),'-u',str(root/'tools/A2_E22_nodo_difuso.py'),
       '--ros','--ros-args','-p','use_sim_time:=true'],cwd=str(root),output='screen')
    shadow=ExecuteProcess(cmd=[str(py),'-u',str(root/'tools/A2_E233_supervisor_integrado_aislado.py'),
       '--nodo-aislado'],cwd=str(root),output='screen',additional_env={'_A2_E233_CHILD':'1'})
    return LaunchDescription([pub,robot,sensor,fuzzy,shadow])
'''


def say(m):
    print(m,flush=True)


def require(ok,label):
    if not ok:raise RuntimeError(label)
    say('[OK] '+label)


def git(*args):
    r=subprocess.run(['git',*args],cwd=REPO,text=True,capture_output=True,timeout=25)
    if r.returncode:raise RuntimeError('Git: '+(r.stderr or r.stdout).strip())
    return r.stdout.strip()


def preflight():
    say('=== R19 - PREFLIGHT INTEGRAL GAZEBO SIMULADO ===')
    require((REPO/'.git').is_dir(),'Repositorio presente')
    require(git('branch','--show-current')=='desarrollo-avance2','Rama correcta')
    require(git('rev-parse','HEAD')==EXPECTED,'Commit base E2.3.6A publicado presente')
    for rel in TRACKED:
        require((REPO/rel).is_file() and git('hash-object','--',rel)==git('rev-parse','HEAD:'+rel),
                'Archivo protegido intacto: '+rel)
    require(VENV.is_file() and ACTIVATE.is_file() and WORLD.is_file(),
            'Entorno ROS y mundo oficial disponibles')
    candidate=(REPO/'tools/A2_E233_supervisor_integrado_aislado.py').read_text(encoding='utf8')
    require("OUTPUT = '/cmd_vel_integrado_prueba'" in candidate,
            'Supervisor candidato sin publicaciones directas /cmd_vel')
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            if (p/'comm').read_text(encoding='utf8').strip() in ('gzserver','gzclient'):
                raise RuntimeError('Ya hay Gazebo en marcha (PID '+p.name+'); no intervenir.')
        except (FileNotFoundError,PermissionError,UnicodeError):pass
    say('[OK] Ningun gzserver/gzclient previo')
    say('[ADVERTENCIA] R19 con umbrales .60/.80 EXPERIMENTALES, no oficiales.')


def env():
    e=os.environ.copy()
    e['ROS_DOMAIN_ID']=DOMAIN
    e['ROS_LOCALHOST_ONLY']='1'
    return e


def child_cmd(extra):
    return ['bash','-c','source "$1" >/dev/null && exec "$2" "$3" "${@:4}"',
            'bash',str(ACTIVATE),str(VENV),str(Path(__file__).resolve()),*extra]


def probe():
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node=Node('a2_r19_aislamiento',enable_rosout=False)
    try:
        deadline=time.monotonic()+3.0
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.12)
        topics=[name for name,_ in node.get_topic_names_and_types()
                if name not in ('/rosout','/parameter_events')]
        require(not topics and node.count_publishers('/cmd_vel')==0,
                'Dominio 224 libre y /cmd_vel sin publicadores')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def factory_probe():
    import rclpy
    from rclpy.node import Node
    require(os.environ.get('ROS_DOMAIN_ID')==DOMAIN and
            os.environ.get('ROS_LOCALHOST_ONLY')=='1','Aislamiento Factory')
    rclpy.init()
    node=Node('a2_r19_factory_probe',enable_rosout=False)
    try:
        until=time.monotonic()+FACTORY_WAIT_S
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.2)
            svc=dict(node.get_service_names_and_types())
            if 'gazebo_msgs/srv/SpawnEntity' in svc.get('/spawn_entity',[]):
                say('[OK] /spawn_entity disponible antes del TurtleBot3')
                return 0
        raise RuntimeError('Factory no disponible; robot NO se insertara.')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def guard(now,phase,stamps,data,publisher_count,initial_pose,arm_at):
    """Decision pura. Devuelve (modo, v, omega, motivo); defecto SIEMPRE cero."""
    def stop(reason):return 'CERO',0.,0.,reason
    if publisher_count!=1:return stop('PUBLICADOR_NO_EXCLUSIVO')
    if phase!='ACTIVO':return stop('FASE_INACTIVA')
    if arm_at is None or not math.isfinite(arm_at) or not 0<=now-arm_at<=12:
        return stop('VENTANA_ARMADO_VENCIDA')
    for name,age in (('scan',.75),('df',.75),('di',.75),('dd',.75),
                     ('odom',.80),('candidate',.38),('state',.38)):
        at=stamps.get(name)
        if at is None or not math.isfinite(at) or not 0<=now-at<=age:
            return stop('DATO_CADUCADO_'+name)
    scan=data.get('scan')
    df,di,dd=(data.get(k) for k in ('df','di','dd'))
    if any(x is None or not math.isfinite(x) for x in (scan,df,di,dd)):
        return stop('RANGO_NO_FINITO')
    if not all(0<=x<=2 for x in (df,di,dd)):
        return stop('RANGO_FUERA_DOMINIO')
    pose=data.get('pose')
    if pose is None or initial_pose is None or not all(math.isfinite(x) for x in (*pose,*initial_pose)):
        return stop('ODOMETRIA_INVALIDA')
    if math.dist(pose,initial_pose)>.18:return stop('DISTANCIA_TOTAL_MAXIMA')
    if scan<=.43 or df<=.40:return stop('BARRERA_FRONTAL_INDEPENDIENTE')
    if di<=.35 or dd<=.35:return stop('BARRERA_LATERAL_INDEPENDIENTE')
    candidate=data.get('candidate')
    if candidate is None or len(candidate)!=2 or not all(math.isfinite(x) for x in candidate):
        return stop('PROPUESTA_INVALIDA')
    v,w=candidate
    if not (0<=v<=.120001 and abs(w)<=.700001):
        return stop('PROPUESTA_FUERA_LIMITES_CANDIDATO')
    state=data.get('state') or ''
    if state=='RUTA|PROPUESTA_RUTA_VALIDA':
        if scan<.85 or df<.80:return stop('RUTA_NO_AUTORIZADA_CON_OBSTACULO')
        if abs(w)>1e-3:return stop('RUTA_GIRO_INESPERADO')
        return 'RUTA',min(v,.04),0.,'PROPUESTA_LIMITADA'
    if state=='DIFUSO|PROPUESTA_DIFUSO_VALIDA':
        return 'DIFUSO',min(v,.025),max(-.20,min(.20,w)),'PROPUESTA_LIMITADA'
    return stop('SUPERVISOR_NO_AUTORIZA_MOVIMIENTO')


def self_test():
    now=100.
    stamps={k:now for k in ('scan','df','di','dd','odom','candidate','state')}
    data={'scan':2.,'df':2.,'di':1.4,'dd':1.3,'pose':(0.,0.),
          'candidate':(.04,0.),'state':'RUTA|PROPUESTA_RUTA_VALIDA'}
    base=(now,'ACTIVO',stamps,data,1,(0.,0.),99.)
    assert guard(*base)[:3]==('RUTA',.04,0.)
    ok=1
    def blocked(**kwargs):
        nonlocal ok
        d=dict(now=now,phase='ACTIVO',stamps=stamps,data=data,
               publisher_count=1,initial_pose=(0.,0.),arm_at=99.)
        d.update(kwargs)
        result=guard(**d)
        assert result[:3]==('CERO',0.,0.),(kwargs,result)
        ok+=1
    blocked(publisher_count=0);blocked(publisher_count=2)
    blocked(phase='HOLD');blocked(phase='DROP_RUTA')
    blocked(arm_at=80.)
    for key in ('scan','df','di','dd','odom','candidate','state'):
        t=dict(stamps);t[key]=now-2.;blocked(stamps=t)
    for key,value in [('scan',.30),('df',.35),('di',.30),('dd',.30),
                      ('scan',math.nan),('df',math.nan),('df',2.1)]:
        d=dict(data);d[key]=value;blocked(data=d)
    d=dict(data,pose=(.19,0.));blocked(data=d)
    d=dict(data,candidate=(math.nan,0.));blocked(data=d)
    d=dict(data,candidate=(.15,0.));blocked(data=d)
    d=dict(data,candidate=(.04,.1));blocked(data=d)
    d=dict(data,state='DETENIDO|PARADA_SCAN_ESTRICTA');blocked(data=d)
    d=dict(data,state='DIFUSO|PROPUESTA_DIFUSO_VALIDA',scan=.52,df=.50,
           candidate=(.12,.70))
    assert guard(now,'ACTIVO',stamps,d,1,(0.,0.),99.)[:3]==('DIFUSO',.025,.20)
    ok+=1
    d=dict(d,candidate=(.013,-.70))
    assert guard(now,'ACTIVO',stamps,d,1,(0.,0.),99.)[:3]==('DIFUSO',.013,-.20)
    ok+=1
    say(f'[OK] {ok} verificaciones puras: limites, datos caducados, barreras y modos.')
    return 0


def box_xml(name):
    return f'''<?xml version="1.0"?>
<sdf version="1.6"><model name="{name}"><static>true</static>
<link name="box"><collision name="collision"><geometry>
<box><size>0.18 0.20 0.40</size></box></geometry></collision>
<visual name="visual"><geometry><box><size>0.18 0.20 0.40</size></box></geometry>
<material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material>
</visual></link></model></sdf>'''


def monitor(folder):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan, Range
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from gazebo_msgs.srv import SpawnEntity, DeleteEntity

    require(os.environ.get('ROS_DOMAIN_ID')==DOMAIN and
            os.environ.get('ROS_LOCALHOST_ONLY')=='1','Monitor ROS aislado en dominio 224')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Node('a2_r19_integral_monitor')
    topics=('scan','odom','df','di','dd','fuzzy','fuzzy_state',
            'candidate','candidate_state','cmd_vel')
    counts={k:0 for k in topics}
    data={'scan':None,'df':None,'di':None,'dd':None,'pose':None,'yaw':None,
          'candidate':None,'state':None}
    stamps={};events=[];stages=[];faults=[];created=[]
    mode_counts={'RUTA':0,'DIFUSO':0,'CERO':0}
    command_sent={'RUTA':0,'DIFUSO':0,'CERO':0}
    command_recv={'RUTA':0,'DIFUSO':0,'CERO':0}
    phase='HOLD';arm_at=None;started=time.monotonic();overall_zero=True
    initial_pose=None; last_pose=None;path_m=0.
    command_last=None;output_seq=0
    pending_commands=deque();recv_active_modes=[]
    active_cmd_modes=[];last_reason='';latest_decision=('CERO',0.,0.,'INICIO')
    last_motion_at=0.;status='NO_APROBADA';return_code=2
    route_pub=node.create_publisher(Twist,'/cmd_vel_ruta',10)
    output_pub=node.create_publisher(Twist,'/cmd_vel',10)
    publish_timer=None;spawn_client=None;delete_client=None

    def record(topic,value,extra=None):
        counts[topic]+=1
        if len(events)<30000:
            events.append({'t_s':round(time.monotonic()-started,3),
                           'fase':phase,'topico':topic,'valor':str(value)[:120],
                           'scan_m':data['scan'],'range_m':data['df'],
                           'x_m':data['pose'][0] if data['pose'] else None,
                           'y_m':data['pose'][1] if data['pose'] else None,
                           'estado':data['state'],'modo_emitido':extra})

    def on_scan(m):
        vals=[]
        if math.isfinite(m.angle_min) and math.isfinite(m.angle_increment):
            for i,dist in enumerate(m.ranges):
                a=math.atan2(math.sin(m.angle_min+i*m.angle_increment),
                             math.cos(m.angle_min+i*m.angle_increment))
                if abs(a)<=math.radians(25) and math.isfinite(dist) and m.range_min<=dist<=m.range_max:
                    vals.append(float(dist))
        data['scan']=min(vals) if vals else None
        stamps['scan']=time.monotonic();record('scan',data['scan'])

    def on_odom(m):
        nonlocal initial_pose,last_pose,path_m
        p=m.pose.pose.position;q=m.pose.pose.orientation
        xy=(float(p.x),float(p.y))
        data['pose']=xy if all(math.isfinite(x) for x in xy) else None
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        data['yaw']=float(yaw) if math.isfinite(yaw) else None
        if data['pose'] is not None:
            if initial_pose is None:initial_pose=data['pose']
            if last_pose is not None:
                step=math.dist(last_pose,data['pose'])
                if math.isfinite(step) and step<.03:path_m+=step
            last_pose=data['pose']
        stamps['odom']=time.monotonic();record('odom',data['pose'])

    def on_range(key,m):
        data[key]=float(m.range);stamps[key]=time.monotonic()
        record(key,round(data[key],4))

    def on_proposal(k,m):
        axes=(m.linear.y,m.linear.z,m.angular.x,m.angular.y)
        if k=='candidate':
            if any(not math.isfinite(float(v)) or abs(v)>1e-7 for v in axes):
                faults.append('PROPUESTA_CANDIDATA_EJES_INVALIDOS')
            data['candidate']=(float(m.linear.x),float(m.angular.z))
            stamps['candidate']=time.monotonic()
        record(k,f'v={m.linear.x:.5f};w={m.angular.z:+.5f}')

    def on_state(k,m):
        if k=='candidate_state':
            data['state']=str(m.data);stamps['state']=time.monotonic()
        record(k,m.data)

    node.create_subscription(LaserScan,'/scan',on_scan,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/odom',on_odom,qos_profile_sensor_data)
    for k,t in (('df','/rango_frontal'),('di','/rango_izq'),('dd','/rango_der')):
        node.create_subscription(Range,t,lambda msg,key=k:on_range(key,msg),10)
    node.create_subscription(Twist,'/cmd_vel_difuso',lambda m:on_proposal('fuzzy',m),10)
    node.create_subscription(String,'/estado_difuso',lambda m:on_state('fuzzy_state',m),10)
    node.create_subscription(Twist,'/cmd_vel_integrado_prueba',lambda m:on_proposal('candidate',m),10)
    node.create_subscription(String,'/estado_integrado_prueba',lambda m:on_state('candidate_state',m),10)

    # El receptor audita todos los ejes del unico topico real.
    def on_cmd(m):
        nonlocal overall_zero
        vals=(m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z)
        if not all(math.isfinite(float(v)) for v in vals):
            faults.append('CMD_NO_FINITO');return
        v,w=float(m.linear.x),float(m.angular.z)
        if any(abs(float(z))>1e-6 for z in (m.linear.y,m.linear.z,m.angular.x,m.angular.y)):
            faults.append('CMD_EJES_NO_AUTORIZADOS')
        if v< -1e-6 or v>.040001 or abs(w)>.200001:
            faults.append('CMD_FUERA_DE_LIMITES')
        if abs(v)>1e-7 or abs(w)>1e-7:
            overall_zero=False
        # Un unico publicador; cotejar cada mensaje recibido con la orden
        # concreta emitida, no inferir su modo a partir del ultimo timer.
        if not pending_commands:
            faults.append('CMD_RECIBIDO_SIN_ENVIO')
            mode='CERO'
        else:
            mode,expected_v,expected_w=pending_commands.popleft()
            if abs(expected_v-v)>1e-5 or abs(expected_w-w)>1e-5:
                faults.append('CMD_RECIBIDO_DIFIERE_DEL_ENVIADO')
        command_recv[mode]+=1
        if v>0.0005:
            recv_active_modes.append((time.monotonic(),mode,v,w))
        record('cmd_vel',f'v={v:.5f};w={w:+.5f}',mode)

    node.create_subscription(Twist,'/cmd_vel',on_cmd,10)

    def tick():
        nonlocal latest_decision,command_last,output_seq,last_motion_at,last_reason
        now=time.monotonic()
        publishers=node.count_publishers('/cmd_vel')
        if publishers!=1:
            faults.append('SEGUNDO_PUBLICADOR_O_SIN_PUBLICADOR')
        route=Twist()
        if phase=='ACTIVO':route.linear.x=.04
        # En DROP_RUTA no se emite propuesta alguna: timeout supervisado.
        if phase!='DROP_RUTA':route_pub.publish(route)
        mode,v,w,why=guard(now,phase,stamps,data,publishers,initial_pose,arm_at)
        msg=Twist();msg.linear.x=v;msg.angular.z=w
        if faults:msg=Twist();mode='CERO';why='FALLO_CRITICO'
        pending_commands.append((mode,float(msg.linear.x),float(msg.angular.z)))
        # Nunca dejar que el monitor se atrase de forma indefinida.
        if len(pending_commands)>15:faults.append('COLA_CMD_VEL_ATRASADA')
        output_pub.publish(msg)
        command_sent[mode]+=1
        command_last=mode
        output_seq+=1
        latest_decision=(mode,float(msg.linear.x),float(msg.angular.z),why)
        if mode in ('RUTA','DIFUSO') and (abs(msg.linear.x)>1e-7 or abs(msg.angular.z)>1e-7):
            active_cmd_modes.append((now,mode,float(msg.linear.x),float(msg.angular.z)))
            last_motion_at=now
        if why!=last_reason:
            last_reason=why
            if len(events)<30000:
                events.append({'t_s':round(now-started,3),'fase':phase,'topico':'decision',
                    'valor':why,'scan_m':data['scan'],'range_m':data['df'],
                    'x_m':data['pose'][0] if data['pose'] else None,
                    'y_m':data['pose'][1] if data['pose'] else None,
                    'estado':data['state'],'modo_emitido':mode})

    publish_timer=node.create_timer(.10,tick)

    def pump():
        if time.monotonic()-started>MAX_GLOBAL_S:
            raise RuntimeError('TIEMPO_GLOBAL_EXCEDIDO')
        rclpy.spin_once(node,timeout_sec=.06)
        time.sleep(.005)
        if faults:raise RuntimeError('SEGURIDAD: '+', '.join(faults[-4:]))
        if node.count_publishers('/cmd_vel')!=1:
            raise RuntimeError('SEGURIDAD: /cmd_vel no tiene publicador unico')

    def until(predicate,timeout,label):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            pump()
            if predicate():return
        raise RuntimeError('Timeout '+label+'; estado='+str(data['state'])+
                           '; scan='+str(data['scan'])+'; range='+str(data['df']))

    def hold(seconds=.6):
        nonlocal phase
        phase='HOLD';end=time.monotonic()+seconds
        while time.monotonic()<end:pump()

    def stage(label,pred,timeout=STAGE_S,mode=None):
        start=time.monotonic()
        before=len(active_cmd_modes)
        until(pred,timeout,label)
        data_stage={'etapa':label,'estado':data['state'],'scan_m':data['scan'],
                    'frontal_m':data['df'],'pose_xy':data['pose'],
                    'comandos_movimiento_nuevos':len(active_cmd_modes)-before,
                    'modo_salida':latest_decision[0],
                    'tiempo_s':round(time.monotonic()-start,3)}
        stages.append(data_stage)
        say(f'[OK] {label}: {data_stage["estado"]}; scan={data_stage["scan_m"]:.3f}; '
            f'Range={data_stage["frontal_m"]:.3f}; salida={data_stage["modo_salida"]}; '
            f'cmds nuevos={data_stage["comandos_movimiento_nuevos"]}')

    def get_client(cls,which):
        names=['/'+which]
        until(lambda:any(n in dict(node.get_service_names_and_types()) for n in names),
              20.,'servicio '+which)
        return node.create_client(cls,names[0])

    def call(client,req,label):
        future=client.call_async(req)
        until(lambda:future.done(),18.,label)
        result=future.result()
        require(result is not None and result.success,
                'Servicio Gazebo '+label+': '+str(getattr(result,'status_message','sin estado')))

    def spawn(name,offset):
        require(data['pose'] is not None and data['yaw'] is not None,
                'Posicion/rumbo validos para obstaculo '+name)
        # Desplazamiento relativo al eje longitudinal local del robot.
        px,py=data['pose'];a=data['yaw']
        req=SpawnEntity.Request();req.name=name;req.xml=box_xml(name)
        req.reference_frame='world';req.robot_namespace=''
        req.initial_pose.position.x=px+offset*math.cos(a)
        req.initial_pose.position.y=py+offset*math.sin(a)
        req.initial_pose.position.z=.30
        req.initial_pose.orientation.w=1.
        # Registrar antes de llamar, de manera que una interrupcion tras
        # aceptacion de Gazebo no impida intentar eliminar el obstaculo.
        created.append(name)
        call(spawn_client,req,'spawn '+name)

    def delete(name):
        req=DeleteEntity.Request();req.name=name
        call(delete_client,req,'delete '+name)
        if name in created:created.remove(name)

    def positive_mode(mode,since,count=2):
        return sum(1 for t,m,v,w in active_cmd_modes
                   if t>=since and m==mode and v>0.0005)>=count

    try:
        say('=== R19 - ENSAYO INTEGRAL CON MOVIMIENTO Y OBSTACULOS ===')
        say('[SEGURIDAD] Un unico /cmd_vel a 10 Hz; salida del candidato limitada y vigilada.')
        until(lambda:all(counts.values()),DISCOVERY_S,'10 topicos activos')
        require(node.count_publishers('/cmd_vel')==1,'10 topicos y autoridad unica /cmd_vel')
        require(data['scan'] is not None and data['scan']>=1.50 and
                data['df'] is not None and data['df']>=1.50,
                'Pasillo libre antes de autorizar movimiento (LiDAR/Range)')
        until(lambda:data['state']=='RUTA|PROPUESTA_RUTA_VALIDA'
              and data['candidate'] is not None and abs(data['candidate'][0])<1e-6,
              STAGE_S,'RUTA inicial cero')
        hold(.4)
        initial_pose=data['pose']  # referencia inmediata antes de primer movimiento
        require(initial_pose is not None,'Odometria inicial valida')
        spawn_client=get_client(SpawnEntity,'spawn_entity')
        delete_client=get_client(DeleteEntity,'delete_entity')

        phase='ACTIVO';arm_at=time.monotonic();t0=arm_at
        stage('RUTA con desplazamiento controlado',
              lambda:positive_mode('RUTA',t0,3) and
                     data['state']=='RUTA|PROPUESTA_RUTA_VALIDA',4)
        # Insertar obstaculo medio MIENTRAS /cmd_vel puede estar moviendo.
        spawn('a2_r19_medio',.56)
        t1=time.monotonic()
        stage('Conmutacion en movimiento: RUTA a DIFUSO',
              lambda:positive_mode('DIFUSO',t0,2) and
                     str(data['state']).startswith('DIFUSO|PROPUESTA_DIFUSO_VALIDA'),8)
        # Insercion cercana durante DIFUSO; supervisor + barrera detienen.
        spawn('a2_r19_cerca',.34)
        stage('Parada estricta por LiDAR',
              lambda:str(data['state']).startswith('DETENIDO|PARADA_SCAN_ESTRICTA')
                     and latest_decision[0]=='CERO' and data['scan'] is not None
                     and data['scan']<=.35,8)
        hold(.8)
        pose_stop=data['pose']
        hold(.5)
        require(pose_stop is not None and data['pose'] is not None and
                math.dist(pose_stop,data['pose'])<.020,
                'Frenado estable durante parada estricta (<2 cm)')

        # Reactivar tras retirar obstaculo proximo, sin reiniciar Gazebo.
        delete('a2_r19_cerca')
        arm_at=time.monotonic();phase='ACTIVO';t2=arm_at
        stage('Recuperacion DIFUSO con movimiento',
              lambda:positive_mode('DIFUSO',t2,2) and
                     str(data['state']).startswith('DIFUSO|PROPUESTA_DIFUSO_VALIDA'),8)
        hold(.25)
        delete('a2_r19_medio')
        arm_at=time.monotonic();phase='ACTIVO';t3=arm_at
        stage('Retorno a RUTA con movimiento',
              lambda:positive_mode('RUTA',t3,2) and
                     data['state']=='RUTA|PROPUESTA_RUTA_VALIDA',8)
        hold(.45)

        # Falla real de una propuesta: no se publica /cmd_vel_ruta durante
        # >0.4 s; arbitro debe entrar en DETENIDO y gateway sigue en CERO.
        phase='DROP_RUTA';t_drop=time.monotonic()
        stage('Fail-safe por perdida de propuesta RUTA',
              lambda:time.monotonic()-t_drop>.42 and
                     str(data['state']).startswith(
                         'DETENIDO|PROPUESTA_RUTA_AUSENTE_O_CADUCADA')
                     and latest_decision[0]=='CERO',5)
        hold(.35)
        stage('Recuperacion de propuesta sin movimiento',
              lambda:data['state']=='RUTA|PROPUESTA_RUTA_VALIDA'
                     and latest_decision[0]=='CERO',5)
        hold(1.25)

        require(len(stages)==7,'Siete etapas integradas observadas')
        require(command_sent['RUTA']>=3 and command_sent['DIFUSO']>=3,
                'Comandos RUTA y DIFUSO transmitidos por la pasarela')
        require(sum(m=='RUTA' for _,m,v,w in recv_active_modes if v>0.0005)>=3 and
                sum(m=='DIFUSO' for _,m,v,w in recv_active_modes if v>0.0005)>=2,
                'Ordenes NO NULAS recibidas desde ambos modos, cotejadas envio-recepcion')
        require(len(pending_commands)<=3,'Sin retrasos anormales en cola /cmd_vel')
        require(command_recv['CERO']>=12,'Freno a cero mantenido al menos 12 ciclos')
        require(data['pose'] is not None and initial_pose is not None,
                'Odometria final disponible')
        net=math.dist(initial_pose,data['pose'])
        say(f'[MEDICION] RUTA emitidos={command_sent["RUTA"]}; '
            f'DIFUSO emitidos={command_sent["DIFUSO"]}; '
            f'ceros={command_sent["CERO"]}; recorrido aprox={path_m:.4f} m; '
            f'desplazamiento neto={net:.4f} m')
        require(.008<=net<=.19 and path_m<=.35,
                'Movimiento observable dentro de limite global')
        require(node.count_publishers('/cmd_vel')==1,'Canal /cmd_vel exclusivo al cierre')
        require(not faults,'Sin violaciones de seguridad')
        status='APROBADA_PREVALIDACION_EXPERIMENTAL';return_code=0
        say('=== R19: BATERIA INTEGRAL GAZEBO APROBADA; UMBRALES OFICIALES PENDIENTES ===')
    except Exception as exc:
        phase='HOLD';say('[ERROR R19 INTEGRAL] '+str(exc))
    finally:
        phase='HOLD'
        if rclpy.ok():
            end=time.monotonic()+1.3
            while time.monotonic()<end:
                try:pump()
                except Exception:pass
        if delete_client is not None and rclpy.ok():
            for name in reversed(created[:]):
                try:delete(name)
                except Exception as e:say('[WARN] Limpieza '+name+': '+str(e))
        net=(math.dist(initial_pose,data['pose'])
             if initial_pose is not None and data['pose'] is not None else None)
        payload={'etapa':'R19_BANCO_INTEGRAL','estado':status,
            'repo_commit_base':EXPECTED,'solo_gazebo':True,'r19_oficial_acreditado':False,
            'umbrales_provisionales':{'stop_m':.35,'fuzzy_in_m':.60,'route_out_m':.80},
            'limites_pasarela':{'ruta_v_mps':.04,'difuso_v_mps':.025,'omega_rps':.20,
                               'frontal_guard_scan_m':.43,'desplazamiento_max_m':.18},
            'etapas':stages,'mensajes_por_topico':counts,
            'comandos_enviados':command_sent,'comandos_recibidos':command_recv,
            'movimientos_transmitidos':len(active_cmd_modes),
            'movimientos_recibidos':len(recv_active_modes),
            'cola_pendiente_cmd_vel':len(pending_commands),
            'desplazamiento_neto_m':net,'recorrido_aproximado_m':path_m,
            'pose_inicial':initial_pose,'pose_final':data['pose'],
            'fallos':faults,'nota':'Prueba en Gazebo con un unico /cmd_vel; '
                'los valores 0.60/0.80 no estan confirmados por la rubrica oficial.'}
        folder.mkdir(parents=True,exist_ok=True)
        (folder/'resumen.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
        with (folder/'observaciones.csv').open('w',encoding='utf8',newline='') as f:
            w=csv.DictWriter(f,fieldnames=['t_s','fase','topico','valor','scan_m',
                'range_m','x_m','y_m','estado','modo_emitido'],lineterminator='\n')
            w.writeheader();w.writerows(events)
        say('[EVIDENCIA] '+str(folder/'resumen.json'))
        say('[EVIDENCIA] '+str(folder/'observaciones.csv'))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return return_code


def stop_own(proc):
    if proc is None or proc.poll() is not None:return
    for sig,wait in ((signal.SIGINT,12),(signal.SIGTERM,8),(signal.SIGKILL,3)):
        if proc.poll() is not None:break
        try:os.killpg(proc.pid,sig)
        except ProcessLookupError:break
        try:proc.wait(timeout=wait)
        except subprocess.TimeoutExpired:pass


def run():
    self_test()
    preflight()
    e=env()
    rc=subprocess.run(child_cmd(['--probe']),cwd=REPO,env=e,timeout=45).returncode
    require(rc==0,'Dominio aislado disponible')
    stamp=dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    folder=REPO/'resultados/A2_R19_integral_gazebo'/stamp
    folder.mkdir(parents=True,exist_ok=False)
    launch=folder/'almacen_r19_integral.launch.py'
    compile(LAUNCH,str(launch),'exec')
    launch.write_text(LAUNCH,encoding='utf8',newline='\n')
    say('[INFO] Lanzador temporal: '+str(launch))
    out_server=(folder/'gzserver_factory.log').open('w',encoding='utf8')
    out_factory=(folder/'factory_probe.log').open('w',encoding='utf8')
    out_launch=(folder/'gazebo_lanzamiento.log').open('w',encoding='utf8')
    out_mon=(folder/'monitor.log').open('w',encoding='utf8')
    server=factory=p=mon=None
    try:
        server_cmd=['bash','-c',
            'source "$1" >/dev/null && exec gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$2"',
            'bash',str(ACTIVATE),str(WORLD)]
        server=subprocess.Popen(server_cmd,cwd=REPO,env=e,stdout=out_server,
                stderr=subprocess.STDOUT,start_new_session=True)
        say('[INFO] gzserver headless iniciado. Esperando ROS Factory...')
        factory=subprocess.Popen(child_cmd(['--factory-probe']),cwd=REPO,env=e,
                stdout=out_factory,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+FACTORY_WAIT_S+8
        while factory.poll() is None:
            if server.poll() is not None:raise RuntimeError('gzserver termino durante Factory')
            if time.monotonic()>deadline:raise RuntimeError('Timeout ROS Factory')
            time.sleep(.3)
        for line in (folder/'factory_probe.log').read_text(encoding='utf8',errors='replace').splitlines():
            say(line)
        require(factory.returncode==0 and server.poll() is None,
                'ROS Factory disponible; insertar robot autorizado')
        cmd=['bash','-c','source "$1" >/dev/null && exec ros2 launch "$2"',
             'bash',str(ACTIVATE),str(launch)]
        p=subprocess.Popen(cmd,cwd=REPO,env=e,stdout=out_launch,
                stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+56
        while time.monotonic()<deadline:
            if server.poll() is not None:raise RuntimeError('gzserver termino al insertar robot')
            if p.poll() is not None:raise RuntimeError('Launch termino al insertar robot')
            log=(folder/'gazebo_lanzamiento.log').read_text(encoding='utf8',errors='replace')
            if 'Successfully spawned entity [waffle_pi]' in log:break
            if 'Spawn service failed' in log or 'Service /spawn_entity unavailable' in log:
                raise RuntimeError('No se inserto robot; se aborta antes de movimiento')
            time.sleep(.3)
        else:raise RuntimeError('Timeout insercion TurtleBot3')
        require(True,'TurtleBot3 insertado correctamente')
        mon=subprocess.Popen(child_cmd(['--monitor','--out',str(folder)]),cwd=REPO,
                env=e,stdout=out_mon,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+DISCOVERY_S+MAX_GLOBAL_S+35
        last_msg=0.
        while mon.poll() is None:
            if server.poll() is not None:raise RuntimeError('gzserver termino durante ensayo')
            if p.poll() is not None:raise RuntimeError('Launch termino durante ensayo')
            if time.monotonic()>deadline:raise RuntimeError('Timeout global monitor')
            if time.monotonic()-last_msg>15:
                say('[INFO] Banco R19 ejecutandose; evidencias en '+str(folder))
                last_msg=time.monotonic()
            time.sleep(.4)
        out_mon.flush()
        for line in (folder/'monitor.log').read_text(encoding='utf8',errors='replace').splitlines()[-60:]:
            say(line)
        if mon.returncode!=0:raise RuntimeError('Banco R19 NO aprobado; monitor codigo '+str(mon.returncode))
        return 0
    except Exception as exc:
        for h in (out_server,out_factory,out_launch,out_mon):h.flush()
        if (folder/'monitor.log').is_file():
            for line in (folder/'monitor.log').read_text(encoding='utf8',errors='replace').splitlines()[-20:]:
                say('[MONITOR] '+line)
        for name in ('gazebo_lanzamiento.log','gzserver_factory.log'):
            fp=folder/name
            if fp.is_file():
                for line in fp.read_text(encoding='utf8',errors='replace').splitlines()[-10:]:
                    say('[LOG '+name+'] '+line)
        raise RuntimeError(str(exc))
    finally:
        if mon is not None and mon.poll() is None:
            mon.terminate()
            try:mon.wait(timeout=5)
            except subprocess.TimeoutExpired:mon.kill();mon.wait(timeout=5)
        stop_own(p);stop_own(factory);stop_own(server)
        for h in (out_server,out_factory,out_launch,out_mon):h.close()
        say('[OK] Procesos creados por el ensayo cerrados; no se intervino Git.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test',action='store_true',help='Solo pruebas puras, sin ROS')
    parser.add_argument('--probe',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--factory-probe',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--monitor',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--out',help=argparse.SUPPRESS)
    args=parser.parse_args()
    try:
        if args.self_test:return self_test()
        if args.probe:return probe()
        if args.factory_probe:return factory_probe()
        if args.monitor:
            require(bool(args.out),'Carpeta --out obligatoria')
            return monitor(Path(args.out))
        return run()
    except KeyboardInterrupt:
        say('[INTERRUMPIDO] Emision de comandos finalizada; cierre seguro.')
        return 130
    except Exception as exc:
        say('[ERROR/PENDIENTE R19] '+str(exc))
        say('[NOTA] Sin cambios a FIS, supervisor original, launch ni Git; R19 oficial pendiente.')
        return 1


if __name__=='__main__':
    sys.exit(main())
