#!/usr/bin/env python3
"""AMR Logistico / E2.3.3 - candidato de supervisor integrado en banco ROS aislado.

EJECUCION EN WSL:
    python3 /mnt/c/Users/Henry/Downloads/A2_E233_supervisor_integrado_aislado.py --instalar-y-probar

Seguridad:
- NO reemplaza tools/supervisor_f13.py, ni edita launch.
- NO publica /cmd_vel. Publica SOLO /cmd_vel_integrado_prueba y /estado_integrado_prueba.
- ROS_DOMAIN_ID dedicado y ROS_LOCALHOST_ONLY=1 (sin Gazebo).
- Prueba seguridad adicional basada en LaserScan, independiente de tres Range.
- Los umbrales de histéresis .60/.80 m son temporales: NO validación final R19.
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
import shlex
import shutil
import subprocess
import sys
import time

REPO = Path.home() / 'AMR_Logistico_Fase_0'
SELF_NAME = 'A2_E233_supervisor_integrado_aislado.py'
TARGET = REPO / 'tools' / SELF_NAME
SHADOW = REPO / 'tools/A2_E23_arbitraje_sombra.py'
NODE_NAME = 'supervisor_integrado_e233_prueba'
OUTPUT = '/cmd_vel_integrado_prueba'
STATUS = '/estado_integrado_prueba'
RANGES = {'df': '/rango_frontal', 'di': '/rango_izq', 'dd': '/rango_der'}
SCAN_MAX_AGE_S = 1.5
SCAN_STOP_M = .35
SCAN_HALF_FOV = math.radians(25.)
PERIOD_S = .1
INPUT_PERIOD = .075
VENV_PY = Path.home() / '.venvs/amr-a2-r18/bin/python'


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def file_sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_repository():
    require((REPO / '.git').is_dir(), f'No existe el repositorio: {REPO}')
    require(SHADOW.is_file(), 'Falta tools/A2_E23_arbitraje_sombra.py (E2.3.1)')
    require((REPO / 'tools/supervisor_f13.py').is_file(), 'Falta supervisor original F1.3')
    branch = subprocess.run(['git','branch','--show-current'],cwd=REPO,
                            text=True,capture_output=True,check=True).stdout.strip()
    require(branch == 'desarrollo-avance2', f'Rama inesperada: {branch}')


def install():
    check_repository()
    source = Path(__file__).resolve()
    if source == TARGET.resolve():
        print('[OK] Candidato ya instalado en tools/', flush=True)
        return
    if TARGET.exists() and file_sha(source) != file_sha(TARGET):
        raise RuntimeError('Existe un candidato E2.3.3 diferente. No se sobrescribe.')
    TARGET.parent.mkdir(parents=True,exist_ok=True)
    if not TARGET.exists():
        shutil.copy2(source,TARGET)
    print(f'[OK] Candidato instalado: {TARGET}',flush=True)


def pure_preflight():
    import importlib.util
    check_repository()
    spec = importlib.util.spec_from_file_location('amr_shadow_e233', SHADOW)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    cfg=mod.Config(); cfg.validate()
    require((cfg.stop_m,cfg.enter_fuzzy_m,cfg.exit_fuzzy_m)==(.35,.60,.80),
            'El contrato de umbrales cambió: es necesario revisar E2.3.3')
    require(cfg.max_v_mps==.12 and cfg.max_w_rps==.70, 'Límites inesperados')
    a=mod.ShadowArbiter()
    r=a.decide(mod.fixture(front=.55),100.)
    require((r.mode,round(r.v,6))==('DIFUSO',.12),'Error arbitraje E2.3.1')
    print('[OK] Contrato previo E2.3.1: histéresis y saturaciones verificadas.',flush=True)


def import_shadow():
    pure_preflight()
    import importlib.util
    spec=importlib.util.spec_from_file_location('amr_shadow_e233_runtime',SHADOW)
    mod=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=mod
    spec.loader.exec_module(mod)
    return mod


def lidar_min_front(scan):
    """Retorna distancia mínima frontal o None si no hay rayos útiles.

    +inf => no retorno, sustituir por range_max como supervisor F1.3.
    NaN o -inf no son rayos útiles; si no queda ninguno, fail-safe.
    """
    if (not math.isfinite(scan.angle_min) or
        not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0 or
        not math.isfinite(scan.range_min) or not math.isfinite(scan.range_max) or
        not (0 <= scan.range_min < scan.range_max)):
        return None
    vals=[]
    for i,r in enumerate(scan.ranges):
        a=scan.angle_min+i*scan.angle_increment
        a=math.atan2(math.sin(a),math.cos(a))
        if abs(a)>SCAN_HALF_FOV:
            continue
        if r==math.inf:
            r=scan.range_max
        if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
            vals.append(float(r))
    return min(vals) if vals else None


def integrated_decision(engine, frame, now, last_scan, scan_distance):
    """Barrera de LiDAR + arbitraje Range, no produce publicaciones."""
    Decision = sys.modules[engine.__class__.__module__].Decision  # Sin mutar histéresis al consultar tipo
    if (last_scan is None or not math.isfinite(last_scan) or
        not 0 <= now-last_scan <= SCAN_MAX_AGE_S):
        engine.previous_mode='DETENIDO'
        return Decision('DETENIDO','SCAN_AUSENTE_O_CADUCADO')
    if scan_distance is None or not math.isfinite(scan_distance):
        engine.previous_mode='DETENIDO'
        return Decision('DETENIDO','SCAN_INVALIDO')
    if scan_distance <= SCAN_STOP_M:
        engine.previous_mode='DETENIDO'
        return Decision('DETENIDO','PARADA_SCAN_ESTRICTA')
    return engine.decide(frame, now)


def run_node():
    require(os.environ.get('_A2_E233_CHILD')=='1','Nodo solo permitido desde banco de pruebas aislado')
    require(os.environ.get('ROS_LOCALHOST_ONLY')=='1','ROS_LOCALHOST_ONLY debe ser 1')
    domain=int(os.environ.get('ROS_DOMAIN_ID','-1'))
    require(200 <= domain <= 224,'Dominio fuera de ventana aislada')
    mod=import_shadow()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, Range
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    class SupervisorIntegradoPrueba(Node):
        def __init__(self):
            super().__init__(NODE_NAME)
            self.engine=mod.ShadowArbiter(mod.Config())
            self.readings={k:None for k in RANGES}
            self.odom_received=None
            self.proposals={'RUTA':None,'DIFUSO':None}
            self.t_scan=None
            self.dist_scan=None
            self.pub=self.create_publisher(Twist,OUTPUT,10)
            self.pub_status=self.create_publisher(String,STATUS,10)
            self.subs=[]
            for key,topic in RANGES.items():
                self.subs.append(self.create_subscription(
                    Range,topic,lambda m,k=key:self.on_range(k,m),10))
            self.subs.append(self.create_subscription(Odometry,'/odom',self.on_odom,10))
            self.subs.append(self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data))
            self.subs.append(self.create_subscription(
                Twist,'/cmd_vel_ruta',lambda m:self.on_twist('RUTA',m),10))
            self.subs.append(self.create_subscription(
                Twist,'/cmd_vel_difuso',lambda m:self.on_twist('DIFUSO',m),10))
            self.timer=self.create_timer(PERIOD_S,self.tick)
            self.last_status=None
            self.get_logger().info('Supervisor de prueba: publica SOLO '+OUTPUT)

        def on_range(self,k,msg):
            now=time.monotonic()
            v,lo,hi=float(msg.range),float(msg.min_range),float(msg.max_range)
            valid=(all(math.isfinite(x) for x in (v,lo,hi))
                   and 0 <= lo <= hi <= 2 and lo <= v <= hi)
            self.readings[k]=mod.Reading(v if valid else math.nan,now)

        def on_odom(self,msg):
            p=msg.pose.pose.position; q=msg.pose.pose.orientation
            vals=(p.x,p.y,p.z,q.x,q.y,q.z,q.w)
            self.odom_received=time.monotonic() if all(math.isfinite(x) for x in vals) else None

        def on_scan(self,msg):
            self.dist_scan=lidar_min_front(msg)
            self.t_scan=time.monotonic()

        def on_twist(self,mode,msg):
            vals=(msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)
            v,w=float(msg.linear.x),float(msg.angular.z)
            if any(not math.isfinite(x) or abs(x)>1e-6 for x in vals):
                v=w=math.nan
            self.proposals[mode]=mod.Proposal(v,w,time.monotonic())

        def tick(self):
            now=time.monotonic()
            frame=mod.Frame(self.readings['df'],self.readings['di'],self.readings['dd'],
                            self.odom_received,self.proposals['RUTA'],self.proposals['DIFUSO'])
            decision=integrated_decision(self.engine,frame,now,self.t_scan,self.dist_scan)
            msg=Twist()
            msg.linear.x=decision.v
            msg.angular.z=decision.w
            self.pub.publish(msg)
            state=decision.mode+'|'+decision.reason
            self.pub_status.publish(String(data=state))
            if state!=self.last_status:
                self.get_logger().info(f'{state}: v={decision.v:.3f} w={decision.w:.3f}')
                self.last_status=state

    rclpy.init()
    node=None
    try:
        node=SupervisorIntegradoPrueba()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            for _ in range(4):
                node.pub.publish(Twist())
                node.pub_status.publish(String(data='DETENIDO|SALIDA'))
                time.sleep(.025)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def bench():
    check_repository()
    require(os.environ.get('ROS_LOCALHOST_ONLY')=='1','Banco sin aislamiento ROS')
    domain=int(os.environ.get('ROS_DOMAIN_ID','-1'))
    require(200<=domain<=224,'Dominio incorrecto')
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan,Range
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    stamp=dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    folder=REPO/'resultados/A2_E233_supervisor_integrado_aislado'/stamp
    i=1
    while folder.exists():
        folder=folder.parent/f'{stamp}_{i:02d}';i+=1
    folder.mkdir(parents=True)
    logpath=folder/'nodo_integrado_prueba.log'
    reportpath=folder/'resumen.json'
    csvpath=folder/'observaciones.csv'
    statuses=[]; speeds=[]; obs=[]; checks=[]
    child=None; handle=None; node=None; initialized=False; success=False
    t0=time.monotonic()
    print('=== E2.3.3 - SUPERVISOR CANDIDATO ROS 2 AISLADO ===',flush=True)
    print(f'[INFO] Dominio ROS_DOMAIN_ID={domain}, ROS_LOCALHOST_ONLY=1',flush=True)
    print('[INFO] Se auditan LaserScan + Range + propuestas; NO Gazebo y NO /cmd_vel.',flush=True)
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        initialized=True
        node=Node('banco_integrado_e233')
        pubs={k:node.create_publisher(Range,t,10) for k,t in RANGES.items()}
        p_odom=node.create_publisher(Odometry,'/odom',10)
        p_scan=node.create_publisher(LaserScan,'/scan',10)
        p_route=node.create_publisher(Twist,'/cmd_vel_ruta',10)
        p_fuzzy=node.create_publisher(Twist,'/cmd_vel_difuso',10)
        node.create_subscription(Twist,OUTPUT,lambda m:speeds.append((time.monotonic(),float(m.linear.x),float(m.angular.z))),10)
        node.create_subscription(String,STATUS,lambda m:statuses.append((time.monotonic(),m.data)),10)
        env=os.environ.copy();env['_A2_E233_CHILD']='1'
        handle=logpath.open('w',encoding='utf8')
        child=subprocess.Popen([str(VENV_PY),'-u',str(TARGET),'--nodo-aislado'],
                               cwd=REPO,env=env,stdout=handle,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.045)
            if child.poll() is not None:
                raise RuntimeError('El candidato terminó en descubrimiento. Revisar nodo_integrado_prueba.log')
            if (NODE_NAME in node.get_node_names() and
                all(node.count_subscribers(t)>=1 for t in RANGES.values()) and
                all(node.count_subscribers(t)>=1 for t in
                    ('/scan','/odom','/cmd_vel_ruta','/cmd_vel_difuso')) and
                node.count_publishers(OUTPUT)==1 and node.count_publishers(STATUS)==1):
                break
        else:
            raise RuntimeError('No se descubrieron las nueve interfaces')
        require(node.count_publishers('/cmd_vel')==0,'Publicador no autorizado de /cmd_vel detectado')
        print('[OK] 9 interfaces ROS y aislamiento /cmd_vel verificados.',flush=True)
        checks.append({'caso':'Descubrimiento y aislamiento','aprobado':True})

        def send_range(k,v):
            m=Range();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='base_scan'
            m.radiation_type=Range.INFRARED;m.field_of_view=math.radians(15)
            m.min_range=0.;m.max_range=2.;m.range=float(v)
            pubs[k].publish(m)

        def send_odom():
            m=Odometry();m.header.stamp=node.get_clock().now().to_msg()
            m.header.frame_id='odom';m.child_frame_id='base_footprint'
            m.pose.pose.orientation.w=1.
            p_odom.publish(m)

        def send_twist(pub,v,w,extra=0):
            m=Twist();m.linear.x=float(v);m.linear.y=float(extra);m.angular.z=float(w)
            pub.publish(m)

        def send_scan(front,invalid=False):
            m=LaserScan();m.header.stamp=node.get_clock().now().to_msg()
            m.header.frame_id='base_scan'
            m.angle_min=-math.pi/2;m.angle_increment=math.radians(5)
            m.angle_max=math.pi/2;m.range_min=.12;m.range_max=3.5
            m.ranges=[math.nan if invalid else 1.7]*37
            if not invalid:
                m.ranges[18]=float(front)
            p_scan.publish(m)

        def phase(label,seconds,*,d=1.7,left=1.7,right=1.7,
                  scan=1.7,invalid_scan=False,odom=True,route=(.10,0.),
                  fuzzy=(.16,.2),lateral=0.,ranges=True):
            begin=time.monotonic();next_send=begin
            while time.monotonic()-begin<seconds:
                now=time.monotonic()
                if now>=next_send:
                    if ranges:
                        for k,v in (('df',d),('di',left),('dd',right)):
                            send_range(k,v)
                    if scan is not None:
                        send_scan(scan,invalid_scan)
                    if odom:send_odom()
                    if route is not None:send_twist(p_route,*route,extra=lateral)
                    if fuzzy is not None:send_twist(p_fuzzy,*fuzzy)
                    next_send=now+INPUT_PERIOD
                rclpy.spin_once(node,timeout_sec=.035)
                if child.poll() is not None:
                    raise RuntimeError(f'El candidato terminó en {label}. Consultar {logpath}')
            trim=time.monotonic()-min(.25,seconds*.25)
            s=[v for t,v in statuses if t>=trim]
            v=[(a,b) for t,a,b in speeds if t>=trim]
            require(len(s)>=2 and len(v)>=2,f'{label}: muestras insuficientes')
            return s,v

        def expect(name,data,mode,reason,v,w,tol=1e-4):
            s,vel=data
            expected=mode+'|'+reason
            passed=all(x==expected for x in s) and all(abs(a-v)<=tol and abs(b-w)<=tol for a,b in vel)
            checks.append({'caso':name,'aprobado':bool(passed),'estado':s[-1],
                           'esperado':expected,'v':vel[-1][0],'omega':vel[-1][1]})
            for x in s:obs.append({'caso':name,'tipo':'estado','valor':x})
            for a,b in vel:obs.append({'caso':name,'tipo':'twist','valor':f'v={a:.6f};w={b:.6f}'})
            print(('[OK] ' if passed else '[ERROR] ')+name+': '+s[-1]+f' / v={vel[-1][0]:.3f} w={vel[-1][1]:+.3f}',flush=True)
            require(passed,f'{name}: esperado {expected} ({v},{w})')
            require(node.count_publishers('/cmd_vel')==0,'Se detectó un publicador /cmd_vel')

        expect('Sin datos: parada',phase('sin datos',.75,scan=None,odom=False,route=None,fuzzy=None,ranges=False),
               'DETENIDO','SCAN_AUSENTE_O_CADUCADO',0,0)
        expect('Range sin LiDAR: parada',phase('sin LiDAR',.75,scan=None),
               'DETENIDO','SCAN_AUSENTE_O_CADUCADO',0,0)
        expect('Pasillo libre: ruta',phase('ruta',.85),'RUTA','PROPUESTA_RUTA_VALIDA',.10,0)
        expect('Distancia .55: difuso',phase('difuso',.85,d=.55),'DIFUSO','PROPUESTA_DIFUSO_VALIDA',.12,.2)
        expect('Histeresis .70: conserva difuso',phase('histeresis',.85,d=.70),
               'DIFUSO','PROPUESTA_DIFUSO_VALIDA',.12,.2)
        expect('Retorno .80: ruta',phase('retorno',.85,d=.80),'RUTA','PROPUESTA_RUTA_VALIDA',.10,0)
        expect('Parada Range exactamente .35',phase('range limite',.85,d=.35),
               'DETENIDO','PARADA_FRONTAL_ESTRICTA',0,0)
        expect('LiDAR peligro aunque Range libre',phase('scan contradice',.85,d=1.7,scan=.30),
               'DETENIDO','PARADA_SCAN_ESTRICTA',0,0)
        expect('Parada LiDAR exactamente .35',phase('scan limite',.85,d=1.7,scan=.35),
               'DETENIDO','PARADA_SCAN_ESTRICTA',0,0)
        expect('LiDAR invalido: parada',phase('scan invalido',.85,invalid_scan=True),
               'DETENIDO','SCAN_INVALIDO',0,0)
        expect('LiDAR caducado: parada',phase('scan caducado',1.90,scan=None),
               'DETENIDO','SCAN_AUSENTE_O_CADUCADO',0,0)
        expect('Range caducado: parada',phase('range caducado',1.15,ranges=False),
               'DETENIDO','CADUCADO_df',0,0)
        expect('Odometria caducada: parada',phase('odom caducada',1.90,odom=False),
               'DETENIDO','ODOMETRIA_AUSENTE_O_CADUCADA',0,0)
        expect('Propuesta difusa caducada',phase('difuso caducado',.85,d=.55,fuzzy=None),
               'DETENIDO','PROPUESTA_DIFUSO_AUSENTE_O_CADUCADA',0,0)
        expect('NaN frontal Range: parada',phase('nan range',.85,d=math.nan),
               'DETENIDO','INVALIDO_df',0,0)
        expect('Recuperacion tras NaN',phase('recuperacion',.85,d=.55),
               'DIFUSO','PROPUESTA_DIFUSO_VALIDA',.12,.2)
        expect('Twist lateral no autorizado',phase('twist lateral',.85,d=1.7,lateral=.05),
               'DETENIDO','PROPUESTA_RUTA_INVALIDA',0,0)
        expect('Saturacion de limites',phase('saturacion',.85,d=.55,fuzzy=(.19,1.3)),
               'DIFUSO','PROPUESTA_DIFUSO_VALIDA',.12,.70)
        require(node.count_publishers('/cmd_vel')==0,'Publicador /cmd_vel detectado al final')
        checks.append({'caso':'/cmd_vel sin publicadores','aprobado':True})
        require(len(checks)==20 and all(x['aprobado'] for x in checks),'Recuento o fallos inesperados')
        success=True
        print('[RESULTADO] 20/20 verificaciones aprobadas',flush=True)
        print('=== E2.3.3: SUPERVISOR INTEGRADO AISLADO APROBADO ===',flush=True)
        return 0
    except Exception as exc:
        print('[ERROR E2.3.3]',str(exc),flush=True)
        return 1
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=4)
            except subprocess.TimeoutExpired:
                child.kill();child.wait(timeout=3)
        if handle is not None:handle.close()
        if node is not None:node.destroy_node()
        if initialized and rclpy.ok():rclpy.shutdown()
        meta={'estado':'APROBADO' if success else 'NO_APROBADO',
              'fecha_local':dt.datetime.now().isoformat(timespec='seconds'),
              'duracion_s':round(time.monotonic()-t0,3),
              'dominio_ros':domain,
              'umbrales':{'parada_m':.35,'ingreso_difuso_m':.60,'retorno_ruta_m':.80},
              'restricciones':'NO Gazebo; NO /cmd_vel; NO modificación supervisor_f13.py; ensayos sintéticos',
              'alcance':'Preintegración ROS 2; no acredita aún el despliegue operativo R19',
              'pruebas':checks}
        reportpath.write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
        with csvpath.open('w',newline='',encoding='utf8') as f:
            writer=csv.DictWriter(f,fieldnames=['caso','tipo','valor'],lineterminator='\n')
            writer.writeheader();writer.writerows(obs)
        print('[EVIDENCIA]',reportpath,flush=True)
        print('[EVIDENCIA]',csvpath,flush=True)
        print('[LOG]',logpath,flush=True)


def prepare_ros():
    install()
    require(VENV_PY.is_file(),'Falta el entorno Python ~/.venvs/amr-a2-r18')
    pure_preflight()
    # Elegir dominio aleatorio libre NO equivale a descubrir procesos existentes;
    # el banco exige que /cmd_vel permanezca sin publicadores.
    domain=random.SystemRandom().randint(200,224)
    env=os.environ.copy()
    env['ROS_DOMAIN_ID']=str(domain)
    env['ROS_LOCALHOST_ONLY']='1'
    env['_A2_E233_CHILD']='1'
    shell='source /opt/ros/humble/setup.bash && source '+shlex.quote(str(REPO/'tools/activar_entorno.sh')) + ' && exec '+shlex.quote(str(VENV_PY))+' -u '+shlex.quote(str(TARGET))+' --banco'
    print('[INFO] Activando ROS 2 Humble y Python R18...',flush=True)
    completed=subprocess.run(['bash','-c',shell],env=env,cwd=REPO,check=False)
    return completed.returncode


def main():
    p=argparse.ArgumentParser(description=__doc__)
    group=p.add_mutually_exclusive_group()
    group.add_argument('--instalar-y-probar',action='store_true')
    group.add_argument('--nodo-aislado',action='store_true',help=argparse.SUPPRESS)
    group.add_argument('--banco',action='store_true',help=argparse.SUPPRESS)
    args=p.parse_args()
    if args.nodo_aislado:
        run_node();return 0
    if args.banco:
        return bench()
    return prepare_ros()


if __name__=='__main__':
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except Exception as exc:
        print('[ERROR E2.3.3]',str(exc),flush=True)
        print('[INFO] No se tocó supervisor_f13.py ni se publicó en /cmd_vel.',flush=True)
        sys.exit(1)
