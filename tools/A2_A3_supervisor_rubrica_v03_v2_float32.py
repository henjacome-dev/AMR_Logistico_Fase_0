#!/usr/bin/env python3
"""AMR Logistico - A3: supervisor de conmutacion conforme al enunciado oficial.

Proteccion: NO escribe /cmd_vel; en A3 usa solo /cmd_vel_a3_prueba.
No reemplaza supervisor_f13.py ni los nodos E2.3 historicos.

Uso WSL2:
 python3 /mnt/c/Users/Henry/Downloads/A2_A3_supervisor_rubrica_v03_v2_float32.py --instalar-y-probar

A3 corrige comparacion float32 de umbrales en ROS sensor_msgs/Range.
A3 valida solo la conmutacion y el fail-safe, NO la reincorporacion A4,
la navegacion Gazebo B1-B4 ni el controlador ROS que debe migrarse al FIS v03.
"""
from __future__ import annotations
import argparse
from dataclasses import dataclass
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import time

ROOT = Path.home() / 'AMR_Logistico_Fase_0'
TARGET = ROOT / 'tools/A2_A3_supervisor_rubrica_v03_v2_float32.py'
EXPECTED_HEAD = 'a604f02dd89c03145d3c253693c656aca47ae1e7'
V01 = ROOT / 'matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis'
V01_BLOB = '6c7dfdd083728d49c64dd1c90a59b19d63222063'
V03 = ROOT / 'matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis'
V03_SHA256 = '67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903'
OUTPUT_DIR = ROOT / 'resultados/A2_A3_supervisor_rubrica_v03_v2_float32'
RANGE_TOPICS = {'df':'/rango_frontal','di':'/rango_izq','dd':'/rango_der'}
OUT = '/cmd_vel_a3_prueba'
STATUS = '/estado_a3_prueba'

# Los 0.60/0.35 m provienen del enunciado, sin histéresis experimental de 0.80.
FRONTAL_RUTA = 0.60
LATERAL_RUTA = 0.35
# Barreras conservadoras de seguridad (decisiones adicionales de ingeniería).
FRONTAL_STOP = 0.30
LATERAL_STOP = 0.16
SCAN_STOP = 0.30
SENSOR_AGE = 0.80
SCAN_AGE = 0.80
ODOM_AGE = 1.50
PROPOSAL_AGE = 0.45
VMAX_RUTA = 0.12
VMAX_DIFUSO = 0.10
WMAX = 0.70


def check(ok: bool, msg: str) -> None:
    if not ok: raise RuntimeError(msg)


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def git_blob(p: Path) -> str:
    raw=p.read_bytes()
    return hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()


def git(*args: str) -> str:
    p=subprocess.run(['git',*args],cwd=ROOT,text=True,capture_output=True,timeout=30)
    check(p.returncode==0,f'Git {args}: {p.stderr.strip()}')
    return p.stdout.strip()


def preflight() -> None:
    check((ROOT/'.git').is_dir(),'No se encuentra el repositorio')
    check(git('branch','--show-current')=='desarrollo-avance2','Rama distinta de desarrollo-avance2')
    check(git('rev-parse','HEAD')==EXPECTED_HEAD,'HEAD cambió. Auditar antes de instalar A3')
    check(V01.is_file() and git_blob(V01)==V01_BLOB,'FIS v01 alterado; se aborta')
    check(V03.is_file() and sha256(V03)==V03_SHA256,'FIS v03 alterado o ausente; se aborta')
    check((ROOT/'tools/supervisor_f13.py').is_file(),'Supervisor histórico ausente')
    check((ROOT/'tools/A2_E233_supervisor_integrado_aislado.py').is_file(),'Supervisor experimental histórico ausente')
    print('[OK] Rama, HEAD, FIS v01/v03 y supervisor histórico protegidos',flush=True)


@dataclass(frozen=True)
class Reading:
    value: float
    stamp: float


@dataclass(frozen=True)
class Proposal:
    v: float
    w: float
    stamp: float


@dataclass(frozen=True)
class Frame:
    df: Reading | None
    di: Reading | None
    dd: Reading | None
    scan: Reading | None
    odom_stamp: float | None
    route: Proposal | None
    fuzzy: Proposal | None


@dataclass(frozen=True)
class Decision:
    mode: str
    reason: str
    v: float=0.0
    w: float=0.0


def fresh(now: float,t:float | None,limit:float)->bool:
    return t is not None and math.isfinite(t) and 0 <= now-t <= limit

def to_float32(value: float) -> float:
    """Normaliza a IEEE-754 binary32: tipo del campo Range.range de ROS 2.

    Ejemplo: 0.60 -> 0.6000000238418579. Normalizar *tambien* el
    umbral evita declarar RUTA cuando la distancia transmitida es 0.60 m.
    En fronteras la decision es conservadora: igualdad -> DIFUSO/CERO.
    """
    return struct.unpack('!f',struct.pack('!f',float(value)))[0]


def at_or_below(value: float,threshold: float) -> bool:
    """Comparacion de umbral inclusivo, coherente con sensor_msgs/Range."""
    return to_float32(value) <= to_float32(threshold)



def decide(f: Frame,now:float)->Decision:
    """Función sin efectos ROS; prioridad fail-safe > DIFUSO > RUTA."""
    def zero(reason):return Decision('CERO',reason)
    if not math.isfinite(now):return zero('TIEMPO_INVALIDO')
    vals={}
    for k in ('df','di','dd'):
        x=getattr(f,k)
        if x is None:return zero('SIN_'+k.upper())
        if not math.isfinite(x.value) or not 0<=x.value<=2.0:return zero('INVALIDO_'+k.upper())
        if not fresh(now,x.stamp,SENSOR_AGE):return zero('CADUCADO_'+k.upper())
        vals[k]=x.value
    if f.scan is None:return zero('SIN_SCAN')
    if not math.isfinite(f.scan.value) or f.scan.value<=0:return zero('INVALIDO_SCAN')
    if not fresh(now,f.scan.stamp,SCAN_AGE):return zero('CADUCADO_SCAN')
    if not fresh(now,f.odom_stamp,ODOM_AGE):return zero('ODOM_AUSENTE_O_CADUCADA')
    df,di,dd=vals['df'],vals['di'],vals['dd']
    if at_or_below(f.scan.value,SCAN_STOP) or at_or_below(df,FRONTAL_STOP):
        return zero('PARADA_FRONTAL_SEGURA')
    if at_or_below(di,LATERAL_STOP) or at_or_below(dd,LATERAL_STOP):
        return zero('PARADA_LATERAL_SEGURA')
    route_mode=(not at_or_below(df,FRONTAL_RUTA)
                and not at_or_below(di,LATERAL_RUTA)
                and not at_or_below(dd,LATERAL_RUTA))
    mode='RUTA' if route_mode else 'DIFUSO'
    prop=f.route if route_mode else f.fuzzy
    if prop is None:return zero('SIN_PROPUESTA_'+mode)
    if not fresh(now,prop.stamp,PROPOSAL_AGE):return zero('PROPUESTA_CADUCADA_'+mode)
    if not all(math.isfinite(z) for z in (prop.v,prop.w)):
        return zero('PROPUESTA_NO_FINITA_'+mode)
    if not (0<=prop.v<=.200001 and abs(prop.w)<=1.500001):
        return zero('PROPUESTA_FUERA_DE_RANGO_'+mode)
    vmax=VMAX_RUTA if route_mode else VMAX_DIFUSO
    return Decision(mode,'PROPUESTA_'+mode+'_VALIDA',min(vmax,prop.v),max(-WMAX,min(WMAX,prop.w)))


def fixture(now=100.0,df=1.7,di=1.7,dd=1.7,scan=1.7,odom=True,
            route=(.08,0.),fuzzy=(.05,-.32),t=100.0):
    def reading(x):return None if x is None else Reading(float(x),float(t))
    return Frame(reading(df),reading(di),reading(dd),reading(scan),
                 t if odom else None,
                 None if route is None else Proposal(float(route[0]),float(route[1]),float(t)),
                 None if fuzzy is None else Proposal(float(fuzzy[0]),float(fuzzy[1]),float(t)))


def self_test() -> dict:
    tests=[]
    def assert_case(name,expected,f):
        d=decide(f,100.)
        check(d.mode==expected,f'{name}: esperado {expected}; actual {d}')
        check(0<=d.v<=.12+1e-9 and abs(d.w)<=.7+1e-9, name+': limites')
        if d.mode=='CERO':check(d.v==0 and d.w==0,name+': cero estricto')
        tests.append({'caso':name,'modo':d.mode,'motivo':d.reason,'v':round(d.v,6),'omega':round(d.w,6)})
    for df,di,dd in itertools.product((.31,.6,.600001,1.7),(.17,.35,.350001,1.7),(.17,.35,.350001,1.7)):
        expect='RUTA' if df>.6 and di>.35 and dd>.35 else 'DIFUSO'
        assert_case(f'frontal={df};izq={di};der={dd}',expect,fixture(df=df,di=di,dd=dd))
    # 4*4*4 = 64 combinaciones, cubriendo igualdad y transiciones fronterizas.
    assert_case('Frontal seguridad', 'CERO',fixture(df=.30))
    assert_case('Izquierda seguridad','CERO',fixture(di=.16))
    assert_case('Derecha seguridad','CERO',fixture(dd=.16))
    assert_case('Scan seguridad','CERO',fixture(scan=.30))
    assert_case('Frontal ausente','CERO',fixture(df=None))
    assert_case('Izquierda ausente','CERO',fixture(di=None))
    assert_case('Derecha ausente','CERO',fixture(dd=None))
    assert_case('Scan ausente','CERO',fixture(scan=None))
    assert_case('Odom ausente','CERO',fixture(odom=False))
    assert_case('Frontal NaN','CERO',fixture(df=math.nan))
    assert_case('Derecha infinita','CERO',fixture(dd=math.inf))
    assert_case('Izquierda negativa','CERO',fixture(di=-.01))
    assert_case('Frontal fuera dominio','CERO',fixture(df=2.1))
    assert_case('Propuesta ruta ausente','CERO',fixture(route=None))
    assert_case('Propuesta difuso ausente','CERO',fixture(df=.6,fuzzy=None))
    assert_case('Propuesta ruta infinita','CERO',fixture(route=(math.inf,0)))
    assert_case('Propuesta difuso NaN','CERO',fixture(di=.35,fuzzy=(0.,math.nan)))
    assert_case('Propuesta negativa','CERO',fixture(route=(-.01,0)))
    assert_case('Velocidad superior universo','CERO',fixture(route=(.21,0)))
    assert_case('Giro superior universo','CERO',fixture(route=(0,1.6)))
    assert_case('Saturacion positiva RUTA','RUTA',fixture(route=(.2,1.5)))
    assert_case('Saturacion negativa DIFUSO','DIFUSO',fixture(di=.35,fuzzy=(.2,-1.5)))
    # Datos recibidos en t=99.0 => caducidad 1.0s.
    assert_case('Rangos caducados','CERO',fixture(t=99.0))
    f=fixture()
    assert_case('Solo ruta caducada','CERO',Frame(f.df,f.di,f.dd,f.scan,f.odom_stamp,Proposal(.07,0,99.),f.fuzzy))
    f=fixture(df=.6)
    assert_case('Solo difuso caducado','CERO',Frame(f.df,f.di,f.dd,f.scan,f.odom_stamp,f.route,Proposal(.04,0,99.)))
    f=fixture()
    assert_case('Scan caducado','CERO',Frame(f.df,f.di,f.dd,Reading(1.,99.),f.odom_stamp,f.route,f.fuzzy))
    assert_case('Odom caducada','CERO',Frame(f.df,f.di,f.dd,f.scan,98.,f.route,f.fuzzy))
    assert_case('Frontal a 0.60 activa difuso','DIFUSO',fixture(df=.6))
    assert_case('Lateral izquierda a 0.35 activa difuso','DIFUSO',fixture(di=.35))
    assert_case('Lateral derecha a 0.35 activa difuso','DIFUSO',fixture(dd=.35))
    assert_case('Ruta liberada umbrales superados','RUTA',fixture(df=.600001,di=.350001,dd=.350001))
    assert_case('Limitacion velocidad ruta','RUTA',fixture(route=(.2,0)))
    assert_case('Limitacion velocidad difuso','DIFUSO',fixture(df=.6,fuzzy=(.2,0)))
    # Regresion REAL: ROS 2 sensor_msgs/Range almacena float32, no float64.
    # La fuente del fallo anterior fue 0.60 -> 0.6000000238418579.
    assert_case('ROS float32 frontal 0.60','DIFUSO',fixture(df=to_float32(.60)))
    assert_case('ROS float32 lateral izquierda 0.35','DIFUSO',fixture(di=to_float32(.35)))
    assert_case('ROS float32 lateral derecha 0.35','DIFUSO',fixture(dd=to_float32(.35)))
    assert_case('ROS float32 frontal seguridad 0.30','CERO',fixture(df=to_float32(.30)))
    assert_case('ROS float32 scan seguridad 0.30','CERO',fixture(scan=to_float32(.30)))
    assert_case('ROS float32 izquierda seguridad 0.16','CERO',fixture(di=to_float32(.16)))
    assert_case('ROS float32 derecha seguridad 0.16','CERO',fixture(dd=to_float32(.16)))
    assert_case('ROS por encima frontal 0.600001','RUTA',fixture(df=to_float32(.600001)))
    assert_case('ROS por encima izquierda 0.350001','RUTA',fixture(di=to_float32(.350001)))
    assert_case('ROS por encima derecha 0.350001','RUTA',fixture(dd=to_float32(.350001)))
    # Confirmación de saturación y uso de propuesta correcta.
    d=decide(fixture(df=.6,route=(.2,0.6),fuzzy=(.03,-.4)),100.)
    check(d.mode=='DIFUSO' and d.v==.03 and d.w==-.4,'DIFUSO no usó propuesta difusa')
    d=decide(fixture(df=.8,di=.4,dd=.4,route=(.07,.1),fuzzy=(.03,-.4)),100.)
    check(d.mode=='RUTA' and d.v==.07 and d.w==.1,'RUTA no usó propuesta de seguidor')
    total=len(tests)
    report={'estado':'APROBADA_PRUEBA_PURA', 'total':total,'casos':tests,
            'rubrica':{'ruta':'df>0.60 AND di>0.35 AND dd>0.35',
                       'difuso':'df<=0.60 OR di<=0.35 OR dd<=0.35'},
            'barreras_seguridad_extras':{'df_stop_m':FRONTAL_STOP,'lateral_stop_m':LATERAL_STOP,
                                       'scan_stop_m':SCAN_STOP},
            'limitacion':'No es prueba de movimiento Gazebo ni de reincorporacion',
            'regresion_float32':'Se cuantizan tanto sensores como umbrales a IEEE-754 binary32'}
    print(f'[OK] {total} casos de arbitraje/seguridad aprobados (incluye regresion float32); umbrales 0.60/0.35 m',flush=True)
    return report


def install_and_test():
    preflight()
    before=git('status','--porcelain=v1','--untracked-files=no')
    report=self_test()
    src=Path(__file__).resolve()
    check(not TARGET.is_symlink(),'El destino de A3 no puede ser un enlace simbolico')
    if TARGET.exists():
        check(TARGET.read_bytes()==src.read_bytes(),'Existe una version distinta de A3 v2; no se sobrescribe')
        print('[OK] Instalacion A3 reanudable: archivo existente identico')
    else:
        temp=TARGET.with_name('.A2_A3_supervisor_rubrica_v03_v2_float32.py.tmp')
        check(not temp.exists(),'Existe archivo temporal ajeno; abortar')
        shutil.copyfile(src,temp)
        temp.replace(TARGET)
        print('[OK] Candidato instalado en tools/ sin tocar componentes previos')
    OUTPUT_DIR.mkdir(parents=True,exist_ok=True)
    path=OUTPUT_DIR/'informe_autopruebas_float32.json'
    if path.exists():
        check(json.loads(path.read_text(encoding='utf8'))==report,
              'Existe informe A3 distinto; no se sobrescribe')
    else:
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    check(git('status','--porcelain=v1','--untracked-files=no')==before,
          'Se detecto modificacion inesperada de un archivo tracked')
    check(git_blob(V01)==V01_BLOB and sha256(V03)==V03_SHA256,'FIS modificado')
    print('[OK] FIS v01/v03 intactos; archivos tracked preexistentes conservados; sin stage/commit/push')
    print('[ROS] Banco ROS 2 v2 float32 en dominio 96; SIN Gazebo y SIN /cmd_vel ...',flush=True)
    env=dict(os.environ,ROS_DOMAIN_ID='96',ROS_LOCALHOST_ONLY='1',PYTHONUNBUFFERED='1')
    shell='source /opt/ros/humble/setup.bash && python3 '+str(TARGET)+' --prueba-ros-aislada'
    proc=subprocess.run(['bash','-lc',shell],cwd=ROOT,env=env,text=True,capture_output=True,timeout=105)
    print(proc.stdout,flush=True)
    if proc.stderr:print(proc.stderr,file=sys.stderr,flush=True)
    check(proc.returncode==0,'Prueba ROS aislada fallida; se conserva candidato para diagnostico')
    check(git('status','--porcelain=v1','--untracked-files=no')==before,
          'Prueba ROS modifico archivos tracked inesperadamente')
    print('=== A3 v2 FLOAT32: ARBITRAJE Y PRUEBA ROS AISLADA APROBADOS; INTEGRACION A4/A5 PENDIENTE ===')
    print('[ARCHIVO] '+str(TARGET))
    print('[EVIDENCIA] '+str(path))


def ros_node_class():
    # Importar ROS únicamente cuando se solicita ejecución ROS.
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Range,LaserScan
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data

    class SupervisorA3(Node):
        def __init__(self):
            super().__init__('a2_a3_supervisor_rubrica_sombra')
            self.inputs={'df':None,'di':None,'dd':None,'scan':None,'odom':None,
                         'route':None,'fuzzy':None}
            self.pub=self.create_publisher(Twist,OUT,10)
            self.status=self.create_publisher(String,STATUS,10)
            self.subs=[]
            for key,topic in RANGE_TOPICS.items():
                self.subs.append(self.create_subscription(Range,topic,
                    lambda msg,k=key:self.receive_range(k,msg),qos_profile_sensor_data))
            self.subs.append(self.create_subscription(LaserScan,'/scan',self.receive_scan,qos_profile_sensor_data))
            self.subs.append(self.create_subscription(Odometry,'/odom',self.receive_odom,qos_profile_sensor_data))
            self.subs.append(self.create_subscription(Twist,'/cmd_vel_ruta',
                                                     lambda m:self.receive_proposal('route',m),10))
            self.subs.append(self.create_subscription(Twist,'/cmd_vel_difuso',
                                                     lambda m:self.receive_proposal('fuzzy',m),10))
            self.timer=self.create_timer(.10,self.control)
            self.get_logger().info('A3 modo SOMBRA: publica '+OUT+'; JAMAS /cmd_vel')

        def receive_range(self,key,msg):
            v=float(msg.range);low=float(msg.min_range);high=float(msg.max_range)
            ok=all(map(math.isfinite,(v,low,high))) and 0<=low<=v<=high and 0<=v<=2
            self.inputs[key]=Reading(v,time.monotonic()) if ok else None

        def receive_scan(self,msg):
            near=[]
            for i,d in enumerate(msg.ranges):
                angle=math.atan2(math.sin(msg.angle_min+i*msg.angle_increment),
                                 math.cos(msg.angle_min+i*msg.angle_increment))
                if abs(angle)>math.radians(25):continue
                if math.isinf(d) and d>0 and math.isfinite(msg.range_max):
                    d=float(msg.range_max)
                if math.isfinite(d) and float(msg.range_min)<=d<=float(msg.range_max):
                    near.append(float(d))
            self.inputs['scan']=Reading(min(near),time.monotonic()) if near else None

        def receive_odom(self,msg):
            pos=msg.pose.pose.position
            vals=(float(pos.x),float(pos.y),float(pos.z))
            self.inputs['odom']=time.monotonic() if all(map(math.isfinite,vals)) else None

        def receive_proposal(self,key,msg):
            components=(float(msg.linear.x),float(msg.linear.y),float(msg.linear.z),
                        float(msg.angular.x),float(msg.angular.y),float(msg.angular.z))
            if not all(map(math.isfinite,components)) or any(abs(z)>1e-8 for z in
                        (components[1],components[2],components[3],components[4])):
                self.inputs[key]=None
            else:self.inputs[key]=Proposal(components[0],components[5],time.monotonic())

        def control(self):
            f=Frame(self.inputs['df'],self.inputs['di'],self.inputs['dd'],
                    self.inputs['scan'],self.inputs['odom'],self.inputs['route'],self.inputs['fuzzy'])
            d=decide(f,time.monotonic())
            cmd=Twist();cmd.linear.x=float(d.v);cmd.angular.z=float(d.w)
            self.pub.publish(cmd)
            payload={'modo':d.mode,'motivo':d.reason,'v':d.v,'omega':d.w}
            self.status.publish(String(data=json.dumps(payload,separators=(',',':'))))

        def stop(self):
            self.pub.publish(Twist())
            self.status.publish(String(data=json.dumps({'modo':'CERO','motivo':'APAGADO','v':0,'omega':0})))
    return SupervisorA3


def run_ros():
    check(os.environ.get('ROS_DOMAIN_ID')=='96' and os.environ.get('ROS_LOCALHOST_ONLY')=='1',
          'A3 solo permite dominio 96 y localhost en modo sombra')
    import rclpy
    rclpy.init()
    node=None
    try:
        node=ros_node_class()()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            for _ in range(5):node.stop();time.sleep(.05)
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def ros_smoke():
    """Ensayo con nodos rclpy reales, propuestas sintéticas; nunca publica /cmd_vel."""
    check(os.environ.get('ROS_DOMAIN_ID')=='96' and os.environ.get('ROS_LOCALHOST_ONLY')=='1',
          'Prueba ROS A3 necesita dominio 96 localhost')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Range,LaserScan
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    sup=None;test=None;executor=SingleThreadedExecutor()
    try:
        sup=ros_node_class()()
        test=Node('a2_a3_banco_ros_sombra')
        rngpub={k:test.create_publisher(Range,t,qos_profile_sensor_data) for k,t in RANGE_TOPICS.items()}
        spub=test.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        opub=test.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
        rp=test.create_publisher(Twist,'/cmd_vel_ruta',10)
        fp=test.create_publisher(Twist,'/cmd_vel_difuso',10)
        data={'state':None,'cmd':None,'states':[]}
        def state_cb(msg):
            try:
                v=json.loads(msg.data)
                data['state']=v;data['states'].append((time.monotonic(),v))
            except Exception:pass
        test.create_subscription(String,STATUS,state_cb,10)
        test.create_subscription(Twist,OUT,
            lambda msg:data.__setitem__('cmd',(msg.linear.x,msg.angular.z)),10)
        executor.add_node(sup);executor.add_node(test)
        for _ in range(50):executor.spin_once(timeout_sec=.04)
        names=[x for x in test.get_node_names() if x not in
               ('a2_a3_supervisor_rubrica_sombra','a2_a3_banco_ros_sombra')]
        check(not names,'Dominio 96 tiene nodos ajenos; abortar ensayo: '+repr(names))
        check(test.count_publishers('/cmd_vel')==0,'Existe publicador externo /cmd_vel')

        def publish(df,di,dd,scan=2.0,route=True,fuzzy=True):
            for k,val in [('df',df),('di',di),('dd',dd)]:
                m=Range();m.min_range=0.;m.max_range=2.;m.range=float(val)
                rngpub[k].publish(m)
            s=LaserScan();s.angle_min=-math.pi/4;s.angle_increment=math.pi/180
            s.angle_max=math.pi/4;s.range_min=.05;s.range_max=3.5
            s.ranges=[float(scan)]*91
            spub.publish(s)
            o=Odometry();o.pose.pose.position.x=0.;o.pose.pose.position.y=0.
            opub.publish(o)
            if route:
                m=Twist();m.linear.x=.08;rp.publish(m)
            if fuzzy:
                m=Twist();m.linear.x=.05;m.angular.z=-.3;fp.publish(m)

        def check_state(label,expect,df,di,dd,scan=2.,route=True,fuzzy=True):
            data['states'].clear()
            start=time.monotonic();end=start+2.2
            while time.monotonic()<end:
                publish(df,di,dd,scan,route,fuzzy)
                executor.spin_once(timeout_sec=.035)
                executor.spin_once(timeout_sec=.035)
            recent=[o for t,o in data['states'] if t>=end-.4]
            check(len(recent)>=2,f'{label}: faltan estados recientes')
            check(all(x['modo']==expect for x in recent),f'{label}: resultado={recent}')
            if expect=='CERO':check(all(x['v']==0 and x['omega']==0 for x in recent),label+': parada')
            print(f'[OK ROS] {label}: {expect}; muestras={len(recent)}',flush=True)

        check_state('Pasillo despejado','RUTA',1.7,1.2,1.2)
        check_state('Obstaculo lateral izquierda 0.35','DIFUSO',1.7,.35,1.2)
        check_state('Obstaculo lateral derecha 0.35','DIFUSO',1.7,1.2,.35)
        check_state('Obstaculo frontal 0.60 float32','DIFUSO',.60,1.2,1.2)
        check_state('Frontal apenas superior a 0.60','RUTA',.60001,1.2,1.2)
        check_state('Regreso ruta sin histeresis 0.80','RUTA',.61,.36,.36)
        check_state('Izquierda apenas superior a 0.35','RUTA',1.2,.35001,1.2)
        check_state('Derecha apenas superior a 0.35','RUTA',1.2,1.2,.35001)
        check_state('Parada segura frontal','CERO',.29,1.2,1.2,scan=.29)
        check_state('Parada segura izquierda','CERO',1.7,.14,1.2)
        check_state('Propuesta difuso ausente','CERO',.6,1.2,1.2,fuzzy=False)
        # Comandos dejan de publicarse, aun con sensores sanos: cero fail-safe.
        data['states'].clear();end=time.monotonic()+1.5
        while time.monotonic()<end:
            publish(1.7,1.2,1.2,route=False,fuzzy=False)
            executor.spin_once(timeout_sec=.04)
            executor.spin_once(timeout_sec=.04)
        recent=[o for t,o in data['states'] if t>=end-.35]
        check(len(recent)>=2 and all(x['modo']=='CERO' for x in recent),'Caducidad ROS no bloqueo movimiento')
        print('[OK ROS] Propuestas caducadas => CERO')
        check(test.count_publishers('/cmd_vel')==0,'A3 nunca debe publicar /cmd_vel')
        print('[OK ROS] /cmd_vel sin publicadores; SOLO /cmd_vel_a3_prueba')
        print('=== A3 v2: PRUEBA ROS AISLADA APROBADA (NO GAZEBO, NO MOVIMIENTO) ===')
    finally:
        try:
            if sup:
                for _ in range(5):sup.stop();executor.spin_once(timeout_sec=.02)
            executor.shutdown()
            if sup:sup.destroy_node()
            if test:test.destroy_node()
        finally:
            if rclpy.ok():rclpy.shutdown()


def main()->int:
    p=argparse.ArgumentParser(description='A3 supervisor RUTA-DIFUSO compatible umbrales oficiales')
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--instalar-y-probar',action='store_true')
    group.add_argument('--self-test',action='store_true')
    group.add_argument('--ros-sombra',action='store_true')
    group.add_argument('--prueba-ros-aislada',action='store_true')
    args,unknown=p.parse_known_args()
    try:
        if args.instalar_y_probar:install_and_test()
        elif args.self_test:self_test()
        elif args.ros_sombra:
            sys.argv=[sys.argv[0],*unknown]
            run_ros()
        elif args.prueba_ros_aislada:ros_smoke()
        return 0
    except Exception as exc:
        print('[ERROR A3] '+str(exc),file=sys.stderr,flush=True)
        return 1


if __name__=='__main__':sys.exit(main())
