#!/usr/bin/env python3
"""AMR A2 / A4.1 v02: seguidor de ruta con pausa/reincorporacion segura.

Unico comando en WSL2:
 python3 /mnt/c/Users/Henry/Downloads/A2_A4_seguidor_reincorporacion_v02_evento.py --instalar-y-probar

CONTRATO: Este es un candidato en SOMBRA. Publica /cmd_vel_ruta_a4_prueba,
no /cmd_vel_ruta ni /cmd_vel. La prueba ROS usa odometria y estados sinteticos;
NO acredita movimiento en Gazebo ni evasión real, pendientes A4.2/A5.

Conserva supervisor_f13.py, seguidor_ruta_f13.py, FIS v01/v03 y el commit
remoto a604f02. Nunca modifica archivos tracked ni realiza git add/commit/push.
"""
from __future__ import annotations
import argparse
import bisect
import csv
from dataclasses import dataclass
import datetime as dt
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT=Path.home()/'AMR_Logistico_Fase_0'
TARGET=ROOT/'tools/A2_A4_seguidor_reincorporacion_v02_evento.py'
A4_V01=ROOT/'tools/A2_A4_seguidor_reincorporacion_v01.py'
A4_V01_SHA='b7ee924299e2799bad5e31ffcc3585479d0abfa894d9c17aa14415d9ffca53d2'
EVID=ROOT/'resultados/A2_A4_reincorporacion_v02_evento'
REF='a604f02dd89c03145d3c253693c656aca47ae1e7'
A3=ROOT/'tools/A2_A3_supervisor_rubrica_v03_v2_float32.py'
A3_SHA='2c820de906497080e84e9ff37c458d8787a290438d6651cf38bdc22e9bcd4e92'
V01=ROOT/'matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis'
V01_BLOB='6c7dfdd083728d49c64dd1c90a59b19d63222063'
V03=ROOT/'matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis'
V03_SHA='67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903'
ROUTE=ROOT/'resultados/F12_ruta_1.csv'
OUT='/cmd_vel_ruta_a4_prueba'
STATUS='/estado_a4_prueba'
STATE_A3='/estado_a3_prueba'
ROS_DOMAIN='97'
VMAX=.10
WMAX=.70
LOOKAHEAD=.16
GOAL_TOL=.08
MAX_DEVIATION=.45
MAX_BACKTRACK=.20
ODOM_AGE=.80
MODE_AGE=.50


def require(ok,msg):
    if not ok:raise RuntimeError(msg)

def git(*args):
    p=subprocess.run(['git',*args],cwd=ROOT,text=True,capture_output=True,timeout=25)
    require(p.returncode==0, 'Git '+str(args)+': '+p.stderr.strip())
    return p.stdout.strip()

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def blob(p):
    raw=p.read_bytes()
    return hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()

def preflight():
    require((ROOT/'.git').is_dir(),'Falta repositorio WSL')
    require(git('branch','--show-current')=='desarrollo-avance2','Rama Git inesperada')
    require(git('rev-parse','HEAD')==REF,'HEAD distinto de a604f02: auditar antes de integrar A4')
    require(V01.is_file() and blob(V01)==V01_BLOB,'FIS v01 original alterado')
    require(V03.is_file() and digest(V03)==V03_SHA,'FIS v03 distinto del A1 aprobado')
    require(A3.is_file() and digest(A3)==A3_SHA,'Candidato A3 v2 ausente o distinto del aprobado')
    require(A4_V01.is_file() and digest(A4_V01)==A4_V01_SHA,'Candidato A4 v01 historico ausente o modificado')
    for path in ['tools/seguidor_ruta_f13.py','tools/supervisor_f13.py','tools/A2_E22_nodo_difuso.py']:
        require((ROOT/path).is_file(),'Falta historico '+path)
    require(not TARGET.is_symlink() and not EVID.is_symlink(),'Ruta de A4 no segura')
    print('[OK] HEAD, rama, FIS v01/v03, A3 v2 y nodos historicos protegidos',flush=True)


def load_route(path):
    require(path.is_file(),'No existe CSV '+str(path))
    with path.open('r',encoding='utf-8-sig',newline='') as fd:
        r=csv.DictReader(fd)
        require(r.fieldnames is not None and {'x_m','y_m'}.issubset(r.fieldnames),'CSV sin columnas x_m,y_m')
        data=[(float(row['x_m']),float(row['y_m'])) for row in r]
    require(len(data)>=2,'Ruta con menos de dos puntos')
    require(all(all(math.isfinite(v) for v in p) for p in data),'Ruta no finita')
    require(all(0<math.dist(a,b)<=.150001 for a,b in zip(data,data[1:])),
            'Ruta discontinua o puntos duplicados')
    return data


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


class Polyline:
    def __init__(self,pts):
        self.pts=list(pts)
        require(len(self.pts)>=2,'Se requiere polilinea')
        self.arcs=[0.]
        for a,b in zip(self.pts,self.pts[1:]):
            d=math.dist(a,b)
            require(d>1e-12 and math.isfinite(d),'Segmento invalido')
            self.arcs.append(self.arcs[-1]+d)
        self.length=self.arcs[-1]

    def at(self,s):
        s=max(0.,min(self.length,float(s)))
        j=min(bisect.bisect_right(self.arcs,s)-1,len(self.pts)-2)
        a,b=self.pts[j],self.pts[j+1]
        t=(s-self.arcs[j])/(self.arcs[j+1]-self.arcs[j])
        return a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])

    def nearest(self,xy,min_s=0.):
        """Proyeccion sobre segmentos, no simple vertice; evita volver al inicio.

        Solo se aceptan segmentos con progreso al menos min_s; un punto
        alejado no da permiso de avanzar: el control lo rechaza luego.
        """
        candidates=[]
        for i,(a,b) in enumerate(zip(self.pts,self.pts[1:])):
            ax,ay=a;bx,by=b;dx=bx-ax;dy=by-ay
            length2=dx*dx+dy*dy
            t=max(0.,min(1.,((xy[0]-ax)*dx+(xy[1]-ay)*dy)/length2))
            s=self.arcs[i]+t*(self.arcs[i+1]-self.arcs[i])
            if s+1e-9<min_s:continue
            qx,qy=ax+t*dx,ay+t*dy
            candidates.append((math.hypot(xy[0]-qx,xy[1]-qy),-s,s,(qx,qy)))
        require(candidates,'No hay segmento elegible para reincorporacion')
        d,neg_s,s,q=min(candidates)
        return (d,s,q)


@dataclass(frozen=True)
class Cmd:
    status:str
    v:float=0.
    w:float=0.
    target:tuple[float,float]|None=None
    progress_m:float=0.
    deviation_m:float|None=None

class Follower:
    """No realiza evasión: difuso lo hace por su propuesta separada.

    Sigue la ruta en modo RUTA. En modo DIFUSO no compite por el mando;
    al volver a RUTA proyecta odometria sobre polilinea y busca un punto
    adelantado LOOKAHEAD. Si no puede reincorporarse -> PARAR/REPLANIFICAR.
    """
    def __init__(self,pts):
        self.path=Polyline(pts)
        self.progress=0.
        self.previous='CERO'
        self.done=False
        self.ever_resumed=False

    def step(self,pose,mode,now,odom_t,mode_t):
        zero=lambda reason,dev=None:Cmd(reason,progress_m=self.progress,deviation_m=dev)
        if self.done:return zero('META_COMPLETADA')
        if (pose is None or len(pose)!=3 or not all(math.isfinite(x) for x in pose)
            or not math.isfinite(now) or odom_t is None or mode_t is None
            or not (0<=now-odom_t<=ODOM_AGE) or not (0<=now-mode_t<=MODE_AGE)):
            self.previous='CERO'
            return zero('PARADA_DATOS_INVALIDOS_O_CADUCADOS')
        if mode not in ('RUTA','DIFUSO','CERO'):
            self.previous='CERO'
            return zero('PARADA_MODO_INVALIDO')
        if mode!='RUTA':
            self.previous=mode
            return zero('CEDER_CONTROL_DIFUSO' if mode=='DIFUSO' else 'PARADA_SUPERVISOR')
        xy=pose[:2]
        min_s=max(0.,self.progress-MAX_BACKTRACK)
        dev,s,q=self.path.nearest(xy,min_s)
        if dev>MAX_DEVIATION:
            self.previous='CERO'
            return zero('REPLANIFICACION_REQUERIDA',dev)
        resuming=self.previous!='RUTA'
        if resuming and self.progress>.0:self.ever_resumed=True
        self.previous='RUTA'
        self.progress=max(self.progress,s)
        if math.dist(xy,self.path.pts[-1])<=GOAL_TOL:
            self.done=True
            return zero('META_COMPLETADA',dev)
        target=self.path.at(min(self.path.length,self.progress+LOOKAHEAD))
        bearing=math.atan2(target[1]-xy[1],target[0]-xy[0])
        err=wrap(bearing-pose[2])
        w=max(-WMAX,min(WMAX,1.6*err))
        v=0. if abs(err)>math.radians(34) else min(VMAX,max(.018,.45*math.dist(xy,target)))
        v*=max(.1,math.cos(err)) if v>0 else 1.
        return Cmd('REINCORPORANDO' if resuming and self.progress>0 else 'SEGUIR_RUTA',
                   v,w,target,self.progress,dev)


def load_a3():
    spec=importlib.util.spec_from_file_location('a2_a3_core_for_a4',A3)
    require(spec is not None and spec.loader is not None,'No importable A3')
    m=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=m;spec.loader.exec_module(m)
    return m


def pure_tests():
    count=0;results=[]
    def yes(name,cond):
        nonlocal count
        require(cond,name)
        count+=1;results.append(name)
    # Geometria real publicada, sin usar el archivo original de resultados F13.
    for i in range(1,5):
        p=load_route(ROOT/f'resultados/F12_ruta_{i}.csv');line=Polyline(p)
        yes(f'Ruta A* {i}: puntos y longitud',len(p)>10 and line.length>.9)
        for fraction in (.1,.5,.9):
            q=line.at(line.length*fraction)
            dev,s,proj=line.nearest(q)
            yes(f'Ruta {i}: proyeccion {fraction}',dev<1e-7 and abs(s-line.length*fraction)<1e-7)
    # Ensayos geométricos reproducibles en una ruta recta, sin mover el robot.
    p=[(round(i*.05,3),0.) for i in range(41)]
    f=Follower(p);t=100.
    c=f.step((0.,0.,0.),'RUTA',t,t,t)
    yes('Inicio: propone avance seguro',c.v>0 and c.w==0 and c.progress_m==0)
    c=f.step((.65,0.,0.),'RUTA',t,t,t)
    yes('Progreso sobre ruta',c.v>0 and abs(c.progress_m-.65)<1e-6)
    c=f.step((.85,.12,0.),'DIFUSO',t,t,t)
    yes('Modo DIFUSO cede control sin avance de ruta',c.v==0 and c.w==0 and f.progress==.65)
    c=f.step((1.12,.16,0.),'CERO',t,t,t)
    yes('CERO de supervisor: propuesta ruta nula',c.v==0 and c.w==0)
    c=f.step((1.27,.12,-.15),'RUTA',t,t,t)
    yes('Reincorporacion desde pose desplazada',c.status=='REINCORPORANDO' and c.progress_m>=1.27 and c.target is not None)
    yes('Reincorporacion produce rumbo correctivo',c.w<0 and c.v>=0)
    c=f.step((1.27,.12,-.15),'RUTA',t,t,t)
    yes('Estado de reincorporacion es transitorio, no permanente',c.status=='SEGUIR_RUTA')
    yes('Transicion no cambia artificialmente progreso',c.progress_m>=1.27)
    c=f.step((1.33,.08,0.),'RUTA',t,t,t)
    yes('Tras reincorporar sigue adelante',c.status=='SEGUIR_RUTA' and c.progress_m>=1.33)
    before=f.progress
    c=f.step((1.12,.05,0.),'RUTA',t,t,t)
    yes('No retrocede progreso tras oscilacion',c.progress_m>=before)
    c=f.step((2.,0.,0.),'RUTA',t,t,t)
    yes('Meta reconocida',c.status=='META_COMPLETADA' and c.v==0)
    yes('Meta bloquea posteriores propuestas',f.step((1.5,0.,0.),'RUTA',t,t,t).v==0)
    g=Follower(p);g.step((.1,0,0),'RUTA',t,t,t)
    c=g.step((.8,.9,0.),'RUTA',t,t,t)
    yes('Desviacion excesiva solicita replanificar',c.status=='REPLANIFICACION_REQUERIDA' and c.v==0)
    c=g.step((.2,0,0),'RUTA',t,t,t-.9)
    yes('Modo caducado bloquea movimiento',c.status=='PARADA_DATOS_INVALIDOS_O_CADUCADOS')
    c=g.step((.2,0,0),'RUTA',t,t-.9,t)
    yes('Odom caducada bloquea movimiento',c.v==0)
    c=g.step((.2,0,0),'INCIERTO',t,t,t)
    yes('Modo desconocido bloquea movimiento',c.v==0)
    g=Follower(p);c=g.step((.2,0,math.pi),'RUTA',t,t,t)
    yes('Giro en sitio antes de avanzar cuando opuesto',c.v==0 and abs(c.w)<=WMAX)
    # Prueba de contrato A3 -> A4, no de Gazebo: sensor float32 y cese.
    a3=load_a3()
    d=a3.decide(a3.fixture(df=1.4,di=1.1,dd=1.1),100.)
    yes('A3 libre selecciona RUTA',d.mode=='RUTA')
    d=a3.decide(a3.fixture(df=a3.to_float32(.6),di=1.,dd=1.),100.)
    yes('A3 obstaculo frontal delega DIFUSO',d.mode=='DIFUSO')
    d=a3.decide(a3.fixture(df=1.,di=a3.to_float32(.35),dd=1.),100.)
    yes('A3 obstaculo izquierdo delega DIFUSO',d.mode=='DIFUSO')
    d=a3.decide(a3.fixture(df=.29,scan=.29),100.)
    yes('A3 en riesgo impone CERO',d.mode=='CERO')
    print(f'[OK] A4 v02: {count} pruebas puras con rutas A* y contrato A3 aprobadas',flush=True)
    return {'estado':'PRUEBAS_A4_1_APROBADAS','n_pruebas':count,'nombres':results,
            'limitacion':'Simulacion matematica y ROS aislado: no Gazebo, no demostracion de evasión fisica'}


def make_follower_node():
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    class FollowerNode(Node):
        def __init__(self):
            super().__init__('a2_a4_seguidor_sombra')
            self.core=Follower(load_route(ROUTE))
            self.pose=None;self.odom_at=None;self.mode=None;self.mode_at=None
            self.pub=self.create_publisher(Twist,OUT,10)
            self.status=self.create_publisher(String,STATUS,10)
            self.create_subscription(Odometry,'/odom',self.odometry,qos_profile_sensor_data)
            self.create_subscription(String,STATE_A3,self.state,10)
            self.create_timer(.10,self.tick)
            self.get_logger().info('A4 SOMBRA: solo '+OUT+'; nunca /cmd_vel_ruta ni /cmd_vel')
        def odometry(self,msg):
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            pose=(float(p.x),float(p.y),float(yaw))
            self.pose=pose if all(math.isfinite(x) for x in pose) else None
            self.odom_at=time.monotonic()
        def state(self,msg):
            try:
                d=json.loads(msg.data);mode=d.get('modo')
                if mode not in ('RUTA','DIFUSO','CERO'):raise ValueError('Modo invalido')
                self.mode=mode
                self.mode_at=time.monotonic()
            except (ValueError,TypeError,KeyError):
                self.mode=None;self.mode_at=None
        def tick(self):
            c=self.core.step(self.pose,self.mode,time.monotonic(),self.odom_at,self.mode_at)
            msg=Twist();msg.linear.x=float(c.v);msg.angular.z=float(c.w)
            self.pub.publish(msg)
            payload={'estado':c.status,'v':c.v,'omega':c.w,
                     'progreso_m':c.progress_m,'desviacion_m':c.deviation_m,
                     'objetivo':c.target}
            self.status.publish(String(data=json.dumps(payload,separators=(',',':'))))
        def stop(self):
            self.pub.publish(Twist())
            self.status.publish(String(data=json.dumps({'estado':'DETENIDO','v':0.,'omega':0.})))
    return FollowerNode


def ros_test():
    """El banco ROS SOLO emplea odometria/estados fabricados en un dominio aislado.

    No reproduce movimiento ni publica comandos operativos. A4.2 es la prueba
    pendiente en Gazebo, con trayectoria real y obstaculos insertados.
    """
    require(os.environ.get('ROS_DOMAIN_ID')==ROS_DOMAIN and os.environ.get('ROS_LOCALHOST_ONLY')=='1',
            'A4 exige ROS_DOMAIN_ID=97 y ROS_LOCALHOST_ONLY=1')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    follow=None;test=None;executor=SingleThreadedExecutor();counts=[]
    try:
        follow=make_follower_node()()
        test=Node('a2_a4_banco_ros_sombra')
        po=test.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
        ps=test.create_publisher(String,STATE_A3,10)
        samples=[];vels=[]
        def on_state(msg):
            try:samples.append((time.monotonic(),json.loads(msg.data)))
            except Exception:pass
        def on_cmd(msg):vels.append((time.monotonic(),(msg.linear.x,msg.angular.z)))
        test.create_subscription(String,STATUS,on_state,10)
        test.create_subscription(Twist,OUT,on_cmd,10)
        executor.add_node(follow);executor.add_node(test)
        for _ in range(32):executor.spin_once(timeout_sec=.04)
        others=[x for x in test.get_node_names() if x not in ('a2_a4_seguidor_sombra','a2_a4_banco_ros_sombra')]
        require(not others,'Dominio 97 contiene nodos ajenos: '+str(others))
        require(test.count_publishers('/cmd_vel')==0 and test.count_publishers('/cmd_vel_ruta')==0,
                'El dominio aislado tiene publicadores operativos')
        route=load_route(ROUTE)
        line=Polyline(route)
        def send(pose,mode):
            m=Odometry();m.pose.pose.position.x=pose[0];m.pose.pose.position.y=pose[1]
            m.pose.pose.orientation.z=math.sin(pose[2]/2)
            m.pose.pose.orientation.w=math.cos(pose[2]/2)
            po.publish(m)
            ps.publish(String(data=json.dumps({'modo':mode,'motivo':'A4_BANCO_ODOMETRIA_SINTETICA'})))
        def verify(label,pose,mode,expected,positive=None,transient=None):
            samples.clear();vels.clear()
            end=time.monotonic()+.75
            while time.monotonic()<end:
                send(pose,mode)
                executor.spin_once(timeout_sec=.03)
                executor.spin_once(timeout_sec=.03)
            # El estado REINCORPORANDO es un EVENTO de un ciclo (100 ms).
            # Los mensajes del ultimo tramo ya deben ser SEGUIR_RUTA.
            all_states=[x for t,x in samples]
            recent=[x for t,x in samples if t>=end-.3]
            speed=[x for t,x in vels if t>=end-.3]
            require(len(all_states)>=3 and len(recent)>=2 and len(speed)>=2,
                    label+': ROS sin muestras suficientes')
            require(all(x['estado']==expected for x in recent),
                    label+': estado final inesperado '+repr(recent))
            if transient is not None:
                events=[(i,x) for i,x in enumerate(all_states) if x['estado']==transient]
                require(len(events)==1,
                        label+': esperado un unico evento '+transient+'; estados='+
                        repr([x['estado'] for x in all_states]))
                idx,event=events[0]
                require(any(x['estado']==expected for x in all_states[idx+1:]),
                        label+': evento sin posterior SEGUIR_RUTA')
                require(float(event['progreso_m'])>=.65 and
                        float(event['desviacion_m'])<=.11 and
                        event['objetivo'] is not None and
                        0<=float(event['v'])<=VMAX and
                        abs(float(event['omega']))<=WMAX+1e-9,
                        label+': reincorporacion con magnitudes incorrectas: '+repr(event))
            if positive is True:
                require(all(v>=0 and abs(w)<=WMAX for v,w in speed) and any(v>0 for v,w in speed),
                        label+': sin avance')
            if positive is False:
                require(all(abs(v)<1e-7 and abs(w)<1e-7 for v,w in speed),
                        label+': propuesta no nula')
            state_display=(transient+' (evento) -> ' if transient else '')+expected+' (estable)'
            print(f'[OK ROS A4] {label}: {state_display}; muestras={len(all_states)}',flush=True)
            counts.append(label)
        a=line.at(0.)
        yaw=math.atan2(route[1][1]-route[0][1],route[1][0]-route[0][0])
        verify('Inicio ruta',(a[0],a[1],yaw),'RUTA','SEGUIR_RUTA',True)
        q=line.at(line.length*.38)
        verify('Avance A*',(q[0],q[1],yaw),'RUTA','SEGUIR_RUTA',True)
        verify('Control cedido a DIFUSO',(q[0],q[1]+.12,yaw),'DIFUSO','CEDER_CONTROL_DIFUSO',False)
        verify('Parada segura A3',(q[0],q[1]+.14,yaw),'CERO','PARADA_SUPERVISOR',False)
        r=line.at(line.length*.68)
        verify('Reincorporacion por proyeccion',(r[0],r[1]+.10,yaw),'RUTA','SEGUIR_RUTA',None,transient='REINCORPORANDO')
        u=line.at(line.length*.75)
        verify('Seguimiento posterior',(u[0],u[1]+.04,yaw),'RUTA','SEGUIR_RUTA',None)
        z=line.at(line.length)
        verify('Meta alcanzada',(z[0],z[1],yaw),'RUTA','META_COMPLETADA',False)
        require(test.count_publishers('/cmd_vel')==0 and test.count_publishers('/cmd_vel_ruta')==0,
                'A4 publicó en tópico operativo inesperado')
        print('[OK ROS A4] Solo '+OUT+'; ningun publicador en /cmd_vel ni /cmd_vel_ruta',flush=True)
        print('=== A4.1 v02: PRUEBA ROS AISLADA Y EVENTO DE REINCORPORACION APROBADOS; GAZEBO PENDIENTE ===',flush=True)
        return counts
    finally:
        try:
            if follow:
                for _ in range(3):follow.stop();executor.spin_once(timeout_sec=.02)
            executor.shutdown()
            if follow:follow.destroy_node()
            if test:test.destroy_node()
        finally:
            if rclpy.ok():rclpy.shutdown()


def install_test():
    preflight()
    before=git('status','--porcelain=v1','--untracked-files=no')
    report=pure_tests()
    src=Path(__file__).resolve()
    if TARGET.exists():
        require(TARGET.read_bytes()==src.read_bytes(), 'Existe A4 v02 distinto; no se sobrescribe')
        print('[OK] A4 ya instalado: archivo identico')
    else:
        TARGET.parent.mkdir(parents=True,exist_ok=True)
        temp=TARGET.with_name('.A2_A4_seguidor_reincorporacion_v02_evento.py.tmp')
        require(not temp.exists(), 'Archivo temporal A4 ajeno; no se sobrescribe')
        shutil.copyfile(src,temp);temp.replace(TARGET)
        print('[OK] Candidato A4 instalado sin reemplazar seguidor ni supervisor historicos')
    env=dict(os.environ,ROS_DOMAIN_ID=ROS_DOMAIN,ROS_LOCALHOST_ONLY='1',PYTHONUNBUFFERED='1')
    shell='source /opt/ros/humble/setup.bash && python3 '+str(TARGET)+' --ros-aislado'
    p=subprocess.run(['bash','-lc',shell],cwd=ROOT,env=env,capture_output=True,text=True,timeout=85)
    if p.stdout:print(p.stdout,flush=True)
    if p.stderr:print(p.stderr,file=sys.stderr,flush=True)
    require(p.returncode==0,'Prueba ROS A4.1 fallida; revisar error; no se realizaron commits')
    report['ros_escenarios']=7
    report['regresion_v02']='REINCORPORANDO se observa una sola vez; SEGUIR_RUTA se estabiliza'
    report['ros_resultado']='PRUEBA_SOMBRA_ODOMETRIA_SINTETICA_APROBADA'
    report['ref_commit']=REF
    report['fis_v03_sha256']=V03_SHA
    report['fecha_utc']=dt.datetime.now(dt.timezone.utc).isoformat(timespec='seconds')
    EVID.mkdir(parents=True,exist_ok=True)
    filename=EVID/('informe_A4_1_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
    require(not filename.exists(),'Coincidencia de ruta del reporte: no sobrescribir')
    filename.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    require(git('status','--porcelain=v1','--untracked-files=no')==before,
            'A4 v02 alteró archivos tracked previos')
    require(blob(V01)==V01_BLOB and digest(V03)==V03_SHA and digest(A4_V01)==A4_V01_SHA,
            'FIS o A4 v01 historico alterado durante A4 v02')
    print('[OK] Archivos tracked preexistentes conservados; sin git add, commit ni push',flush=True)
    print('[EVIDENCIA] '+str(filename),flush=True)
    print('=== A4.1 v02: REINCORPORACION ROS AISLADA APROBADA; A4.2 EN GAZEBO PENDIENTE ===',flush=True)


def main():
    parser=argparse.ArgumentParser(description='AMR A4.1 - reincorporacion a A* sin tocar comandos operativos')
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--instalar-y-probar',action='store_true')
    group.add_argument('--self-test',action='store_true')
    group.add_argument('--ros-aislado',action='store_true')
    args=parser.parse_args()
    try:
        if args.instalar_y_probar:install_test()
        elif args.self_test:preflight();pure_tests()
        else:ros_test()
        return 0
    except Exception as e:
        print('[ERROR A4.1] '+str(e),file=sys.stderr,flush=True)
        return 1

if __name__=='__main__':sys.exit(main())
