#!/usr/bin/env python3
"""B2.3: UN ensayo integrado de navegacion de la ruta A* de rescate.

Revision v02: ejecuta --monitor con ESTE script (no con el verificador spawn).

Uso (Ubuntu/WSL, junto a los dos scripts de rescate anteriores):
 python3 /mnt/c/Users/Henry/Downloads/A2_B2_rescate_ensayo_integrado_v02.py --ensayar

Reutiliza los archivos ya aprobados. No modifica Git, mundo original, mapa,
FIS, A3 ni A4. Un unico puente publica /cmd_vel; los controladores locales
A3/A4 se importan SIN alteracion, salvo redireccionar A4.ROUTE EN MEMORIA.
Los comandos pueden detenerse por barreras adicionales, nunca se eluden
las decisiones de seguridad de A3. La distancia geometrica NO equivale
por si misma a evidencia de ausencia de contactos fisicos Gazebo.
"""
from __future__ import annotations
import argparse, csv, datetime as dt, hashlib, importlib.util, json, math, os
from pathlib import Path
import signal, sqlite3, subprocess, sys, time, xml.etree.ElementTree as ET

REPO = Path.home() / 'AMR_Logistico_Fase_0'
SELF = Path(__file__).resolve()
SPAWN_SCRIPT = SELF.with_name('A2_B2_rescate_verificar_spawn_v01.py')
A3 = REPO/'tools/A2_A3_supervisor_rubrica_v03_v2_float32.py'
A4 = REPO/'tools/A2_A4_seguidor_reincorporacion_v02_evento.py'
R18 = REPO/'tools/A2_E21_equivalencia_MATLAB_Python.py'
FIS = REPO/'matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis'
EXPECTED = {
    A3: '2c820de906497080e84e9ff37c458d8787a290438d6651cf38bdc22e9bcd4e92',
    A4: '5f03cfaea0ee52769e33b32924441ea72761d4c8300f778290b17dc57db5e02d',
    FIS: '67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903',
}
DOM = '120'
START = (-1.975,-2.775)
GOAL = (-3.975,-2.775)
YAW = math.pi
MAX_WALL_S = 220.0
MAX_IDLE_S = 22.0
STOP_GEOM = 0.255  # 0.21 m de inflado mas 0.045 de reserva experimental
STOP_FRONT = 0.23
MAX_ROUTE_DEV = 0.38
MAX_V, MAX_W = 0.20, 1.5
TOPICS = ['/clock','/odom','/scan','/rango_frontal','/rango_izq','/rango_der',
          '/cmd_vel_ruta_a4_prueba','/estado_a4_prueba','/cmd_vel_ruta',
          '/cmd_vel_difuso','/estado_a3_prueba','/cmd_vel_a3_prueba','/cmd_vel']


def say(msg): print(msg,flush=True)
def need(ok,msg):
    if not ok: raise RuntimeError(msg)
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def wrap(a): return math.atan2(math.sin(a),math.cos(a))
def eucl(p,q): return math.hypot(p[0]-q[0],p[1]-q[1])

def import_file(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    need(spec is not None and spec.loader is not None,'No se puede importar: '+str(path))
    mod=importlib.util.module_from_spec(spec)
    sys.modules[name]=mod; spec.loader.exec_module(mod)
    return mod


def load_spawn():
    need(SPAWN_SCRIPT.is_file(),'Guardar junto a este script: '+SPAWN_SCRIPT.name)
    return import_file(SPAWN_SCRIPT,'b2_spawn_aprobado')


def route_points(path):
    with path.open(newline='',encoding='utf-8-sig') as h:
        r=csv.DictReader(h)
        need({'x_m','y_m'}<=set(r.fieldnames or []),'Ruta A* sin cabecera esperada')
        points=[(float(q['x_m']),float(q['y_m'])) for q in r]
    need(len(points)==35 and eucl(points[0],START)<.006 and eucl(points[-1],GOAL)<.006,
         'La ruta preparada ya no es la ruta de rescate de 35 puntos')
    need(all(eucl(a,b)<.15 for a,b in zip(points,points[1:])),
         'La ruta tiene discontinuidad para A4')
    return points


def collision_rectangles(world):
    root=ET.parse(world).getroot()
    rects=[];uns=[]
    for m in root.findall('./world/model'):
        name=m.get('name','')
        pose=list(map(float,m.findtext('pose','0 0 0 0 0 0').split()))
        if len(pose)!=6: continue
        x,y,z,roll,pitch,yaw=pose
        for link in m.findall('link'):
            for coll in link.findall('collision'):
                dims=coll.findtext('./geometry/box/size')
                if not dims or any(abs(q)>1e-8 for q in (roll,pitch,yaw)) or link.find('pose') is not None or coll.find('pose') is not None:
                    uns.append(name); continue
                sx,sy,sz=map(float,dims.split())
                if z+sz/2<.02:continue
                rects.append((name,x-sx/2,x+sx/2,y-sy/2,y+sy/2))
    need(any(r[0]=='a2_b2_obstaculo_estatico_centro' for r in rects),
         'No se detecta el obstaculo en la geometria de colision')
    # Fallar en lugar de ignorar colisiones desconocidas.
    need(not uns,'Colisiones SDF no modeladas por esta prueba: '+str(sorted(set(uns))))
    return rects


def clearance(pos,rects):
    x,y=pos
    return min(math.hypot(max(a-x,0.0,x-b),max(c-y,0.0,y-d))
               for _,a,b,c,d in rects)


def nearest_distance(p,points):
    best=float('inf')
    x,y=p
    for a,b in zip(points,points[1:]):
        vx,vy=b[0]-a[0],b[1]-a[1]
        L=vx*vx+vy*vy
        t=max(0.,min(1.,((x-a[0])*vx+(y-a[1])*vy)/L)) if L>0 else 0.
        best=min(best,math.hypot(x-a[0]-t*vx,y-a[1]-t*vy))
    return best


def projected_xy(x,y,yaw,v,w,h=0.35):
    if abs(w)<1e-6:return x+v*h*math.cos(yaw),y+v*h*math.sin(yaw)
    return x+(v/w)*(math.sin(yaw+w*h)-math.sin(yaw)),\
           y-(v/w)*(math.cos(yaw+w*h)-math.cos(yaw))


def parser_fis():
    # R18 (solo lectura) acepta un FIS cuyo Name es v01.
    # Adaptar unicamente Name EN MEMORIA para consumir el FIS v03 sin editarlo.
    r18=import_file(R18,'b2_rescate_r18')
    txt=FIS.read_text(encoding='utf-8')
    original="Name='AMR_Mamdani_A2_v03_rubrica'"
    old="Name='AMR_Mamdani_A2_v01'"
    need(txt.startswith('[System]\n'+original+'\n') and txt.count(original)==1,
         'Cabecera FIS v03 inesperada')
    mem=txt.replace(original,old,1)
    class ReadOnly:
        def read_text(self,encoding='utf-8'):
            need(encoding.lower().replace('_','-')=='utf-8','Codificacion inesperada')
            return mem
    ins,outs,rules=r18.parse_fis(ReadOnly())
    need(len(ins)==3 and len(outs)==2 and len(rules)==27,
         'La topologia del FIS v03 no es 3-2-27')
    need(sha(FIS)==EXPECTED[FIS],'FIS distinto durante lectura')
    return r18.build_skfuzzy(ins,outs,rules)


def preflight():
    spawn=load_spawn()
    folder,world,route,head=spawn.check_prepared()  # Verificacion ya aprobada en B2 SPAWN.
    for p,d in EXPECTED.items():
        need(p.is_file() and not p.is_symlink() and sha(p)==d,
             'Se modifico un componente protegido: '+str(p))
    need(R18.is_file(),'Falta el parser R18')
    pts=route_points(route)
    rects=collision_rectangles(world)
    c0=clearance(START,rects)
    cm=min(clearance(p,rects) for p in pts)
    need(c0>.35 and cm>.29,'Geometria preparada demasiado estrecha: '+str((c0,cm)))
    need(len(list((folder/'B2_rescate_spawn').glob('*/B2_rescate_spawn_resultado.json')))>=1,
         'No se encuentra comprobacion SPAWN previa')
    valid=[]
    for p in (folder/'B2_rescate_spawn').glob('*/B2_rescate_spawn_resultado.json'):
        try:
            d=json.loads(p.read_text(encoding='utf-8'))
            if d.get('estado')=='SPAWN_VALIDADO_SIN_MOVIMIENTO':valid.append(p)
        except (ValueError,OSError):pass
    need(valid,'El spawn previo no figura como validado')
    # Si existe mundo/base del GAZEBO externo se aborta en spawn.check_prepared.
    need(os.statvfs(str(REPO)).f_bavail*os.statvfs(str(REPO)).f_frsize>700*1024*1024,
         'Espacio libre insuficiente para registrar evidencia')
    say(f'[OK B2 RESCATE INTEGRADO] Spawn aprobado; 35 waypoints; despeje teorico minimo {cm:.3f} m')
    say(f'[OK B2 RESCATE INTEGRADO] HEAD {head[:12]} auditado; A3/A4/FIS intactos')
    return spawn,folder,world,route,pts,rects,head


def monitor(folder,route,world):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan,Range
    from std_msgs.msg import String
    from skfuzzy import control as ctrl

    need(os.environ.get('ROS_DOMAIN_ID')==DOM and os.environ.get('ROS_LOCALHOST_ONLY')=='1',
         'Dominio ROS 120 aislado requerido')
    pts=route_points(route);rects=collision_rectangles(world)
    for p,d in EXPECTED.items():need(sha(p)==d,'Componente protegido distinto: '+str(p))
    a3=import_file(A3,'b2_rescate_a3')
    a4=import_file(A4,'b2_rescate_a4')
    a4.ROUTE=route  # SOLO EN MEMORIA. Ruta original A4 intacta.
    loaded=a4.load_route(a4.ROUTE)
    need(len(loaded)==len(pts) and all(eucl(p,q)<.00001 for p,q in zip(pts,loaded)),
         'A4 no cargaria exactamente la ruta de rescate')
    fuzzy_system=parser_fis()
    sim=ctrl.ControlSystemSimulation(fuzzy_system,clip_to_bounds=False,cache=False,lenient=False)
    report={'actividad':'B2.3 RESCATE INTEGRADO', 'estado':'NO_APROBADO',
            'evasion_completa_demostrada':False,'evasion_geometrica_demostrada':False,
            'contactos_gazebo_verificados':False,
            'advertencia_contactos':'No se dispone de sensor Gazebo de contacto; distancia geometrica es una estimacion',
            'mundo':str(world),'ruta':str(route),'ruta_puntos':len(pts),
            'dominio_ros':DOM,'inicio_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
            'meta_m':GOAL,'modos_a3':[], 'eventos_a4':[], 'fallos':[],
            'barreras':{},'muestras_odom':[],'muestras_comandos':[],
            'mensajes_controlador_difuso':0}
    t0=time.monotonic()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    ex=SingleThreadedExecutor()
    nodes=[];bridge=None;status_node=None
    try:
        class Mamdani(Node):
            def __init__(self):
                super().__init__('b2_rescate_mamdani_inalterado')
                self.pub=self.create_publisher(Twist,'/cmd_vel_difuso',10)
                self.data={};self.published=0;self.failures=0
                for key,topic in [('df','/rango_frontal'),('di','/rango_izq'),('dd','/rango_der')]:
                    self.create_subscription(Range,topic,lambda msg,k=key:self.on_range(k,msg),qos_profile_sensor_data)
                self.create_timer(.20,self.tick)
            def on_range(self,k,msg):
                v=float(msg.range)
                if math.isfinite(v) and 0<=v<=2:self.data[k]=(v,time.monotonic())
                else:self.data.pop(k,None)
            def tick(self):
                cmd=Twist();now=time.monotonic()
                if len(self.data)==3 and all(now-t<.9 for _,t in self.data.values()):
                    try:
                        for k in ('df','di','dd'):sim.input[k]=self.data[k][0]
                        sim.compute()
                        v,w=float(sim.output['v']),float(sim.output['omega'])
                        need(math.isfinite(v) and math.isfinite(w) and 0<=v<=MAX_V+.00001
                             and abs(w)<=MAX_W+.00001,'Salida Mamdani fuera de limites')
                        cmd.linear.x=v;cmd.angular.z=w
                        self.published+=1
                    except Exception as e:
                        self.failures+=1
                        report['fallos'].append('MAMDANI '+str(e))
                self.pub.publish(cmd)

        class Bridge(Node):
            def __init__(self):
                super().__init__('b2_rescate_unico_puente_cmd_vel')
                self.pub=self.create_publisher(Twist,'/cmd_vel',10)
                self.to_route=self.create_publisher(Twist,'/cmd_vel_ruta',10)
                self.create_subscription(Twist,a4.OUT,self.route_cb,10)
                self.create_subscription(Twist,a3.OUT,self.a3_cb,10)
                self.create_subscription(String,a3.STATUS,self.status_cb,10)
                self.create_subscription(Odometry,'/odom',self.odom_cb,qos_profile_sensor_data)
                self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
                for key,topic in [('df','/rango_frontal'),('di','/rango_izq'),('dd','/rango_der')]:
                    self.create_subscription(Range,topic,lambda msg,k=key:self.range_cb(k,msg),qos_profile_sensor_data)
                self.create_timer(.10,self.tick)
                self.armed=False;self.stop_requested=False;self.failure=None
                self.cmd=None;self.cmd_at=0.;self.status=None;self.status_at=0.
                self.pose=None;self.yaw=None;self.odom_at=0.;self.sensors={}
                self.route_count=0;self.last_odom=None;self.path_len=0.
                self.min_clear=float('inf');self.max_dev=0.;self.last_progress=time.monotonic()
                self.best_goal_dist=eucl(START,GOAL)
                self.last_valid=0.;self.modes=[];self.terminal=False
                self.nonzero=0;self.blocked={};self.n_cmd=0
                self.max_pubs=0
            def route_cb(self,msg):
                if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
                    self.to_route.publish(msg);self.route_count+=1
            def a3_cb(self,msg):
                self.cmd=(float(msg.linear.x),float(msg.angular.z))
                self.cmd_at=time.monotonic()
            def status_cb(self,msg):
                try:s=json.loads(msg.data)
                except (ValueError,TypeError):return
                if s.get('modo') in ('RUTA','DIFUSO','CERO'):
                    self.status=s;self.status_at=time.monotonic()
                    if not self.modes or self.modes[-1].get('modo')!=s['modo']:
                        change={'t_s':round(time.monotonic()-t0,2),'modo':s['modo'],
                                'motivo':s.get('motivo')}
                        self.modes.append(change)
                        report['modos_a3'].append(change)
            def odom_cb(self,msg):
                p=msg.pose.pose.position;q=msg.pose.pose.orientation
                xyz=(float(p.x),float(p.y))
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                if not all(map(math.isfinite,(*xyz,yaw))):return
                now=time.monotonic()
                if self.last_odom is not None:
                    d=eucl(xyz,self.last_odom)
                    if d<.05:self.path_len+=d
                self.last_odom=xyz;self.pose=xyz;self.yaw=yaw;self.odom_at=now
                self.min_clear=min(self.min_clear,clearance(xyz,rects))
                self.max_dev=max(self.max_dev,nearest_distance(xyz,pts))
                dist=eucl(xyz,GOAL)
                if self.armed and dist<self.best_goal_dist-.035:
                    self.best_goal_dist=dist;self.last_progress=now
                if len(report['muestras_odom'])<10000:
                    report['muestras_odom'].append((round(now-t0,3),round(xyz[0],5),
                        round(xyz[1],5),round(yaw,5),round(dist,4),
                        round(clearance(xyz,rects),4),round(nearest_distance(xyz,pts),4)))
            def scan_cb(self,msg):
                vals=[]
                for i,d in enumerate(msg.ranges):
                    ang=wrap(msg.angle_min+i*msg.angle_increment)
                    if abs(ang)>math.radians(25):continue
                    if math.isinf(d) and d>0 and math.isfinite(msg.range_max):d=msg.range_max
                    if math.isfinite(d) and msg.range_min<=d<=msg.range_max:vals.append(float(d))
                self.sensors['scan']=(min(vals) if vals else None,time.monotonic())
            def range_cb(self,key,msg):
                v=float(msg.range);self.sensors[key]=(v,time.monotonic())
            def raise_safe(self,m):
                if self.failure is None:self.failure=m
                self.blocked[m]=self.blocked.get(m,0)+1
                self.stop_requested=True
            def stop(self):
                self.stop_requested=True;self.armed=False
                for _ in range(8):self.pub.publish(Twist())
            def tick(self):
                now=time.monotonic();safe=Twist();reason='DESARMADO'
                self.max_pubs=max(self.max_pubs,self.count_publishers('/cmd_vel'))
                if self.max_pubs!=1:self.raise_safe('MULTIPLES_PUBLICADORES_CMD_VEL')
                if self.armed and not self.stop_requested:
                    if self.cmd is None or self.status is None or self.pose is None:
                        reason='DATOS_NO_PRESENTES'
                    elif now-self.cmd_at>.7 or now-self.status_at>.7 or now-self.odom_at>.9:
                        reason='DATOS_CADUCADOS'
                    elif set(self.sensors)!={'df','di','dd','scan'} or any(
                            val is None or not math.isfinite(val) or now-ts>.9
                            for val,ts in self.sensors.values()):
                        reason='SENSORES_CADUCADOS'
                    else:
                        v,w=self.cmd;mode=self.status['modo']
                        if not math.isfinite(v) or not math.isfinite(w) or not (0<=v<=MAX_V+.00001) or abs(w)>MAX_W+.00001:
                            self.raise_safe('COMANDO_A3_FUERA_RANGO')
                        elif self.status.get('v') is not None and abs(v-float(self.status['v']))>.002:
                            self.raise_safe('ESTADO_A3_DESACOPLADO_V')
                        elif self.status.get('omega') is not None and abs(w-float(self.status['omega']))>.002:
                            self.raise_safe('ESTADO_A3_DESACOPLADO_W')
                        elif clearance(self.pose,rects)<=STOP_GEOM:
                            self.raise_safe('DISTANCIA_GEOMETRICA_CRITICA')
                        elif nearest_distance(self.pose,pts)>MAX_ROUTE_DEV:
                            self.raise_safe('DESVIACION_MAXIMA_ASTAR')
                        elif min(self.sensors['scan'][0],self.sensors['df'][0])<=STOP_FRONT:
                            self.raise_safe('FRENO_FRONTAL_ADICIONAL')
                        else:
                            predicted=projected_xy(*self.pose,self.yaw,v,w,.35)
                            if clearance(predicted,rects)<=STOP_GEOM:
                                self.raise_safe('COLISION_PROYECTADA')
                            else:
                                safe.linear.x=v;safe.angular.z=w
                                reason=mode
                                self.last_valid=now
                                if abs(v)>.005 or abs(w)>.025:self.nonzero+=1
                if self.armed and self.last_valid and now-self.last_valid>2.0:
                    self.raise_safe('FALTAN_DATOS_VALIDOS_2S')
                if self.stop_requested or not self.armed:safe=Twist()
                self.pub.publish(safe);self.n_cmd+=1
                if len(report['muestras_comandos'])<10000:
                    dist=eucl(self.pose,GOAL) if self.pose else None
                    df=self.sensors.get('df',(None,0))[0]
                    fr=self.sensors.get('scan',(None,0))[0]
                    report['muestras_comandos'].append((round(now-t0,3),
                        self.status.get('modo','') if self.status else '',
                        round(float(safe.linear.x),5),round(float(safe.angular.z),5),
                        reason, round(df,4) if df is not None else None,
                        round(fr,4) if fr is not None else None,
                        round(dist,4) if dist is not None else None))
        class EventObserver(Node):
            def __init__(self):
                super().__init__('b2_rescate_observador_a4')
                self.events=[]
                self.create_subscription(String,a4.STATUS,self.cb,10)
            def cb(self,msg):
                try:d=json.loads(msg.data)
                except (ValueError,TypeError):return
                state=d.get('estado')
                if state and (not self.events or self.events[-1]!=state):
                    self.events.append(state)
                    report['eventos_a4'].append({'t_s':round(time.monotonic()-t0,2),
                          'estado':state,'progreso_m':d.get('progreso_m')})
        fuzzy=Mamdani();bridge=Bridge();status_node=EventObserver()
        node_a3=a3.ros_node_class()();node_a4=a4.make_follower_node()()
        nodes=[fuzzy,bridge,status_node,node_a3,node_a4]
        for n in nodes:ex.add_node(n)
        need(len(node_a4.core.path.pts)==35,
             'A4 inicializo ruta diferente; movimiento prohibido')
        def spin(sec=.05):
            ex.spin_once(timeout_sec=sec)
            if bridge and bridge.failure:raise RuntimeError(bridge.failure)
            if time.monotonic()-t0>MAX_WALL_S+55:
                raise RuntimeError('TIMEOUT_GLOBAL')
        deadline=time.monotonic()+40
        while time.monotonic()<deadline:
            spin()
            if (bridge.pose and bridge.yaw is not None and bridge.status and bridge.cmd
                and bridge.route_count>=3 and fuzzy.published>=3
                and len(bridge.sensors)==4):break
        else:raise RuntimeError('NO_HAY_SENSORES_A3_A4_EN_40S')
        need(eucl(bridge.pose,START)<.08 and abs(wrap(bridge.yaw-YAW))<.15,
             'Robot fuera de posicion inicial antes de armar')
        need(bridge.sensors['scan'][0]>.90 and bridge.sensors['df'][0]>.90,
             'Sensor frontal detecta obstaculo demasiado cercano al inicio')
        need(bridge.count_publishers('/cmd_vel')==1,
             'Existe otro publicador /cmd_vel')
        need(bridge.max_dev<.10 and bridge.min_clear>.30,
             'Spawn incompatible con ruta o geometria')
        bridge.armed=True;bridge.last_valid=time.monotonic();bridge.last_progress=time.monotonic()
        say('[OK B2 RESCATE] Mamdani+A3+A4 listos; /cmd_vel exclusivo; navegacion habilitada')
        begin=time.monotonic(); last_print=begin
        reached=False
        while time.monotonic()-begin<MAX_WALL_S:
            spin()
            now=time.monotonic()
            if bridge.pose is None:continue
            dist=eucl(bridge.pose,GOAL)
            if now-last_print>=8:
                say(f'[B2] t={now-begin:.1f}s; meta={dist:.3f}m; avance={bridge.path_len:.2f}m; '
                    f'A3={bridge.status.get("modo") if bridge.status else "?"}; '
                    f'despeje_min={bridge.min_clear:.3f}m')
                last_print=now
            if dist<=.15:
                # Confirmar que A4 llega cerca de final y que los datos siguen vigentes.
                if bridge.max_dev<=MAX_ROUTE_DEV and bridge.nonzero>=10:
                    reached=True;break
            if now-bridge.last_progress>MAX_IDLE_S:
                raise RuntimeError('SIN_PROGRESO_HACIA_META_'+str(round(MAX_IDLE_S))+'S')
        if not reached:raise RuntimeError('NO_LLEGO_META_EN_220S')
        bridge.stop()
        for _ in range(10):ex.spin_once(timeout_sec=.05)
        need(bridge.min_clear>STOP_GEOM,'Distancia geometrica debajo del minimo')
        need(bridge.max_dev<=MAX_ROUTE_DEV,'Error lateral maximo excedido')
        need(bridge.max_pubs==1,'Publicador unico perdido')
        report['estado']='LLEGADA_VALIDADA_GEOMETRICAMENTE'
        report['evasion_geometrica_demostrada']=True
        say(f'[OK B2 RESCATE] LLEGADA: error={eucl(bridge.pose,GOAL):.3f} m; '
            f'despeje_min={bridge.min_clear:.3f} m; recorrido={bridge.path_len:.3f} m')
        return 0
    except Exception as e:
        report['fallos'].append(str(e))
        say('[ERROR B2 RESCATE INTEGRADO] '+str(e))
        return 2
    finally:
        if bridge is not None:
            bridge.stop()
            for _ in range(12):
                try:ex.spin_once(timeout_sec=.04)
                except Exception:break
        if bridge is not None:
            report.update({'n_publicaciones_cmd_vel':bridge.n_cmd,
                'n_propuestas_ruta_a4':bridge.route_count,
                'n_ordenes_activas':bridge.nonzero,
                'distancia_recorrida_m':round(bridge.path_len,4),
                'distancia_minima_a_colisiones_sdf_m':round(bridge.min_clear,4) if math.isfinite(bridge.min_clear) else None,
                'desviacion_max_a_ruta_m':round(bridge.max_dev,4),
                'distancia_final_meta_m':round(eucl(bridge.pose,GOAL),4) if bridge.pose else None,
                'num_publicadores_cmd_vel_max':bridge.max_pubs,
                'barreras':bridge.blocked})
        if 'fuzzy' in locals():
            report['mensajes_controlador_difuso']=fuzzy.published
            report['fallos_controlador_difuso']=fuzzy.failures
        for n in reversed(nodes):
            try:ex.remove_node(n);n.destroy_node()
            except Exception:pass
        try:ex.shutdown()
        except Exception:pass
        if rclpy.ok():rclpy.shutdown()
        od=report.pop('muestras_odom');commands=report.pop('muestras_comandos')
        folder.mkdir(parents=True,exist_ok=True)
        for file,header,rows in [
            ('odometria.csv',['t_s','x_m','y_m','yaw_rad','distancia_meta_m','despeje_geometrico_m','error_ruta_m'],od),
            ('control.csv',['t_s','modo_a3','v_mps','omega_radps','razon','df_m','scan_frontal_m','distancia_meta_m'],commands)]:
            with (folder/file).open('w',newline='',encoding='utf-8') as f:
                w=csv.writer(f);w.writerow(header);w.writerows(rows)
        report['inicio']=[*START,YAW]
        report['fin_utc']=dt.datetime.now(dt.timezone.utc).isoformat()
        (folder/'resumen.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        say('[EVIDENCIA] '+str(folder/'resumen.json'))


def bag_stats(folder):
    dbs=list((folder/'rosbag2').glob('*.db3'))
    need(dbs,'No hay bag sqlite')
    counts={}
    for file in dbs:
        need(file.stat().st_size>4096,'Rosbag vacia')
        con=sqlite3.connect(f'file:{file}?mode=ro',uri=True,timeout=10)
        try:
            for name,count in con.execute('SELECT t.name,COUNT(m.id) FROM topics t LEFT JOIN messages m ON t.id=m.topic_id GROUP BY t.name'):
                counts[name]=counts.get(name,0)+count
        finally:con.close()
    minimum={'/odom':15,'/scan':10,'/cmd_vel':10,'/cmd_vel_ruta':5,
             '/cmd_vel_difuso':5,'/cmd_vel_a3_prueba':5}
    return counts,all(counts.get(k,0)>=n for k,n in minimum.items())


def execute():
    spawn,base,world,route,pts,rects,head=preflight()
    original=spawn.git('status','--porcelain=v1','--untracked-files=no')
    need(subprocess.run(spawn.worker(['--probe','dominio']),cwd=REPO,env=spawn.env(),timeout=30).returncode==0,
         'Dominio 120 no aislado')
    run=base/'B2_rescate_integrado'/dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    run.mkdir(parents=True,exist_ok=False)
    processes=[];files=[]
    def own(name,cmd):
        h=(run/(name+'.log')).open('w',encoding='utf-8');files.append(h)
        p=subprocess.Popen(cmd,cwd=REPO,env=spawn.env(),stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((name,p));return p
    worker_rc=None
    try:
        server=own('gazebo',spawn.source_command(['gzserver','--verbose','-s','libgazebo_ros_init.so',
                                                    '-s','libgazebo_ros_factory.so',world]))
        factory=own('factory',spawn.worker(['--probe','factory']))
        factory.wait(timeout=65)
        need(server.poll() is None and factory.returncode==0,'Factory Gazebo no disponible')
        pkg=subprocess.run(spawn.source_command(['ros2','pkg','prefix','turtlebot3_gazebo']),cwd=REPO,
                           env=spawn.env(),text=True,capture_output=True,timeout=25)
        need(pkg.returncode==0 and pkg.stdout.strip(),'TurtleBot3 no encontrado')
        model=Path(pkg.stdout.strip())/'share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf'
        need(model.is_file(),'Modelo SDF TurtleBot3 ausente')
        robot=own('spawn',spawn.source_command(['ros2','run','gazebo_ros','spawn_entity.py',
                     '-entity','waffle_pi','-file',model,'-x',str(START[0]),'-y',str(START[1]),
                     '-z','0.05','-Y',str(YAW)]))
        robot.wait(timeout=65)
        need(server.poll() is None and robot.returncode==0,'Spawn no confirmado')
        say('[OK B2 RESCATE] Gazebo y TurtleBot3 iniciados en geometria validada')
        emulator=own('emulador',spawn.source_command([spawn.PYTHON_ROS,REPO/'tools/emulador_sensores_f1.py',
                             '--ros-args','-p','use_sim_time:=true']))
        bag=own('rosbag',spawn.source_command(['ros2','bag','record','--storage','sqlite3',
                   '-o',run/'rosbag2',*TOPICS]))
        time.sleep(2.)
        need(emulator.poll() is None and bag.poll() is None,'Emulador o ROS bag no arrancan')
        control=own('navegacion',spawn.source_command([spawn.PYTHON_ROS,SELF,'--monitor',str(run),str(route),str(world)]))
        last=0.
        while control.poll() is None:
            need(server.poll() is None and emulator.poll() is None and bag.poll() is None,
                 'Se cerro prematuramente Gazebo, emulador o rosbag')
            if time.monotonic()-last>15:
                say('[B2] Ensayo en curso. Evidencias: '+str(run))
                last=time.monotonic()
            time.sleep(.35)
        worker_rc=control.returncode
        for line in (run/'navegacion.log').read_text(encoding='utf-8',errors='replace').splitlines()[-30:]:
            say(line)
    finally:
        for _,p in reversed(processes):spawn.stop_owned(p)
        for h in files:h.close()
        need(spawn.git('status','--porcelain=v1','--untracked-files=no')==original,
             'Cambios en archivos rastreados por Git; revisar')
        for p,d in EXPECTED.items():need(sha(p)==d,'Archivo protegido cambiado: '+str(p))
        say('[OK] Se cerraron solamente procesos iniciados por el ensayo')
    summary=run/'resumen.json'
    need(summary.is_file(),'No se genero resumen (revisar navegacion.log)')
    report=json.loads(summary.read_text(encoding='utf-8'))
    try:
        counts,bags_ok=bag_stats(run)
    except Exception as e:
        counts={};bags_ok=False;report.setdefault('fallos',[]).append('ROSBAG '+str(e))
    report['rosbag_conteos']=counts;report['rosbag_valido']=bags_ok
    report['head_git']=head
    report['hashes_sha256']={str(p.relative_to(REPO)):d for p,d in EXPECTED.items()}
    if worker_rc==0 and bags_ok and report.get('estado')=='LLEGADA_VALIDADA_GEOMETRICAMENTE':
        report['estado']='LLEGADA_Y_ROSBAG_VALIDADOS_CONTACTOS_NO_MEDIDOS'
        say('[OK B2 RESCATE] LLEGADA + ROSBAG VALIDADO; contacto Gazebo no observado directamente')
        rc=0
    else:
        say('[NO APROBADO B2] Movimiento/ruta o rosbag no cumplieron; evidencias preservadas para diagnostico')
        rc=2
    summary.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    say('[EVIDENCIA FINAL] '+str(summary))
    return rc


def main():
    p=argparse.ArgumentParser(description=__doc__)
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument('--preflight',action='store_true')
    g.add_argument('--ensayar',action='store_true')
    g.add_argument('--monitor',nargs=3,metavar=('CARPETA','RUTA','MUNDO'),help=argparse.SUPPRESS)
    args=p.parse_args()
    if args.monitor:return monitor(*map(Path,args.monitor))
    if args.preflight:preflight();say('[OK] Preflight aprobado, ningun proceso iniciado');return 0
    return execute()

if __name__=='__main__':
    try:sys.exit(main())
    except Exception as e:
        say('[ERROR B2 RESCATE INTEGRADO] '+str(e));sys.exit(2)
