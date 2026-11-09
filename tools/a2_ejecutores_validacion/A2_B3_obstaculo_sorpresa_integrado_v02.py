#!/usr/bin/env python3
"""B3 v02 - ensayo piloto de APARICION sorpresa; control HIBRIDO ASISTIDO.

Reutiliza el banco B2 validado, SIN programar previamente el obstaculo
en la ruta A*. Se retira la caja central de una COPIA TEMPORAL del mundo
y se planifica una ruta libre inicial. Despues de avanzar >=0.25 m,
la navegacion se pausa para insertar una caja en Gazebo 1.00 m por delante
(de superficie a centro del robot; se mide LiDAR tras el spawn).
La pausa transitoria es una medida de seguridad, no movimiento continuo.
El obstaculo permanecera en Gazebo hasta el fin del ensayo.

Solo un puente publica /cmd_vel. Mamdani/A3/A4 y Git INTACTOS.
Al detectar DIFUSO, el puente RETIENE la maniobra frente a la caja, y
usa subobjetivos geométricos temporales generados DESPUES del spawn,
con velocidad limitada por el motor Mamdani y orientación asistida.
No se atribuye al Mamdani SOLO la evasión completa: es una prueba híbrida.
Las barreras de colisión originales son obligatorias; se añaden muestreos
intermedios para impedir trayectoria insegura. No se mide contacto Gazebo.

WSL:
  python3 /mnt/c/Users/Henry/Downloads/A2_B3_obstaculo_sorpresa_integrado_v02.py --preflight
  python3 /mnt/c/Users/Henry/Downloads/A2_B3_obstaculo_sorpresa_integrado_v02.py --ensayar
"""
from __future__ import annotations
import argparse, csv, datetime as dt, hashlib, heapq, importlib.util, json, math, os
from pathlib import Path
import signal, sqlite3, subprocess, sys, tempfile, time, xml.etree.ElementTree as ET

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
B3_NAME = 'a2_b3_obstaculo_sorpresa'
B3_BOX = (0.20, 0.20, 0.42)
B3_SURFACE_GAP = 1.00  # objetivo a partir del CENTRO del robot, cara frontal
TRIGGER_TRAVEL = 0.25  # inyeccion despues de >=0.25 m de avance real
SPAWN_WAIT_S = 12.0
SPAWN_SCAN_WAIT_S = 9.0
MAX_IDLE_S = 45.0
STOP_GEOM = 0.255  # 0.21 m de inflado mas 0.045 de reserva experimental
STOP_FRONT = 0.23
MAX_ROUTE_DEV = 0.55  # obstaculo no mapeado requiere desvio lateral; STOP_GEOM sigue intacto
MAX_V, MAX_W = 0.20, 1.5
# Política experimental v02, NO modifica A3/A4/FIS ni el A* inicial.
BYPASS_ENTER_M = 0.68              # activar solo tras A3=DIFUSO y LiDAR confirma
BYPASS_RELEASE_M = 0.80            # histéresis de salida + final geométrico
BYPASS_LATERAL_M = 0.45           # corredor de 1.6 m: despeje ~0.35 a ambas paredes/caja
BYPASS_BEFORE_M = 0.32           # distancia longitudinal antes de la cara frontal
BYPASS_AFTER_M = 0.34            # distancia longitudinal después de la cara posterior
BYPASS_REJOIN_M = 0.42
BYPASS_MAX_V = 0.055
BYPASS_MAX_W = 0.70
BYPASS_GOAL_TOL_M = 0.080
BYPASS_ADDITIONAL_RESERVE = 0.040
BYPASS_STEP_S = 0.35
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
    need(len(points)==41 and eucl(points[0],START)<.006 and eucl(points[-1],GOAL)<.006,
         'B3 requiere ruta A* inicial recta y 41 puntos (2m / 0.05m)')
    need(all(eucl(a,b)<.15 for a,b in zip(points,points[1:])),
         'Ruta A* discontinua para A4')
    need(all(abs(y-START[1])<1e-6 for _,y in points),
         'B3 ya contiene desvio en ruta; invalidaria aparicion sorpresa')
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
    need(not any(r[0] in ('a2_b2_obstaculo_estatico_centro',B3_NAME) for r in rects),
         'B3 debe iniciar SIN la caja central ni la caja sorpresa')
    need(any(r[0]=='a2_b2_pared_norte' for r in rects) and
         any(r[0]=='a2_b2_pared_sur' for r in rects), 'Faltan paredes del pasillo B3')
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


def sample_segment(a,b,rects,pts,step=0.020):
    """Despeje minimo comprobando TODA la recta, no solo sus extremos."""
    n=max(1,math.ceil(eucl(a,b)/step))
    values=[]
    for i in range(n+1):
        t=i/n;p=(a[0]*(1-t)+b[0]*t,a[1]*(1-t)+b[1]*t)
        values.append((clearance(p,rects),nearest_distance(p,pts)))
    return min(c for c,d in values),max(d for c,d in values)


def build_dynamic_detour(pose,box,rects,pts,preferred_y_sign=-1):
    """Subobjetivos solo DESPUES del spawn, sin tocar ruta A* original.

    Preferencia de lado alineada con el primer giro difuso; si la geometria
    no valida, se prueba el otro lado. No se desactiva ninguna barrera.
    """
    need(box[0]==B3_NAME,'Modelo dinámico no corresponde a B3')
    for side in (preferred_y_sign,-preferred_y_sign):
        lane_y=START[1]+side*BYPASS_LATERAL_M
        wp=[(box[2]+BYPASS_BEFORE_M,lane_y),
            (box[1]-BYPASS_AFTER_M,lane_y),
            (box[1]-BYPASS_REJOIN_M,START[1])]
        path=[pose,*wp,GOAL]
        clearances=[];deviations=[]
        for a,b in zip(path,path[1:]):
            c,d=sample_segment(a,b,rects,pts)
            clearances.append(c);deviations.append(d)
        minimum=min(clearances);maximum=max(deviations)
        if (minimum>STOP_GEOM+BYPASS_ADDITIONAL_RESERVE
                and maximum<MAX_ROUTE_DEV-.015):
            return wp,side,minimum,maximum
    raise RuntimeError('B3_SIN_DESVIO_GEOMETRICO_SEGURO_A_MARGENES_EXISTENTES')


def guidance_to_waypoint(pos,yaw,target,fuzzy_v,fuzzy_w):
    """Control geométrico asistido: Mamdani limita la velocidad y participa
    solo cuando su giro coincide con el rumbo seguro. Nunca supera límites.
    """
    desired=math.atan2(target[1]-pos[1],target[0]-pos[0])
    err=wrap(desired-yaw)
    w_heading=max(-BYPASS_MAX_W,min(BYPASS_MAX_W,2.2*err))
    w_fuzzy=max(-BYPASS_MAX_W,min(BYPASS_MAX_W,fuzzy_w))
    w=(.90*w_heading+.10*w_fuzzy) if w_heading*w_fuzzy>0 else w_heading
    if abs(err)>.35:
        v=0.0             # gira en sitio antes de avanzar hacia un subobjetivo
    else:
        v=min(BYPASS_MAX_V,max(0.,fuzzy_v)) * max(.3,1.-abs(err)/.70)
    return v,w,err


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
    base,b2world,b2route,head=spawn.check_prepared()  # Conserva precondiciones B2, hashes y rama.
    for p,d in EXPECTED.items():
        need(p.is_file() and not p.is_symlink() and sha(p)==d,
             'Se modifico un componente protegido: '+str(p))
    need(R18.is_file(),'Falta parser R18')
    # Verificar la base B2 ya aprobada; no reejecutarla.
    summaries=sorted((base/'B2_rescate_integrado').glob('*/resumen.json'))
    valid=[]
    for p in summaries:
        try:
            j=json.loads(p.read_text(encoding='utf-8'))
            if j.get('estado')=='LLEGADA_Y_ROSBAG_VALIDADOS_CONTACTOS_NO_MEDIDOS' \
                    and j.get('rosbag_valido') is True: valid.append(p)
        except (ValueError,OSError):pass
    need(len(valid)>=5,'B2 no tiene cinco llegadas + rosbags verificadas')
    need(os.statvfs(str(REPO)).f_bavail*os.statvfs(str(REPO)).f_frsize>700*1024*1024,
         'Espacio libre insuficiente')
    say(f'[OK B3] B2 cerrado con {len(valid)} evidencias, sin repetir sus pruebas')
    say('[OK B3] Componentes A3/A4/Mamdani por SHA256, rama Git y espacio auditados')
    say(f'[OK B3] HEAD {head[:12]} (no se exige HEAD historico)')
    return spawn,base,b2world,head


def make_b3_assets(b2world,run):
    """Solo escribe en carpeta NUEVA; mundo/mapa original y B2 intactos."""
    rt=ET.parse(b2world)
    root=rt.getroot()
    world_el=root.find('world')
    need(world_el is not None, 'El SDF no contiene world')
    matches=[m for m in world_el.findall('model')
             if m.get('name')=='a2_b2_obstaculo_estatico_centro']
    need(len(matches)==1,'B2 no tiene exactamente una caja central')
    world_el.remove(matches[0])
    newworld=run/'B3_mundo_inicial_sin_obstaculo_sorpresa.world'
    rt.write(newworld,encoding='utf-8',xml_declaration=True)
    # El SDF NO contiene obstaculo B3. Su insercion por servicio se audita en ejecucion.
    rects=collision_rectangles(newworld)
    x0,y0,res,nx,ny=-5.,-4.,.05,200,160
    def xy(i,j):return (x0+(i+.5)*res,y0+(ny-j-.5)*res)
    def cell(p):
        x,y=p
        i=round((x-x0)/res-.5)
        j=ny-1-round((y-y0)/res-.5)
        need(0<=i<nx and 0<=j<ny, 'Inicio o fin fuera de malla')
        need(eucl(xy(i,j),p)<1e-6,'Inicio/fin no coincide con celda')
        return i,j
    def clear_to_rect(p,r):
        x,y=p;_,a,b,c,d=r
        return math.hypot(max(a-x,0.,x-b),max(c-y,0.,y-d))
    data=bytearray(nx*ny)
    for j in range(ny):
        for i in range(nx):
            p=xy(i,j)
            data[j*nx+i]=254 if min(clear_to_rect(p,r) for r in rects)>.21+1e-9 else 0
    def free(p):
        i,j=p
        return 0<=i<nx and 0<=j<ny and data[j*nx+i]==254
    start,goal=cell(START),cell(GOAL)
    need(free(start) and free(goal), 'B3 inicio/meta bloqueados en mapa inflado')
    h=lambda p:res*(abs(p[0]-goal[0])+abs(p[1]-goal[1]))
    queue=[(h(start),0,start)];scores={start:0};pred={};counter=0
    while queue:
        _,cost,cur=heapq.heappop(queue)
        if cost>scores.get(cur,float('inf'))+1e-8:continue
        if cur==goal:break
        for di,dj in ((-1,0),(0,-1),(0,1),(1,0)):
            nbr=(cur[0]+di,cur[1]+dj)
            if not free(nbr):continue
            d=cost+res
            if d+1e-9<scores.get(nbr,float('inf')):
                scores[nbr]=d;pred[nbr]=cur;counter+=1
                heapq.heappush(queue,(d+h(nbr),d,nbr))
    need(goal in scores,'A* no encuentra camino inicial libre')
    seq=[goal]
    while seq[-1]!=start:seq.append(pred[seq[-1]])
    seq.reverse();points=[xy(*p) for p in seq]
    need(len(points)==41 and all(abs(y-START[1])<1e-6 for _,y in points),
         'A* no obtuvo camino inicial recto de 2 m; revisar escenario')
    route=run/'B3_ruta_Astar_sin_obstaculo.csv'
    with route.open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(('x_m','y_m'))
        for x,y in points:w.writerow((f'{x:.3f}',f'{y:.3f}'))
    p=run/'B3_mapa_inicial_inflado.pgm'
    p.write_bytes(f'P5\n{nx} {ny}\n255\n'.encode('ascii')+bytes(data))
    (run/'B3_mapa_inicial_inflado.yaml').write_text(
        'image: B3_mapa_inicial_inflado.pgm\nmode: trinary\nresolution: 0.05\n'
        'origin: [-5.0, -4.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',
        encoding='utf-8')
    route_points(route)
    (run/'B3_manifest.json').write_text(json.dumps({
        'estado':'B3_INICIAL_PREPARADO_OBSTACULO_AUN_NO_EXISTE',
        'mundo_inicial':str(newworld),'sha256_mundo':sha(newworld),
        'ruta_Astar_libre':str(route),'sha256_ruta':sha(route),
        'ruta_num_puntos':len(points),'distancia_Astar_m':round(scores[goal],3),
        'centro_robot_antes_de_spawn':START,'meta':GOAL,
        'evento_obstaculo':'despues de avanzar 0.25m, a ~1.0m superficie frente al robot',
        'NOTA':'El obstaculo NO fue introducido en mapa ni ruta A* iniciales',
    },indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    say(f'[OK B3] Mundo sin caja sorpresa; mapa inflado y A* inicial: {len(points)} puntos / {scores[goal]:.3f}m')
    return newworld,route,rects


def dynamic_sdf():
    sx,sy,sz=B3_BOX
    return (f'<sdf version="1.6"><model name="{B3_NAME}"><static>true</static>'
            f'<link name="body"><pose>0 0 {sz/2:.4f} 0 0 0</pose>'
            f'<collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>'
            f'<visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
            '<material><ambient>0.95 0.22 0.12 1</ambient><diffuse>0.95 0.22 0.12 1</diffuse></material>'
            '</visual></link></model></sdf>')


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
    from gazebo_msgs.srv import SpawnEntity
    from geometry_msgs.msg import Pose
    from skfuzzy import control as ctrl

    need(os.environ.get('ROS_DOMAIN_ID')==DOM and os.environ.get('ROS_LOCALHOST_ONLY')=='1',
         'Dominio ROS 120 aislado requerido')
    pts=route_points(route);rects=collision_rectangles(world)
    dynamic_rects=[]
    def all_rects():return rects+dynamic_rects
    for p,d in EXPECTED.items():need(sha(p)==d,'Componente protegido distinto: '+str(p))
    a3=import_file(A3,'b2_rescate_a3')
    a4=import_file(A4,'b2_rescate_a4')
    a4.ROUTE=route  # SOLO EN MEMORIA. Ruta original A4 intacta.
    loaded=a4.load_route(a4.ROUTE)
    need(len(loaded)==len(pts) and all(eucl(p,q)<.00001 for p,q in zip(pts,loaded)),
         'A4 no cargaria exactamente la ruta de rescate')
    fuzzy_system=parser_fis()
    sim=ctrl.ControlSystemSimulation(fuzzy_system,clip_to_bounds=False,cache=False,lenient=False)
    report={'actividad':'B3 APARICION DE OBSTACULO 1M','estado':'NO_APROBADO',
            'evasion_completa_demostrada':False,'evasion_reactiva_demostrada':False,
            'obstaculo_dinamico_insertado':False,'distancia_a_superficie_esperada_m':B3_SURFACE_GAP,
            'contactos_gazebo_verificados':False,
            'advertencia_contactos':'No se dispone de sensor Gazebo de contacto; distancia geometrica es una estimacion',
            'mundo':str(world),'ruta':str(route),'ruta_puntos':len(pts),
            'dominio_ros':DOM,'inicio_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
            'meta_m':GOAL,'modos_a3':[], 'eventos_a4':[], 'fallos':[],
            'barreras':{},'muestras_odom':[],'muestras_comandos':[],
            'mensajes_controlador_difuso':0, 'modalidad':'HIBRIDA_ASISTIDA_NO_MAMDANI_PURO',
            'politica_arbitraje':'RETENCION_LIDAR_SUBOBJETIVOS_TEMPORALES_Y_FIS_VELOCIDAD',
            'eventos_arbitraje_b3':[], 'bypass_completado':False}
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
                self.create_subscription(Twist,'/cmd_vel_difuso',self.fuzzy_cb,10)
                self.create_subscription(String,a3.STATUS,self.status_cb,10)
                self.create_subscription(Odometry,'/odom',self.odom_cb,qos_profile_sensor_data)
                self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
                for key,topic in [('df','/rango_frontal'),('di','/rango_izq'),('dd','/rango_der')]:
                    self.create_subscription(Range,topic,lambda msg,k=key:self.range_cb(k,msg),qos_profile_sensor_data)
                self.create_timer(.10,self.tick)
                self.armed=False;self.stop_requested=False;self.failure=None
                self.spawn_hold=False;self.spawned=False;self.fuzzy_move_after_spawn=0
                self.route_after_fuzzy=0;self.first_fuzzy_t=None
                self.cmd=None;self.cmd_at=0.;self.status=None;self.status_at=0.
                self.pose=None;self.yaw=None;self.odom_at=0.;self.sensors={}
                self.route_count=0;self.last_odom=None;self.path_len=0.
                self.min_clear=float('inf');self.max_dev=0.;self.last_progress=time.monotonic()
                self.best_goal_dist=eucl(START,GOAL)
                self.last_valid=0.;self.modes=[];self.terminal=False
                self.nonzero=0;self.blocked={};self.n_cmd=0
                self.max_pubs=0
                self.fuzzy_cmd=None;self.fuzzy_at=0.
                self.bypass=[];self.bypass_index=0;self.bypass_started=False
                self.bypass_done=False;self.bypass_side=None;self.bypass_stable=0
                self.bypass_commands=0;self.bypass_turns=0;self.bypass_rejected=0
                self.bypass_fuzzy_blended=0;self.a3_fuzzy_seen_after_spawn=0
                self.bypass_min_pred=float('inf')
            def route_cb(self,msg):
                if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
                    self.to_route.publish(msg);self.route_count+=1
            def fuzzy_cb(self,msg):
                v,w=float(msg.linear.x),float(msg.angular.z)
                if math.isfinite(v) and math.isfinite(w) and 0<=v<=MAX_V+.00001 and abs(w)<=MAX_W+.00001:
                    self.fuzzy_cmd=(v,w);self.fuzzy_at=time.monotonic()
                else:self.fuzzy_cmd=None;self.fuzzy_at=0.
            def a3_cb(self,msg):
                self.cmd=(float(msg.linear.x),float(msg.angular.z))
                self.cmd_at=time.monotonic()
            def status_cb(self,msg):
                try:s=json.loads(msg.data)
                except (ValueError,TypeError):return
                if s.get('modo') in ('RUTA','DIFUSO','CERO'):
                    self.status=s;self.status_at=time.monotonic()
                    if self.spawned and s['modo']=='DIFUSO':
                        self.a3_fuzzy_seen_after_spawn+=1
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
                self.min_clear=min(self.min_clear,clearance(xyz,all_rects()))
                self.max_dev=max(self.max_dev,nearest_distance(xyz,pts))
                dist=eucl(xyz,GOAL)
                if self.armed and dist<self.best_goal_dist-.035:
                    self.best_goal_dist=dist;self.last_progress=now
                if len(report['muestras_odom'])<10000:
                    report['muestras_odom'].append((round(now-t0,3),round(xyz[0],5),
                        round(xyz[1],5),round(yaw,5),round(dist,4),
                        round(clearance(xyz,all_rects()),4),round(nearest_distance(xyz,pts),4)))
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
                if self.armed and not self.stop_requested and not self.spawn_hold:
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
                        elif clearance(self.pose,all_rects())<=STOP_GEOM:
                            self.raise_safe('DISTANCIA_GEOMETRICA_CRITICA')
                        elif nearest_distance(self.pose,pts)>MAX_ROUTE_DEV:
                            self.raise_safe('DESVIACION_MAXIMA_ASTAR')
                        elif min(self.sensors['scan'][0],self.sensors['df'][0])<=STOP_FRONT:
                            self.raise_safe('FRENO_FRONTAL_ADICIONAL')
                        else:
                            # A3 sigue siendo la fuente del estado auditado. El puente
                            # evita los cambios RUTA-DIFUSO cuando /scan confirma caja.
                            df,scan=self.sensors['df'][0],self.sensors['scan'][0]
                            if (self.spawned and not self.bypass_started
                                    and mode=='DIFUSO' and min(df,scan)<=BYPASS_ENTER_M):
                                if (self.fuzzy_cmd is None or now-self.fuzzy_at>.9
                                        or not dynamic_rects):
                                    self.raise_safe('SIN_FIS_VALIDO_PARA_INICIAR_EVASION')
                                else:
                                    side=-1 if self.fuzzy_cmd[1]>=0 else +1
                                    try:
                                        path,side,c,d=build_dynamic_detour(self.pose,dynamic_rects[0],
                                                                          all_rects(),pts,side)
                                        self.bypass=path;self.bypass_side=side
                                        self.bypass_started=True
                                        report['eventos_arbitraje_b3'].append({
                                            't_s':round(now-t0,3),'evento':'RETENCION_EVASION',
                                            'modo_a3_inicial':mode,'scan_m':round(scan,4),
                                            'df_m':round(df,4),'pose':list(self.pose),
                                            'lado':'SUR' if side<0 else 'NORTE',
                                            'subobjetivos':path,
                                            'despeje_min_predicho_m':round(c,4),
                                            'desviacion_max_predicha_m':round(d,4)})
                                        say('[OK B3 v02] EVASION retenida; detour temporal '+
                                            ('SUR' if side<0 else 'NORTE')+
                                            '; despeje previsto '+format(c,'.3f')+'m')
                                    except Exception as exc:
                                        self.raise_safe('EVASION_GEOMETRICA_INVALIDA_'+str(exc))
                            # No se sobrepasa A3=CERO. Sin FIS fresco tampoco
                            # se reemplaza con una orden experimental.
                            if self.bypass_started and not self.bypass_done and not self.stop_requested:
                                if mode=='CERO' and str(self.status.get('motivo','')).startswith('PARADA_'):
                                    self.raise_safe('A3_PARADA_ESTRICTA_DURANTE_EVASION')
                                elif mode not in ('RUTA','DIFUSO'):
                                    v,w=0.,0.;reason='A3_NO_AUTORIZA_MOVIMIENTO'
                                elif self.fuzzy_cmd is None or now-self.fuzzy_at>.9:
                                    v,w=0.,0.;reason='ESPERA_PROPUESTA_FIS_FRESCA'
                                else:
                                    # Despues del ultimo subobjetivo se espera que
                                    # A3 recupere RUTA con LiDAR estable y margen.
                                    while (self.bypass_index<len(self.bypass) and
                                           eucl(self.pose,self.bypass[self.bypass_index])<BYPASS_GOAL_TOL_M):
                                        report['eventos_arbitraje_b3'].append({
                                            't_s':round(now-t0,3),'evento':'SUBOBJETIVO_ALCANZADO',
                                            'indice':self.bypass_index+1,'pose':list(self.pose)})
                                        self.bypass_index+=1
                                    if self.bypass_index==len(self.bypass):
                                        if (mode=='RUTA' and min(df,scan)>BYPASS_RELEASE_M
                                                and clearance(self.pose,all_rects())>STOP_GEOM+.045):
                                            self.bypass_stable+=1
                                        else:self.bypass_stable=0
                                        if self.bypass_stable>=4:
                                            self.bypass_done=True
                                            report['bypass_completado']=True
                                            report['eventos_arbitraje_b3'].append({
                                                't_s':round(now-t0,3),'evento':'REINCORPORACION_A4',
                                                'pose':list(self.pose),'modo_a3':mode,
                                                'scan_m':round(scan,4),'df_m':round(df,4)})
                                            say('[OK B3 v02] Obstaculo superado; retorno estable a A3=RUTA/A4')
                                        else:
                                            v,w=0.,0.;reason='ESPERA_REINCORPORACION_SEGURA'
                                    else:
                                        target=self.bypass[self.bypass_index]
                                        f_v,f_w=self.fuzzy_cmd
                                        v,w,err=guidance_to_waypoint(self.pose,self.yaw,target,f_v,f_w)
                                        reason='B3_EVASION_HIBRIDA_ASISTIDA'
                                        if abs(err)>.35:self.bypass_turns+=1
                                        if v>0 or abs(w)>.025:self.bypass_commands+=1
                                        if f_w*w>0 and abs(f_w)>.02:self.bypass_fuzzy_blended+=1
                                        if mode=='DIFUSO' and (v>.005 or abs(w)>.025):
                                            self.fuzzy_move_after_spawn+=1
                                        if self.first_fuzzy_t is None:self.first_fuzzy_t=now
                            if not self.stop_requested:
                                # Barrera prospectiva: comprobar varios puntos
                                # de la trayectoria de 0 a 0.35s, no solo extremo.
                                min_prediction=min(clearance(
                                    projected_xy(*self.pose,self.yaw,v,w,h),all_rects())
                                    for h in (.07,.14,.21,.28,.35))
                                self.bypass_min_pred=min(self.bypass_min_pred,min_prediction)
                                if min_prediction<=STOP_GEOM:
                                    # Se permite solo giro en sitio si mantiene el
                                    # margen actual y el comando lineal es el riesgo.
                                    if (self.bypass_started and not self.bypass_done and v>0 and
                                            clearance(self.pose,all_rects())>STOP_GEOM+.008):
                                        v=0.
                                        check_turn=min(clearance(
                                            projected_xy(*self.pose,self.yaw,0.,w,h),all_rects())
                                            for h in (.07,.14,.21,.28,.35))
                                        if check_turn>STOP_GEOM:
                                            reason='B3_GIRO_PREVENTIVO_V_CERO'
                                            self.bypass_rejected+=1
                                        else:self.raise_safe('COLISION_PROYECTADA')
                                    else:self.raise_safe('COLISION_PROYECTADA')
                                if not self.stop_requested:
                                    safe.linear.x=v;safe.angular.z=w
                                    if reason=='DESARMADO':reason=mode
                                    self.last_valid=now
                                    if abs(v)>.005 or abs(w)>.025:self.nonzero+=1
                                    if (self.spawned and not self.bypass_started and mode=='DIFUSO'
                                            and (abs(v)>.005 or abs(w)>.025)):
                                        self.fuzzy_move_after_spawn+=1
                                        if self.first_fuzzy_t is None:self.first_fuzzy_t=now
                                    if (self.spawned and self.bypass_done and
                                            self.first_fuzzy_t is not None and mode=='RUTA'):
                                        self.route_after_fuzzy+=1
                if self.armed and not self.spawn_hold and self.last_valid and now-self.last_valid>2.0:
                    self.raise_safe('FALTAN_DATOS_VALIDOS_2S')
                if self.stop_requested or not self.armed or self.spawn_hold:safe=Twist()
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
        need(len(node_a4.core.path.pts)==41,
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
        # Gazebo /spawn_entity disponible, exclusivo en dominio aislado.
        actor=Node('b3_spawn_operador',enable_rosout=False)
        nodes.append(actor);ex.add_node(actor)
        spawn_cli=actor.create_client(SpawnEntity,'/spawn_entity')
        need(spawn_cli.wait_for_service(timeout_sec=12.),'Gazebo /spawn_entity no disponible')
        say('[OK B3] Mamdani+A3+A4 y /cmd_vel exclusivo; movimiento habilitado')
        begin=time.monotonic();last_print=begin
        injected=False;reached=False
        trigger_pose=None
        while time.monotonic()-begin<MAX_WALL_S:
            spin()
            now=time.monotonic()
            if bridge.pose is None:continue
            dist=eucl(bridge.pose,GOAL)
            if now-last_print>=7.0:
                say(f'[B3] t={now-begin:.1f}s; meta={dist:.3f}m; '
                    f'avance={bridge.path_len:.2f}m; A3={bridge.status.get("modo") if bridge.status else "?"}; '
                    f'caja={"INSERTADA" if injected else "AUSENTE"}; '
                    f'despeje_min={bridge.min_clear:.3f}m')
                last_print=now
            # INYECCION EXTERNA. Se detiene momentaneamente el movimiento mientras
            # Gazebo crea la entidad (no debe anticiparse en ruta ni mapa A*).
            if not injected and bridge.path_len>=TRIGGER_TRAVEL:
                need(bridge.pose is not None and bridge.yaw is not None,'Pose no disponible para spawn')
                need(abs(wrap(bridge.yaw-YAW))<.22,'Robot giro inesperadamente antes de crear obstaculo')
                trigger_pose=bridge.pose
                x=trigger_pose[0]+(B3_SURFACE_GAP+B3_BOX[0]/2)*math.cos(bridge.yaw)
                y=trigger_pose[1]+(B3_SURFACE_GAP+B3_BOX[1]/2)*math.sin(bridge.yaw)
                # Poner caja sobre el eje del corredor sin modificar el punto objetivo.
                need(-3.80<x<-2.90 and abs(y-START[1])<.07,
                     'Posicion del obstaculo fuera del pasillo B3: '+str((x,y)))
                rect=(B3_NAME,x-B3_BOX[0]/2,x+B3_BOX[0]/2,
                      y-B3_BOX[1]/2,y+B3_BOX[1]/2)
                need(clearance(trigger_pose,[rect])>=.99,'Distancia inicial insuficiente a la caja')
                need(all(not (min(rect[2],a[2])>max(rect[1],a[1]) and
                                  min(rect[4],a[4])>max(rect[3],a[3])) for a in rects),
                     'Obstaculo nuevo colisionaria con objeto estatico')
                before_scan=bridge.sensors.get('scan',(None,0))[0]
                need(before_scan is not None and before_scan>1.15,
                     'Ya hay obstaculo antes de insercion: scan '+str(before_scan))
                bridge.spawn_hold=True;bridge.last_valid=time.monotonic()
                for _ in range(4):spin()
                req=SpawnEntity.Request();req.name=B3_NAME;req.xml=dynamic_sdf()
                req.robot_namespace='';req.reference_frame='world'
                req.initial_pose=Pose()
                req.initial_pose.position.x=x;req.initial_pose.position.y=y
                req.initial_pose.position.z=0.0
                future=spawn_cli.call_async(req)
                timeout=time.monotonic()+SPAWN_WAIT_S
                while not future.done() and time.monotonic()<timeout:spin()
                need(future.done(),'TIMEOUT_SPAWN_OBSTACULO_GAZEBO')
                response=future.result()
                need(response is not None and response.success,
                     'SPAWN_GAZEBO_RECHAZADO_'+str(response.status_message if response else 'sin_respuesta'))
                dynamic_rects.append(rect)
                injected=True;bridge.spawned=True
                report['obstaculo_dinamico_insertado']=True
                report['obstaculo_evento']={'t_s':round(time.monotonic()-t0,3),
                     'pose_robot':list(trigger_pose),'pose_caja': [round(x,5),round(y,5)],
                     'desplazamiento_antes_de_insertar_m':round(bridge.path_len,4),
                     'distancia_centro_robot_a_superficie_caja_m':round(clearance(trigger_pose,[rect]),4),
                     'scan_previo_m':before_scan,'respuesta_gazebo':response.status_message,
                     'pausa_movimiento_durante_spawn':True}
                # Esperar confirmacion del LiDAR, no confiar solo en respuesta de servicio.
                look_until=time.monotonic()+SPAWN_SCAN_WAIT_S
                while time.monotonic()<look_until:
                    spin()
                    scan=bridge.sensors.get('scan',(None,0))[0]
                    df=bridge.sensors.get('df',(None,0))[0]
                    if (scan is not None and df is not None and
                        .70<=scan<=1.16 and .70<=df<=1.16):break
                else:raise RuntimeError('SPAWN_EXISTE_PERO_LIDAR_NO_CONFIRMA_CAJA_A_UN_METRO')
                report['obstaculo_evento'].update({'scan_despues_m':round(scan,4),
                                                   'df_despues_m':round(df,4)})
                bridge.spawn_hold=False
                bridge.last_valid=time.monotonic();bridge.last_progress=time.monotonic()
                say('[OK B3] Obstaculo insertado durante orden: '
                    f'caja=({x:.3f},{y:.3f}); LiDAR={scan:.3f}m, frontal={df:.3f}m')
            if (injected and bridge.bypass_done and dist<=.15
                    and bridge.max_dev<=MAX_ROUTE_DEV and bridge.nonzero>=10):
                reached=True;break
            if now-bridge.last_progress>MAX_IDLE_S:
                raise RuntimeError('SIN_PROGRESO_B3_'+str(round(MAX_IDLE_S))+'S')
        need(injected,'No hubo insercion del obstaculo durante la navegacion')
        if not reached:raise RuntimeError('B3_NO_LLEGO_META_EN_220S')
        bridge.stop()
        for _ in range(10):ex.spin_once(timeout_sec=.05)
        need(bridge.min_clear>STOP_GEOM,'Separacion geometrica por debajo del minimo')
        need(bridge.max_dev<=MAX_ROUTE_DEV,'Desviacion lateral superior al limite seguro')
        need(bridge.max_pubs==1,'Publicador /cmd_vel no exclusivo')
        report['comandos_difuso_activos_despues_spawn']=bridge.fuzzy_move_after_spawn
        report['comandos_ruta_despues_difuso']=bridge.route_after_fuzzy
        if (bridge.a3_fuzzy_seen_after_spawn>=2 and bridge.bypass_done
                and bridge.bypass_commands>=8 and bridge.route_after_fuzzy>=3):
            report['estado']='B3_LLEGADA_HIBRIDA_ASISTIDA_GEOMETRICA'
            report['evasion_hibrida_asistida_demostrada']=True
            # Transparencia: NO se atribuye evasión completa al Mamdani SOLO.
            report['evasion_reactiva_demostrada']=False
            say('[OK B3 v02] LLEGADA HIBRIDA ASISTIDA: A3 DIFUSO observado, '
                'FIS participo, asistencia de subobjetivos temporales y retorno RUTA')
        else:
            report['estado']='B3_LLEGADA_SIN_EVASION_HIBRIDA_DEMOSTRADA'
            say('[NO ACREDITADO B3] Hubo llegada pero falta evidencia completa de arbitraje')
        say(f'[B3] LLEGADA: error={eucl(bridge.pose,GOAL):.3f}m; '
            f'despeje_min={bridge.min_clear:.3f}m; recorrido={bridge.path_len:.3f}m')
        return 0 if report.get('evasion_hibrida_asistida_demostrada') else 2
    except Exception as e:
        report['fallos'].append(str(e))
        say('[ERROR B3] '+str(e))
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
                'barreras':bridge.blocked,
                'comandos_difuso_activos_despues_spawn':bridge.fuzzy_move_after_spawn,
                'comandos_ruta_despues_difuso':bridge.route_after_fuzzy,
                'bypass_completado':bridge.bypass_done,
                'subobjetivos_desvio':bridge.bypass,
                'comandos_evasion_asistida':bridge.bypass_commands,
                'ciclos_giro_en_sitio':bridge.bypass_turns,
                'intervenciones_v_cero':bridge.bypass_rejected,
                'ciclos_mezcla_difusa':bridge.bypass_fuzzy_blended,
                'ciclos_a3_difuso_tras_spawn':bridge.a3_fuzzy_seen_after_spawn,
                'min_proyeccion_geometrica_m':round(bridge.bypass_min_pred,5) if math.isfinite(bridge.bypass_min_pred) else None})
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
    spawn,base,b2world,head=preflight()
    original=spawn.git('status','--porcelain=v1','--untracked-files=no')
    need(subprocess.run(spawn.worker(['--probe','dominio']),cwd=REPO,env=spawn.env(),timeout=30).returncode==0,
         'Dominio 120 no aislado')
    run=base/'B3_obstaculo_sorpresa_v02'/dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    run.mkdir(parents=True,exist_ok=False)
    world,route,rects=make_b3_assets(b2world,run)
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
        robot=own('spawn_robot',spawn.source_command(['ros2','run','gazebo_ros','spawn_entity.py',
                     '-entity','waffle_pi','-file',model,'-x',str(START[0]),'-y',str(START[1]),
                     '-z','0.05','-Y',str(YAW)]))
        robot.wait(timeout=65)
        need(server.poll() is None and robot.returncode==0,'Insercion TurtleBot3 no confirmada')
        say('[OK B3] Gazebo listo; robot en ruta recta y caja sorpresa AUSENTE')
        emulator=own('emulador',spawn.source_command([spawn.PYTHON_ROS,REPO/'tools/emulador_sensores_f1.py',
                             '--ros-args','-p','use_sim_time:=true']))
        bag=own('rosbag',spawn.source_command(['ros2','bag','record','--storage','sqlite3',
                   '-o',run/'rosbag2',*TOPICS]))
        time.sleep(2.)
        need(emulator.poll() is None and bag.poll() is None,'Emulador o ROS bag no arrancaron')
        control=own('navegacion',spawn.source_command([spawn.PYTHON_ROS,SELF,'--monitor',str(run),str(route),str(world)]))
        last=0.
        while control.poll() is None:
            need(server.poll() is None and emulator.poll() is None and bag.poll() is None,
                 'Gazebo, emulador o rosbag se cerro durante B3')
            if time.monotonic()-last>15:
                say('[B3] Ensayo en curso. Evidencias: '+str(run))
                last=time.monotonic()
            time.sleep(.35)
        worker_rc=control.returncode
        for line in (run/'navegacion.log').read_text(encoding='utf-8',errors='replace').splitlines()[-35:]:say(line)
    finally:
        for _,p in reversed(processes):spawn.stop_owned(p)
        for h in files:h.close()
        need(spawn.git('status','--porcelain=v1','--untracked-files=no')==original,
             'Cambios en archivos rastreados por Git')
        for p,d in EXPECTED.items():need(sha(p)==d,'Archivo protegido distinto: '+str(p))
        say('[OK B3] Procesos propios cerrados; archivos tracked y A3/A4/FIS intactos')
    summary=run/'resumen.json'
    need(summary.is_file(),'No existe resumen del monitor (revisar navegacion.log)')
    report=json.loads(summary.read_text(encoding='utf-8'))
    try:counts,bags_ok=bag_stats(run)
    except Exception as e:
        counts={};bags_ok=False;report.setdefault('fallos',[]).append('ROSBAG '+str(e))
    report['rosbag_conteos']=counts;report['rosbag_valido']=bags_ok
    report['head_git']=head
    report['hashes_sha256']={str(p.relative_to(REPO)):d for p,d in EXPECTED.items()}
    if (worker_rc==0 and bags_ok and report.get('estado')=='B3_LLEGADA_HIBRIDA_ASISTIDA_GEOMETRICA'
        and report.get('obstaculo_dinamico_insertado') is True):
        report['estado']='B3_PILOTO_HIBRIDO_ASISTIDO_APROBADO_CONTACTOS_NO_MEDIDOS'
        say('[OK B3 v02] Piloto hibrido 1/5: spawn real + FIS+A3 + desvio asistido + retorno A4 + rosbag')
        rc=0
    else:
        say('[NO APROBADO B3] Evidencia conservada. Analizar causa exacta antes de repetir.')
        rc=2
    summary.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    say('[EVIDENCIA FINAL] '+str(summary))
    return rc


def main():
    p=argparse.ArgumentParser(description=__doc__)
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument('--preflight',action='store_true')
    g.add_argument('--self-test',action='store_true',help='Pruebas geométricas y de guía sin ROS ni Gazebo')
    g.add_argument('--ensayar',action='store_true')
    g.add_argument('--monitor',nargs=3,metavar=('CARPETA','RUTA','MUNDO'),help=argparse.SUPPRESS)
    args=p.parse_args()
    if args.self_test:
        box=(B3_NAME,-3.42554,-3.22554,-2.875,-2.675)
        rects=[('pared_sur',-4.225,-2.725,-3.675,-3.575),
               ('pared_norte',-4.225,-2.725,-1.975,-1.875),box]
        pts=[(START[0]-.05*i,START[1]) for i in range(41)]
        start=(-2.69,START[1]);seq,side,c,d=build_dynamic_detour(start,box,rects,pts,-1)
        need(side==-1 and c>STOP_GEOM+BYPASS_ADDITIONAL_RESERVE,
             'Geometria B3 SUR insegura')
        need(len(seq)==3 and d<MAX_ROUTE_DEV,'Ruta local B3 incorrecta')
        pos=start;yaw=YAW;n=0;trace=[]
        for wp in seq:
            for _ in range(2500):
                if eucl(pos,wp)<BYPASS_GOAL_TOL_M:break
                v,w,err=guidance_to_waypoint(pos,yaw,wp,.06,.52)
                ahead=min(clearance(projected_xy(*pos,yaw,v,w,h),rects)
                          for h in (.07,.14,.21,.28,.35))
                need(ahead>STOP_GEOM,'Movimiento proyectado inseguro en autopueba')
                dt_=0.10
                pos=projected_xy(*pos,yaw,v,w,dt_);yaw=wrap(yaw+w*dt_)
                n+=1;trace.append(clearance(pos,rects))
            else:raise RuntimeError('Control local no converge a '+str(wp))
        need(min(trace)>STOP_GEOM,'Autoprueba geometrica vulnerada')
        say('[OK B3 v02 SELF-TEST] 3 subobjetivos alcanzados; '+str(n)+' ciclos; '+
            'despeje mínimo='+format(min(trace),'.3f')+' m; sin ROS ni archivos modificados')
        return 0
    if args.monitor:return monitor(*map(Path,args.monitor))
    if args.preflight:
        _,_,b2world,_=preflight()
        with tempfile.TemporaryDirectory(prefix='b3_preflight_') as d:
            make_b3_assets(b2world,Path(d))
        say('[OK B3] PREFLIGHT: mundo sin obstaculo, ruta A* y seguridad geométrica aprobados; no ROS')
        return 0
    return execute()

if __name__=='__main__':
    try:sys.exit(main())
    except Exception as e:
        say('[ERROR B3] '+str(e));sys.exit(2)
