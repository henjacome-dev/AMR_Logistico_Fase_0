#!/usr/bin/env python3
"""B4 v02 - PASILLO ANGOSTO REAL de 0.60 m; A* inflado, Gazebo y ROS 2.

Dos etapas independientes, en Ubuntu/WSL:
  python3 A2_B4_pasillo_angosto_integrado_v02.py --preparar
  python3 A2_B4_pasillo_angosto_integrado_v02.py --ensayar

--preparar construye una COPIA temporal de mundo con dos paredes y un mapa
inflado por radio 0.21 m. A* debe pasar por 0.18 m de ancho residual.
--ensayar verifica la misma geometria y lanza UN piloto de navegacion.

No altera Git, A3, A4, Mamdani, mundo, B1/B2/B3 ni rutas oficiales.
Un puente exclusivo publica /cmd_vel. Seguridad adicional: limite 0.06 m/s,
error lateral <=0.030 m y distancia centro robot/superficie SDF >0.265 m.
Se usa solo distancia GEOMETRICA: no se afirman contactos reales Gazebo.
"""
from __future__ import annotations
import argparse, csv, datetime as dt, hashlib, importlib.util, json, math, os
from pathlib import Path
import signal, sqlite3, subprocess, sys, time, heapq, xml.etree.ElementTree as ET

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
MAX_WALL_S = 150.0
MAX_IDLE_S = 35.0
STOP_GEOM = 0.265  # 0.21 m de radio + 0.055 m de reserva lateral
STOP_FRONT = 0.23  # INALTERADO: guarda de seguridad durante movimiento (LiDAR +/-25 y df)
START_SCAN_AXIS_MIN = 0.90  # solo sector frontal central +/-7.5 grados
START_SCAN_WIDE_MIN = 0.40  # sector +/-25 grados ve las PAREDES laterales incluso con frente libre
MAX_ROUTE_DEV = 0.030  # El centro solo puede separarse 3 cm del eje de pasillo
MAX_V, MAX_W = 0.20, 1.5
B4_V_CAP = 0.060
B4_W_CAP = 0.25
B4_WIDTH = 0.600
B4_WALL_THICKNESS = 0.100
B4_WALL_CENTER_X = -2.975
B4_WALL_LENGTH = 2.400
B4_Y = START[1]
B4_NORTH = "a2_b4_pared_norte"
B4_SOUTH = "a2_b4_pared_sur"
B4_RUNS = "B4_pasillo_angosto_v02"
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
         'B4 exige A* de 41 puntos sobre eje, 2 m a 0.05 m')
    need(all(abs(y-B4_Y)<1e-6 for x,y in points),
         'El mapa no permite un paso recto por el centro del pasillo de 0.60 m')
    need(all(eucl(a,b)<.15 for a,b in zip(points,points[1:])),
         'La ruta tiene discontinuidad para A4')
    return points


def collision_rectangles(world):
    """Importa colisiones SDF como rectangulos; falla ante geometria no modelada."""
    root=ET.parse(world).getroot()
    rects=[];uns=[]
    for m in root.findall('./world/model'):
        name=m.get('name','')
        pose=list(map(float,m.findtext('pose','0 0 0 0 0 0').split()))
        if len(pose)!=6:continue
        x,y,z,roll,pitch,yaw=pose
        for link in m.findall('link'):
            for coll in link.findall('collision'):
                dims=coll.findtext('./geometry/box/size')
                if (not dims or any(abs(q)>1e-8 for q in (roll,pitch,yaw))
                        or link.find('pose') is not None or coll.find('pose') is not None):
                    uns.append(name);continue
                sx,sy,sz=map(float,dims.split())
                if z+sz/2<.02:continue
                rects.append((name,x-sx/2,x+sx/2,y-sy/2,y+sy/2))
    names={r[0] for r in rects}
    need(B4_NORTH in names and B4_SOUTH in names,'No estan las dos paredes reales B4')
    need(not any(k.startswith('a2_b2_') for k in names),'Aun quedan obstaculos B2 en mundo B4')
    need(not uns,'Colisiones SDF no modeladas: '+str(sorted(set(uns))))
    return rects


def add_box(world,name,x,y,z,sx,sy,sz):
    m=ET.SubElement(world,'model',name=name)
    ET.SubElement(m,'static').text='true'
    ET.SubElement(m,'pose').text=f'{x:.6f} {y:.6f} {z:.6f} 0 0 0'
    link=ET.SubElement(m,'link',name='body')
    for typ in ('collision','visual'):
        sub=ET.SubElement(link,typ,name=typ)
        box=ET.SubElement(ET.SubElement(sub,'geometry'),'box')
        ET.SubElement(box,'size').text=f'{sx:.6f} {sy:.6f} {sz:.6f}'


def make_b4_assets(b2world,folder):
    """Genera SDF temporal, mapa PGM realmente inflado, A* 4-vecinos y CSV."""
    folder.mkdir(parents=True,exist_ok=False)
    tree=ET.parse(b2world);root=tree.getroot();world=root.find('world')
    need(world is not None,'SDF base sin world')
    removed=[]
    for model in list(world.findall('model')):
        name=model.get('name','')
        if name in ('a2_b2_obstaculo_estatico_centro','a2_b2_pared_norte','a2_b2_pared_sur'):
            removed.append(name);world.remove(model)
    need(set(removed)=={'a2_b2_obstaculo_estatico_centro','a2_b2_pared_norte','a2_b2_pared_sur'},
         'No se identificaron los 3 elementos B2 originales; NO crear B4')
    offset=(B4_WIDTH+B4_WALL_THICKNESS)/2.
    add_box(world,B4_NORTH,B4_WALL_CENTER_X,B4_Y+offset,.35,
            B4_WALL_LENGTH,B4_WALL_THICKNESS,.70)
    add_box(world,B4_SOUTH,B4_WALL_CENTER_X,B4_Y-offset,.35,
            B4_WALL_LENGTH,B4_WALL_THICKNESS,.70)
    world_path=folder/'B4_pasillo_060m.world'
    tree.write(world_path,encoding='utf-8',xml_declaration=True)
    rects=collision_rectangles(world_path)
    north=next(r for r in rects if r[0]==B4_NORTH)
    south=next(r for r in rects if r[0]==B4_SOUTH)
    corridor=(north[3]-south[4])
    need(abs(corridor-.6)<1e-9,'Pasillo fisico !=0.60 m')
    width_free=corridor-2*.21
    need(abs(width_free-.18)<1e-9,'Ancho inflado no corresponde a 0.18 m')
    # Semantica PGM: 254 libre, 0 ocupado; origen (-5,-4), resolucion 0.05.
    x0,y0,res,nx,ny=-5.0,-4.0,.05,200,160
    def xy(i,j):return x0+(i+.5)*res,y0+(ny-j-.5)*res
    def cell(p):
        i=round((p[0]-x0)/res-.5)
        j=ny-1-round((p[1]-y0)/res-.5)
        need(0<=i<nx and 0<=j<ny and eucl(xy(i,j),p)<1e-6,
             'Inicio/meta no coincide con malla B4')
        return i,j
    def dist_rect(p,r):
        x,y=p;_,a,b,c,d=r
        return math.hypot(max(a-x,0.,x-b),max(c-y,0.,y-d))
    data=bytearray(nx*ny)
    for j in range(ny):
        for i in range(nx):
            p=xy(i,j)
            # Inflado geometricamente continuo, incluyendo todas las cajas del mundo.
            data[j*nx+i]=254 if min(dist_rect(p,r) for r in rects)>.21+1e-9 else 0
    def free(p):
        i,j=p
        return 0<=i<nx and 0<=j<ny and data[j*nx+i]==254
    start,goal=cell(START),cell(GOAL)
    need(free(start) and free(goal),'B4 inicio o meta ocupados tras inflar por 0.21m')
    def heur(p):return res*(abs(p[0]-goal[0])+abs(p[1]-goal[1]))
    q=[(heur(start),0.,start)];g={start:0.};prev={}
    while q:
        _,cost,p=heapq.heappop(q)
        if cost>g.get(p,float('inf'))+1e-9:continue
        if p==goal:break
        for di,dj in ((-1,0),(1,0),(0,-1),(0,1)):
            nxt=(p[0]+di,p[1]+dj)
            if not free(nxt):continue
            nc=cost+res
            if nc+1e-9<g.get(nxt,float('inf')):
                g[nxt]=nc;prev[nxt]=p
                heapq.heappush(q,(nc+heur(nxt),nc,nxt))
    need(goal in g,'A* NO encuentra ruta en pasillo 0.60m inflado; abortar movimiento')
    cells=[goal]
    while cells[-1]!=start:cells.append(prev[cells[-1]])
    cells.reverse();pts=[xy(*c) for c in cells]
    # Verificar todo el segmento y no solo el centro de las celdas.
    for a,b in zip(pts[:-1],pts[1:]):
        for n in range(6):
            t=n/5.;p=(a[0]*(1-t)+b[0]*t,a[1]*(1-t)+b[1]*t)
            need(min(dist_rect(p,r) for r in rects)>.21,
                 'Un segmento A* cruza una colision inflada')
    route=folder/'B4_ruta_astar_centro.csv'
    with route.open('w',newline='',encoding='utf-8') as f:
        wr=csv.writer(f,lineterminator='\n');wr.writerow(('x_m','y_m'))
        for x,y in pts:wr.writerow((f'{x:.5f}',f'{y:.5f}'))
    route_points(route)
    pgm=folder/'B4_mapa_inflado_021.pgm'
    pgm.write_bytes(f'P5\n{nx} {ny}\n255\n'.encode('ascii')+bytes(data))
    (folder/'B4_mapa_inflado_021.yaml').write_text(
        'image: B4_mapa_inflado_021.pgm\nmode: trinary\nresolution: 0.05\n'
        'origin: [-5.0, -4.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',
        encoding='utf-8')
    minimum=min(min(dist_rect(p,r) for r in rects) for p in pts)
    need(minimum>=.299-1e-8 and minimum<=.301+1e-8,
         'Ruta central no deja 0.300m a la superficie de las paredes')
    need(minimum>STOP_GEOM+.020,'Despeje insuficiente para piloto B4')
    length=sum(eucl(a,b) for a,b in zip(pts[:-1],pts[1:]))
    manifest={'estado':'B4_GEOMETRIA_ASTAR_VERIFICADOS_SIN_GAZEBO',
              'ancho_fisico_m':corridor,'radio_robot_m':.21,'ancho_tras_inflado_m':width_free,
              'centrolinea_y_m':B4_Y,'mundo':str(world_path),'sha256_mundo':sha(world_path),
              'ruta':str(route),'sha256_ruta':sha(route),'puntos_astar':len(pts),
              'longitud_ruta_m':round(length,4),'despeje_centro_ruta_m':round(minimum,4),
              'margen_lateral_robot_m':round(minimum-.21,4),
              'limite_guardia_geometrica_m':STOP_GEOM,'max_desviacion_m':MAX_ROUTE_DEV,
              'pos_inicial':START,'yaw_inicial_rad':YAW,'meta':GOAL,
              'contactos_gazebo_medidos':False}
    (folder/'B4_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    say(f'[OK B4] Pasillo REAL {corridor:.3f}m, libre tras inflacion 0.21m: {width_free:.3f}m')
    say(f'[OK B4] A* 4-vecinos: {len(pts)} puntos / {length:.3f} m; centro-pared min {minimum:.3f} m')
    say('[OK B4] SDF, PGM, CSV y manifiesto creados como evidencias NO TRACKED')
    return world_path,route,pts,rects

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
    base,b2world,b2route,head=spawn.check_prepared()
    for p,d in EXPECTED.items():
        need(p.is_file() and not p.is_symlink() and sha(p)==d,
             'Componente protegido ausente o modificado: '+str(p))
    need(R18.is_file(),'Falta parser R18')
    need(os.statvfs(str(REPO)).f_bavail*os.statvfs(str(REPO)).f_frsize>700*1024*1024,
         'Espacio insuficiente para rosbag')
    need(len(list((base/'B2_rescate_integrado').glob('*/resumen.json')))>=5,
         'No se encuentran las cinco evidencias B2; revisar ruta de trabajo')
    say('[OK B4] Controladores SHA256 y rama auditados; B1/B2/B3 intactos')
    say(f'[OK B4] HEAD {head[:12]} auditado sin exigir un commit historico')
    return spawn,base,b2world,head


def prepare_only():
    _,base,b2world,head=preflight()
    folder=base/B4_RUNS/dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    make_b4_assets(b2world,folder)
    say('[SALIDA] '+str(folder))
    say('[SIGUIENTE] Si geometria=OK, ejecutar --ensayar (crea otro mundo temporal identico)')
    return 0

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
    report={'actividad':'B4 PASILLO ANGOSTO 0.60M', 'estado':'NO_APROBADO',
            'evasion_completa_demostrada':False,'pasillo_angosto_atravesado':False,
            'contactos_gazebo_verificados':False,'ancho_pasillo_m':B4_WIDTH,'ancho_libre_inflado_m':0.18,
            'advertencia_contactos':'No se dispone de sensor Gazebo de contacto; distancia geometrica es una estimacion',
            'mundo':str(world),'sha256_mundo':sha(world),'ruta':str(route),'sha256_ruta':sha(route),
            'ruta_puntos':len(pts),
            'dominio_ros':DOM,'inicio_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
            'interpretacion_lidar':'scan ±25° incluye paredes; comprobacion inicial eje ±7.5°; guarda geometrica 0.265m intacta',
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
                self.scan_eje=(None,0.)  # sector +/-7.5°, separado del minimo LiDAR amplio +/-25°
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
                # El minimo +/-25° observa paredes laterales a ~0.71m en
                # pasillos 0.60m. No es correcto exigirle >0.90m al inicio.
                # Se conserva completo para el freno operativo y se crea un
                # sector central +/-7.5° solo para validar eje frontal libre.
                wide=[];axis=[]
                for i,d in enumerate(msg.ranges):
                    ang=wrap(msg.angle_min+i*msg.angle_increment)
                    if abs(ang)>math.radians(25):continue
                    if math.isinf(d) and d>0 and math.isfinite(msg.range_max):d=msg.range_max
                    if math.isfinite(d) and msg.range_min<=d<=msg.range_max:
                        wide.append(float(d))
                        if abs(ang)<=math.radians(7.5):axis.append(float(d))
                now=time.monotonic()
                self.sensors['scan']=(min(wide) if wide else None,now)
                self.scan_eje=(min(axis) if axis else None,now)
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
                    elif (self.scan_eje[0] is None or not math.isfinite(self.scan_eje[0])
                          or now-self.scan_eje[1]>.9):
                        self.raise_safe('B4_LIDAR_CENTRAL_CADUCADO')
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
                            # Mantener el eje del pasillo como requisito independiente del FIS.
                            # En un paso de 0.60 m un desvio lateral difuso no es seguro:
                            # detener en lugar de dejar que el robot se acerque a una pared.
                            if mode!='RUTA':
                                self.raise_safe('B4_CAMBIO_MODO_A3_NO_APTO_PARA_PASILLO_060')
                            else:
                                v=min(v,B4_V_CAP)
                                w=max(-B4_W_CAP,min(B4_W_CAP,w))
                                vals=[clearance(projected_xy(*self.pose,self.yaw,v,w,.35*k/7),rects)
                                      for k in range(1,8)]
                                pred=projected_xy(*self.pose,self.yaw,v,w,.35)
                                if min(vals)<=STOP_GEOM:
                                    self.raise_safe('B4_COLISION_PROYECTADA')
                                elif nearest_distance(pred,pts)>MAX_ROUTE_DEV:
                                    self.raise_safe('B4_DESVIACION_PROYECTADA')
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
        # En un pasillo de 0.60m, rayos +/-25° ven paredes laterales:
        # distancia aprox. 0.30/sin(25°)=0.71m. Por ello no se exige >0.90m
        # al scan ANCHO; ese umbral pertenece al scan CENTRAL +/-7.5°.
        scan_wide=bridge.sensors['scan'][0]
        scan_axis=bridge.scan_eje[0]
        df=bridge.sensors['df'][0]
        report['lecturas_iniciales_m']={
            'scan_ancho_25deg':scan_wide,'scan_central_7p5deg':scan_axis,
            'frontal_emulado':df,'lateral_izquierdo':bridge.sensors['di'][0],
            'lateral_derecho':bridge.sensors['dd'][0]}
        say('[B4 SENSOR] LiDAR +/-25°='+str(round(scan_wide,3) if scan_wide is not None else None)+
            'm; LiDAR +/-7.5°='+str(round(scan_axis,3) if scan_axis is not None else None)+
            'm; df='+str(round(df,3) if df is not None else None)+'m')
        need(scan_axis is not None and math.isfinite(scan_axis)
             and time.monotonic()-bridge.scan_eje[1] <=.9,
             'No hay medicion central valida en +/-7.5°; no armar')
        need(scan_axis>START_SCAN_AXIS_MIN and df>START_SCAN_AXIS_MIN
             and scan_wide is not None and scan_wide>START_SCAN_WIDE_MIN,
             'Obstaculo inicial: LiDAR central, frontal emulado o minima angular fuera de limites')
        need(bridge.count_publishers('/cmd_vel')==1,
             'Existe otro publicador /cmd_vel')
        need(bridge.max_dev<.015 and bridge.min_clear>.285,
             'Spawn incompatible con ruta o geometria')
        bridge.armed=True;bridge.last_valid=time.monotonic();bridge.last_progress=time.monotonic()
        say('[OK B4] A3/A4/Mamdani listos; un /cmd_vel; velocidad <=0.060m/s')
        begin=time.monotonic(); last_print=begin
        reached=False
        while time.monotonic()-begin<MAX_WALL_S:
            spin()
            now=time.monotonic()
            if bridge.pose is None:continue
            dist=eucl(bridge.pose,GOAL)
            if now-last_print>=8:
                say(f'[B4] t={now-begin:.1f}s; meta={dist:.3f}m; avance={bridge.path_len:.2f}m; '
                    f'A3={bridge.status.get("modo") if bridge.status else "?"}; '
                    f'despeje_min={bridge.min_clear:.3f}m')
                last_print=now
            if dist<=.15:
                # Confirmar que A4 llega cerca de final y que los datos siguen vigentes.
                if bridge.max_dev<=MAX_ROUTE_DEV and bridge.nonzero>=10:
                    reached=True;break
            if now-bridge.last_progress>MAX_IDLE_S:
                raise RuntimeError('SIN_PROGRESO_HACIA_META_'+str(round(MAX_IDLE_S))+'S')
        if not reached:raise RuntimeError('NO_LLEGO_META_EN_150S')
        report['tiempo_desde_armado_s']=round(time.monotonic()-begin,3)
        bridge.stop()
        for _ in range(10):ex.spin_once(timeout_sec=.05)
        need(bridge.min_clear>STOP_GEOM,'Distancia geometrica debajo del minimo')
        need(bridge.max_dev<=MAX_ROUTE_DEV,'Error lateral maximo excedido')
        need(bridge.max_pubs==1,'Publicador unico perdido')
        report['estado']='B4_LLEGADA_VALIDADA_GEOMETRICAMENTE'
        report['pasillo_angosto_atravesado']=True
        say(f'[OK B4] LLEGADA: error={eucl(bridge.pose,GOAL):.3f} m; '
            f'despeje_min={bridge.min_clear:.3f} m; recorrido={bridge.path_len:.3f} m')
        return 0
    except Exception as e:
        report['fallos'].append(str(e))
        say('[ERROR B4] '+str(e))
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
        omega=[float(r[3]) for r in commands]
        active=[w for w in omega if abs(w)>.02]
        report['cambios_de_signo_omega_n']=sum(a*b<0 for a,b in zip(active[:-1],active[1:]))
        report['variacion_angular_acumulada_rad_s']=round(sum(abs(a-b) for a,b in zip(omega[:-1],omega[1:])),6)
        report['velocidad_lineal_maxima_m_s']=max((abs(float(r[2])) for r in commands),default=0.)
        report['parada_por_barrera']=bool(bridge and bridge.failure)
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
    run=base/B4_RUNS/dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    world,route,pts,rects=make_b4_assets(b2world,run)
    need(min(clearance(p,rects) for p in pts)>STOP_GEOM+.020,
         'Ruta preparada tiene margen insuficiente para movimiento')
    say('[OK B4] Verificacion previa A* y mundo 0.60m aprobada: autorizacion limitada de UN piloto')
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
        say('[OK B4] Gazebo y TurtleBot3 en pasillo 0.60m, pose verificada antes de armar')
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
                say('[B4] Ensayo en curso. Evidencias: '+str(run))
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
        say('[OK B4] Solo procesos propios detenidos; controladores y Git preservados')
    summary=run/'resumen.json'
    need(summary.is_file(),'No se genero resumen B4 (revisar navegacion.log)')
    report=json.loads(summary.read_text(encoding='utf-8'))
    try:
        counts,bags_ok=bag_stats(run)
    except Exception as e:
        counts={};bags_ok=False;report.setdefault('fallos',[]).append('ROSBAG '+str(e))
    report['rosbag_conteos']=counts;report['rosbag_valido']=bags_ok
    report['head_git']=head
    report['hashes_sha256']={str(p.relative_to(REPO)):d for p,d in EXPECTED.items()}
    if worker_rc==0 and bags_ok and report.get('estado')=='B4_LLEGADA_VALIDADA_GEOMETRICAMENTE':
        report['estado']='B4_LLEGADA_Y_ROSBAG_VALIDOS_CONTACTOS_NO_MEDIDOS'
        say('[OK B4] PILOTO 1/5: paso por pasillo 0.60m + ROSBAG; contactos no medidos')
        rc=0
    else:
        say('[NO APROBADO B4] Guardias, ruta o rosbag no cumplieron; no repetir sin diagnostico')
        rc=2
    summary.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    say('[EVIDENCIA FINAL] '+str(summary))
    return rc


def main():
    p=argparse.ArgumentParser(description=__doc__)
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument('--preflight',action='store_true')
    g.add_argument('--preparar',action='store_true')
    g.add_argument('--ensayar',action='store_true')
    g.add_argument('--monitor',nargs=3,metavar=('CARPETA','RUTA','MUNDO'),help=argparse.SUPPRESS)
    args=p.parse_args()
    if args.monitor:return monitor(*map(Path,args.monitor))
    if args.preflight:preflight();say('[OK B4] Preflight aprobado, ningun proceso iniciado');return 0
    if args.preparar:return prepare_only()
    return execute()

if __name__=='__main__':
    try:sys.exit(main())
    except Exception as e:
        say('[ERROR B4] '+str(e));sys.exit(2)
