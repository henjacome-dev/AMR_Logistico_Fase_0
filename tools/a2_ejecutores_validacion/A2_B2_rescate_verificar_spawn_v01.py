#!/usr/bin/env python3
"""B2.3 rescate: comprobar EN GAZEBO el nuevo spawn, SIN MOVIMIENTO.

WSL/Ubuntu:
  python3 /mnt/c/Users/Henry/Downloads/A2_B2_rescate_verificar_spawn_v01.py --verificar

Una sola instruccion: audita el experimento preparado, arranca Gazebo
Classic en un dominio ROS 2 aislado, inserta TurtleBot3 en la nueva pose,
comprueba /odom y /scan sin publicar en /cmd_vel, guarda evidencia y
termina SOLO los procesos que inicio. A3, A4, FIS, Git y mundos originales
no se modifican. La ejecucion sin movimiento NO acredita evasión de B2.3.
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
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

REPO = Path.home() / 'AMR_Logistico_Fase_0'
DOMAIN = '120'
START = (-1.975, -2.775)
YAW = math.pi
GOAL = (-3.975, -2.775)
OBS = (-3.475, -2.775)
WORLD_NAME = 'a2_b2_obstaculo_estatico_centro'
ACTIVATE = REPO / 'tools/activar_entorno.sh'
PYTHON_ROS = Path.home() / '.venvs/amr-a2-r18/bin/python'
EXPECTED_HASHES = {
    'tools/A2_A3_supervisor_rubrica_v03_v2_float32.py': '2c820de906497080e84e9ff37c458d8787a290438d6651cf38bdc22e9bcd4e92',
    'tools/A2_A4_seguidor_reincorporacion_v02_evento.py': '5f03cfaea0ee52769e33b32924441ea72761d4c8300f778290b17dc57db5e02d',
    'matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis': '67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903',
}


def say(s):
    print(s, flush=True)


def need(p, s):
    if not p:
        raise RuntimeError(s)


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def git(*args):
    r = subprocess.run(['git', *args], cwd=REPO, text=True, capture_output=True, timeout=15)
    need(r.returncode == 0, 'Git fallo: ' + (r.stderr or r.stdout))
    return r.stdout.strip()


def env():
    return {**os.environ, 'ROS_DOMAIN_ID': DOMAIN, 'ROS_LOCALHOST_ONLY': '1',
            'PYTHONUNBUFFERED': '1'}


def source_command(argv):
    return ['bash', '-c', 'source "$1" >/dev/null && shift && exec "$@"',
            'b2_rescate', str(ACTIVATE), *map(str, argv)]


def worker(args):
    return source_command([PYTHON_ROS, Path(__file__).resolve(), *args])


def heading_to_yaw(q):
    return math.atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


def wrapped(a):
    return math.atan2(math.sin(a), math.cos(a))


def latest_prepared():
    root = REPO / 'resultados/A2_B2_rescate_geometria_astar'
    paths = sorted(root.glob('*/B2_rescate_manifest.json'), reverse=True)
    need(paths, 'No se encuentra salida preparada B2. Ejecutar primero --preparar')
    for p in paths:
        if p.is_file() and not p.is_symlink():
            d = json.loads(p.read_text(encoding='utf-8'))
            if d.get('estado') == 'PREPARADO_NO_EJECUTADO':
                return p.parent, d
    raise RuntimeError('No hay un manifiesto B2 PREPARADO_NO_EJECUTADO')


def in_proc(name):
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            if (p / 'comm').read_text().strip() == name:
                return p.name
        except (FileNotFoundError, PermissionError, UnicodeError, OSError):
            continue
    return None


def check_prepared():
    need((REPO / '.git').is_dir(), 'Repositorio local no encontrado')
    need(git('branch', '--show-current') == 'desarrollo-avance2',
         'La rama local no es desarrollo-avance2; detener para auditar')
    head = git('rev-parse', 'HEAD')
    # No se exige HEAD historico: cambiar commits no altera la validez de la
    # geometria; se comprueban POR HASH los tres artefactos protegidos.
    for rel, digest in EXPECTED_HASHES.items():
        p = REPO / rel
        need(p.is_file() and not p.is_symlink() and sha(p) == digest,
             'Artefacto protegido ausente o distinto: ' + rel)
    need(ACTIVATE.is_file() and PYTHON_ROS.is_file(),
         'Falta entorno ROS anterior (activar_entorno.sh o .venvs/amr-a2-r18)')
    need(7400 + 250*int(DOMAIN) + 240 < 65536, 'Dominio ROS fuera de rango UDP')
    for name in ('gzserver', 'gzclient'):
        pid = in_proc(name)
        need(pid is None, f'{name} existente (PID {pid}); cerrar de forma controlada antes del ensayo')
    folder, m = latest_prepared()
    world = folder / 'B2_rescate_obstaculo_central.world'
    route = folder / 'B2_rescate_ruta_astar.csv'
    mapa = folder / 'B2_rescate_mapa_inflado.pgm'
    for p in (world, route, mapa):
        need(p.is_file() and not p.is_symlink(), 'Archivo preparado no disponible: ' + str(p))
    need(tuple(m.get('inicio_xy_m', [])) == START and
         abs(float(m.get('inicio_yaw_rad', -100)) - YAW) < 1e-9 and
         tuple(m.get('meta_xy_m', [])) == GOAL,
         'El manifiesto no coincide con el escenario de rescate')
    need(tuple(m.get('obstaculo', {}).get('centro_xy_m', [])) == OBS,
         'Obstaculo del manifiesto en otra coordenada')
    root = ET.parse(world).getroot()
    world_node = root.find('world')
    need(world_node is not None, 'SDF mundo invalido')
    boxes = [x for x in world_node.findall('model') if x.get('name') == WORLD_NAME]
    need(len(boxes) == 1, 'Debe existir exactamente un obstaculo B2 central')
    pose = list(map(float, boxes[0].findtext('pose', '').split()))
    need(len(pose) == 6 and math.dist(pose[:2], OBS) < 1e-6,
         'Posicion del obstaculo no coincide con la ruta')
    with route.open('r', encoding='utf-8-sig', newline='') as h:
        rd = csv.DictReader(h)
        need({'x_m','y_m'} <= set(rd.fieldnames or []), 'CSV de ruta mal formado')
        pts = [(float(row['x_m']), float(row['y_m'])) for row in rd]
    need(len(pts) >= 10 and math.dist(pts[0], START) < 0.005
         and math.dist(pts[-1], GOAL) < 0.005,
         'Los extremos del CSV no coinciden con spawn y meta')
    need(all(math.dist(a,b) <= .15 for a,b in zip(pts[:-1],pts[1:])),
         'Saltos de la ruta mayores que el contrato del seguidor A4')
    need(float(m.get('dist_minima_ruta_a_superficie_obstaculo_m', 0)) > .30,
         'Ruta insuficientemente alejada del obstaculo')
    say('[OK B2 RESCATE] Git en rama correcta; HEAD registrado, sin dependencia del commit antiguo')
    say('[OK B2 RESCATE] A3, A4 y FIS v03 coinciden con SHA256 validados')
    say(f'[OK B2 RESCATE] Mundo, ruta {len(pts)} puntos, spawn {START} / yaw pi y meta: coherentes')
    return folder, world, route, head


def probe(kind):
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node('b2_rescate_probe_' + kind, enable_rosout=False)
    try:
        limit = time.monotonic() + (4 if kind == 'dominio' else 50)
        while time.monotonic() < limit:
            rclpy.spin_once(node, timeout_sec=.15)
            if kind == 'dominio':
                topics = [x for x,_ in node.get_topic_names_and_types()
                          if x not in ('/rosout','/parameter_events')]
                need(not topics and node.count_publishers('/cmd_vel') == 0,
                     'Dominio DDS 120 no esta aislado: ' + str(topics))
            else:
                services = dict(node.get_service_names_and_types())
                if 'gazebo_msgs/srv/SpawnEntity' in services.get('/spawn_entity', []):
                    say('[OK] Gazebo factory lista')
                    return 0
        need(kind == 'dominio', 'Gazebo no activo tras 50 s: /spawn_entity no disponible')
        say('[OK] Dominio ROS 120 aislado')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def observar(path):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    class Watch(Node):
        def __init__(self):
            super().__init__('b2_rescate_observar_spawn', enable_rosout=False)
            self.odom = []
            self.scan = []
            self.pub_max = 0
            self.sub_o = self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.sub_s = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        def on_odom(self, msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            pos = (float(p.x), float(p.y))
            yaw = heading_to_yaw(q)
            if all(math.isfinite(v) for v in (*pos,yaw)):
                self.odom.append((pos[0],pos[1],yaw,time.monotonic()))
        def on_scan(self, msg):
            rs = []
            for i,d in enumerate(msg.ranges):
                a = wrapped(msg.angle_min + i*msg.angle_increment)
                if abs(a) > math.radians(25):
                    continue
                if math.isinf(d) and d > 0 and math.isfinite(msg.range_max):
                    d = msg.range_max
                if math.isfinite(d) and msg.range_min <= d <= msg.range_max:
                    rs.append(float(d))
            if rs:
                self.scan.append((min(rs),time.monotonic()))

    rclpy.init()
    n = Watch()
    report = {'actividad':'B2.3 RESCATE SPAWN','estado':'NO_APROBADO',
              'movimiento_habilitado': False, 'publicaciones_cmd_vel': 0,
              'ubicacion_esperada_m': START, 'yaw_esperado_rad': YAW,
              'fuente_contactos_gazebo':'NO OBSERVADA: prueba inmovil'}
    try:
        deadline = time.monotonic() + 38
        while time.monotonic() < deadline:
            rclpy.spin_once(n, timeout_sec=.12)
            n.pub_max = max(n.pub_max, n.count_publishers('/cmd_vel'))
            if len(n.odom) >= 8 and len(n.scan) >= 5:
                break
        need(len(n.odom) >= 8 and len(n.scan) >= 5,
             'No se recibieron al menos 8 /odom y 5 /scan en 38 segundos')
        need(n.pub_max == 0, 'Hay otro publicador de /cmd_vel en este dominio')
        od = n.odom[-8:]
        med = (sum(x for x,_,_,_ in od)/len(od),sum(y for _,y,_,_ in od)/len(od))
        yy = od[-1][2]
        shift = max(math.dist((x,y), START) for x,y,_,_ in od)
        ang = abs(wrapped(yy-YAW))
        front = min(z for z,_ in n.scan[-5:])
        report.update({'ubicacion_medida_m': [round(x,5) for x in med],
                       'yaw_medido_rad':round(yy,6),
                       'error_posicion_max_m':round(shift,5),
                       'error_yaw_rad':round(ang,5),
                       'distancia_frontal_scan_min_m':round(front,4),
                       'mensajes_odom':len(n.odom), 'mensajes_scan':len(n.scan),
                       'publicadores_cmd_vel_max':n.pub_max})
        need(shift < .08, f'Spawn en posicion distinta (error {shift:.3f} m)')
        need(ang < .15, f'Yaw inicial incorrecto (error {ang:.3f} rad)')
        need(front > .90, f'Obstaculo frontal demasiado proximo (scan={front:.3f} m)')
        report['estado'] = 'SPAWN_VALIDADO_SIN_MOVIMIENTO'
        say(f'[OK B2 RESCATE] /odom: x={med[0]:.3f}, y={med[1]:.3f}, yaw={yy:.3f} rad; error<0.08 m')
        say(f'[OK B2 RESCATE] /scan frontal: {front:.3f} m (>0.90); sin publicar /cmd_vel')
        return 0
    except Exception as e:
        report['error'] = str(e)
        say('[ERROR B2 RESCATE OBSERVADOR] ' + str(e))
        return 2
    finally:
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n',encoding='utf-8')
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def stop_owned(p):
    if p is None or p.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT,8),(signal.SIGTERM,5),(signal.SIGKILL,3)):
        if p.poll() is not None:
            break
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            break
        try:
            p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue


def execute():
    folder, world, route, head = check_prepared()
    orig_status = git('status','--porcelain=v1','--untracked-files=no')
    need(subprocess.run(worker(['--probe','dominio']), cwd=REPO, env=env(), timeout=30).returncode == 0,
         'Dominio ROS 120 ocupado o no disponible')
    run = folder/'B2_rescate_spawn'/dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    run.mkdir(parents=True, exist_ok=False)
    report_path = run/'B2_rescate_spawn_resultado.json'
    processes = []
    streams = []
    def own(label, cmd):
        h = (run/(label+'.log')).open('w',encoding='utf-8')
        streams.append(h)
        p = subprocess.Popen(cmd,cwd=REPO,env=env(),stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((label,p))
        return p
    try:
        server = own('gazebo',source_command(['gzserver','--verbose','-s','libgazebo_ros_init.so',
                                                  '-s','libgazebo_ros_factory.so',world]))
        factory = own('factory',worker(['--probe','factory']))
        try:
            factory.wait(timeout=63)
        except subprocess.TimeoutExpired:
            raise RuntimeError('Tiempo agotado esperando fábrica Gazebo')
        need(server.poll() is None and factory.returncode == 0,
             'Gazebo no disponible; revisar gazebo.log/factory.log')
        # Resolver el paquete TurtleBot3 desde el entorno REAL de WSL.
        pkg = subprocess.run(source_command(['ros2','pkg','prefix','turtlebot3_gazebo']),
                             cwd=REPO, env=env(), text=True, capture_output=True, timeout=25)
        need(pkg.returncode == 0 and pkg.stdout.strip(),
             'No se pudo ubicar turtlebot3_gazebo: ' + pkg.stderr)
        model = Path(pkg.stdout.strip())/'share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf'
        need(model.is_file(), 'Modelo Waffle Pi no encontrado: ' + str(model))
        # Spawn unicamente de esta entidad; sin A3/A4, sin /cmd_vel.
        spawn = own('spawn',source_command(['ros2','run','gazebo_ros','spawn_entity.py',
                         '-entity','waffle_pi','-file',model,
                         '-x',f'{START[0]:.3f}','-y',f'{START[1]:.3f}',
                         '-z','0.05','-Y',str(YAW)]))
        try:
            spawn.wait(timeout=65)
        except subprocess.TimeoutExpired:
            raise RuntimeError('Tiempo agotado insertando TurtleBot3')
        need(spawn.returncode == 0 and server.poll() is None,
             'Spawn ROS fallido; revisar spawn.log')
        say('[OK B2 RESCATE] Gazebo arrancó y TurtleBot3 insertado en nueva pose')
        observer = own('observer',worker(['--observar',str(report_path)]))
        try:
            observer.wait(timeout=51)
        except subprocess.TimeoutExpired:
            raise RuntimeError('Tiempo agotado comprobando odometria/LiDAR')
        need(observer.returncode == 0 and report_path.is_file(),
             'Verificacion de pose/sensores fallida: revisar observer.log')
        report = json.loads(report_path.read_text(encoding='utf-8'))
        report.update({'head_git':head,'archivo_mundo':str(world),
                       'archivo_ruta':str(route),'dominio_ros':DOMAIN})
        report_path.write_text(json.dumps(report, ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        say('[OK B2 RESCATE] Spawn en Gazebo validado. A3/A4/Mamdani no ejecutados ni alterados.')
        say('[EVIDENCIA] '+str(report_path))
        return 0
    finally:
        for _,p in reversed(processes):
            stop_owned(p)
        for h in streams:
            h.close()
        need(git('status','--porcelain=v1','--untracked-files=no')==orig_status,
             'Cambió un archivo tracked durante el procedimiento; auditar')
        for rel,digest in EXPECTED_HASHES.items():
            need(sha(REPO/rel)==digest,'Se altero artefacto protegido: '+rel)
        say('[OK] Procesos propios cerrados; archivos tracked y controladores preservados')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--verificar',action='store_true')
    group.add_argument('--preflight',action='store_true')
    group.add_argument('--probe',choices=('dominio','factory'),help=argparse.SUPPRESS)
    group.add_argument('--observar',metavar='JSON',help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args.probe:
        return probe(args.probe)
    if args.observar:
        return observar(Path(args.observar))
    if args.preflight:
        check_prepared()
        return 0
    return execute()

if __name__=='__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('[ERROR B2 RESCATE SPAWN] ' + str(exc),file=sys.stderr,flush=True)
        sys.exit(2)
