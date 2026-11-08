#!/usr/bin/env python3
"""AMR Logistico E2.3.4 - Gazebo + controlador difuso + arbitraje EN SOMBRA.

Ejecucion WSL2 (Ubuntu 22.04, ROS 2 Humble):
  python3 /mnt/c/Users/Henry/Downloads/A2_E234_Gazebo_sombra_seguro.py

PRUEBA SIN MOVIMIENTO: no arranca seguidor de ruta ni supervisor F1.3,
no publica /cmd_vel y NO habilita la salida candidata de arbitraje.
No realiza git add/commit/push, ni modifica archivos versionados.
En caso de error interrumpe exclusivamente el lanzamiento que inició.
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
REF = '30fe276b594cdcdd5f2b5d2a81c0f1a265422e4e'
DOMAIN = '223'  # E2.3.3 exige dominios 200..224 para el candidato aislado.
OBSERVE_S = 18.0
DISCOVER_MAX_S = 135.0

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

LAUNCH_SOURCE = '''#!/usr/bin/env python3
"""E2.3.4: Gazebo real, propuestas difusas y supervisor candidato SOLO sombra.
NO incorpora seguidor de ruta ni publicadores /cmd_vel.
No equivale a integrar el sistema operativo R19.
"""
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    repo = Path.home() / 'AMR_Logistico_Fase_0'
    py = Path.home() / '.venvs/amr-a2-r18/bin/python'
    world = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(repo / 'launch/almacen_f1.launch.py')))
    emulator = ExecuteProcess(
        cmd=[sys.executable, '-u', str(repo / 'tools/emulador_sensores_f1.py'),
             '--ros-args', '-p', 'use_sim_time:=true'],
        cwd=str(repo), output='screen')
    fuzzy = ExecuteProcess(
        cmd=[str(py), '-u', str(repo / 'tools/A2_E22_nodo_difuso.py'), '--ros',
             '--ros-args', '-p', 'use_sim_time:=true'],
        cwd=str(repo), output='screen')
    candidate = ExecuteProcess(
        cmd=[str(py), '-u', str(repo / 'tools/A2_E233_supervisor_integrado_aislado.py'),
             '--nodo-aislado'],
        cwd=str(repo), output='screen',
        additional_env={'_A2_E233_CHILD': '1'})
    return LaunchDescription([world, emulator, fuzzy, candidate])
'''


def emit(message: str) -> None:
    print(message, flush=True)


def check(condition: bool, detail: str) -> None:
    if not condition:
        raise RuntimeError(detail)
    emit('[OK] ' + detail)


def git(*args: str) -> str:
    proc = subprocess.run(['git', *args], cwd=REPO, text=True,
                          capture_output=True, timeout=20)
    if proc.returncode:
        raise RuntimeError('Git: ' + (proc.stderr or proc.stdout).strip())
    return proc.stdout.strip()


def check_processes() -> None:
    # Impedir colisiones con Gazebo Classic existente incluso en otros dominios.
    found=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            name = (proc / 'comm').read_text(encoding='utf-8').strip()
            if name in ('gzserver', 'gzclient'):
                found.append((proc.name, name))
        except (OSError, UnicodeError):
            continue
    if found:
        raise RuntimeError('Ya hay Gazebo activo: '+str(found)+'. No se inicia otro.')
    emit('[OK] No hay procesos gzserver/gzclient previos.')


def preflight() -> None:
    emit('=== E2.3.4 - PREFLIGHT DE GAZEBO EN SOMBRA ===')
    check((REPO / '.git').is_dir(), 'Repositorio Git localizado')
    check(git('branch', '--show-current') == 'desarrollo-avance2', 'Rama desarrollo-avance2')
    check(git('rev-parse', 'HEAD') == REF, 'Commit E2.3 publicado presente')
    for rel in TRACKED:
        p=REPO/rel
        check(p.is_file() and git('hash-object','--',rel)==git('rev-parse','HEAD:'+rel),
              'Archivo versionado intacto: '+rel)
    check(VENV.is_file() and ACTIVATE.is_file(), 'Python R18 y entorno ROS disponibles')
    script=(REPO/'tools/A2_E233_supervisor_integrado_aislado.py').read_text(encoding='utf-8')
    check("OUTPUT = '/cmd_vel_integrado_prueba'" in script and
          "STATUS = '/estado_integrado_prueba'" in script,
          'Supervisor candidato sigue en salida aislada')
    check('create_publisher(Twist,OUTPUT,10)' in script,
          'Publicador del supervisor es exclusivamente el de prueba')
    check_processes()
    emit('[PENDIENTE] Umbrales .60/.80 m provisorios; prueba sombra NO cierra R19.')


def environment():
    env=os.environ.copy()
    env['ROS_DOMAIN_ID']=DOMAIN
    env['ROS_LOCALHOST_ONLY']='1'
    return env


def sourced_python(extra: list[str]) -> list[str]:
    return ['bash','-c','source "$1" >/dev/null && exec "$2" "$3" "${@:4}"',
            'bash',str(ACTIVATE),str(VENV),str(Path(__file__).resolve()),*extra]


def probe() -> int:
    import rclpy
    from rclpy.node import Node
    import time as clock
    if os.environ.get('ROS_DOMAIN_ID') != DOMAIN or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Sonda fuera de dominio aislado')
    rclpy.init()
    node=Node('e234_inspector_pre_lanzamiento', enable_rosout=False)
    try:
        deadline=clock.monotonic()+2.5
        while clock.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.15)
        topics={t for t,_ in node.get_topic_names_and_types()}
        interferencia=sorted(t for t in topics if t not in ('/rosout','/parameter_events'))
        if node.count_publishers('/cmd_vel') or interferencia:
            emit('[BLOQUEADO] Dominio 223 ocupado: '+', '.join(interferencia))
            return 2
        emit('[OK] Dominio ROS 223 libre; sin publicadores /cmd_vel.')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def monitor(path: Path) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan, Range
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    if os.environ.get('ROS_DOMAIN_ID') != DOMAIN or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Monitor no aislado')
    # Monitor se ejecuta con un intérprete que comparte ROS y numpy compatibles.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Node('e234_monitor_gazebo_sombra')
    events=[]
    count={k:0 for k in ('scan','odom','df','di','dd','fuzzy','fuzzy_state','candidate','candidate_state')}
    t_first={}
    states={'fuzzy':[], 'candidate':[]}
    max_shadow_v=0.0
    min_shadow_v=0.0
    all_finite=True
    def take(k, detail=''):
        count[k]+=1
        if k not in t_first: t_first[k]=time.monotonic()
        if len(events)<15000:
            events.append({'t_sec':round(time.monotonic()-start,3),'topic':k,
                           'detail':str(detail)[:180]})
    def ranges(k,msg):
        take(k,round(float(msg.range),5))
    def scan_cb(msg):
        take('scan',len(msg.ranges))
    def odom_cb(msg):
        take('odom',round(float(msg.pose.pose.position.x),4))
    def twist_cb(which,msg):
        nonlocal max_shadow_v,min_shadow_v,all_finite
        vals=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
        if not all(math.isfinite(float(v)) for v in vals): all_finite=False
        if which=='candidate':
            max_shadow_v=max(max_shadow_v,float(msg.linear.x))
            min_shadow_v=min(min_shadow_v,float(msg.linear.x))
        take(which,'v=%.5f w=%+.5f'%(msg.linear.x,msg.angular.z))
    def state_cb(which,msg):
        if msg.data not in states[which]:states[which].append(msg.data)
        take(which+'_state',msg.data)
    node.create_subscription(LaserScan,'/scan',scan_cb,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/odom',odom_cb,qos_profile_sensor_data)
    node.create_subscription(Range,'/rango_frontal',lambda m:ranges('df',m),10)
    node.create_subscription(Range,'/rango_izq',lambda m:ranges('di',m),10)
    node.create_subscription(Range,'/rango_der',lambda m:ranges('dd',m),10)
    node.create_subscription(Twist,'/cmd_vel_difuso',lambda m:twist_cb('fuzzy',m),10)
    node.create_subscription(String,'/estado_difuso',lambda m:state_cb('fuzzy',m),10)
    node.create_subscription(Twist,'/cmd_vel_integrado_prueba',lambda m:twist_cb('candidate',m),10)
    node.create_subscription(String,'/estado_integrado_prueba',lambda m:state_cb('candidate',m),10)
    start=time.monotonic()
    observed_s=None
    errors=[]
    try:
        while time.monotonic()-start < DISCOVER_MAX_S+OBSERVE_S:
            rclpy.spin_once(node,timeout_sec=.15)
            if node.count_publishers('/cmd_vel') != 0:
                errors.append('Se descubrió un publicador /cmd_vel: abortar')
                break
            # No seguidor de ruta; si aparece, dominio contaminado por otro proceso.
            if node.count_publishers('/cmd_vel_ruta') != 0:
                errors.append('Se descubrió publicador /cmd_vel_ruta no autorizado')
                break
            if all(count.values()) and observed_s is None:
                observed_s=time.monotonic()
                emit('[OK] Nueve tópicos con datos reales de Gazebo/nodos E2.2 y E2.3.3.')
            if observed_s is not None and time.monotonic()-observed_s >= OBSERVE_S:
                break
        if observed_s is None:
            errors.append('No llegaron los nueve tópicos esperados dentro del periodo de arranque')
        elif time.monotonic()-observed_s < OBSERVE_S:
            errors.append('Muestras insuficientes para observación continua')
        if not all_finite:
            errors.append('Se recibió al menos un Twist no finito')
        if node.count_publishers('/cmd_vel'):
            errors.append('Existe publicador /cmd_vel al finalizar')
        if not count['scan'] or not count['odom']:
            errors.append('Sensores reales incompletos')
        ok=not errors
        summary={
            'etapa':'E2.3.4','estado':'APROBADA_SOMBRA' if ok else 'NO_APROBADA',
            'domain_id':DOMAIN,'solo_loopback':True,'gazebo':True,
            'publication_cmd_vel':False if ok else None,
            'topic_counts':count,'initial_states':states,'max_shadow_v':max_shadow_v,
            'min_shadow_v':min_shadow_v,'seconds_monitored':round(time.monotonic()-start,3),
            'errors':errors,'note':'Ni el lanzamiento ni el monitor publican en /cmd_vel. '
               'R19 operativo sigue pendiente. Umbrales .60/.80 provisionales.'}
        path.mkdir(parents=True,exist_ok=True)
        (path/'resumen.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
        with (path/'observaciones.csv').open('w',encoding='utf-8',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=['t_sec','topic','detail'],lineterminator='\n')
            writer.writeheader();writer.writerows(events)
        for topic,n in count.items():
            emit(('[OK] ' if n else '[ERROR] ')+topic+': '+str(n)+' mensajes')
        emit('[ESTADOS] Difuso='+', '.join(states['fuzzy'])+' | Supervisor='+', '.join(states['candidate']))
        if errors:
            for err in errors:emit('[ERROR] '+err)
        emit('[EVIDENCIA] '+str(path/'resumen.json'))
        emit('[EVIDENCIA] '+str(path/'observaciones.csv'))
        return 0 if ok else 2
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def terminate_own_launch(proc):
    if proc is None or proc.poll() is not None: return
    try: os.killpg(proc.pid,signal.SIGINT)
    except ProcessLookupError:return
    try: proc.wait(timeout=14)
    except subprocess.TimeoutExpired:
        emit('[WARN] Lanzamiento tardó en detenerse; enviando SIGTERM solo a su grupo.')
        try: os.killpg(proc.pid,signal.SIGTERM)
        except ProcessLookupError:return
        try:proc.wait(timeout=7)
        except subprocess.TimeoutExpired:
            emit('[WARN] Terminación forzada del grupo de lanzamiento.')
            try:os.killpg(proc.pid,signal.SIGKILL)
            except ProcessLookupError:pass


def run() -> int:
    preflight()
    env=environment()
    emit('[INFO] Sonda de dominio ROS aislado antes de Gazebo...')
    test=subprocess.run(sourced_python(['--probe']),env=env,cwd=REPO,timeout=45)
    check(test.returncode==0,'Dominio 223 libre para pruebas en sombra')
    output=REPO/'resultados/A2_E234_gazebo_sombra'/dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    output.mkdir(parents=True,exist_ok=False)
    launch=output/'almacen_e234_sombra.launch.py'
    launch.write_text(LAUNCH_SOURCE,encoding='utf-8',newline='\n')
    compile(LAUNCH_SOURCE,str(launch),'exec')
    emit('[OK] Lanzamiento alternativo preparado: '+str(launch))
    emit('[SEGURIDAD] Gazebo inicia sin seguidor de ruta ni supervisor F1.3. '+
         'Candidato solo publica /cmd_vel_integrado_prueba.')
    launch_log=(output/'gazebo_lanzamiento.log').open('w',encoding='utf-8')
    mon_log=(output/'monitor.log').open('w',encoding='utf-8')
    launcher=None; observer=None
    start_run=time.monotonic()
    try:
        cmd=['bash','-c','source "$1" >/dev/null && exec ros2 launch "$2"',
             'bash',str(ACTIVATE),str(launch)]
        launcher=subprocess.Popen(cmd,cwd=REPO,env=env,
                                  stdout=launch_log,stderr=subprocess.STDOUT,
                                  start_new_session=True)
        observer=subprocess.Popen(sourced_python(['--monitor','--out',str(output)]),
                                  cwd=REPO,env=env,stdout=mon_log,stderr=subprocess.STDOUT)
        prev=0
        while observer.poll() is None:
            if launcher.poll() is not None:
                raise RuntimeError('Gazebo/lanzamiento terminó prematuramente. Revisar gazebo_lanzamiento.log.')
            elapsed=time.monotonic()-prev if prev else 0
            # Notificar sin bloquear ni depender de otro terminal.
            if not prev or elapsed>=17:
                emit('[INFO] Lanzamiento/observación en curso; logs en '+str(output))
                prev=time.monotonic()
            if time.monotonic()-start_run > DISCOVER_MAX_S+OBSERVE_S+25:
                raise RuntimeError('Tiempo máximo de observación agotado')
            time.sleep(.6)
        rc=observer.returncode
        emit('[MONITOR] '+str(output/'monitor.log'))
        monitor_text=(output/'monitor.log').read_text(encoding='utf-8',errors='replace')
        for l in monitor_text.strip().splitlines()[-26:]:emit(l)
        if rc:
            raise RuntimeError('Monitor ROS2 no aprobado, código '+str(rc))
        check((output/'resumen.json').is_file(), 'Resumen de Gazebo en sombra generado')
        emit('=== E2.3.4: GAZEBO + ROS 2 EN SOMBRA APROBADOS; R19 OPERATIVO PENDIENTE ===')
        emit('[LOG] '+str(output/'gazebo_lanzamiento.log'))
        return 0
    except Exception:
        if launch_log and (output/'gazebo_lanzamiento.log').is_file():
            launch_log.flush()
            tail=(output/'gazebo_lanzamiento.log').read_text(encoding='utf-8',errors='replace').splitlines()[-24:]
            emit('[DIAGNOSTICO] Ultimas lineas del lanzamiento Gazebo:')
            for row in tail: emit(row)
        raise
    finally:
        if observer is not None and observer.poll() is None:
            observer.terminate()
            try:observer.wait(timeout=5)
            except subprocess.TimeoutExpired:observer.kill();observer.wait(timeout=5)
        terminate_own_launch(launcher)
        launch_log.close();mon_log.close()
        emit('[OK] Se inició cierre de los procesos del ensayo; no se tocaron procesos ajenos.')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--probe',action='store_true',help=argparse.SUPPRESS)
    ap.add_argument('--monitor',action='store_true',help=argparse.SUPPRESS)
    ap.add_argument('--out',default=None,help=argparse.SUPPRESS)
    opts=ap.parse_args()
    try:
        if opts.probe:return probe()
        if opts.monitor:
            if not opts.out:raise RuntimeError('Falta --out en monitor')
            return monitor(Path(opts.out))
        return run()
    except KeyboardInterrupt:
        emit('[CANCELADO] Se detuvo la prueba; Ubuntu permanece abierto.')
        return 130
    except Exception as error:
        emit('[ERROR/PENDIENTE] '+str(error))
        emit('[NOTA] No se modificaron archivos versionados, FIS, supervisor ni lanzamiento F1.')
        return 1

if __name__=='__main__':
    sys.exit(main())
