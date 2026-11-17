#!/usr/bin/env python3
"""A2/ACT11: diez pruebas independientes de actuación Mamdani en Gazebo.

Reutiliza sin modificar tools/A3_E426_evasion_real_gazebo.py. Captura ROS bag
(excluye /cmd_vel para respetar un control de seguridad del ensayo E426),
reconoce los dos niveles probatorios y nunca presenta una activacion momentanea
como evasión terminada. Python estándar, ROS2/Gazebo se necesitan solo en --run.
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
import re
import shutil
import signal
import sqlite3
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
PRIVATE = Path.home() / 'AMR_evidencias_privadas' / 'A2_ACT11'
FIS = ROOT / 'matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis'
FIS_SHA = '67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903'
E426 = ROOT / 'tools/A3_E426_evasion_real_gazebo.py'
ROUTE = ROOT / 'resultados/F12_ruta_2.csv'
WORLD = ROOT / 'modelos/almacen_f1.world'
BAG_TOPICS = [
    '/clock','/odom','/scan','/rango_frontal','/rango_izq','/rango_der',
    '/estado_supervisor','/cmd_vel_difuso','/estado_difuso',
    '/cmd_vel_ruta','/estacion_cnn_info',
]
MIN_BAG = {'/odom':10,'/scan':5,'/estado_supervisor':5,
           '/cmd_vel_difuso':3,'/rango_frontal':3}
COLUMNS = [
    'intento','inicio_utc','estado','intervencion_difusa_verificada',
    'evasion_completa_verificada','modo_difuso_ticks','comandos_difuso_activos',
    'movimiento_m','rango_frontal_activacion_m','obstaculo_eliminado',
    'rosbag_verificada','muestras_odom','muestras_estado_supervisor','motivo',
    'directorio_privado',
]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def need(test: bool, msg: str) -> None:
    if not test:
        raise RuntimeError(msg)


def command(args, timeout=15, env=None) -> subprocess.CompletedProcess:
    return subprocess.run([str(x) for x in args], cwd=ROOT, env=env,
                          capture_output=True, text=True, timeout=timeout)


def static_preflight() -> dict:
    need(ROOT.is_dir() and (ROOT/'.git').exists(), 'El script debe estar en la raiz del repositorio Git')
    for p in [FIS,E426,ROUTE,WORLD,ROOT/'tools/A3_nodo_mamdani_v03.py',
              ROOT/'tools/seguidor_ruta_f13.py',ROOT/'tools/supervisor_u3s7_cnn.py',
              ROOT/'tools/emulador_sensores_f1.py',ROOT/'tools/reconocedor_estacion.py']:
        need(p.is_file(),f'Fuente ausente: {p.relative_to(ROOT)}')
    need(sha(FIS)==FIS_SHA,'El FIS oficial v03 cambió: no utilizar un controlador distinto')
    s=E426.read_text(encoding='utf8')
    for token in ['RUTA->DIFUSO','obstaculo_eliminado','difuso_comandos',
                  "count_subscribers('/cmd_vel')==2",'OBSTACULO_XY = (-3.975, -1.70)']:
        need(token in s,'El ejecutor histórico E426 no coincide: '+token)
    with ROUTE.open(newline='',encoding='utf-8-sig') as f:
        points=[(float(r['x_m']),float(r['y_m'])) for r in csv.DictReader(f)]
    need(len(points)>10 and math.dist(points[0],(-3.975,-2.775))<.01,
         'La ruta 2 no inicia en Recepción; no ejecutar')
    need(all(math.dist(a,b)<=.151 for a,b in zip(points,points[1:])),
         'Ruta 2 con discontinuidades')
    git=command(['git','rev-parse','HEAD']);need(git.returncode==0,'Git inaccesible')
    branch=command(['git','branch','--show-current'])
    need(branch.returncode==0 and branch.stdout.strip()=='u3s7-entrega-final-2026-11-16',
         'Se requiere rama u3s7-entrega-final-2026-11-16; no modificar main/integracion-final')
    return dict(commit=git.stdout.strip(),branch=branch.stdout.strip(),fis_sha256=FIS_SHA,
                e426_sha256=sha(E426),ruta_sha256=sha(ROUTE),
                mundo_sha256=sha(WORLD),longitud_ruta_m=round(sum(math.dist(a,b)
                            for a,b in zip(points,points[1:])),4),
                rosbag_topics=BAG_TOPICS)


def runtime_preflight() -> Path:
    data=static_preflight()
    need(os.getenv('ROS_DISTRO')=='humble','Primero: source tools/activar_entorno.sh (ROS Humble)')
    need(os.getenv('ROS_DOMAIN_ID')=='173','Se necesita ROS_DOMAIN_ID=173')
    need(os.getenv('ROS_LOCALHOST_ONLY')=='1','Se necesita ROS_LOCALHOST_ONLY=1')
    for app in ['ros2','gzserver','git']:
        need(shutil.which(app) is not None,'No se encuentra ejecutable: '+app)
    need((Path.home()/'.venvs/amr-a2-r18/bin/python').is_file(),
         'Falta virtualenv del controlador A3 Mamdani v03')
    need(not command(['pgrep','-x','gzserver'],3).stdout.strip(),
         'Hay otro gzserver activo. Cierra antes Gazebo; no se matan procesos ajenos.')
    active=command(['ros2','topic','list'],12)
    need(active.returncode==0,'No se puede consultar el grafo ROS2')
    for topic in ('/odom','/cmd_vel'):
        need(topic not in active.stdout.splitlines(),
             'Hay una simulación activa en el dominio 173 ('+topic+'). Ciérrala primero.')
    pkg=command(['ros2','pkg','prefix','turtlebot3_gazebo'],15)
    need(pkg.returncode==0,'No se encuentra paquete turtlebot3_gazebo')
    model=Path(pkg.stdout.strip())/'share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf'
    need(model.is_file(),f'Modelo SDF ausente: {model}')
    free=shutil.disk_usage(PRIVATE.parent if PRIVATE.parent.exists() else Path.home()).free
    need(free>2*1024**3,'Se requieren al menos 2 GiB libres para 10 ROS bags')
    print('[OK] Fuentes, FIS, ruta, Gazebo y aislamiento validados')
    print('[INFO] Commit:',data['commit'])
    print('[INFO] ROS bag sin /cmd_vel por precondición E426; comandos aplicados auditados en vivo por E426')
    return model


def owned_start(args, logfile: Path, env=None):
    handle=logfile.open('w',encoding='utf-8')
    try:
        proc=subprocess.Popen([str(x) for x in args],cwd=ROOT,
                              env=env or os.environ.copy(),stdout=handle,
                              stderr=subprocess.STDOUT,start_new_session=True)
    except BaseException:
        handle.close();raise
    return proc,handle


def owned_stop(proc, grace=12):
    if proc is None:return
    if proc.poll() is not None:return
    for sign,timeout in ((signal.SIGINT,grace),(signal.SIGTERM,5),(signal.SIGKILL,2)):
        if proc.poll() is not None:break
        try:os.killpg(proc.pid,sign)
        except ProcessLookupError:break
        try:proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:continue


def wait_ros_service(srv,timeout,server):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if server.poll() is not None:raise RuntimeError('Gazebo finalizó antes de habilitar '+srv)
        p=command(['ros2','service','list'],8)
        if p.returncode==0 and srv in p.stdout.splitlines():return
        time.sleep(1)
    raise TimeoutError('No apareció el servicio '+srv)


def bags_stats(bagdir: Path) -> dict:
    counts={}
    need((bagdir/'metadata.yaml').is_file(),'ROS bag sin metadata.yaml')
    dbs=list(bagdir.glob('*.db3'))
    need(dbs,'ROS bag sin archivos .db3')
    for db in dbs:
        need(db.stat().st_size>4096,'Archivo rosbag SQLite vacío')
        cx=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
        try:
            check=cx.execute('PRAGMA integrity_check').fetchone()[0]
            need(check=='ok','SQLite dañada: '+str(db))
            for topic,count in cx.execute('SELECT t.name,COUNT(m.id) FROM topics t LEFT JOIN messages m ON t.id=m.topic_id GROUP BY t.name'):
                counts[topic]=counts.get(topic,0)+count
        finally:cx.close()
    return counts


def classify(folder:Path,index:int) -> dict:
    meta_path=folder/'metadata.json'
    info=json.loads(meta_path.read_text(encoding='utf8')) if meta_path.is_file() else {}
    row={'intento':index,'inicio_utc':info.get('inicio_utc',''),
         'estado':'FALLO','intervencion_difusa_verificada':False,
         'evasion_completa_verificada':False,'modo_difuso_ticks':0,
         'comandos_difuso_activos':0,'movimiento_m':'',
         'rango_frontal_activacion_m':'','obstaculo_eliminado':False,
         'rosbag_verificada':False,'muestras_odom':0,
         'muestras_estado_supervisor':0,'motivo':'Sin resumen de E426',
         'directorio_privado':str(folder)}
    summary=folder/'captura_E426/resumen.json'
    try:
        counts=bags_stats(folder/'rosbag')
        row['muestras_odom']=counts.get('/odom',0)
        row['muestras_estado_supervisor']=counts.get('/estado_supervisor',0)
        row['rosbag_verificada']=all(counts.get(t,0)>=n for t,n in MIN_BAG.items())
        (folder/'conteos_rosbag.json').write_text(json.dumps(counts,indent=2,sort_keys=True)+'\n')
    except Exception as e:
        row['motivo']='ROS bag: '+str(e)
    if not summary.is_file():return row
    d=json.loads(summary.read_text(encoding='utf8'))
    row['modo_difuso_ticks']=int(d.get('muestras_modo_difuso_posteriores_ruta',0))
    row['comandos_difuso_activos']=int(d.get('comandos_activos_difuso',0))
    row['movimiento_m']=d.get('movimiento_acumulado_m','')
    row['obstaculo_eliminado']=d.get('obstaculo_eliminado',False) is True
    sample=d.get('rango_al_activar_difuso') or {}
    row['rango_frontal_activacion_m']=sample.get('frontal_range','')
    bounded=any(isinstance(sample.get(k),(int,float)) and math.isfinite(sample[k])
                and sample[k] <= threshold for k,threshold in
                (('frontal_range',.600001),('izq_range',.350001),('der_range',.350001)))
    ok=(d.get('aprobado') is True and d.get('transicion')=='RUTA->DIFUSO'
        and d.get('fis_sha256')==FIS_SHA and d.get('error') is None
        and row['comandos_difuso_activos']>=1 and row['modo_difuso_ticks']>=3
        and float(d.get('movimiento_acumulado_m',0))>=.12 and bounded
        and row['obstaculo_eliminado'] and row['rosbag_verificada'])
    row['intervencion_difusa_verificada']=bool(ok)
    # E426 intencionalmente aborta tras transición: NO acredita rodeo ni llegada.
    row['evasion_completa_verificada']=False
    row['estado']='INTERVENCION_DIFUSA_VERIFICADA' if ok else 'NO_ACREDITADA'
    row['motivo']=('E426 documenta activacion, no rodeo completo del obstaculo'
                   if ok else 'Comprobar resumen.json, logs, umbral y conteos ROS bag')
    return row


def dump_csv(rows,out):
    with out.open('w',newline='',encoding='utf-8') as h:
        writer=csv.DictWriter(h,fieldnames=COLUMNS)
        writer.writeheader();writer.writerows(rows)


def summarize(session:Path,expected:int) -> dict:
    dirs=sorted(p for p in (session/'intentos').glob('intento_*') if p.is_dir())
    rows=[classify(p,int(re.search(r'(\d+)$',p.name).group(1))) for p in dirs]
    dump_csv(rows,session/'tabla_10_intentos.csv')
    a=sum(x['intervencion_difusa_verificada'] for x in rows)
    full=sum(x['evasion_completa_verificada'] for x in rows)
    res={'esperados':expected,'ejecutados':len(rows),'intervenciones_mamdani_verificadas':a,
         'evasiones_completas_verificadas':full,
         'estado':('10_INTERVENCIONES_VERIFICADAS_EVASIONES_COMPLETAS_NO_DEMOSTRADAS'
                   if len(rows)==expected and a==expected else 'INCOMPLETO_O_NO_ACREDITADO'),
         'nota':'Las repeticiones E426 solo acreditan RUTA->DIFUSO con movimiento; no prueba de rodeo completo ni cero contacto.',
         'resultados':rows}
    (session/'consolidado.json').write_text(json.dumps(res,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    lines=['# A2 · Actividad 11 — Registro de episodios difusos (ejecución nueva)', '',
           f'- Sesión privada: `{session.name}`.',
           f'- Intentos registrados: **{len(rows)}/{expected}**.',
           f'- Intervenciones Mamdani verificadas con ROS bag: **{a}/{expected}**.',
           f'- Evasiones completas acreditadas: **{full}**.',
           '- Modalidad: Gazebo, obstáculo temporal, FIS v03; verificación E426 de RUTA→DIFUSO.',
           '- Límite: no se midió contacto físico; no se probó llegada al destino ni paso completo alrededor de la caja.',
           '- `/cmd_vel` se audita mediante el monitor E426, pero no se incluye en ROS bag por su control de suscriptores; el bag conserva `/estado_supervisor` y `/cmd_vel_difuso`.',
           '', '| Intento | Intervención | Evasión completa | Comandos DIFUSO | ROS bag |',
           '|---:|---|---|---:|---|']
    for row in rows:
        lines.append(f"| {row['intento']} | {'Sí' if row['intervencion_difusa_verificada'] else 'No'} | No comprobada | {row['comandos_difuso_activos']} | {'Válido' if row['rosbag_verificada'] else 'No válido'} |")
    lines+=['', '**Interpretación obligatoria:** la ejecución repetida del ensayo E426 prueba actuación difusa reactiva. No debe titularse «diez evasiones completas» sin un ensayo adicional de rodeo y retorno a ruta.',
             '', 'Archivos brutos (bags, logs y trazas): almacenamiento privado; no se incluyen en este resumen.', '']
    (session/'A2_ACT11_RESULTADO_PUBLICABLE.md').write_text('\n'.join(lines),encoding='utf8')
    return res


def attempt(index: int,folder:Path,model:Path) -> None:
    folder.mkdir(parents=True,exist_ok=False)
    (folder/'metadata.json').write_text(json.dumps({
        'intento':index,'inicio_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
        'git_commit':command(['git','rev-parse','HEAD']).stdout.strip(),
        'fis_sha256':FIS_SHA,'script_e426_sha256':sha(E426),
        'tipo':'intervencion_mamdani_E426_no_evasion_completa'
    },indent=2,ensure_ascii=False)+'\n')
    server=bag=worker=None
    handles=[]
    old={p.name for p in (ROOT/'resultados/A3_E426_evasion_real_gazebo').glob('*') if p.is_dir()}
    print(f'\n[INTENTO {index:02d}] Iniciando Gazebo aislado',flush=True)
    try:
        server,h=owned_start(['gzserver','--verbose','-s','libgazebo_ros_init.so',
                               '-s','libgazebo_ros_factory.so',WORLD],folder/'gazebo.log');handles.append(h)
        wait_ros_service('/spawn_entity',65,server)
        p=command(['ros2','run','gazebo_ros','spawn_entity.py',
                   '-entity','waffle_pi','-file',model,'-x','-3.975',
                   '-y','-2.775','-z','0.05','-Y',str(math.pi/2)],85)
        (folder/'spawn.log').write_text(p.stdout+'\n--- STDERR ---\n'+p.stderr,encoding='utf8')
        need(p.returncode==0,'Gazebo rechazó spawn; consultar spawn.log')
        print(f'[INTENTO {index:02d}] Robot ubicado; iniciando ROS bag',flush=True)
        bag,h=owned_start(['ros2','bag','record','--storage','sqlite3',
                           '-o',folder/'rosbag',*BAG_TOPICS],folder/'rosbag.log');handles.append(h)
        time.sleep(3)
        need(bag.poll() is None,'ROS bag finalizó prematuramente')
        need(server.poll() is None,'gzserver se cerró antes de E426')
        worker,h=owned_start([sys.executable,'-u',E426],folder/'ejecutor_E426.log')
        handles.append(h)
        try:
            rc=worker.wait(timeout=125)
        except subprocess.TimeoutExpired:
            owned_stop(worker,8)
            raise TimeoutError('E426 excedió 125 s; su grupo de procesos se detuvo por seguridad')
        (folder/'codigo_E426.txt').write_text(str(rc)+'\n',encoding='utf8')
        print(f'[INTENTO {index:02d}] E426 terminó con código {rc}',flush=True)
    except KeyboardInterrupt:
        raise
    except Exception as e:
        (folder/'error_ejecucion.txt').write_text(f'{type(e).__name__}: {e}\n',encoding='utf8')
        print(f'[INTENTO {index:02d}] Fallo: {e}',flush=True)
    finally:
        owned_stop(worker,8)
        owned_stop(bag,15)
        owned_stop(server,10)
        for h in handles:h.close()
        runbase=ROOT/'resultados/A3_E426_evasion_real_gazebo'
        new=[p for p in runbase.iterdir() if p.is_dir() and p.name not in old] if runbase.exists() else []
        if len(new)==1:
            source=new[0]
            shutil.copytree(source,folder/'captura_E426')
            # Retirar solamente archivos creados por ESTA ejecución, después de copiarlos.
            for path in source.glob('F12_ruta_2_E426_*.csv'):
                temp=ROOT/'resultados'/('F13_'+path.stem.removeprefix('F12_')+'_log.csv')
                if temp.is_file():
                    shutil.copy2(temp,folder/'trayectoria_seguidor_generada.csv')
                    temp.unlink()
            shutil.rmtree(source)
        elif len(new)>1:
            (folder/'aviso_captura.txt').write_text('Hay múltiples carpetas E426 nuevas; revisión manual necesaria. No se borró ninguna.\n')
        try:
            row=classify(folder,index)
            print(f"[INTENTO {index:02d}] {row['estado']} | bag={row['rosbag_verificada']} | comandos={row['comandos_difuso_activos']}",flush=True)
        except Exception as exc:
            print(f'[INTENTO {index:02d}] No se pudo clasificar: {exc}',flush=True)


def run(count):
    model=runtime_preflight()
    PRIVATE.mkdir(parents=True,exist_ok=True)
    session=PRIVATE/('sesion_'+dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    session.mkdir(exist_ok=False)
    (session/'intentos').mkdir()
    data=static_preflight();data.update({'inicio_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
                                        'intentos_programados':count})
    (session/'manifest_fuentes.json').write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
    print('[SESIÓN PRIVADA]',session,flush=True)
    print('[PRECAUCIÓN] Cada intento reinicia el Gazebo que crea este programa; no afecta procesos ajenos',flush=True)
    confirm=input('¿Iniciar movimiento AUTÓNOMO en Gazebo, diez episodios repetidos? [s/N] ').strip().lower()
    if confirm!='s':print('[CANCELADO] Sin movimiento');return 0
    try:
        for i in range(1,count+1):
            attempt(i,session/'intentos'/f'intento_{i:02d}',model)
            summarize(session,count)
    except KeyboardInterrupt:
        print('[INTERRUMPIDO] Resultados existentes conservados',flush=True)
    results=summarize(session,count)
    print('[RESULTADO]',results['estado'])
    print('[INTERVENCIONES]',results['intervenciones_mamdani_verificadas'],'/',count)
    print('[EVASIONES COMPLETAS] No verificadas en este protocolo')
    print('[RUTA]',session)
    print('[RESUMEN]',session/'A2_ACT11_RESULTADO_PUBLICABLE.md')
    return 0 if results['intervenciones_mamdani_verificadas']==count else 3


def self_test():
    with tempfile.TemporaryDirectory(prefix='a2_act11_selftest_') as d:
        p=Path(d)/'rosbag';p.mkdir();db=p/'test.db3'
        c=sqlite3.connect(db)
        c.executescript('CREATE TABLE topics (id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT,offered_qos_profiles TEXT);'
                        'CREATE TABLE messages (id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
        for i,(topic,n) in enumerate(MIN_BAG.items(),1):
            c.execute('INSERT INTO topics VALUES (?,?,?,?,?)',(i,topic,'std_msgs/msg/String','cdr',''))
            c.executemany('INSERT INTO messages (topic_id,timestamp,data) VALUES (?,?,?)',[(i,j,b'x') for j in range(n)])
        c.commit();c.close();(p/'metadata.yaml').write_text('rosbag2_bagfile_information:\n')
        need(all(bags_stats(p)[t]>=n for t,n in MIN_BAG.items()),'Autoprueba de conteos falló')
        rowbase=Path(d)/'intentos/intento_01';(rowbase/'captura_E426').mkdir(parents=True)
        rowbase.joinpath('metadata.json').write_text('{}')
        shutil.copytree(p,rowbase/'rosbag')
        x={'aprobado':True,'transicion':'RUTA->DIFUSO','fis_sha256':FIS_SHA,
           'error':None,'muestras_modo_difuso_posteriores_ruta':3,
           'comandos_activos_difuso':3,'movimiento_acumulado_m':.4,
           'obstaculo_eliminado':True,'rango_al_activar_difuso':{'frontal_range':.55}}
        rowbase.joinpath('captura_E426/resumen.json').write_text(json.dumps(x))
        row=classify(rowbase,1)
        need(row['intervencion_difusa_verificada'] and not row['evasion_completa_verificada'],
             'Nunca convertir activación E426 en evasión completa')
        x['obstaculo_eliminado']=False
        rowbase.joinpath('captura_E426/resumen.json').write_text(json.dumps(x))
        need(not classify(rowbase,1)['intervencion_difusa_verificada'],
             'No se debe aprobar si falla limpieza de obstáculo')
    print('[RESULTADO] AUTOPRUEBA A2-ACT11 APROBADA (fixtures sintéticos SOLO en carpeta temporal)')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    group=ap.add_mutually_exclusive_group(required=True)
    group.add_argument('--self-test',action='store_true')
    group.add_argument('--preflight',action='store_true')
    group.add_argument('--run',action='store_true')
    group.add_argument('--check',type=Path,metavar='SESION_PRIVADA')
    ap.add_argument('--count',type=int,default=10)
    a=ap.parse_args()
    need(1<=a.count<=10,'Solo se permiten entre 1 y 10 intentos')
    if a.self_test:self_test();return 0
    if a.preflight:
        static_preflight();print('[RESULTADO] PREVALIDACIÓN ESTÁTICA A2-ACT11 APROBADA; no implica nuevas evasiones')
        return 0
    if a.check:
        r=summarize(a.check,a.count)
        print('[RESULTADO]',r['estado'])
        print('[INTERVENCIONES]',r['intervenciones_mamdani_verificadas'],'/',a.count)
        print('[EVASIONES COMPLETAS]',r['evasiones_completas_verificadas'],'(E426 no acredita rodeo)')
        return 0 if r['intervenciones_mamdani_verificadas']==a.count else 2
    return run(a.count)


if __name__=='__main__':
    try:sys.exit(main())
    except Exception as exc:
        print('[ERROR ACT11]',type(exc).__name__+':',exc,file=sys.stderr)
        sys.exit(2)
