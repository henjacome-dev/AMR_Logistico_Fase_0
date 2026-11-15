#!/usr/bin/env python3
"""AMR A3 E4.2.6: obstaculo geometrico real de Gazebo, RUTA->DIFUSO.

Simulacion exclusivamente. Conserva FIS v03 y archivos historicos. Introduce
una unica caja temporal en el pasillo de ruta 2, la elimina al terminar.
No acredita finalizacion de mision ni ausencia de contacto con sensor fisico.
"""
from __future__ import annotations

import collections
import csv
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import rclpy
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
FIS = ROOT / 'matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis'
FIS_SHA = '67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903'
VENV = Path.home() / '.venvs/amr-a2-r18/bin/python'
RUTA = ROOT / 'resultados/F12_ruta_2.csv'
INICIO = (-3.975, -2.775)
OBSTACULO_XY = (-3.975, -1.70)
NOMBRE = 'amr_e426_obstaculo_' + dt.datetime.now().strftime('%H%M%S%f')
RANGOS = ('/rango_frontal', '/rango_izq', '/rango_der')


def esperar_futuro(nodo: Node, futuro, segundos=10.0):
    limite = time.monotonic() + segundos
    while not futuro.done() and time.monotonic() < limite:
        rclpy.spin_once(nodo, timeout_sec=0.08)
    if not futuro.done():
        raise TimeoutError('Servicio Gazebo no respondio')
    if futuro.exception() is not None:
        raise RuntimeError(str(futuro.exception()))
    return futuro.result()


def main():
    assert os.getenv('ROS_DOMAIN_ID') == '173', 'Se requiere dominio 173 de Gazebo'
    assert os.getenv('ROS_LOCALHOST_ONLY') == '1'
    assert subprocess.run(['git','-C',str(ROOT),'rev-parse','--is-inside-work-tree'],
                          capture_output=True, text=True, check=True).stdout.strip() == 'true'
    assert FIS.is_file() and hashlib.sha256(FIS.read_bytes()).hexdigest() == FIS_SHA
    assert VENV.is_file() and RUTA.is_file()
    assert (ROOT / 'tools/A3_nodo_mamdani_v03.py').is_file()
    assert (ROOT / 'tools/A3_E42_integracion_gazebo.py').is_file()
    with RUTA.open(newline='', encoding='utf-8-sig') as h:
        puntos = [(float(r['x_m']),float(r['y_m'])) for r in csv.DictReader(h)]
    assert math.dist(puntos[0], INICIO) < 0.001
    assert all(math.dist(a,b) < 0.151 for a,b in zip(puntos,puntos[1:]))

    marca = dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    carpeta = ROOT / 'resultados/A3_E426_evasion_real_gazebo' / marca
    carpeta.mkdir(parents=True, exist_ok=False)
    logs, procesos, subs = {}, {}, []
    creado = False
    solicitud_spawn = False
    auditor = None
    pub_emergencia = None
    error = None
    navegando = False
    inicio_navegacion = None
    res = dict(aprobado=False, escenario='obstaculo_fisico_temporal_gazebo',
               ruta='resultados/F12_ruta_2.csv', obstaculo=OBSTACULO_XY,
               obstaculo_nombre=NOMBRE, fis_sha256=FIS_SHA)
    estado = dict(pose=None, pose_t=None, frontal_lidar=None, laser_t=None,
                  rango={t: None for t in RANGOS}, rango_t={t: None for t in RANGOS},
                  n_rangos=collections.Counter(), modo=None, modos=collections.Counter(),
                  ruta_propuestas=0, mamdani_propuestas=0, cnn=0,
                  difuso_comandos=0, min_lidar=float('inf'),
                  max_v=0.0, max_w=0.0, transiciones=[],
                  muestra_difuso=None, ultimo_cmd=(0., 0.), distancia_movida=0.,
                  modo_ruta_primero=False, ticks_difuso=0)
    traza=[]

    rclpy.init()
    try:
        auditor = Node('auditor_e426_evasion_gazebo')
        assert auditor.count_publishers('/cmd_vel') == 0, 'Otro controlador activo en /cmd_vel'
        for topic in ('/cmd_vel_ruta','/cmd_vel_difuso','/estado_difuso',
                      '/estacion_cnn_info',*RANGOS):
            assert auditor.count_publishers(topic) == 0, f'Nodo previo activo: {topic}'

        # Espera DDS: otro nodo puede tardar en aparecer.
        ok_robot = False
        for _ in range(100):
            informacion = auditor.get_subscriptions_info_by_topic('/cmd_vel')
            if (len(informacion)==1 and
                    informacion[0].node_name=='turtlebot3_diff_drive'):
                ok_robot = True
                break
            rclpy.spin_once(auditor, timeout_sec=.1)
        assert ok_robot, 'No se encuentra controlador unico turtlebot3_diff_drive'

        def odom(msg):
            p=msg.pose.pose.position
            if math.isfinite(p.x) and math.isfinite(p.y):
                xy=(float(p.x),float(p.y))
                if inicio_navegacion is not None and estado['pose'] is not None:
                    estado['distancia_movida'] += min(.1, math.dist(xy,estado['pose']))
                estado['pose']=xy
                estado['pose_t']=time.monotonic()
        def laser(msg):
            lecturas=[]
            for i,v in enumerate(msg.ranges):
                a=math.atan2(math.sin(msg.angle_min+i*msg.angle_increment),
                             math.cos(msg.angle_min+i*msg.angle_increment))
                if abs(a)>math.radians(25): continue
                if math.isinf(v) and v>0: v=msg.range_max
                if math.isfinite(v) and msg.range_min <= v <= msg.range_max:
                    lecturas.append(float(v))
            estado['frontal_lidar']=min(lecturas) if lecturas else None
            estado['laser_t']=time.monotonic()
        def rango(topic,msg):
            estado['rango'][topic]=float(msg.range)
            estado['rango_t'][topic]=time.monotonic()
            estado['n_rangos'][topic]+=1
        def estado_supervisor(msg):
            try:
                d=json.loads(msg.data)
                modo=d['modo']
            except (ValueError,TypeError,KeyError): return
            estado['modos'][modo]+=1
            anterior=estado['modo']
            if anterior!=modo:
                print(f'[MODO] {anterior} -> {modo}: {d.get("motivo")}',flush=True)
                estado['transiciones'].append([anterior,modo,d.get('motivo')])
                estado['modo']=modo
            if navegando and modo=='RUTA': estado['modo_ruta_primero']=True
            if navegando and modo=='DIFUSO' and estado['modo_ruta_primero']:
                estado['ticks_difuso']+=1
                if ((float(d.get('v',0))>0.001 or abs(float(d.get('w',0)))>0.05)
                        and estado['muestra_difuso'] is None):
                    estado['muestra_difuso']={
                        'posicion':estado['pose'], 'v':d.get('v'), 'w':d.get('w'),
                        'frontal_range':estado['rango']['/rango_frontal'],
                        'izq_range':estado['rango']['/rango_izq'],
                        'der_range':estado['rango']['/rango_der'],
                        'lidar_frontal':estado['frontal_lidar']}
            if navegando:
                traza.append({'t_s':round(time.monotonic()-inicio_navegacion,3),
                              'x_m': '' if estado['pose'] is None else round(estado['pose'][0],4),
                              'y_m': '' if estado['pose'] is None else round(estado['pose'][1],4),
                              'modo':modo,'motivo':d.get('motivo'),
                              'v_m_s':d.get('v'),'omega_rad_s':d.get('w'),
                              'frontal_range_m':estado['rango']['/rango_frontal'],
                              'lidar_frontal_m':estado['frontal_lidar']})
        def comando(msg):
            v=float(msg.linear.x); w=float(msg.angular.z)
            estado['ultimo_cmd']=(v,w)
            estado['max_v']=max(estado['max_v'],abs(v))
            estado['max_w']=max(estado['max_w'],abs(w))
            if navegando and estado['modo']=='DIFUSO' and (v>.001 or abs(w)>.05):
                estado['difuso_comandos']+=1
        def p_ruta(msg): estado.__setitem__('ruta_propuestas',estado['ruta_propuestas']+1)
        def p_difuso(msg): estado.__setitem__('mamdani_propuestas',estado['mamdani_propuestas']+1)
        def cnn(msg):
            try:
                d=json.loads(msg.data)
                if isinstance(d,dict): estado['cnn']+=1
            except ValueError: pass

        subs.extend([
            auditor.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data),
            auditor.create_subscription(LaserScan,'/scan',laser,qos_profile_sensor_data),
            auditor.create_subscription(String,'/estado_supervisor',estado_supervisor,10),
            auditor.create_subscription(Twist,'/cmd_vel',comando,10),
            auditor.create_subscription(Twist,'/cmd_vel_ruta',p_ruta,10),
            auditor.create_subscription(Twist,'/cmd_vel_difuso',p_difuso,10),
            auditor.create_subscription(String,'/estacion_cnn_info',cnn,10),
        ])
        for topic in RANGOS:
            subs.append(auditor.create_subscription(
                Range,topic,lambda msg,t=topic:rango(t,msg),qos_profile_sensor_data))

        print('=== PREVUELO GAZEBO Y RUTA 2 ===',flush=True)
        tope=time.monotonic()+12
        while time.monotonic()<tope:
            rclpy.spin_once(auditor,timeout_sec=.08)
            if (estado['pose'] is not None and estado['frontal_lidar'] is not None
                    and time.monotonic()-estado['pose_t'] < 1.5
                    and time.monotonic()-estado['laser_t'] < 1.5): break
        assert estado['pose'] is not None and estado['frontal_lidar'] is not None, 'Gazebo sin sensores'
        print('[POSE]',estado['pose'],'| LiDAR frontal',estado['frontal_lidar'])
        assert math.dist(estado['pose'],INICIO)<=.25, 'Robot fuera del inicio de ruta 2 (RECEPCION)'
        assert estado['frontal_lidar']>=.35, 'Obstaculo frontal critico antes del ensayo'
        assert math.dist(estado['pose'],OBSTACULO_XY)>.65, 'Robot muy cerca del obstaculo previsto'
        assert auditor.count_publishers('/cmd_vel')==0
        assert auditor.count_subscribers('/cmd_vel')==2, 'Esperado un actuador y el auditor'

        print('=== INSERTAR OBSTACULO FISICO TEMPORAL ===',flush=True)
        cliente_spawn=auditor.create_client(SpawnEntity,'/spawn_entity')
        cliente_delete=auditor.create_client(DeleteEntity,'/delete_entity')
        assert cliente_spawn.wait_for_service(timeout_sec=8), 'No existe servicio /spawn_entity'
        assert cliente_delete.wait_for_service(timeout_sec=8), 'No existe servicio /delete_entity'
        xml='''<?xml version="1.0"?>
<sdf version="1.6"><model name="amr_caja_e426"><static>true</static>
<link name="bloque"><collision name="col"><geometry><box><size>0.30 0.30 0.30</size></box></geometry></collision>
<visual name="vis"><geometry><box><size>0.30 0.30 0.30</size></box></geometry>
<material><ambient>1 0.15 0.05 1</ambient><diffuse>1 0.15 0.05 1</diffuse></material></visual>
</link></model></sdf>'''
        solicitud=SpawnEntity.Request()
        solicitud.name=NOMBRE
        solicitud.xml=xml
        solicitud.robot_namespace=''
        solicitud.reference_frame='world'
        solicitud.initial_pose=Pose()
        solicitud.initial_pose.position.x=OBSTACULO_XY[0]
        solicitud.initial_pose.position.y=OBSTACULO_XY[1]
        solicitud.initial_pose.position.z=.15
        solicitud.initial_pose.orientation.w=1.0
        solicitud_spawn=True
        respuesta=esperar_futuro(auditor,cliente_spawn.call_async(solicitud),10)
        assert respuesta.success, f'Gazebo rechazo caja: {respuesta.status_message}'
        creado=True
        print('[OK] Caja real en Gazebo colocada:',NOMBRE,OBSTACULO_XY,flush=True)
        # Pausa para que LiDAR de Gazebo incorpore la nueva geometria.
        tiempo=time.monotonic()+1
        while time.monotonic()<tiempo:rclpy.spin_once(auditor,timeout_sec=.06)

        def iniciar(nombre,interpreter,script,extras=()):
            handle=(carpeta/f'{nombre}.log').open('w',encoding='utf-8')
            logs[nombre]=handle
            p=subprocess.Popen([str(interpreter),'-u',str(ROOT/script),*map(str,extras)],
                               cwd=ROOT,env={**os.environ,'PYTHONUNBUFFERED':'1'},
                               stdout=handle,stderr=subprocess.STDOUT)
            procesos[nombre]=p
            print(f'[INICIADO] {nombre} PID={p.pid}',flush=True)

        print('=== INICIAR NODOS INTEGRADOS ===',flush=True)
        iniciar('emulador',sys.executable,'tools/emulador_sensores_f1.py')
        iniciar('supervisor',sys.executable,'tools/supervisor_u3s7_cnn.py',
               ('--ros-args','-p','use_sim_time:=true'))
        iniciar('mamdani_v03',VENV,'tools/A3_nodo_mamdani_v03.py',
               ('--ros','--ros-args','-p','use_sim_time:=true'))
        iniciar('cnn',sys.executable,'tools/reconocedor_estacion.py')
        tope=time.monotonic()+30
        listo=False
        while time.monotonic()<tope:
            rclpy.spin_once(auditor,timeout_sec=.08)
            assert all(p.poll() is None for p in procesos.values()),'Subsistema termino antes de navegar'
            listo=(auditor.count_publishers('/cmd_vel')==1 and
                   auditor.count_publishers('/cmd_vel_difuso')==1 and
                   auditor.count_publishers('/estacion_cnn_info')==1 and
                   all(auditor.count_publishers(t)==1 and estado['n_rangos'][t]>=3
                       for t in RANGOS) and
                   estado['cnn']>=2 and estado['mamdani_propuestas']>=3 and
                   estado['modos']['DETENIDO']>=2)
            if listo:break
        assert listo,'Subsistemas no listos (consultar logs)'
        assert estado['max_v'] <1e-9, 'Movimiento antes de iniciar seguidor'
        assert math.dist(estado['pose'],INICIO)<=.25
        print('[OK] Sistema conectado, sin movimiento previo',flush=True)

        # Ruta local con nombre univoco: seguidor genera log historico separado.
        copia=carpeta/f'F12_ruta_2_E426_{marca}.csv'
        shutil.copy2(RUTA,copia)
        print('=== NAVEGACION CON OBSTACULO GAZEBO ===',flush=True)
        iniciar('seguidor',sys.executable,'tools/seguidor_ruta_f13.py',
               ('--ruta',copia.relative_to(ROOT),'--ros-args','-p','use_sim_time:=true'))
        navegando=True
        inicio_navegacion=time.monotonic()
        proximo_reporte=inicio_navegacion
        tope=inicio_navegacion+75
        suficiente=False
        while time.monotonic()<tope:
            rclpy.spin_once(auditor,timeout_sec=.07)
            if any(procesos[n].poll() is not None for n in
                   ('emulador','supervisor','mamdani_v03','cnn')):
                raise RuntimeError('Un subsistema termino durante el ensayo')
            if auditor.count_publishers('/cmd_vel')!=1:
                raise RuntimeError('Publicadores /cmd_vel inesperados')
            if estado['max_v']>.1201 or estado['max_w']>.7001:
                raise RuntimeError('Limite de velocidad del supervisor superado')
            # Despeje aproximado centro a caja: se exige margen al detener prueba.
            if estado['pose'] is not None:
                dx=abs(estado['pose'][0]-OBSTACULO_XY[0])
                dy=abs(estado['pose'][1]-OBSTACULO_XY[1])
                # Contacto geometrico aproximado, robot radius aprox. 0.18.
                if math.hypot(max(dx-.15,0),max(dy-.15,0))<.19:
                    raise RuntimeError('Robot demasiado cercano a la caja: abortar')
            suficiente=(estado['modo_ruta_primero'] and estado['ticks_difuso']>=3
                        and estado['muestra_difuso'] is not None
                        and estado['difuso_comandos']>=1
                        and estado['distancia_movida']>=.12
                        and estado['cnn']>0
                        and estado['ruta_propuestas']>0
                        and estado['mamdani_propuestas']>0)
            if suficiente:
                print('[OK] Transicion RUTA -> DIFUSO con movimiento y sensores Gazebo',flush=True)
                break
            if procesos['seguidor'].poll() is not None:
                raise RuntimeError('El seguidor termino antes de activar modo DIFUSO')
            if time.monotonic()>=proximo_reporte:
                print('[AVANCE] pose',estado['pose'],'| RUTA',estado['modos']['RUTA'],
                      '| DIFUSO',estado['modos']['DIFUSO'],
                      '| frente Range',estado['rango']['/rango_frontal'],flush=True)
                proximo_reporte=time.monotonic()+5
        if not suficiente:raise TimeoutError('No se observo transicion difusa dentro del plazo')
        # Evitar confundir un comando residual con la observacion por obstaculo.
        m=estado['muestra_difuso']
        assert m is not None
        assert any(x is not None and math.isfinite(x) and x<=.600001
                   for x in (m['frontal_range'],
                             m['izq_range'] if m['izq_range'] is not None and m['izq_range']<=.35 else None,
                             m['der_range'] if m['der_range'] is not None and m['der_range']<=.35 else None)), (
            'DIFUSO no coincide con umbral de sensor Range')
        assert m['lidar_frontal'] is not None and math.isfinite(m['lidar_frontal']), 'Sin LiDAR durante DIFUSO'
        assert hashlib.sha256(FIS.read_bytes()).hexdigest()==FIS_SHA
        res.update(aprobado=True, transicion='RUTA->DIFUSO',
                   movimiento_acumulado_m=round(estado['distancia_movida'],4),
                   modo_ruta_ticks=estado['modos']['RUTA'],
                   modo_difuso_ticks=estado['modos']['DIFUSO'],
                   muestras_modo_difuso_posteriores_ruta=estado['ticks_difuso'],
                   comandos_activos_difuso=estado['difuso_comandos'],
                   cnn_inferencias=estado['cnn'],
                   propuestas_ruta=estado['ruta_propuestas'],
                   propuestas_mamdani_v03=estado['mamdani_propuestas'],
                   max_v_m_s=round(estado['max_v'],5),
                   max_w_rad_s=round(estado['max_w'],5),
                   rango_al_activar_difuso=m,
                   posicion_final_ensayo=estado['pose'],
                   nota='Ensayo termina al comprobar cambio modal; no equivale a ruta completa ni valida colisiones por contacto.')
        print('=== RESULTADO EVASION REAL ===')
        print(json.dumps(res,ensure_ascii=False,indent=2),flush=True)
        print('[OK] E4.2.6: MODO DIFUSO ACTIVADO EN NAVEGACION GAZEBO',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        error=f'{type(exc).__name__}: {exc}'
        print('[ERROR/INTERRUPCION]',error,flush=True)
    finally:
        print('=== PARADA Y LIMPIEZA SEGURA ===',flush=True)
        navegando=False
        # Parar antes de retirar obstaculo; solo procesos creados aqui.
        for nombre in ('seguidor','cnn','mamdani_v03','emulador','supervisor'):
            proc=procesos.get(nombre)
            if proc is None:continue
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill();proc.wait(timeout=3)
            print('[DETENIDO]',nombre,'codigo',proc.returncode,flush=True)
        if auditor is not None:
            # Solo despues de parar el supervisor, enviar ceros a la base.
            pub_emergencia=auditor.create_publisher(Twist,'/cmd_vel',10)
            for _ in range(12):
                pub_emergencia.publish(Twist())
                rclpy.spin_once(auditor,timeout_sec=.05)
            auditor.destroy_publisher(pub_emergencia)
            if solicitud_spawn:
                try:
                    cli=auditor.create_client(DeleteEntity,'/delete_entity')
                    if cli.wait_for_service(timeout_sec=5):
                        req=DeleteEntity.Request();req.name=NOMBRE
                        resultado=esperar_futuro(auditor,cli.call_async(req),8)
                        res['obstaculo_eliminado']=bool(resultado.success)
                        print('[LIMPIEZA] Obstaculo eliminado:',resultado.success,
                              resultado.status_message,flush=True)
                    else:
                        res['obstaculo_eliminado']=False
                        print('[AVISO] Servicio de borrado ausente; retirar caja manualmente',NOMBRE,flush=True)
                except Exception as exc:
                    res['obstaculo_eliminado']=False
                    print('[AVISO] No se pudo retirar caja:',exc,'Nombre:',NOMBRE,flush=True)
            auditor.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for handle in logs.values():handle.close()
        if traza:
            with (carpeta/'traza_estados.csv').open('w',newline='',encoding='utf-8') as h:
                writer=csv.DictWriter(h,fieldnames=list(traza[0]));writer.writeheader();writer.writerows(traza)
        if 'seguidor' in procesos:
            ruta_copia=carpeta/f'F12_ruta_2_E426_{marca}.csv'
            origen=ROOT/'resultados'/('F13_'+ruta_copia.stem.removeprefix('F12_')+'_log.csv')
            if origen.exists():shutil.copy2(origen,carpeta/'trayectoria_seguidor.csv')
        res['error']=error
        res['aprobado']=bool(res['aprobado'] and res.get('obstaculo_eliminado') is True and error is None)
        (carpeta/'resumen.json').write_text(json.dumps(res,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print('[EVIDENCIA]',carpeta,flush=True)
        if error:
            for nombre in ('seguidor','supervisor','mamdani_v03','cnn'):
                path=carpeta/f'{nombre}.log'
                if path.exists():
                    print('---',nombre,'---',flush=True)
                    print('\n'.join(path.read_text(errors='replace').splitlines()[-6:]),flush=True)
        print('=== FIN PRUEBA E4.2.6 ===',flush=True)
    return 0 if res['aprobado'] else 1

if __name__=='__main__':
    sys.exit(main())
