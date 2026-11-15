#!/usr/bin/env python3
"""Crea un nuevo ejecutor ROS 2 con alineacion de yaw como rescate CNN.

Solo edita una COPIA de tools/A3_E432_ejecutor_misiones_ros.py.
No arranca Gazebo, ROS, ni mueve el robot. Falla si los anchors difieren.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ORIG = ROOT / 'tools/A3_E432_ejecutor_misiones_ros.py'
DEST = ROOT / 'tools/A3_E441_ejecutor_misiones_alineacion.py'
E438 = ROOT / 'tools/A3_E438_alineacion_visual_gazebo.py'
OUT = ROOT / 'resultados/A3_E441_ejecutor_alineacion'
FIS = ROOT / 'matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis'
FIS_SHA = '67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903'


def reemplazar_unico(source, old, new, nombre):
    n = source.count(old)
    if n != 1:
        raise RuntimeError(f'[BLOQUEADO] {nombre}: coincidencias={n}; la fuente no coincide con el formato verificado')
    return source.replace(old, new, 1)


NUEVO_AUDITOR_Y_GIRO = '''class AuditorIntegrado(Auditor):
    """Extiende el auditor original sin reemplazar sus validaciones."""
    def __init__(self):
        self.yaw = None
        self.t_yaw = 0.0
        self.en_giro = False
        self.max_lineal_giro = 0.0
        self.max_angular_giro = 0.0
        super().__init__()
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        self.sub_yaw = self.node.create_subscription(
            Odometry, '/odom', self._odom_yaw, qos_profile_sensor_data)

    def _odom_yaw(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w*q.z + q.x*q.y),
                              1 - 2*(q.y*q.y + q.z*q.z))
        self.t_yaw = time.monotonic()

    def _cmd(self, msg):
        super()._cmd(msg)
        if self.en_giro:
            self.max_lineal_giro = max(self.max_lineal_giro, abs(float(msg.linear.x)))
            self.max_angular_giro = max(self.max_angular_giro, abs(float(msg.angular.z)))

    def postura_fresca(self):
        return (self.sensores_frescos() and self.yaw is not None
                and time.monotonic() - self.t_yaw < EDAD_SENSORES)

    def validar_panel(self, meta):
        assert self.postura_fresca(), 'Orientacion o sensores caducados'
        d = math.dist(self.pose, PUNTOS[meta])
        assert d <= RADIUS_INICIO_M, (
            f'No girar: error meta {d:.3f} m > {RADIUS_INICIO_M} m')
        assert self.frontal >= .35, 'No girar: obstaculo frontal critico'
        return d


def alinear_en_gestor(a, meta, vigilar, carpeta, numero):
    """Giro v=0 mediante /cmd_vel_ruta, nunca publica directamente /cmd_vel.

    El llamante debe haber detenido al seguidor Y a Mamdani, manteniendo
    supervisor, emulador y CNN. Si /cmd_vel_difuso persiste, BLOQUEA.
    """
    from geometry_msgs.msg import Twist
    assert meta in PANELES
    a.validar_panel(meta)
    assert a.node.count_publishers('/cmd_vel') == 1, 'Supervisor no es unico publicador'
    assert a.node.count_publishers('/cmd_vel_difuso') == 0, 'Mamdani aun activo; giro bloqueado'
    assert a.node.count_publishers('/cmd_vel_ruta') == 0, 'Hay un seguidor de ruta activo'
    assert a.node.count_publishers('/estacion_cnn_info') == 1, 'CNN duplicada o ausente'
    assert a.estado_reciente and a.estado_reciente.get('modo') == 'DETENIDO', (
        'Supervisor no esta detenido antes del giro')

    pub = None
    a.max_lineal_giro = 0.0
    a.max_angular_giro = 0.0
    yaw_inicial = math.degrees(a.yaw)
    error_inicial = math.degrees(error_yaw(a.pose, a.yaw, meta))
    muestras = []
    try:
        pub = a.node.create_publisher(Twist, '/cmd_vel_ruta', 10)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and pub.get_subscription_count() < 1:
            vigilar()
        assert pub.get_subscription_count() >= 1, 'Supervisor no escucha propuestas de giro'

        a.en_giro = True
        estable = 0
        hubo_ruta = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < TIMEOUT_GIRO:
            vigilar()
            a.validar_panel(meta)
            assert a.node.count_publishers('/cmd_vel_difuso') == 0, 'Mamdani aparecio durante giro'
            assert a.node.count_publishers('/cmd_vel_ruta') == 1, 'Propuesta de ruta duplicada'
            assert a.max_lineal_giro < .001, 'Velocidad lineal no autorizada durante giro'
            modo = (a.estado_reciente or {}).get('modo', '?')
            assert modo != 'DIFUSO', 'Supervisor pide DIFUSO: alineacion cancelada'
            err = error_yaw(a.pose, a.yaw, meta)
            w = orden_giro(err)
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = w
            pub.publish(msg)
            muestras.append((round(time.monotonic()-t0, 3),
                             round(math.degrees(a.yaw), 3),
                             round(math.degrees(err), 3),
                             round(w, 4), modo))
            if abs(err) <= TOL_YAW_RAD:
                estable += 1
                if estable >= 5:
                    break
            else:
                estable = 0
            if modo == 'RUTA':
                hubo_ruta = True
            if (time.monotonic() - t0 > 2.0 and not hubo_ruta
                    and abs(err) > TOL_YAW_RAD):
                raise RuntimeError('Supervisor no autorizo modo RUTA para giro')
        assert estable >= 5, 'No se logro orientacion estable en el tiempo limite'
    finally:
        # Anular propuesta ANTES de abandonar; en caso de error, parada garantizada
        if pub is not None:
            for _ in range(8):
                pub.publish(Twist())
                a.spin(.06)
            a.node.destroy_publisher(pub)
        a.en_giro = False

    t_stop = time.monotonic() + 3
    detenido = False
    while time.monotonic() < t_stop:
        vigilar()
        estado = a.estado_reciente or {}
        if (estado.get('modo') == 'DETENIDO'
                and abs(float(estado.get('v', 1))) <= 1e-5
                and abs(float(estado.get('w', 1))) <= 1e-5
                and time.monotonic() - a.t_estado < .4):
            detenido = True
            break
    assert detenido, 'No se confirmo parada efectiva despues del giro'
    a.validar_panel(meta)
    error_final = math.degrees(error_yaw(a.pose, a.yaw, meta))
    assert abs(math.radians(error_final)) <= TOL_YAW_RAD, 'Error angular final fuera de tolerancia'
    assert a.max_lineal_giro < .001, 'Se observo movimiento lineal en giro'

    csv_salida = carpeta / f'alineacion_{numero:02d}_{meta.lower()}.csv'
    with csv_salida.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f, lineterminator='\\n')
        writer.writerow(('t_s', 'yaw_deg', 'error_deg', 'w_propuesto_rad_s', 'modo_supervisor'))
        writer.writerows(muestras)
    print(f'[OK] Alineacion {meta}: yaw={math.degrees(a.yaw):+.2f} deg; '
          f'error panel={error_final:+.2f} deg; v=0', flush=True)
    return {
        'aplicada': True, 'yaw_inicial_deg': round(yaw_inicial, 3),
        'error_angular_inicial_deg': round(error_inicial, 3),
        'yaw_final_deg': round(math.degrees(a.yaw), 3),
        'error_angular_final_deg': round(error_final, 3),
        'max_lineal_giro_m_s': round(a.max_lineal_giro, 6),
        'max_angular_giro_rad_s': round(a.max_angular_giro, 6),
        'registro_giro': str(csv_salida.relative_to(ROOT)),
    }


'''


BLOQUE_NUEVO_CONFIRMACION = '''            # Primero esperar reconocimiento pasivo: no girar si ya funciona.
            alineacion = {'aplicada': False}
            espera_pasiva = time.monotonic() + 4.0
            while not auditor.confirmador.confirmado and time.monotonic() < espera_pasiva:
                vigilar()
            if not auditor.confirmador.confirmado:
                print(f'[CNN] {meta}: no confirmada; iniciar rescate angular', flush=True)
                auditor.confirmador = None  # Eliminar votos acumulados antes del giro
                detener('mamdani')
                t_retirada = time.monotonic() + 6
                while time.monotonic() < t_retirada:
                    vigilar()
                    if auditor.node.count_publishers('/cmd_vel_difuso') == 0:
                        break
                assert auditor.node.count_publishers('/cmd_vel_difuso') == 0, (
                    'Mamdani sigue publicando: alineacion bloqueada')
                t_parada = time.monotonic() + 3.0
                parado = False
                while time.monotonic() < t_parada:
                    vigilar()
                    estado = auditor.estado_reciente or {}
                    parado = (estado.get('modo') == 'DETENIDO'
                              and abs(float(estado.get('v', 1))) <= 1e-5
                              and abs(float(estado.get('w', 1))) <= 1e-5
                              and time.monotonic() - auditor.t_estado < .4)
                    if parado:
                        break
                assert parado, 'Supervisor no detuvo tras suspender Mamdani'
                alineacion = alinear_en_gestor(
                    auditor, meta, vigilar, carpeta, etapa['numero'])
                auditor.confirmador = ConfirmadorCNN(meta)  # 3 frames posteriores al giro
                t_cnn = time.monotonic() + 25.0
                while not auditor.confirmador.confirmado and time.monotonic() < t_cnn:
                    vigilar()
            assert auditor.confirmador.confirmado, (
                f'CNN no confirmo {meta} con 3 frames validos a menos de {RADIO_CNN} m')
            print(f'[OK] CNN {meta}: tres frames recientes; odometria y parada OK', flush=True)
            resultado['etapas'].append({
                'numero': etapa['numero'], 'origen': etapa['origen'], 'destino': meta,
                'codigo_seguidor': codigo, 'error_final_m': round(err_llegada,4),
                'confirmacion_cnn': True, 'alineacion': alineacion,
                'duracion_s': round(time.monotonic()-tiempo_inicio,2),
            })
            auditor.confirmador = None
            # Para el siguiente trayecto, recuperar Mamdani antes de crear seguidor.
            if ('mamdani' not in procesos
                    and etapa['numero'] != plan['etapas'][-1]['numero']):
                anterior = auditor.num_difuso
                iniciar('mamdani', VENV, 'tools/A3_nodo_mamdani_v03.py',
                        ('--ros', '--ros-args', '-p', 'use_sim_time:=true'))
                listo_mamdani = False
                limite_reinicio = time.monotonic() + 20
                contador_cmd_prev = auditor.count_movimiento
                while time.monotonic() < limite_reinicio:
                    vigilar()
                    assert auditor.count_movimiento == contador_cmd_prev, (
                        'Se detecto comando de movimiento durante reinicio de Mamdani')
                    listo_mamdani = (
                        auditor.node.count_publishers('/cmd_vel_difuso') == 1
                        and auditor.num_difuso >= anterior + 2)
                    if listo_mamdani:
                        break
                assert listo_mamdani, 'Mamdani no reanudo mensajes antes del proximo tramo'
                print('[OK] Mamdani reanudado antes de siguiente mision', flush=True)
'''


def generar(src: str) -> str:
    assert 'class Auditor' in src and 'def ejecutar(desde, orden):' in src
    texto = src
    texto = reemplazar_unico(
        texto, "OUT = ROOT / 'resultados/A3_E432_ejecutor_misiones_ros'",
        "OUT = ROOT / 'resultados/A3_E441_ejecutor_alineacion'", 'Directorio independiente')
    texto = reemplazar_unico(
        texto, '\n\nclass Auditor:\n',
        '\n\nfrom A3_E438_alineacion_visual_gazebo import (\n'
        '    PANELES, RADIUS_INICIO_M, TOL_YAW_RAD, TIMEOUT_GIRO,\n'
        '    error_yaw, orden_giro, autoprueba as autoprueba_alineacion,\n'
        ')\n\n\nclass Auditor:\n', 'Importacion alineador')
    texto = reemplazar_unico(texto, '\n\ndef comprobar_dominio():\n',
                             '\n\n' + NUEVO_AUDITOR_Y_GIRO + 'def comprobar_dominio():\n',
                             'Ampliacion auditor y giro')
    texto = reemplazar_unico(
        texto,
        "        log = (carpeta / f'{nombre}.log').open('w', encoding='utf-8')\n        logs[nombre] = log",
        "        candidato = nombre\n"
        "        i_log = 0\n"
        "        while (carpeta / f'{candidato}.log').exists():\n"
        "            i_log += 1\n"
        "            candidato = f'{nombre}_reinicio_{i_log:02d}'\n"
        "        log = (carpeta / f'{candidato}.log').open('w', encoding='utf-8')\n"
        "        logs[candidato] = log", 'Logs independientes de reinicios')
    cabeza, separador, resto = texto.partition('def ejecutar(desde, orden):')
    assert separador, 'No aparece la funcion de ejecucion'
    resto = reemplazar_unico(
        resto,
        '    try:\n        auditor = Auditor()\n        auditor.espera_gazebo()\n',
        '    try:\n        auditor = AuditorIntegrado()\n        auditor.espera_gazebo()\n', 'Auditor de ejecutar')
    texto = cabeza + separador + resto
    viejo = '''            # Un destino requiere 3 frames CNN concordantes y proximidad real.
            t_cnn = time.monotonic() + TIMEOUT_CNN_S
            while not auditor.confirmador.confirmado and time.monotonic() < t_cnn:
                vigilar()
            assert auditor.confirmador.confirmado, (
                f'CNN no confirmo {meta} con 3 frames validos a menos de {RADIO_CNN} m')
            print(f'[OK] CNN {meta}: tres frames recientes; odometria y parada OK')
            resultado['etapas'].append({
                'numero': etapa['numero'], 'origen': etapa['origen'], 'destino': meta,
                'codigo_seguidor': codigo, 'error_final_m': round(err_llegada,4),
                'confirmacion_cnn': True, 'duracion_s': round(time.monotonic()-tiempo_inicio,2),
            })
            auditor.confirmador = None
'''
    texto = reemplazar_unico(texto, viejo, BLOQUE_NUEVO_CONFIRMACION,
                             'Confirmacion y rescate por alineacion')
    texto = reemplazar_unico(
        texto,
        '    if args.self_test:\n        autopueba()\n        return 0\n',
        '    if args.self_test:\n        autopueba()\n        autoprueba_alineacion()\n        print("[OK] Ejecutor E441: 19 + 14 pruebas de seguridad")\n        return 0\n', 'Autopruebas 19 + 14')
    texto = texto.replace('Etapa 4.3.2: ejecutar misiones A* consecutivas',
                          'Etapa 4.3.41: misiones A* con rescate de alineacion angular', 1)
    # La version generada no se puede ejecutar en robot fisico.
    ast.parse(texto)
    return texto


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--preparar', action='store_true', required=True)
    args = p.parse_args()
    if not (ORIG.is_file() and E438.is_file()):
        raise RuntimeError('Faltan ejecutor original o alineador probado')
    if hashlib.sha256(FIS.read_bytes()).hexdigest() != FIS_SHA:
        raise RuntimeError('FIS original modificado; abortar')
    original = ORIG.read_bytes()
    assert b'\x00' not in original, 'El ejecutor original no es texto valido'
    contenido = generar(original.decode('utf-8'))
    if DEST.exists():
        if DEST.read_text(encoding='utf-8') != contenido:
            raise RuntimeError('Destino ya existe y tiene diferente contenido; no se sobrescribe')
        print('[OK] E441 ya existe y coincide; no se sobrescribio')
    else:
        DEST.write_text(contenido, encoding='utf-8')
        print('[CREADO]', DEST.relative_to(ROOT))
    OUT.mkdir(parents=True, exist_ok=True)
    manifiesto = {
        'origen': str(ORIG.relative_to(ROOT)),
        'origen_sha256': hashlib.sha256(original).hexdigest(),
        'alineador': str(E438.relative_to(ROOT)),
        'alineador_sha256': hashlib.sha256(E438.read_bytes()).hexdigest(),
        'nuevo': str(DEST.relative_to(ROOT)),
        'nuevo_sha256': hashlib.sha256(DEST.read_bytes()).hexdigest(),
        'fis_sha256': FIS_SHA,
        'nota': 'Solo preparacion de nueva herramienta. Ningun proceso ROS iniciado.'
    }
    manifest = OUT / 'preparacion_E441.json'
    if not manifest.exists():
        manifest.write_text(json.dumps(manifiesto, ensure_ascii=False, indent=2) + '\n')
    print('[OK] Fuente original E432 sin modificar')
    print('[OK] Alineador original E438 sin modificar')
    print('[OK] FIS v03 intacto')
    print('[OK] Autorizacion de movimiento requerida para ejecutar E441')
    print('[EVIDENCIA]', manifest.relative_to(ROOT))
    print('[SEGURIDAD] No se inicio ROS, Gazebo ni movimiento')


if __name__ == '__main__':
    main()
