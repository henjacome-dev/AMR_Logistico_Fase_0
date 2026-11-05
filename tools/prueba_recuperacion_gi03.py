#!/usr/bin/env python3
"""F1.3.3 / GI-03: recuperación controlada en Gazebo, NO para robot físico.

El ensayo no modifica supervisor_f13.py ni publica directamente a /cmd_vel.
Verifica: (1) inmovilidad sin propuestas, (2) reanudación con una propuesta
nueva cuando el LiDAR está despejado y (3) parada al cesar propuestas.

IMPORTANTE: Este ensayo NO verifica enclavamiento ante fallas. El supervisor
actual puede reanudar al recibir propuestas válidas si desaparece el bloqueo.

Uso (desde la raíz del proyecto, con ROS 2 activado):
  python3 tools/prueba_recuperacion_gi03.py --preflight
  python3 tools/prueba_recuperacion_gi03.py --ejecutar
"""

import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


VELOCIDAD = 0.04              # m/s; orden deliberadamente pequeña
DURACION_AVANCE = 2.0        # s de reloj real
DISTANCIA_MINIMA = 0.80      # m de despeje inicial y durante avance
MAX_DESPLAZAMIENTO = 0.15    # m; abortar ante avance excesivo
UMBRAL_CERO = 0.005          # m/s y rad/s para observación de parada


class Ensayo(Node):
    def __init__(self):
        super().__init__('gi03_observador')
        self.ultima_scan = None
        self.ultima_odom = None
        self.ultima_vel = None
        self.ultimo_clock = None
        self.primera_hora_clock = None
        self.hora_clock_previa = None
        self.clock_avanza = False
        self.t_scan = self.t_odom = self.t_vel = 0.0
        self.historial = []
        self.acciones = []
        self.crear_publicador = None
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.cb_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.cb_vel, 10)
        self.create_subscription(Clock, '/clock', self.cb_clock, qos_profile_sensor_data)

    def cb_scan(self, msg):
        self.ultima_scan = msg
        self.t_scan = time.monotonic()

    def cb_odom(self, msg):
        self.ultima_odom = msg
        self.t_odom = time.monotonic()

    def cb_vel(self, msg):
        self.ultima_vel = msg
        self.t_vel = time.monotonic()

    def cb_clock(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.primera_hora_clock is None:
            self.primera_hora_clock = t
        if self.hora_clock_previa is not None and t > self.hora_clock_previa:
            self.clock_avanza = True
        self.hora_clock_previa = t
        self.ultimo_clock = t

    def frontal(self):
        """Rango mínimo válido dentro de ±25 grados (en coordenadas LiDAR)."""
        if self.ultima_scan is None:
            return None
        s = self.ultima_scan
        distancias = []
        for i, rango in enumerate(s.ranges):
            angulo = s.angle_min + i * s.angle_increment
            angulo = math.atan2(math.sin(angulo), math.cos(angulo))
            if (abs(angulo) <= math.radians(25)
                    and math.isfinite(rango)
                    and s.range_min <= rango <= s.range_max):
                distancias.append(float(rango))
        return min(distancias) if distancias else None

    def xy(self):
        if self.ultima_odom is None:
            return None
        p = self.ultima_odom.pose.pose.position
        return (float(p.x), float(p.y))

    def velocidad(self):
        if self.ultima_vel is None:
            return None
        return (float(self.ultima_vel.linear.x), float(self.ultima_vel.angular.z))

    def guardar_muestra(self, etapa):
        v = self.velocidad()
        pos = self.xy()
        self.historial.append({
            't_real': round(time.monotonic(), 4),
            'etapa': etapa,
            'cmd_v_m_s': v[0] if v else '',
            'cmd_w_rad_s': v[1] if v else '',
            'x_m': pos[0] if pos else '',
            'y_m': pos[1] if pos else '',
            'frontal_m': self.frontal() if self.frontal() is not None else '',
        })

    def girar(self, duracion, etapa, publicar=None, referencia=None):
        comienzo = time.monotonic()
        proxima_publicacion = 0.0
        proxima_muestra = 0.0
        while time.monotonic() - comienzo < duracion:
            ahora = time.monotonic() - comienzo
            rclpy.spin_once(self, timeout_sec=0.02)
            if publicar is not None and ahora >= proxima_publicacion:
                self.crear_publicador.publish(publicar)
                proxima_publicacion = ahora + 0.05
            if ahora >= proxima_muestra:
                self.guardar_muestra(etapa)
                proxima_muestra = ahora + 0.05
            if publicar is not None and publicar.linear.x > 0:
                d = self.frontal()
                xy = self.xy()
                if d is None or time.monotonic() - self.t_scan > 1.0:
                    raise RuntimeError('LiDAR sin distancia frontal válida durante el avance')
                if d < DISTANCIA_MINIMA:
                    raise RuntimeError(f'Proximidad frontal {d:.3f} m: avance cancelado')
                if xy is None or time.monotonic() - self.t_odom > 1.0:
                    raise RuntimeError('Odometría perdida durante el avance')
                if referencia is not None and math.dist(xy, referencia) > MAX_DESPLAZAMIENTO:
                    raise RuntimeError('Desplazamiento superior al límite interno de GI-03')
                pubs = self.get_publishers_info_by_topic('/cmd_vel')
                if len(pubs) != 1 or pubs[0].node_name != 'supervisor_f13':
                    raise RuntimeError('Cambio inesperado de autoridad sobre /cmd_vel')

    def preflight(self):
        self.girar(3.0, 'preflight')
        nombres = set(self.get_node_names())
        publicaciones = self.get_publishers_info_by_topic('/cmd_vel')
        otros = self.get_publishers_info_by_topic('/cmd_vel_ruta')
        edad_scan = time.monotonic() - self.t_scan
        edad_odom = time.monotonic() - self.t_odom
        d = self.frontal()
        comprobaciones = {
            'Gazebo Classic activo': 'gazebo' in nombres,
            'Supervisor activo': 'supervisor_f13' in nombres,
            'Unico publicador de /cmd_vel es supervisor_f13':
                len(publicaciones) == 1 and publicaciones[0].node_name == 'supervisor_f13',
            'Sin publicadores externos de /cmd_vel_ruta': len(otros) == 0,
            'Reloj /clock avanzando': self.clock_avanza,
            'LiDAR reciente': edad_scan <= 1.0,
            'Odometria reciente': edad_odom <= 1.0,
            f'Despeje frontal >= {DISTANCIA_MINIMA:.2f} m':
                d is not None and d >= DISTANCIA_MINIMA,
        }
        for nombre, estado in comprobaciones.items():
            print(f"[{'OK' if estado else 'FALLO'}] {nombre}")
        print(f'[INFO] Distancia frontal: {d if d is not None else "sin lectura"} m')
        return all(comprobaciones.values())


def ultimas_cero(muestras, etapa, n=5):
    candidatos = [x for x in muestras if x['etapa'] == etapa]
    ultimas = candidatos[-n:]
    return (len(ultimas) >= n
            and all(isinstance(x['cmd_v_m_s'], (float, int))
                    and abs(x['cmd_v_m_s']) <= UMBRAL_CERO
                    and abs(x['cmd_w_rad_s']) <= UMBRAL_CERO for x in ultimas))


def distancia(a, b):
    return math.dist(a, b)


def main():
    parser = argparse.ArgumentParser(description='GI-03 recuperacion en Gazebo (solo simulacion)')
    grupo = parser.add_mutually_exclusive_group(required=True)
    grupo.add_argument('--preflight', action='store_true', help='Solo comprobar, sin publicar movimiento')
    grupo.add_argument('--ejecutar', action='store_true', help='Autorizar avance de 0.04 m/s por 2 s')
    args = parser.parse_args()
    rclpy.init()
    nodo = Ensayo()
    salida = None
    resultados = {}
    estado = 1
    try:
        if not nodo.preflight():
            print('[FALLO] Preflight: no se ha publicado ningún comando de movimiento.')
            return 2
        if args.preflight:
            print('[OK] Preflight: SIN movimiento. Listo para --ejecutar.')
            return 0

        raiz = Path.cwd() / 'resultados'
        salida = raiz / ('F133_GI03_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
        salida.mkdir(parents=True, exist_ok=False)
        print(f'[EVIDENCIA] {salida}')

        # Este es el ÚNICO tema en el que GI-03 publica. Nunca escribe /cmd_vel.
        nodo.crear_publicador = nodo.create_publisher(Twist, '/cmd_vel_ruta', 10)
        nodo.girar(1.5, 'GI03_A_sin_propuesta')
        parada_inicial = ultimas_cero(nodo.historial, 'GI03_A_sin_propuesta')
        print(f"[{'OK' if parada_inicial else 'FALLO'}] A: inmóvil sin propuestas")
        resultados['A_parada_sin_propuestas'] = parada_inicial
        if not parada_inicial:
            raise RuntimeError('El supervisor no presenta parada inicial estable')

        pos_inicio = nodo.xy()
        mensaje = Twist()
        mensaje.linear.x = VELOCIDAD
        print(f'[INFO] B: nueva propuesta válida {VELOCIDAD:.2f} m/s durante {DURACION_AVANCE:.1f} s')
        nodo.girar(DURACION_AVANCE, 'GI03_B_reanudacion', publicar=mensaje, referencia=pos_inicio)
        pos_avance = nodo.xy()
        recorrido = distancia(pos_inicio, pos_avance)
        vistos = [x['cmd_v_m_s'] for x in nodo.historial
                  if x['etapa'] == 'GI03_B_reanudacion'
                  and isinstance(x['cmd_v_m_s'], (int, float))]
        hubo_orden = any(v >= 0.025 for v in vistos)
        hubo_movimiento = recorrido >= 0.015
        resultados.update({
            'B_orden_positiva': hubo_orden,
            'B_desplazamiento_odometrico': hubo_movimiento,
            'desplazamiento_m': round(recorrido, 4),
        })
        print(f"[{'OK' if hubo_orden and hubo_movimiento else 'FALLO'}] B: orden positiva {hubo_orden}, "
              f'desplazamiento={recorrido:.4f} m')

        # Se deja de publicar propuesta: el watchdog de 0.4 s debe parar.
        nodo.girar(2.0, 'GI03_C_sin_propuesta')
        parada_final = ultimas_cero(nodo.historial, 'GI03_C_sin_propuesta')
        pos_fin = nodo.xy()
        resultado_c = parada_final and distancia(pos_avance, pos_fin) <= 0.05
        resultados.update({
            'C_parada_tras_retirar_propuesta': resultado_c,
            'C_avance_residual_m': round(distancia(pos_avance, pos_fin), 4),
        })
        print(f"[{'OK' if resultado_c else 'FALLO'}] C: parada tras retirar propuesta; "
              f'avance residual={distancia(pos_avance, pos_fin):.4f} m')
        estado = 0 if all([parada_inicial, hubo_orden, hubo_movimiento, resultado_c]) else 1
        return estado
    except Exception as exc:
        print(f'[FALLO] GI-03 abortada: {exc}', file=sys.stderr)
        resultados['error'] = str(exc)
        return 1
    finally:
        if nodo.crear_publicador is not None:
            # Comando cero como propuesta; el supervisor mantiene la única
            # autoridad sobre /cmd_vel. No garantiza freno físico independiente.
            cero = Twist()
            for _ in range(20):
                nodo.crear_publicador.publish(cero)
                rclpy.spin_once(nodo, timeout_sec=0.03)
                time.sleep(0.02)
        if salida is not None:
            try:
                with (salida / 'muestras.csv').open('w', newline='', encoding='utf-8') as f:
                    campos = ['t_real', 'etapa', 'cmd_v_m_s', 'cmd_w_rad_s', 'x_m', 'y_m', 'frontal_m']
                    escritor = csv.DictWriter(f, fieldnames=campos)
                    escritor.writeheader()
                    escritor.writerows(nodo.historial)
                (salida / 'resumen.json').write_text(
                    json.dumps(resultados, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
                print('[EVIDENCIA] muestras.csv y resumen.json guardados')
            except Exception as exc:
                print(f'[ADVERTENCIA] No se pudo guardar evidencia: {exc}', file=sys.stderr)
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
