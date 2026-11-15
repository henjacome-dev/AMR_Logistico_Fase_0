#!/usr/bin/env python3
"""U3S7: Supervisor integrado con arbitraje seguro y reconocimiento CNN.

Mantiene todas las garantias de supervisor_f13.py:
- Unico publicador en /cmd_vel.
- Parada estricta si LiDAR < 0.35 m o sensores caducados.
- Limites VMAX = 0.12 m/s, WMAX = 0.70 rad/s.
- Integra /estacion_detectada y /estacion_cnn_info.

Uso:
  python3 tools/supervisor_u3s7_cnn.py --ros-args -p use_sim_time:=true
  python3 tools/supervisor_u3s7_cnn.py --ros-args -p use_sim_time:=true -p estacion_objetivo:=DESPACHO
"""
from collections import deque
import json
import math
import signal
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String

VMAX_RUTA = 0.12
VMAX_DIFUSO = 0.10
WMAX = 0.70
LIMITE_OBSTACULO = 0.35
FRONTAL_DIFUSO = 0.60
LATERAL_DIFUSO = 0.35
EDAD_SENSORES = 1.5
EDAD_COMANDO = 0.4
EDAD_CNN = 2.0
ANGULO_FRONTAL = math.radians(25)


class SupervisorU3S7(Node):

    def __init__(self):
        super().__init__(
            'supervisor_u3s7_cnn',
            automatically_declare_parameters_from_overrides=True,
        )
        self.estacion_objetivo = str(
            self.get_parameter_or('estacion_objetivo', '').value
        ).strip().upper()
        self.votos_req = int(
            self.get_parameter_or('confirmaciones_requeridas', 3).value
        )

        # Publicadores
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_estado = self.create_publisher(
            String, '/estado_supervisor', 10
        )

        # Suscripciones base (F1.3)
        self.sub_prop = self.create_subscription(
            Twist, '/cmd_vel_ruta', self.cb_prop_ruta, 10
        )
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.cb_odom, qos_profile_sensor_data
        )

        # Suscripciones opcionales Avance 2 (Difuso + Rangos)
        self.sub_difuso = self.create_subscription(
            Twist, '/cmd_vel_difuso', self.cb_prop_difuso, 10
        )
        self.rangos = {'df': None, 'di': None, 'dd': None}
        self.t_rangos = {'df': None, 'di': None, 'dd': None}
        for k, topico in [
            ('df', '/rango_frontal'),
            ('di', '/rango_izq'),
            ('dd', '/rango_der'),
        ]:
            self.create_subscription(
                Range,
                topico,
                lambda msg, key=k: self.cb_rango(key, msg),
                qos_profile_sensor_data,
            )

        # Suscripciones U3S7 (CNN)
        self.sub_cnn = self.create_subscription(
            String, '/estacion_detectada', self.cb_estacion, 10
        )
        self.sub_cnn_info = self.create_subscription(
            String, '/estacion_cnn_info', self.cb_estacion_info, 10
        )

        # Estado interno
        self.t_prop_ruta = self.t_prop_difuso = None
        self.prop_ruta = self.prop_difuso = None
        self.t_scan = self.t_odom = None
        self.dist_frente = None

        self.historial_cnn = deque(maxlen=self.votos_req)
        self.t_cnn = None
        self.estacion_confirmada = 'NINGUNA'
        self.ultima_info_cnn = {}
        self.objetivo_cumplido = False

        self.warn_t = 0.0
        self.ultimo_estado_log = ''
        self.timer = self.create_timer(0.1, self.control)
        self.get_logger().info(
            f'Supervisor U3S7 activo | Objetivo: {self.estacion_objetivo or "MONITOREO_CONTINUO"}'
        )

    def cb_prop_ruta(self, msg: Twist):
        self.prop_ruta = msg
        self.t_prop_ruta = time.monotonic()

    def cb_prop_difuso(self, msg: Twist):
        self.prop_difuso = msg
        self.t_prop_difuso = time.monotonic()

    def cb_rango(self, key: str, msg: Range):
        v = float(msg.range)
        if math.isfinite(v) and msg.min_range <= v <= msg.max_range:
            self.rangos[key] = v
            self.t_rangos[key] = time.monotonic()

    def cb_odom(self, msg: Odometry):
        pose = msg.pose.pose.position
        if all(math.isfinite(v) for v in [pose.x, pose.y, pose.z]):
            self.t_odom = time.monotonic()

    def cb_scan(self, msg: LaserScan):
        valores = []
        for i, r in enumerate(msg.ranges):
            angulo = math.atan2(
                math.sin(msg.angle_min + i * msg.angle_increment),
                math.cos(msg.angle_min + i * msg.angle_increment),
            )
            if abs(angulo) > ANGULO_FRONTAL:
                continue
            if math.isinf(r) and r > 0 and math.isfinite(msg.range_max):
                r = msg.range_max
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                valores.append(r)
        self.dist_frente = min(valores) if valores else None
        self.t_scan = time.monotonic()

    def cb_estacion(self, msg: String):
        etiqueta = msg.data.strip().upper()
        self.t_cnn = time.monotonic()
        self.historial_cnn.append(etiqueta)

        if (
            len(self.historial_cnn) == self.votos_req
            and len(set(self.historial_cnn)) == 1
            and etiqueta not in ('', 'INCIERTO')
        ):
            if self.estacion_confirmada != etiqueta:
                self.estacion_confirmada = etiqueta
                conf = self.ultima_info_cnn.get('confianza', 0.0)
                self.get_logger().info(
                    f'[CNN CONFIRMADA] Estacion: {etiqueta} (confianza={conf*100:.1f}%)'
                )
            if (
                self.estacion_objetivo
                and etiqueta == self.estacion_objetivo
                and not self.objetivo_cumplido
            ):
                self.objetivo_cumplido = True
                self.get_logger().info(
                    f'[MISION COMPLETADA] Robot detenido en estacion objetivo: {etiqueta}'
                )

    def cb_estacion_info(self, msg: String):
        try:
            self.ultima_info_cnn = json.loads(msg.data)
        except Exception:
            pass

    def _twist_valido(self, p: Twist) -> bool:
        valores = [
            p.linear.x,
            p.linear.y,
            p.linear.z,
            p.angular.x,
            p.angular.y,
            p.angular.z,
        ]
        if not all(math.isfinite(v) for v in valores):
            return False
        if p.linear.x < 0 or any(
            abs(v) > 1e-6
            for v in [p.linear.y, p.linear.z, p.angular.x, p.angular.y]
        ):
            return False
        return True

    def _publicar_estado(self, modo: str, motivo: str, v: float, w: float):
        payload = {
            'modo': modo,
            'motivo': motivo,
            'v': round(v, 4),
            'w': round(w, 4),
            'estacion_confirmada': self.estacion_confirmada,
            'objetivo': self.estacion_objetivo or 'NINGUNO',
            'objetivo_cumplido': self.objetivo_cumplido,
        }
        self.pub_estado.publish(String(data=json.dumps(payload, separators=(',', ':'))))

    def control(self):
        ahora = time.monotonic()

        if self.objetivo_cumplido:
            self.pub.publish(Twist())
            self._publicar_estado('DETENIDO', 'ESTACION_OBJETIVO_ALCANZADA', 0.0, 0.0)
            return

        if self.t_scan is None or self.t_odom is None:
            self._detener(ahora, 'esperando sensores')
            return
        if (
            ahora - self.t_scan > EDAD_SENSORES
            or ahora - self.t_odom > EDAD_SENSORES
        ):
            self._detener(ahora, 'datos de sensores caducados')
            return
        if self.dist_frente is None or self.dist_frente < LIMITE_OBSTACULO:
            self._detener(ahora, 'LiDAR invalido u obstaculo frontal (<0.35m)')
            return

        # Verificar si el controlador difuso y los rangos estan activos
        rangos_frescos = all(
            self.t_rangos[k] is not None
            and (ahora - self.t_rangos[k] <= EDAD_SENSORES)
            for k in ('df', 'di', 'dd')
        )
        usar_difuso = False
        if rangos_frescos:
            if (
                self.rangos['df'] <= FRONTAL_DIFUSO
                or self.rangos['di'] <= LATERAL_DIFUSO
                or self.rangos['dd'] <= LATERAL_DIFUSO
            ):
                usar_difuso = True

        if usar_difuso:
            if (
                self.t_prop_difuso is None
                or ahora - self.t_prop_difuso > EDAD_COMANDO
            ):
                self._detener(ahora, 'sin propuesta reciente de control difuso')
                return
            if not self._twist_valido(self.prop_difuso):
                self._detener(ahora, 'Twist difuso invalido')
                return
            cmd = Twist()
            cmd.linear.x = min(VMAX_DIFUSO, self.prop_difuso.linear.x)
            cmd.angular.z = max(-WMAX, min(WMAX, self.prop_difuso.angular.z))
            self.pub.publish(cmd)
            self._publicar_estado('DIFUSO', 'EVASION_ACTIVA', cmd.linear.x, cmd.angular.z)
            return

        # Modo RUTA (igual que supervisor_f13.py)
        if self.t_prop_ruta is None or ahora - self.t_prop_ruta > EDAD_COMANDO:
            self._detener(ahora, 'no hay propuesta reciente de ruta')
            return
        if not self._twist_valido(self.prop_ruta):
            self._detener(ahora, 'Twist de ruta invalido')
            return

        cmd = Twist()
        cmd.linear.x = min(VMAX_RUTA, self.prop_ruta.linear.x)
        cmd.angular.z = max(-WMAX, min(WMAX, self.prop_ruta.angular.z))
        self.pub.publish(cmd)
        self._publicar_estado('RUTA', 'NAVEGACION_NOMINAL', cmd.linear.x, cmd.angular.z)

    def _detener(self, ahora: float, motivo: str):
        self.pub.publish(Twist())
        self._publicar_estado('DETENIDO', motivo, 0.0, 0.0)
        if ahora - self.warn_t > 5.0:
            self.get_logger().warn('Parada segura: ' + motivo)
            self.warn_t = ahora

    def parada(self):
        self.pub.publish(Twist())


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    nodo = SupervisorU3S7()
    try:
        param = nodo.get_parameter_or('use_sim_time', None)
        if param is None or not bool(param.value):
            raise RuntimeError(
                'Supervisor: se requiere --ros-args -p use_sim_time:=true'
            )
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(10):
            nodo.parada()
            time.sleep(0.05)
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
