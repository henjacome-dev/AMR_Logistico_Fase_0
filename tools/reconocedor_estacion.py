#!/usr/bin/env python3
"""U3S7: Nodo clasificador de estaciones logisticas mediante MobileNetV2."""
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
import tensorflow as tf

REPO = Path.home() / 'AMR_Logistico_Fase_0'
DEFAULT_MODEL = REPO / 'modelos' / 'modelo_estaciones_mobilenetv2.keras'
DEFAULT_CLASSES = REPO / 'modelos' / 'clases_estaciones.json'
CLASES_FALLBACK = ['ALMACENAMIENTO', 'DESPACHO', 'EMPAQUE', 'RECEPCION']


class ReconocedorEstacion(Node):

    def __init__(self):
        super().__init__(
            'reconocedor_estacion',
            automatically_declare_parameters_from_overrides=True,
        )
        model_path = Path(
            self.get_parameter_or('model_path', str(DEFAULT_MODEL)).value
        )
        classes_path = Path(
            self.get_parameter_or('classes_path', str(DEFAULT_CLASSES)).value
        )
        self.umbral = float(self.get_parameter_or('umbral_confianza', 0.80).value)
        self.periodo_inferencia = float(
            self.get_parameter_or('periodo_s', 0.25).value
        )
        self.image_topic = str(
            self.get_parameter_or('image_topic', '/camera/image_raw').value
        )

        if not model_path.is_file():
            raise FileNotFoundError(
                f'No se encuentra el modelo en: {model_path}'
            )

        if classes_path.is_file():
            self.clases = json.loads(classes_path.read_text(encoding='utf-8'))
        else:
            self.clases = CLASES_FALLBACK

        self.get_logger().info(f'Cargando modelo CNN desde {model_path}...')
        self.model = tf.keras.models.load_model(str(model_path), compile=False)

        # Calentamiento (warm-up) para evitar latencia en el primer fotograma real
        dummy = np.zeros((1, 224, 224, 3), dtype=np.float32)
        self.model(dummy, training=False)

        self.pub_estacion = self.create_publisher(
            String, '/estacion_detectada', 10
        )
        self.pub_info = self.create_publisher(
            String, '/estacion_cnn_info', 10
        )

        self.sub_img = None
        self.ultima_inferencia = 0.0
        self.ultimo_log = 0.0
        self._conectar_camara()
        self.timer_discovery = self.create_timer(2.0, self._verificar_camara)

    def _conectar_camara(self):
        if self.count_publishers(self.image_topic) == 0:
            for nombre, tipos in self.get_topic_names_and_types():
                if 'sensor_msgs/msg/Image' in tipos and 'depth' not in nombre.lower():
                    self.image_topic = nombre
                    break

        self.sub_img = self.create_subscription(
            Image,
            self.image_topic,
            self.cb_imagen,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Reconocedor CNN listo. Escuchando camara en: {self.image_topic} | Clases: {self.clases}'
        )

    def _verificar_camara(self):
        if self.count_publishers(self.image_topic) == 0:
            for nombre, tipos in self.get_topic_names_and_types():
                if (
                    'sensor_msgs/msg/Image' in tipos
                    and 'depth' not in nombre.lower()
                    and nombre != self.image_topic
                ):
                    self.destroy_subscription(self.sub_img)
                    self.image_topic = nombre
                    self.sub_img = self.create_subscription(
                        Image,
                        self.image_topic,
                        self.cb_imagen,
                        qos_profile_sensor_data,
                    )
                    self.get_logger().info(
                        f'Camara reasignada automaticamente a: {self.image_topic}'
                    )
                    break

    def _ros_image_a_rgb(self, msg: Image) -> np.ndarray | None:
        enc = msg.encoding.lower()
        h, w = msg.height, msg.width
        arr = np.frombuffer(msg.data, dtype=np.uint8)

        if enc == 'rgb8':
            return arr.reshape((h, w, 3))
        if enc == 'bgr8':
            return arr.reshape((h, w, 3))[:, :, ::-1]
        if enc == 'rgba8':
            return arr.reshape((h, w, 4))[:, :, :3]
        if enc == 'bgra8':
            return arr.reshape((h, w, 4))[:, :, 2::-1]
        return None

    def cb_imagen(self, msg: Image):
        ahora = time.monotonic()
        if ahora - self.ultima_inferencia < self.periodo_inferencia:
            return
        self.ultima_inferencia = ahora

        rgb = self._ros_image_a_rgb(msg)
        if rgb is None:
            self.get_logger().warn(
                f'Codificacion de imagen no soportada: {msg.encoding}',
                throttle_duration_sec=5.0,
            )
            return

        t0 = time.perf_counter()
        img_224 = tf.image.resize(rgb, (224, 224)).numpy()
        batch = np.expand_dims(img_224, axis=0).astype(np.float32)
        probs = self.model(batch, training=False).numpy()[0]
        latencia_ms = (time.perf_counter() - t0) * 1000.0

        idx = int(np.argmax(probs))
        confianza = float(probs[idx])
        etiqueta = self.clases[idx] if confianza >= self.umbral else 'INCIERTO'

        msg_str = String()
        msg_str.data = etiqueta
        self.pub_estacion.publish(msg_str)

        detalle = {
            'estacion': etiqueta,
            'clase_top1': self.clases[idx],
            'confianza': round(confianza, 4),
            'latencia_ms': round(latencia_ms, 2),
            'probs': {
                self.clases[i]: round(float(probs[i]), 4)
                for i in range(len(self.clases))
            },
        }
        msg_info = String()
        msg_info.data = json.dumps(detalle, separators=(',', ':'))
        self.pub_info.publish(msg_info)

        if ahora - self.ultimo_log >= 2.0:
            self.get_logger().info(
                f'CNN -> {etiqueta} ({confianza*100:.1f}%, {latencia_ms:.1f} ms)'
            )
            self.ultimo_log = ahora


def main():
    rclpy.init()
    nodo = ReconocedorEstacion()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
