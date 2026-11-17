#!/usr/bin/env python3
"""U3S7: Nodo clasificador de estaciones logisticas mediante MobileNetV2."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

REPO = Path(__file__).resolve().parent.parent
MODELO_FINAL = REPO / 'modelos' / 'modelo_u3s7.keras'
DEFAULT_MODEL = (MODELO_FINAL if MODELO_FINAL.is_file() else
                 REPO / 'modelos' / 'modelo_estaciones_mobilenetv2_compat.keras')
DEFAULT_WORKER_PYTHON = Path.home() / '.venvs' / 'amr-cnn' / 'bin' / 'python'
DEFAULT_CLASSES = REPO / 'modelos' / 'clases_estaciones.json'
CLASES_FALLBACK = ['ALMACENAMIENTO', 'DESPACHO', 'EMPAQUE', 'RECEPCION']

WORKER_CODE = """
import sys, time
import numpy as np
import tensorflow as tf

model = tf.keras.models.load_model(sys.argv[1], compile=False)
model(np.zeros((1, 224, 224, 3), dtype=np.float32), training=False)
sys.stdout.buffer.write(b'OK\\n')
sys.stdout.buffer.flush()

frame_bytes = 224 * 224 * 3
stdin = sys.stdin.buffer
stdout = sys.stdout.buffer

while True:
    buf = bytearray()
    while len(buf) < frame_bytes:
        chunk = stdin.read(frame_bytes - len(buf))
        if not chunk:
            sys.exit(0)
        buf.extend(chunk)
    arr = np.frombuffer(buf, dtype=np.uint8).reshape((1, 224, 224, 3)).astype(np.float32)
    t0 = time.perf_counter()
    probs = model(arr, training=False).numpy()[0].astype(np.float32)
    dt_ms = np.array([(time.perf_counter() - t0) * 1000.0], dtype=np.float32)
    stdout.write(probs.tobytes() + dt_ms.tobytes())
    stdout.flush()
"""


class ReconocedorEstacion(Node):

    def __init__(self):
        super().__init__(
            'reconocedor_estacion',
            automatically_declare_parameters_from_overrides=True,
        )
        model_path = Path(self._param('model_path', str(DEFAULT_MODEL)))
        classes_path = Path(self._param('classes_path', str(DEFAULT_CLASSES)))
        self.umbral = float(self._param('umbral_confianza', 0.80))
        self.periodo_inferencia = float(self._param('periodo_s', 0.25))
        self.image_topic = str(self._param('image_topic', '/camera/image_raw'))

        if not model_path.is_file():
            raise FileNotFoundError(f'No se encuentra el modelo en: {model_path}')

        if classes_path.is_file():
            self.clases = json.loads(classes_path.read_text(encoding='utf-8'))
        else:
            self.clases = CLASES_FALLBACK

        self.get_logger().info(f'Iniciando motor CNN con {model_path}...')
        worker_python = Path(self._param('worker_python', str(DEFAULT_WORKER_PYTHON)))
        if not worker_python.is_file():
            raise FileNotFoundError(f'Python CNN no encontrado: {worker_python}')
        worker_env = os.environ.copy()
        worker_env.pop('PYTHONPATH', None)
        worker_env.pop('PYTHONHOME', None)
        worker_env.update({
            'PYTHONNOUSERSITE': '1',
            'TF_CPP_MIN_LOG_LEVEL': '2',
            'CUDA_VISIBLE_DEVICES': '-1',
            'OPENBLAS_NUM_THREADS': '1',
            'OMP_NUM_THREADS': '1',
        })
        self.worker = subprocess.Popen(
            [str(worker_python), '-u', '-c', WORKER_CODE, str(model_path)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            env=worker_env,
        )
        listo = self.worker.stdout.readline()
        if listo.strip() != b'OK':
            raise RuntimeError('No se pudo inicializar el proceso de TensorFlow.')

        self.pub_estacion = self.create_publisher(String, '/estacion_detectada', 10)
        self.pub_info = self.create_publisher(String, '/estacion_cnn_info', 10)

        self.sub_img = None
        self.ultima_inferencia = 0.0
        self.ultimo_log = 0.0
        self._conectar_camara()
        self.timer_discovery = self.create_timer(2.0, self._verificar_camara)

    def _param(self, nombre: str, default):
        p = self.get_parameter_or(nombre, None)
        return default if (p is None or p.value is None) else p.value


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

    def _ros_image_a_rgb224(self, msg: Image) -> np.ndarray | None:
        enc = msg.encoding.lower()
        h, w = msg.height, msg.width
        arr = np.frombuffer(msg.data, dtype=np.uint8)

        if enc == 'rgb8':
            rgb = arr.reshape((h, w, 3))
        elif enc == 'bgr8':
            rgb = arr.reshape((h, w, 3))[:, :, ::-1]
        elif enc == 'rgba8':
            rgb = arr.reshape((h, w, 4))[:, :, :3]
        elif enc == 'bgra8':
            rgb = arr.reshape((h, w, 4))[:, :, 2::-1]
        else:
            return None

        # Redimensionar sin deformar la relacion de aspecto.
        escala = min(224.0 / w, 224.0 / h)
        nuevo_w = max(1, min(224, int(round(w * escala))))
        nuevo_h = max(1, min(224, int(round(h * escala))))

        redimensionada = cv2.resize(
            np.ascontiguousarray(rgb),
            (nuevo_w, nuevo_h),
            interpolation=cv2.INTER_LINEAR,
        )

        salida = np.zeros((224, 224, 3), dtype=np.uint8)
        x0 = (224 - nuevo_w) // 2
        y0 = (224 - nuevo_h) // 2
        salida[y0:y0+nuevo_h, x0:x0+nuevo_w] = redimensionada
        return salida

    def _leer_exacto(self, n_bytes: int) -> bytes:
        buf = bytearray()
        while len(buf) < n_bytes:
            chunk = self.worker.stdout.read(n_bytes - len(buf))
            if not chunk:
                raise RuntimeError('El proceso worker de TensorFlow se detuvo.')
            buf.extend(chunk)
        return bytes(buf)

    def cb_imagen(self, msg: Image):
        ahora = time.monotonic()
        if ahora - self.ultima_inferencia < self.periodo_inferencia:
            return
        self.ultima_inferencia = ahora

        img_224 = self._ros_image_a_rgb224(msg)
        if img_224 is None:
            self.get_logger().warn(
                f'Codificacion de imagen no soportada: {msg.encoding}',
                throttle_duration_sec=5.0,
            )
            return

        self.worker.stdin.write(img_224.tobytes())
        self.worker.stdin.flush()

        n_clases = len(self.clases)
        raw = self._leer_exacto((n_clases + 1) * 4)
        salida = np.frombuffer(raw, dtype=np.float32)
        probs = salida[:n_clases]
        latencia_ms = float(salida[n_clases])

        idx = int(np.argmax(probs))
        confianza = float(probs[idx])
        etiqueta = self.clases[idx] if confianza >= self.umbral else 'INCIERTO'

        msg_str = String()
        msg_str.data = etiqueta
        self.pub_estacion.publish(msg_str)

        detalle = {
            'frame_id': time.monotonic_ns(),
            'estacion': etiqueta,
            'clase_top1': self.clases[idx],
            'confianza': round(confianza, 4),
            'latencia_ms': round(latencia_ms, 2),
            'probs': {
                self.clases[i]: round(float(probs[i]), 4)
                for i in range(n_clases)
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

    def cerrar(self):
        if getattr(self, 'worker', None) and self.worker.poll() is None:
            self.worker.terminate()


def main():
    rclpy.init()
    nodo = ReconocedorEstacion()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.cerrar()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
