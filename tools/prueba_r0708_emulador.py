#!/usr/bin/env python3
"""R07/R08: prueba aislada del callback del emulador ROS 2 Humble.

No publica en /scan, /cmd_vel ni inicia Gazebo. Los mensajes LaserScan
sinteticos se entregan directamente al callback real; los Range resultantes
se capturan con publicadores de prueba en memoria.
"""
from __future__ import annotations

import csv
import hashlib
import importlib.util
import math
from pathlib import Path
import statistics
import sys

import rclpy
from sensor_msgs.msg import LaserScan

ROOT = Path(__file__).resolve().parent.parent
EMULADOR = ROOT / 'tools/emulador_sensores_f1.py'
RESULTADOS = ROOT / 'resultados/R0708_validacion'
MUESTRAS = 600


class Captura:
    def __init__(self):
        self.mensajes = []

    def publish(self, mensaje):
        self.mensajes.append(mensaje)

    def ultimo(self):
        assert self.mensajes, 'No se publico Range'
        return self.mensajes[-1]


def scan_virtual(lecturas=None):
    msg = LaserScan()
    msg.header.frame_id = 'base_scan'
    msg.angle_min = math.radians(-90)
    msg.angle_max = math.radians(90)
    msg.angle_increment = math.radians(1)
    msg.range_min = 0.12
    msg.range_max = 3.5
    msg.ranges = [math.inf] * 181
    if lecturas:
        for angulo, distancia in lecturas.items():
            assert isinstance(angulo, int) and -90 <= angulo <= 90
            msg.ranges[angulo + 90] = float(distancia)
    return msg


def exigir(condicion, etiqueta):
    if not condicion:
        raise AssertionError(etiqueta)
    print('[OK]', etiqueta)


def main():
    if not EMULADOR.is_file():
        raise SystemExit(f'[ERROR] No existe el emulador: {EMULADOR}')
    if RESULTADOS.exists() and any(RESULTADOS.iterdir()):
        raise SystemExit(f'[ALTO] Evidencias ya existentes: {RESULTADOS}. No se sobrescriben.')

    espec = importlib.util.spec_from_file_location('emulador_r0708', EMULADOR)
    modulo = importlib.util.module_from_spec(espec)
    sys.modules[espec.name] = modulo
    espec.loader.exec_module(modulo)
    rclpy.init()
    nodo = None
    try:
        nodo = modulo.EmuladorSensores()
        capturas = {nombre: Captura() for nombre in modulo.SECTORES}
        # Sustitucion exclusivamente dentro del objeto de prueba; el nodo
        # mantiene sus publicadores ROS originales, pero el callback usa
        # publicadores de captura en memoria.
        nodo.publicadores = capturas
        frontal = '/rango_frontal'
        izq = '/rango_izq'
        der = '/rango_der'
        exigir(set(capturas) == {frontal, izq, der}, 'Tres salidas Range identificadas')
        exigir(modulo.SECTORES == {frontal: 0.0, izq: 45.0, der: -45.0}, 'Centros 0, +45 y -45 grados')
        exigir(abs(math.degrees(modulo.AMPLITUD_RAD)-15) < 1e-10, 'Campo de vision de 15 grados')
        exigir(modulo.ALCANCE_MAX_M == 2.0, 'Alcance maximo 2.0 m')
        exigir(modulo.SIGMA_M == 0.01, 'Sigma configurada 0.01 m')

        # Prueba de enrutamiento angular sin interferencia entre sensores.
        for ang, destino in [(0, frontal), (45, izq), (-45, der)]:
            for salida in capturas.values():
                salida.mensajes.clear()
            nodo.recibir_scan(scan_virtual({ang: 0.80}))
            for nombre, salida in capturas.items():
                m = salida.ultimo()
                if nombre == destino:
                    exigir(abs(m.range - 0.80) < 0.06, f'Deteccion a {ang:+d} grados en {nombre}')
                else:
                    exigir(m.range == 2.0, f'Sensor ajeno sin contaminacion: {ang:+d} grados en {nombre}')
                exigir(abs(math.degrees(m.field_of_view)-15) < 0.0001, f'Metadato FOV correcto en {nombre}')
                exigir(m.min_range == 0 and m.max_range == 2.0, f'Metadatos alcance correctos en {nombre}')
                exigir(m.header.frame_id == 'base_scan', f'Marco correcto en {nombre}')

        # Bordes inclusivos discretizados: +/-7 grados pertenece al sector,
        # +/-8 grados no pertenece. Se comprueba la funcion real sin ruido.
        for centro in (0, 45, -45):
            for desplazamiento in (-7, 7):
                s = scan_virtual({centro + desplazamiento: 0.4})
                exigir(math.isclose(modulo.distancia_sector(s, math.radians(centro)), 0.4, rel_tol=0.0, abs_tol=1e-6),
                       f'Sector {centro:+d}: +{desplazamiento:+d} grados dentro')
            for desplazamiento in (-8, 8):
                s = scan_virtual({centro + desplazamiento: 0.4})
                exigir(modulo.distancia_sector(s, math.radians(centro)) == 2.0,
                       f'Sector {centro:+d}: {desplazamiento:+d} grados fuera')

        # Sin detecciones: cada Range debe saturar exactamente en 2.0 m.
        nodo.recibir_scan(scan_virtual())
        exigir(all(c.ultimo().range == 2.0 for c in capturas.values()),
               'Saturacion a 2.0 m sin obstaculos (sin ruido añadido)')

        distancias = {frontal: 0.70, izq: 1.10, der: 1.40}
        valores = {nombre: [] for nombre in distancias}
        registro = []
        muestra = scan_virtual({0: 0.70, 45: 1.10, -45: 1.40})
        for indice in range(MUESTRAS):
            nodo.recibir_scan(muestra)
            actual = {}
            for nombre, captura in capturas.items():
                lectura = float(captura.ultimo().range)
                exigir(0 <= lectura <= 2.0, 'Rango dentro de [0,2] m') if indice == 0 else None
                valores[nombre].append(lectura)
                actual[nombre] = lectura
            registro.append((indice+1, actual[frontal], actual[izq], actual[der]))

        resumen = []
        for nombre, objetivo in distancias.items():
            media = statistics.mean(valores[nombre])
            desvio = statistics.stdev(valores[nombre])
            ok_media = abs(media - objetivo) <= 0.003
            ok_sigma = 0.008 <= desvio <= 0.012
            exigir(ok_media, f'Ruido {nombre}: media {media:.5f} m, valor esperado {objetivo:.2f} m')
            exigir(ok_sigma, f'Ruido {nombre}: sigma muestral {desvio:.5f} m (objetivo 0.010 m)')
            resumen.append((nombre, objetivo, media, desvio, MUESTRAS, 'OK'))

        RESULTADOS.mkdir(parents=True, exist_ok=True)
        with (RESULTADOS / 'muestras.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f, lineterminator='\n')
            writer.writerow(['indice', 'rango_frontal_m', 'rango_izq_m', 'rango_der_m'])
            writer.writerows(registro)
        with (RESULTADOS / 'resumen.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f, lineterminator='\n')
            writer.writerow(['sensor', 'distancia_base_m', 'media_m', 'sigma_muestral_m', 'muestras', 'estado'])
            for nombre, objetivo, media, desvio, n, estado in resumen:
                writer.writerow([nombre, objetivo, f'{media:.6f}', f'{desvio:.6f}', n, estado])
        hash_archivo = hashlib.sha256(EMULADOR.read_bytes()).hexdigest()
        (RESULTADOS / 'informe.txt').write_text(
            'R07/R08 - Ensayo aislado del callback real del emulador\n'
            'ROS 2 Humble; LiDAR sintetico, sin Gazebo ni movimiento\n'
            'Angulos centrales: 0, +45, -45 grados; apertura 15 grados\n'
            'Rango maximo: 2.0 m; ruido normal sigma=0.01 m\n'
            f'Muestras por sensor: {MUESTRAS}; semilla del emulador: 20261105\n'
            f'SHA256 emulador: {hash_archivo}\n'
            'Criterios: |media-base| <=0.003; 0.008<=sigma_muestral<=0.012\n'
            'Limite: este ensayo no mide rendimiento fisico ni errores de\n'
            'posicionamiento de obstaculos en Gazebo.\n'
            'Resultado: OK\n', encoding='utf-8')
        print(f'[OK] Evidencias guardadas en {RESULTADOS.relative_to(ROOT)}')
        print('[R0708] VALIDACION AISLADA COMPLETADA')
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
