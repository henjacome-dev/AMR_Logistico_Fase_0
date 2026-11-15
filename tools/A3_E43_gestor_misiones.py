#!/usr/bin/env python3
"""E4.3.1: planificador determinista de ordenes A* consecutivas, sin ROS.

Basado exclusivamente en los cuatro CSV F12 existentes. Permite invertir
segmentos y concatenarlos con continuidad, sin modificar el algoritmo A*.
No inicia Gazebo, no publica /cmd_vel ni presupone posicion del robot.

Uso:
  python3 tools/A3_E43_gestor_misiones.py --self-test
  python3 tools/A3_E43_gestor_misiones.py --desde ROBOT --orden RECEPCION ALMACENAMIENTO EMPAQUE DESPACHO --guardar
  python3 tools/A3_E43_gestor_misiones.py --desde EMPAQUE --orden RECEPCION DESPACHO
"""
from __future__ import annotations

import argparse
import csv
import datetime
import hashlib
import heapq
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIS = ROOT / 'matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis'
FIS_SHA = '67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903'
PUNTOS = {
    'ROBOT': (-2.975, -2.775),
    'RECEPCION': (-3.975, -2.775),
    'ALMACENAMIENTO': (-3.975, 2.825),
    'EMPAQUE': (4.025, 2.825),
    'DESPACHO': (4.025, -2.775),
}
SEGMENTOS = (
    ('ROBOT', 'RECEPCION', 'F12_ruta_1.csv'),
    ('RECEPCION', 'ALMACENAMIENTO', 'F12_ruta_2.csv'),
    ('ALMACENAMIENTO', 'EMPAQUE', 'F12_ruta_3.csv'),
    ('EMPAQUE', 'DESPACHO', 'F12_ruta_4.csv'),
)
MAX_SALTO_M = 0.15001


def distancia(a, b):
    return math.dist(a, b)


def leer_csv(ruta):
    with ruta.open('r', encoding='utf-8-sig', newline='') as archivo:
        lector = csv.DictReader(archivo)
        if not lector.fieldnames or not {'x_m', 'y_m'}.issubset(lector.fieldnames):
            raise ValueError(f'CSV sin cabeceras x_m,y_m: {ruta}')
        puntos = [(float(r['x_m']), float(r['y_m'])) for r in lector]
    if len(puntos) < 2:
        raise ValueError(f'Ruta insuficiente: {ruta}')
    if not all(math.isfinite(v) for punto in puntos for v in punto):
        raise ValueError(f'Ruta contiene datos no finitos: {ruta}')
    if any(distancia(a, b) > MAX_SALTO_M for a, b in zip(puntos, puntos[1:])):
        raise ValueError(f'Ruta presenta salto > 0.15 m: {ruta}')
    return puntos


def catalogo():
    if not FIS.is_file() or hashlib.sha256(FIS.read_bytes()).hexdigest() != FIS_SHA:
        raise ValueError('El FIS Mamdani v03 oficial no coincide con SHA256 esperado')
    grafo = {nombre: [] for nombre in PUNTOS}
    for desde, hasta, nombre in SEGMENTOS:
        ruta = ROOT / 'resultados' / nombre
        puntos = leer_csv(ruta)
        if distancia(puntos[0], PUNTOS[desde]) > 1e-4:
            raise ValueError(f'Origen distinto en {nombre}')
        if distancia(puntos[-1], PUNTOS[hasta]) > 1e-4:
            raise ValueError(f'Destino distinto en {nombre}')
        longitud = sum(distancia(a, b) for a, b in zip(puntos, puntos[1:]))
        grafo[desde].append((hasta, longitud, nombre, False, puntos))
        grafo[hasta].append((desde, longitud, nombre, True, list(reversed(puntos))))
    return grafo


def camino(grafo, origen, destino):
    if origen not in grafo or destino not in grafo:
        raise ValueError(f'Estacion no reconocida: {origen} -> {destino}')
    if origen == destino:
        return []
    pendientes = [(0.0, origen, [])]
    costos = {origen: 0.0}
    while pendientes:
        costo, actual, pasos = heapq.heappop(pendientes)
        if costo > costos[actual] + 1e-9:
            continue
        if actual == destino:
            return pasos
        for vecino, longitud, archivo, inversa, puntos in grafo[actual]:
            nuevo = costo + longitud
            if nuevo < costos.get(vecino, math.inf) - 1e-9:
                costos[vecino] = nuevo
                detalle = {'desde': actual, 'hasta': vecino, 'f12': archivo,
                           'sentido_inverso': inversa, 'longitud_m': longitud,
                           'puntos': puntos}
                heapq.heappush(pendientes, (nuevo, vecino, pasos + [detalle]))
    raise ValueError(f'No existe recorrido: {origen} -> {destino}')


def planificar(grafo, origen, destinos):
    if origen not in PUNTOS:
        raise ValueError(f'Origen invalido: {origen}')
    if not destinos:
        raise ValueError('Se requiere al menos un destino')
    etapas = []
    actual = origen
    total_m = 0.0
    for numero, destino in enumerate(destinos, 1):
        if destino == 'ROBOT' or destino not in PUNTOS:
            raise ValueError(f'Destino invalido #{numero}: {destino}')
        segmentos = camino(grafo, actual, destino)
        puntos = []
        metadatos = []
        for s in segmentos:
            secuencia = s['puntos']
            if puntos and distancia(puntos[-1], secuencia[0]) > 1e-5:
                raise ValueError('Error de continuidad al concatenar segmentos')
            puntos.extend(secuencia[1:] if puntos else secuencia)
            metadatos.append({k: s[k] for k in ('desde', 'hasta', 'f12',
                                                'sentido_inverso', 'longitud_m')})
        if not puntos:
            puntos = [PUNTOS[actual]]
        if distancia(puntos[0], PUNTOS[actual]) > 1e-4:
            raise ValueError('El plan inicia fuera de la estacion declarada')
        if distancia(puntos[-1], PUNTOS[destino]) > 1e-4:
            raise ValueError('El plan termina fuera de la estacion objetivo')
        if any(distancia(a, b) > MAX_SALTO_M for a, b in zip(puntos, puntos[1:])):
            raise ValueError('El plan contiene un salto espacial no permitido')
        longitud = sum(distancia(a, b) for a, b in zip(puntos, puntos[1:]))
        etapas.append({'numero': numero, 'origen': actual, 'destino': destino,
                       'longitud_m': round(longitud, 5), 'num_waypoints': len(puntos),
                       'sin_desplazamiento': len(puntos) == 1,
                       'segmentos': metadatos, 'puntos': puntos})
        total_m += longitud
        actual = destino
    return {'origen': origen, 'destinos': list(destinos), 'etapas': etapas,
            'longitud_total_m': round(total_m, 5), 'fis_sha256': FIS_SHA,
            'modo': 'PLANIFICACION_ESTATICA_SIN_MOVIMIENTO',
            'nota': ('La ejecucion ROS requiere validacion de pose actual, '
                     'obstaculos, seguimiento y confirmacion CNN; '
                     'este plan no acredita misiones ejecutadas.')}


def resumen_publico(plan):
    resultado = {k: v for k, v in plan.items() if k != 'etapas'}
    resultado['etapas'] = [
        {k: v for k, v in etapa.items() if k != 'puntos'}
        for etapa in plan['etapas']
    ]
    return resultado


def guardar_plan(plan):
    destino_base = ROOT / 'resultados/A3_E43_gestor_misiones'
    destino_base.mkdir(parents=True, exist_ok=True)
    nombre = 'plan_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    carpeta = destino_base / nombre
    carpeta.mkdir(exist_ok=False)
    for etapa in plan['etapas']:
        if etapa['sin_desplazamiento']:
            continue
        archivo = carpeta / (f"mision_{etapa['numero']:02d}_" +
                             etapa['destino'].lower() + '.csv')
        with archivo.open('w', encoding='utf-8', newline='') as f:
            escritor = csv.writer(f, lineterminator='\n')
            escritor.writerow(('x_m', 'y_m'))
            escritor.writerows(etapa['puntos'])
    (carpeta / 'plan.json').write_text(
        json.dumps(resumen_publico(plan), ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    return carpeta


def auto_prueba():
    graf = catalogo()
    realizadas = 0

    def ok(condicion, etiqueta):
        nonlocal realizadas
        assert condicion, etiqueta
        realizadas += 1
        print(f'[OK {realizadas:02d}] {etiqueta}')

    ok(set(graf) == set(PUNTOS), 'Catalogo: cinco ubicaciones')
    ok(sum(map(len, graf.values())) == 8, 'Cuatro conexiones en ambos sentidos')
    ok(all(len(leer_csv(ROOT / 'resultados' / s[2])) >= 2 for s in SEGMENTOS),
       'Cuatro rutas A* existentes y validas')
    ok(all(distancia(PUNTOS[x], camino(graf, x, y)[0]['puntos'][0]) < 1e-5
           for x, y, _ in SEGMENTOS), 'Conservacion del origen de cada tramo')
    ok(all(camino(graf, x, y) for x in PUNTOS for y in PUNTOS if x != y),
       'Conectividad para todos los pares de ubicaciones')
    ok(len(camino(graf, 'RECEPCION', 'DESPACHO')) == 3,
       'Ruta RECEPCION a DESPACHO: tres tramos')
    ok(len(camino(graf, 'DESPACHO', 'RECEPCION')) == 3,
       'Ruta inversa DESPACHO a RECEPCION: tres tramos')
    p = planificar(graf, 'ROBOT', ['RECEPCION', 'ALMACENAMIENTO', 'EMPAQUE', 'DESPACHO'])
    ok(len(p['etapas']) == 4 and all(not x['sin_desplazamiento'] for x in p['etapas']),
       'Secuencia original de cuatro misiones')
    p2 = planificar(graf, 'EMPAQUE', ['RECEPCION', 'DESPACHO'])
    ok(len(p2['etapas']) == 2 and p2['etapas'][1]['destino'] == 'DESPACHO',
       'Orden arbitraria de dos destinos')
    ok(planificar(graf, 'RECEPCION', ['RECEPCION'])['etapas'][0]['sin_desplazamiento'],
       'Destino repetido: sin desplazamiento')
    try:
        planificar(graf, 'ROBOT', ['INEXISTENTE'])
    except ValueError:
        ok(True, 'Destino no reconocido rechazado')
    else:
        raise AssertionError('Destino no reconocido aceptado')
    ok(hashlib.sha256(FIS.read_bytes()).hexdigest() == FIS_SHA,
       'FIS Mamdani v03 oficial intacto')
    assert realizadas == 12
    print('[OK] 12/12 pruebas del planificador aprobadas')
    print('[SEGURIDAD] No se inicio ROS ni se envio /cmd_vel')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--desde', default='ROBOT', choices=tuple(PUNTOS))
    parser.add_argument('--orden', nargs='+', metavar='ESTACION')
    parser.add_argument('--guardar', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        auto_prueba()
        return
    if not args.orden:
        parser.error('Indique --self-test o --orden ESTACION [ESTACION ...]')
    grafo = catalogo()
    plan = planificar(grafo, args.desde, [d.upper() for d in args.orden])
    print(json.dumps(resumen_publico(plan), indent=2, ensure_ascii=False))
    if args.guardar:
        print('[EVIDENCIA]', guardar_plan(plan))
    print('[SEGURIDAD] Planificacion estatica; ningun robot fue controlado')


if __name__ == '__main__':
    main()
