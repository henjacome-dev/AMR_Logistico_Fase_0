#!/usr/bin/env python3
"""Crea una variante de seguimiento R27 SOLO para simulacion.

NO modifica los programas E441, F13 o U3S7 originales. NO inicia ROS.
Preparacion conservadora: avance nominal 0.19 m/s, giro limite 1.0 rad/s,
arranque lineal limitado a 0.45 m/s^2, mayor velocidad entre waypoints.

Este candidato NO garantiza R27: requiere autopruebas y pruebas en Gazebo.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import subprocess
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ORIGS = {
    'seguidor': 'tools/seguidor_ruta_f13.py',
    'supervisor': 'tools/supervisor_u3s7_cnn.py',
    'ejecutor': 'tools/A3_E441_ejecutor_misiones_alineacion.py',
}
DESTS = {
    'seguidor': 'tools/A3_E6_R27_seguidor.py',
    'supervisor': 'tools/A3_E6_R27_supervisor.py',
    'ejecutor': 'tools/A3_E6_R27_ejecutor.py',
}
FIS = 'matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis'
CNN = 'modelos/modelo_estaciones_mobilenetv2_compat.keras'
FIS_HASH = '67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903'
CNN_HASH = '58b7b072b0e98361a908ca5bc07acb767ed7ab8ed6b28ade2e97c42b4b47800d'


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def git(*args: str) -> bytes:
    return subprocess.check_output(['git', '-C', str(ROOT), *args])


def sustituir(texto: str, anterior: str, nuevo: str, detalle: str) -> str:
    veces = texto.count(anterior)
    if veces != 1:
        raise RuntimeError(f'[BLOQUEADO] {detalle}: se esperaban 1 coincidencia, encontradas {veces}')
    return texto.replace(anterior, nuevo, 1)


def preparar():
    assert ROOT.resolve() == Path(git('rev-parse', '--show-toplevel').decode().strip()).resolve(), 'Raiz Git inesperada'
    assert git('branch', '--show-current').decode().strip() == 'integracion-final', 'Rama distinta de integracion-final'
    assert git('rev-parse', '--short', 'HEAD').decode().strip() == '73793ac', 'Commit de referencia diferente de 73793ac'
    assert sha((ROOT / FIS).read_bytes()) == FIS_HASH, 'FIS modificado'
    assert sha((ROOT / CNN).read_bytes()) == CNN_HASH, 'CNN modificada'

    originales = {}
    for clave, nombre in ORIGS.items():
        original = (ROOT / nombre).read_bytes()
        versionado = git('show', 'HEAD:' + nombre)
        assert original == versionado, f'Fuente {nombre} no coincide con HEAD; detener'
        originales[clave] = original.decode('utf-8')

    for nombre in DESTS.values():
        assert not (ROOT / nombre).exists(), f'Existe destino, no se sobrescribe: {nombre}'

    seg = originales['seguidor']
    seg = sustituir(seg, 'VMAX = 0.12                   #', 'VMAX = 0.19                   #', 'VMAX de seguidor')
    seg = sustituir(seg, 'WMAX = 0.70                   #', 'WMAX = 1.00                   #', 'WMAX de seguidor')
    seg = sustituir(seg,
                    'velocidad_base = max(0.075, 0.45 * d)',
                    'velocidad_base = max(0.185, 0.45 * d)',
                    'crucero entre waypoints')
    seg = sustituir(seg,
                    'velocidad_base = 0.45 * d',
                    'velocidad_base = max(0.075, 0.80 * d)',
                    'aproximacion final')
    seg = sustituir(seg,
                    '            self.registros = []\n',
                    '            self.registros = []\n'
                    '            self.v_anterior = 0.0\n'
                    '            self.t_v_anterior = time.monotonic()\n',
                    'estado de aceleracion')
    seg = sustituir(seg,
                    '            self.pub.publish(comando)\n            self.registros.append(',
                    '            # R27: limitar aceleracion; permitir frenado inmediato por seguridad.\n'
                    '            dt_v = max(0.0, min(0.25, ahora - self.t_v_anterior))\n'
                    '            comando.linear.x = min(comando.linear.x, self.v_anterior + 0.45*dt_v)\n'
                    '            self.v_anterior = comando.linear.x\n'
                    '            self.t_v_anterior = ahora\n'
                    '            self.pub.publish(comando)\n            self.registros.append(',
                    'aceleracion lineal')

    sup = originales['supervisor']
    sup = sustituir(sup, 'VMAX_RUTA = 0.12', 'VMAX_RUTA = 0.19', 'limite supervisor')
    sup = sustituir(sup, 'WMAX = 0.70', 'WMAX = 1.00', 'limite angular supervisor')

    eje = originales['ejecutor']
    eje = sustituir(eje, "'tools/seguidor_ruta_f13.py'", "'tools/A3_E6_R27_seguidor.py'", 'lanzamiento seguidor E6')
    eje = sustituir(eje, "'tools/supervisor_u3s7_cnn.py'", "'tools/A3_E6_R27_supervisor.py'", 'lanzamiento supervisor E6')
    eje = sustituir(eje,
                    "OUT = ROOT / 'resultados/A3_E441_ejecutor_alineacion'",
                    "OUT = ROOT / 'resultados/A3_E6_R27_pilotos'",
                    'directorio de evidencia E6')

    textos = {'seguidor': seg, 'supervisor': sup, 'ejecutor': eje}
    for clave, codigo in textos.items():
        ast.parse(codigo, filename=DESTS[clave])
        assert codigo != originales[clave]
    assert 'VMAX_DIFUSO = 0.10' in sup
    assert 'LIMITE_OBSTACULO = 0.35' in sup
    assert 'DISTANCIA_FRENADO = 0.35' in seg
    assert 'TOL_FINAL = 0.080' in seg
    assert "'tools/A3_nodo_mamdani_v03.py'" in eje

    # Todas las verificaciones anteriores se completan ANTES de crear archivos.
    creados = []
    try:
        for clave, rel in DESTS.items():
            ruta = ROOT / rel
            datos = textos[clave].encode('utf-8')
            descriptor = os.open(str(ruta), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
            try:
                with os.fdopen(descriptor, 'wb') as archivo:
                    archivo.write(datos)
            except BaseException:
                ruta.unlink(missing_ok=True)
                raise
            creados.append(ruta)

        salida = ROOT / 'resultados/A3_E6_R27_pilotos'
        salida.mkdir(parents=True, exist_ok=True)
        manifiesto = {
            'fecha': datetime.now().isoformat(),
            'base_commit': '73793ac',
            'tipo': 'CANDIDATO_EXPERIMENTAL_NO_VALIDADO',
            'parametros': {
                'vmax_ruta_m_s': 0.19,
                'wmax_rad_s': 1.0,
                'aceleracion_max_m_s2': 0.45,
                'v_min_intermedia_m_s': 0.185,
                'v_max_difuso_m_s': 0.10,
            },
            'originales': {ORIGS[k]: sha(v.encode()) for k,v in originales.items()},
            'variantes': {DESTS[k]: sha(v.encode()) for k,v in textos.items()},
            'fis_sha256': FIS_HASH,
            'modelo_sha256': CNN_HASH,
            'advertencia': 'Preparacion offline; no equivale a certificacion R27 ni autoriza movimiento.',
        }
        evidencia = salida / 'preparacion_R27_candidata.json'
        assert not evidencia.exists(), 'Ya existe un manifiesto; no se sobrescribe'
        evidencia.write_text(json.dumps(manifiesto, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
        print('=== CANDIDATA R27 PREPARADA ===')
        for rel in DESTS.values():
            print('[CREADO]', rel)
        print('[OK] FIS, CNN, seguidor F13, supervisor U3S7 y E441 originales intactos')
        print('[OK] AST Python de las tres variantes aprobado')
        print('[OK] Respeta limite lineal 0.20 m/s y angular 1.50 rad/s')
        print('[EVIDENCIA]', evidencia.relative_to(ROOT))
        print('[SEGURIDAD] Sin ROS, Gazebo, movimientos, commit ni push')
        print('[PENDIENTE] Autopruebas y prevuelo antes de cualquier ensayo de movimiento')
    except BaseException:
        for ruta in creados:
            ruta.unlink(missing_ok=True)
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preparar', action='store_true', required=True)
    args = parser.parse_args()
    preparar()
