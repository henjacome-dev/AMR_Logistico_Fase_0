#!/usr/bin/env python3
"""Revisa evidencias de E2.3 capturadas en Ubuntu; NO ejecuta ROS ni Gazebo."""
from pathlib import Path
import csv
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
    'tools/A2_E23_arbitraje_sombra.py': 'e135f50bf338883f6be71b98845b745b2e6ccaed7047d1f84890f717a5274f2d',
    'tools/A2_E232_prueba_arbitraje_ROS_sombra.py': '2fbcb5291cb6642b04879819374c47e87b19de2881e84d0342929a635e763827',
    'tools/A2_E233_supervisor_integrado_aislado.py': '9b8088afebe3bc394443decc1a6eab3d577002efd5159c41e5541df8582a4617',
}
CONFIGS = (
    ('E2.3.1', 'resultados/A2_E23_arbitraje_sombra/evidencia_publicada',
     'estado', 'APROBADO', 'pruebas', 17, 'pruebas.csv', None),
    ('E2.3.2', 'resultados/A2_E232_arbitraje_ros_sombra/evidencia_publicada',
     'aprobado', True, 'tests', 14, 'observaciones.csv', 'nodo_sombra.log'),
    ('E2.3.3', 'resultados/A2_E233_supervisor_integrado_aislado/evidencia_publicada',
     'estado', 'APROBADO', 'pruebas', 20, 'observaciones.csv', 'nodo_integrado_prueba.log'),
)
try:
    for path, expected in SOURCES.items():
        source = ROOT / path
        assert source.is_file(), path
        assert hashlib.sha256(source.read_bytes()).hexdigest() == expected, path
    for label, folder, status_key, status_ok, cases_key, count, csvname, logname in CONFIGS:
        path = ROOT / folder
        report = json.loads((path / 'resumen.json').read_text(encoding='utf-8'))
        assert report[status_key] is status_ok if isinstance(status_ok, bool) else report[status_key] == status_ok, label
        cases = report[cases_key]
        assert isinstance(cases, list) and len(cases) == count, label
        assert all(x.get('aprobado') is True for x in cases), label
        if label == 'E2.3.1':
            assert report['total'] == 17 and report['aprobados'] == 17
            assert report['umbrales']['stop_m'] == 0.35
        if label == 'E2.3.2':
            assert report['fis_blob'] == '6c7dfdd083728d49c64dd1c90a59b19d63222063'
            assert report['muestras_estado'] > 0 and report['muestras_sombra'] > 0
        if label == 'E2.3.3':
            assert report['umbrales']['parada_m'] == 0.35
            assert any(x['caso'] == 'LiDAR peligro aunque Range libre' for x in cases)
        with (path / csvname).open(encoding='utf-8', newline='') as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) >= (count if label == 'E2.3.1' else 1), label
        if logname:
            assert (path / logname).is_file() and (path / logname).stat().st_size > 60, label
        print(f'[OK] {label}: {count}/{count}, evidencia publicada coherente')
    print('=== E2.3: 51/51 VERIFICACIONES Y EVIDENCIAS REGISTRADAS ===')
    print('Nota: CI no ha ejecutado ROS 2 ni Gazebo; R19 operativo permanece pendiente.')
except Exception as exc:
    print('[ERROR E2.3 EVIDENCIA]', str(exc))
    sys.exit(1)
