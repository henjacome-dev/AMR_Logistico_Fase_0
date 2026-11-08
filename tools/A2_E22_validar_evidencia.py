#!/usr/bin/env python3
"""Comprobar esquema y resultados declarados del ensayo E2.2 ejecutado en Ubuntu.
No ejecuta ROS y no equivale a una prueba de integración en GitHub Actions.
"""
from pathlib import Path
import json
import sys
ROOT = Path(__file__).resolve().parents[1]
EVID = ROOT / "resultados/A2_E22_prueba_ros_aislada/evidencia_publicada/resumen.json"
LOG = EVID.with_name("nodo_ros.log")
REQUERIDOS = {
    "interfaces_ros", "sin_publicador_cmd_vel", "sin_sensores",
    "tres_sensores_activo", "caducidad", "recuperacion",
    "nan_frontal", "aislamiento_confirmado",
}
try:
    data = json.loads(EVID.read_text(encoding="utf-8"))
    assert data.get("aprobado") is True
    assert data.get("fis_git_blob") == "6c7dfdd083728d49c64dd1c90a59b19d63222063"
    assert isinstance(data.get("checks"), dict)
    assert REQUERIDOS.issubset(data["checks"])
    assert all(data["checks"][key] == "OK" for key in REQUERIDOS)
    assert int(data.get("muestras_estado", 0)) > 0
    assert int(data.get("muestras_cmd_vel_difuso", 0)) > 0
    assert LOG.is_file() and LOG.stat().st_size > 10
    print("=== E2.2: EVIDENCIA ROS AISLADA REGISTRADA Y COHERENTE ===")
    print("Nota: no se ha reejecutado ROS 2 en esta verificación.")
except Exception as exc:
    print("[ERROR E2.2 evidencia]", exc)
    sys.exit(1)
