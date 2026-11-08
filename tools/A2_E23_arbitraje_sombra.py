#!/usr/bin/env python3
"""AMR Logistico - E2.3.1, contrato de arbitraje RUTA/DIFUSO, modo sombra.

No importa ROS, no publica topicos, no arranca Gazebo, no modifica el
supervisor_f13.py ni el FIS. Prototipo de logica verificable ANTES de integracion.

Uso desde WSL2:
    python3 /mnt/c/Users/Henry/Downloads/A2_E23_arbitraje_sombra.py --instalar-y-probar

Los umbrales de conmutacion 0.60/0.80 m son propuestas de ensayo, NO una
atribucion a la rubrica oficial. Deben aprobarse contra el enunciado antes
de E2.3.3 (integracion del supervisor operativo).
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import sys

ROOT = Path.home() / "AMR_Logistico_Fase_0"
SOURCE_NAME = "A2_E23_arbitraje_sombra.py"


@dataclass(frozen=True)
class Config:
    stop_m: float = 0.35             # Conservador; supervisor F1.3 usa r < 0.35
    enter_fuzzy_m: float = 0.60      # PROPUESTA, NO UMBRAL DOCUMENTAL VALIDADO
    exit_fuzzy_m: float = 0.80       # PROPUESTA, histéresis 0.20 m
    sensor_age_s: float = 0.80      # Coherente con nodo E2.2
    odom_age_s: float = 1.50        # Supervisor F1.3
    command_age_s: float = 0.40     # Supervisor F1.3
    max_v_mps: float = 0.12         # Supervisor F1.3
    max_w_rps: float = 0.70         # Supervisor F1.3

    def validate(self):
        assert 0 < self.stop_m < self.enter_fuzzy_m < self.exit_fuzzy_m <= 2.0
        assert 0 < self.command_age_s <= self.sensor_age_s
        assert 0 < self.max_v_mps <= 0.20 and 0 < self.max_w_rps <= 1.5


@dataclass(frozen=True)
class Reading:
    meters: float
    received_s: float


@dataclass(frozen=True)
class Proposal:
    v: float
    w: float
    received_s: float


@dataclass(frozen=True)
class Frame:
    df: Reading | None
    di: Reading | None
    dd: Reading | None
    odom_received_s: float | None
    route: Proposal | None
    fuzzy: Proposal | None


@dataclass(frozen=True)
class Decision:
    mode: str
    reason: str
    v: float = 0.0
    w: float = 0.0


class ShadowArbiter:
    """Contrato puro, con estado para histéresis. NUNCA emite órdenes ROS."""

    def __init__(self, config: Config = Config()):
        config.validate()
        self.config = config
        self.previous_mode = "DETENIDO"

    @staticmethod
    def _recent(now, timestamp, max_age):
        return (timestamp is not None and math.isfinite(timestamp)
                and 0 <= now - timestamp <= max_age)

    def decide(self, frame: Frame, now: float) -> Decision:
        cfg = self.config
        if not math.isfinite(now):
            self.previous_mode = "DETENIDO"
            return Decision("DETENIDO", "TIEMPO_INVALIDO")

        for name, reading in (("df", frame.df), ("di", frame.di), ("dd", frame.dd)):
            if reading is None:
                self.previous_mode = "DETENIDO"
                return Decision("DETENIDO", f"SIN_DATO_{name}")
            if not math.isfinite(reading.meters) or not (0 <= reading.meters <= 2):
                self.previous_mode = "DETENIDO"
                return Decision("DETENIDO", f"INVALIDO_{name}")
            if not self._recent(now, reading.received_s, cfg.sensor_age_s):
                self.previous_mode = "DETENIDO"
                return Decision("DETENIDO", f"CADUCADO_{name}")

        if not self._recent(now, frame.odom_received_s, cfg.odom_age_s):
            self.previous_mode = "DETENIDO"
            return Decision("DETENIDO", "ODOMETRIA_AUSENTE_O_CADUCADA")

        # Barrera de seguridad independiente de Mamdani. Igualdad incluida.
        if frame.df.meters <= cfg.stop_m:
            self.previous_mode = "DETENIDO"
            return Decision("DETENIDO", "PARADA_FRONTAL_ESTRICTA")

        if self.previous_mode == "DIFUSO":
            mode = "RUTA" if frame.df.meters >= cfg.exit_fuzzy_m else "DIFUSO"
        else:
            mode = "DIFUSO" if frame.df.meters <= cfg.enter_fuzzy_m else "RUTA"

        proposal = frame.fuzzy if mode == "DIFUSO" else frame.route
        if proposal is None or not self._recent(now, proposal.received_s, cfg.command_age_s):
            self.previous_mode = "DETENIDO"
            return Decision("DETENIDO", "PROPUESTA_" + mode + "_AUSENTE_O_CADUCADA")

        if not all(math.isfinite(x) for x in (proposal.v, proposal.w)):
            self.previous_mode = "DETENIDO"
            return Decision("DETENIDO", "PROPUESTA_" + mode + "_INVALIDA")
        if proposal.v < 0:
            self.previous_mode = "DETENIDO"
            return Decision("DETENIDO", "MARCHA_ATRAS_NO_AUTORIZADA")

        self.previous_mode = mode
        return Decision(mode, "PROPUESTA_" + mode + "_VALIDA",
                        min(cfg.max_v_mps, proposal.v),
                        max(-cfg.max_w_rps, min(cfg.max_w_rps, proposal.w)))


def fixture(*, now=100.0, front=1.7, left=1.7, right=1.7,
            t_sensor=100.0, t_odom=100.0, t_route=100.0, t_fuzzy=100.0,
            route=(0.10, 0.0), fuzzy=(0.16, 0.2)):
    return Frame(
        None if front is None else Reading(front, t_sensor),
        None if left is None else Reading(left, t_sensor),
        None if right is None else Reading(right, t_sensor),
        t_odom,
        None if route is None else Proposal(route[0], route[1], t_route),
        None if fuzzy is None else Proposal(fuzzy[0], fuzzy[1], t_fuzzy),
    )


def execute_checks():
    tests = []
    arb = ShadowArbiter()

    def check(label, expected_mode, expected_reason=None, *, same_arb=False, **kwargs):
        a = arb if same_arb else ShadowArbiter()
        result = a.decide(fixture(**kwargs), 100.0)
        good = result.mode == expected_mode
        if expected_reason:
            good &= result.reason == expected_reason
        if expected_mode == "DETENIDO":
            good &= result.v == 0 and result.w == 0
        tests.append(dict(prueba=label, aprobado=bool(good),
                          modo=result.mode, motivo=result.reason,
                          v=result.v, omega=result.w))
        print(("[OK]" if good else "[ERROR]") + f" {label}: {result.mode} / {result.reason} / v={result.v:.3f} w={result.w:+.3f}", flush=True)
        return result

    check("Sin frontal", "DETENIDO", "SIN_DATO_df", front=None)
    check("Frontal NaN", "DETENIDO", "INVALIDO_df", front=float('nan'))
    check("Derecho fuera de rango", "DETENIDO", "INVALIDO_dd", right=2.1)
    check("Sensor caducado", "DETENIDO", "CADUCADO_df", t_sensor=99.0)
    check("Odometria caducada", "DETENIDO", "ODOMETRIA_AUSENTE_O_CADUCADA", t_odom=98.0)
    check("Parada frontal inferior al limite", "DETENIDO", "PARADA_FRONTAL_ESTRICTA", front=0.20)
    check("Parada frontal en el limite", "DETENIDO", "PARADA_FRONTAL_ESTRICTA", front=0.35)
    check("Libre usa ruta", "RUTA", front=1.7)
    check("Conmutacion a difuso", "DIFUSO", front=0.60)
    check("Faltan datos de propuesta difusa", "DETENIDO", "PROPUESTA_DIFUSO_AUSENTE_O_CADUCADA", front=0.50, fuzzy=None)
    check("Propuesta difusa caducada", "DETENIDO", "PROPUESTA_DIFUSO_AUSENTE_O_CADUCADA", front=0.50, t_fuzzy=99.3)
    check("Propuesta ruta caducada", "DETENIDO", "PROPUESTA_RUTA_AUSENTE_O_CADUCADA", front=1.7, t_route=99.3)
    check("Propuesta no finita", "DETENIDO", "PROPUESTA_DIFUSO_INVALIDA", front=0.5, fuzzy=(0.12,float('inf')))
    check("Marcha atras rechazada", "DETENIDO", "MARCHA_ATRAS_NO_AUTORIZADA", front=1.7, route=(-0.1,0.0))
    s = check("Saturacion velocidad y giro", "DIFUSO", front=0.50, fuzzy=(0.19,1.3))
    tests[-1]["aprobado"] &= s.v == 0.12 and s.w == 0.70
    a = ShadowArbiter()
    first = a.decide(fixture(front=0.55), 100.0)
    middle = a.decide(fixture(front=0.70), 100.0)
    last = a.decide(fixture(front=0.80), 100.0)
    good = (first.mode, middle.mode, last.mode) == ("DIFUSO", "DIFUSO", "RUTA")
    tests.append(dict(prueba="Histeresis 0.60-0.80", aprobado=good, modo="DIFUSO>DIFUSO>RUTA", motivo="", v=0,omega=0))
    print(("[OK]" if good else "[ERROR]") + " Histeresis 0.60-0.80: DIFUSO -> DIFUSO -> RUTA", flush=True)

    a = ShadowArbiter()
    initial = a.decide(fixture(front=0.50), 100.0)
    stopped = a.decide(fixture(front=0.20), 100.0)
    recovered = a.decide(fixture(front=0.55), 100.0)
    good = (initial.mode, stopped.mode, recovered.mode) == ("DIFUSO", "DETENIDO", "DIFUSO")
    tests.append(dict(prueba="Parada y recuperacion", aprobado=good, modo="DIFUSO>DETENIDO>DIFUSO", motivo="", v=0,omega=0))
    print(("[OK]" if good else "[ERROR]") + " Parada y recuperacion", flush=True)
    return tests


def run_selftest(save: bool):
    print("=== E2.3.1 - ARBITRAJE EN MODO SOMBRA (SIN ROS/GAZEBO) ===",flush=True)
    checks = execute_checks()
    passed = sum(c["aprobado"] for c in checks)
    if save:
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        target = ROOT / "resultados" / "A2_E23_arbitraje_sombra" / now
        i = 1
        while target.exists():
            target = target.parent / (now + f"_{i:02d}")
            i += 1
        target.mkdir(parents=True)
        report = {
            "estado": "APROBADO" if passed == len(checks) else "NO_APROBADO",
            "total": len(checks), "aprobados": passed,
            "umbrales": asdict(Config()),
            "advertencia": "0.60/0.80 m son umbrales de ingeniería propuestos, sujetos a verificación con enunciado.",
            "alcance": "Modo sombra sin ROS, sin publicar /cmd_vel; NO acredita R19 operativo.",
            "pruebas": checks,
        }
        (target / "resumen.json").write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n",encoding="utf8")
        with (target / "pruebas.csv").open("w",newline="",encoding="utf8") as f:
            w = csv.DictWriter(f,fieldnames=list(checks[0].keys()),lineterminator="\n")
            w.writeheader();w.writerows(checks)
        print(f"[EVIDENCIA] {target / 'resumen.json'}",flush=True)
        print(f"[EVIDENCIA] {target / 'pruebas.csv'}",flush=True)
    print(f"[RESULTADO] {passed}/{len(checks)} casos aprobados",flush=True)
    if passed == len(checks):
        print("=== E2.3.1: LOGICA DE ARBITRAJE EN SOMBRA APROBADA ===",flush=True)
        return 0
    print("=== E2.3.1: PRUEBAS NO APROBADAS ===",flush=True)
    return 1


def install_and_test():
    if not (ROOT / ".git").is_dir():
        print(f"[ERROR] Repositorio no encontrado: {ROOT}",flush=True)
        return 1
    dest = ROOT / "tools" / SOURCE_NAME
    source = Path(__file__).resolve()
    if source != dest.resolve():
        if dest.exists():
            if dest.read_bytes() != source.read_bytes():
                print(f"[ERROR] Existe una version distinta, no se sobrescribe: {dest}",flush=True)
                return 1
            print(f"[OK] Script previamente instalado: {dest}",flush=True)
        else:
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source,dest)
            print(f"[OK] Script instalado: {dest}",flush=True)
    return run_selftest(save=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instalar-y-probar",action="store_true")
    parser.add_argument("--self-test",action="store_true")
    args=parser.parse_args()
    if args.instalar_y_probar:
        return install_and_test()
    if args.self_test:
        return run_selftest(save=False)
    parser.print_help()
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"[ERROR] {exc}",flush=True)
        print("[INFO] No se realizaron publicaciones ni cambios en el supervisor.",flush=True)
        sys.exit(1)
