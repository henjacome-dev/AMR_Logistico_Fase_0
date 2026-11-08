#!/usr/bin/env python3
"""AMR Logistico - Avance 2, Etapa 2.1 (R18)

Compara las diez salidas criticas publicadas de MATLAB Online con un
controlador Mamdani reconstruido desde el FIS v01 mediante scikit-fuzzy.

Ejecucion: python3 /mnt/c/Users/Henry/Downloads/A2_E21_equivalencia_MATLAB_Python.py
No toca el FIS, no altera archivos oficiales y no realiza Git commit/push.
"""

from __future__ import annotations
import csv
import datetime as dt
import hashlib
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import traceback
import configparser

REPO = (Path(__file__).resolve().parents[1] if Path(__file__).resolve().parent.name == 'tools' else Path.home() / 'AMR_Logistico_Fase_0')
FIS_REL = 'matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis'
CSV_REL = 'matlab/A2_MAMDANI/evidencias/A2_F1_casos_criticos.csv'
EXPECTED_FIS_GIT_BLOB = '6c7dfdd083728d49c64dd1c90a59b19d63222063'
EXPECTED_REFERENCE_BLOB = '5a49f16e1c4c7dc7c5428df53a7fd3480dea591e'
VENV = Path.home() / '.venvs' / 'amr-a2-r18'
PKGS = ('numpy==1.26.4', 'scipy==1.11.4', 'scikit-fuzzy==0.5.0', 'packaging==24.2', 'networkx==3.2.1', 'matplotlib==3.8.4')
THRESHOLD_PCT = 2.0
NEAR_ZERO = 1e-6
ABS_ZERO_TOL = {'v': 1e-5, 'omega': 1e-4}


def log(message):
    print(message, flush=True)


def run(command, *, cwd=None, capture=False):
    p = subprocess.run(command, cwd=cwd, text=True, capture_output=capture)
    if p.returncode != 0:
        detail = (p.stderr or p.stdout or '').strip() if capture else ''
        raise RuntimeError(f'Fallo ({p.returncode}): {" ".join(map(str, command))}' +
                           (f'\n{detail}' if detail else ''))
    return p.stdout.strip() if capture else ''


def git_blob_sha(data: bytes):
    return hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()


def verify_inputs():
    assert (REPO / '.git').is_dir(), f'No existe el repositorio: {REPO}'
    branch = run(['git', 'branch', '--show-current'], cwd=REPO, capture=True)
    assert branch == 'desarrollo-avance2' or os.environ.get('GITHUB_ACTIONS') == 'true', f'Rama inesperada: {branch}'
    for rel, expected in [(FIS_REL, EXPECTED_FIS_GIT_BLOB),
                          (CSV_REL, EXPECTED_REFERENCE_BLOB)]:
        p = REPO / rel
        assert p.is_file(), f'Falta: {p}'
        value = git_blob_sha(p.read_bytes())
        assert value == expected, (f'Archivo distinto de la referencia publicada: {rel}\n'
                                   f'Esperado: {expected}\nActual:   {value}')
        # No se exige que el resto del arbol de trabajo este limpio.
        log(f'[OK] Referencia inalterada: {rel}')
    return branch


def prepare_python():
    import venv
    python = VENV / 'bin' / 'python'
    if not python.is_file():
        log(f'[INFO] Creando entorno virtual aislado en {VENV} ...')
        try:
            venv.EnvBuilder(with_pip=True, clear=False).create(VENV)
        except Exception as exc:
            raise RuntimeError('Falta python3-venv. Instale en Ubuntu: sudo apt install python3-venv') from exc
    test_code = """import importlib.metadata as im
checks={'numpy':'1.26.4','scipy':'1.11.4','scikit-fuzzy':'0.5.0','packaging':'24.2','networkx':'3.2.1','matplotlib':'3.8.4'}
assert all(im.version(k)==v for k,v in checks.items())
import numpy,scipy,skfuzzy,packaging,networkx,matplotlib
"""
    test = subprocess.run([str(python), '-c', test_code],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if test.returncode:
        log('[INFO] Instalando todas las dependencias fijadas de R18...')
        run([str(python), '-m', 'pip', 'install', '--disable-pip-version-check', *PKGS])
        run([str(python), '-c', test_code])
    run([str(python), '-m', 'pip', 'check'])
    return python

def parse_fis(fis_path):
    cp = configparser.ConfigParser(interpolation=None, strict=True)
    cp.optionxform = str
    file_text = fis_path.read_text(encoding='utf-8')
    # Las reglas de MATLAB .fis no usan el formato clave=valor de INI.
    # Por eso ConfigParser recibe solo las secciones anteriores a [Rules].
    assert file_text.count('[Rules]') == 1, 'Seccion de reglas ausente/duplicada'
    cp.read_string(file_text.split('[Rules]', 1)[0])
    syssec = cp['System']
    required = {
        'Name': "'AMR_Mamdani_A2_v01'", 'Type': "'mamdani'",
        'NumInputs': '3', 'NumOutputs': '2', 'NumRules': '27',
        'AndMethod': "'min'", 'OrMethod': "'max'", 'ImpMethod': "'min'",
        'AggMethod': "'max'", 'DefuzzMethod': "'centroid'",
    }
    for key, val in required.items():
        assert syssec.get(key, '').strip() == val, f'FIS no coincide en {key}'
    mf_pattern = re.compile(r"^'([^']+)':'(trapmf|trimf)',\[([^\]]+)\]$")

    def variable(kind, idx):
        section = cp[f'{kind}{idx}']
        label = section['Name'].strip("'")
        lo, hi = [float(x) for x in section['Range'].strip('[]').split()]
        count = int(section['NumMFs'])
        mfs = []
        for i in range(1, count + 1):
            spec = section[f'MF{i}']
            match = mf_pattern.fullmatch(spec.strip())
            assert match, f'Formato de MF no reconocido: {spec}'
            name, kind_mf, params = match.groups()
            values = [float(s) for s in params.split()]
            assert len(values) == (3 if kind_mf == 'trimf' else 4)
            assert values == sorted(values), f'MF no monotona: {name}'
            mfs.append((name, kind_mf, values))
        return label, lo, hi, mfs

    inputs = [variable('Input', i) for i in range(1, 4)]
    outputs = [variable('Output', i) for i in range(1, 3)]
    assert [v[0] for v in inputs] == ['df', 'di', 'dd']
    assert [v[0] for v in outputs] == ['v', 'omega']
    assert [(a,b) for _,a,b,_ in inputs] == [(0.,2.)] * 3
    assert [(outputs[i][1],outputs[i][2]) for i in range(2)] == [(0.,0.2),(-1.5,1.5)]
    assert [len(v[3]) for v in inputs] == [3,3,3]
    assert [len(v[3]) for v in outputs] == [3,5]

    rule_pattern = re.compile(r'^\s*(\d+)\s+(\d+)\s+(\d+),\s*'
                              r'(\d+)\s+(\d+)\s+\(([^)]+)\)\s*:\s*(\d+)\s*$')
    rules = []
    rule_text = file_text.split('[Rules]', 1)[1].strip()
    for line in rule_text.splitlines():
        if not line.strip() or line.strip().startswith(('#', '%')):
            continue
        m = rule_pattern.fullmatch(line)
        assert m, f'Regla desconocida: {line}'
        a,b,c,d,e,weight,conn = m.groups()
        row = (int(a),int(b),int(c),int(d),int(e))
        assert float(weight) == 1.0 and int(conn) == 1, f'Regla no equivalente: {line}'
        assert all(1 <= k <= 3 for k in row[:3])
        assert 1 <= row[3] <= 3 and 1 <= row[4] <= 5
        rules.append(row)
    assert len(rules) == 27
    assert len(set(rule[:3] for rule in rules)) == 27
    log('[OK] FIS: 3 entradas, 2 salidas, 27 reglas AND=min; operadores Mamdani verificados.')
    return inputs, outputs, rules


def build_skfuzzy(inputs, outputs, rules):
    import numpy as np
    import skfuzzy as fuzz
    from skfuzzy import control as ctrl

    ins = []
    outs = []
    for label, lo, hi, mfs in inputs:
        obj = ctrl.Antecedent(np.linspace(lo, hi, 2001), label)
        for name, ftype, params in mfs:
            obj[name] = getattr(fuzz, ftype)(obj.universe, params)
        ins.append(obj)
    for label, lo, hi, mfs in outputs:
        # MATLAB audited with evalfisOptions(NumSamplePoints=1001).
        obj = ctrl.Consequent(np.linspace(lo, hi, 1001), label,
                              defuzzify_method='centroid')
        obj.accumulation_method = np.fmax
        for name, ftype, params in mfs:
            obj[name] = getattr(fuzz, ftype)(obj.universe, params)
        outs.append(obj)

    srules = []
    for j, (a,b,c,d,e) in enumerate(rules, start=1):
        ant = ins[0][inputs[0][3][a-1][0]] & ins[1][inputs[1][3][b-1][0]]
        ant = ant & ins[2][inputs[2][3][c-1][0]]
        consequent = [outs[0][outputs[0][3][d-1][0]],
                      outs[1][outputs[1][3][e-1][0]]]
        srules.append(ctrl.Rule(ant, consequent, label=f'R{j:02d}'))
    control = ctrl.ControlSystem(srules)
    return control


def load_reference(path):
    with path.open('r', encoding='utf-8-sig', newline='') as f:
        reader = csv.DictReader(f)
        assert reader.fieldnames == ['Escenario','df','di','dd','v','omega'], \
            'Columnas de CSV MATLAB inesperadas'
        rows = list(reader)
    assert len(rows) == 10
    assert len(set(r['Escenario'] for r in rows)) == 10
    for row in rows:
        for key in ['df','di','dd','v','omega']:
            row[key] = float(row[key])
            assert math.isfinite(row[key])
        assert all(0 <= row[k] <= 2 for k in ['df','di','dd'])
        assert 0 <= row['v'] <= .2 and abs(row['omega']) <= 1.5
    return rows


def compare_value(matlab, python, name):
    absolute = abs(matlab-python)
    normalized_pct = absolute / (0.2 if name == 'v' else 3.0) * 100.
    if abs(matlab) < NEAR_ZERO:
        relative_pct = None
        passed = absolute <= ABS_ZERO_TOL[name]
    else:
        relative_pct = absolute / abs(matlab) * 100.
        passed = relative_pct < THRESHOLD_PCT
    return absolute, relative_pct, normalized_pct, passed


def execute_worker():
    import numpy as np
    import skfuzzy
    import scipy
    from skfuzzy import control as ctrl

    inputs, outputs, rules = parse_fis(REPO / FIS_REL)
    controller = build_skfuzzy(inputs, outputs, rules)
    reference = load_reference(REPO / CSV_REL)
    sim = ctrl.ControlSystemSimulation(controller, clip_to_bounds=False,
                                       cache=False, lenient=False)
    timestamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    base = REPO / 'resultados' / 'A2_R18_equivalencia'
    out = base / timestamp
    increment = 1
    while out.exists():
        out = base / f'{timestamp}_{increment:02d}'
        increment += 1
    out.mkdir(parents=True)
    results = []
    log('=== E2.1 / R18: MATLAB ONLINE VS SCIKIT-FUZZY ===')
    log(f'[INFO] Python={sys.version.split()[0]}; scikit-fuzzy={skfuzzy.__version__}; NumPy={np.__version__}; SciPy={scipy.__version__}')
    for row in reference:
        for k in ['df', 'di', 'dd']:
            sim.input[k] = row[k]
        sim.compute()
        v_py = float(sim.output['v'])
        w_py = float(sim.output['omega'])
        assert math.isfinite(v_py) and math.isfinite(w_py)
        assert -1e-8 <= v_py <= .2 + 1e-8
        assert -1.5 - 1e-8 <= w_py <= 1.5 + 1e-8
        eva, evr, evn, ok_v = compare_value(row['v'], v_py, 'v')
        ewa, ewr, ewn, ok_w = compare_value(row['omega'], w_py, 'omega')
        result = dict(
            Escenario=row['Escenario'], df=row['df'],di=row['di'],dd=row['dd'],
            v_matlab=row['v'], v_python=v_py,
            v_error_absoluto=eva, v_error_relativo_pct='' if evr is None else evr,
            v_error_rango_pct=evn,
            omega_matlab=row['omega'], omega_python=w_py,
            omega_error_absoluto=ewa, omega_error_relativo_pct='' if ewr is None else ewr,
            omega_error_rango_pct=ewn,
            v_aprobado=ok_v, omega_aprobado=ok_w, caso_aprobado=ok_v and ok_w)
        results.append(result)
        log(f'[{"OK" if result["caso_aprobado"] else "ERROR"}] '
            f'{row["Escenario"]:<25} '
            f'v(M/P)={row["v"]:.7f}/{v_py:.7f} '
            f'w(M/P)={row["omega"]:+.7f}/{w_py:+.7f} '
            f'err(v/w)={eva:.2e}/{ewa:.2e}')

    outfile = out / 'A2_R18_comparacion_10_casos.csv'
    with outfile.open('w', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0].keys()), lineterminator='\n')
        writer.writeheader()
        writer.writerows(results)
    passed = all(r['caso_aprobado'] for r in results)
    fail_count = sum(not r['caso_aprobado'] for r in results)
    max_v_abs = max(r['v_error_absoluto'] for r in results)
    max_w_abs = max(r['omega_error_absoluto'] for r in results)
    max_v_rel = max([float(r['v_error_relativo_pct']) for r in results if r['v_error_relativo_pct'] != ''] or [0])
    max_w_rel = max([float(r['omega_error_relativo_pct']) for r in results if r['omega_error_relativo_pct'] != ''] or [0])
    summary = [
        '# Avance 2 — Etapa 2.1: equivalencia MATLAB–Python (R18)',
        '',
        f'**Estado:** {"APROBADO" if passed else "NO APROBADO"}; {10-fail_count}/10 casos.',
        f'**Fecha local:** {dt.datetime.now().isoformat(timespec="seconds")}.',
        f'**FIS de referencia:** `{FIS_REL}` (Git blob `{EXPECTED_FIS_GIT_BLOB}`).',
        f'**CSV MATLAB:** `{CSV_REL}` (Git blob `{EXPECTED_REFERENCE_BLOB}`).',
        f'**Biblioteca:** scikit-fuzzy {skfuzzy.__version__}; Python {sys.version.split()[0]}; NumPy {np.__version__}; SciPy {scipy.__version__}.',
        '',
        '## Método',
        '',
        '- 27 reglas copiadas directamente del FIS, AND=min, OR=max, implicación=min, agregación=max, defuzzificación=centroide.',
        '- Universos de salida con 1001 puntos, como en la auditoría MATLAB. scikit-fuzzy puede refinar numéricamente el centroide, por lo que no se exige identidad bit a bit.',
        '- Para referencias de magnitud >=1e-6 se exige error relativo **estrictamente menor al 2 %**, para cada salida en cada caso.',
        '- Para referencias próximas a cero (<1e-6) el error relativo no es significativo: se exige error absoluto <=1e-5 m/s (v) o <=1e-4 rad/s (omega).',
        '- Se registran también errores absolutos y porcentajes normalizados por el rango de salida.',
        '',
        f'**Máximo error relativo v:** {max_v_rel:.6f} %.',
        f'**Máximo error relativo omega:** {max_w_rel:.6f} %.',
        f'**Máximo error absoluto v:** {max_v_abs:.9g} m/s.',
        f'**Máximo error absoluto omega:** {max_w_abs:.9g} rad/s.',
        '',
        '## Resultados',
        '',
        '| Escenario | v MATLAB | v Python | omega MATLAB | omega Python | Cumple |',
        '|---|---:|---:|---:|---:|:---:|',
    ]
    for r in results:
        summary.append(f'| {r["Escenario"]} | {r["v_matlab"]:.8f} | {r["v_python"]:.8f} | '
                       f'{r["omega_matlab"]:+.8f} | {r["omega_python"]:+.8f} | '
                       f'{"Sí" if r["caso_aprobado"] else "NO"} |')
    summary.extend([
        '',
        '## Limitaciones y continuidad',
        '',
        '- La comparación usa resultados de MATLAB previamente aprobados, no ejecuta MATLAB Online de nuevo.',
        '- No acredita integración ROS 2, evasión de obstáculos ni seguridad operacional.',
        '- En `df=di=dd=0.2 m`, la salida Mamdani puede mantener v>0. La parada estricta debe imponerse mediante el supervisor.',
        '- No modificar el FIS `v01`; preparar la integración ROS 2 solamente después de aprobar esta prueba.',
        '',
    ])
    (out / 'A2_R18_resumen.md').write_text('\n'.join(summary), encoding='utf-8')
    log(f'[RESULTADO] {10-fail_count}/10 casos aprobados')
    log(f'[ERROR MAX] v={max_v_rel:.6f}% relativo; omega={max_w_rel:.6f}% relativo (excluye referencias ~0)')
    log(f'[ARCHIVOS] {outfile}')
    log(f'[ARCHIVOS] {out / "A2_R18_resumen.md"}')
    if passed:
        log('=== R18 / ETAPA 2.1: EQUIVALENCIA NUMERICA APROBADA ===')
    else:
        log('=== R18 / ETAPA 2.1: EQUIVALENCIA NO APROBADA ===')
        log('[ACCION] Revisar diferencias sin modificar el FIS original.')
    return int(not passed)


def main():
    log('=== AMR LOGISTICO / ETAPA 2.1 / R18 ===')
    try:
        verify_inputs()
        if '--worker' in sys.argv:
            return execute_worker()
        interpreter = prepare_python()
        log('[INFO] Ejecutando comparacion en entorno Python aislado...')
        command = [str(interpreter), str(Path(__file__).resolve()), '--worker']
        return subprocess.run(command).returncode
    except Exception as exc:
        log(f'[ERROR] {exc}')
        log('[INFO] No se modifico FIS, no se hizo commit ni push.')
        return 1


if __name__ == '__main__':
    sys.exit(main())
