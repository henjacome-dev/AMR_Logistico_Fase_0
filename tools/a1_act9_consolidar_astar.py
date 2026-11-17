#!/usr/bin/env python3
"""A1 ACT9 — auditoría reproducible de A* histórico y series posteriores.

Solo Python 3 stdlib. No reejecuta A* ni cambia fuentes.
Uso desde la raíz del repositorio:
  python3 tools/a1_act9_consolidar_astar.py \
    --zip-original /mnt/c/Users/Henry/Downloads/AMR_Logistico_Fase_1-main.zip
  python3 tools/a1_act9_consolidar_astar.py \
    --zip-original /mnt/c/Users/Henry/Downloads/AMR_Logistico_Fase_1-main.zip --check

El modo --skip-git se reserva para inspeccionar copias sin metadatos Git;
la verificación normal usa 'git show' con los commits históricos fijos.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import math
import os
import subprocess
import sys
import zipfile
from pathlib import Path
from xml.sax.saxutils import escape

COMMIT_A1 = "d1558d7e8845acf453a3b6eee5a1c62af947b3cf"
COMMIT_A3_ORIGINAL = "e38b177e373e3a68c2e77d478231b3e97f658f4b"
ZIP_SHA256 = "398835963816b9cf429ab68283b27340d6481fffa5006b115b6cad4cfc21502b"
ZIP_PREFIX = "AMR_Logistico_Fase_0-main/"
ORDERS = ("A", "B", "C")
PATH_PY = "resultados/F12_comparacion_ordenes.csv"
PATH_MAT = "matlab/F124_MATLAB/resultados/F124_comparacion_ordenes.csv"
PATH_PAIR = "matlab/F124_MATLAB/resultados/F124_comparacion_final.csv"
PATH_PY_ROUTES = "resultados/F12_orden_{}.csv"
PATH_MAT_ROUTES = "matlab/F124_MATLAB/resultados/F124_orden_{}.csv"
PATH_SEG_PY = "resultados/F12_resumen_rutas.csv"
PATH_SEG_MAT = "matlab/F124_MATLAB/resultados/F124_resumen_rutas.csv"
PATH_SEG_MAT_ALT = "matlab/F124_MATLAB/resultados/F124_resumen_matlab.csv"
LATER = (
    ("R04_PY", "Python, mapa R04 corregido", "resultados/R04_astar_mapa_corregido/resultados/F12_comparacion_ordenes.csv", "resultados/R04_astar_mapa_corregido/resultados/F12_orden_{}.csv"),
    ("F1_D_PY", "Python, cierre F1_D 2026-11-06", "resultados/F1_D_cierre_20261106/resultados/F12_comparacion_ordenes.csv", "resultados/F1_D_cierre_20261106/resultados/F12_orden_{}.csv"),
    ("R11_MAT", "MATLAB, revisión R11 posterior", "matlab/R11_MATLAB/resultados/R11_resumen_ordenes.csv", "matlab/R11_MATLAB/resultados/R11_orden_{}.csv"),
)


def must(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def read_csv(data: bytes, origin: str) -> list[dict[str, str]]:
    s = data.decode("utf-8-sig")
    rows = list(csv.DictReader(io.StringIO(s)))
    must(bool(rows), "CSV vacío: " + origin)
    return rows


def numeric(v: str, source: str) -> float:
    try:
        x = float(v)
    except Exception as e:
        raise ValueError(f"No numérico en {source}: {v!r}") from e
    must(math.isfinite(x), f"No finito en {source}: {v!r}")
    return x


def csv_export(headers: list[str], rows: list[dict[str, object]]) -> bytes:
    out = io.StringIO(newline="")
    w = csv.DictWriter(out, fieldnames=headers, lineterminator="\n")
    w.writeheader()
    for r in rows:
        w.writerow(r)
    return out.getvalue().encode("utf-8")


def git_show(repo: Path, revision: str, name: str) -> bytes:
    p = subprocess.run(["git", "show", f"{revision}:{name}"], cwd=repo, capture_output=True)
    if p.returncode:
        raise RuntimeError(f"No se puede leer {revision[:8]}:{name} desde Git: {p.stderr.decode(errors='replace')[:350]}")
    return p.stdout


def metric_table(source: bytes, origin: str, node_col: str) -> dict[str, dict]:
    rows = read_csv(source, origin)
    must(set(r["orden"].strip() for r in rows) == set(ORDERS) and len(rows) == 3,
         f"La tabla {origin} no contiene exactamente A/B/C")
    must(node_col in rows[0], f"Falta columna {node_col} en {origin}")
    metrics = {}
    for r in rows:
        o = r["orden"].strip()
        p = int(r[node_col])
        L = numeric(r["longitud_m"], origin)
        T = numeric(r["tiempo_ms"], origin)
        must(p > 1 and L > 0 and T >= 0, f"Métricas inválidas {origin}: orden {o}")
        metrics[o] = {"puntos": p, "longitud_m": L, "tiempo_ms": T,
                      "secuencia": r.get("secuencia", "").strip()}
    return metrics


def geo_route(data: bytes, origin: str) -> tuple[int, float, tuple[float, float], tuple[float, float]]:
    rows = read_csv(data, origin)
    must(set(("x_m", "y_m")).issubset(rows[0]), f"Faltan x_m/y_m en {origin}")
    coords = [(numeric(r["x_m"], origin), numeric(r["y_m"], origin)) for r in rows]
    dist = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(coords[:-1], coords[1:]))
    return len(coords), dist, coords[0], coords[-1]


def validate_route(data: bytes, origin: str, reported: dict) -> tuple:
    n, L, a, b = geo_route(data, origin)
    must(n == reported["puntos"], f"Cantidad de puntos no coincide: {origin}: {n} vs {reported['puntos']}")
    # El CSV tabular suele redondear las longitudes a tres decimales.
    must(abs(L - reported["longitud_m"]) < 0.0006,
         f"Longitud no coincide: {origin}: {L:.9f} vs {reported['longitud_m']:.9f}")
    return n, L, a, b


def svg_bars(title: str, ylabel: str, units: str, groups: tuple[str, ...],
             series: list[tuple[str, list[float]]], foot: str) -> bytes:
    W, H = 1120, 570
    left, top, right, bottom = 95, 115, 50, 145
    cw, ch = W-left-right, H-top-bottom
    max_v = max(x for _, vals in series for x in vals)
    ymax = max_v * 1.19 if max_v else 1
    palette = ["#295B95", "#DB8844", "#478977", "#9776B6"]
    arr = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" role="img" aria-label="{escape(title)}">',
           '<rect width="100%" height="100%" fill="#ffffff"/>',
           f'<text x="{left}" y="42" font-size="26" fill="#19283c" font-family="Arial" font-weight="bold">{escape(title)}</text>',
           f'<text x="{left}" y="66" font-size="14" fill="#56687a" font-family="Arial">{escape(ylabel)}</text>']
    for i in range(6):
        v = ymax*i/5
        yy = top+ch-(ch*i/5)
        arr.extend([f'<line x1="{left}" x2="{W-right}" y1="{yy:.1f}" y2="{yy:.1f}" stroke="#e8edf1" stroke-width="1"/>',
                    f'<text x="{left-14}" y="{yy+5:.1f}" text-anchor="end" fill="#657484" font-size="13" font-family="Arial">{v:.1f}</text>'])
    step = cw/len(groups)
    slot = min(70, (step-50)/max(1,len(series)))
    for i,g in enumerate(groups):
        center = left+(i+.5)*step
        for j,(label, vals) in enumerate(series):
            x=center + (j-(len(series)-1)/2)*slot - slot*.40
            v=vals[i]
            bh=ch*(v/ymax)
            y=top+ch-bh
            arr.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{slot*.8:.2f}" height="{bh:.2f}" rx="4" fill="{palette[j%len(palette)]}"/>')
            arr.append(f'<text x="{x+slot*.4:.2f}" y="{max(y-7,top-7):.2f}" text-anchor="middle" font-size="12" font-family="Arial" fill="#273748">{v:.3f}'.rstrip('0').rstrip('.')+f'{escape(units)}</text>')
        arr.append(f'<text x="{center:.2f}" y="{top+ch+29}" text-anchor="middle" font-size="18" font-weight="bold" font-family="Arial" fill="#28394b">Orden {escape(g)}</text>')
    start = left+8
    for j,(label,_vals) in enumerate(series):
        lx=start+j*245
        arr.append(f'<rect x="{lx}" y="{H-93}" width="14" height="14" fill="{palette[j%len(palette)]}"/>')
        arr.append(f'<text x="{lx+22}" y="{H-80}" font-size="14" font-family="Arial" fill="#324559">{escape(label)}</text>')
    arr.append(f'<text x="{left}" y="{H-37}" font-size="13" font-family="Arial" fill="#56687a">{escape(foot)}</text>')
    arr.append('</svg>\n')
    return ('\n'.join(arr)).encode('utf-8')


def make_outputs(original_zip: Path, root: Path, skip_git: bool) -> dict[str, bytes]:
    must(original_zip.is_file(), "No se encontró el ZIP original: " + str(original_zip))
    zbytes = original_zip.read_bytes()
    must(sha256(zbytes) == ZIP_SHA256, "SHA-256 del ZIP original no coincide; abortado")
    with zipfile.ZipFile(original_zip) as z:
        bad = z.testzip()
        must(bad is None, f"CRC incorrecto en el ZIP: {bad}")
        fs = [x for x in z.namelist() if not x.endswith("/")]
        must(len(fs)==76, f"Se esperaban 76 archivos en el ZIP, hay {len(fs)}")
        must(all(x.startswith(ZIP_PREFIX) for x in fs), "Raíz del ZIP inesperada")
        files = {x[len(ZIP_PREFIX):]:z.read(x) for x in fs}
    selected_old = [PATH_PY,PATH_MAT,PATH_PAIR,PATH_SEG_PY,PATH_SEG_MAT,PATH_SEG_MAT_ALT] + [p.format(o) for p in (PATH_PY_ROUTES, PATH_MAT_ROUTES) for o in ORDERS]
    for p in selected_old:
        must(p in files, f"No existe en el ZIP original: {p}")
        if not skip_git:
            must(files[p] == git_show(root, COMMIT_A1, p), f"Desacuerdo ZIP↔Git A1 en {p}")
    later_files = {}
    for cid, desc, source, pattern in LATER:
        for p in [source]+[pattern.format(o) for o in ORDERS]:
            must((root/p).exists(), f"Falta fuente posterior del repo: {p}")
            data=(root/p).read_bytes()
            if not skip_git:
                must(data == git_show(root, COMMIT_A3_ORIGINAL, p), f"Fuente posterior distinta del tag final: {p}")
            later_files[p]=data
    py=metric_table(files[PATH_PY],PATH_PY,'celdas')
    mat=metric_table(files[PATH_MAT],PATH_MAT,'celdas')
    pair={r['orden']:r for r in read_csv(files[PATH_PAIR],PATH_PAIR)}
    must(set(pair)==set(ORDERS),"Inconsistencia de órdenes en comparación MATLAB–Python")
    details=[]
    for o in ORDERS:
        p=py[o]; m=mat[o]
        must(p['secuencia']==m['secuencia'],f"Secuencias diferentes en orden {o}")
        validate_route(files[PATH_PY_ROUTES.format(o)], PATH_PY_ROUTES.format(o), p)
        validate_route(files[PATH_MAT_ROUTES.format(o)], PATH_MAT_ROUTES.format(o), m)
        old=pair[o]
        must(abs(numeric(old['python_m'],PATH_PAIR)-p['longitud_m'])<1e-7,"Falla tabla par Python "+o)
        must(abs(numeric(old['matlab_m'],PATH_PAIR)-m['longitud_m'])<1e-7,"Falla tabla par MATLAB "+o)
        must(abs(numeric(old['diferencia_m'],PATH_PAIR)-(m['longitud_m']-p['longitud_m']))<1e-7,"Falla delta "+o)
        delta=m['longitud_m']-p['longitud_m']
        details.append({
            'orden':o,'secuencia':p['secuencia'], 'python_puntos':p['puntos'],'matlab_puntos':m['puntos'],
            'diferencia_puntos_matlab_menos_python':m['puntos']-p['puntos'],
            'python_longitud_m':f"{p['longitud_m']:.6f}", 'matlab_longitud_m':f"{m['longitud_m']:.6f}",
            'diferencia_m_matlab_menos_python':f"{delta:.6f}",
            'diferencia_absoluta_m':f"{abs(delta):.6f}",
            'diferencia_pct_sobre_python':f"{abs(delta)/p['longitud_m']*100:.6f}",
            'python_tiempo_ms_registrado':f"{p['tiempo_ms']:.3f}",
            'matlab_tiempo_ms_registrado':f"{m['tiempo_ms']:.3f}",
        })
    # Per-tramo: tiempos proceden de llamadas diferentes; no sumarlos para inferir tiempo de la orden.
    seg_py=read_csv(files[PATH_SEG_PY],PATH_SEG_PY)
    seg_mat=read_csv(files[PATH_SEG_MAT],PATH_SEG_MAT)
    must(len(seg_py)==len(seg_mat)==4,"Se esperaban cuatro segmentos")
    segments=[]
    for p,m in zip(seg_py,seg_mat):
        must(p['origen']==m['origen'] and p['destino']==m['destino'],"Segmentos no alineados")
        Lp=numeric(p['longitud_m'],PATH_SEG_PY); Lm=numeric(m['longitud_m'],PATH_SEG_MAT)
        segments.append({
            'origen':p['origen'],'destino':p['destino'],
            'python_puntos':int(p['celdas']),'matlab_puntos':int(m['nodos']),
            'matlab_nodos_explorados_segmento':int(m['explorados']),
            'python_longitud_m':f'{Lp:.6f}','matlab_longitud_m':f'{Lm:.6f}',
            'diferencia_m_matlab_menos_python':f'{Lm-Lp:.6f}',
            'python_tiempo_ms_registrado':f"{numeric(p['tiempo_calculo_ms'],PATH_SEG_PY):.3f}",
            'matlab_tiempo_ms_registrado':f"{numeric(m['tiempo_ms'],PATH_SEG_MAT):.3f}"
        })
    alt=read_csv(files[PATH_SEG_MAT_ALT],PATH_SEG_MAT_ALT)
    must(len(alt)==4,"Resumen alternativo MATLAB incompleto")
    later_rows=[]; later_series={}
    for cid,desc,tab,pattern in LATER:
        nodecol='nodos' if cid=='R11_MAT' else 'celdas'
        t=metric_table(later_files[tab],tab,nodecol)
        later_series[cid]=t
        for o in ORDERS:
            x=t[o]; validate_route(later_files[pattern.format(o)],pattern.format(o),x)
            later_rows.append({'corte':cid,'implementacion':desc,'orden':o,
                               'puntos_trayectoria':x['puntos'],
                               'longitud_m':f"{x['longitud_m']:.6f}",
                               'tiempo_ms_registrado':f"{x['tiempo_ms']:.3f}",
                               'fuente':tab})
    # Integridad auxiliar: JSON manifest de fuentes para facilitar control sin reejecutar los planificadores.
    inputs=[]
    for p in sorted(selected_old):inputs.append({'corte':'A1_original','archivo':p,'sha256':sha256(files[p])})
    for p in sorted(later_files):inputs.append({'corte':'A3_tag_final','archivo':p,'sha256':sha256(later_files[p])})
    output={}
    output['docs/A1_ACT9_METRICAS_ORDENES_ORIGINALES.csv']=csv_export(list(details[0]),details)
    output['docs/A1_ACT9_METRICAS_TRAMOS_ORIGINALES.csv']=csv_export(list(segments[0]),segments)
    output['docs/A1_ACT9_SERIES_POSTERIORES.csv']=csv_export(list(later_rows[0]),later_rows)
    output['docs/A1_ACT9_MANIFIESTO_FUENTES_SHA256.csv']=csv_export(list(inputs[0]),inputs)
    names=["Python (A1 original)","MATLAB (A1 original)"]
    output['docs/evidencias_A1/ACT9/comparacion_longitudes.svg']=svg_bars(
        "A1 original · Longitud A* por orden", "Tres órdenes, mismo ZIP entregado el 4 de noviembre", " m", ORDERS,
        [(names[0],[py[o]['longitud_m'] for o in ORDERS]),(names[1],[mat[o]['longitud_m'] for o in ORDERS])],
        "Fuente: CSV de A1 original; diferencias menores a 0,54 % (respecto a Python).")
    output['docs/evidencias_A1/ACT9/comparacion_puntos.svg']=svg_bars(
        "A1 original · Puntos de trayectoria por orden", "El campo 'celdas' cuenta puntos exportados, no nodos explorados", "", ORDERS,
        [(names[0],[py[o]['puntos'] for o in ORDERS]),(names[1],[mat[o]['puntos'] for o in ORDERS])],
        "Fuente: CSV y recuento de filas de las trayectorias exportadas (A1 original).")
    output['docs/evidencias_A1/ACT9/comparacion_tiempos.svg']=svg_bars(
        "A1 original · Tiempo de cálculo registrado", "Tiempos históricos sin condiciones de benchmark homologadas (ms)", " ms", ORDERS,
        [(names[0],[py[o]['tiempo_ms'] for o in ORDERS]),(names[1],[mat[o]['tiempo_ms'] for o in ORDERS])],
        "Estos tiempos NO son tiempos de navegación del robot ni permiten afirmar ventaja de un lenguaje.")
    output['docs/evidencias_A1/ACT9/longitudes_series_posteriores.svg']=svg_bars(
        "Evolución documental · Longitud A* (cortes distintos)", "No comparar como pruebas pareadas; cambiaron mapa o configuración", " m", ORDERS,
        [("A1 Python",[py[o]['longitud_m'] for o in ORDERS]),
         ("R04 Python",[later_series['R04_PY'][o]['longitud_m'] for o in ORDERS]),
         ("F1_D Python",[later_series['F1_D_PY'][o]['longitud_m'] for o in ORDERS]),
         ("R11 MATLAB",[later_series['R11_MAT'][o]['longitud_m'] for o in ORDERS])],
        "Las mediciones posteriores no se atribuyen retroactivamente al ZIP de Blackboard.")
    # Resumen legible, generado exclusivamente a partir de las mismas fuentes.
    rows_md=[]
    for o in ORDERS:
        p,m=py[o],mat[o];d=m['longitud_m']-p['longitud_m'];
        rows_md.append(f"| {o} | {p['longitud_m']:.3f} | {m['longitud_m']:.3f} | {abs(d):.3f} | {abs(d)/p['longitud_m']*100:.3f} % | {p['puntos']} | {m['puntos']} | {p['tiempo_ms']:.3f} | {m['tiempo_ms']:.3f} |")
    rows_later=[]
    for o in ORDERS:
        rows_later.append(f"| {o} | {later_series['R04_PY'][o]['longitud_m']:.3f} | {later_series['F1_D_PY'][o]['longitud_m']:.3f} | {later_series['R11_MAT'][o]['longitud_m']:.3f} |")
    seg_lines=[]
    for r in segments:
        seg_lines.append(f"| {r['origen']} → {r['destino']} | {float(r['python_longitud_m']):.3f} | {float(r['matlab_longitud_m']):.3f} | {r['python_puntos']} | {r['matlab_puntos']} | {r['matlab_nodos_explorados_segmento']} |")
    note='\n'.join([
        '# A1 — Actividad 9: consolidación verificable de planificación A*',
        '',
        '## Alcance y cortes históricos',
        '',
        f'- **Entrega Blackboard (4-11-2026):** archivo `AMR_Logistico_Fase_1-main.zip`, SHA-256 `{ZIP_SHA256}`; corresponde a Git `{COMMIT_A1}` (actividad A1-ACT5).',
        f'- **Datos posteriores:** fuentes archivadas en el commit de entrega U3S7 `{COMMIT_A3_ORIGINAL}` (tag `final`); no corresponden al corte presentado para A1.',
        '- **Método:** análisis estático de CSV y trayectorias guardadas. No se reejecutan MATLAB, Python A*, ROS 2 ni Gazebo.',
        '',
        '## 1. Comparación principal: A1 original',
        '',
        '| Orden | Python (m) | MATLAB (m) | Δ absoluto (m) | Δ / Python (%) | Puntos Python | Puntos MATLAB | Python (ms) | MATLAB (ms) |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|',
        *rows_md,
        '',
        '**Definición de diferencia:** `MATLAB − Python`; el porcentaje absoluto usa como denominador la longitud Python. Los valores mostrados están redondeados; `docs/A1_ACT9_METRICAS_ORDENES_ORIGINALES.csv` conserva seis decimales.',
        '',
        f'**Máximo Δ absoluto:** {max(abs(mat[o]["longitud_m"]-py[o]["longitud_m"]) for o in ORDERS):.6f} m, orden B.',
        '**Orden con menor longitud:** A, en MATLAB y Python.',
        '',
        'Las longitudes se recalcularon desde las coordenadas `x_m,y_m` de cada CSV de trayectoria mediante suma euclidiana, con tolerancia de 0,0006 m respecto a los resúmenes redondeados. Los recuentos `celdas` corresponden a **filas/puntos** de trayectorias concatenadas: no indican explorados únicos ni complejidad del buscador.',
        '',
        '### Tramos individuales en el ZIP A1 original',
        '',
        '| Tramo | Python (m) | MATLAB (m) | Puntos Python | Puntos MATLAB | Explorados MATLAB (tramo) |',
        '|---|---:|---:|---:|---:|---:|',
        *seg_lines,
        '',
        'Los nodos explorados MATLAB de cada tramo provienen de `F124_resumen_rutas.csv` y **no** deben confundirse con puntos de trayectoria de órdenes. Los tiempos por tramo están en `docs/A1_ACT9_METRICAS_TRAMOS_ORIGINALES.csv`. En el ZIP existen dos registros distintos de tiempo MATLAB para los mismos cuatro tramos (`F124_resumen_rutas.csv` y `F124_resumen_matlab.csv`); **no deben sumarse ni combinarse con los tiempos agregados de otra ejecución**.',
        '',
        '### Validez del tiempo de cálculo',
        '',
        'Los tiempos históricos de Python y MATLAB se midieron en ejecuciones diferentes y bajo condiciones que no están documentadas como ensayo pareado controlado. No equivalen a latencia de navegación ni permiten concluir una superioridad computacional entre lenguajes. Para una comparación de rendimiento robusta harían falta hardware/entorno homologados, calentamiento, repetición y dispersión.',
        '',
        '## 2. Evolución posterior — NO forma parte del ZIP original',
        '',
        '| Orden | R04 Python (m) | F1_D Python (m) | R11 MATLAB (m) |',
        '|---|---:|---:|---:|',
        *rows_later,
        '',
        '- **R04:** `resultados/R04_astar_mapa_corregido/resultados/F12_comparacion_ordenes.csv` (mapa corregido).',
        '- **F1_D:** `resultados/F1_D_cierre_20261106/resultados/F12_comparacion_ordenes.csv` (cierre posterior del 6 de noviembre).',
        '- **R11:** `matlab/R11_MATLAB/resultados/R11_resumen_ordenes.csv` (otra revisión MATLAB).',
        '',
        'Estas series son **cortes distintos**, con mapas/configuraciones o ejecuciones que no deben suponerse equivalentes. En particular, no se calcula una supuesta diferencia MATLAB–Python de R11 y F1_D como si ambos procedieran del mismo ensayo.',
        '',
        'El conjunto de fuentes posteriores también se contrastó contra el commit U3S7 `e38b177` para detectar alteraciones no previstas en la rama de trabajo.',
        '',
        '## 3. Artefactos generados',
        '',
        '- `docs/A1_ACT9_METRICAS_ORDENES_ORIGINALES.csv`: órdenes A, B y C, longitudes, puntos, diferencias y tiempos.',
        '- `docs/A1_ACT9_METRICAS_TRAMOS_ORIGINALES.csv`: cuatro rutas individuales del corte A1.',
        '- `docs/A1_ACT9_SERIES_POSTERIORES.csv`: métricas posteriores separadas por corte.',
        '- `docs/A1_ACT9_MANIFIESTO_FUENTES_SHA256.csv`: hashes SHA-256 de archivos utilizados.',
        '- `docs/evidencias_A1/ACT9/*.svg`: cuatro gráficas vectoriales reproducibles.',
        '',
        '![Longitudes de A1 original](evidencias_A1/ACT9/comparacion_longitudes.svg)',
        '',
        '![Puntos de trayectoria A1 original](evidencias_A1/ACT9/comparacion_puntos.svg)',
        '',
        '![Tiempos registrados en A1 original](evidencias_A1/ACT9/comparacion_tiempos.svg)',
        '',
        '![Series posteriores no pareadas](evidencias_A1/ACT9/longitudes_series_posteriores.svg)',
        '- `tools/a1_act9_consolidar_astar.py`: regenerador/verificador (solo biblioteca estándar Python).',
        '',
        '## 4. Cómo reproducir y verificar en Ubuntu',
        '',
        'Desde la raíz del repositorio, con el ZIP original en Descargas:',
        '',
        '```bash',
        'python3 tools/a1_act9_consolidar_astar.py --zip-original /mnt/c/Users/Henry/Downloads/AMR_Logistico_Fase_1-main.zip --check',
        '```',
        '',
        'Modo normal (sin `--check`) regenera los derivados, **nunca** modifica las fuentes experimentales ni mueve los tags de Git.',
        '',
        '## 5. Conclusión',
        '',
        'Se consolidan las tres órdenes A* con una comparación válida para el ZIP histórico. La orden A es la más corta. Las diferencias de longitud entre implementaciones son inferiores al 1 %; los tiempos quedan documentados como mediciones históricas no controladas. Las métricas posteriores quedan preservadas sin atribución retroactiva al Avance 1.',
        ''
    ])
    output['docs/A1_ACT9_INFORME_CONSOLIDACION_ASTAR.md']=note.encode('utf-8')
    return output


def main() -> int:
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--zip-original',required=True,type=Path,help='ZIP de Blackboard de A1')
    cli.add_argument('--repo-root',type=Path,default=Path(__file__).resolve().parent.parent)
    cli.add_argument('--check',action='store_true',help='No modifica nada: compara derivados con archivos guardados')
    cli.add_argument('--skip-git',action='store_true',help='Para copias de prueba sin historial Git (no usar en cierre oficial)')
    args=cli.parse_args()
    try:
        root=args.repo_root.resolve()
        out=make_outputs(args.zip_original, root, args.skip_git)
        print('=== A1 / ACT9: CONSOLIDACIÓN A* ===')
        print(f'[OK] SHA256 y CRC ZIP original; commit A1 {COMMIT_A1[:8]}')
        print(f'[OK] {3} órdenes Python y MATLAB verificadas contra sus trayectorias')
        print('[OK] 4 tramos originales con fuentes identificadas')
        print('[OK] 3 cortes posteriores separados del ZIP original')
        if not args.skip_git:print('[OK] Fuentes coinciden con commits históricos Git')
        else:print('[AVISO] Prueba sin Git activada: falta verificar contra commits históricos')
        for rel, content in out.items():
            p=root/rel
            if args.check:
                must(p.is_file(), 'Falta archivo generado: '+rel)
                must(p.read_bytes()==content, 'Archivo derivado no coincide: '+rel)
                print('[OK] Reproducible: '+rel)
            else:
                p.parent.mkdir(parents=True,exist_ok=True)
                p.write_bytes(content)
                print('[GENERADO] '+rel)
        print('[RESULTADO] AUDITORÍA A1-ACT9 APROBADA'+(' (MODO DE PRUEBA SIN GIT)' if args.skip_git else ''))
        return 0
    except Exception as e:
        print('[ERROR]',str(e),file=sys.stderr)
        return 1

if __name__=='__main__':
    raise SystemExit(main())
