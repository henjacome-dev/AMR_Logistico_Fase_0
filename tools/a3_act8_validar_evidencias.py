#!/usr/bin/env python3
"""Recalcula la justificación A3/ACT8 sin TensorFlow, Gazebo ni alterar originales."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile

HERE = Path(__file__).resolve()
ROOT = HERE.parents[1]

SHA = {
    'resultados/U3S7/secuencia/resumen.json': '7754ac02816b609e1333e179351a7e9b461d47cb217f6cc61631db77fa9421f9',
    'resultados/cnn_u3s7_20261116/evaluacion.json': 'd076638e12f3e9a00dcc98d791846ec6046f7b1fdbc5e1a3e8aa61f98a336aeb',
    'resultados/cnn_u3s7_20261116/robustez.json': 'bf8709480fabc1e3313ca7d14dcfdef0e585b004b177a16bd220493ffdfcb84c',
    'resultados/U3S7/tabla_ordenes.csv': '8a76b9380cfc7773ce4fe12e6c7eeed358d1e1ef25f4a8550bf38151a3559e1d',
}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def check(condition, message):
    if not condition:
        raise AssertionError(message)
    print('[OK]', message)


def read_csv(path):
    with path.open(encoding='utf-8-sig', newline='') as handle:
        return list(csv.DictReader(handle))


def verify():
    for rel, wanted in SHA.items():
        path = ROOT / rel
        check(path.exists(), f'Fuente encontrada: {rel}')
        data = path.read_bytes()
        if rel.endswith('tabla_ordenes.csv'):
            data = data.replace(b'\r\n', b'\n')
        check(sha(data) == wanted, f'Contenido original identificado: {rel}')

    seq = json.loads((ROOT / 'resultados/U3S7/secuencia/resumen.json').read_text(encoding='utf-8'))
    ev = json.loads((ROOT / 'resultados/cnn_u3s7_20261116/evaluacion.json').read_text(encoding='utf-8'))
    rob = json.loads((ROOT / 'resultados/cnn_u3s7_20261116/robustez.json').read_text(encoding='utf-8'))
    orders = read_csv(ROOT / 'resultados/U3S7/tabla_ordenes.csv')
    derived = read_csv(ROOT / 'docs/A3_ACT8_TIEMPOS_POR_ORDEN.csv')
    check(len(orders) == len(seq['etapas']) == len(derived) == 10, 'Diez órdenes, etapas y resultados derivados')
    cond = []
    for src, stage, row in zip(orders, seq['etapas'], derived):
        n = int(src['numero'])
        length, limit = float(src['longitud_astar_m']), float(src['limite_s'])
        sim, total = float(src['navegacion_sim_s']), float(src['confirmacion_total_s'])
        check(n == stage['numero'] == int(row['numero']), f'Orden {n}: índice coincidente')
        assert math.isclose(limit, 1.3 * length / 0.20, abs_tol=0.011)
        assert math.isclose(float(stage['navegacion_sim_s']), sim, abs_tol=0.02)
        assert math.isclose(float(stage['duracion_total_orden_s']), total, abs_tol=0.02)
        assert math.isclose(float(row['margen_Gazebo_s']),round(limit - sim, 3), abs_tol=0.001)
        assert (row['cumple_Gazebo'] == 'True') == (sim <= limit)
        assert (row['comparacion_exploratoria_total_vs_limite'] == 'True') == (total <= limit)
        assert (row['alineacion_visual'] == 'True') == bool(stage['alineacion'].get('aplicada'))
        assert row['ruta'] == f"{stage['origen']} → {stage['destino']}"
        assert stage['confirmacion_cnn'] and not stage['contactos'] and src['contactos'] == '0'
        cond.append((n,sim <= limit,total <= limit,bool(stage['alineacion'].get('aplicada'))))
    check(sum(x[1] for x in cond) == 10, 'Navegación simulada dentro de cota: 10/10')
    check(sum(x[2] for x in cond) == 6, 'Comparación informativa de pared: 6/10')
    check([n for n,_,ok,_ in cond if not ok] == [2,4,8,10], 'Órdenes por encima del valor numérico: 2,4,8,10')
    check(all(al for n,_,_,al in cond if n in (2,4,8,10)), 'Alineación visual registrada en esas cuatro órdenes')
    check(seq['modos'].get('DIFUSO',0) == 0, 'No se atribuye DIFUSO a la secuencia automática')
    check(not seq['contactos'], 'Sin contactos externos registrados')
    check(len(ev['errores']) == 1 and math.isclose(ev['exactitud_test'],89/90,abs_tol=1e-10), 'Exactitud nominal y un solo error (89/90)')
    matrix = ev['matriz_confusion']
    check(sum(sum(r) for r in matrix)==90 and sum(matrix[i][i] for i in range(4))==89, 'Matriz nominal de confusión consistente')
    check(len(rob)==3 and [(r['condicion'],r['aciertos'],r['n']) for r in rob] == [('oclusion',39,90),('oscuridad',42,90),('desenfoque',36,90)], 'Tres pruebas perturbadas separadas y exactas')
    d=read_csv(ROOT/'docs/A3_ACT8_FALLOS_CNN.csv')
    check(len(d)==4 and d[0]['caso']=='Nominal', 'Tabla de fallos: un caso nominal y tres perturbados')
    for row,src in zip(d[1:],rob):
        assert math.isclose(float(row['exactitud_pct']),100*src['aciertos']/src['n'],abs_tol=0.0001)
        assert math.isclose(float(row['confianza_pct']),100*src['caso']['confianza'],abs_tol=0.0006)
        assert row['real']==src['caso']['real'] and row['predicha']==src['caso']['predicha']
        for key in ('archivo_original','archivo_alterado'):
            check((ROOT/row[key]).exists(),f"Ilustración verificada: {row[key]}")
    check(seq['modelo_sha256']==ev['modelo_sha256'], 'Mismo SHA de modelo CNN en test y secuencia')
    tool=(ROOT/'tools/evaluar_robustez_cnn.py').read_text(encoding='utf-8')
    check('ImageEnhance.Brightness(im).enhance(.12)' in tool and 'ImageFilter.GaussianBlur(20)' in tool and 'rectangle((120, 65, 520, 295)' in tool,'Perturbaciones documentadas corresponden al código')

    # Verify core updated sections and 8 pages (if pdfinfo is available).
    docx=ROOT/'docs/entrega_u3s7/Informe_IEEE_U3S7.docx'
    pdf=ROOT/'docs/entrega_u3s7/Informe_IEEE_U3S7.pdf'
    assert docx.is_file() and pdf.is_file()
    with zipfile.ZipFile(docx) as z:
        root=ET.fromstring(z.read('word/document.xml'))
        text=' '.join(el.text or '' for el in root.iter() if el.tag.endswith('}t'))
    check('6/10' in text and '10/10' in text and 'relojes distintos' in text,'Informe incluye matiz temporal')
    check('tres errores del test nominal' in text,'Informe distingue fallos nominales de perturbados')
    try:
        result=subprocess.run(['pdfinfo',str(pdf)],capture_output=True,text=True,check=True)
        npages=next(int(line.split(':',1)[1].strip()) for line in result.stdout.splitlines() if line.startswith('Pages:'))
        check(npages==8,'Informe PDF conserva ocho páginas exactas')
    except FileNotFoundError:
        print('[AVISO] Falta pdfinfo: instale poppler-utils para validar las páginas')
    print('[RESULTADO] AUDITORIA A3-ACT8 APROBADA. Ninguna simulación o inferencia fue reejecutada.')

if __name__ == '__main__':
    try:
        verify()
    except (AssertionError, KeyError, FileNotFoundError, ValueError) as e:
        print('[ERROR]',str(e));sys.exit(1)
