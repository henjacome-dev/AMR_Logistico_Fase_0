#!/usr/bin/env python3
"""Verificador no invasivo de evidencias A2. Python >=3.9, sin dependencias externas.
Lee datos originales, nunca modifica .fis, CSV históricos o imágenes.
"""
from pathlib import Path
from itertools import product
import csv
import hashlib
import math
import re
import struct
import sys
import zlib

ROOT = Path(__file__).resolve().parents[1]
HIST = ROOT / 'matlab/A2_MAMDANI'
V01 = HIST / 'AMR_Mamdani_A2_v01.fis'
V03 = HIST / 'A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis'
CSV01 = HIST / 'A2_F1_tabla_27_reglas.csv'
CSV03 = ROOT / 'docs/A2_ACT4_REGLAS_V03.csv'
R18 = ROOT / 'resultados/A2_R18_equivalencia/evidencia_publicada/A2_R18_comparacion_10_casos.csv'
PNGS = ['entrada_df.png','entrada_di.png','entrada_dd.png','salida_v.png','salida_omega.png',
        'superficie_v_df_di_dd_lejos.png','superficie_v_df_dd_di_media.png',
        'superficie_omega_di_dd_df_cerca.png','superficie_omega_di_dd_df_lejos.png']
EXPECTED = {
    V03: '67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903',
    V01: 'ae1ea3dc2a005b159a1edc7bd1b8b8001c66c815fa475ddce3c17c1a35b16de3',
    R18: 'dfdbbf1be40d8e9ed9d53bb36a9a97ec184b15930a9706248d7d6c430bb4747b',
}


def check(condition, message):
    if not condition:
        raise AssertionError(message)
    print('[OK]', message)


def parse_fis(path):
    sections = {}
    section = None
    for line in path.read_text(encoding='utf-8-sig').splitlines():
        line = line.strip()
        if not line or line.startswith(('%','#')):
            continue
        if line.startswith('[') and line.endswith(']'):
            section = line[1:-1]
            sections[section] = []
        else:
            if section is None:
                raise AssertionError(f'FIS sin sección: {line}')
            sections[section].append(line)
    system = dict(row.split('=', 1) for row in sections['System'])
    check(system.get('Type') == "'mamdani'" and system.get('NumInputs') == '3'
          and system.get('NumOutputs') == '2' and system.get('NumRules') == '27',
          f'{path.name}: configuración Mamdani (3 entradas, 2 salidas, 27 reglas)')
    for key,val in {'AndMethod':"'min'",'OrMethod':"'max'",'ImpMethod':"'min'",
                    'AggMethod':"'max'",'DefuzzMethod':"'centroid'"}.items():
        assert system.get(key) == val, f'{path.name}: operador {key} != {val}'
    vars_ = {}
    for kind, amount in [('Input',3),('Output',2)]:
        for idx in range(1, amount+1):
            data = sections[f'{kind}{idx}']
            names = [re.fullmatch(r"MF\d+='([^']+)':'(\w+)',\[(.+)\]",row)
                     for row in data if row.startswith('MF')]
            assert names and all(names), f'{path}: membretes no interpretables'
            name = next(re.fullmatch(r"Name='([^']+)'",x).group(1) for x in data if x.startswith('Name='))
            rng = next(re.fullmatch(r'Range=\[([^]]+)\]',x).group(1) for x in data if x.startswith('Range='))
            vars_[name] = {'range':tuple(float(v) for v in rng.split()),'mfs':[m.group(1) for m in names],
                           'defs':[(m.group(2),tuple(float(v) for v in m.group(3).split())) for m in names]}
    assert [x for x in vars_] == ['df','di','dd','v','omega']
    assert all(vars_[x]['range'] == (0.0,2.0) and vars_[x]['mfs'] == ['Cerca','Media','Lejos'] for x in ['df','di','dd'])
    assert vars_['v']['range'] == (0.0,.2) and vars_['omega']['range'] == (-1.5,1.5)
    rules = []
    for line in sections['Rules']:
        m = re.fullmatch(r'(\d+) (\d+) (\d+), (\d+) (\d+) \(([^)]+)\) : (\d+)',line)
        assert m, f'Regla no parseable: {line}'
        a,b,c,d,e = (int(m.group(i)) for i in range(1,6))
        assert m.group(6) == '1' and m.group(7) == '1', 'Pesos/conector inesperados'
        assert all(1 <= v <= 3 for v in (a,b,c,d)) and 1 <= e <= 5, 'Indice fuera de rango'
        rules.append((a,b,c,d,e))
    check(len(rules)==27 and {r[:3] for r in rules}==set(product(range(1,4),repeat=3)),
          f'{path.name}: 27/27 antecedentes únicos, sin duplicación exacta')
    return vars_,rules


def mapped_rows(vars_, rules):
    return [dict(regla=str(i),df=vars_['df']['mfs'][r[0]-1],di=vars_['di']['mfs'][r[1]-1],
                 dd=vars_['dd']['mfs'][r[2]-1],v=vars_['v']['mfs'][r[3]-1],
                 omega=vars_['omega']['mfs'][r[4]-1],peso='1',conexion='AND')
            for i,r in enumerate(rules,1)]


def verify_png(path):
    blob = path.read_bytes()
    assert blob.startswith(b'\x89PNG\r\n\x1a\n')
    pos = 8
    ihdr = None
    got_end = False
    chunks = 0
    while pos + 12 <= len(blob):
        length = struct.unpack('>I',blob[pos:pos+4])[0]
        marker = blob[pos+4:pos+8]
        end = pos+12+length
        assert end<=len(blob), f'{path}: PNG truncado'
        payload = blob[pos+8:pos+8+length]
        expected_crc = struct.unpack('>I',blob[pos+8+length:end])[0]
        assert zlib.crc32(marker+payload)&0xffffffff == expected_crc, f'{path}: CRC inválido'
        if marker==b'IHDR':
            ihdr=struct.unpack('>II',payload[:8])
        if marker==b'IEND':
            got_end=True
            break
        pos=end
        chunks+=1
    assert got_end and ihdr==(900,650) and chunks>=3, f'{path}: estructura PNG inesperada'


def main():
    print('=== A2 / ACTIVIDAD 4: VERIFICACIÓN ESTÁTICA SIN MODIFICAR ARCHIVOS ===')
    for path, sha in EXPECTED.items():
        check(path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest()==sha,
              f'SHA-256: {path.relative_to(ROOT)}')
    v1,r1=parse_fis(V01)
    v3,r3=parse_fis(V03)
    check(v3['v']['mfs']==['Lenta','Media','Rapida'], 'v03: etiquetas Lenta / Media / Rapida')
    check(v1['v']['mfs']==['Detenido','Lenta','Rapida'], 'v01 histórico: etiquetas Detenido / Lenta / Rapida')
    check(sum(a[3]!=b[3] for a,b in zip(r1,r3))==13 and all(a[:3]==b[:3] and a[4]==b[4] for a,b in zip(r1,r3)),
          'v01 vs v03: 13 consecuentes de velocidad distintos; giro sin cambios')
    expected_01 = mapped_rows(v1,r1)
    with CSV01.open(encoding='utf-8-sig',newline='') as f:
        original = list(csv.DictReader(f))
    check(len(original)==27 and all([row[x] for x in ['df','di','dd','v','omega']]
          == [expected_01[i][x] for x in ['df','di','dd','v','omega']] for i,row in enumerate(original)),
          'CSV histórico: tabla 27/27 coincide con FIS v01 (NO v03)')
    with CSV03.open(encoding='utf-8',newline='') as f:
        current = list(csv.DictReader(f))
    check(current==mapped_rows(v3,r3), 'CSV nuevo: 27 reglas v03 reproducidas del FIS oficial')
    for name in PNGS:
        verify_png(HIST/'evidencias'/name)
    check(True,'9 PNG históricos v01 con CRC correcto, dimensiones 900x650')
    with R18.open(encoding='utf-8-sig',newline='') as f:
        records = list(csv.DictReader(f))
    check(len(records)==10 and len({r['Escenario'] for r in records})==10,
          'R18: 10 escenarios únicos registrados (referencia v01)')
    maxima = {'v':0.0,'omega':0.0}
    for r in records:
        assert r['caso_aprobado']=='True' and r['v_aprobado']=='True' and r['omega_aprobado']=='True'
        for field, eps in [('v',1e-5),('omega',1e-4)]:
            base=float(r[f'{field}_matlab']);val=float(r[f'{field}_python'])
            diff=abs(base-val)
            assert all(math.isfinite(x) for x in (base,val,diff))
            assert abs(diff-float(r[f'{field}_error_absoluto']))<1e-10
            if abs(base)>=1e-6:
                pct=100*diff/abs(base)
                assert pct<2 and abs(pct-float(r[f'{field}_error_relativo_pct']))<1e-7
                maxima[field]=max(maxima[field],pct)
            else:
                assert diff<=eps
    print(f'[OK] R18: 10/10 v01 bajo 2 % (v={maxima["v"]:.6f} %, omega={maxima["omega"]:.6f} %)')
    print('[RESERVA] No se ha reejecutado MATLAB/Python, ni validado concordancia v03.')
    print('[RESULTADO] Documentación estática A2 validada sin modificar el controlador.')


if __name__=='__main__':
    try:
        main()
    except Exception as exc:
        print('[ERROR]', type(exc).__name__,str(exc),file=sys.stderr)
        sys.exit(1)
