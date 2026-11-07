#!/usr/bin/env python3
"""AC-03: validar que el ZIP de 25 archivos coincide con el checkout GitHub."""
from pathlib import Path
from zipfile import ZipFile
import hashlib
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'matlab/A2_MAMDANI'
ZIP=BASE/'A2_F1_cierre_AC03_20261107.zip'
MANIFEST=BASE/'A2_F1_cierre_AC03_20261107.sha256'
PNG=['entrada_df.png','entrada_di.png','entrada_dd.png','salida_v.png','salida_omega.png',
     'superficie_v_df_di_dd_lejos.png','superficie_omega_di_dd_df_cerca.png',
     'superficie_omega_di_dd_df_lejos.png','superficie_v_df_dd_di_media.png']
FILES=sorted([
    'A2_F1_membresias.m', 'A2_F1_reglas_27.m', 'A2_F1_auditoria.m',
    'A2_F1_superficies.m', 'A2_F1_cierre.m', 'A2_F1_reparar_png.m',
    'A2_F1_verificacion_auditoria.m', 'AMR_Mamdani_A2_v01.fis',
    'A2_F1_tabla_27_reglas.csv', 'A2_F1_membresias_base.mat',
    'A2_F1_modelo_27_reglas.mat', 'A2_F1_auditoria_v01.mat',
    'A2_F1_resumen_tecnico.md','A2_F1_acta_auditoria_20261106.md',
    'evidencias/A2_F1_barrido_1331.csv',
    'evidencias/A2_F1_casos_criticos.csv',
] + ['evidencias/'+p for p in PNG])

try:
    assert len(FILES)==25 and len(set(FILES))==25
    lines=MANIFEST.read_text('ascii').splitlines()
    assert len(lines)==25,'Manifiesto: conteo incorrecto'
    expected={}
    for line in lines:
        sha,sep,name=line.partition('  ')
        assert sep and len(sha)==64 and name not in expected
        assert name in FILES
        expected[name]=sha
    assert set(expected)==set(FILES)
    with ZipFile(ZIP) as z:
        assert z.testzip() is None,'ZIP: CRC incorrecto'
        assert set(z.namelist())==set(FILES) and len(z.namelist())==25,'ZIP: archivos inesperados'
        for rel in FILES:
            recorded=z.read(rel)
            local=(BASE/rel).read_bytes()
            assert recorded==local,'ZIP difiere del archivo publicado: '+rel
            assert hashlib.sha256(local).hexdigest()==expected[rel],'SHA-256 incorrecto: '+rel
    print('=== AC-03: ZIP 25/25, manifiesto y checkout: APROBADOS ===')
except Exception as e:
    print('[ERROR AC-03]',e)
    sys.exit(1)
