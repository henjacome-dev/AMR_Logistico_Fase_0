#!/usr/bin/env python3
"""Verificacion automatica de integridad del Avance 2 (biblioteca estandar).
Ejecutar en la raiz de un checkout o desde GitHub Actions.
No modifica archivos ni intenta reemplazar auditorias numericas de MATLAB.
"""
from pathlib import Path
import hashlib
import re
import struct
import sys
import zlib

ROOT = Path(__file__).resolve().parent.parent
BASE = ROOT / 'matlab' / 'A2_MAMDANI'
PNGS = [
    'entrada_df.png', 'entrada_di.png', 'entrada_dd.png',
    'salida_v.png', 'salida_omega.png',
    'superficie_v_df_di_dd_lejos.png',
    'superficie_omega_di_dd_df_cerca.png',
    'superficie_omega_di_dd_df_lejos.png',
    'superficie_v_df_dd_di_media.png',
]
FIS_SHA = '6c7dfdd083728d49c64dd1c90a59b19d63222063'
ERRORS = []

def failure(message):
    ERRORS.append(message)
    print('[ERROR]', message)

def check_file(relative):
    f = ROOT / relative
    if not f.is_file() or f.stat().st_size == 0:
        failure('Falta o esta vacio: ' + relative)
        return None
    return f

def validate_png(path):
    data = path.read_bytes()
    if len(data) < 2000 or data[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('Firma PNG invalida o datos insuficientes')
    pos = 8
    widths = []
    payload = bytearray()
    seen_end = False
    while pos < len(data):
        if pos + 12 > len(data):
            raise ValueError('Chunk incompleto')
        size = struct.unpack('>I', data[pos:pos+4])[0]
        typ = data[pos+4:pos+8]
        end = pos + 12 + size
        if end > len(data):
            raise ValueError('Chunk truncado')
        chunk = data[pos+8:pos+8+size]
        crc_expected = struct.unpack('>I', data[pos+8+size:end])[0]
        crc_actual = zlib.crc32(typ + chunk) & 0xffffffff
        if crc_expected != crc_actual:
            raise ValueError('Error CRC en ' + str(typ))
        if typ == b'IHDR':
            if widths or size != 13:
                raise ValueError('IHDR duplicado o invalido')
            width, height = struct.unpack('>II', chunk[:8])
            widths.append((width, height))
        if typ == b'IDAT':
            payload.extend(chunk)
        if typ == b'IEND':
            seen_end = True
            if end != len(data):
                raise ValueError('Bytes sobrantes despues de IEND')
            break
        pos = end
    if not widths or not seen_end or not payload:
        raise ValueError('No contiene IHDR, IDAT e IEND completos')
    width, height = widths[0]
    if width < 400 or height < 300:
        raise ValueError('Resolucion demasiado baja')
    raw = zlib.decompress(payload)
    if not raw:
        raise ValueError('Datos de imagen vacios')
    return width, height

print('=== VERIFICACION AUTOMATICA DE PUBLICACION A2 ===')
required = [
    'README.md', 'docs/F0_requisitos.md',
    'matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis',
    'matlab/A2_MAMDANI/A2_F1_membresias.m',
    'matlab/A2_MAMDANI/A2_F1_reglas_27.m',
    'matlab/A2_MAMDANI/A2_F1_auditoria.m',
    'matlab/A2_MAMDANI/A2_F1_superficies.m',
    'matlab/A2_MAMDANI/A2_F1_cierre.m',
    'matlab/A2_MAMDANI/A2_F1_reparar_png.m',
    'matlab/A2_MAMDANI/A2_F1_verificacion_auditoria.m',
    'matlab/A2_MAMDANI/A2_F1_acta_auditoria_20261106.md',
    'matlab/A2_MAMDANI/A2_F1_resumen_tecnico.md',
    'matlab/A2_MAMDANI/A2_F1_tabla_27_reglas.csv',
    'matlab/A2_MAMDANI/evidencias/A2_F1_barrido_1331.csv',
    'matlab/A2_MAMDANI/evidencias/A2_F1_casos_criticos.csv',
]
for rel in required:
    check_file(rel)

fis = BASE / 'AMR_Mamdani_A2_v01.fis'
if fis.is_file():
    data = fis.read_bytes()
    git_hash = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
    if git_hash != FIS_SHA:
        failure('FIS v01 distinto de la referencia inmutable: ' + git_hash)
    txt = data.decode('utf-8', errors='replace')
    for field, expected in [('NumInputs',3),('NumOutputs',2),('NumRules',27)]:
        if not re.search(r'^' + field + r'\s*=\s*' + str(expected) + r'\s*$',txt,re.M):
            failure('FIS: valor inesperado de ' + field)

folder = BASE / 'evidencias'
if folder.is_dir():
    actual = {f.name for f in folder.glob('*.png')}
    if actual != set(PNGS):
        failure('PNG esperados vs encontrados: faltan=%s, extras=%s' %
                (sorted(set(PNGS) - actual), sorted(actual - set(PNGS))))
    dimensions = set()
    for name in PNGS:
        path = folder / name
        if not path.is_file():
            failure('Falta PNG: ' + name)
            continue
        try:
            dims = validate_png(path)
            dimensions.add(dims)
            print('[OK] PNG', name, f'{dims[0]}x{dims[1]}', f'({path.stat().st_size} bytes)')
        except Exception as e:
            failure('PNG ' + name + ': ' + str(e))
    if len(dimensions) > 1:
        failure('Las nueve imagenes no comparten resolucion: ' + str(dimensions))
else:
    failure('Falta carpeta evidencias')

requirements = ROOT / 'docs/F0_requisitos.md'
if requirements.is_file():
    txt = requirements.read_text(encoding='utf-8')
    for key in ('R15', 'R16', 'R17', 'R18', 'R19', 'R20', 'R21'):
        if not re.search(r'^\|\s*' + key + r'\s*\|', txt, re.M):
            failure('Falta requisito ' + key)

if ERRORS:
    print(f'=== PUBLICACION NO VALIDADA: {len(ERRORS)} ERROR(ES) ===')
    sys.exit(1)
print('=== PUBLICACION A2: INTEGRIDAD ESTATICA APROBADA ===')
print('Nota: la revision de ejes y la reproduccion matematica requieren MATLAB.')
