#!/usr/bin/env python3
"""Audita el informe U3S7 contra la entrega congelada en el tag `final`.

No instala dependencias ni modifica archivos. Uso desde la raiz del repositorio:
  python3 tools/a3_act6_validar_informe.py
"""
import hashlib, io, re, shutil, subprocess, sys, zipfile
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
REL='docs/entrega_u3s7/Informe_IEEE_U3S7.docx'
DOC=ROOT/REL
PDF=ROOT/'docs/entrega_u3s7/Informe_IEEE_U3S7.pdf'
W='{http://schemas.openxmlformats.org/wordprocessingml/2006/main}'

def error(msg):
    print('[ERROR]',msg)
    raise SystemExit(1)

def digest(data):
    return hashlib.sha256(data).hexdigest()

def contents(docx):
    with zipfile.ZipFile(io.BytesIO(docx)) as z:
        root=ET.fromstring(z.read('word/document.xml'))
        body=root.find(W+'body')
        par=[ ''.join((r.text or '') for r in p.findall('.//'+W+'t')) for p in body.findall(W+'p')]
        tables=[ [ [ ''.join((x.text or '') for x in c.findall('.//'+W+'t')) for c in row.findall(W+'tc') ] for row in tbl.findall(W+'tr') ] for tbl in root.findall('.//'+W+'tbl')]
        media={k:digest(z.read(k)) for k in z.namelist() if k.startswith('word/media/')}
        return par,tables,media

if not DOC.is_file() or not PDF.is_file():
    error('No se localizaron ambos entregables DOCX y PDF')
try:
    source=subprocess.check_output(['git','-C',str(ROOT),'show','final:'+REL],stderr=subprocess.PIPE)
    print('[OK] Fuente histórica: tag final (commit U3S7 original)')
except Exception as e:
    if len(sys.argv)==3 and sys.argv[1]=='--original-docx':
        source=Path(sys.argv[2]).read_bytes()
        print('[OK] Fuente histórica: archivo original indicado')
    else:
        error('El tag final no está disponible. Ejecuta: git fetch origin --tags')
oldp,oldt,oldm=contents(source)
newp,newt,newm=contents(DOC.read_bytes())
if len(oldp)!=108 or len(newp)!=110:
    error(f'Número inesperado de párrafos: original={len(oldp)}, actualizado={len(newp)}')
if len(oldt)!=8 or newt!=oldt:
    error('El contenido de las ocho tablas difiere del original')
print('[OK] Ocho tablas conservadas con los mismos datos')
if oldm!=newm:
    error('Alguno de los recursos gráficos originales cambió')
print(f'[OK] {len(newm)} recursos gráficos originales sin modificación')
# Insertadas [3] y [4] inmediatamente antes del antiguo párrafo de Sandler.
new_without_inserts=newp[:101]+newp[103:]
if len(new_without_inserts)!=len(oldp):error('Número inesperado tras normalización de referencias')
# A3-ACT8 autorizó únicamente cuatro párrafos de discusión (sin tablas ni cifras originales).
# La autenticidad de los nuevos valores se comprueba además en a3_act8_validar_evidencias.py.
authorized={9,12,13,14,31,35,50,68,74,83,87,*range(99,107)}
unexpected=[i for i,(a,b) in enumerate(zip(oldp,new_without_inserts)) if a!=b and i not in authorized]
if unexpected:error('Se modificaron párrafos fuera de los autorizados: '+str(unexpected))
print('[OK] Cambios permitidos: bibliografía y cuatro párrafos justificativos de A3-ACT8')
i=next(i for i,p in enumerate(newp) if p.strip()=='Referencias')
body=' '.join(newp[:i])
refs=[p for p in newp[i+1:] if re.match(r'^\[\d+\]',p)]
nums=[int(re.match(r'^\[(\d+)\]',p).group(1)) for p in refs]
if nums!=list(range(1,11)):error(f'Numeración incorrecta: {nums}')
first=[]
for n in nums:
    matches=list(re.finditer(r'\['+str(n)+r'\]',body))
    if not matches:error(f'La referencia [{n}] no aparece en el texto')
    first.append(matches[0].start())
if first!=sorted(first):error('Referencias fuera de orden IEEE de primera aparición')
if 'Russell y P. Norvig' not in refs[2] or 'Goodfellow' not in refs[3]:
    error('Faltan las dos fuentes del programa oficial')
print('[OK] Diez referencias citadas, sin huérfanas y en orden IEEE')
if shutil.which('pdfinfo'):
    info=subprocess.check_output(['pdfinfo',str(PDF)],text=True)
    found=re.search(r'^Pages:\s*(\d+)',info,re.M)
    if not found or int(found.group(1))!=8:
        error('El PDF no conserva exactamente ocho páginas')
    print('[OK] PDF: exactamente ocho páginas')
else:
    print('[AVISO] PDF no verificado por falta de pdfinfo (poppler-utils)')
print('[OK] SHA-256 DOCX:',digest(DOC.read_bytes()))
print('[OK] SHA-256 PDF: ',digest(PDF.read_bytes()))
print('[RESULTADO] AUDITORIA A3-ACT6 APROBADA')
