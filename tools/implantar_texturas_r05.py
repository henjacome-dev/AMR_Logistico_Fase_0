#!/usr/bin/env python3
"""R05: instala texturas PNG y materiales OGRE para los 4 paneles de Gazebo Classic.
Conserva los 26 modelos y los colliders del escenario. Ejecutar desde la raíz
AMR_Logistico_Fase_0, en rama de corrección, después de R04.
"""
from pathlib import Path
import struct
import zlib
import xml.etree.ElementTree as ET

ROOT = Path.cwd()
MUNDO = ROOT / 'tools/generar_mundo_f1.py'
ENTORNO = ROOT / 'tools/activar_entorno.sh'
RUTA = ROOT / 'modelos/senales_almacen'

if not MUNDO.is_file() or not ENTORNO.is_file() or not (ROOT/'config/escenario_f1.yaml').is_file():
    raise SystemExit('[ERROR] Ejecutar desde la raiz del repositorio AMR.')

world = MUNDO.read_text(encoding='utf-8')
entorno = ENTORNO.read_text(encoding='utf-8')

# Asegurar que no se duplique ni se sobreescriba una implementación previa.
if 'material_script=' in world or 'GAZEBO_MODEL_PATH' in entorno and 'senales_almacen' in world:
    raise SystemExit('[ALTO] R05 ya parece aplicado; no se sobrescribe.')

cambios = [
    (
        'def crear_modelo(nombre, xyz, forma, dimensiones,\n                 color, colision=True):',
        'def crear_modelo(nombre, xyz, forma, dimensiones,\n                 color, colision=True, material_script=None):'
    ),
    (
        '''        if tipo == "visual":
            material = ET.SubElement(nodo, "material")
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color''',
        '''        if tipo == "visual":
            material = ET.SubElement(nodo, "material")
            if material_script is not None:
                script = ET.SubElement(material, "script")
                ET.SubElement(script, "uri").text = (
                    "model://senales_almacen/materials/scripts"
                )
                ET.SubElement(script, "uri").text = (
                    "model://senales_almacen/materials/textures"
                )
                ET.SubElement(script, "name").text = material_script
            else:
                ET.SubElement(material, "ambient").text = color
                ET.SubElement(material, "diffuse").text = color'''
    ),
    (
        '''        (0.04, 0.90, 0.72),
        color,
        colision=False
    )''',
        '''        (0.04, 0.90, 0.72),
        color,
        colision=False,
        material_script=f"AMR/{identificador.upper()}"
    )'''
    ),
]
for old, new in cambios:
    if world.count(old) != 1:
        raise SystemExit('[ERROR] Estructura del generador inesperada; no se han aplicado cambios.')
for old, new in cambios:
    world = world.replace(old, new, 1)

if 'GAZEBO_MODEL_PATH' not in entorno:
    entorno += '''\n# R05 - Recursos OGRE de paneles con texturas para Gazebo Classic.
_AMR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GAZEBO_MODEL_PATH="${_AMR_ROOT}/modelos${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"
unset _AMR_ROOT
'''
else:
    # Otra personalizacion anterior: evitar reemplazarla a ciegas.
    raise SystemExit('[ALTO] activar_entorno.sh ya configura GAZEBO_MODEL_PATH; revisar manualmente.')

# Generación de PNG válidos 512 x 512 (sin dependencias externas).
# El material tiene un pictograma numérico visible en la textura.
COLORES = {
    'recepcion': ((20, 73, 214), 1),
    'almacenamiento': ((197, 44, 54), 2),
    'empaque': ((31, 154, 68), 3),
    'despacho': ((222, 160, 18), 4),
}
DIGITOS = {
    1: ['00100', '01100', '00100', '00100', '00100', '00100', '01110'],
    2: ['01110', '10001', '00001', '00010', '00100', '01000', '11111'],
    3: ['11110', '00001', '00001', '01110', '00001', '00001', '11110'],
    4: ['00010', '00110', '01010', '10010', '11111', '00010', '00010'],
}

def png_rgba(color, digit, size=512):
    w = h = size
    cr, cg, cb = color
    pix = bytearray(w*h*4)
    for y in range(h):
        for x in range(w):
            # Fondo de estación, borde blanco y número blanco centrado.
            border = x<23 or x>=w-23 or y<23 or y>=h-23
            near_border = (x<31 or x>=w-31 or y<31 or y>=h-31)
            # Número de fuente matricial 5 x 7, celdas 54 x 54.
            xx, yy = (x-121)//54, (y-67)//54
            number = (0<=xx<5 and 0<=yy<7 and DIGITOS[digit][yy][xx]=='1'
                      and 121<=x<121+270 and 67<=y<67+378)
            band = (64<=y<73 and 85<=x<w-85)
            if border or number or band:
                r,g,b = (250,250,250)
            elif near_border:
                r,g,b = (15,20,28)
            else:
                r,g,b = (cr,cg,cb)
            i=(y*w+x)*4
            pix[i:i+4]=bytes((r,g,b,255))
    data=b''.join(b'\x00'+pix[y*w*4:(y+1)*w*4] for y in range(h))
    def chunk(kind, payload):
        return (struct.pack('!I',len(payload))+kind+payload+
                struct.pack('!I',zlib.crc32(kind+payload)&0xffffffff))
    return (b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',w,h,8,6,0,0,0))+
            chunk(b'IDAT',zlib.compress(data,9))+chunk(b'IEND',b''))

# Crear archivos solo cuando se haya comprobado el patrón de actualización.
(RUTA/'materials/scripts').mkdir(parents=True,exist_ok=True)
(RUTA/'materials/textures').mkdir(parents=True,exist_ok=True)

materiales=[]
for nombre,(rgb,num) in COLORES.items():
    p=RUTA/'materials/textures'/f'{nombre}.png'
    if p.exists():
        raise SystemExit(f'[ALTO] Ya existe una textura R05: {p}')
    p.write_bytes(png_rgba(rgb,num))
    materiales.append(f'''material AMR/{nombre.upper()}
{{
  receive_shadows off
  technique
  {{
    pass
    {{
      lighting off
      texture_unit
      {{
        texture {nombre}.png
      }}
    }}
  }}
}}''')

(RUTA/'materials/scripts/estaciones.material').write_text('\n\n'.join(materiales)+'\n',encoding='utf-8')
(RUTA/'model.config').write_text('''<?xml version="1.0"?>
<model>
  <name>senales_almacen</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>Proyecto AMR</name></author>
  <description>Recursos visuales de las cuatro estaciones.</description>
</model>
''',encoding='utf-8')
(RUTA/'model.sdf').write_text('''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="senales_almacen">
    <static>true</static>
    <link name="placa">
      <visual name="visual">
        <geometry><box><size>0.04 0.9 0.72</size></box></geometry>
      </visual>
    </link>
  </model>
</sdf>
''',encoding='utf-8')
ET.parse(RUTA/'model.config')
ET.parse(RUTA/'model.sdf')
MUNDO.write_text(world,encoding='utf-8')
ENTORNO.write_text(entorno,encoding='utf-8')
print('[OK] 4 PNG de textura creados (512 x 512).')
print('[OK] 4 materiales OGRE referenciados mediante model://.')
print('[OK] Generador SDF adaptado; permanecen los 26 modelos sin añadir colisiones.')
print('[OK] GAZEBO_MODEL_PATH incorporado a activar_entorno.sh.')
print('SIGUIENTE: regenerar mundo y realizar inspeccion en Gazebo Classic.')
