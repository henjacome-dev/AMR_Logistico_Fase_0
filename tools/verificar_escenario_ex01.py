#!/usr/bin/env python3
"""EX-01: coherencia estatica de Gazebo, configuracion y mapas PGM."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

raiz = Path(__file__).resolve().parents[1]
cfg = yaml.safe_load((raiz / 'config/escenario_f1.yaml').read_text())
world = ET.parse(raiz / 'modelos/almacen_f1.world').getroot()
modelos = {m.attrib['name']: m for m in world.findall('./world/model')}
w, h = map(float, cfg['world']['size_m'])
t = float(cfg['world']['wall_thickness_m'])
res = float(cfg['map']['resolution_m'])
radio = float(cfg['map']['inflation_radius_m'])
W, H = round(w / res), round(h / res)
x0, y0 = -w / 2, -h / 2


def modelo(nombre, pos, tam=None, colision=True):
    m = modelos[nombre]
    pose = [float(v) for v in m.findtext('pose').split()[:3]]
    assert all(abs(a-b) < 1e-6 for a, b in zip(pose, pos)), f'Pose incorrecta: {nombre}'
    c = m.find('./link/collision')
    assert (c is not None) == colision, f'Colision incorrecta: {nombre}'
    if tam is not None:
        dimensiones = [float(v) for v in c.findtext('./geometry/box/size').split()]
        assert all(abs(a-b) < 1e-6 for a,b in zip(dimensiones, tam)), f'Tamano incorrecto: {nombre}'


paredes = {
    'pared_oeste': ((-w/2+t/2, 0, .8), (t,h,1.6)),
    'pared_este': ((w/2-t/2, 0, .8), (t,h,1.6)),
    'pared_sur': ((0,-h/2+t/2,.8), (w,t,1.6)),
    'pared_norte': ((0,h/2-t/2,.8), (w,t,1.6)),
}
for nombre, (pos, tam) in paredes.items():
    modelo(nombre,pos,tam)

rectangulos=[]
for obj in cfg['racks'] + cfg.get('obstacles',[]):
    x,y=map(float,obj['center_xy_m'])
    sx,sy=map(float,obj['size_xy_m'])
    z=float(obj['height_m'])
    modelo(obj['id'],(x,y,z/2),(sx,sy,z))
    rectangulos.append((x-sx/2,x+sx/2,y-sy/2,y+sy/2))

colores=cfg['stations']
for n, estacion in enumerate(colores,1):
    nombre=estacion['id'].lower()
    x,y=map(float,estacion['goal_xy_m'])
    modelo('estacion_'+nombre, (x,y,.008),colision=False)
    lado=-1 if x<0 else 1
    xp=lado*(w/2-t-.12)
    modelo('panel_'+nombre,(xp,y,.60),colision=False)
    for i in range(n):
        dy=(i-(n-1)/2)*.18
        modelo(f'panel_{nombre}_marca_{i+1}',(xp-lado*.037,y+dy,.62),colision=False)
assert len(modelos)==26, f'Se esperaban 26 modelos, hay {len(modelos)}'
print('[OK] Gazebo: paredes, estantes, obstaculos y 4 estaciones coinciden con YAML')
print('[OK] 4 paneles, 10 marcas, sin colisiones nuevas')


def leer_pgm(archivo):
    with archivo.open('rb') as f:
        assert f.readline().strip() == b'P5', f'Formato incorrecto: {archivo}'
        dimensiones=tuple(map(int, f.readline().split()))
        assert f.readline().strip()==b'255'
        datos=f.read()
    assert dimensiones==(W,H) and len(datos)==W*H, f'Dimensiones PGM incorrectas: {archivo}'
    return datos

base=bytearray([254])*(W*H)
for fila in range(H):
    y=y0+(H-fila-.5)*res
    for col in range(W):
        x=x0+(col+.5)*res
        pared=(x<=x0+t or x>=x0+w-t or y<=y0+t or y>=y0+h-t)
        ocupado=any(a<=x<=b and c<=y<=d for a,b,c,d in rectangulos)
        if pared or ocupado:
            base[fila*W+col]=0

inflado=bytearray(base)
n=math.ceil(radio/res)
offsets=[(dy,dx) for dy in range(-n,n+1) for dx in range(-n,n+1)
         if (dx*res)**2+(dy*res)**2<=radio**2+1e-10]
for indice, valor in enumerate(base):
    if valor:
        continue
    fila,col=divmod(indice,W)
    for dy,dx in offsets:
        fy,fx=fila+dy,col+dx
        if 0<=fy<H and 0<=fx<W:
            inflado[fy*W+fx]=0

for archivo, esperado in [('almacen_f1.pgm',base),('almacen_f1_inflado.pgm',inflado)]:
    real=leer_pgm(raiz/'mapas'/archivo)
    assert real==esperado, f'El mapa no coincide con la geometria declarada: {archivo}'
    print(f'[OK] {archivo}: {W}x{H}, coherencia celda a celda')

for estacion in cfg['stations']:
    x,y=map(float,estacion['goal_xy_m'])
    col=int((x-x0)/res)
    fila=H-1-int((y-y0)/res)
    assert inflado[fila*W+col]==254, f'Estacion bloqueada: {estacion["id"]}'
print('[OK] Las 4 metas siguen libres en el mapa inflado')

sur,norte=sorted(cfg['racks'],key=lambda r:r['center_xy_m'][1])
y_sur=float(sur['center_xy_m'][1]); hs=float(sur['size_xy_m'][1])/2
y_norte=float(norte['center_xy_m'][1]); hn=float(norte['size_xy_m'][1])/2
pasillos=[('sur',y0+t,y_sur-hs),('central',y_sur+hs,y_norte-hn),('norte',y_norte+hn,y0+h-t)]
for nombre, a, b in pasillos:
    ancho=b-a
    assert ancho>=1.0, f'Pasillo nominal menor de 1 m: {nombre}'
    print(f'[OK] Pasillo {nombre}: ancho nominal {ancho:.3f} m')
    for obj in cfg.get('obstacles',[]):
        yc=float(obj['center_xy_m'][1]); sy=float(obj['size_xy_m'][1])
        lo,hi=yc-sy/2,yc+sy/2
        if lo<b and hi>a:
            hueco=max(max(0,lo-a),max(0,b-hi))
            if hueco<1.0:
                print(f'[ADVERTENCIA] {nombre} junto a {obj["id"]}: paso lateral local maximo {hueco:.3f} m (<1.0 m)')
print('[EX-01] Consistencia estatica completada. Revisar advertencias de pasillos.')
