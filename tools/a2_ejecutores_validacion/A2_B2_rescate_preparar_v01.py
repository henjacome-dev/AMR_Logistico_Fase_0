#!/usr/bin/env python3
"""B2.3 rescate geometrico / A* - SIN modificar A3, A4, Mamdani, Git ni Gazebo.

Ejecutar en Ubuntu WSL:
    python3 /mnt/c/Users/Henry/Downloads/A2_B2_rescate_preparar_v01.py --preparar

No mueve el robot ni lanza procesos ROS. Solo prepara un experimento auditable.
"""
import argparse
import csv
import heapq
import json
import math
import re
import shutil
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

REPO = Path.home() / "AMR_Logistico_Fase_0"
START = (-1.975, -2.775)
GOAL = (-3.975, -2.775)
OBS_CENTER = (-3.475, -2.775)
OBS_SIZE = (0.20, 0.20, 0.42)
OBS_NAME = "a2_b2_obstaculo_estatico_centro"
YAW = math.pi
MIN_D_CENTROS = 1.20


def leer_pgm(p):
    with p.open("rb") as f:
        def token():
            while True:
                c = f.read(1)
                if not c:
                    raise ValueError("Cabecera PGM incompleta")
                if c == b"#":
                    f.readline()
                elif not c.isspace():
                    break
            s = bytearray(c)
            while True:
                c = f.read(1)
                if not c or c.isspace():
                    break
                s.extend(c)
            return s.decode("ascii")
        fmt, w, h, vmax = token(), int(token()), int(token()), int(token())
        if fmt != "P5" or vmax != 255:
            raise ValueError("Formato PGM no admitido; se requiere P5 de 8 bits")
        data = bytearray(f.read(w * h))
        if len(data) != w * h:
            raise ValueError("PGM incompleto")
    return w, h, data


def metadatos_mapa(p):
    txt = p.read_text(encoding="utf-8")
    def valor(clave):
        m = re.search(r"^\s*" + re.escape(clave) + r"\s*:\s*(.+?)\s*$", txt, flags=re.M)
        if not m:
            raise ValueError(f"Falta {clave} en {p}")
        return m.group(1).split("#")[0].strip()
    resol = float(valor("resolution"))
    origin = [float(x.strip()) for x in valor("origin").strip("[]").split(",")]
    if len(origin) != 3 or abs(origin[2]) > 1e-9:
        raise ValueError("Esta herramienta solo admite mapas sin rotación")
    return resol, origin, txt


def generar_world(original, salida):
    if original and original.is_file():
        tree = ET.parse(original)
        world = tree.getroot().find("world")
        obs = next((x for x in world.findall("model") if x.attrib.get("name") == OBS_NAME), None)
        if obs is None:
            raise ValueError("El mundo B2 encontrado no contiene el obstáculo esperado")
        pose = obs.findtext("pose", "").split()
        if len(pose) < 3 or math.hypot(float(pose[0])-OBS_CENTER[0], float(pose[1])-OBS_CENTER[1]) > 0.02:
            raise ValueError("El obstáculo del mundo B2 tiene coordenadas distintas; no se sobrescribe")
        shutil.copy2(original, salida)
        return "Mundo B2 original copiado y verificado (sin cambios)"
    base = REPO / "modelos" / "almacen_f1.world"
    if not base.is_file():
        raise FileNotFoundError(f"No existe mundo de referencia: {base}")
    tree = ET.parse(base)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("SDF sin etiqueta world")
    if any(x.get("name") == OBS_NAME for x in world.findall("model")):
        raise ValueError("El obstáculo ya está presente en el mundo base")
    # Construir el MISMO pasillo B2.1 (1.60 m) mas el obstaculo, no solo la caja.
    specs = [
        ("a2_b2_pared_norte", -3.475, -2.775 + .85, .35, 1.50, .10, .70),
        ("a2_b2_pared_sur", -3.475, -2.775 - .85, .35, 1.50, .10, .70),
        (OBS_NAME, OBS_CENTER[0], OBS_CENTER[1], OBS_SIZE[2]/2, *OBS_SIZE),
    ]
    for name, x, y, z, sx, sy, sz in specs:
        m = ET.SubElement(world, "model", name=name)
        ET.SubElement(m, "static").text = "true"
        ET.SubElement(m, "pose").text = f"{x} {y} {z} 0 0 0"
        link = ET.SubElement(m, "link", name="body")
        for typ in ("visual", "collision"):
            e = ET.SubElement(link, typ, name=typ)
            geo = ET.SubElement(e, "geometry")
            box = ET.SubElement(geo, "box")
            ET.SubElement(box, "size").text = f"{sx} {sy} {sz}"
    tree.write(salida, encoding="utf-8", xml_declaration=True)
    return "Obstáculo insertado en copia temporal del mundo base"


def leer_cajas_b2(mundo):
    world = ET.parse(mundo).getroot().find("world")
    if world is None:
        raise ValueError("Falta world en el SDF")
    nombres = {OBS_NAME, "a2_b2_pared_norte", "a2_b2_pared_sur"}
    cajas = []
    for m in world.findall("model"):
        name=m.attrib.get("name", "")
        if name not in nombres:
            continue
        xyz=list(map(float,m.findtext("pose", "").split()))
        sizes=[]
        for link in m.findall("link"):
            for coll in link.findall("collision"):
                size=coll.findtext("./geometry/box/size")
                if size:
                    sizes.append(list(map(float,size.split())))
        if len(xyz)!=6 or len(sizes)!=1 or len(sizes[0])!=3 or any(abs(x)>1e-8 for x in xyz[3:]):
            raise ValueError(f"Geometria B2 no compatible: {name}")
        cajas.append((name,xyz[0],xyz[1],sizes[0][0],sizes[0][1]))
    if {x[0] for x in cajas} != nombres or len(cajas)!=3:
        raise ValueError("Mundo B2 no incluye exactamente las dos paredes y el obstaculo")
    return cajas


def montar_mapa(original, w, h, res, origin, margen, cajas):
    ox, oy, _ = origin
    a = bytearray(original)
    for r in range(h):
        y = oy + (h - r - .5) * res
        for c in range(w):
            x = ox + (c + .5) * res
            for nombre, cx, cy, sx, sy in cajas:
                radio = margen if nombre==OBS_NAME else 0.21
                dx = max(abs(x-cx) - sx/2, 0)
                dy = max(abs(y-cy) - sy/2, 0)
                if math.hypot(dx, dy) <= radio + 1e-9:
                    a[r*w+c] = 0
                    break
    return a


def celda(x, y, w, h, res, origin):
    ox, oy, _ = origin
    return math.floor((x-ox)/res), h-1-math.floor((y-oy)/res)


def centro(c, r, h, res, origin):
    ox, oy, _ = origin
    return (ox + (c+.5)*res, oy + (h-r-.5)*res)


def planear(mapa, w, h, res, origin):
    inicio = celda(*START, w, h, res, origin)
    destino = celda(*GOAL, w, h, res, origin)
    def libre(p):
        c,r = p
        return 0 <= c < w and 0 <= r < h and mapa[r*w+c] >= 250
    if not libre(inicio):
        raise ValueError(f"Inicio ocupado: {inicio}")
    if not libre(destino):
        raise ValueError(f"Meta ocupada: {destino}")
    def heur(a):
        dx,dy=abs(a[0]-destino[0]),abs(a[1]-destino[1])
        return res*(max(dx,dy)+(math.sqrt(2)-1)*min(dx,dy))
    cont=0
    cola=[(heur(inicio),cont,0.,inicio)]
    g={inicio:0.}
    ant={}
    while cola:
        _,_,cost,p=heapq.heappop(cola)
        if cost > g[p]+1e-9:
            continue
        if p==destino:
            ruta=[p]
            while ruta[-1]!=inicio:
                ruta.append(ant[ruta[-1]])
            ruta.reverse()
            return [centro(*q, h, res, origin) for q in ruta],cost
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                if not (dx or dy):
                    continue
                q=(p[0]+dx,p[1]+dy)
                if not libre(q):
                    continue
                if dx and dy and (not libre((p[0]+dx,p[1])) or not libre((p[0],p[1]+dy))):
                    continue
                nc=cost+res*math.hypot(dx,dy)
                if nc+1e-9<g.get(q,math.inf):
                    g[q]=nc;ant[q]=p;cont+=1
                    heapq.heappush(cola,(nc+heur(q),cont,nc,q))
    raise ValueError("No existe ruta A* con este margen")


def segmento_libre(p,q,mapa,w,h,res,origin):
    largo=math.dist(p,q)
    n=max(1,math.ceil(largo/(res/3)))
    for j in range(n+1):
        t=j/n
        x=p[0]*(1-t)+q[0]*t
        y=p[1]*(1-t)+q[1]*t
        c,r=celda(x,y,w,h,res,origin)
        if not (0<=c<w and 0<=r<h and mapa[r*w+c]>=250):
            return False
    return True


def suavizar(ruta,mapa,w,h,res,origin):
    # Simplificacion con visibilidad, seguida de remuestreo para el seguidor A4.
    puntos=[ruta[0]]
    i=0
    while i<len(ruta)-1:
        for j in range(len(ruta)-1,i,-1):
            if segmento_libre(ruta[i],ruta[j],mapa,w,h,res,origin):
                break
        if j<=i:
            raise ValueError("Falló la simplificación de ruta")
        puntos.append(ruta[j]);i=j
    out=[puntos[0]]
    for p,q in zip(puntos[:-1],puntos[1:]):
        n=max(1,math.ceil(math.dist(p,q)/0.075))
        for k in range(1,n+1):
            t=k/n
            out.append((p[0]*(1-t)+q[0]*t,p[1]*(1-t)+q[1]*t))
    for p,q in zip(out[:-1],out[1:]):
        if not segmento_libre(p,q,mapa,w,h,res,origin):
            raise ValueError("Ruta suavizada cruza una celda ocupada")
    return out


def guardar_csv(path, puntos):
    with path.open("w",encoding="utf-8",newline="") as f:
        csvw=csv.writer(f,lineterminator="\n")
        csvw.writerow(("x_m","y_m"))
        for x,y in puntos:
            csvw.writerow((f"{x:.5f}",f"{y:.5f}"))


def guardar_svg(path, puntos, mapa, w, h, res, origin):
    # Vista superior: solo para comprobación visual, sin dependencias gráficas.
    ox,oy,_=origin
    def coords(p):
        x,y=p
        return ((x-ox)/res, h-(y-oy)/res)
    track=" ".join(f"{coords(p)[0]:.1f},{coords(p)[1]:.1f}" for p in puntos)
    dibujo=[f'<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="800" viewBox="0 0 {w} {h}">',
            '<rect width="100%" height="100%" fill="#f9f9f9"/>']
    for r in range(h):
        cc=0
        while cc<w:
            if mapa[r*w+cc]<250:
                c0=cc
                while cc<w and mapa[r*w+cc]<250: cc+=1
                dibujo.append(f'<rect x="{c0}" y="{r}" width="{cc-c0}" height="1" fill="#414b59"/>')
            else: cc+=1
    dibujo.append(f'<polyline points="{track}" fill="none" stroke="#ef6c22" stroke-width="1.7" stroke-linecap="round"/>')
    for p,col,nombre in ((START,"#10a37f","Inicio"),(GOAL,"#085ab0","Meta"),(OBS_CENTER,"#bf173b","Obstáculo")):
        x,y=coords(p)
        dibujo.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2.2" fill="{col}" stroke="white" stroke-width=".4"/>')
        dibujo.append(f'<text x="{x+3:.2f}" y="{y-2:.2f}" font-size="2.5" fill="{col}">{nombre}</text>')
    dibujo.append('</svg>')
    path.write_text("\n".join(dibujo),encoding="utf-8")


def main():
    global REPO
    arg=argparse.ArgumentParser()
    arg.add_argument("--preparar",action="store_true",help="prepara nuevo escenario sin tocar archivos originales")
    arg.add_argument("--repo",type=Path,default=REPO)
    a=arg.parse_args()
    REPO=a.repo.expanduser().resolve()
    if not a.preparar:
        arg.print_help();return 2
    mapa_base=REPO/"mapas/almacen_f1_inflado.pgm"
    mapa_yaml=REPO/"mapas/almacen_f1_inflado.yaml"
    for p in (mapa_base,mapa_yaml):
        if not p.is_file(): raise FileNotFoundError(f"No existe: {p}")
    w,h,base=leer_pgm(mapa_base)
    res,origin,_=metadatos_mapa(mapa_yaml)
    if (w,h)!=(200,160) or abs(res-.05)>1e-8:
        print("[ADVERTENCIA] Resolución/dimensiones distintas del escenario B2 registrado")
    d_centros=math.dist(START,OBS_CENTER)
    if d_centros < MIN_D_CENTROS:
        raise ValueError("Inicio demasiado cerca del obstáculo")
    salida=REPO/"resultados"/"A2_B2_rescate_geometria_astar"/datetime.now().strftime("%Y%m%d_%H%M%S")
    salida.mkdir(parents=True,exist_ok=False)
    candidatos=sorted((REPO/"resultados"/"A2_B2_obstaculo_estatico_centro").glob("*/almacen_B2_obstaculo_central_temporal.world"),reverse=True)
    world_original=candidatos[0] if candidatos else None
    descripcion=generar_world(world_original,salida/"B2_rescate_obstaculo_central.world")
    cajas=leer_cajas_b2(salida/"B2_rescate_obstaculo_central.world")
    # El margen 0.21 es el radio original. Se prueban márgenes mayores SOLO para el obstáculo B2.
    intentos=[]; elegido=None
    for margen in (0.34,0.28,0.21):
        mapa=montar_mapa(base,w,h,res,origin,margen,cajas)
        try:
            ruta_cruda,longitud=planear(mapa,w,h,res,origin)
            ruta=suavizar(ruta_cruda,mapa,w,h,res,origin)
            elegido=(margen,mapa,ruta_cruda,ruta,longitud)
            intentos.append({"margen_m":margen,"estado":"OK"});break
        except (ValueError,RuntimeError) as e:
            intentos.append({"margen_m":margen,"estado":"NO_VIABLE","causa":str(e)})
    if elegido is None:
        raise RuntimeError("No hay camino libre. Revisar geometría; no ejecutar el robot. " + repr(intentos))
    margen,mapa,cruda,ruta,longitud=elegido
    if any(abs(y-OBS_CENTER[1]) > 1.60/2 - 0.21 + 1e-8 for x,y in ruta):
        raise ValueError("Ruta sale del pasillo B2 de 1.60 m: no se entregará al seguidor")
    if not any(abs(x-OBS_CENTER[0])<0.026 and abs(y-OBS_CENTER[1])>0.30 for x,y in ruta):
        raise ValueError("No se demostró desvío lateral efectivo en el cruce del obstáculo")
    ruta_f=salida/"B2_rescate_ruta_astar.csv"
    guardar_csv(ruta_f,ruta)
    guardar_csv(salida/"B2_rescate_ruta_astar_cruda.csv",cruda)
    (salida/"B2_rescate_mapa_inflado.pgm").write_bytes(f"P5\n{w} {h}\n255\n".encode()+bytes(mapa))
    nuevo_yaml=re.sub(r"(?m)^\s*image:\s*.*$","image: B2_rescate_mapa_inflado.pgm",mapa_yaml.read_text())
    (salida/"B2_rescate_mapa_inflado.yaml").write_text(nuevo_yaml,encoding="utf-8")
    guardar_svg(salida/"B2_rescate_trayectoria.svg",ruta,mapa,w,h,res,origin)
    # Comprobación de separaciones y longitudes.
    mdist=math.inf
    for p in ruta:
        ddx=max(abs(p[0]-OBS_CENTER[0])-OBS_SIZE[0]/2,0)
        ddy=max(abs(p[1]-OBS_CENTER[1])-OBS_SIZE[1]/2,0)
        mdist=min(mdist,math.hypot(ddx,ddy))
    traj=sum(math.dist(p,q) for p,q in zip(ruta[:-1],ruta[1:]))
    minpared = min(
        math.hypot(max(abs(p[0]-cx)-sx/2,0),max(abs(p[1]-cy)-sy/2,0))
        for p in ruta for name,cx,cy,sx,sy in cajas if name != OBS_NAME
    )
    if minpared < 0.21-1e-8:
        raise ValueError("Ruta demasiado cerca de una pared B2")
    manifiesto={
        "estado":"PREPARADO_NO_EJECUTADO",
        "repo":str(REPO),"inicio_xy_m":START,"inicio_yaw_rad":YAW,"meta_xy_m":GOAL,
        "obstaculo":{"nombre":OBS_NAME,"centro_xy_m":OBS_CENTER,"tamano_xyz_m":OBS_SIZE},
        "distancia_inicial_centros_m":round(d_centros,4),
        "margen_obstaculo_b2_m":margen,"inflado_base_m":0.21,
        "origen_mapa":origin,"resolucion_m":res,
        "longitud_astar_crudo_m":round(longitud,4),"longitud_ruta_suavizada_m":round(traj,4),
        "dist_minima_ruta_a_superficie_obstaculo_m":round(mdist,4),
        "dist_minima_ruta_a_paredes_b2_m":round(minpared,4),"waypoints":len(ruta),
        "seleccion_margen":intentos,"descripcion_mundo":descripcion,
        "mundo_original":str(world_original) if world_original else None,
        "salida":str(salida),
        "instruccion":"Cargar EN CONJUNTO el mundo temporal, inicio/yaw del manifiesto y el nuevo CSV. No usar un launch con la posición inicial anterior.",
        "advertencia":"No se ha probado el piloto integrado Gazebo A3/A4 ni el arranque ROS. La validacion A* es estatica."
    }
    (salida/"B2_rescate_manifest.json").write_text(json.dumps(manifiesto,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print("[OK] B2 rescate: geometría y ruta A* PREPARADAS (sin Gazebo)")
    print("[OK] A3/A4/Mamdani/Git: sin modificaciones")
    print(f"[OK] Robot {START}; yaw={YAW:.5f}; obstáculo {OBS_CENTER}; meta {GOAL}")
    print(f"[OK] Separación inicial {d_centros:.2f} m; margen B2 elegido {margen:.2f} m")
    print(f"[OK] Ruta {len(ruta)} puntos, longitud {traj:.3f} m; despeje obstáculo {mdist:.3f} m y paredes {minpared:.3f} m")
    print(f"[SALIDA] {salida}")
    print(f"[RUTA] {ruta_f}")
    print("[SIGUIENTE] Antes de lanzar Gazebo, comprobar que el spawn use el nuevo inicio. No usar D10.")
    return 0

if __name__=="__main__":
    try:sys.exit(main())
    except Exception as e:
        print(f"[ERROR B2 RESCATE] {type(e).__name__}: {e}",file=sys.stderr)
        sys.exit(1)
