#!/usr/bin/env python3
"""A1-ACT14: valida fuentes históricas y una repetición real de la orden A.
Solo librería estándar; NO lanza Gazebo ni altera fuentes. Los logs de la repetición
se guardan fuera de Git y nunca se infiere ausencia de colisiones sin contacto real.
"""
import argparse
import csv
import hashlib
import json
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FILES = {
    'resultados/F1_A_cierre_20261106/seguidor_orden_A.py': 'beae27fe69d3c9fa1ca3a4f3079057224a8ae7a00f4cb9285c2055ff2ccbf51c',
    'resultados/R04_astar_mapa_corregido/resultados/F12_orden_A.csv': '7c8d99a0c1cf27e703fac5f524b6fc515753281933a08fb3bb594692fb90f844',
    'resultados/F1_A_cierre_20261106/seguimiento_orden_A.csv': 'f1b7afb65ecd1bafa5602abcb0ffc3f36941b6d8f46b942f7737d2c9f4a0e9e8',
    'docs/F1_cierre_avance1_20261106.md': '3e5dc2342e5007f25464999a3075fcdb181d2d8b5ce234dee3eb6f5c90df2212',
    'launch/almacen_operativo_f1.launch.py': '6ecf9bcb2a8f2b25b063ea14447f23e1be3380832dc34ba1bd0f77c07a4f0ad0',
    'tools/supervisor_f13.py': 'f883589ecaa6026d8e2f9f32115e5ea74efc7eee76a294e6a9828bf20fa27bba',
    'tools/emulador_sensores_f1.py': '88ffc3afca5b638b7570d4d284d464d604018a1f0e70895a78f827e0c4d15e41',
    'modelos/almacen_f1.world': 'ae7084f4810f84a22b38499a28205beae7fd91ce36351831aba1dc1ccd1f02e0',
    'resultados/F1_A_cierre_20261106/ejecucion.log': 'b49236aaba2b16ef5155ac738a032a9c12d3a9977dbb2e2e12d4388446b304d4',
}
STATIONS = [('Recepción',(-3.975,-2.775)), ('Almacenamiento',(-3.975,2.825)),
            ('Empaque',(4.025,2.825)), ('Despacho',(4.025,-2.775))]


def insist(condition, message):
    if not condition:
        raise RuntimeError(message)
    print('[OK] '+message)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows_of(path):
    with Path(path).open(encoding='utf-8-sig',newline='') as f:
        return list(csv.DictReader(f))


def route_length(rows):
    pts=[(float(r['x_m']),float(r['y_m'])) for r in rows]
    return sum(math.dist(a,b) for a,b in zip(pts[:-1],pts[1:]))


def check_static():
    for path,sha in FILES.items():
        f=ROOT/path
        insist(f.is_file(), 'Existe '+path)
        insist(sha256(f)==sha, 'SHA-256 histórico: '+path)
    route=rows_of(ROOT/'resultados/R04_astar_mapa_corregido/resultados/F12_orden_A.csv')
    insist(len(route)==405,'Ruta A del cierre F1: 405 puntos')
    insist(abs(route_length(route)-20.2)<0.005,'Longitud de ruta A: 20,200 m')
    insist(abs(float(route[0]['x_m'])+2.975)<1e-9,'Origen de ruta A: (-2.975,-2.775) m')
    insist(abs(float(route[-1]['x_m'])-4.025)<1e-9,'Destino de ruta A: (4.025,-2.775) m')
    baseline=rows_of(ROOT/'resultados/F1_A_cierre_20261106/seguimiento_orden_A.csv')
    insist(len(baseline)>2700,'Log histórico de la orden A disponible')
    text=(ROOT/'resultados/F1_A_cierre_20261106/ejecucion.log').read_text(encoding='utf-8')
    insist('Meta alcanzada' in text,'La ejecución histórica registró llegada')
    print('[REFERENCIA] 2026-11-06: 4 estaciones, 275,03 s, 0,0711 m de error final')
    print('[LÍMITE] La referencia NO fue parte del ZIP presentado el 4 de noviembre.')
    print('[RESULTADO] PREVALIDACIÓN A1-ACT14 APROBADA; aún NO se realizó una nueva simulación.')


def evaluate(run_dir):
    run_dir=Path(run_dir).expanduser().resolve()
    check_static()
    log=run_dir/'seguidor.log'
    path=run_dir/'resultados/ACT14_seguimiento_orden_A.csv'
    rc=run_dir/'seguidor_exit_code.txt'
    meta=run_dir/'metadata_ejecucion.json'
    for p in [log,path,rc,meta]:
        insist(p.is_file(), 'Disponible '+p.name)
    exit_code=int(rc.read_text(encoding='utf-8').strip())
    txt=log.read_text(encoding='utf-8',errors='replace')
    rows=rows_of(path)
    insist(exit_code==0,'Seguidor terminó con código 0')
    m=re.search(r'Meta alcanzada; error final\s+([0-9.]+)\s*m',txt)
    insist(m is not None,'Mensaje explícito de llegada al destino')
    error_reportado=float(m.group(1))
    insist(error_reportado<=0.080+0.001,'Error final del seguidor dentro de 0,08 m')
    insist(len(rows)>100,'Log ROS contiene muestras de odometría')
    t=[float(r['t_s']) for r in rows]
    xy=[(float(r['x_m']),float(r['y_m'])) for r in rows]
    insist(all(math.isfinite(q) for r in xy for q in r) and all(math.isfinite(q) for q in t),'Datos numéricos finitos')
    insist(all(a<=b+0.001 for a,b in zip(t[:-1],t[1:])),'Tiempo registrado no decreciente')
    index=0;distances=[];hit_times=[]
    for name,station in STATIONS:
        seg=xy[index:]
        assert seg
        d=[math.dist(x,station) for x in seg]
        j=min(range(len(d)),key=d.__getitem__)
        index+=j
        distances.append(d[j]);hit_times.append(t[index])
        insist(d[j]<=0.20, f'Estación {name}: paso a {d[j]:.3f} m')
    traveled=sum(math.dist(a,b) for a,b in zip(xy[:-1],xy[1:]))
    secs=t[-1]
    insist(secs<=720,'Ejecución dentro del máximo de 720 s del seguidor histórico')
    mdata=json.loads(meta.read_text(encoding='utf-8'))
    insist(mdata.get('baseline')=='F1_A_cierre_20261106','Procedencia de ejecución registrada')
    rosbag=run_dir/'rosbag'
    has_bag=rosbag.is_dir() and any(x.stat().st_size>0 for x in rosbag.glob('*.db3')) and (rosbag/'metadata.yaml').is_file()
    insist(has_bag, 'ROS bag SQLite3 y metadata.yaml finalizados en la carpeta privada')
    result={
        'actividad':'A1-ACT14', 'estado':'APROBADA', 'naturaleza':'Reejecución posterior del cierre F1 del 06-11-2026',
        'es_entrega_4nov':False,'tiempo_log_s':round(secs,3),'distancia_odometrica_m':round(traveled,4),
        'muestras':len(rows), 'error_final_m':error_reportado,
        'estaciones':[
            {'nombre':name,'proximidad_min_m':round(dist,4),'t_s':round(hit,3)}
            for (name,_),dist,hit in zip(STATIONS,distances,hit_times)
        ],
        'rosbag_completo_detectado':has_bag,
        'colisiones':'NO EVALUADAS: sin medición independiente de contactos',
        'fuentes_privadas':str(run_dir),
        'modelo':'TurtleBot3 Waffle Pi en Gazebo Classic',
    }
    (run_dir/'resumen_validacion.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    public=f'''# A1 — ACT14: Reproducción de la orden A\n\n**Estado: APROBADA en el ensayo posterior.**\n\nEsta evidencia corresponde a una nueva ejecución posterior del cierre F1 del 6 de noviembre de 2026. **No constituye una modificación ni una prueba retroactiva del ZIP presentado el 4 de noviembre.**\n\n| Indicador | Resultado |\n|---|---:|\n| Llegada reportada por el seguidor | Sí (código de salida 0) |\n| Estaciones visitadas | 4/4 en secuencia |\n| Longitud de ruta A* | 20,200 m; 405 puntos |\n| Tiempo del log de seguimiento | {secs:.3f} s (tiempo monotónico del proceso) |\n| Recorrido odométrico registrado | {traveled:.3f} m |\n| Error final reportado | {error_reportado:.3f} m |\n| Muestras del seguimiento | {len(rows)} |\n| ROS bag disponible localmente | {'Sí' if has_bag else 'No'} |\n| Contactos físicos | No verificados con sensor independiente |\n\n## Proximidad a las estaciones\n\n| Estación | Proximidad mínima (m) | Tiempo del log (s) |\n|---|---:|---:|\n'''
    for (name,_),dist,hit in zip(STATIONS,distances,hit_times):
        public+=f'| {name} | {dist:.4f} | {hit:.3f} |\n'
    public+='''\n**Trazabilidad:** referencia histórica `resultados/F1_A_cierre_20261106/`; protocolo `docs/A1_ACT14_PROTOCOLO_REPRODUCCION.md`. El ROS bag, el video y los logs íntegros permanecen fuera del repositorio público. El movimiento se realizó con seguidor F1 y supervisor, no con el controlador difuso ni la CNN.\n'''
    (run_dir/'A1_ACT14_RESULTADO_PUBLICABLE.md').write_text(public,encoding='utf-8')
    print('[RESULTADO] AUDITORÍA A1-ACT14 APROBADA; resumen preparado en '+str(run_dir/'A1_ACT14_RESULTADO_PUBLICABLE.md'))
    print('[LIMITACIÓN] La ausencia de colisiones NO se acredita sin sensor de contacto o inspección independiente.')


def main():
    ap=argparse.ArgumentParser()
    grp=ap.add_mutually_exclusive_group(required=True)
    grp.add_argument('--check',action='store_true',help='Valida fuentes sin movimiento')
    grp.add_argument('--evaluar',metavar='CARPETA',help='Valida una ejecución real posterior y genera resumen')
    args=ap.parse_args()
    try:
        if args.check:check_static()
        else:evaluate(args.evaluar)
    except Exception as ex:
        print('[ERROR] '+str(ex),file=sys.stderr)
        sys.exit(1)


if __name__=='__main__':main()
