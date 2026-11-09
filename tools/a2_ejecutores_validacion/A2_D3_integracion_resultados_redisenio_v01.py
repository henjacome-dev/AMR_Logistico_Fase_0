#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D3: Integra C4 y D1 auditados para redactar la evidencia del Avance 2.

No ejecuta ROS/Gazebo, no modifica Git ni los controladores, ni publica.
Uso:
  python3 A2_D3_integracion_resultados_redisenio_v01.py
  python3 A2_D3_integracion_resultados_redisenio_v01.py --c4 /ruta/C4_cierre_... --d1 /ruta/D1_auditoria_integral_...
"""
from __future__ import annotations
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

DICT_C4 = 'CIERRE_DESCRIPTIVO_HIBRIDO_ASISTIDO_CON_RESERVAS'
DICT_D1 = 'D1_EVIDENCIA_VERIFICADA_CON_RESERVAS'
REQUIRED_METRICS = ('despeje_min_corrida_m', 'ventana_duracion_s',
                    'ventana_giro_abs_medio_ponderado_radps')


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def sha256(path):
    hash_ = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            hash_.update(block)
    return hash_.hexdigest()


def num(v, label):
    try:
        f = float(v)
    except (TypeError, ValueError) as exc:
        raise ValueError('Dato no numerico: '+label) from exc
    require(math.isfinite(f), 'Dato no finito: '+label)
    return f


def folder_of(base, prefix, required_filename, selected):
    if selected is not None:
        folder = selected.expanduser().resolve()
    else:
        matches = [p for p in base.glob(prefix+'*') if p.is_dir() and (p/required_filename).is_file()]
        require(bool(matches), f'No se encontro {prefix}* en {base}; indica su ruta con --c4/--d1')
        folder = max(matches, key=lambda p: p.stat().st_mtime).resolve()
    require(folder.is_dir() and (folder/required_filename).is_file(),
            f'Falta dictamen en {folder}')
    return folder


def verify_c4(path):
    required = ['dictamen_C4.json', 'informe_C4.md', 'tabla_C4.csv',
                'reporte_visual_C4.html', 'manifiesto_SHA256.csv']
    for name in required:
        require((path/name).is_file(), 'Falta fuente C4: '+name)
    d = json.loads((path/'dictamen_C4.json').read_text(encoding='utf-8'))
    require(d.get('etapa')=='C4' and d.get('dictamen')==DICT_C4,
            'El dictamen C4 no es el cierre validado esperado')
    for key in ('comparacion_pareada','atribucion_aislada_Mamdani',
                'contactos_gazebo_verificados','rosbags_reproducidos'):
        require(d.get(key) is False, 'C4 no conserva reserva obligatoria: '+key)
    require(d.get('guia_geometrica_asistida') is True,
            'C4 debe declarar asistencia geometrica')
    for field, wanted in [('n_antes',1),('n_despues',5),('llegadas_antes',0),
                          ('llegadas_despues',5),('paradas_preventivas_antes',1),
                          ('paradas_preventivas_despues',0)]:
        require(d.get(field)==wanted, f'C4: valor inesperado de {field}')
    require(abs(num(d.get('barrera_m'),'barrera_m') - 0.255)<1e-8,
            'Se altero el umbral preventivo de C4')
    require(abs(num(d.get('horizonte_odometrico_comun_m'),'horizonte')-0.60)<1e-8,
            'La ventana comun no es 0.60 m')

    metrics = {f['id']: f for f in d.get('metricas', [])}
    for k in REQUIRED_METRICS:
        require(k in metrics, 'Metrica C4 ausente: '+k)
        f = metrics[k]
        old = num(f.get('antes_n1'), k+'.antes')
        new = num(f.get('despues_media_n5'), k+'.despues')
        require(abs(num(f.get('delta_absoluta'),k+'.delta')-(new-old))<1e-7,
                'Delta C4 inconsistente: '+k)
        if abs(old)>1e-9:
            calc=(new/old-1)*100
            require(abs(num(f.get('delta_porcentual'),k+'.pct')-calc)<0.05,
                    'Porcentaje C4 inconsistente: '+k)
    # Se comprueba la cadena de hashes creada por C4, incluidos C3 y fuentes originales.
    n = 0
    unique=set()
    with (path/'manifiesto_SHA256.csv').open(newline='', encoding='utf-8') as stream:
        reader=csv.DictReader(stream)
        require({'tipo','archivo','sha256'}.issubset(reader.fieldnames or []),
                'Esquema de manifiesto C4 no reconocido')
        for row in reader:
            source = Path(row['archivo'])
            require(source.is_file(), 'Archivo faltante en manifiesto C4: '+str(source))
            require(str(source) not in unique, 'Entrada de hash duplicada: '+str(source))
            unique.add(str(source))
            require(sha256(source)==row['sha256'], 'SHA256 no coincide: '+str(source))
            n+=1
    require(n>=28, 'Manifiesto C4 incompleto: solo '+str(n)+' entradas')
    return d, metrics, n


def verify_d1(path):
    for file in ('dictamen_D1.json','informe_D1.md','matriz_D1.csv',
                 'manifiesto_SHA256.csv','checklist_entrega_D1.md'):
        require((path/file).is_file(), 'Falta fuente D1: '+file)
    d=json.loads((path/'dictamen_D1.json').read_text(encoding='utf-8'))
    require(d.get('etapa')=='D1' and d.get('dictamen')==DICT_D1,
            'D1 no corresponde a la auditoria acreditada')
    require(d.get('ensayos_b1_b4_individualmente_acreditados')==20,
            'D1 no acredita 20 ensayos')
    require(d.get('por_escenario')=={k:5 for k in ('B1','B2','B3','B4')},
            'D1 no acredita cinco ejecuciones por escenario')
    require(all(d.get('cierres_C',{}).get(k) is True for k in ('C1','C2','C3','C4')),
            'D1 no valida los cuatro cierres C')
    require(not d.get('bloqueos'), 'D1 contiene bloqueos que impiden el cierre')
    require(d.get('rosbags_reproducidos') is False and
            d.get('contactos_gazebo_medidos') is False,
            'D1 no preserva reservas sobre rosbags/contactos')
    return d


def fmt(value, places=4):
    return f'{value:.{places}f}'.replace('.', ',')


def change(old,new):
    return f'{(new / old - 1)*100:+.2f}%'.replace('.', ',')


def build_text(metrics, c4, d1):
    sep=metrics['despeje_min_corrida_m']
    time=metrics['ventana_duracion_s']
    turn=metrics['ventana_giro_abs_medio_ponderado_radps']
    osep,nsep=num(sep['antes_n1'],'sep.old'),num(sep['despues_media_n5'],'sep.new')
    ot,nt=num(time['antes_n1'],'time.old'),num(time['despues_media_n5'],'time.new')
    ow,nw=num(turn['antes_n1'],'turn.old'),num(turn['despues_media_n5'],'turn.new')
    paragraph=(
        'Se identificó como debilidad principal la conmutación inestable entre RUTA y DIFUSO '
        'ante un obstáculo introducido durante el movimiento (B3). La configuración anterior '
        'terminó con una detención preventiva por COLISION_PROYECTADA (0/1 llegadas), '
        'mientras que la intervención híbrida asistida, con retención evasiva y subobjetivos '
        'geométricos temporales, completó cinco recorridos (5/5). El despeje geométrico mínimo '
        'por ejecución pasó de '+fmt(osep)+' a '+fmt(nsep)+' m de media; en una ventana común de '
        '0,60 m odométricos, el tiempo aumentó de '+fmt(ot)+' a '+fmt(nt)+' s ('+change(ot,nt)+') '
        'y el giro absoluto medio descendió de '+fmt(ow)+' a '+fmt(nw)+' rad/s ('+change(ow,nw)+'). '
        'La comparación es descriptiva (1 frente a 5 ensayos), no pareada, y no permite atribuir '
        'la mejora exclusivamente a Mamdani. No se midieron contactos físicos ni se reprodujeron los rosbags.'
    )
    return paragraph


def report(md_text, metrics, c4, d1, c4_dir, d1_dir, n_hash, stamp):
    se=metrics['despeje_min_corrida_m']
    ti=metrics['ventana_duracion_s']
    gi=metrics['ventana_giro_abs_medio_ponderado_radps']
    rows=[
        ('Llegadas documentadas','0/1','5/5','No comparable estadisticamente'),
        ('Paradas preventivas','1','0 observadas','Intervencion compuesta'),
        ('Despeje minimo por corrida',fmt(num(se['antes_n1'],'sep'))+' m',fmt(num(se['despues_media_n5'],'sep'))+' m (media)',
         fmt(num(se['delta_absoluta'],'sep'))+' m'),
        ('Tiempo, ventana comun 0,60 m',fmt(num(ti['antes_n1'],'time'))+' s',fmt(num(ti['despues_media_n5'],'time'))+' s',
         change(num(ti['antes_n1'],'t1'),num(ti['despues_media_n5'],'t2'))),
        ('Giro absoluto medio, ventana comun',fmt(num(gi['antes_n1'],'g1'))+' rad/s',
         fmt(num(gi['despues_media_n5'],'g2'))+' rad/s',
         change(num(gi['antes_n1'],'g1'),num(gi['despues_media_n5'],'g2'))),
    ]
    lines=[
        '# D3 — Integracion de resultados del rediseno operacional', '',
        '**Fecha UTC:** '+stamp,
        '**Estado:** C1–C4 integrados; dictamen descriptivo con reservas.',
        '**Procedencia:** dictamen C4 validado por SHA256 y auditoria D1 de 20/20 llegadas.',
        '', '## Diagnostico', '',
        'B3 v01 evidencio conmutaciones inestables RUTA/DIFUSO tras insertar una caja 1 m delante '
        'del robot, con detencion por COLISION_PROYECTADA antes de completar el recorrido. '
        'B3 v02 introdujo retencion de la evasión, parada/giro preventivo y desvio geometrico '
        'temporal. El FIS Mamdani participo, pero la maniobra no dependio exclusivamente de el.',
        '', '## Matriz consolidada de la intervencion', '',
        '| Indicador | Antes B3 v01 (n=1) | Despues B3 v02 (n=5) | Cambio |',
        '|---|---:|---:|---:|',
    ]
    for label,old,new,diff in rows:
        lines.append('| '+label+' | '+old+' | '+new+' | '+diff+' |')
    lines += [
        '', '## Texto listo para el informe de dos paginas (D4)', '', md_text,
        '', '## Alcance y reservas', '',
        '- B1–B4 acreditan 20/20 **llegadas experimentales** en D1, no cumplimiento integral por si solo.',
        '- B3 = hibrido asistido; B4 = centrado geometrico asistido con controladores en sombra.',
        '- El minimo de despeje anterior corresponde a una corrida; el posterior es media de cinco minimos.',
        '- Tiempo y giro se comparan en una ventana comun de 0,60 m odometricos desde la insercion.',
        '- No es un A/B pareado, aleatorizado ni una ablacion de Mamdani.',
        '- Los contactos fisicos de Gazebo no se midieron; los rosbags se verificaron por presencia, no por replay.',
        '- R21 (comparacion numerica entre FIS) constituye una evaluacion distinta y de resultados mixtos.',
        '', '## Trazabilidad', '',
        '- C4: `'+str(c4_dir/'dictamen_C4.json')+'`',
        '- D1: `'+str(d1_dir/'dictamen_D1.json')+'`',
        '- Registros verificados contra manifiesto C4: '+str(n_hash),
        '- D1: 20/20, bloqueos 0, revisiones manuales '+str(len(d1.get('revisiones_manuales',[])))+'.',
        '', '## Paso siguiente', '',
        'D4: incorporar el texto sintetico anterior y las metricas seleccionadas en el informe final '
        'de dos paginas. D3 no publica GitHub, crea tags ni envia Blackboard.', ''
    ]
    return '\n'.join(lines),rows


def main(args=None):
    cli=argparse.ArgumentParser(description='D3: integrar C4 + D1, sin modificar Git ni experimentos')
    cli.add_argument('--c4',type=Path,help='Carpeta C4 ya auditada')
    cli.add_argument('--d1',type=Path,help='Carpeta D1 ya auditada')
    cli.add_argument('--salida',type=Path,help='Nueva carpeta de destino fuera del repositorio')
    a=cli.parse_args(args)
    home=Path.home()
    base=home/'AMR_A2_entrega_local'
    require(base.is_dir() or (a.c4 is not None and a.d1 is not None),
            'No existe '+str(base)+'; usa --c4 y --d1 para indicar las fuentes')
    c4_path=folder_of(base,'C4_cierre_','dictamen_C4.json',a.c4)
    d1_path=folder_of(base,'D1_auditoria_integral_','dictamen_D1.json',a.d1)
    c4,metrics,n_hash=verify_c4(c4_path)
    d1=verify_d1(d1_path)
    utc=datetime.now(timezone.utc)
    timestamp=utc.isoformat()
    out=(a.salida or base/('D3_integracion_'+utc.strftime('%Y%m%d_%H%M%S_%f_UTC'))).expanduser().resolve()
    repo=(home/'AMR_Logistico_Fase_0').resolve()
    require(not out.exists(), 'No se sobreescribe: '+str(out))
    require(repo!=out and repo not in out.parents,'No escribir dentro del repositorio Git')
    require(out!=c4_path and c4_path not in out.parents and out!=d1_path and d1_path not in out.parents,
            'No escribir dentro de carpetas de evidencias auditadas')
    text=build_text(metrics,c4,d1)
    report_md,rows=report(text,metrics,c4,d1,c4_path,d1_path,n_hash,timestamp)
    out.mkdir(parents=True,exist_ok=False)
    (out/'informe_D3.md').write_text(report_md,encoding='utf-8')
    (out/'texto_para_D4.md').write_text('# Fragmento listo para D4 — Rediseño operacional\n\n'+text+'\n',encoding='utf-8')
    with (out/'matriz_D3.csv').open('w',newline='',encoding='utf-8') as stream:
        w=csv.writer(stream)
        w.writerow(('indicador','antes_n1','despues_n5','cambio'))
        w.writerows(rows)
    data={
        'etapa':'D3','fecha_utc':timestamp,'estado':'D3_INTEGRACION_DOCUMENTAL_CON_RESERVAS',
        'fuentes':{'C4':str(c4_path/'dictamen_C4.json'),'D1':str(d1_path/'dictamen_D1.json')},
        'fuentes_C4_sha256_verificadas':n_hash,'llegadas_B1_B4':20,
        'llegadas_B3_antes':0,'ensayos_B3_antes':1,
        'llegadas_B3_despues':5,'ensayos_B3_despues':5,
        'comparacion_pareada':False,'mejora_atribuible_solo_Mamdani':False,
        'contactos_gazebo_medidos':False,'rosbags_reproducidos':False,
        'reservas':'Evasion hibrida asistida; n=1 vs n=5; tiempo de ventana aumenta; giro medio disminuye; B4 centrado asistido.',
        'accion_siguiente':'D4, informe final de exactamente dos paginas'
    }
    (out/'dictamen_D3.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    sources=[c4_path/n for n in ('dictamen_C4.json','informe_C4.md','tabla_C4.csv','manifiesto_SHA256.csv')]
    sources += [d1_path/n for n in ('dictamen_D1.json','informe_D1.md','matriz_D1.csv','manifiesto_SHA256.csv')]
    outputs=[out/n for n in ('informe_D3.md','texto_para_D4.md','matriz_D3.csv','dictamen_D3.json')]
    with (out/'manifiesto_SHA256.csv').open('w',newline='',encoding='utf-8') as stream:
        w=csv.writer(stream)
        w.writerow(('rol','archivo','sha256'))
        for p in sources: w.writerow(('fuente',str(p),sha256(p)))
        for p in outputs: w.writerow(('resultado_D3',str(p),sha256(p)))
    print('=== D3 — INTEGRACION DOCUMENTAL DEL REDISENO ===')
    print('D1: 20/20 llegadas; C1-C4=OK; sin bloqueos criticos')
    print('C4: cadena SHA256 verificada, entradas:',n_hash)
    print('B3: 0/1 -> 5/5 llegadas, con asistencia geometrica')
    m=metrics
    for key,label,unit in [('despeje_min_corrida_m','DESPEJE','m'),
                           ('ventana_duracion_s','TIEMPO EN 0.60m','s'),
                           ('ventana_giro_abs_medio_ponderado_radps','GIRO ABS MEDIO','rad/s')]:
        v=m[key]; old=num(v['antes_n1'],label);new=num(v['despues_media_n5'],label)
        print(f'{label}: {old:.4f} -> {new:.4f} {unit}; cambio '+change(old,new))
    print('DICTAMEN: D3_INTEGRACION_DOCUMENTAL_CON_RESERVAS')
    print('SALIDA:',out)
    print('ARCHIVOS: informe_D3.md, texto_para_D4.md, matriz_D3.csv, dictamen_D3.json, manifiesto_SHA256.csv')
    print('NOTA: sin ROS/Gazebo, cambios en Git, tag ni envio Blackboard.')
    return 0

if __name__=='__main__':
    try:
        raise SystemExit(main())
    except (OSError,ValueError,KeyError,json.JSONDecodeError) as exc:
        print('[D3 NO VALIDADO]',exc,file=sys.stderr)
        raise SystemExit(2)
