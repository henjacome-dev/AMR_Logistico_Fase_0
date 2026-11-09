#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Avance 2 / C2 — Línea base operacional B3, solo lectura.

Analiza exactamente seis pruebas existentes:
  - B3 v01 fallido (línea base, N=1).
  - B3 v02 híbrido asistido (N=5).

Produce indicadores de corrida completa y de una ventana normalizada de
0.60 m de recorrido odométrico tras la inserción de la caja. La ventana común
NO transforma el conjunto en A/B pareado; las políticas de control difieren.

No inicia ROS, Gazebo ni bibliotecas de robótica. No modifica código, mundos,
rosbags ni resultados originales. Python 3.8+ (solo biblioteca estándar).

Uso WSL:
    python3 /mnt/c/Users/Henry/Downloads/A2_C2_linea_base_operacional_v01.py
    python3 A2_C2_linea_base_operacional_v01.py --raiz /ruta/a/20261108_171723
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path

RUNS = [
    ('ANTES_V01', 'B3_obstaculo_sorpresa', '20261108_192430'),
    ('DESPUES_V02', 'B3_obstaculo_sorpresa_v02', '20261108_195247'),
    ('DESPUES_V02', 'B3_obstaculo_sorpresa_v02', '20261108_195643'),
    ('DESPUES_V02', 'B3_obstaculo_sorpresa_v02', '20261108_195749'),
    ('DESPUES_V02', 'B3_obstaculo_sorpresa_v02', '20261108_195855'),
    ('DESPUES_V02', 'B3_obstaculo_sorpresa_v02', '20261108_195957'),
]
HORIZON_M = 0.60
SAFETY_M = 0.255
CRITICAL_SCAN_M = 0.35
SENSOR_MISMATCH_DF_M = 0.80
MODE_NAMES = {'RUTA', 'DIFUSO', 'CERO'}


def require(ok, message):
    if not ok:
        raise ValueError(message)


def finite(value, field, path, required=True):
    if value is None or value == '':
        if required:
            raise ValueError('Falta %s en %s' % (field, path))
        return None
    try:
        x = float(value)
    except (TypeError, ValueError):
        if required:
            raise ValueError('%s no numerico: %r en %s' % (field, value, path))
        return None
    if not math.isfinite(x):
        if required:
            raise ValueError('%s no finito en %s' % (field, path))
        return None
    return x


def rounded(x, digits=5):
    return None if x is None else round(float(x), digits)


def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def load_csv(path, required, numeric_required, numeric_optional=()):
    require(path.is_file(), 'No existe: %s' % path)
    with path.open(encoding='utf-8-sig', newline='') as f:
        rdr = csv.DictReader(f)
        require(set(required).issubset(set(rdr.fieldnames or [])),
                'Cabeceras invalidas en %s: %s' % (path, rdr.fieldnames))
        rows = []
        last_t = -math.inf
        for row in rdr:
            for k in numeric_required:
                row[k] = finite(row[k], k, path)
            for k in numeric_optional:
                row[k] = finite(row.get(k), k, path, required=False)
            t = row['t_s']
            require(t >= last_t - 1e-8,
                    'Tiempo no monotono en %s: %s < %s' % (path, t, last_t))
            last_t = t
            rows.append(row)
    require(len(rows) >= 10, 'Insuficientes filas en %s: %d' % (path, len(rows)))
    return rows


def interpolate_odom_at(odom, t):
    """Posición/holgura interpoladas al instante de aparición de la caja."""
    require(odom[0]['t_s'] <= t <= odom[-1]['t_s'],
            'Instante de spawn no cubierto por odometria: t=%.3f [%.3f,%.3f]' %
            (t, odom[0]['t_s'], odom[-1]['t_s']))
    for i, item in enumerate(odom):
        if item['t_s'] == t or i == 0 and item['t_s'] >= t:
            return {'t_s': t,
                    'x_m': item['x_m'], 'y_m': item['y_m'],
                    'despeje_geometrico_m': item['despeje_geometrico_m']}
        if item['t_s'] > t:
            prev = odom[i-1]
            k = (t-prev['t_s'])/(item['t_s']-prev['t_s'])
            return {'t_s':t,
                    'x_m':prev['x_m']+(item['x_m']-prev['x_m'])*k,
                    'y_m':prev['y_m']+(item['y_m']-prev['y_m'])*k,
                    'despeje_geometrico_m':prev['despeje_geometrico_m']+
                       (item['despeje_geometrico_m']-prev['despeje_geometrico_m'])*k}
    return {k:odom[-1][k] for k in
            ('t_s','x_m','y_m','despeje_geometrico_m')}


def distance_window(odom, spawn_t, horizon):
    """Devuelve punto final interpolado a la misma distancia odométrica."""
    origin = interpolate_odom_at(odom, spawn_t)
    previous = origin
    length = 0.0
    minimum = origin['despeje_geometrico_m']
    max_route_error = 0.0
    points_in_window = 1
    for row in odom:
        if row['t_s'] <= spawn_t:
            continue
        segment = math.hypot(row['x_m']-previous['x_m'], row['y_m']-previous['y_m'])
        # Las trayectorias publicadas deben ser densas: rechazar saltos atípicos.
        require(segment < 0.35,
                'Salto odometrico > 0.35 m: no se extrapolan ventanas C2')
        if length+segment >= horizon and segment > 0:
            alpha = max(0.0, min(1.0, (horizon-length)/segment))
            end_t = previous['t_s']+alpha*(row['t_s']-previous['t_s'])
            clearance = previous['despeje_geometrico_m']+alpha*(
                row['despeje_geometrico_m']-previous['despeje_geometrico_m'])
            minimum = min(minimum, clearance)
            require(end_t > spawn_t, 'Ventana C2 nula en odometria')
            return {'inicio_s':rounded(spawn_t,4),'fin_s':rounded(end_t,4),
                    'duracion_s':rounded(end_t-spawn_t,4),
                    'recorrido_ventana_m':horizon,
                    'despeje_min_ventana_m':rounded(minimum,5),
                    'puntos_odom_hasta_ventana':points_in_window,
                    'error_max_ruta_m_muestras':rounded(max_route_error,5)}
        length += segment
        minimum = min(minimum, row['despeje_geometrico_m'])
        max_route_error = max(max_route_error, abs(row['error_ruta_m']))
        previous = row
        points_in_window += 1
    raise ValueError('Una corrida no cubre %.2f m despues del spawn (cubrio %.3f m); '
                     'no puede entrar en la comparacion por ventana' % (horizon,length))


def interval_metrics(commands, start, end):
    """Métricas sobre comandos efectivos publicados en ventana [start,end].

    Cada comando se supone retenido hasta el siguiente registro; cuando no hay
    un comando antes de start, NO se estima el valor inicial.
    """
    relevant = [r for r in commands if start <= r['t_s'] <= end]
    previous = [r for r in commands if r['t_s'] <= start]
    require(previous and relevant, 'Controles no cubren la ventana comun')
    require(commands[-1]['t_s'] >= end-0.15,
            'Log de control termina antes del final de la ventana')
    active = previous[-1]
    timeline = [dict(active, t_s=start)]
    timeline.extend(r for r in relevant if r['t_s']>start)
    # Filtrar entradas de igual sello temporal: la ultima lectura prevalece.
    cleaned = []
    for row in timeline:
        if cleaned and abs(row['t_s']-cleaned[-1]['t_s'])<1e-9:
            cleaned[-1]=row
        else:
            cleaned.append(row)
    durations = {k:0.0 for k in MODE_NAMES}
    sum_v = sum_w = 0.0
    stopped = 0.0
    angular_variation = 0.0
    flips = 0
    critical = mismatch = assist = 0
    previous_w = None
    previous_mode = None
    mode_flips = 0
    for i,row in enumerate(cleaned):
        mode = row['modo_a3'].strip()
        require(mode in MODE_NAMES, 'Modo A3 desconocido: '+str(mode))
        t1 = min(end, cleaned[i+1]['t_s']) if i+1<len(cleaned) else end
        delta = max(0.0, t1-row['t_s'])
        w = row['omega_radps']
        v = row['v_mps']
        durations[mode] += delta
        sum_v += abs(v)*delta
        sum_w += abs(w)*delta
        if abs(v)<1e-5:
            stopped += delta
        if previous_w is not None:
            angular_variation += abs(w-previous_w)
            if abs(w)>0.08 and abs(previous_w)>0.08 and w*previous_w<0:
                flips += 1
        previous_w=w
        if previous_mode is not None and mode!=previous_mode:
            mode_flips += 1
        previous_mode=mode
        scan = row['scan_frontal_m']
        df = row['df_m']
        if (mode=='RUTA' and scan is not None and scan<=CRITICAL_SCAN_M):
            critical+=1
            if df is not None and df>=SENSOR_MISMATCH_DF_M:
                mismatch+=1
        if row['razon']=='B3_EVASION_HIBRIDA_ASISTIDA':
            assist+=1
    duration = end-start
    require(duration>0.01, 'Ventana temporal insuficiente')
    return {
        'n_comandos_ventana':len(cleaned),
        'velocidad_abs_media_ponderada_mps':rounded(sum_v/duration,6),
        'giro_abs_medio_ponderado_radps':rounded(sum_w/duration,6),
        'variacion_angular_total_radps':rounded(angular_variation,5),
        'variacion_angular_normalizada_por_s':rounded(angular_variation/duration,5),
        'inversiones_signo_omega':flips,
        'cambios_modo_a3_muestreados':mode_flips,
        'tiempo_ruta_s':rounded(durations['RUTA'],5),
        'tiempo_difuso_s':rounded(durations['DIFUSO'],5),
        'tiempo_cero_s':rounded(durations['CERO'],5),
        'porcentaje_tiempo_difuso':rounded(100*durations['DIFUSO']/duration,3),
        'porcentaje_tiempo_v_cero':rounded(100*stopped/duration,3),
        'comandos_ruta_con_scan_critico':critical,
        'comandos_ruta_con_sensor_discrepante':mismatch,
        'comandos_asistencia_geom':assist,
    }


def full_control_metrics(cmds):
    switches = sum(a['modo_a3']!=b['modo_a3'] for a,b in zip(cmds,cmds[1:]))
    critical = [r for r in cmds if r['modo_a3']=='RUTA' and
                r['scan_frontal_m'] is not None and r['scan_frontal_m']<=CRITICAL_SCAN_M]
    mismatch = sum(r['df_m'] is not None and r['df_m']>=SENSOR_MISMATCH_DF_M for r in critical)
    assisted = sum(r['razon']=='B3_EVASION_HIBRIDA_ASISTIDA' for r in cmds)
    return {'n_comandos':len(cmds),'cambios_modo_a3_muestras':switches,
            'muestras_ruta_scan_critico':len(critical),
            'muestras_ruta_scan_critico_df_alto':mismatch,
            'comandos_asistencia_geom':assisted}


def load_run(root, phase, kind, run_id):
    directory = root/kind/run_id
    summary = directory/'resumen.json'
    require(summary.is_file(), 'Falta resumen: '+str(summary))
    report = json.loads(summary.read_text(encoding='utf-8'))
    require(isinstance(report,dict),'Resumen no es objeto: '+str(summary))
    require(report.get('obstaculo_dinamico_insertado') is True,
            'No se confirma aparicion de caja: '+str(summary))
    event = report.get('obstaculo_evento')
    require(isinstance(event,dict) and event.get('respuesta_gazebo'),
            'No se confirma spawn real: '+str(summary))
    spawn_t = finite(event.get('t_s'),'obstaculo_evento.t_s',summary)
    require(0.0<spawn_t<180.0,'Instante de insercion anómalo en '+str(summary))
    control = load_csv(directory/'control.csv',
        ('t_s','modo_a3','v_mps','omega_radps','razon','df_m','scan_frontal_m'),
        ('t_s','v_mps','omega_radps'),('df_m','scan_frontal_m'))
    odom = load_csv(directory/'odometria.csv',
        ('t_s','x_m','y_m','despeje_geometrico_m','error_ruta_m'),
        ('t_s','x_m','y_m','despeje_geometrico_m','error_ruta_m'))
    require(control[0]['t_s']<spawn_t<control[-1]['t_s'],
            'Control.csv no cubre instante de insercion en '+str(directory))
    window = distance_window(odom,spawn_t,HORIZON_M)
    window.update(interval_metrics(control,spawn_t,window['fin_s']))
    states = report.get('modos_a3') or []
    require(isinstance(states,list) and len(states)>=1,'Sin estados A3 en '+str(summary))
    modes = [str(e.get('modo')) for e in states if isinstance(e,dict)]
    require(modes and all(m in MODE_NAMES for m in modes),'Modos A3 invalidos en '+str(summary))
    a3_flips = sum(x!=y for x,y in zip(modes,modes[1:]))
    a3_diff = sum(m=='DIFUSO' and (i==0 or modes[i-1]!='DIFUSO') for i,m in enumerate(modes))
    a3_returns = sum(i>0 and m=='RUTA' and modes[i-1]=='DIFUSO' for i,m in enumerate(modes))
    min_clear = finite(report.get('distancia_minima_a_colisiones_sdf_m'),'min_clear',summary)
    goal_error = finite(report.get('distancia_final_meta_m'),'goal_error',summary)
    distance = finite(report.get('distancia_recorrida_m'),'recorrido',summary)
    require(min_clear>=0 and goal_error>=0 and distance>=0,'Metricas fisicas invalidas en '+str(directory))
    bar = report.get('barreras') or {}
    failures = report.get('fallos') or []
    require(isinstance(bar,dict) and isinstance(failures,list),
            'Barreras/fallos invalidos en '+str(directory))
    projected_collision = ('COLISION_PROYECTADA' in failures or
                           int(bar.get('COLISION_PROYECTADA',0))>0)
    arrival = (phase=='DESPUES_V02' and
               report.get('evasion_hibrida_asistida_demostrada') is True and
               report.get('bypass_completado') is True and
               not failures and goal_error<=0.15001)
    if phase=='ANTES_V01':
        require(report.get('estado')=='NO_APROBADO' and projected_collision,
                'La linea base no es el fallo B3 v01 identificado en C1')
        require(not arrival,'La corrida ANTES no debe contabilizar llegada')
    else:
        require(arrival,'B3 v02 no coincide con ensayo asistido exitoso: '+str(directory))
    bags = [p for p in (directory/'rosbag2').glob('*.db3') if p.is_file() and p.stat().st_size>4096]
    require(bags,'Falta rosbag2 *.db3: '+str(directory))
    source_files = [summary,directory/'control.csv',directory/'odometria.csv']
    full = full_control_metrics(control)
    total_time = max(control[-1]['t_s'],odom[-1]['t_s'])
    # Inicio/fin medidos en registros de callbacks, no reloj de pared reproducido.
    result = {
        'fase':phase,'ensayo':run_id,
        'estado':report.get('estado'),'llegada':int(arrival),
        'tipo_actuacion':('SIN_ASISTENCIA_GEOMETRICA_V01' if phase=='ANTES_V01'
                          else 'HIBRIDA_ASISTIDA_V02'),
        'instante_spawn_s':rounded(spawn_t,4),
        'duracion_registro_s':rounded(total_time,4),
        'duracion_desde_spawn_s':rounded(total_time-spawn_t,4),
        'recorrido_total_m':rounded(distance,5),
        'despeje_min_corrida_m':rounded(min_clear,5),
        'margen_sobre_barrera_m':rounded(min_clear-SAFETY_M,5),
        'error_final_meta_m':rounded(goal_error,5),
        'eventos_a3_cambios':a3_flips,
        'eventos_a3_entradas_difuso':a3_diff,
        'eventos_a3_retorno_ruta':a3_returns,
        'colision_proyectada':int(projected_collision),
        'detenciones_barrera_total':sum(int(v) for v in bar.values() if isinstance(v,(int,float))),
        'barreras_json':json.dumps(bar,ensure_ascii=False,sort_keys=True),
        'rosbag_db3_validado_presencia':len(bags),
        'directorio_origen':str(directory),
        **full,
        **{'ventana_'+k:v for k,v in window.items()},
    }
    return result, source_files, {'spawn':event,'barreras':bar,'fallos':failures,
                                  'politica':report.get('politica_arbitraje'),
                                  'mensajes_FIS':report.get('mensajes_controlador_difuso'),
                                  'eventos_arbitraje':report.get('eventos_arbitraje_b3',[]),
                                  'rosbag_reproducido':False,
                                  'contactos_fisicos_gazebo_medido':False}


def fmt(x,digits=3):
    return ('ND' if x is None else ('%.'+str(digits)+'f') % x)


def report_md(rows,created):
    before = rows[0]
    after = rows[1:]
    avg=lambda key:statistics.mean(float(r[key]) for r in after)
    minimum=lambda key:min(float(r[key]) for r in after)
    maximum=lambda key:max(float(r[key]) for r in after)
    lines = [
        '# C2 — Línea base y protocolo de comparabilidad operacional',
        '',
        '**Fecha de auditoría (UTC):** '+created,
        '**Escenario:** B3, obstáculo dinámico aparecido a ~1 m del robot. ',
        '**Alcance:** 1 ejecución B3 v01 detenida por barrera, 5 ejecuciones B3 v02 con navegación híbrida asistida.',
        '',
        '## Métricas de las corridas completas (descriptivas)',
        '',
        '| Indicador | B3 v01 (n=1) | B3 v02 (n=5) |',
        '|---|---:|---:|',
        '| Llegadas registradas | 0/1 | 5/5 |',
        '| Interrupción `COLISION_PROYECTADA` | '+str(before['colision_proyectada'])+' | '
           +str(sum(int(r['colision_proyectada']) for r in after))+' de 5 |',
        '| Distancia total recorrida (m) | '+fmt(before['recorrido_total_m'])+' | '+fmt(avg('recorrido_total_m'))+' (media) |',
        '| Distancia mínima geométrica (m) | '+fmt(before['despeje_min_corrida_m'],4)+' | '+fmt(avg('despeje_min_corrida_m'),4)+' (media por ensayo) |',
        '| Error final a meta (m) | '+fmt(before['error_final_meta_m'])+' | '+fmt(avg('error_final_meta_m'))+' (media) |',
        '| Cambios A3 (total en evento JSON) | '+str(before['eventos_a3_cambios'])+' | '+fmt(avg('eventos_a3_cambios'),2)+' (media) |',
        '',
        '**Advertencia:** los recorridos completos NO tienen el mismo alcance temporal ni espacial: v01 terminó prematuramente; las diferencias globales no representan un efecto causal.',
        '',
        '## Ventana descriptiva normalizada: 0,60 m recorridos desde el spawn de la caja',
        '',
        'Se interpola el instante en que cada ensayo acumula 0,60 m de desplazamiento **odometrico** desde la inserción real en Gazebo; esto no usa las distancias euclidianas del punto de partida.',
        '',
        '| Indicador ventana | B3 v01 | B3 v02: media [mín, máx] |',
        '|---|---:|---:|',
    ]
    common = [
        ('ventana_duracion_s','Duración de ventana (s)',2),
        ('ventana_despeje_min_ventana_m','Despeje geométrico mínimo (m)',4),
        ('ventana_velocidad_abs_media_ponderada_mps','|v| media ponderada (m/s)',4),
        ('ventana_giro_abs_medio_ponderado_radps','|ω| medio ponderado (rad/s)',4),
        ('ventana_variacion_angular_normalizada_por_s','Σ|Δω| / duración (rad/s², índice discreto)',3),
        ('ventana_porcentaje_tiempo_difuso','Tiempo A3 en DIFUSO (%)',2),
        ('ventana_porcentaje_tiempo_v_cero','Tiempo con velocidad lineal cero (%)',2),
        ('ventana_cambios_modo_a3_muestreados','Cambios A3 muestreados (conteo)',2),
        ('ventana_comandos_ruta_con_sensor_discrepante','Muestras RUTA con scan≤0,35 y df≥0,80',2),
        ('ventana_comandos_asistencia_geom','Comandos de asistencia geométrica',2),
    ]
    for key, label, digits in common:
        lines.append('| '+label+' | '+fmt(before[key],digits)+' | '+fmt(avg(key),digits)+
            ' ['+fmt(minimum(key),digits)+', '+fmt(maximum(key),digits)+'] |')
    lines.extend([
        '',
        '### Condiciones compartidas y variables de confusión',
        '',
        '- Coinciden el escenario B3, la posición inicial nominal, la meta, la ruta A* inicial de 2 m y el concepto de inserción dinámica a 1 m; las poses efectivas y el ruido de sensor pueden variar.',
        '- La ventana iguala **recorrido odométrico desde la inserción**, no condiciones sensoriales, tiempo, trayectorias ni órdenes de control.',
        '- **Intervención combinada:** v02 introduce retención de evasión, subobjetivos geométricos y giros asistidos; no es un cambio único aislado.',
        '- v01 es **una sola ejecución** y v02 cinco; no hay aleatorización, réplica baseline, asignación pareada ni control cruzado.',
        '- No inferir p-valores, significancia estadística, reducción causal de colisiones ni mayor rendimiento intrínseco de Mamdani.',
        '- El despeje es geométrico según SDF; **no hay medición directa de contactos físicos**. Los rosbags se verifican por existencia, no se reproducen.',
        '',
        '## Criterios para C3 y C4',
        '',
        '1. **C3:** aceptar los resultados anteriores solo como contraste descriptivo del sistema completo y justificar por qué el arbitraje original no conservaba la evasión.',
        '2. **C3 experimental (opcional, si la rúbrica exige mejora causal):** ejecutar múltiples réplicas v01 y v02 bajo las mismas semillas de ruido, condiciones de spawn y política de parada, sin reducir márgenes de seguridad. Si una variante incorpora subobjetivos, declarar exactamente esa intervención.',
        '3. **C4:** producir tabla antes/después con N, resultados, mecanismo, errores y reservas, sin llamar A/B pareado a este conjunto.',
        '4. Conservar intactos A3, A4, FIS y todos los archivos originales; mantener la evidencia CNN y el cierre documental de D.',
        '',
        '## Dictamen C2',
        '',
        '**LINEA_BASE_Y_METRICAS_OPERACIONALES_DEFINIDAS_CON_RESERVAS.** Establece una comparación descriptiva reproducible; todavía NO demuestra causalidad ni optimización aislada de la lógica difusa.',
        '',
        'Las seis rutas de origen y los hashes SHA256 están en `tabla_C2.csv` y `manifiesto_SHA256.csv`.',
    ])
    return '\n'.join(lines)+'\n'


def main(argv=None):
    ap = argparse.ArgumentParser(description='C2 B3: linea base y ventana comparable, SOLO LECTURA')
    default_root = (Path.home()/'AMR_Logistico_Fase_0'/'resultados'/
                    'A2_B2_rescate_geometria_astar'/'20261108_171723')
    ap.add_argument('--raiz',type=Path,default=default_root,
                    help='Carpeta que contiene B3_obstaculo_sorpresa y B3_obstaculo_sorpresa_v02')
    ap.add_argument('--salida',type=Path,default=None,
                    help='Carpeta NUEVA fuera del repositorio para archivos C2')
    args = ap.parse_args(argv)
    root = args.raiz.expanduser().resolve()
    require(root.is_dir(),'No se encontro carpeta base: '+str(root))
    source_files=[]
    rows=[]
    details={}
    for phase,kind,runid in RUNS:
        result,used,info=load_run(root,phase,kind,runid)
        rows.append(result);source_files.extend(used);details[runid]=info
    require(len(rows)==6 and sum(int(r['llegada']) for r in rows[1:])==5,
            'No se verificaron los seis experimentos esperados')
    stamp=dt.datetime.now(dt.timezone.utc).strftime('%Y%m%d_%H%M%S_UTC')
    project=Path.home()/'AMR_Logistico_Fase_0'
    out=(args.salida or Path.home()/'AMR_A2_entrega_local'/('C2_linea_base_'+stamp)).expanduser().resolve()
    require(not out.exists(),'La carpeta de salida ya existe y NO se sobrescribe: '+str(out))
    require(not (out==project or project in out.parents),
            'La salida NO debe ubicarse dentro del repositorio')
    out.mkdir(parents=True,exist_ok=False)
    fields=list(rows[0].keys())
    with (out/'tabla_C2.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=fields)
        writer.writeheader();writer.writerows(rows)
    with (out/'manifiesto_SHA256.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f)
        writer.writerow(['archivo_fuente','sha256'])
        for p in source_files:
            writer.writerow([str(p),sha256(p)])
    timestamp=dt.datetime.now(dt.timezone.utc).isoformat()
    doc={
        'etapa':'C2','fecha_utc':timestamp,
        'dictamen':'LINEA_BASE_OPERACIONAL_DEFINIDA_CON_RESERVAS',
        'escenario':'B3: obstaculo dinamico a 1 m; v01 vs v02',
        'n_antes':1,'n_despues':5,'horizonte_recorrido_comun_post_spawn_m':HORIZON_M,
        'barrera_geometrica_m':SAFETY_M,
        'ventana':'Distancia odometrica de 0.60 m posterior a insercion, con inicio y fin interpolados',
        'conclusiones':['B3 v01 detenida por COLISION_PROYECTADA; 0/1 llegadas',
                        'B3 v02 con asistencia geometrica 5/5 llegadas',
                        'Resultados no pareados; no atribuir mejora exclusivamente a Mamdani',
                        'No se midieron contactos fisicos ni reprodujeron los rosbags'],
        'filas':rows,'detalles':details,
        'fuentes':[{'archivo':str(p),'sha256':sha256(p)} for p in source_files],
    }
    (out/'dictamen_C2.json').write_text(json.dumps(doc,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    (out/'informe_C2.md').write_text(report_md(rows,timestamp),encoding='utf-8')
    # No medir un nuevo experimento: solo consolidar registros originales.
    b=rows[0];a=rows[1:]
    print('\n=== C2 — LINEA BASE OPERACIONAL B3 ===')
    print('ARCHIVOS ORIGINALES VERIFICADOS: 6 resumen.json + 6 control.csv + 6 odometria.csv')
    print('ROSBAGS: 6/6 con bases db3 presentes (NO reproducidos)')
    print('ANTES (v01): 0/1 llegadas | barrera COLISION_PROYECTADA:',b['colision_proyectada'],
          '| A3 cambios:',b['eventos_a3_cambios'])
    print('DESPUES (v02): %d/5 llegadas híbridas asistidas' % sum(int(r['llegada']) for r in a))
    print('DESPEJE TOTAL min: ANTES %.4f m | DESPUES media %.4f m' %
          (b['despeje_min_corrida_m'],statistics.mean(float(r['despeje_min_corrida_m']) for r in a)))
    print('VENTANA COMUN: %.2f m de recorrido odometrico posterior a la aparicion' % HORIZON_M)
    print('TIEMPO EN VENTANA: ANTES %.3f s | DESPUES media %.3f s' %
          (b['ventana_duracion_s'],statistics.mean(float(r['ventana_duracion_s']) for r in a)))
    print('GIRO ABS MEDIO EN VENTANA: ANTES %.4f | DESPUES media %.4f rad/s' %
          (b['ventana_giro_abs_medio_ponderado_radps'],
           statistics.mean(float(r['ventana_giro_abs_medio_ponderado_radps']) for r in a)))
    print('DICTAMEN C2: LINEA BASE OPERACIONAL DEFINIDA CON RESERVAS')
    print('LIMITACION: NO experimento pareado ni mejora aislada de Mamdani')
    print('SALIDA:',out)
    print('ARCHIVOS: informe_C2.md, tabla_C2.csv, dictamen_C2.json, manifiesto_SHA256.csv')
    return 0


if __name__=='__main__':
    try:
        sys.exit(main())
    except (ValueError,OSError,json.JSONDecodeError) as exc:
        print('[NO VALIDADO C2] '+str(exc),file=sys.stderr)
        sys.exit(2)
