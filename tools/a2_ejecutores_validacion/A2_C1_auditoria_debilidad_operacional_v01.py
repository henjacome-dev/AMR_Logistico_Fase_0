#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""C1 Avance 2 — auditoría de la debilidad operacional observada en B3.

Lee EXCLUSIVAMENTE seis ensayos concretos ya ejecutados: B3 v01 fallido y
cinco B3 v02 híbridos asistidos. Nunca abre ROS/Gazebo, no modifica el repo,
no edita controladores ni altera las evidencias originales.

Ubuntu / WSL:
    python3 A2_C1_auditoria_debilidad_operacional_v01.py
    python3 A2_C1_auditoria_debilidad_operacional_v01.py --raiz /otra/ruta

Salida en ~/AMR_A2_entrega_local/C1_auditoria_YYYYmmdd_HHMMSS_UTC/:
    informe_C1.md, tabla_C1.csv, dictamen_C1.json, manifiesto_SHA256.csv

IMPORTANTE: una comparación de 1 fallo frente a 5 éxitos con asistencia
geométrica NO es una evaluación A/B operacional pareada ni demuestra mejora
exclusiva de Mamdani o A3. El objetivo C1 es identificar y medir el fallo.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import statistics
from pathlib import Path

RUNS = [
    ('ANTES', 'B3_obstaculo_sorpresa', '20261108_192430'),
    ('DESPUES_HIBRIDO', 'B3_obstaculo_sorpresa_v02', '20261108_195247'),
    ('DESPUES_HIBRIDO', 'B3_obstaculo_sorpresa_v02', '20261108_195643'),
    ('DESPUES_HIBRIDO', 'B3_obstaculo_sorpresa_v02', '20261108_195749'),
    ('DESPUES_HIBRIDO', 'B3_obstaculo_sorpresa_v02', '20261108_195855'),
    ('DESPUES_HIBRIDO', 'B3_obstaculo_sorpresa_v02', '20261108_195957'),
]
STOP_GEOM_M = 0.255
FIELDS = ['fase','ejecucion','estado','llegada','despeje_min_m','holgura_sobre_umbral_m',
          'error_final_meta_m','recorrido_m','modos_a3_cambios','entradas_difuso',
          'retornos_ruta','mandos_en_modo_ruta_con_scan_critico','mandos_ruta_con_df_alto_y_scan_critico',
          'mandos_asistencia_geometrica','evasion_hibrida_asistida','colision_proyectada',
          'rosbag_archivos_db3','directorio_origen']


def fail(msg):
    raise ValueError(msg)


def number(x, field, where):
    if x is None:
        fail(f'Falta {field} en {where}')
    try:
        a = float(x)
    except (TypeError, ValueError) as ex:
        raise ValueError(f'Valor no numérico {field}: {x!r} en {where}') from ex
    if not math.isfinite(a):
        fail(f'Valor no finito en {field} de {where}')
    return a


def optional_float(x):
    try:
        val = float(x)
        return val if math.isfinite(val) else None
    except (TypeError, ValueError):
        return None


def digest(file):
    h = hashlib.sha256()
    with file.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def audit(run: Path, phase: str):
    needed = [run/'resumen.json', run/'control.csv', run/'odometria.csv']
    for f in needed:
        if not f.is_file():
            fail(f'Falta archivo de evidencia. No se simula su contenido: {f}')
    try:
        report = json.loads(needed[0].read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as ex:
        raise ValueError(f'No se pudo leer JSON de {run}: {ex}') from ex
    if not isinstance(report, dict):
        fail(f'JSON inesperado en {run}')
    if report.get('obstaculo_dinamico_insertado') is not True:
        fail(f'No hay confirmación de caja insertada en {run}')
    event = report.get('obstaculo_evento') or {}
    if not (isinstance(event, dict) and event.get('respuesta_gazebo')):
        fail(f'No hay respuesta de spawn Gazebo registrada en {run}')

    min_clear = number(report.get('distancia_minima_a_colisiones_sdf_m'), 'despeje', run)
    target_error = number(report.get('distancia_final_meta_m'), 'error_meta', run)
    length = number(report.get('distancia_recorrida_m'), 'recorrido', run)
    if min_clear < 0 or target_error < 0 or length < 0:
        fail(f'Magnitudes imposibles en {run}')
    states = report.get('modos_a3') or []
    if not isinstance(states, list):
        fail(f'Lista de modos inválida en {run}')
    modes = [i.get('modo') for i in states if isinstance(i, dict)]
    if not modes or any(m not in ('RUTA','DIFUSO','CERO') for m in modes):
        fail(f'No se pudieron verificar los modos A3 en {run}')
    switches = sum(a != b for a,b in zip(modes, modes[1:]))
    dif = sum(m=='DIFUSO' and (i == 0 or modes[i-1] != 'DIFUSO') for i,m in enumerate(modes))
    returns = sum(m=='RUTA' and i>0 and modes[i-1]=='DIFUSO' for i,m in enumerate(modes))
    after_start = [i for i in states if isinstance(i,dict) and i.get('modo')=='DIFUSO']
    span = None
    if phase=='ANTES' and after_start and len(states)>1:
        first = optional_float(after_start[0].get('t_s'))
        last = optional_float(states[-1].get('t_s'))
        if first is not None and last is not None and last>first:
            span = {'primer_difuso_s':first,'ultimo_modo_s':last,'cambios_tras_primero':max(0,switches-1),
                    'frecuencia_cambios_hz':round((max(0,switches-1))/(last-first),3)}

    inconsistent = route_critical = assisted_commands = controls_total = 0
    min_scan_observed = math.inf
    with (run/'control.csv').open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        required = {'modo_a3','df_m','scan_frontal_m','v_mps','omega_radps','razon'}
        if not required.issubset(set(reader.fieldnames or [])):
            fail(f'Cabeceras control.csv inesperadas en {run}: {reader.fieldnames}')
        for row in reader:
            controls_total += 1
            mode = row['modo_a3'].strip()
            scan = optional_float(row['scan_frontal_m'])
            df = optional_float(row['df_m'])
            if scan is not None:
                min_scan_observed = min(scan, min_scan_observed)
            if mode == 'RUTA' and scan is not None and scan <= 0.35:
                route_critical += 1
                if df is not None and df >= 0.80:
                    inconsistent += 1
            if row.get('razon','') == 'B3_EVASION_HIBRIDA_ASISTIDA':
                assisted_commands += 1
    if controls_total < 10:
        fail(f'control.csv con menos de 10 muestras en {run}')
    # Verificar solo existencia del registro, no afirmar reproducción del rosbag.
    bag = run/'rosbag2'
    dbfiles = [p for p in bag.glob('*.db3') if p.is_file() and p.stat().st_size>4096]
    if not dbfiles:
        fail(f'Falta rosbag2 *.db3 con contenido en {run}')

    errors = report.get('fallos') or []
    barriers = report.get('barreras') or {}
    collision_projected = ('COLISION_PROYECTADA' in errors or barriers.get('COLISION_PROYECTADA',0)>0)
    success = (phase=='DESPUES_HIBRIDO' and report.get('evasion_hibrida_asistida_demostrada') is True
               and report.get('bypass_completado') is True and not errors
               and target_error <= 0.15001)
    if phase=='ANTES':
        if not collision_projected or target_error < 0.80:
            fail(f'La evidencia v01 no coincide con la prueba fallida identificada: {run}')
        if report.get('estado') != 'NO_APROBADO':
            fail(f'Se esperaba NO_APROBADO en v01, se leyó {report.get("estado")}')
    elif not success:
        fail(f'La evidencia v02 no cumple los criterios híbridos reportados: {run}')

    row = {
        'fase':phase, 'ejecucion':run.name, 'estado':str(report.get('estado','')), 'llegada':int(success),
        'despeje_min_m':round(min_clear,5),'holgura_sobre_umbral_m':round(min_clear-STOP_GEOM_M,5),
        'error_final_meta_m':round(target_error,5), 'recorrido_m':round(length,5),
        'modos_a3_cambios':switches,'entradas_difuso':dif,'retornos_ruta':returns,
        'mandos_en_modo_ruta_con_scan_critico':route_critical,
        'mandos_ruta_con_df_alto_y_scan_critico':inconsistent,
        'mandos_asistencia_geometrica':assisted_commands,
        'evasion_hibrida_asistida':int(report.get('evasion_hibrida_asistida_demostrada') is True),
        'colision_proyectada':int(collision_projected),'rosbag_archivos_db3':len(dbfiles),
        'directorio_origen':str(run)
    }
    notes = {'momento_obstaculo':event,
             'modos':states,
             'primera_ventana_conmutacion':span,
             'min_scan_registrado_m':round(min_scan_observed,5) if math.isfinite(min_scan_observed) else None,
             'paradas':barriers,
             'muestras_control':controls_total,
             'num_comandos_difuso':report.get('comandos_difuso_activos_despues_spawn'),
             'num_comandos_ruta_despues_difuso':report.get('comandos_ruta_despues_difuso'),
             'ciclos_difusos_a3_tras_spawn':report.get('ciclos_a3_difuso_tras_spawn'),
             'ciclos_asistencia_geometrica':report.get('comandos_evasion_asistida'),
             'intervenciones_v_cero':report.get('intervenciones_v_cero'),
             'ciclos_mezcla_difusa':report.get('ciclos_mezcla_difusa'),
             'eventos_arbitraje':report.get('eventos_arbitraje_b3',[]),
             'rosbag_reproducido':False,
             'colisiones_fisicas_medido':False}
    return row, notes, needed


def md_summary(rows, notes, created):
    before=rows[0]
    after=rows[1:]
    means=lambda key:statistics.mean(float(r[key]) for r in after)
    min_success_clear = min(r['despeje_min_m'] for r in after)
    count = sum(r['llegada'] for r in after)
    freq=notes[0].get('primera_ventana_conmutacion') or {}
    mismatch=int(before['mandos_ruta_con_df_alto_y_scan_critico'])
    return f'''# C1 — Diagnóstico de debilidad operacional (Avance 2)

**Fecha del análisis (UTC):** {created}  
**Dictamen:** Debilidad operacional principal observada: **retorno prematuro del supervisor a `RUTA` durante la aparición de un obstáculo dinámico**, con oscilación de modos `RUTA`/`DIFUSO`, comandos de orientación incompatibles y detención por la barrera `COLISION_PROYECTADA`.

## Evidencia del fallo original (B3 v01: un ensayo)

- Estado: `{before['estado']}`; llegada a meta: **NO**; error restante: **{before['error_final_meta_m']:.4f} m**.
- Despeje mínimo geométrico: **{before['despeje_min_m']:.4f} m** frente a barrera **{STOP_GEOM_M:.3f} m**; reserva final: **{before['holgura_sobre_umbral_m']:.4f} m**. Esta cifra es una estimación de geometría de SDF, no un sensor de contacto físico.
- Cambios de modo A3 después del estado inicial: **{before['modos_a3_cambios']}**; entradas a `DIFUSO`: **{before['entradas_difuso']}**; retornos de `DIFUSO` a `RUTA`: **{before['retornos_ruta']}**.
- En controles registrados, hubo **{before['mandos_en_modo_ruta_con_scan_critico']}** muestras con `RUTA` y `scan ≤ 0.35 m`; en **{mismatch}** de ellas además `df ≥ 0.80 m` (discrepancia entre sensores).
- La primera ventana `DIFUSO`→última transición cubrió **{freq.get('primer_difuso_s','ND')}–{freq.get('ultimo_modo_s','ND')} s**; frecuencia aproximada de alternancias posteriores al primer `DIFUSO`: **{freq.get('frecuencia_cambios_hz','ND')} Hz**.
- La interrupción fue preventiva; **no se ha demostrado una colisión física**.

## Evidencia posterior, NO pareada (B3 v02: cinco ensayos)

- Llegadas con evasión híbrida asistida: **{count}/5**, bajo la validación registrada por el ejecutor.
- Error final medio: **{means('error_final_meta_m'):.4f} m**; recorrido medio: **{means('recorrido_m'):.4f} m**.
- Despeje mínimo medio por ensayo: **{means('despeje_min_m'):.4f} m**; mínimo entre los cinco ensayos: **{min_success_clear:.4f} m**.
- En B3 v02 se añaden **subobjetivos temporales**, giro asistido y retención del bypass. A3 y Mamdani siguen participando, pero no controlan por sí solos toda la maniobra.

## Diagnóstico causal sustentado

1. La ruta A* inicial no incluía la caja, por lo que su aparición exigió reacción online.
2. Con la caja próxima, se observó alternancia entre `DIFUSO` y `RUTA`; hubo instantes con `scan` crítico y `df` elevado, lo que permite recuperar erróneamente el seguimiento de una ruta bloqueada.
3. En v01 se conservó velocidad positiva de avance y la separación a la caja descendió hasta activar `COLISION_PROYECTADA`.
4. La intervención v02 mantuvo la evasión y añadió control geométrico temporal; **su éxito no aísla qué porcentaje de mejora proviene de la histéresis, Mamdani o la asistencia geométrica**.

## Conclusión y límites de C1

**C1: DIAGNÓSTICO IDENTIFICADO Y CUANTIFICADO.** La prioridad de C2–C4 es definir métricas y evaluar el antes/después de manera comparable; **1 fallo v01 y 5 éxitos v02 no son una comparación operacional pareada**. Tampoco se ha medido el contacto real con Gazebo ni reproducido íntegramente cada rosbag. Los CSV y JSON provienen de los seis directorios originales enumerados en `tabla_C1.csv`.

**Recomendación:** no modificar ni volver a ejecutar por el momento los controladores auditados. Llevar esta causa y sus reservas al cierre del Avance 2 y definir si C2–C4 requiere un experimento controlado adicional.
'''


def main():
    p=argparse.ArgumentParser(description='Auditoría C1 sin ROS/Gazebo ni modificaciones del repositorio')
    p.add_argument('--raiz',type=Path,default=Path.home()/'AMR_Logistico_Fase_0'/'resultados'/'A2_B2_rescate_geometria_astar'/'20261108_171723')
    p.add_argument('--salida',type=Path,default=None,help='Directorio NUEVO donde guardar los resultados')
    a=p.parse_args()
    root=a.raiz.expanduser().resolve()
    if not root.is_dir():
        p.error(f'Carpeta de experimentos no encontrada: {root}')
    rows=[];details=[];files=[]
    for phase,kind,runid in RUNS:
        run=root/kind/runid
        row,note,sources=audit(run,phase)
        rows.append(row);details.append(note);files.extend(sources)
    if sum(int(r['llegada']) for r in rows[1:])!=5:
        fail('B3 v02 no tiene cinco llegadas válidas')
    stamp=dt.datetime.now(dt.timezone.utc).strftime('%Y%m%d_%H%M%S_UTC')
    out=(a.salida or (Path.home()/'AMR_A2_entrega_local'/f'C1_auditoria_{stamp}')).expanduser().absolute()
    if out.exists():
        fail(f'La salida ya existe; no se sobrescribe: {out}')
    if out == root or root in out.parents:
        fail('Por seguridad, la salida no puede ubicarse dentro de las evidencias B3 originales')
    out.mkdir(parents=True,exist_ok=False)
    with (out/'tabla_C1.csv').open('w',encoding='utf-8',newline='') as handle:
        writer=csv.DictWriter(handle,fieldnames=FIELDS)
        writer.writeheader();writer.writerows(rows)
    with (out/'manifiesto_SHA256.csv').open('w',encoding='utf-8',newline='') as handle:
        writer=csv.writer(handle);writer.writerow(['archivo_fuente','sha256'])
        for f in files:
            writer.writerow([str(f),digest(f)])
    doc={'fecha_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
         'dictamen_C1':'DEBILIDAD_IDENTIFICADA_Y_CUANTIFICADA',
         'debilidad':'Conmutacion inestable RUTA/DIFUSO con obstruccion dinamica; perdida de continuidad evasiva y paro preventivo',
         'evidencia_previa':rows[0], 'evidencia_post_hibrida':rows[1:],
         'detalles_previo':details[0], 'detalles_post_hibrida':details[1:],
         'advertencias':['No es comparacion operacional pareada',
                          'v02 incorpora guiado geometrico asistido y no aisla efecto del Mamdani',
                          'rosbags comprobados por presencia, no reproducidos',
                          'contactos fisicos Gazebo no medidos',
                          'no equivale a cumplimiento formal de toda la rubrica']}
    (out/'dictamen_C1.json').write_text(json.dumps(doc,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    report=md_summary(rows,details,doc['fecha_utc'])
    (out/'informe_C1.md').write_text(report,encoding='utf-8')
    print('\n=== C1 — DIAGNOSTICO OPERACIONAL ===')
    print('Evidencia previa:',rows[0]['estado'],'| cambios A3:',rows[0]['modos_a3_cambios'],
          '| entradas DIFUSO:',rows[0]['entradas_difuso'],'| retornos RUTA:',rows[0]['retornos_ruta'])
    print('Despeje previo:',f"{rows[0]['despeje_min_m']:.4f} m",
          '| holgura a barrera:',f"{rows[0]['holgura_sobre_umbral_m']:.4f} m")
    print('Discrepancias RUTA/scan critico/df alto:',rows[0]['mandos_ruta_con_df_alto_y_scan_critico'])
    print('Llegadas posteriores híbridas asistidas: 5/5',
          '| despeje medio:',f"{statistics.mean(float(r['despeje_min_m']) for r in rows[1:]):.4f} m")
    print('DICTAMEN C1: DEBILIDAD IDENTIFICADA Y CUANTIFICADA')
    print('LIMITACION: NO es un A/B pareado ni una mejora atribuible solo a Mamdani')
    print('SALIDA:',out)
    print('ARCHIVOS: informe_C1.md, tabla_C1.csv, dictamen_C1.json, manifiesto_SHA256.csv')
    return 0


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except (ValueError,OSError) as error:
        print('[NO APROBADO C1] '+str(error))
        raise SystemExit(2)
