#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""C3 — Evaluación descriptiva del rediseño operacional B3 (solo lectura).

Lee C1 y C2 ya auditados. NO ejecuta Gazebo, NO modifica ROS/Git ni
pretende atribuir los resultados al FIS Mamdani de forma aislada.

Ejemplo:
 python3 /mnt/c/Users/Henry/Downloads/A2_C3_evaluacion_rediseno_operacional_v01.py
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

C1_FOLDER='C1_auditoria_20261109_015809_UTC'
C2_FOLDER='C2_linea_base_20261109_021010_UTC'
SAFETY_M=0.255


def require(condition,message):
    if not condition:
        raise ValueError(message)


def sha256(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):
            h.update(block)
    return h.hexdigest()


def finite_num(x, key):
    try:
        v=float(x)
    except (ValueError,TypeError):
        raise ValueError('Campo no numerico %s: %r' % (key,x))
    require(math.isfinite(v),'Valor no finito %s' % key)
    return v


def mean(data, key):
    return statistics.mean(finite_num(row.get(key),key) for row in data)


def fmt(x,n=4):
    return f'{x:.{n}f}'


def pct(before,after):
    return 100*(after/before-1) if before else None


def retrieve(a):
    base=Path.home()/'AMR_A2_entrega_local'
    c1=(a.c1 or base/C1_FOLDER).expanduser().resolve()
    c2=(a.c2 or base/C2_FOLDER).expanduser().resolve()
    for folder in (c1,c2):
        require(folder.is_dir(),'No se encontro carpeta auditada: '+str(folder))
        for name in ('dictamen_'+('C1' if folder==c1 else 'C2')+'.json',
                     'manifiesto_SHA256.csv',
                     'informe_'+('C1' if folder==c1 else 'C2')+'.md'):
            require((folder/name).is_file(),'Falta evidencia '+str(folder/name))
    d1=json.loads((c1/'dictamen_C1.json').read_text(encoding='utf-8'))
    d2=json.loads((c2/'dictamen_C2.json').read_text(encoding='utf-8'))
    require(d1.get('dictamen_C1')=='DEBILIDAD_IDENTIFICADA_Y_CUANTIFICADA',
            'C1 no tiene dictamen validado')
    require(d2.get('dictamen')=='LINEA_BASE_OPERACIONAL_DEFINIDA_CON_RESERVAS',
            'C2 no tiene dictamen validado')
    rows=d2.get('filas')
    require(isinstance(rows,list) and len(rows)==6,'C2 debe tener exactamente seis ensayos')
    before=rows[0];after=rows[1:]
    require(before.get('fase')=='ANTES_V01' and all(r.get('fase')=='DESPUES_V02' for r in after),
            'Orden o identificadores de fases invalidos')
    require(int(before.get('llegada',-1))==0 and sum(int(r.get('llegada',-1)) for r in after)==5,
            'Resultados de llegada no coinciden con C2')
    require(int(before.get('colision_proyectada',-1))==1 and
            all(int(r.get('colision_proyectada',-1))==0 for r in after),
            'Barreras proyectadas no coinciden con el cierre C2')
    b1=d1.get('evidencia_previa') or {}
    require(int(b1.get('modos_a3_cambios',-1))==int(before.get('eventos_a3_cambios',-2)),
            'Discrepancia entre conteo de conmutaciones C1 y C2')
    require(int(b1.get('mandos_ruta_con_df_alto_y_scan_critico',-1))==
            int(before.get('muestras_ruta_scan_critico_df_alto',-2)),
            'Discrepancia entre muestras de sensor C1 y C2')
    require(abs(finite_num(b1.get('despeje_min_m'),'C1 despeje')-
                finite_num(before.get('despeje_min_corrida_m'),'C2 despeje'))<1e-4,
            'Despejes de linea base C1/C2 no coinciden')
    require(finite_num(d2.get('horizonte_recorrido_comun_post_spawn_m'),'horizonte')==0.60,
            'La ventana comparativa no es de 0.60 m')

    # Validar los 18 archivos de origen identificados por el auditor C2.
    sources=d2.get('fuentes')
    require(isinstance(sources,list) and len(sources)==18,
            'Faltan referencias a los 18 archivos fuente originales')
    checked=[]
    for src in sources:
        p=Path(src['archivo'])
        require(p.is_file(),'Archivo de origen ausente: '+str(p))
        digest=sha256(p)
        require(digest==src.get('sha256'),'Archivo original alterado desde C2: '+str(p))
        checked.append((str(p),digest))
    require(len({p for p,h in checked})==18,'Las fuentes C2 no son 18 archivos distintos')
    # Verificar las seis evidencias de rosbag, por presencia y tamaño, sin reproducirlas.
    for row in rows:
        root=Path(row['directorio_origen'])
        require(root.is_dir(),'Directorio original ausente: '+str(root))
        bags=list((root/'rosbag2').glob('*.db3'))
        require(any(b.is_file() and b.stat().st_size>4096 for b in bags),
                'Rosbag ausente: '+str(root))
    return c1,c2,d1,d2,rows,checked


def evaluate(rows):
    b=rows[0];a=rows[1:]
    keys={
       'despeje_min_corrida_m':'Distancia minima geometrica de corrida (m)',
       'ventana_duracion_s':'Tiempo para 0.60m posteriores al spawn (s)',
       'ventana_giro_abs_medio_ponderado_radps':'Giro absoluto medio en ventana (rad/s)',
       'ventana_variacion_angular_normalizada_por_s':'Indice de variacion angular por tiempo',
       'ventana_porcentaje_tiempo_difuso':'Tiempo de A3 en DIFUSO (%)',
       'ventana_cambios_modo_a3_muestreados':'Cambios de A3 muestreados en ventana',
       'ventana_comandos_asistencia_geom':'Comandos de asistencia geometrica en ventana',
       'eventos_a3_cambios':'Conmutaciones A3 registradas en corrida',
       'recorrido_total_m':'Recorrido de corrida completa (m)',
    }
    metrics={}
    for key,label in keys.items():
        old=finite_num(b.get(key),key)
        new=mean(a,key)
        metrics[key]={'descripcion':label,'antes':old,'despues_media':new,
                      'despues_min':min(finite_num(r.get(key),key) for r in a),
                      'despues_max':max(finite_num(r.get(key),key) for r in a),
                      'delta_absoluta':new-old,'delta_porcentual':pct(old,new)}
    safety=[finite_num(r.get('despeje_min_corrida_m'),'despeje') for r in a]
    assisted=[int(r.get('comandos_asistencia_geom',0)) for r in a]
    # Clasificación automática: resultado positivo DEL SISTEMA sin inferir causalidad.
    require(all(x>=SAFETY_M for x in safety),'Se detectó despeje inferior al margen preventivo en una réplica')
    return {
        'n_antes':1,'n_despues':5,
        'llegadas_antes':0,'llegadas_despues':5,
        'detenciones_por_colision_proyectada_antes':1,
        'detenciones_por_colision_proyectada_despues':0,
        'despeje_preventivo_m':SAFETY_M,
        'despeje_minimo_despues_m':min(safety),
        'asistencia_geometrica_n_comandos_por_ensayo':assisted,
        'metricas':metrics,
        'resultado':'REDISENO_HIBRIDO_ASISTIDO_OBSERVADO_CON_RESERVAS',
        'atribuible_al_FIS_aisladamente':False,
        'comparacion_pareada':False,
        'contactos_fisicos_medidos':False,
        'rosbags_reproducidos':False,
    }


def report_md(c1,c2,ev,timestamp):
    m=ev['metricas']
    lines=[
      '# C3 — Evaluación del rediseño operacional: B3',
      '',
      '**Fecha UTC:** '+timestamp,
      '**Dictamen:** '+ev['resultado'],
      '',
      '## Diagnóstico y acción correctiva',
      '',
      'En B3 v01, A3 conmutó repetidamente entre `RUTA` y `DIFUSO` ante una caja insertada durante el desplazamiento. '
      'La ruta A* original continuó habilitándose aunque el LiDAR mostraba peligro; la protección detuvo el robot antes de llegar a la meta.',
      '',
      'B3 v02 combinó retención evasiva, bloqueo preventivo del avance, giro asistido y subobjetivos geométricos temporales. '
      '**La intervención es híbrida y compuesta.** A3 y Mamdani participaron, pero el efecto de sus propuestas no puede separarse '
      'del guiado geométrico introducido.',
      '',
      '## Comparación descriptiva de ensayos',
      '',
      '| Indicador | B3 v01 (n=1) | B3 v02 (n=5) |',
      '|---|---:|---:|',
      '| Llegadas | **0/1** | **5/5** |',
      '| Detenciones preventivas `COLISION_PROYECTADA` | 1 | 0 observadas |',
    ]
    for key in ['despeje_min_corrida_m','ventana_duracion_s',
                'ventana_giro_abs_medio_ponderado_radps',
                'ventana_variacion_angular_normalizada_por_s',
                'ventana_porcentaje_tiempo_difuso',
                'ventana_cambios_modo_a3_muestreados',
                'eventos_a3_cambios']:
        d=m[key]
        pct_str='ND' if d['delta_porcentual'] is None else ('%+.2f%%'%d['delta_porcentual'])
        lines.append('| '+d['descripcion']+' | '+fmt(d['antes'],4)+
                     ' | '+fmt(d['despues_media'],4)+' (media; '+pct_str+') |')
    lines.extend(['',
      'El despeje se calcula geométricamente respecto de objetos del SDF. No es un evento de contacto físico.',
      'Los porcentajes se calculan sobre **medias descriptivas**, con muestras no equivalentes.',
      '',
      '## Criterios técnicos y alcance',
      '',
      '1. **Seguridad geométrica observada:** las cinco ejecuciones asistidas permanecieron por encima del margen de '+fmt(SAFETY_M,3)+' m, según su auditoría de trayectorias.',
      '2. **Eficacia observada del sistema:** las cinco ejecuciones asistidas llegaron a la meta y no registraron `COLISION_PROYECTADA`. No significa tasa de éxito poblacional del 100 %.',
      '3. **Coste de la intervención:** comparar la duración de la ventana común y el giro, que pueden mejorar en sentidos opuestos.',
      '4. **No atribuibilidad aislada:** el aporte de subobjetivos geométricos, conmutación asistida y Mamdani no se ha separado con un ensayo de ablación.',
      '5. **Controles y trazabilidad:** C1 y C2 coinciden en la línea base. Se validaron hashes de 18 archivos originales; seis rosbags comprobados solo por presencia.',
      '6. **Limitaciones:** n=1 antes, n=5 después, sin aleatorización ni emparejamiento; no se midieron contactos físicos de Gazebo.',
      '',
      '## Dictamen C3',
      '',
      '**Aceptación operacional descriptiva con reservas:** el sistema híbrido B3 v02 superó en estas cinco ejecuciones la falla de llegada de B3 v01, conservando la barrera de seguridad. '
      'No se demuestra mejora causal aislada del controlador Mamdani ni un experimento A/B estadísticamente controlado.',
      '',
      '## Enlace con C4 y entrega',
      '',
      'C4 debe presentar en una sola tabla la debilidad inicial, las intervenciones combinadas, el resultado antes/después y sus reservas. '
      'Para el informe académico usar el lenguaje «mejora observada del sistema híbrido asistido»; no «optimización demostrada del FIS».',
      '',
      '**Fuentes:**',
      '- '+str(c1/'dictamen_C1.json'),
      '- '+str(c2/'dictamen_C2.json'),
      '- 18 fuentes originales validadas y manifiesto SHA256 anexado.',
      '',
    ])
    return '\n'.join(lines)


def main(argv=None):
    ap=argparse.ArgumentParser(description='C3: comparacion operacional B3, solo lectura')
    ap.add_argument('--c1',type=Path,default=None)
    ap.add_argument('--c2',type=Path,default=None)
    ap.add_argument('--salida',type=Path,default=None)
    a=ap.parse_args(argv)
    c1,c2,d1,d2,rows,sources=retrieve(a)
    ev=evaluate(rows)
    timestamp=dt.datetime.now(dt.timezone.utc).isoformat()
    base=Path.home()/'AMR_A2_entrega_local'
    stamp=dt.datetime.now(dt.timezone.utc).strftime('%Y%m%d_%H%M%S_UTC')
    output=(a.salida or base/('C3_evaluacion_'+stamp)).expanduser().resolve()
    repo=(Path.home()/'AMR_Logistico_Fase_0').resolve()
    require(output!=repo and repo not in output.parents,'No se permiten salidas dentro del repositorio')
    require(not output.exists(),'No se sobrescribe carpeta de salida existente: '+str(output))
    output.mkdir(parents=True,exist_ok=False)
    (output/'informe_C3.md').write_text(report_md(c1,c2,ev,timestamp),encoding='utf-8')
    (output/'dictamen_C3.json').write_text(json.dumps({
      'etapa':'C3','fecha_utc':timestamp,'resultado':ev,
      'fuentes_C1_C2':[str(c1/'dictamen_C1.json'),str(c2/'dictamen_C2.json')],
    },ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    with (output/'tabla_C3.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['indicador','antes','despues_media','despues_min','despues_max','delta','delta_pct'])
        for key,v in ev['metricas'].items():
            w.writerow([v['descripcion'],v['antes'],v['despues_media'],v['despues_min'],
                        v['despues_max'],v['delta_absoluta'],v['delta_porcentual']])
    with (output/'manifiesto_SHA256.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['archivo_fuente','sha256'])
        for p in [c1/'dictamen_C1.json',c2/'dictamen_C2.json']:
            w.writerow([str(p),sha256(p)])
        for name,directory in [('C1',c1),('C2',c2)]:
            for s in ['informe_'+name+'.md','manifiesto_SHA256.csv']:
                p=directory/s;w.writerow([str(p),sha256(p)])
        for p,digest in sources:w.writerow([p,digest])
    m=ev['metricas']
    print('=== C3 — EVALUACION REDISENO OPERACIONAL B3 ===')
    print('FUENTES: C1 + C2 concordantes; 18 archivos fuente SHA256; 6 rosbag DB3 presentes')
    print('LLEGADAS: antes 0/1; despues 5/5 (hibrido asistido)')
    print('DESPEJE: %.4f -> %.4f m (media posterior)' % (
        m['despeje_min_corrida_m']['antes'],m['despeje_min_corrida_m']['despues_media']))
    for key,label in [('ventana_duracion_s','TIEMPO VENTANA 0.60m'),
                      ('ventana_giro_abs_medio_ponderado_radps','GIRO MEDIO VENTANA')]:
        item=m[key]
        print('%s: %.4f -> %.4f (%+.2f%%)' % (label,item['antes'],
            item['despues_media'],item['delta_porcentual']))
    print('DICTAMEN: '+ev['resultado'])
    print('LIMITACION: comparacion descriptiva 1 vs 5; guia geometrica, NO mejora aislada de Mamdani')
    print('SALIDA: '+str(output))
    print('ARCHIVOS: informe_C3.md, tabla_C3.csv, dictamen_C3.json, manifiesto_SHA256.csv')
    return 0

if __name__=='__main__':
    try:
        sys.exit(main())
    except (ValueError,OSError,KeyError,json.JSONDecodeError) as e:
        print('[C3 NO VALIDADO] '+str(e),file=sys.stderr)
        sys.exit(2)
