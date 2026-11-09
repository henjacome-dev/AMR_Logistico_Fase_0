#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""C4 — Cierre documental y matriz antes/despues del escenario B3.

Usa exclusivamente las evidencias auditadas C1–C3, NO corre ROS/Gazebo,
NO altera archivos preexistentes ni hace commit Git.

Uso:
  python3 A2_C4_cierre_comparativo_operacional_v01.py
  python3 A2_C4_cierre_comparativo_operacional_v01.py --c3 /ruta/C3_evaluacion_..._UTC
"""
from __future__ import annotations
import argparse
import csv
import datetime as dt
import hashlib
import html
import json
import math
import sys
from pathlib import Path

ESTADO = 'CIERRE_DESCRIPTIVO_HIBRIDO_ASISTIDO_CON_RESERVAS'
CRITERIO_C3 = 'REDISENO_HIBRIDO_ASISTIDO_OBSERVADO_CON_RESERVAS'
METRICAS = [
    ('despeje_min_corrida_m', 'Despeje minimo por corrida', 'm'),
    ('ventana_duracion_s', 'Duracion ventana odometrica comun de 0.60 m', 's'),
    ('ventana_giro_abs_medio_ponderado_radps', 'Giro absoluto medio en ventana comun', 'rad/s'),
    ('ventana_variacion_angular_normalizada_por_s', 'Variacion angular por tiempo en ventana', 'indice'),
    ('ventana_porcentaje_tiempo_difuso', 'Tiempo A3 DIFUSO en ventana', '%'),
    ('ventana_cambios_modo_a3_muestreados', 'Cambios muestreados A3 en ventana', 'eventos'),
    ('eventos_a3_cambios', 'Conmutaciones A3 por corrida completa', 'eventos'),
    ('recorrido_total_m', 'Distancia total recorrida', 'm'),
]


def exigir(valor, mensaje):
    if not valor:
        raise ValueError(mensaje)


def sha256(archivo):
    h = hashlib.sha256()
    with archivo.open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def numero(valor, nombre):
    try:
        v=float(valor)
    except (ValueError,TypeError):
        raise ValueError('Valor no numerico en '+nombre)
    exigir(math.isfinite(v),'Valor no finito en '+nombre)
    return v


def fecha():
    return dt.datetime.now(dt.timezone.utc)


def buscar_c3(arg):
    base=Path.home()/'AMR_A2_entrega_local'
    if arg:
        ruta=arg.expanduser().resolve()
    else:
        candidatos=[p for p in base.glob('C3_evaluacion_*_UTC')
                    if p.is_dir() and (p/'dictamen_C3.json').is_file()]
        exigir(bool(candidatos),'No hay dictamen C3. Usa --c3 RUTA para seleccionarlo.')
        ruta=max(candidatos,key=lambda p:p.stat().st_mtime).resolve()
    exigir((ruta/'dictamen_C3.json').is_file(),'No existe dictamen C3 en '+str(ruta))
    exigir((ruta/'manifiesto_SHA256.csv').is_file(),'No existe manifiesto C3 en '+str(ruta))
    return ruta


def cargar_y_verificar(c3):
    data=json.loads((c3/'dictamen_C3.json').read_text(encoding='utf-8'))
    exigir(data.get('etapa')=='C3','Etapa de dictamen incorrecta')
    res=data.get('resultado')
    exigir(isinstance(res,dict) and res.get('resultado')==CRITERIO_C3,
           'C3 no tiene un dictamen aceptable para cierre descriptivo')
    exigir(res.get('comparacion_pareada') is False and res.get('atribuible_al_FIS_aisladamente') is False,
           'C3 no declara reservas metodologicas obligatorias')
    exigir(res.get('contactos_fisicos_medidos') is False and res.get('rosbags_reproducidos') is False,
           'C3 no conserva reservas sobre contactos y rosbags')
    exigir(int(res.get('n_antes',0))==1 and int(res.get('n_despues',0))==5,
           'Tamanos de muestra inesperados')
    exigir(int(res.get('llegadas_antes',-1))==0 and int(res.get('llegadas_despues',-1))==5,
           'Consolidado de llegadas no coincide con el auditado')
    exigir(int(res.get('detenciones_por_colision_proyectada_antes',-1))==1 and
           int(res.get('detenciones_por_colision_proyectada_despues',-1))==0,
           'No coinciden los eventos de barrera registrados')
    exigir(abs(numero(res.get('despeje_preventivo_m'),'barrera')-0.255)<1e-8,
           'La barrera preventiva fue modificada')
    met=res.get('metricas',{})
    exigir(all(k in met for k,_,_ in METRICAS),'Faltan indicadores comparables de C3')
    for k,_,_ in METRICAS:
        d=met[k]
        for x in ('antes','despues_media','despues_min','despues_max','delta_absoluta'):
            numero(d.get(x),k+'.'+x)
        exigir(d['despues_min']<=d['despues_media']<=d['despues_max'],
               'Rango posterior invalido '+k)
    for nombre in ('informe_C3.md','tabla_C3.csv'):
        exigir((c3/nombre).is_file(),'Falta archivo C3: '+nombre)
    # C3 incluye hashes de documentos C1/C2 y 18 fuentes originales.
    verificados=[]
    with (c3/'manifiesto_SHA256.csv').open('r',newline='',encoding='utf-8') as f:
        rows=list(csv.DictReader(f))
    exigir(len(rows)>=24,'Manifiesto C3 incompleto (se esperan 18 fuentes + anexos C1/C2)')
    rutas=set()
    for row in rows:
        p=Path(row['archivo_fuente'])
        exigir(p.is_file(),'Fuente del manifiesto ausente: '+str(p))
        exigir(str(p) not in rutas,'Fuente repetida en manifiesto: '+str(p))
        rutas.add(str(p))
        real=sha256(p)
        exigir(real==row['sha256'],'Hash no coincide con C3: '+str(p))
        verificados.append((p,real))
    for src in data.get('fuentes_C1_C2',[]):
        exigir(str(src) in rutas,'C1/C2 no figuran en el manifiesto de C3: '+str(src))
    return data,res,met,verificados


def fila(k,etiqueta,unidad,d):
    a=numero(d['antes'],k+'.antes')
    b=numero(d['despues_media'],k+'.despues')
    variacion=None if abs(a)<1e-12 else (b/a-1)*100
    return {'id':k,'indicador':etiqueta,'unidad':unidad,
            'antes_n1':a,'despues_media_n5':b,
            'despues_min':numero(d['despues_min'],k+'.min'),
            'despues_max':numero(d['despues_max'],k+'.max'),
            'delta_absoluta':b-a,'delta_porcentual':variacion}


def fmt(n,d=4):
    return f'{n:.{d}f}'


def porc(n):
    return 'No definida' if n is None else f'{n:+.2f}%'


def reporte(filas,res,c3,tstamp):
    lookup={f['id']:f for f in filas}
    d=lookup['despeje_min_corrida_m']
    t=lookup['ventana_duracion_s']
    g=lookup['ventana_giro_abs_medio_ponderado_radps']
    cuerpo=[
        '# C4 — Cierre comparativo del rediseño operacional B3',
        '',
        '**Fecha UTC:** '+tstamp,
        '**Dictamen:** '+ESTADO,
        '**Fuentes:** C1 (diagnóstico), C2 (línea base), C3 (evaluación). Fuente inmediata: `'+str(c3/'dictamen_C3.json')+'`.',
        '',
        '## Objetivo y diseño comparable',
        '',
        'Documentar la respuesta del sistema móvil ante un obstáculo insertado dinámicamente a 1 m. '
        'B3 v01 representa el sistema inicial (un ensayo detenido), mientras B3 v02 incorpora '
        '**retención de evasión, giro preventivo y subobjetivos geométricos temporales** (cinco ensayos). '
        'Se utiliza una ventana común de **0,60 m odométricos desde la inserción del obstáculo** para '
        'comparar duración y giro. El despeje se refiere al mínimo de la corrida completa, '
        'no exclusivamente a esa ventana.',
        '',
        '## Matriz antes/después',
        '',
        '| Indicador | Antes (v01, n=1) | Después (v02, n=5; media) | Variación |',
        '|---|---:|---:|---:|',
        '| Llegadas | 0/1 | 5/5 | No interpretable como tasa poblacional |',
        '| `COLISION_PROYECTADA` | 1 | 0 registradas | Diferente tamaño muestral |',
    ]
    for f in filas:
        cuerpo.append('| '+f['indicador']+' | '+fmt(f['antes_n1'])+' '+f['unidad']+
                      ' | '+fmt(f['despues_media_n5'])+' '+f['unidad']+
                      ' | '+porc(f['delta_porcentual'])+' |')
    cuerpo.extend([
        '',
        '## Interpretación del resultado',
        '',
        '- **Llegada y seguridad geométrica observadas:** B3 v02 obtuvo 5/5 llegadas y 0 detenciones preventivas registradas; '
        'el mínimo de despeje posterior entre los cinco ensayos fue '+fmt(res['despeje_minimo_despues_m'])+' m, '
        'por encima de la barrera de '+fmt(res['despeje_preventivo_m'],3)+' m.',
        '- **Despeje:** '+fmt(d['antes_n1'])+' m antes versus '+fmt(d['despues_media_n5'])+
        ' m de media después; diferencia descriptiva de '+fmt(d['delta_absoluta'])+' m.',
        '- **Tiempo de recorrido comparable:** '+fmt(t['antes_n1'])+' s antes versus '+fmt(t['despues_media_n5'])+
        ' s después ('+porc(t['delta_porcentual'])+'). La maniobra asistida resultó más lenta en esta ventana.',
        '- **Giro absoluto medio:** '+fmt(g['antes_n1'])+' frente a '+fmt(g['despues_media_n5'])+
        ' rad/s ('+porc(g['delta_porcentual'])+'). Menor giro medio no prueba por sí solo menor oscilación.',
        '',
        '## Reservas obligatorias para el informe docente',
        '',
        '1. Comparación descriptiva **no pareada**, con n=1 antes y n=5 después; no se realizó aleatorización.',
        '2. Cambió la **arquitectura de control**: el efecto del supervisor y Mamdani no se puede aislar '
        'del desvío y centrado geométricos. No afirmar «Mamdani mejoró un X %».',
        '3. No se midieron directamente los **contactos físicos** de Gazebo; el despeje es geométrico.',
        '4. Se verificó la presencia de seis rosbags, pero **no** su reproducción.',
        '5. La ventana común está igualada por **distancia odométrica**, no implica igualdad de todos '
        'los estados iniciales, perturbaciones ni comandos.',
        '6. El experimento R21 sobre FIS v01/v02 es una comparación **numérica separada**; '
        'no constituye esta validación operacional.',
        '',
        '## Dictamen de cierre C1–C4',
        '',
        '**Aceptación descriptiva condicionada del rediseño híbrido asistido**: '
        'el sistema posterior completó cinco recorridos donde la ejecución original se detuvo. '
        'Se conservan el coste temporal y todas las reservas de atribución causal. '
        'Si la rúbrica exige ensayo A/B pareado o prueba exclusiva de lógica difusa, esa exigencia '
        'permanece sin acreditar con estos datos.',
        '',
        '## Continuidad hacia D1–D7',
        '',
        'Utilizar esta matriz como evidencia C4 en el informe de dos páginas, junto a las pruebas B1–B4, '
        'la concordancia MATLAB/Python y la CNN preliminar. Conservar la fuente completa y su manifiesto SHA256. '
        'La generación de este reporte no publica en GitHub ni entrega en Blackboard.',
        '',
    ])
    return '\n'.join(cuerpo)


def reporte_html(filas,res,c3,tstamp):
    lookup={f['id']:f for f in filas}
    t=lookup['ventana_duracion_s'];g=lookup['ventana_giro_abs_medio_ponderado_radps'];d=lookup['despeje_min_corrida_m']
    def row(label,old,new,unit,delta):
        return '<tr><th>'+html.escape(label)+'</th><td>'+fmt(old)+' '+html.escape(unit)+'</td><td>'+fmt(new)+' '+html.escape(unit)+'</td><td>'+html.escape(porc(delta))+'</td></tr>'
    table=''.join(row(f['indicador'],f['antes_n1'],f['despues_media_n5'],f['unidad'],f['delta_porcentual']) for f in filas)
    cards=''.join('<article><small>'+html.escape(label)+'</small><strong>'+html.escape(value)+'</strong><span>'+html.escape(extra)+'</span></article>'
                  for label,value,extra in [
                     ('Llegadas documentadas','0/1 → 5/5','Números de prueba distintos'),
                     ('Despeje','+ %.1f cm'%((d['despues_media_n5']-d['antes_n1'])*100),'Media posterior, mínimo por corrida'),
                     ('Tiempo ventana',porc(t['delta_porcentual']),'Más tiempo por 0,60 m'),
                     ('Giro medio ventana',porc(g['delta_porcentual']),'No equivale a menor oscilación'),
                  ])
    return '''<!DOCTYPE html><html lang="es"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>C4 | AMR — Antes y después</title>
<style>*{box-sizing:border-box}body{font-family:Arial,Helvetica,sans-serif;background:#f6f8fb;color:#142339;margin:0;padding:28px;line-height:1.52}main{max-width:1100px;margin:auto}h1{font-size:27px;line-height:1.2}h2{font-size:18px;margin-top:28px}.sub{color:#52647b}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(195px,1fr));gap:12px}article,section{background:white;border:1px solid #dce4ee;border-radius:10px;padding:16px}article small{display:block;color:#516179}article strong{display:block;font-size:25px;margin:6px 0}article span{font-size:12px;color:#607085}.scroll{overflow-x:auto}table{width:100%;border-collapse:collapse;font-size:14px}th,td{padding:10px 12px;border-bottom:1px solid #e2e8f0;text-align:left}td{font-variant-numeric:tabular-nums}tr:nth-child(even){background:#f8fafc}.note{border-left:4px solid #778ba5;padding-left:14px}footer{margin-top:20px;font-size:12px;color:#52647b}@media print{body{padding:0;background:white}section,article{break-inside:avoid}}</style></head>
<body><main><h1>C4 · Rediseño operacional: antes y después</h1><p class="sub">''' + html.escape(tstamp) + ''' · B3, caja insertada a 1 m · Comparación descriptiva no pareada</p>
<div class="grid">''' + cards + '''</div><h2>Matriz de resultados</h2><section class="scroll"><table><thead><tr><th>Indicador</th><th>Antes · n=1</th><th>Después · media n=5</th><th>Cambio</th></tr></thead><tbody>
<tr><th>Llegadas</th><td>0/1</td><td>5/5</td><td>No causal</td></tr><tr><th>Paradas COLISION_PROYECTADA</th><td>1</td><td>0 registradas</td><td>Muestras distintas</td></tr>''' + table + '''</tbody></table></section>
<h2>Alcance de la comparación</h2><section><p><strong>Ventana común:</strong> 0,60 m de recorrido odométrico posterior a la aparición del obstáculo, para las variables de duración y giro. El despeje es el mínimo por corrida.</p>
<p>La intervención posterior combina retención de evasión, giro preventivo y subobjetivos geométricos temporales; por tanto, refleja el rendimiento del <strong>sistema híbrido asistido</strong>, no una mejora aislada del FIS Mamdani.</p>
<p class="note">Limitaciones: un ensayo anterior frente a cinco posteriores; no pareados ni aleatorizados. Contactos físicos de Gazebo no medidos; rosbags comprobados por presencia, no reproducidos. El menor giro medio no equivale por sí mismo a menor oscilación.</p>
<p><strong>Dictamen:</strong> aceptación descriptiva condicionada del rediseño híbrido asistido. Si se exige un A/B pareado o control exclusivamente difuso, esa exigencia permanece sin acreditar.</p></section>
<footer>Fuentes: C1/C2/C3 y sus 18 archivos originales con SHA256 verificados. Fuente inmediata: ''' + html.escape(str(c3/'dictamen_C3.json')) + '''.</footer></main></body></html>'''


def main(argv=None):
    pa=argparse.ArgumentParser(description='C4 — cierre comparativo con auditoria de hashes, sin ROS/Git')
    pa.add_argument('--c3',type=Path,default=None,help='Carpeta C3 validada (si no se da, usa la mas reciente)')
    pa.add_argument('--salida',type=Path,default=None,help='Carpeta NUEVA, fuera del repositorio')
    a=pa.parse_args(argv)
    c3=buscar_c3(a.c3)
    original,res,metricas,fuentes=cargar_y_verificar(c3)
    filas=[fila(k,t,u,metricas[k]) for k,t,u in METRICAS]
    tstamp=fecha().isoformat()
    stamp=fecha().strftime('%Y%m%d_%H%M%S_UTC')
    repo=(Path.home()/'AMR_Logistico_Fase_0').resolve()
    out=(a.salida or Path.home()/'AMR_A2_entrega_local'/('C4_cierre_'+stamp)).expanduser().resolve()
    exigir(not out.exists(),'La salida existe. No se sobrescribe: '+str(out))
    exigir(out!=repo and repo not in out.parents,'Salida dentro del repositorio no permitida')
    out.mkdir(parents=True,exist_ok=False)
    contenido=reporte(filas,res,c3,tstamp)
    (out/'informe_C4.md').write_text(contenido,encoding='utf-8')
    (out/'reporte_visual_C4.html').write_text(reporte_html(filas,res,c3,tstamp),encoding='utf-8')
    with (out/'tabla_C4.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=list(filas[0].keys()))
        writer.writeheader();writer.writerows(filas)
    registro={'etapa':'C4','fecha_utc':tstamp,'dictamen':ESTADO,'escenario':'B3',
              'comparacion_pareada':False,'atribucion_aislada_Mamdani':False,
              'guia_geometrica_asistida':True,'contactos_gazebo_verificados':False,
              'rosbags_reproducidos':False,'n_antes':1,'n_despues':5,
              'llegadas_antes':0,'llegadas_despues':5,
              'paradas_preventivas_antes':1,'paradas_preventivas_despues':0,
              'barrera_m':numero(res['despeje_preventivo_m'],'barrera'),
              'despeje_minimo_posterior_m':numero(res['despeje_minimo_despues_m'],'minimo'),
              'horizonte_odometrico_comun_m':0.60,'metricas':filas,
              'fuente_C3':str(c3/'dictamen_C3.json'),'manifiesto_C3_verificado_n':len(fuentes)}
    (out/'dictamen_C4.json').write_text(json.dumps(registro,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    with (out/'manifiesto_SHA256.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f);writer.writerow(['tipo','archivo','sha256'])
        for p in [c3/'dictamen_C3.json',c3/'informe_C3.md',c3/'tabla_C3.csv',c3/'manifiesto_SHA256.csv']:
            writer.writerow(['fuente_C3',str(p),sha256(p)])
        for p,digest in fuentes:writer.writerow(['fuente_original_auditada',str(p),digest])
        for n in ['informe_C4.md','reporte_visual_C4.html','tabla_C4.csv','dictamen_C4.json']:
            p=out/n;writer.writerow(['salida_C4',str(p),sha256(p)])
    t=next(v for v in filas if v['id']=='ventana_duracion_s')
    g=next(v for v in filas if v['id']=='ventana_giro_abs_medio_ponderado_radps')
    d=next(v for v in filas if v['id']=='despeje_min_corrida_m')
    print('=== C4 — CIERRE COMPARATIVO OPERACIONAL B3 ===')
    print('FUENTES C3: '+str(c3))
    print('SHA256 C1/C2 + 18 archivos originales: %d referencias verificadas'%len(fuentes))
    print('LLEGADAS: 0/1 -> 5/5 (hibrido asistido)')
    print('DESPEJE MEDIA: %.4f -> %.4f m (diferencia %.4f m)'%(d['antes_n1'],d['despues_media_n5'],d['delta_absoluta']))
    print('TIEMPO VENTANA 0.60m: %.4f -> %.4f s (%s)'%(t['antes_n1'],t['despues_media_n5'],porc(t['delta_porcentual'])))
    print('GIRO MEDIO VENTANA: %.4f -> %.4f rad/s (%s)'%(g['antes_n1'],g['despues_media_n5'],porc(g['delta_porcentual'])))
    print('DICTAMEN C4: '+ESTADO)
    print('LIMITACIONES: no A/B pareado; no atribucion exclusiva a Mamdani; contactos sin medir; rosbags sin reproducir')
    print('SALIDA: '+str(out))
    print('ARCHIVOS: informe_C4.md, reporte_visual_C4.html, tabla_C4.csv, dictamen_C4.json, manifiesto_SHA256.csv')
    return 0

if __name__=='__main__':
    try:sys.exit(main())
    except (ValueError,OSError,KeyError,json.JSONDecodeError) as e:
        print('[C4 NO VALIDADO] '+str(e),file=sys.stderr)
        sys.exit(2)
