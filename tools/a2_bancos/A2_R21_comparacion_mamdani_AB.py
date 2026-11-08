#!/usr/bin/env python3
"""Avance 2 / R21 - Comparación A/B REPRODUCIBLE de Mamdani v01 vs v02.

Ejecutar desde Windows Downloads en Ubuntu WSL2:
    python3 /mnt/c/Users/Henry/Downloads/A2_R21_comparacion_mamdani_AB.py

Se ejecutan 4 perfiles de distancias x 5 repeticiones, con RUIDO IDENTICO
por pareja y 36 instantes por perfil. Cada pareja usa el motor R18 de
scikit-fuzzy previamente validado y exactamente las mismas entradas.

No mueve el robot, no inicia Gazebo/ROS, no abre /cmd_vel, no modifica el FIS
original ni los archivos Git y no publica en GitHub. Las métricas describen
RESPUESTAS DEL CONTROLADOR, NO desempeño de navegación ni colisiones reales.
La variante v02 es una HIPOTESIS de rediseño que puede resultar peor.

Genera: FIS v02 candidato, prueba de integridad, perfiles CSV, comparativa
por escenario y global, hipótesis, informe Markdown y dashboard HTML.
"""
from __future__ import annotations
import argparse
import copy
import csv
import datetime as dt
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import re
import subprocess
import sys
import tempfile
import traceback

ROOT = Path.home()/'AMR_Logistico_Fase_0'
R18 = ROOT/'tools/A2_E21_equivalencia_MATLAB_Python.py'
FIS = ROOT/'matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis'
VENV = Path.home()/'.venvs/amr-a2-r18/bin/python'
BASE_SHA = '82ba1d98a4201b01e7eaaa42146c163808dcffec'
FIS_BLOB = '6c7dfdd083728d49c64dd1c90a59b19d63222063'
SEED = 20261108
PROFILES=('APROXIMACION_FRONTAL','OBSTACULO_IZQUIERDO',
          'OBSTACULO_DERECHO','PASILLO_CAMBIANTE')
REPS=5
STEPS=36
STEP_S=.20
SIGMA=.01

# Cambios de funciones de pertenencia documentados en el FIS v02.
# Todos los cambios afectan conjuntos consecuentes; se conservan entradas,
# 27 reglas, dominios y operadores Mamdani.
REPLACEMENTS=(
    ("Name='AMR_Mamdani_A2_v01'", "Name='AMR_Mamdani_A2_v02'"),
    ("MF2='Lenta':'trimf',[0.02 0.07 0.12]", "MF2='Lenta':'trimf',[0.02 0.055 0.1]"),
    ("MF3='Rapida':'trapmf',[0.1 0.15 0.2 0.2]", "MF3='Rapida':'trapmf',[0.11 0.17 0.2 0.2]"),
    ("MF1='DerechaFuerte':'trapmf',[-1.5 -1.5 -1.05 -0.65]",
     "MF1='DerechaFuerte':'trapmf',[-1.25 -1.25 -0.9 -0.6]"),
    ("MF2='DerechaSuave':'trimf',[-0.9 -0.5 -0.15]",
     "MF2='DerechaSuave':'trimf',[-0.75 -0.38 -0.14]"),
    ("MF4='IzquierdaSuave':'trimf',[0.15 0.5 0.9]",
     "MF4='IzquierdaSuave':'trimf',[0.14 0.38 0.75]"),
    ("MF5='IzquierdaFuerte':'trapmf',[0.65 1.05 1.5 1.5]",
     "MF5='IzquierdaFuerte':'trapmf',[0.6 0.9 1.25 1.25]"),
)

def out(s): print(s,flush=True)
def ensure(cond,s):
    if not cond: raise RuntimeError(s)
    out('[OK] '+s)
def sha256(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def blob(data):return hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()

def git(*args):
    r=subprocess.run(['git',*args],cwd=ROOT,text=True,capture_output=True)
    if r.returncode:raise RuntimeError('git '+' '.join(args)+': '+r.stderr.strip())
    return r.stdout.strip()

def variant_text(text):
    v=text
    for old,new in REPLACEMENTS:
        if v.count(old)!=1:raise RuntimeError('Rediseño ambiguo, esperaba una ocurrencia: '+old)
        v=v.replace(old,new)
    if v.count('[Rules]')!=1:raise RuntimeError('Reglas ausentes')
    if text.split('[Rules]',1)[1]!=v.split('[Rules]',1)[1]:
        raise RuntimeError('Las 27 reglas NO se pueden cambiar en este ensayo')
    for marker in ('NumRules=27','NumInputs=3','NumOutputs=2','DefuzzMethod=\'centroid\''):
        if marker not in v:raise RuntimeError('Contrato Mamdani inesperado: '+marker)
    return v

def preflight():
    out('=== R21 - PREFLIGHT COMPARACION MAMDANI v01 vs v02 ===')
    ensure((ROOT/'.git').is_dir(),'Repositorio presente')
    ensure(git('branch','--show-current')=='desarrollo-avance2','Rama esperada')
    ensure(git('rev-parse','HEAD')==BASE_SHA,'Commit base E2.3.6A sin publicaciones intermedias')
    ensure(FIS.is_file() and blob(FIS.read_bytes())==FIS_BLOB,'FIS v01 exactamente inalterado')
    ensure(R18.is_file() and VENV.is_file(),'Motor R18 y venv scikit-fuzzy disponibles')
    for f in (R18, ROOT/'tools/A2_E22_nodo_difuso.py'):
        rel=str(f.relative_to(ROOT))
        ensure(git('hash-object','--',rel)==git('rev-parse','HEAD:'+rel),
               'Dependencia inalterada: '+rel)
    text=FIS.read_text(encoding='utf8')
    ensure(variant_text(text)!=text,'Variación explícita de conjuntos de salida')
    out('[ALCANCE] Banco NUMERICO PAREADO; NO valida comportamiento ROS/Gazebo ni acredita mejora real de navegación.')


def sample(profile,t,rep):
    """Entradas de incertidumbre para 4 escenarios, 5 semillas apareadas."""
    f=t/(STEPS-1); a=2*math.pi*f
    if profile==PROFILES[0]:
        # Aproximación y retirada del frontal, incluyendo distancia de paro
        df=1.8-1.60*(1-abs(2*f-1))
        di=1.45+.15*math.sin(a);dd=1.45-.12*math.sin(a)
    elif profile==PROFILES[1]:
        df=.52+.16*math.sin(a*.75);di=.28+.45*abs(math.sin(a))
        dd=1.70-.1*math.cos(a)
    elif profile==PROFILES[2]:
        df=.52+.16*math.sin(a*.75);di=1.70-.1*math.cos(a)
        dd=.28+.45*abs(math.sin(a))
    else:
        df=.40+.70*abs(math.sin(a*1.4))
        di=.30+.98*abs(math.cos(a*.8))
        dd=.30+.98*abs(math.sin(a*.8))
    rng=random.Random(SEED+PROFILES.index(profile)*100000+rep*200+t)
    return tuple(round(max(.0,min(2.,x+rng.gauss(0,SIGMA))),5) for x in (df,di,dd))

def parse_engine():
    import skfuzzy
    ensure(skfuzzy.__version__=='0.5.0','scikit-fuzzy 0.5.0 verificado')
    spec=importlib.util.spec_from_file_location('a2_r21_r18',R18)
    assert spec and spec.loader
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module
    spec.loader.exec_module(module)
    return module

def new_sim(mod,inp,outp,rules):
    from skfuzzy import control as ctrl
    return ctrl.ControlSystemSimulation(mod.build_skfuzzy(inp,outp,rules),
                                       clip_to_bounds=False, cache=False,lenient=False)

def compute(sim,x):
    for k,v in zip(('df','di','dd'),x):sim.input[k]=float(v)
    sim.compute()
    v,w=float(sim.output['v']),float(sim.output['omega'])
    if not (math.isfinite(v) and math.isfinite(w) and -.000001<=v<=.200001 and abs(w)<=1.500001):
        raise RuntimeError(f'Salida fuera de dominio: v={v};w={w}')
    return max(0.,min(.2,v)),max(-1.5,min(1.5,w))

def supervisar(df,v,w):
    # Solo barrera numérica independiente de referencia.
    if df<=.35:return 0.,0.,'PARADA_FRONTAL'
    return min(.12,v),max(-.70,min(.70,w)),'LIMITADO'

def summary(values):
    n=len(values)
    if n<2:raise RuntimeError('Muestras insuficientes')
    v=[x['v_s'] for x in values];w=[x['w_s'] for x in values]
    close=[x for x in values if .35<x['df']<=.60]
    stops=[x for x in values if x['df']<=.35]
    return {'muestras':n,
      'v_media_m_s':sum(v)/n,
      'omega_abs_medio_rad_s':sum(map(abs,w))/n,
      'variacion_omega_rad_s':sum(abs(b-a) for a,b in zip(w,w[1:])),
      'salto_omega_max_rad_s':max(abs(b-a) for a,b in zip(w,w[1:])),
      'v_media_cerca_m_s':sum(x['v_s'] for x in close)/len(close) if close else None,
      'n_lecturas_cerca':len(close),'n_paradas_frontal':len(stops),
      'violaciones_parada':sum(abs(x['v_s'])>1e-8 or abs(x['w_s'])>1e-8 for x in stops),
      'violaciones_limites':sum(x['v_s']<0 or x['v_s']>.1200001 or abs(x['w_s'])>.7000001 for x in values),
      'retroceso':sum(x['v_s']<0 for x in values)}

def write_csv(path,rows):
    ensure(bool(rows),'Datos presentes: '+path.name)
    with path.open('w',encoding='utf8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]),extrasaction='ignore',lineterminator='\n')
        writer.writeheader();writer.writerows(rows)

def pct(a,b):
    if a is None or b is None or abs(a)<1e-10:return None
    return 100*(b-a)/abs(a)

def render_report(folder,aggregated,global_metrics,variants):
    def pretty(x,unit=''):
        return 'N/D' if x is None else f'{x:.5f}{unit}'
    h=['# R21 — Comparación A/B del controlador difuso','',
       '**Alcance:** evaluación numérica pareada con scikit-fuzzy 0.5.0; no representa movimiento real del robot ni Gazebo.',
       '','**Controlador A:** Mamdani v01 oficial (SHA de blob Git '+FIS_BLOB+').',
       '**Controlador B:** v02 experimental generado por cambios explícitos en consecuentes; 3 entradas, 2 salidas y 27 reglas intactas.',
       '','## Hipótesis y cambios','',
       'Se busca disminuir amplitud y variación de giro ante incertidumbre, con un movimiento lineal más conservador.',
       'Los resultados pueden refutar la hipótesis y deben reportarse igualmente.','']
    for v in variants:h.append('- '+v[0]+' → '+v[1])
    h.extend(['','## Diseño experimental','',
      f'- 4 escenarios × {REPS} repeticiones × {STEPS} instantes.','- Entrada pareada idéntica para A y B, ruido gaussiano de 0,01 m y semilla fija.',
      '- Criterios supervisor numérico: paro si frontal <=0,35 m; v<=0,12 m/s; |ω|<=0,70 rad/s.',
      '- Las métricas son de salida y suavidad; NO son tasas de éxito, distancias reales ni colisiones.',
      '', '## Comparación por escenario','',
      '| Escenario | ω absoluta A | ω absoluta B | Cambio (%) | Variación ω A | Variación ω B | Cambio (%) |',
      '|---|---:|---:|---:|---:|---:|---:|'])
    for s in aggregated:
        h.append(f"| {s['escenario']} | {s['omega_abs_media_A']:.4f} | {s['omega_abs_media_B']:.4f} | {pretty(s['cambio_omega_pct'])} | {s['variacion_omega_A']:.3f} | {s['variacion_omega_B']:.3f} | {pretty(s['cambio_variacion_pct'])} |")
    h.extend(['','## Global y criterio de aceptación','',
              f"- Velocidad media A/B: {global_metrics['A']['v_media_m_s']:.5f}/{global_metrics['B']['v_media_m_s']:.5f} m/s.",
              f"- Giro absoluto medio A/B: {global_metrics['A']['omega_abs_medio_rad_s']:.5f}/{global_metrics['B']['omega_abs_medio_rad_s']:.5f} rad/s.",
              f"- Variación total de giro A/B: {global_metrics['A']['variacion_omega_rad_s']:.5f}/{global_metrics['B']['variacion_omega_rad_s']:.5f} rad/s (suma de incrementos).",
              f"- Violaciones de parada/fronteras A: {global_metrics['A']['violaciones_parada']}/{global_metrics['A']['violaciones_limites']}; B: {global_metrics['B']['violaciones_parada']}/{global_metrics['B']['violaciones_limites']}.",
              '', '**Interpretación:** si el giro medio o su variación disminuye, solo demuestra una respuesta numérica más suave en estos perfiles. No es evidencia de menor tiempo de viaje, mejores trayectorias o reducción de colisiones.',
              '', '## Pendientes externos a esta batería','',
              '1. Revisar tabla de resultados antes de aceptar o rechazar el rediseño.',
              '2. Integrar la variante en una comparación controlada en Gazebo si se pretende afirmar mejora operacional.',
              '3. Mantener el FIS original como referencia; publicar solo en el cierre consolidado de A2.',
              '4. Contrastar umbrales R19 con enunciado oficial.', ''])
    (folder/'informe_R21.md').write_text('\n'.join(h),encoding='utf8')
    html=['<!doctype html><html lang="es"><meta charset="utf-8"><title>R21 A/B Mamdani</title>',
      '<style>body{font:15px system-ui;max-width:1080px;margin:40px auto;color:#17283b}h1{font-size:27px}table{border-collapse:collapse;width:100%}td,th{border-bottom:1px solid #ccd5dc;padding:8px;text-align:right}td:first-child,th:first-child{text-align:left}.foot{color:#546678} .bars{height:12px;background:#eff2f6;border-radius:8px;overflow:hidden}.bars i{display:block;height:100%;background:#445a78}</style>',
      '<h1>R21 — Rediseño Mamdani A/B</h1><p class="foot">4 perfiles × 5 repeticiones; pruebas numéricas con datos idénticos. No es Gazebo ni movimiento físico.</p>',
      '<table><tr><th>Escenario</th><th>|ω| A</th><th>|ω| B</th><th>Cambio</th><th>Variación A</th><th>Variación B</th></tr>']
    for s in aggregated:
        html.append(f"<tr><td>{s['escenario']}</td><td>{s['omega_abs_media_A']:.4f}</td><td>{s['omega_abs_media_B']:.4f}</td><td>{s['cambio_omega_pct']:+.1f}%</td><td>{s['variacion_omega_A']:.3f}</td><td>{s['variacion_omega_B']:.3f}</td></tr>")
    html.extend(['</table><h2>Interpretación</h2><p>Los cambios observados describen únicamente el controlador (salida limitada por una barrera numérica idéntica). Un valor menor no acredita necesariamente mejor navegación.</p>',
      '<p>Consultar <code>informe_R21.md</code>, <code>resumen_R21.json</code>, <code>comparacion_20_ensayos.csv</code> y <code>observaciones_AB.csv</code>.</p></html>'])
    (folder/'panel_R21.html').write_text('\n'.join(html),encoding='utf8')

def run_worker():
    import skfuzzy
    from skfuzzy import control as ctrl
    preflight()
    mod=parse_engine()
    inp,outp,rules=mod.parse_fis(FIS)
    ensure(len(rules)==27,'27 reglas originales reconstruidas')
    now=dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    folder=ROOT/'resultados/A2_R21_comparacion_Mamdani_AB'/now
    folder.mkdir(parents=True,exist_ok=False)
    original=FIS.read_text(encoding='utf8');v02=variant_text(original)
    edited=folder/'AMR_Mamdani_A2_v02_experimental.fis';edited.write_text(v02,encoding='utf8')
    # La verificación del parser R18 exige el nombre interno v01. Solo este
    # parse temporal usa v01; archivo real v02 conserva su propio nombre.
    with tempfile.TemporaryDirectory() as d:
        tmp=Path(d)/'v02_compatible_parser.fis'
        tmp.write_text(v02.replace("Name='AMR_Mamdani_A2_v02'", "Name='AMR_Mamdani_A2_v01'",1),encoding='utf8')
        inp2,outp2,rules2=mod.parse_fis(tmp)
    ensure(rules==rules2 and inp==inp2,'Reglas y entradas idénticas en A y B')
    ensure(outp!=outp2,'Modificación real de funciones consecuentes')
    sims={'A':new_sim(mod,inp,outp,rules),'B':new_sim(mod,inp2,outp2,rules2)}
    all_rows=[];trials=[];aggregate=[];global_samples={'A':[],'B':[]}
    out('=== R21 — 4 ESCENARIOS × 5 REPETICIONES, PAREJAS A/B ===')
    for scenario in PROFILES:
        for rep in range(1,REPS+1):
            samples={'A':[],'B':[]}
            for i in range(STEPS):
                values=sample(scenario,i,rep)
                for k in ('A','B'):
                    v,w=compute(sims[k],values);vs,ws,reason=supervisar(values[0],v,w)
                    row={'escenario':scenario,'repeticion':rep,'instante':i,
                         'tiempo_s':round(i*STEP_S,2),'modelo':k,
                         'df':values[0],'di':values[1],'dd':values[2],
                         'v_fis':round(v,7),'omega_fis':round(w,7),
                         'v_s':round(vs,7),'w_s':round(ws,7),'barrera':reason}
                    all_rows.append(row);samples[k].append(row);global_samples[k].append(row)
            a=summary(samples['A']);b=summary(samples['B'])
            ensure(a['violaciones_parada']==b['violaciones_parada']==0 and
                   a['violaciones_limites']==b['violaciones_limites']==0,
                   f'Seguridad A/B escenario {scenario} rep {rep}')
            trials.append({'escenario':scenario,'repeticion':rep,
                 'omega_abs_A':a['omega_abs_medio_rad_s'],
                 'omega_abs_B':b['omega_abs_medio_rad_s'],
                 'variacion_omega_A':a['variacion_omega_rad_s'],
                 'variacion_omega_B':b['variacion_omega_rad_s'],
                 'salto_max_A':a['salto_omega_max_rad_s'],
                 'salto_max_B':b['salto_omega_max_rad_s'],
                 'v_media_A':a['v_media_m_s'],'v_media_B':b['v_media_m_s'],
                 'v_cerca_A':a['v_media_cerca_m_s'],'v_cerca_B':b['v_media_cerca_m_s'],
                 'paradas_frontal':a['n_paradas_frontal']})
            out(f"[PAR {len(trials):02d}/20] {scenario} rep {rep}: |w| {a['omega_abs_medio_rad_s']:.3f} -> {b['omega_abs_medio_rad_s']:.3f}, var {a['variacion_omega_rad_s']:.3f} -> {b['variacion_omega_rad_s']:.3f}")
    ensure(len(trials)==20 and len(all_rows)==2*len(PROFILES)*REPS*STEPS,
           '20 parejas, 1440 inferencias y mediciones completas')
    for name in PROFILES:
        t=[x for x in trials if x['escenario']==name]
        def mean(key):return sum(x[key] for x in t)/len(t)
        aa,bb,va,vb=mean('omega_abs_A'),mean('omega_abs_B'),mean('variacion_omega_A'),mean('variacion_omega_B')
        aggregate.append({'escenario':name,'parejas':len(t),
                          'omega_abs_media_A':aa,'omega_abs_media_B':bb,'cambio_omega_pct':pct(aa,bb),
                          'variacion_omega_A':va,'variacion_omega_B':vb,'cambio_variacion_pct':pct(va,vb),
                          'v_media_A':mean('v_media_A'),'v_media_B':mean('v_media_B')})
    global_metrics={k:summary(rows) for k,rows in global_samples.items()}
    # Sumar variación por ensayo; no contar saltos artificiales entre distintas
    # repeticiones/escenarios, que no forman una trayectoria continua.
    for model in ('A','B'):
        col='variacion_omega_'+model
        jump='salto_max_'+model
        global_metrics[model]['variacion_omega_rad_s']=sum(t[col] for t in trials)
        global_metrics[model]['salto_omega_max_rad_s']=max(t[jump] for t in trials)
    # El informe no debe inventar mejoras: solo se considera mejor si aumenta
    # suavidad angular, y se marca que no se ha verificado el objetivo real.
    numeric_improvement=(global_metrics['B']['omega_abs_medio_rad_s']<global_metrics['A']['omega_abs_medio_rad_s']
                       and global_metrics['B']['variacion_omega_rad_s']<global_metrics['A']['variacion_omega_rad_s'])
    manifest={'estado':'COMPARACION_NUMERICA_EJECUTADA_PENDIENTE_VALIDACION_GAZEBO',
      'modelo_original':'AMR_Mamdani_A2_v01.fis','blob_original':FIS_BLOB,
      'modelo_experimental':'AMR_Mamdani_A2_v02_experimental.fis','sha256_v02':sha256(edited),
      'escenarios':len(PROFILES),'repeticiones_por_escenario':REPS,
      'parejas':len(trials),'instantes_por_pareja':STEPS,'inferencias_total':len(all_rows),
      'semilla':SEED,'desviacion_ruido_m':SIGMA,'umbrales_experimentales':{'parada_m':.35,'entrada_difuso_m':.60,'salida_ruta_m':.80},
      'suavidad_numerica_mejorada_en_ambas_metricas':numeric_improvement,
      'metricas':global_metrics,'comparacion_por_escenario':aggregate,
      'cambios_fis':[{'antes':a,'despues':b} for a,b in REPLACEMENTS],
      'alcance':'Sin ROS/Gazebo; no se infiere mejoria en navegación ni acreditacion R19/R20',
      'integridad_git':'No se realizan escrituras en archivos versionados'}
    (folder/'resumen_R21.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    write_csv(folder/'observaciones_AB.csv',all_rows)
    write_csv(folder/'comparacion_20_ensayos.csv',trials)
    write_csv(folder/'resumen_4_escenarios.csv',aggregate)
    render_report(folder,aggregate,global_metrics,REPLACEMENTS)
    ensure(blob(FIS.read_bytes())==FIS_BLOB,'FIS oficial intacto al finalizar')
    ensure(git('rev-parse','HEAD')==BASE_SHA,'No se alteró el commit Git')
    out('=== R21: COMPARACION NUMERICA PAREADA TERMINADA; VALIDACION OPERACIONAL PENDIENTE ===')
    for n in ('resumen_R21.json','comparacion_20_ensayos.csv','panel_R21.html',
              'informe_R21.md','AMR_Mamdani_A2_v02_experimental.fis'):
        out('[EVIDENCIA] '+str(folder/n))
    return 0

def selftest():
    template='''[System]\nName='AMR_Mamdani_A2_v01'\nNumInputs=3\nNumOutputs=2\nNumRules=27\nDefuzzMethod='centroid'\n[Output1]\nMF2='Lenta':'trimf',[0.02 0.07 0.12]\nMF3='Rapida':'trapmf',[0.1 0.15 0.2 0.2]\n[Output2]\nMF1='DerechaFuerte':'trapmf',[-1.5 -1.5 -1.05 -0.65]\nMF2='DerechaSuave':'trimf',[-0.9 -0.5 -0.15]\nMF4='IzquierdaSuave':'trimf',[0.15 0.5 0.9]\nMF5='IzquierdaFuerte':'trapmf',[0.65 1.05 1.5 1.5]\n[Rules]\n1 1 1, 1 3 (1) : 1\n'''
    v=variant_text(template)
    assert v.count('v02')==1 and '[Rules]\n1 1 1' in v
    assert len(PROFILES)==4 and REPS*STEPS*len(PROFILES)==720
    for profile in PROFILES:
        for rep in range(1,REPS+1):
            for t in range(STEPS):
                d=sample(profile,t,rep)
                assert len(d)==3 and all(0<=v<=2 for v in d)
                assert d==sample(profile,t,rep)
                assert supervisar(0.30,.10,.3)==(0.,0.,'PARADA_FRONTAL')
                v,w,_=supervisar(1.2,.19,1.4)
                assert v==.12 and w==.70
    vals=[{'v_s':.04,'w_s':.0,'df':1.2},{'v_s':0.,'w_s':0.,'df':.3}]
    assert summary(vals)['n_paradas_frontal']==1
    assert summary(vals)['violaciones_parada']==0
    out('[OK] Autopruebas: 4 escenarios, 720 entradas pareadas, límites, paro, rediseño trazable.')
    return 0

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    p.add_argument('--autoprueba',action='store_true',help='Solo validación lógica sin inferencia')
    args=p.parse_args()
    selftest()
    if args.autoprueba:return 0
    if args.worker:return run_worker()
    ensure(VENV.is_file(),'Entorno Python R18 localizado')
    r=subprocess.run([str(VENV),'-u',str(Path(__file__).resolve()),'--worker'],cwd=ROOT,check=False)
    return r.returncode

if __name__=='__main__':
    try:sys.exit(main())
    except Exception as exc:
        out('[ERROR R21] '+str(exc))
        traceback.print_exc()
        out('[NOTA] FIS oficial y repositorio Git no se alteran.')
        sys.exit(1)
