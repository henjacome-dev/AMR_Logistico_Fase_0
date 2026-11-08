# R21 — Comparación A/B del controlador difuso

**Alcance:** evaluación numérica pareada con scikit-fuzzy 0.5.0; no representa movimiento real del robot ni Gazebo.

**Controlador A:** Mamdani v01 oficial (SHA de blob Git 6c7dfdd083728d49c64dd1c90a59b19d63222063).
**Controlador B:** v02 experimental generado por cambios explícitos en consecuentes; 3 entradas, 2 salidas y 27 reglas intactas.

## Hipótesis y cambios

Se busca disminuir amplitud y variación de giro ante incertidumbre, con un movimiento lineal más conservador.
Los resultados pueden refutar la hipótesis y deben reportarse igualmente.

- Name='AMR_Mamdani_A2_v01' → Name='AMR_Mamdani_A2_v02'
- MF2='Lenta':'trimf',[0.02 0.07 0.12] → MF2='Lenta':'trimf',[0.02 0.055 0.1]
- MF3='Rapida':'trapmf',[0.1 0.15 0.2 0.2] → MF3='Rapida':'trapmf',[0.11 0.17 0.2 0.2]
- MF1='DerechaFuerte':'trapmf',[-1.5 -1.5 -1.05 -0.65] → MF1='DerechaFuerte':'trapmf',[-1.25 -1.25 -0.9 -0.6]
- MF2='DerechaSuave':'trimf',[-0.9 -0.5 -0.15] → MF2='DerechaSuave':'trimf',[-0.75 -0.38 -0.14]
- MF4='IzquierdaSuave':'trimf',[0.15 0.5 0.9] → MF4='IzquierdaSuave':'trimf',[0.14 0.38 0.75]
- MF5='IzquierdaFuerte':'trapmf',[0.65 1.05 1.5 1.5] → MF5='IzquierdaFuerte':'trapmf',[0.6 0.9 1.25 1.25]

## Diseño experimental

- 4 escenarios × 5 repeticiones × 36 instantes.
- Entrada pareada idéntica para A y B, ruido gaussiano de 0,01 m y semilla fija.
- Criterios supervisor numérico: paro si frontal <=0,35 m; v<=0,12 m/s; |ω|<=0,70 rad/s.
- Las métricas son de salida y suavidad; NO son tasas de éxito, distancias reales ni colisiones.

## Comparación por escenario

| Escenario | ω absoluta A | ω absoluta B | Cambio (%) | Variación ω A | Variación ω B | Cambio (%) |
|---|---:|---:|---:|---:|---:|---:|
| APROXIMACION_FRONTAL | 0.2012 | 0.1672 | -16.87098 | 3.525 | 3.351 | -4.93040 |
| OBSTACULO_IZQUIERDO | 0.6701 | 0.6269 | -6.45458 | 0.952 | 1.144 | 20.20793 |
| OBSTACULO_DERECHO | 0.6722 | 0.6283 | -6.53612 | 0.626 | 0.799 | 27.54568 |
| PASILLO_CAMBIANTE | 0.3240 | 0.2695 | -16.82603 | 4.430 | 4.066 | -8.20692 |

## Global y criterio de aceptación

- Velocidad media A/B: 0.06070/0.05295 m/s.
- Giro absoluto medio A/B: 0.46687/0.42296 rad/s.
- Variación total de giro A/B: 47.66207/46.79914 rad/s (suma de incrementos).
- Violaciones de parada/fronteras A: 0/0; B: 0/0.

**Interpretación:** si el giro medio o su variación disminuye, solo demuestra una respuesta numérica más suave en estos perfiles. No es evidencia de menor tiempo de viaje, mejores trayectorias o reducción de colisiones.

## Pendientes externos a esta batería

1. Revisar tabla de resultados antes de aceptar o rechazar el rediseño.
2. Integrar la variante en una comparación controlada en Gazebo si se pretende afirmar mejora operacional.
3. Mantener el FIS original como referencia; publicar solo en el cierre consolidado de A2.
4. Contrastar umbrales R19 con enunciado oficial.
