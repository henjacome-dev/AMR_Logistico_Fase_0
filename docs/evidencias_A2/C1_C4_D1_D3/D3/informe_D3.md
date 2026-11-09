# D3 — Integracion de resultados del rediseno operacional

**Fecha UTC:** 2026-11-09T02:55:57.215700+00:00
**Estado:** C1–C4 integrados; dictamen descriptivo con reservas.
**Procedencia:** dictamen C4 validado por SHA256 y auditoria D1 de 20/20 llegadas.

## Diagnostico

B3 v01 evidencio conmutaciones inestables RUTA/DIFUSO tras insertar una caja 1 m delante del robot, con detencion por COLISION_PROYECTADA antes de completar el recorrido. B3 v02 introdujo retencion de la evasión, parada/giro preventivo y desvio geometrico temporal. El FIS Mamdani participo, pero la maniobra no dependio exclusivamente de el.

## Matriz consolidada de la intervencion

| Indicador | Antes B3 v01 (n=1) | Despues B3 v02 (n=5) | Cambio |
|---|---:|---:|---:|
| Llegadas documentadas | 0/1 | 5/5 | No comparable estadisticamente |
| Paradas preventivas | 1 | 0 observadas | Intervencion compuesta |
| Despeje minimo por corrida | 0,2601 m | 0,3133 m (media) | 0,0532 m |
| Tiempo, ventana comun 0,60 m | 8,7804 s | 10,4563 s | +19,09% |
| Giro absoluto medio, ventana comun | 0,1315 rad/s | 0,1145 rad/s | -12,91% |

## Texto listo para el informe de dos paginas (D4)

Se identificó como debilidad principal la conmutación inestable entre RUTA y DIFUSO ante un obstáculo introducido durante el movimiento (B3). La configuración anterior terminó con una detención preventiva por COLISION_PROYECTADA (0/1 llegadas), mientras que la intervención híbrida asistida, con retención evasiva y subobjetivos geométricos temporales, completó cinco recorridos (5/5). El despeje geométrico mínimo por ejecución pasó de 0,2601 a 0,3133 m de media; en una ventana común de 0,60 m odométricos, el tiempo aumentó de 8,7804 a 10,4563 s (+19,09%) y el giro absoluto medio descendió de 0,1315 a 0,1145 rad/s (-12,91%). La comparación es descriptiva (1 frente a 5 ensayos), no pareada, y no permite atribuir la mejora exclusivamente a Mamdani. No se midieron contactos físicos ni se reprodujeron los rosbags.

## Alcance y reservas

- B1–B4 acreditan 20/20 **llegadas experimentales** en D1, no cumplimiento integral por si solo.
- B3 = hibrido asistido; B4 = centrado geometrico asistido con controladores en sombra.
- El minimo de despeje anterior corresponde a una corrida; el posterior es media de cinco minimos.
- Tiempo y giro se comparan en una ventana comun de 0,60 m odometricos desde la insercion.
- No es un A/B pareado, aleatorizado ni una ablacion de Mamdani.
- Los contactos fisicos de Gazebo no se midieron; los rosbags se verificaron por presencia, no por replay.
- R21 (comparacion numerica entre FIS) constituye una evaluacion distinta y de resultados mixtos.

## Trazabilidad

- C4: `/home/enry/AMR_A2_entrega_local/C4_cierre_20261109_022639_UTC/dictamen_C4.json`
- D1: `/home/enry/AMR_A2_entrega_local/D1_auditoria_integral_20261109_024440_006886_UTC/dictamen_D1.json`
- Registros verificados contra manifiesto C4: 32
- D1: 20/20, bloqueos 0, revisiones manuales 10.

## Paso siguiente

D4: incorporar el texto sintetico anterior y las metricas seleccionadas en el informe final de dos paginas. D3 no publica GitHub, crea tags ni envia Blackboard.
