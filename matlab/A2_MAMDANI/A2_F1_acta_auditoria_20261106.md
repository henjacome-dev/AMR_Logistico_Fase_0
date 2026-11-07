# Acta de auditoría — Avance 2, Fase 1

**Fecha:** 6 de noviembre de 2026  
**Entorno:** MATLAB Online  
**Repositorio:** `henjacome-dev/AMR_Logistico_Fase_0`  
**Rama:** `desarrollo-avance2`  
**Commit de reparación de evidencias:** `6b46367c16d07540957022e14e0eeade340d7974`

## Evidencia verificada

Conforme a la salida de `A2_F1_verificacion_auditoria.m` mostrada por el estudiante en MATLAB Online:

| Comprobación | Resultado |
|---|---|
| Archivos técnicos básicos | Presentes |
| Tres MAT | Cargados con `load()`; variables internas comprobadas |
| FIS | Tres entradas, dos salidas, 27 reglas y 27 antecedentes distintos |
| CSV | Concordancia con MAT: barrido de 1.331 entradas y diez casos críticos |
| Reproducción `evalfis` del barrido | Error máximo mostrado: 0 |
| Reproducción `evalfis` de casos críticos | Error máximo mostrado: 0 |
| PNG | Nueve imágenes leídas y verificadas |

El error máximo reportado como cero corresponde a los datos evaluados por el script; no significa una prueba de todos los puntos posibles del universo de entrada.

## Condición crítica y limitaciones

Caso de tres obstáculos próximos (`df=di=dd=0,2 m`):

- `v = 0,011815589 m/s`.
- `omega = 0 rad/s`.

**La inferencia Mamdani por sí sola no garantiza parada estricta.** La lógica de seguridad corresponde al supervisor ROS 2 y requiere pruebas separadas.

Esta auditoría no acredita saturación física de actuadores, ejecución integrada en ROS 2, equivalencia con `scikit-fuzzy`, navegación sin colisiones en Gazebo ni robustez en hardware real.

## Dictamen

**Fase 1 validada en diseño numérico, reproducción MATLAB e integridad de evidencias.** El cierre documental robustecido fue probado en MATLAB Online y versionado posteriormente en el commit `3f0acc7`: nueve PNG legibles y un ZIP verificado de 24 archivos. Las auditorías posteriores se registran en el anexo cronológico siguiente. R15 y R16 quedan validados a nivel de diseño; R17 se mantiene parcial por la validación operacional pendiente. R18–R21 continúan pendientes.

El archivo `AMR_Mamdani_A2_v01.fis` debe conservarse sin modificaciones.

## Anexo de seguimiento — 7 de noviembre de 2026

Este anexo documenta actividades posteriores sin alterar el dictamen histórico del 6 de noviembre:

- **AC-02, completada:** las dos copias originales de `A2_F1_membresias_base.mat` tenían SHA-256 distintos, pero coincidían técnicamente en MATLAB. El ZIP final incorporó la copia versionada en GitHub para asegurar identidad binaria de paquete y repositorio (`54abfd3`); la copia local alternativa no se alteró.
- **AC-01, completada y publicada:** nueve PNG de 900 × 650 píxeles, legibles, inspeccionados visualmente y generados con `getframe`/`imwrite`. Los generadores y las evidencias oficiales actualizadas se publicaron en el commit `36ce917`.
- **AC-05, completada y publicada:** cuatro superficies incluyen explícitamente la entrada fija (`dd=1.7 m`, `df=0.2 m`, `df=1.7 m`, `di=0.9 m`); se publicaron en el commit `36ce917`.
- **AC-03, completada y publicada:** ZIP definitivo de 25 archivos, con esta acta y manifiesto SHA-256. Reconciliación binaria con el repositorio en `54abfd3`; GitHub Actions `validar-avance2` aprobó ambos verificadores (run `42426641603`).

Las evidencias AC-01/AC-05 dejaron de ser candidatas y están publicadas en GitHub. Esta actualización documental sincroniza el acta, el README y el ZIP sin modificar el FIS `AMR_Mamdani_A2_v01.fis`, los MAT, las reglas ni los PNG. La validación corresponde al diseño numérico; R18–R21 y la seguridad operativa continúan pendientes.
