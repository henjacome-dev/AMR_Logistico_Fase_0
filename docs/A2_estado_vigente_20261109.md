# Avance 2 MEC451 — Estado vigente tras A12–A16 (09/11/2026)

**Controlador oficial: Mamdani FIS v03 para todo el trabajo restante y Avance 3.**
La variante v04 es exclusivamente resultado experimental A13.

**Resultados medidos:** A12 20/20, A13 10/10 pareados, A14 30/30 rosbags y 6/6 replays seguros, A15 600+600 imágenes y 90/90 test sintético, A16 dos paquetes íntegros con permiso de descarga confirmado por el usuario.

**Reserva metodológica:** B2 navega con A* sin actuación difusa efectiva; B3 emplea asistencia geométrica e intervención de Mamdani; B4 usa centrado geométrico y FIS en sombra. No se alega evasión Mamdani pura. CNN no es MobileNetV2 final. La publicación no acredita por sí sola el lanzamiento permanente ni la demostración ante docente.

Evidencia, CSV originales ligeros, hashes y decisión del FIS: [acta A17](evidencias_A2/A17_cierre_20261109/acta_cierre.md).

---

## Estado anterior (archivo histórico, no vigente)

# Avance 2 MEC451 — Estado vigente y alcance (09/11/2026)

**Rama:** `desarrollo-avance2`. **Tag de congelacion:** `avance2` (verificar en GitHub tras publicacion).
**Corte:** D1 con 20/20 llegadas en los escenarios oficiales B1–B4; C1–C4 y D3 auditados.

## Escenarios acreditados

| Escenario | Llegadas | Tipo de control observado | Reserva |
|---|---:|---|---|
| B1 — pasillo 1,0 m | 5/5 | Seguimiento de ruta | Un intento previo fallido excluido |
| B2 — obstaculo estatico | 5/5 | A* con desvio planificado | No acredita evasion difusa reactiva |
| B3 — obstaculo insertado a 1 m | 5/5 | Evasion hibrida asistida, FIS participando | Subobjetivos geometricos temporales |
| B4 — pasillo angosto 0,60 m | 5/5 | Centrado asistido; A3/A4/FIS en sombra | No es actuacion exclusiva del FIS |

- **Contactos fisicos de Gazebo:** no medidos directamente. El despeje geometrico no equivale a ausencia de contactos.
- **Rosbags:** se verifico estructura/presencia de 21 intentos (incluye un B1 fallido), sin reproducir los `.db3`.
- **Evidencias individuales ligeras:** `docs/evidencias_A2/ESCENARIOS_B1_B4/`.
- **Controladores experimentales:** A3/A4 y FIS v03 conservados por SHA256, sin reemplazar el FIS v01 historico.
- **Scripts utilizados:** `tools/a2_ejecutores_validacion/`. Los scripts originales incluyen referencias a la ruta WSL del repositorio.

## Rediseño operacional C1–C4

- Debilidad original B3 v01: 12 cambios registrados de estado A3, 6 entradas DIFUSO,
  5 retornos RUTA, cuatro discrepancias RUTA/LiDAR y detencion `COLISION_PROYECTADA`.
- Antes: 0/1 llegada, despeje 0,2601 m, ventana comun de 0,60 m en 8,7804 s,
  giro absoluto medio 0,1315 rad/s.
- Despues: 5/5 llegadas, despeje medio 0,3133 m, ventana en 10,4563 s
  (**+19,09 %**), giro absoluto medio 0,1145 rad/s (**-12,91 %**).
- El +20,45 % de despeje y el aumento de llegadas describen una **mejora observada
  del sistema hibrido asistido**, no una mejora aislada de reglas ni de membresias Mamdani.
- R21 contiene una comparacion FIS v01/v02 **numerica** separada con resultados mixtos:
  menor magnitud de giro pero mayor variacion angular en casos laterales.
- La rubrica solicita ajustar reglas o membresias y repetir escenario. Los resultados
  disponibles NO prueban operacionalmente ese cambio de forma aislada.

## Adelanto CNN

- Dataset sintetico: 600 PNG RGB 640x360 de cuatro estaciones; 150 por clase.
- Entrenamiento: padding 224x224, particion por grupos 420/90/90, CNN propia
  de 28.004 parametros durante ocho epocas, 90/90 aciertos en test sintetico.
- Modelo **NO es MobileNetV2 final**. No hay validacion externa en otra camara/mundo.
- SHA256 del ZIP distribuido separadamente:
  `9ac794dc4615254c9373f55c17969b495a581b1fb594552892c36d3beeaf7434`.
- Reportes ligeros y mosaico: `docs/evidencias_A2/R22_R23/`.
- La accesibilidad externa del ZIP fue confirmada por el grupo; este repositorio
  NO contiene el ZIP pesado. No se incorpora una URL no verificada.

## MATLAB, integracion y siguientes pasos

- Mamdani: 3 entradas (0–2 m), 2 salidas (v y omega), 27 reglas.
- Concordancia MATLAB/Python: 10/10 entradas; errores maximos inferiores al 2 %.
- Superficies: `matlab/A2_MAMDANI/evidencias/`.
- Reserva de integracion: las demostraciones A3/A4 de las pruebas B se realizaron
  con puentes experimentales; revisar lanzamiento operacional permanente.
- El informe academico D4 de dos paginas se valida **despues** de D5–D6 y de la
  auditoria final D1, de acuerdo con el orden de trabajo solicitado. El tag de
  codigo/evidencias puede anteceder a la version final del informe.

**No confundir 20/20 llegadas simuladas con satisfaccion completa de la rubrica
ni con la entrega del informe o demostracion.**
