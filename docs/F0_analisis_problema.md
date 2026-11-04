# Borrador técnico inicial — tareas 1.1 y 1.2 del Avance 1

## Formalización de entradas y salidas

**Entradas/Percepción:** tres distancias (`d_f`, `d_i`, `d_d`) con alcance emulado de 0–2 m; pose y velocidad estimadas mediante `/odom`; imágenes de `/camera/image_raw`.

**Salidas/Actuación:** consigna de velocidad lineal `v` [0, 0.20] m/s y angular `omega` [-1.5, 1.5] rad/s para un robot de tracción diferencial, bajo REP-103 (positivo izquierda).

**Condiciones del entorno:** mapa conocido de almacén con cuatro estaciones y al menos tres pasillos de 1 m. Huella conservadora con radio circunscrito de 0.21 m; planificador en cuadrícula de resolución 0.05 m o mejor.

**Incertidumbres:** ruido de sensores (simulado con sigma 0.01 m), obstáculos no representados en el mapa, deslizamiento de ruedas/errores de odometría, variaciones de ángulo e iluminación para reconocimiento de señales.

## Comparación de enfoques
| Enfoque | Función | Ventaja | Limitación |
|---|---|---|---|
| Simbólico (A*) | Buscar rutas óptimas respecto a mapa/costo especificados | Determinista y auditable con mapa conocido | No resuelve por sí solo obstáculos inesperados |
| Difuso (Mamdani) | Regular `v` y `omega` a partir de distancias ruidosas | Reglas interpretables y respuesta gradual | Depende de diseño de reglas y ajustes; no planifica rutas globales |
| Conexionista (CNN) | Clasificar marcador visual de estación | Aprende características de imagen y tolera variaciones entrenadas | Requiere datos representativos, entrenamiento y validación |

**Justificación de hibridación:** A* elige el trayecto, Mamdani gestiona reacción local bajo incertidumbre y CNN valida que el destino observado corresponde con el solicitado. ROS 2 integra mensajes y decisiones entre ellos.

**Nota:** Este borrador aún necesita resultados, diagramas y referencias antes de convertirse en las dos páginas finales del Avance 1.
