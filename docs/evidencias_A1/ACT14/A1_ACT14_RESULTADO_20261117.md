# A1 — ACT14: Reproducción de la orden A

**Estado: APROBADA en el ensayo posterior.**

Esta evidencia corresponde a una nueva ejecución posterior del cierre F1 del 6 de noviembre de 2026. **No constituye una modificación ni una prueba retroactiva del ZIP presentado el 4 de noviembre.**

| Indicador | Resultado |
|---|---:|
| Llegada reportada por el seguidor | Sí (código de salida 0) |
| Estaciones visitadas | 4/4 en secuencia |
| Longitud de ruta A* | 20,200 m; 405 puntos |
| Tiempo del log de seguimiento | 272.737 s (tiempo monotónico del proceso) |
| Recorrido odométrico registrado | 20.020 m |
| Error final reportado | 0.078 m |
| Muestras del seguimiento | 2748 |
| ROS bag disponible localmente | Sí |
| Contactos físicos | No verificados con sensor independiente |

## Proximidad a las estaciones

| Estación | Proximidad mínima (m) | Tiempo del log (s) |
|---|---:|---:|
| Recepción | 0.0272 | 17.822 |
| Almacenamiento | 0.0277 | 92.760 |
| Empaque | 0.0263 | 199.713 |
| Despacho | 0.0819 | 272.737 |

**Trazabilidad:** referencia histórica `resultados/F1_A_cierre_20261106/`; protocolo `docs/A1_ACT14_PROTOCOLO_REPRODUCCION.md`. El ROS bag, el video y los logs íntegros permanecen fuera del repositorio público. El movimiento se realizó con seguidor F1 y supervisor, no con el controlador difuso ni la CNN.
