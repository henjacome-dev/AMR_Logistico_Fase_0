# Avance 2 — Concordancia MATLAB–Python (R18)

**Estado:** aprobado en Ubuntu/WSL2, pendiente de verificación remota si aún no se publicó.
**FIS de referencia:** `AMR_Mamdani_A2_v01.fis` (inalterado).
**Motor Python:** `scikit-fuzzy` 0.5.0, Python 3.10, dependencias fijadas en `tools/requirements_a2_r18.txt`.

## Método y resultados

Se reconstruyeron las 27 reglas desde el FIS oficial sin editar el archivo de MATLAB.
Se evaluaron diez escenarios frente a `A2_F1_casos_criticos.csv`; criterio estricto menor al 2% por salida,
con tolerancias absolutas documentadas para referencias próximas a cero.

| Métrica | Resultado | Umbral |
|---|---:|---:|
| Casos aprobados | 10 de 10 | 10 de 10 |
| Máximo error relativo de v | 0.502821% | <2% |
| Máximo error relativo de omega | 0.145568% | <2% |

Evidencias seleccionadas: `resultados/A2_R18_equivalencia/evidencia_publicada/A2_R18_comparacion_10_casos.csv` y
`resultados/A2_R18_equivalencia/evidencia_publicada/A2_R18_resumen.md`.

## Reproducción

```bash
python3 tools/A2_E21_equivalencia_MATLAB_Python.py
```

La ejecución crea su entorno virtual aislado (requiere `python3-venv`) y asegura todas las dependencias fijadas.
GitHub Actions ejecuta también `--worker` con Python 3.10 y las dependencias explícitas.

**Limitaciones:** esta prueba no acredita integración ROS 2, seguridad ante colisiones ni parada estricta.
Con tres lados cercanos el Mamdani v01 puede producir velocidad positiva; R19 exige supervisor de seguridad.
