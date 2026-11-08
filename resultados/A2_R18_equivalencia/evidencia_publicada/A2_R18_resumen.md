# Avance 2 — Etapa 2.1: equivalencia MATLAB–Python (R18)

**Estado:** APROBADO; 10/10 casos.
**Fecha local:** 2026-11-07T19:03:24.
**FIS de referencia:** `matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis` (Git blob `6c7dfdd083728d49c64dd1c90a59b19d63222063`).
**CSV MATLAB:** `matlab/A2_MAMDANI/evidencias/A2_F1_casos_criticos.csv` (Git blob `5a49f16e1c4c7dc7c5428df53a7fd3480dea591e`).
**Biblioteca:** scikit-fuzzy 0.5.0; Python 3.10.12; NumPy 1.26.4; SciPy 1.11.4.

## Método

- 27 reglas copiadas directamente del FIS, AND=min, OR=max, implicación=min, agregación=max, defuzzificación=centroide.
- Universos de salida con 1001 puntos, como en la auditoría MATLAB. scikit-fuzzy puede refinar numéricamente el centroide, por lo que no se exige identidad bit a bit.
- Para referencias de magnitud >=1e-6 se exige error relativo **estrictamente menor al 2 %**, para cada salida en cada caso.
- Para referencias próximas a cero (<1e-6) el error relativo no es significativo: se exige error absoluto <=1e-5 m/s (v) o <=1e-4 rad/s (omega).
- Se registran también errores absolutos y porcentajes normalizados por el rango de salida.

**Máximo error relativo v:** 0.502821 %.
**Máximo error relativo omega:** 0.145568 %.
**Máximo error absoluto v:** 5.94112769e-05 m/s.
**Máximo error absoluto omega:** 0.000940595574 rad/s.

## Resultados

| Escenario | v MATLAB | v Python | omega MATLAB | omega Python | Cumple |
|---|---:|---:|---:|---:|:---:|
| PasilloLibre | 0.16116298 | 0.16111111 | +0.00000000 | +0.00000000 | Sí |
| ObstaculoIzquierdo | 0.07000000 | 0.07000000 | -0.51666691 | -0.51666691 | Sí |
| ObstaculoDerecho | 0.07000000 | 0.07000000 | +0.51666691 | +0.51666691 | Sí |
| FrenteIzquierdaLibre | 0.01181559 | 0.01187500 | +1.16551580 | +1.16474161 | Sí |
| FrenteDerechaLibre | 0.01181559 | 0.01187500 | -1.16551580 | -1.16474161 | Sí |
| TresLadosCerca | 0.01181559 | 0.01187500 | +0.00000000 | +0.00000000 | Sí |
| FrenteCercaLadosMedios | 0.01181559 | 0.01187500 | +1.16551580 | +1.16474161 | Sí |
| FrenteCercaLadosLejos | 0.01181559 | 0.01187500 | +1.16551580 | +1.16474161 | Sí |
| DistanciaFrontalUmbral | 0.06044835 | 0.06048760 | +0.64615767 | +0.64521707 | Sí |
| DistanciaLateralUmbral | 0.07000000 | 0.07000000 | -0.51689808 | -0.51689815 | Sí |

## Limitaciones y continuidad

- La comparación usa resultados de MATLAB previamente aprobados, no ejecuta MATLAB Online de nuevo.
- No acredita integración ROS 2, evasión de obstáculos ni seguridad operacional.
- En `df=di=dd=0.2 m`, la salida Mamdani puede mantener v>0. La parada estricta debe imponerse mediante el supervisor.
- No modificar el FIS `v01`; preparar la integración ROS 2 solamente después de aprobar esta prueba.
