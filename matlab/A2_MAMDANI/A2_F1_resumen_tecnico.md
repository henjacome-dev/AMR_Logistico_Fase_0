# Avance 2 - Fase 1: Diseno Mamdani

Fecha: 2026-11-06
Estado: validado numericamente en MATLAB Online.

## Configuracion
- Tipo: Mamdani.
- Entradas: df, di, dd (0 a 2 m).
- Salidas: v (0 a 0.20 m/s) y omega (-1.5 a 1.5 rad/s).
- Operadores: AND=min, OR=max, implicacion=min.
- Agregacion=max; defuzzificacion=centroide.
- Convencion REP-103: omega positivo gira a izquierda.

## Resultados
- 27 de 27 combinaciones cubiertas.
- 1331 entradas evaluadas sin NaN ni Inf.
- Velocidad observada: 0.011816 a 0.161163 m/s.
- Giro observado: aproximadamente +/-1.165516 rad/s.
- Pruebas criticas y direcciones de giro satisfactorias.
- Cuatro superficies generadas.

## Limitaciones
- Tres lados cercanos producen v=0.011816 m/s.
- El controlador difuso no garantiza parada absoluta.
- La seguridad estricta debe implementarse en el supervisor.
- La ausencia de colisiones requiere ensayos en Gazebo.
- Concordancia MATLAB-Python: pendiente de Fase 2.

## Version
- AMR_Mamdani_A2_v01.fis: prototipo validado de Fase 1.
- No se han modificado los artefactos del Avance 1.
