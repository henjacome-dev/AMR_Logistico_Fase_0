# R05 — Marcadores visuales de estaciones

Fecha: 2026-11-05
Rama: correccion-avance1-r04

## Implementacion

Se incorporaron cuatro texturas PNG de 512 x 512
pixeles mediante materiales OGRE para Gazebo Classic.

Estaciones:
- RECEPCION: azul, identificador I.
- ALMACENAMIENTO: rojo, identificador II.
- EMPAQUE: verde, identificador III.
- DESPACHO: amarillo, identificador IV.

Los paneles mantienen sus posiciones originales.
No se añadieron colisiones.

## Validacion estatica

- Cuatro PNG encontrados y vinculados al SDF.
- Cuatro materiales correspondientes.
- Mundo con 26 modelos propios.
- Cuatro paneles sin colision.
- Mapas de ocupacion coherentes con la geometria.
- Sin advertencias de ancho de pasillos.

Evidencia:
resultados/R05_validacion_estatica.txt

## Validacion visual

Se inicio Gazebo Classic 11.10.2 con TurtleBot3
Waffle Pi. El modelo se inserto correctamente.

Las capturas permiten observar los cuatro
marcadores con sus identificadores visuales.

Evidencias:
- resultados/R05_capturas/R05_gazebo_recepcion_almacenamiento.png
- resultados/R05_capturas/R05_gazebo_empaque_despacho.png

## Alcance

R05: implementacion estatica y visual satisfactoria.

No se ejecutaron nuevas pruebas de navegacion.
