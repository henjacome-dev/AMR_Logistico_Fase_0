# Fase 0 — Protocolo de validación

**Estado:** pendiente hasta ejecutarse en el equipo del grupo.

## F0.1 · Requisitos
- [ ] Matriz revisada contra enunciado.
- [ ] Fechas y pesos comprendidos.
- [ ] Alcance para mañana (Avance 1) asignado a responsables.

## F0.2 · Entorno
- [ ] Ubuntu 22.04 validado.
- [ ] ROS_DISTRO=humble.
- [ ] Gazebo abre sin errores críticos.
- [ ] TurtleBot3 Waffle Pi aparece y se mueve por teclado.
- [ ] `/odom` publica mensajes.
- [ ] `/scan` publica mensajes.
- [ ] `/camera/image_raw` existe y publica imágenes.
- [ ] MATLAB y toolboxes requeridas localizadas o alternativa institucional coordinada.

## F0.3 · Organización
- [ ] Repositorio Git iniciado y remoto compartido.
- [ ] README actualizado con pasos reproducibles.
- [ ] Estructura oficial de seis carpetas creada.
- [ ] Todos los integrantes tienen permisos y pueden clonar.
- [ ] Evitar subir datos pesados innecesarios o claves al Git.

## F0.4 · Interfaces
- [ ] Diagrama consensuado.
- [ ] Nombres y tipos de tópicos fijados.
- [ ] Convención de giro y ejes revisada con el grupo.
- [ ] Responsable de publicación final `/cmd_vel` definido.
- [ ] CSV de waypoints definido antes de programar MATLAB/ROS.

**Puerta F0 — Aprobación:** 100 % de verificaciones funcionales críticas (ROS, robot en Gazebo y tópicos) y acuerdos de arquitectura. El resto de tareas documentales puede avanzar en paralelo por urgencia del Avance 1.

## Evidencia sugerida
- `resultados/f0_diagnostico.txt`
- `resultados/f0_gazebo_waffle_pi.png`
- `resultados/f0_ros_topics.txt`
- Captura o exportación de configuración MATLAB

## Registro de problemas
| Fecha | Prueba | Resultado | Error | Responsable | Acción correctiva |
|---|---|---|---|---|---|
| Pendiente | TurtleBot3 en Gazebo | Por ejecutar | — | Líder | — |
