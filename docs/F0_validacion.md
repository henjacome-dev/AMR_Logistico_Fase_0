# Fase 0 — Protocolo de validación

**Estado:** pendiente hasta ejecutarse en el equipo del grupo.

## F0.1 · Requisitos
- [ ] Matriz revisada contra enunciado.
- [ ] Fechas y pesos comprendidos.
- [ ] Alcance para mañana (Avance 1) asignado a responsables.

## F0.2 · Entorno
- [x] Ubuntu 22.04 validado.
- [x] ROS_DISTRO=humble.
- [x] Gazebo abre sin errores críticos.
- [x] TurtleBot3 Waffle Pi aparece y se mueve por teclado.
- [x] `/odom` publica mensajes.
- [x] `/scan` publica mensajes.
- [x] `/camera/image_raw` existe y publica imágenes.
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
| 2026-11-04 | TurtleBot3 en Gazebo | Aprobado: 2 arranques consecutivos | Fallo inicial no reproducido | Lider | Continuar monitoreo |


## Evidencias técnicas verificadas — entorno local

### F0.2 — Entorno robótico
- Sistema: Ubuntu 22.04 LTS sobre WSL2.
- ROS 2: Humble.
- Robot: TurtleBot3 Waffle Pi.
- Gazebo Classic: escenario cargado correctamente.
- Inserción automática: validada en dos arranques consecutivos.
- Movimiento mediante teclado: comprobado.
- Odometría: `/odom` operativo.
- LiDAR: `/scan` operativo.
- Cámara: `/camera/image_raw` operativo.

### F0.3 — Organización del proyecto
- Repositorio Git local inicializado en rama main.
- Commit inicial: 9df1077.
- Commit de automatización: c08d0c4.
- Archivo .gitignore revisado.
- Script `tools/activar_entorno.sh` probado.
- Integración con `~/turtlebot3_ws` verificada.
- Espacio `ros2_ws/src` preparado para código propio.

### Observaciones y pendientes
- El fallo inicial de inserción automática no se reprodujo
  durante las dos pruebas posteriores.
- MATLAB y toolboxes pendientes de validación.
- Repositorio remoto y colaboración pendientes.
- Verificaciones documentales e interfaces aún por completar.
- Las capturas existen como evidencia de las pruebas,
  pero todavía deben archivarse en el repositorio.
