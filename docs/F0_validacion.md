# Fase 0 — Protocolo de validación

**Estado al 2026-11-06:** validaciones técnicas ejecutadas; cierre organizativo y acuerdos grupales pendientes. Las auditorías fechadas anteriormente son registros históricos.

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
- [x] MATLAB Online utilizado para ejecutar `binaryOccupancyMap`, `inflate` y `plannerAStarGrid`; instalación local no acreditada.

## F0.3 · Organización
- [x] Repositorio Git iniciado y remoto configurado.
- [x] README actualizado con pasos reproducibles.
- [x] Estructura oficial de seis carpetas creada.
- [ ] Todos los integrantes tienen permisos y pueden clonar.
- [ ] Evitar subir datos pesados innecesarios o claves al Git.

## F0.4 · Interfaces
- [ ] Diagrama consensuado.
- [ ] Nombres y tipos de tópicos fijados.
- [ ] Convención de giro y ejes revisada con el grupo.
- [x] Responsable de publicación final `/cmd_vel` definido.
- [x] CSV `x_m,y_m` implementado y validado; acuerdo formal del grupo no acreditado.

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
- Repositorio remoto configurado; accesos de colaboradores pendientes de verificacion.
- Verificaciones documentales e interfaces aún por completar.
- Las capturas existen como evidencia de las pruebas,
  pero todavía deben archivarse en el repositorio.

## Evidencias archivadas - F0.5

Fecha: 2026-11-04.

- `resultados/F0_diagnostico.txt`: versiones y paquetes instalados.
- `resultados/F0_ros_topics.txt`: interfaces ROS 2 y publicadores.
- `resultados/F0_gazebo_waffle_pi.png`: captura de simulacion.

### Resultados tecnicos

- ROS 2 Humble y Gazebo Classic 11.10.2 disponibles.
- TurtleBot3 Waffle Pi identificado.
- `/scan`, `/odom` y `/camera/image_raw`: un publicador cada uno.
- `/cmd_vel`: un suscriptor y cero publicadores durante la captura.
- Movimiento y sensores comprobados en pruebas anteriores.

### Limitaciones pendientes

- Verificar visualmente el contenido de la imagen archivada.
- MATLAB y toolboxes sin validacion.
- Acuerdos y aprobaciones grupales pendientes.
- Seguidor proporcional y supervisor de seguridad implementados; otros modulos autonomos pendientes.
- Umbrales de seguridad pendientes de calibracion.

## Auditoria de cierre F0 — 2026-11-05

### Estado del repositorio
- Rama main sincronizada con origin/main.
- Repositorio remoto operativo.
- No existen etiquetas Git.
- Existen registros experimentales locales pendientes de clasificacion.
- No se eliminaran evidencias ni respaldos sin revision.

### Estado de MATLAB
- MATLAB no esta instalado en el entorno Ubuntu consultado.
- Existen scripts de planificacion A* en matlab/F124_MATLAB/.
- MATLAB Online se mantiene como entorno de trabajo.
- Pendiente: consolidar evidencia de ejecucion y toolboxes.

### Documentacion
- Analisis, requisitos, arquitectura, entorno, roles y validacion disponibles.
- Pendiente: actualizar estados de requisitos conforme a resultados.
- Pendiente: confirmar asignacion y acceso de integrantes.
- Pendiente: consolidar evidencias de la Etapa 1.

### Condicion de cierre
La Etapa 0 permanece en cierre documental.
No se declara completada al 100 por ciento hasta resolver
las verificaciones y acuerdos pendientes.

## Consolidacion tecnica — 2026-11-05

- GitHub sincronizado con la rama main.
- Ultimo commit confirmado de GI-03: d616273.
- Planificacion A* y seguimiento de rutas con evidencias.
- Seguridad logica S01-S12: 12/12 aprobadas.
- Pruebas integradas GI-01, GI-02 y GI-03 ejecutadas.
- Las evidencias corresponden al entorno simulado.
- MATLAB Online utilizado; disponibilidad local de toolboxes
  pendiente de documentacion.
- Acceso de integrantes y acuerdos grupales pendientes.

La Etapa 0 conserva pendientes documentales y organizativos.

## Actualización técnica — 2026-11-06

Las afirmaciones de auditorías anteriores corresponden
a sus respectivas fechas y no al estado actual.

- Existen las etiquetas `avance1` y `avance1-v2`.
- La rama de corrección contiene las mejoras R04,
  R05, R07, R08 y R11.
- MATLAB Online ejecutó la planificación y el inflado.
- R04: geometría corregida y validada estáticamente.
- R05: texturas de estaciones verificadas en Gazebo.
- R07/R08: ensayo sintético del emulador satisfactorio.
- R11: inflado MATLAB y planificación A* ejecutados.
- Tarea 1.7: protocolo y 600 capturas planificadas;
  ninguna captura real acreditada todavía.
- R22/R23: pendientes.
- R14: pendiente de confirmar la entrega del informe.
- Responsables, accesos y acuerdos grupales:
  pendientes de acreditación.

La geometría corregida no cuenta todavía con una
nueva batería completa de navegación dinámica.

La Fase 0 mantiene pendientes organizativos.
No se declara cerrada al 100 %.
