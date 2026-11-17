# A2 · Actividad 11 — Diez ensayos nuevos con actuación Mamdani y ROS bag

**Estado:** protocolo y capturador preparados; **no se han ejecutado las diez pruebas nuevas en el equipo del operador**. No confundir con las 20 repeticiones históricas A17 ni con las diez evasiones que el operador comunicó verbalmente.

## 1. Alcance probatorio

Se reutiliza **sin editar** `tools/A3_E426_evasion_real_gazebo.py` de A3, que en Gazebo coloca una caja física temporal, valida `/scan` y tres `Range`, inicia A* más Mamdani v03 bajo supervisión y observa la transición `RUTA → DIFUSO` con comandos aplicados y movimiento real simulado. El ensayo **finaliza al constatar la intervención**, no al completar el rodeo ni al llegar a la meta. No mide contactos mediante sensor de colisión.

El capturador `tools/a2_act11_10_pruebas.py` ejecuta diez sesiones **independientes**: reinicia Gazebo Classic headless en dominio ROS aislado 173, coloca `waffle_pi` en Recepción `(-3.975, -2.775)` con orientación norte, ejecuta E426, genera una ROS bag SQLite3 y guarda las fuentes en una carpeta privada.

**Criterio de auditoría por intento:** FIS v03 SHA256 oficial, objeto real de Gazebo creado y retirado, transición RUTA→DIFUSO, al menos 3 estados DIFUSO posteriores a RUTA, al menos 1 comando aplicado en modo DIFUSO, recorrido ≥0,12 m, sensores dentro de umbral, y una ROS bag con mínimos de muestras. Un intento fallido se conserva y **no cuenta** como evidencia aprobada.

### Diferencia entre niveles de prueba

1. **Intervención difusa verificada:** el controlador Mamdani genera propuestas, el supervisor selecciona modo DIFUSO, el AMR recibe comandos no nulos y avanza cerca de un obstáculo detectado.
2. **Evasión completa verificada:** se acredita, además, que el robot rodeó totalmente el obstáculo, preservó despeje seguro y retomó su ruta o alcanzó meta. **Este protocolo basado en E426 no acredita ese segundo nivel.** El cierre estricto de la actividad 11 requerirá una prueba de rodeo completo (por ejemplo, adaptar B3 v02 y auditar su asistencia geométrica) y su metrología, no únicamente multiplicar eventos RUTA→DIFUSO.

**No atribuible a Mamdani puro:** B3 histórico contó 162 comandos DIFUSO en cinco corridas A17, pero era una navegación *híbrida asistida* con subobjetivos geométricos. B2 no actuó en DIFUSO y B4 usó FIS en sombra. Estos resultados NO son diez evasiones nuevas.

## 2. Preparación de Ubuntu (WSL2)

Cerrar primero todas las instancias de Gazebo, sus lanzadores y los seguidores previos. El capturador **no mata procesos ajenos**; aborta si detecta `gzserver` existente, `/odom` o `/cmd_vel` activos en el dominio 173. No necesita ejecutarse con `sudo`.

```bash
amr iniciar
unzip -o /mnt/c/Users/Henry/Downloads/AMR_A2_Actividad11_10_Pruebas_Mamdani.zip \
  -d ~/AMR_publicacion_u3s7-entrega-final-2026-11-16

python3 tools/a2_act11_10_pruebas.py --self-test
python3 tools/a2_act11_10_pruebas.py --preflight
```

La primera prueba es **sintética, temporal, de lógica de validación**, sin evidencia de Gazebo. La segunda revisa rutas, fuentes y hashes sin movimiento.

Se puede publicar el protocolo antes de mover el robot:

```bash
amr guardar "test(a2): preparar captura de 10 actuaciones Mamdani con ROS bag"
```

## 3. Ejecutar los diez intentos en Gazebo

No se necesita iniciar Gazebo en otra terminal: el capturador lo inicia y cierra **por intento**.

```bash
cd ~/AMR_publicacion_u3s7-entrega-final-2026-11-16
source tools/activar_entorno.sh
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
python3 tools/a2_act11_10_pruebas.py --run --count 10
```

El programa pide confirmación antes de activar movimiento. Cada ensayo guarda los logs incluso cuando falla. Si lo interrumpes con `Ctrl+C`, conserva la evidencia ya capturada y apaga únicamente sus procesos. No renombres ni elimines las carpetas privadas antes de la validación.

**Seguridad:** usar solamente Gazebo; no conectar este ejecutor a un robot físico. La auditoría verifica límites declarados por el supervisor y la geometría de aproximación, pero no certifica cero contactos físicos.

## 4. Evidencia por intento

Carpeta base: `~/AMR_evidencias_privadas/A2_ACT11/sesion_AAAAMMDDTHHMMSSZ/`.

- `manifest_fuentes.json`: commit y SHA-256 de controlador, ruta y mundo.
- `intentos/intento_01` … `intento_10`: `metadata.json`, `gazebo.log`, `spawn.log`, `rosbag.log`, `ejecutor_E426.log`, código de retorno y `captura_E426/` (resumen, trazas y logs de los nodos).
- `rosbag/`: `metadata.yaml` y `.db3` con `/clock`, `/odom`, `/scan`, tres `Range`, `/estado_supervisor`, `/cmd_vel_difuso`, `/estado_difuso`, `/cmd_vel_ruta` y `/estacion_cnn_info`.
- `tabla_10_intentos.csv`, `consolidado.json`, `A2_ACT11_RESULTADO_PUBLICABLE.md`: métricas auditadas, agregados y extracto público.

**Advertencia de diseño:** E426 exige exactamente un suscriptor actuador más su auditor en `/cmd_vel`. Por ello **no se graba `/cmd_vel` con ROS bag durante ese ensayo**: agregar otro suscriptor invalidaría su precondición. El proceso E426 ya cuenta los comandos `/cmd_vel` aplicados y registra los valores del estado del supervisor en su traza. Si se requiere el `/cmd_vel` bruto en bag, se debe desarrollar otra versión del auditor con conteo de suscriptores adecuado y revalidar las condiciones de seguridad.

Opcional: graba video en Gazebo GUI mediante OBS o un grabador de pantalla, con rótulo de intento, reloj y tópico `DIFUSO` visible. **El video no sustituye el CSV ni la ROS bag**.

## 5. Validación y publicación final

Tras el ensayo, copia la ruta `sesion_...` mostrada en la consola y ejecuta:

```bash
python3 tools/a2_act11_10_pruebas.py \
  --check ~/AMR_evidencias_privadas/A2_ACT11/sesion_AAAAMMDDTHHMMSSZ --count 10
```

Si valida diez intervenciones, el resumen indicará `10_INTERVENCIONES_VERIFICADAS_EVASIONES_COMPLETAS_NO_DEMOSTRADAS`. **Ese mensaje no cierra todavía el requisito estricto de diez evasiones completas.**

Para publicar solo el resumen liviano (sin datos brutos privados):

```bash
mkdir -p docs/evidencias_A2/ACT11
cp ~/AMR_evidencias_privadas/A2_ACT11/sesion_AAAAMMDDTHHMMSSZ/A2_ACT11_RESULTADO_PUBLICABLE.md \
  docs/evidencias_A2/ACT11/A2_ACT11_RESULTADO_20261117.md
amr guardar "test(a2): documentar 10 intervenciones Mamdani verificadas en Gazebo"
```

Comprueba la fecha de tu sesión y el contenido antes de publicar. No incluyas bases SQLite3, logs pesados, capturas privadas ni conclusiones de evasión completa no medidas.

## 6. Próxima fase probatoria necesaria

Para cerrar la **actividad 11 de forma estricta** se necesitarán diez ensayos de rodeo completo con: aparición o presencia del obstáculo, activación de Mamdani, desplazamiento alrededor del cuerpo, despeje longitudinal y lateral, retorno a ruta, distancia mínima, temporización y medición de contactos. Si se emplean subobjetivos geométricos, debe indicarse **control híbrido asistido** y no evasión Mamdani exclusiva.
