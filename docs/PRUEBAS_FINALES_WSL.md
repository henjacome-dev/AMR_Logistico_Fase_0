# Pruebas finales del AMR en Ubuntu 22.04

## Activar Ubuntu en Windows

La aplicación Ubuntu 22.04 ya está instalada. Abre PowerShell como administrador y ejecuta:

```powershell
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```

Reinicia Windows cuando puedas. Luego actualiza WSL, configura WSL2 y abre
**Ubuntu 22.04 LTS** desde Inicio para crear el usuario y la contraseña de Linux.

```powershell
wsl --update
wsl --set-default-version 2
```

Comprueba con `wsl -l -v` que Ubuntu aparece con versión 2. Si `wsl --update` no
está disponible, usa la instalación manual oficial de Microsoft:
https://learn.microsoft.com/es-es/windows/wsl/install-manual

## Preparar ROS y Gazebo

El script de instalación configura el repositorio oficial de ROS 2, actualiza
Ubuntu 22.04 e instala ROS 2 Humble, Gazebo Classic y TurtleBot3. Desde Ubuntu:

```bash
cd /mnt/c/Users/panch/OneDrive/Documentos/ChatGPT/AUTONOMASYEXPERTOS/AMR_ramas_editables/integracion-final
bash tools/instalar_ros_humble.sh
```

Referencia: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Si apt no encuentra `ros-humble-turtlebot3-gazebo`, compila las simulaciones en
un workspace adicional con la rama Humble según el manual de ROBOTIS:
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

En la terminal de Ubuntu abre la carpeta local con las correcciones:

```bash
cd /mnt/c/Users/panch/OneDrive/Documentos/ChatGPT/AUTONOMASYEXPERTOS/AMR_ramas_editables/integracion-final
python3 -m venv --system-site-packages ~/.venvs/amr-a2-r18
~/.venvs/amr-a2-r18/bin/python -m pip install -r tools/requirements_a2_r18.txt
python3 -m venv ~/.venvs/amr-cnn
~/.venvs/amr-cnn/bin/python -m pip install 'tensorflow==2.16.2' 'numpy<2' matplotlib pydot
```

Comprueba el modelo antes de iniciar la simulación:

```bash
~/.venvs/amr-cnn/bin/python -c "import tensorflow as tf; m=tf.keras.models.load_model('modelos/modelo_estaciones_mobilenetv2_compat.keras',compile=False); print(m.input_shape,m.output_shape)"
```

## Primera prueba de integración

Para comprobar primero el arranque sin ventana ni movimiento:

```bash
bash tools/verificar_gazebo_local.sh
```

Esta comprobación guarda los mensajes de odometría, LiDAR y cámara en
`resultados/verificacion_gazebo_*/`.

En ambas terminales, desde la carpeta del proyecto, carga ROS y los recursos:

```bash
source /opt/ros/humble/setup.bash
if [ -f ~/turtlebot3_ws/install/setup.bash ]; then source ~/turtlebot3_ws/install/setup.bash; fi
export TURTLEBOT3_MODEL=waffle_pi ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1
export GAZEBO_MODEL_PATH="$PWD/modelos${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
```

En la primera abre el mundo del almacén con la cámara v6 utilizada en las pruebas de CNN.

```bash
ros2 launch ./launch/almacen_f1_optica_v6.launch.py
```

En la segunda comprueba el origen y luego ejecuta la primera misión simulada.

```bash
python3 tools/A3_E6_R27_ejecutor.py --self-test
python3 tools/A3_E6_R27_ejecutor.py --preflight-ros --desde ROBOT --orden RECEPCION
python3 tools/A3_E6_R27_ejecutor.py --ejecutar --autorizar-movimiento --desde ROBOT --orden RECEPCION
```

Se guardan registros en `resultados/A3_E6_R27_pilotos/`. Confirmar el
funcionamiento del modelo, odometría y giro antes de la tanda completa.

## Evidencia U3S7

- Recuperación visual. En una prueba controlada, inducir una llegada física a
  otra estación. Confirmar que odometría y tres imágenes coincidan en la
  estación alternativa, que se genere un CSV nuevo desde allí y que el robot
  alcance y confirme la meta original. El ejecutor intenta una recuperación
  por orden usando los tramos A* existentes. Una lectura incompatible con la
  odometría no se utiliza para mover el robot.
- Circuito final. Ejecutar una secuencia única de diez órdenes. Registrar la
  semilla y pose de los obstáculos insertados en Gazebo, y medir contactos
  robot–obstáculo con instrumentación del simulador. Una distancia LiDAR por sí
  sola no acredita ausencia de colisiones. Guardar, por orden, estado final,
  tiempo de navegación, tiempo hasta confirmar la estación, longitud A* y
  límite calculado como 1,30 × longitud / 0,20. Las diez órdenes y sus contactos
  aún no se han ejecutado con esta revisión.
- Entrenamiento. Obtener los PNG originales junto a su manifiesto del equipo.
  Ejecutar el comando de abajo desde la carpeta del proyecto. El script
  verifica los hashes y que no se compartan imágenes o grupos entre train,
  validación y test. Genera un modelo y sus propias métricas, curvas y diagrama.

```bash
~/.venvs/amr-cnn/bin/python tools/entrenar_cnn_final.py /ruta/al/dataset/manifiesto.csv resultados/cnn_final_entrenada
```

Utilizar en el informe los resultados del mismo archivo Keras que se pruebe
en Gazebo. Los resultados históricos del modelo compatible se conservan.
Incluir el análisis visual de tres fallos reales, con una prueba adicional
identificada por separado si el test principal produjo menos de tres.

Completar el informe IEEE de ocho páginas con las evidencias finales, grabar
el video de hasta tres minutos y preparar la defensa. Etiquetar el commit
validado como `final` al cerrar la entrega.
