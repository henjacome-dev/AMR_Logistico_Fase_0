# Probar el AMR de la entrega U3S7

Esta copia corresponde al commit d504662 de la etiqueta final. Incluye
el modelo entrenado, 600 imágenes de Gazebo, el informe, el video y las
evidencias de las diez órdenes y del desvío. El proyecto se ejecuta con
Ubuntu 22.04, ROS 2 Humble, Gazebo Classic y TurtleBot3 Waffle Pi.

## 1. Extraer y abrir la carpeta

Extrae todo el ZIP en una carpeta propia y entra en AMR_Logistico_U3S7.
En Ubuntu abre una terminal en esa carpeta. Si usas WSL2 y la extrajiste
en Windows, sustituye la ruta de ejemplo por la ubicación que ves en
el Explorador de archivos.

```bash
cd "$(wslpath 'C:\Users\tu_usuario\Downloads\AMR_Logistico_U3S7')"
```

Comprueba con `ls` que aparecen `tools`, `modelos` y
`LEEME_PRIMERO.md`. Para instalar WSL en Windows consulta
https://learn.microsoft.com/es-es/windows/wsl/install/

## 2. Preparar el entorno

Si todavía no tienes ROS 2 Humble, Gazebo Classic y TurtleBot3, ejecuta:

```bash
bash tools/instalar_ros_humble.sh
```

Prepara los entornos de Python una sola vez por computadora:

```bash
python3 -m venv --system-site-packages ~/.venvs/amr-a2-r18
~/.venvs/amr-a2-r18/bin/python -m pip install -r tools/requirements_a2_r18.txt
python3 -m venv ~/.venvs/amr-cnn
~/.venvs/amr-cnn/bin/python -m pip install 'tensorflow==2.16.2' 'numpy<2' matplotlib pydot
```

## 3. Ejecutar la primera misión

Desde la raíz del proyecto y con otras pruebas de Gazebo cerradas:

```bash
bash tools/verificar_gazebo_local.sh --mision
```

El script abre el simulador en segundo plano. Primero comprueba odometría,
LiDAR y cámara. Después envía el robot a Recepción. Busca CNN RECEPCION
y 1/1 MISIONES CONSECUTIVAS COMPLETADAS en la terminal.

Para ver el robot en la ventana gráfica, abre dos terminales de Ubuntu
en la raíz del proyecto. En la primera:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1
export GAZEBO_MODEL_PATH="$PWD/modelos:/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models"
export GAZEBO_MODEL_DATABASE_URI=""
ros2 launch ./launch/almacen_f1_optica_v6.launch.py
```

Con Gazebo abierto, en la segunda terminal:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1
python3 tools/A3_E6_R27_ejecutor.py --ejecutar --autorizar-movimiento --desde ROBOT --orden RECEPCION
```

Para ejecutar las diez órdenes con cajas, cierra la simulación gráfica y ejecuta:

```bash
AMR_SEMILLA=451 bash tools/ejecutar_prueba_u3s7.sh
```

La prueba usa el modelo ya entrenado. Para consultar la entrega abre
`docs/entrega_u3s7/`. Los resultados publicados están en
`resultados/U3S7/` y la guía de defensa en `docs/DEFENSA_U3S7.md`.

El ZIP es una copia para ejecutar y revisar archivos. Si necesitas
historial, etiquetas o colaborar mediante Git, solicita acceso al
repositorio privado: https://github.com/panchofran10-ffs/AMR_Logistico_U3S7
