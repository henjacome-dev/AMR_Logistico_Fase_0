#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi ROS_DOMAIN_ID="${AMR_DOMINIO:-175}" ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI="http://127.0.0.1:${AMR_PUERTO_GAZEBO:-11380}" GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH="$PWD/modelos:/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models"
base=$(mktemp -d "$PWD/resultados/ensayo_u3s7_XXXXXX")
export AMR_MODELO_GAZEBO="$base/waffle_contactos.sdf"
PYTHONPATH="$PWD/tools${PYTHONPATH:+:$PYTHONPATH}" python3 -c '
import os
from prueba_final_gazebo import instrumentar_modelo
instrumentar_modelo("modelos/variantes_opticas/waffle_pi_optica_v6.sdf", os.environ["AMR_MODELO_GAZEBO"])
'
timeout --signal=INT --kill-after=10s 1500s ros2 launch ./launch/almacen_f1_optica_v6.launch.py gui:=false \
    >"$base/gazebo.log" 2>&1 &
simulacion=$!
trap 'kill -INT "$simulacion" 2>/dev/null || true; wait "$simulacion" 2>/dev/null || true' EXIT
printf 'Registro del ensayo en %s\n' "$base"
timeout 50s ros2 topic echo /camera/image_raw sensor_msgs/msg/Image --qos-reliability best_effort --once --field width
if [[ "${1:-}" == --desvio ]]; then
    python3 -u tools/A3_E6_R27_ejecutor.py --ejecutar --autorizar-movimiento --desde ROBOT \
        --orden ALMACENAMIENTO --ensayo-desvio RECEPCION | tee "$base/desvio.log"
    exit
fi
python3 -u tools/A3_E6_R27_ejecutor.py --ejecutar --autorizar-movimiento --desde ROBOT \
    --orden RECEPCION | tee "$base/posicionamiento.log"
ordenes=(ALMACENAMIENTO EMPAQUE DESPACHO EMPAQUE ALMACENAMIENTO RECEPCION \
         ALMACENAMIENTO EMPAQUE DESPACHO EMPAQUE)
if [[ "${1:-}" == --piloto ]]; then ordenes=(ALMACENAMIENTO); fi
python3 -u tools/A3_E6_R27_ejecutor.py --ejecutar --autorizar-movimiento \
    --desde RECEPCION --orden "${ordenes[@]}" --semilla-obstaculos "${AMR_SEMILLA:-451}" \
    | tee "$base/ordenes.log"
