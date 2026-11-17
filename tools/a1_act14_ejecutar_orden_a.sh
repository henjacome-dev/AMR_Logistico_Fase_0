#!/usr/bin/env bash
# Reproducción supervisada de A1: requiere Gazebo ya arrancado en terminal separada.
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if ! command -v ros2 >/dev/null 2>&1; then
  echo 'ERROR: ros2 no está en PATH; ejecuta primero source tools/activar_entorno.sh' >&2; exit 1
fi
python3 "$ROOT/tools/a1_act14_validar_orden_a.py" --check
for node in /supervisor_f13 /emulador_sensores; do
  if ! ros2 node list 2>/dev/null | grep -Fxq "$node"; then
    echo "ERROR: falta nodo $node; iniciar Gazebo con launch/almacen_operativo_f1.launch.py" >&2; exit 2
  fi
done
for topic in /odom /scan /cmd_vel /cmd_vel_ruta /rango_frontal /rango_izq /rango_der /clock; do
  if ! ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then echo "ERROR: falta tópico $topic" >&2;exit 2;fi
done
n_pub="$(ros2 topic info /cmd_vel | awk '/Publisher count:/ {print $3}')"
if [[ "$n_pub" != '1' ]]; then echo "ERROR: /cmd_vel debe tener UN publicador, tiene $n_pub" >&2;exit 2;fi
n_path="$(ros2 topic info /cmd_vel_ruta | awk '/Publisher count:/ {print $3}')"
if [[ "$n_path" != '0' ]]; then echo "ERROR: otro seguidor está publicando /cmd_vel_ruta ($n_path)" >&2;exit 2;fi
if ! timeout 15s ros2 topic echo --qos-reliability best_effort --once /odom >/dev/null 2>&1; then
  echo 'ERROR: no llega odometría /odom' >&2;exit 2
fi
if ! timeout 15s ros2 topic echo --qos-reliability best_effort --once /scan >/dev/null 2>&1; then
  echo 'ERROR: no llega /scan' >&2;exit 2
fi
python3 "$ROOT/resultados/F1_A_cierre_20261106/seguidor_orden_A.py" \
  --ruta "$ROOT/resultados/R04_astar_mapa_corregido/resultados/F12_orden_A.csv" \
  --validar
base="${HOME}/AMR_evidencias_privadas/A1_ACT14"
mkdir -p "$base"
free_mb="$(df -Pm "$base" | awk 'NR==2 {print $4}')"
if (( free_mb < 500 )); then echo 'ERROR: menos de 500 MB disponibles para registrar ROS bag' >&2;exit 2;fi
run="$base/ejecucion_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$run/resultados"
# Evitar sobrescribir el registro histórico. El único cambio de la copia de ejecución es el destino del log.
python3 - "$ROOT/resultados/F1_A_cierre_20261106/seguidor_orden_A.py" "$run/seguidor_ejecucion.py" <<'PY'
from pathlib import Path
import sys
origin=Path(sys.argv[1]).read_text(encoding='utf-8')
needle="self.salida = Path('/home/enry/AMR_Logistico_Fase_0/resultados/F1_A_cierre_20261106/seguimiento_orden_A.csv')"
assert origin.count(needle)==1, 'No se encontró una única ruta de salida histórica.'
origin=origin.replace(needle,"self.salida = Path('resultados/ACT14_seguimiento_orden_A.csv')")
Path(sys.argv[2]).write_text(origin,encoding='utf-8')
PY
(cd "$run" && diff -u "$ROOT/resultados/F1_A_cierre_20261106/seguidor_orden_A.py" seguidor_ejecucion.py > ajuste_temporal.diff) || [[ $? == 1 ]]
python3 - "$run" "$ROOT" <<'PY'
import json,os,sys,datetime,hashlib
from pathlib import Path
run=Path(sys.argv[1]);root=Path(sys.argv[2]);r=root/'resultados/R04_astar_mapa_corregido/resultados/F12_orden_A.csv'
m={'baseline':'F1_A_cierre_20261106','inicio_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
   'raiz_repo':str(root),'ruta_csv':str(r),'ruta_sha256':hashlib.sha256(r.read_bytes()).hexdigest(),
   'ros_domain_id':os.environ.get('ROS_DOMAIN_ID','0'),'nota':'copia temporal del seguidor: solo ruta de salida cambiada'}
(run/'metadata_ejecucion.json').write_text(json.dumps(m,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
PY
printf '\n[PREPARADO] Evidencia privada: %s\n' "$run"
printf '%s\n' '[SEGURIDAD] Solo Gazebo; supervisor único; entorno despejado. Ctrl+C para detener.'
read -r -p '¿Confirmas inicio del movimiento AUTÓNOMO en Gazebo? [s/N] ' confirm
if [[ "$confirm" != 's' && "$confirm" != 'S' ]]; then echo 'Cancelado sin movimiento';exit 0;fi
bag_pid=''
cleanup() {
 if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    kill -INT "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
 fi
}
trap cleanup EXIT
ros2 bag record -s sqlite3 -o "$run/rosbag" /odom /scan /cmd_vel /cmd_vel_ruta /rango_frontal /rango_izq /rango_der /clock >"$run/rosbag.log" 2>&1 &
bag_pid=$!
sleep 3
if ! kill -0 "$bag_pid" 2>/dev/null; then echo 'ERROR: no inició ros2 bag record';exit 3;fi
set +e
(cd "$run" && python3 -u seguidor_ejecucion.py --ruta "$ROOT/resultados/R04_astar_mapa_corregido/resultados/F12_orden_A.csv" --ros-args -p use_sim_time:=true) 2>&1 | tee "$run/seguidor.log"
code=${PIPESTATUS[0]}
set -e
printf '%s\n' "$code" > "$run/seguidor_exit_code.txt"
cleanup
bag_pid=''
printf '[FINALIZÓ] Código del seguidor: %s\n' "$code"
if python3 "$ROOT/tools/a1_act14_validar_orden_a.py" --evaluar "$run"; then
  printf '[APROBADA] Revisión: %s\n' "$run/A1_ACT14_RESULTADO_PUBLICABLE.md"
else
  echo '[NO APROBADA] Revisa seguidor.log, simulador y rosbag; no publiques éxito.' >&2
  exit 4
fi
printf '\nEVIDENCIAS LOCALES PRIVADAS: %s\n' "$run"
