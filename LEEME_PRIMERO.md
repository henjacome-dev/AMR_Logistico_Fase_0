# AMR — A2 Actividad 11: capturador de 10 pruebas Mamdani

Este paquete **prepara** la adquisición; no contiene nuevas pruebas de Gazebo ni declara diez evasiones completas.

## Desde Ubuntu

```bash
amr iniciar
unzip -o /mnt/c/Users/Henry/Downloads/AMR_A2_Actividad11_10_Pruebas_Mamdani.zip \
  -d ~/AMR_publicacion_u3s7-entrega-final-2026-11-16
python3 tools/a2_act11_10_pruebas.py --self-test
python3 tools/a2_act11_10_pruebas.py --preflight
amr guardar "test(a2): preparar captura de 10 actuaciones Mamdani con ROS bag"
```

Cierra primero cualquier Gazebo o seguidor de otro experimento.

```bash
cd ~/AMR_publicacion_u3s7-entrega-final-2026-11-16
source tools/activar_entorno.sh
export ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1
python3 tools/a2_act11_10_pruebas.py --run --count 10
```

**Lee antes** `docs/A2_ACT11_PROTOCOLO_DIEZ_PRUEBAS.md`: define niveles de evidencia, límites de contacto, archivos privados y publicación posterior del resultado. No ejecutes el programa en robot físico.
