"""Instrumentación de la prueba U3S7 en Gazebo Classic."""
import math
import random
import time
import xml.etree.ElementTree as ET


def instrumentar_modelo(origen, salida):
    arbol = ET.parse(origen)
    for i, enlace in enumerate(arbol.findall("./model/link")):
        for j, colision in enumerate(enlace.findall("collision")):
            sensor = ET.SubElement(enlace, "sensor", name=f"contacto_{i}_{j}", type="contact")
            ET.SubElement(sensor, "always_on").text = "true"
            ET.SubElement(sensor, "update_rate").text = "100"
            ET.SubElement(ET.SubElement(sensor, "contact"), "collision").text = colision.attrib["name"]
            complemento = ET.SubElement(sensor, "plugin", name=f"auditoria_contacto_{i}_{j}",
                                        filename="libgazebo_ros_bumper.so")
            ET.SubElement(ET.SubElement(complemento, "ros"), "remapping").text = (
                "bumper_states:=/amr_contactos")
    arbol.write(salida, encoding="utf-8", xml_declaration=False)


class PruebaFinal:
    def __init__(self, auditor, carpeta, semilla):
        from gazebo_msgs.msg import ContactsState
        from gazebo_msgs.srv import DeleteEntity, SpawnEntity
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image
        from rclpy.qos import qos_profile_sensor_data
        from PIL import Image as PillowImage

        self.auditor = auditor
        self.rng = random.Random(semilla)
        self.carpeta = carpeta
        self.contactos = []
        self.activo = False
        self.mensajes_contacto = 0
        self.reloj = None
        self.ultimo_frame = -10.0
        self.frames = 0
        self.pillow = PillowImage
        self.nombre = None
        self.servicio_spawn = auditor.node.create_client(SpawnEntity, "/spawn_entity")
        self.servicio_delete = auditor.node.create_client(DeleteEntity, "/delete_entity")
        self.SpawnEntity, self.DeleteEntity = SpawnEntity, DeleteEntity
        self.subs = [
            auditor.node.create_subscription(ContactsState, "/amr_contactos", self.cb_contactos,
                                             qos_profile_sensor_data),
            auditor.node.create_subscription(Clock, "/clock", self.cb_clock, qos_profile_sensor_data),
            auditor.node.create_subscription(Image, "/camera/image_raw", self.cb_camara,
                                             qos_profile_sensor_data),
        ]

    def cb_clock(self, msg):
        self.reloj = msg.clock.sec + msg.clock.nanosec / 1e9

    def cb_contactos(self, msg):
        self.mensajes_contacto += 1
        for estado in msg.states:
            par = sorted((estado.collision1_name, estado.collision2_name))
            if (self.activo and any("waffle_pi::" in nombre for nombre in par)
                    and not any("ground_plane" in nombre for nombre in par)
                    and not all("waffle_pi::" in nombre for nombre in par)):
                if not self.contactos or self.contactos[-1]["par"] != par:
                    self.contactos.append({"par": par, "sim_s": self.reloj})

    def cb_camara(self, msg):
        if not self.activo or self.reloj is None or self.reloj - self.ultimo_frame < 1.0:
            return
        if msg.encoding.lower() not in ("rgb8", "bgr8"):
            return
        self.ultimo_frame = self.reloj
        frames = self.carpeta / "video_camara"
        frames.mkdir(exist_ok=True)
        imagen = self.pillow.frombytes("RGB", (msg.width, msg.height), bytes(msg.data),
                                      "raw", msg.encoding[:3].upper(), msg.step, 1)
        imagen.save(frames / f"{self.frames:05d}.jpg", quality=80)
        self.frames += 1

    def servicio(self, cliente, solicitud):
        if not cliente.wait_for_service(timeout_sec=8):
            raise RuntimeError("Servicio de Gazebo no disponible")
        futuro = cliente.call_async(solicitud)
        limite = time.monotonic() + 10
        while not futuro.done() and time.monotonic() < limite:
            self.auditor.spin(.05)
        if not futuro.done():
            raise RuntimeError("Gazebo no respondió al modificar el obstáculo")
        respuesta = futuro.result()
        if not respuesta.success:
            raise RuntimeError(respuesta.status_message)

    def insertar(self, etapa):
        # Caja de 20 cm en un punto aleatorio del costado interior del pasillo.
        # El conjunto de candidatos mantiene despejadas la salida y las estaciones.
        from PIL import Image
        from A3_E43_gestor_misiones import PUNTOS, ROOT
        imagen = Image.open(ROOT / "mapas/almacen_f1_inflado.pgm")
        puntos = etapa["puntos"]
        candidatos = []
        for x, y in puntos[::4]:
            if min(math.dist((x, y), sitio) for sitio in PUNTOS.values()) < 1.10:
                continue
            for dx, dy in ((.53, 0), (-.53, 0), (0, .53), (0, -.53)):
                px, py = x+dx, y+dy
                if not (-4.5 < px < 4.5 and -3.5 < py < 3.5):
                    continue
                if imagen.getpixel((int((px+5)/.05), 159-int((py+4)/.05))) < 128:
                    continue
                if min(math.dist((px, py), sitio) for sitio in puntos) >= .46:
                    candidatos.append((round(px, 3), round(py, 3)))
        if not candidatos:
            raise RuntimeError("No hay una pose válida para la caja de esta orden")
        x, y = self.rng.choice(candidatos)
        self.nombre = nombre = f'amr_final_caja_{etapa["numero"]:02d}'
        solicitud = self.SpawnEntity.Request()
        solicitud.name = nombre
        solicitud.xml = (f'<sdf version="1.6"><model name="{nombre}"><static>true</static>'
                         '<link name="caja"><collision name="colision"><geometry><box>'
                         '<size>0.20 0.20 0.32</size></box></geometry></collision>'
                         '<visual name="caja"><geometry><box><size>0.20 0.20 0.32</size></box>'
                         '</geometry><material><ambient>0.92 0.43 0.10 1</ambient>'
                         '</material></visual></link></model></sdf>')
        solicitud.initial_pose.position.x = x
        solicitud.initial_pose.position.y = y
        solicitud.initial_pose.position.z = .16
        solicitud.initial_pose.orientation.w = 1.0
        self.servicio(self.servicio_spawn, solicitud)
        return {"nombre": nombre, "x_m": x, "y_m": y, "lado_m": .20}

    def retirar(self):
        if self.nombre is not None:
            solicitud = self.DeleteEntity.Request()
            solicitud.name = self.nombre
            self.servicio(self.servicio_delete, solicitud)
            self.nombre = None
