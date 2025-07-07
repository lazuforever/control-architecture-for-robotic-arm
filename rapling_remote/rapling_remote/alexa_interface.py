#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from flask_ask_sdk.skill_adapter import SkillAdapter

# Importa el mensaje y la acción correspondiente
from rapling_msgs.action import ArduinobotTask
from geometry_msgs.msg import PoseArray  # Para recibir las posiciones

#######################################
# Nodo de interfaz ROS para Alexa
#######################################
class AlexaInterfaceNode(Node):
    def __init__(self):
        super().__init__('alexa_interface')
        # Crear ActionClient para enviar goals
        self.action_client = ActionClient(self, ArduinobotTask, "task_server")
        # Crear suscriptor para recibir PoseArray con las posiciones de finger_detector
        self.finger_pose_subscription = self.create_subscription(
            PoseArray,
            "finger_poses",
            self.finger_pose_callback,
            10
        )
        self.latest_finger_pose = None

    def finger_pose_callback(self, msg):
        self.latest_finger_pose = msg
        self.get_logger().info("Recibida PoseArray con {} poses".format(len(msg.poses)))
        # Si se reciben exactamente 2 posiciones, las mostramos en el log
        if len(msg.poses) == 2:
            p1 = msg.poses[0].position
            p2 = msg.poses[1].position
            self.get_logger().info("Punto 1 - X: {:.2f}, Y: {:.2f}, Z: {:.2f}".format(p1.x, p1.y, p1.z))
            self.get_logger().info("Punto 2 - X: {:.2f}, Y: {:.2f}, Z: {:.2f}".format(p2.x, p2.y, p2.z))
        else:
            self.get_logger().warn("Número de posiciones recibido distinto de 2")

# Inicialización de ROS
rclpy.init(args=None)
alexa_node = AlexaInterfaceNode()
# Iniciar el spin en un hilo separado para procesar callbacks de ROS (action client y subscription)
threading.Thread(target=lambda: rclpy.spin(alexa_node), daemon=True).start()

#######################################
# Configuración de Flask y Alexa SDK
#######################################
app = Flask(__name__)

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return handler_input.request_envelope.request.object_type == "LaunchRequest"

    def handle(self, handler_input):
        # Preparar mensaje de bienvenida
        speech_text = "Hola, Envia el comando "
        # También, por ejemplo, incluir la posición actual recibida si existe
        if (alexa_node.latest_finger_pose is not None and
            len(alexa_node.latest_finger_pose.poses) >= 1):
            # Usamos el primer punto (puedes adaptar esto según tu necesidad)
            p = alexa_node.latest_finger_pose.poses[0].position
            speech_text += " La primera posicion  detectada es X: {:.2f}, Y: {:.2f}, Z: {:.2f}.".format(p.x, p.y, p.z)
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Online", speech_text)).set_should_end_session(False)
        
        p = alexa_node.latest_finger_pose.poses[1].position
        speech_text += " La segunda  posición detectada es X: {:.2f}, Y: {:.2f}, Z: {:.2f}.".format(p.x, p.y, p.z)
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Online", speech_text)).set_should_end_session(False)

        # Enviar un goal de ejemplo (task 0)
        goal = ArduinobotTask.Goal()
        goal.task_number = 9
        alexa_node.action_client.send_goal_async(goal)
        return handler_input.response_builder.response

class PickIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return handler_input.request_envelope.request.intent.name == "PickIntent"

    def handle(self, handler_input):
        # Ejemplo para intent "PickIntent"
        slots = handler_input.request_envelope.request.intent.slots
        posx = slots["x"].value if "x" in slots else "desconocido"
        speech_text = f"Posición en equis {posx}. Ejecutando tarea de Pick."
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(True)

        goal = ArduinobotTask.Goal()
        goal.task_number = 6
        alexa_node.action_client.send_goal_async(goal)
        return handler_input.response_builder.response

class SleepIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return handler_input.request_envelope.request.intent.name == "SleepIntent"

    def handle(self, handler_input):
        speech_text = "Ejecutando SleepIntent."
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Sleep", speech_text)).set_should_end_session(True)
        goal = ArduinobotTask.Goal()
        goal.task_number = 2
        alexa_node.action_client.send_goal_async(goal)
        return handler_input.response_builder.response

class WakeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return handler_input.request_envelope.request.intent.name == "WakeIntent"

    def handle(self, handler_input):
        speech_text = "Listo, Pick And place activado.  "
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Wake", speech_text)).set_should_end_session(True)
        goal = ArduinobotTask.Goal()
        goal.task_number = 1
        alexa_node.action_client.send_goal_async(goal)
        return handler_input.response_builder.response

class AllExceptionHandler(AbstractExceptionHandler):
    def can_handle(self, handler_input, exception):
        return True

    def handle(self, handler_input, exception):
        speech = "Hmm, I don't know that. Can you please say it again?"
        handler_input.response_builder.speak(speech).ask(speech)
        # Enviar por ejemplo otro goal en caso de error
        goal = ArduinobotTask.Goal()
        goal.task_number = 2
        alexa_node.action_client.send_goal_async(goal)
        return handler_input.response_builder.response

# Configuración del SkillBuilder y registro de handlers
skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickIntentHandler())
skill_builder.add_request_handler(SleepIntentHandler())
skill_builder.add_request_handler(WakeIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())

skill_adapter = SkillAdapter(
    skill=skill_builder.create(), 
    skill_id="amzn1.ask.skill.367b1616-6380-457e-be52-8abfdebc0a0a",
    app=app)

skill_adapter.register(app=app, route="/")

@app.route("/")
def invoke_skill():
    return skill_adapter.dispatch_request()

if __name__ == '__main__':
    app.run()
