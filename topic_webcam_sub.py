import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import time
import math

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

class HandTracker(Node):
    def __init__(self):
        super().__init__("hand_tracker")
        self.sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(String, 'hand_direction', 10)
        self.cv_bridge = CvBridge()
        
        self.hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)

        # Parámetros generales
        self.last_sent_time = time.time()
        self.cooldown = 1.0  # segundos
        self.last_command = ""  # Para mostrar lo último enviado

        # Parámetros del círculo único (se usa para XY y Z)
        self.circle_radius = 150       # Radio del círculo
        self.movement_threshold = 20   # Umbral para detectar movimiento en X/Y
        self.history_size = 5
        self.movement_history = []
        self.consistent_frames = 0
        self.required_consistency = 6
        self.last_direction = None
        self.circle_center = None      # Se definirá en image_callback

        # Parámetros para la detección de pinza (usando punta de pulgar e índice)
        self.pinch_threshold = 0.05    # Umbral para considerar pinza cerrada (ajustable)
        self.last_pinch_state = "open" # Estado inicial de la pinza

    def image_callback(self, msg):
        frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = frame.shape

        # Usamos un único círculo en el centro de la imagen
        self.circle_center = (w // 2, h // 2)

        # Procesar la imagen con Mediapipe
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # --------------------------------------------------------------------------------
                # 1) MOVIMIENTO EN X/Y: Usamos la punta del índice (landmark 8)
                # --------------------------------------------------------------------------------
                index_tip = (int(hand_landmarks.landmark[8].x * w),
                             int(hand_landmarks.landmark[8].y * h))
                # Dibujar la punta del índice
                cv2.circle(frame, index_tip, 8, (0, 255, 0), -1)
                self.track_movement_xy(index_tip)

                # --------------------------------------------------------------------------------
                # 2) DETECCIÓN DE PINZA PARA COMANDOS EN Z
                # Utilizamos la punta del pulgar (landmark 4) y la punta del índice (landmark 8)
                # --------------------------------------------------------------------------------
                thumb_tip = (hand_landmarks.landmark[4].x * w,
                             hand_landmarks.landmark[4].y * h)
                cv2.circle(frame, (int(thumb_tip[0]), int(thumb_tip[1])), 8, (0, 255, 255), -1)

                pinch_distance = math.dist(thumb_tip, index_tip)
                current_pinch_state = "closed" if pinch_distance < (self.pinch_threshold * w) else "open"

                # Usamos el mismo círculo para determinar si se activa el comando Z
                if self.is_in_circle(index_tip):
                    # Detectar transición: de "closed" a "open"
                    if self.last_pinch_state == "closed" and current_pinch_state == "open":
                        # Si la punta del índice está en la mitad superior del círculo → Z+
                        # Si está en la mitad inferior → Z-
                        if index_tip[1] < self.circle_center[1]:
                            self.publish_z_direction("Arriba")
                        else:
                            self.publish_z_direction("Abajo")
                else:
                    current_pinch_state = "open"
                self.last_pinch_state = current_pinch_state

        # --------------------------------------------------------------------------------
        # DIBUJAR EL CÍRCULO Y DIVISIÓN
        # --------------------------------------------------------------------------------
        current_time = time.time()
        circle_color = (0, 255, 0) if (current_time - self.last_sent_time >= self.cooldown) else (0, 0, 255)
        cv2.circle(frame, self.circle_center, self.circle_radius, circle_color, 2)
        # Dibujar línea horizontal que divide el círculo en dos (para Z+ / Z-)
        cv2.line(frame,
                 (self.circle_center[0] - self.circle_radius, self.circle_center[1]),
                 (self.circle_center[0] + self.circle_radius, self.circle_center[1]),
                 (255, 0, 0), 1)

        # Mostrar el último comando enviado
        cv2.putText(frame, f"Comando: {self.last_command}", (20, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        cv2.putText(frame, "Tablero de comandos XYZ",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Control de Gestos", frame)
        cv2.waitKey(10)

    # ------------------------------------------------------------------------------
    # FUNCIONES AUXILIARES
    # ------------------------------------------------------------------------------
    def track_movement_xy(self, index_tip):
        """ Detecta movimientos en X/Y dentro del círculo central, usando la punta del índice. """
        if self.circle_center is None:
            return
        
        distance = math.dist(index_tip, self.circle_center)
        if distance > self.circle_radius:
            return
        
        self.movement_history.append(index_tip)
        if len(self.movement_history) > self.history_size:
            self.movement_history.pop(0)
        
        if len(self.movement_history) == self.history_size:
            dx = self.movement_history[-1][0] - self.movement_history[0][0]
            dy = self.movement_history[-1][1] - self.movement_history[0][1]
            direction = None
            if abs(dx) > self.movement_threshold:
                direction = "Derecha" if dx > 0 else "Izquierda"
            elif abs(dy) > self.movement_threshold:
                direction = "Adelante" if dy > 0 else "Atras"
            
            if direction == self.last_direction:
                self.consistent_frames += 1
            else:
                self.consistent_frames = 0
            
            if self.consistent_frames >= self.required_consistency:
                current_time = time.time()
                if current_time - self.last_sent_time >= self.cooldown:
                    self.get_logger().info(f"[XY] Movimiento detectado: {direction}")
                    if direction is not None:
                        self.publish_direction(direction)
                    self.last_sent_time = current_time
                    self.consistent_frames = 0
            if direction:
                self.last_direction = direction

    def is_in_circle(self, point):
        """ Retorna True si el punto está dentro del círculo central. """
        if self.circle_center is None:
            return False
        return math.dist(point, self.circle_center) <= self.circle_radius

    def publish_z_direction(self, direction_str):
        """ Publica el comando Z (Z+ o Z-) respetando el cooldown. """
        current_time = time.time()
        if current_time - self.last_sent_time >= self.cooldown:
            self.last_command = direction_str
            self.get_logger().info(f"[Z] Movimiento detectado: {direction_str}")
            msg = String()
            msg.data = direction_str
            self.publisher_.publish(msg)
            self.last_sent_time = current_time

    def publish_direction(self, direction_str):
        """ Publica el comando XY. """
        self.last_command = direction_str
        msg = String()
        msg.data = direction_str
        self.publisher_.publish(msg)
        self.get_logger().info(f"[XY] Publicando comando: {direction_str}")

def main(args=None):
    rclpy.init(args=args)
    node = HandTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
