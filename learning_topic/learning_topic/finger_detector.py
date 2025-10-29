#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, PointStamped, Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker
import mediapipe as mp

class FingerDetector(Node):
    def __init__(self):
        super().__init__('finger_detector')
        
        # Configuración ROS
        self.subscriber = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )
        self.marker_pub = self.create_publisher(Marker, 'finger_marker', 10)
        self.poses_pub = self.create_publisher(PoseArray, 'finger_poses', 10)
        self.cv_bridge = CvBridge()
        
        # MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Sistema de coordenadas y TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Parámetros de calibración / camara propia 
        
        """
        self.fx = 786.84289    # Distancia focal x
        self.fy = 886.81534    # Distancia focal y
        self.cx = 327.57656    # Centro óptico x
        self.cy = 250.2981     # Centro óptico y
        self.Z = -0.38         # Distancia estimada 
        """
        # Parámetros de calibración/ Camara Andres_Vargas 
        self.fx = 600.078450   # Distancia focal x
        self.fy = 535.976126    # Distancia focal y
        self.cx = 268.669021   # Centro óptico x
        self.cy = 289.272643   # Centro óptico y
        self.Z = -0.475         # Distancia estimada
        
        # Variables de estado
        self.state = "WAITING_FIRST_POINT"
        self.first_point = None
        self.second_point = None
        self.current_position = None
        self.stable_start_time = None
        self.countdown = 0
        self.last_positions = []
        
        # Estados de la mano
        self.hand_state = "NO_HAND"  # ["NO_HAND", "OPEN", "CLOSED"]
        self.last_hand_state = "NO_HAND"

        # Valores por defecto
        self.default_cam_coords = (0.0, 0.0, 0.0)
        self.default_world_coords = geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0)
        self.last_valid_world_coords = self.default_world_coords

    def image_callback(self, msg):
        frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w = frame.shape[:2]
        
        current_cam_coords = self.default_cam_coords
        current_world_coords = self.default_world_coords
        self.current_position = None
        self.hand_state = "NO_HAND"



# --------------- MARCAR CENTRO ÓPTICO EN LA IMAGEN -----------------
        cv2.circle(frame, (int(self.cx), int(self.cy)), 5, (255, 0, 255), -1)
        cv2.putText(frame, "Centro optico", (int((self.cx)*100)+5, (int(self.cy)*100)-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        # -> NUEVO BLOQUE: coordenadas (0,0,Z) en world
        opt_cam_coords = (0.0, 0.0, self.Z)                  # (X=0,Y=0,Z)
        opt_world = self.camera_to_world(opt_cam_coords)     # TF camera→world
        if opt_world:                                         # TF disponible
            world_txt = f"W: {opt_world.x:.3f}, {opt_world.y:.3f}, {opt_world.z:.3f}"
        else:                                                 # TF aún no listo
            world_txt = "W: N/A"
        cv2.putText(frame, world_txt,
                    (int(self.cx)+5, int(self.cy)+25),        # justo debajo
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        # -------------------------------------------------------------------

        # Detección de manos
        results = self.hands.process(rgb_frame)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame, 
                    hand_landmarks, 
                    self.mp_hands.HAND_CONNECTIONS
                )
                
                # Obtener posición del índice (landmark 8)
                index_tip = hand_landmarks.landmark[8]
                cx_px = int(index_tip.x * w)
                cy_px = int(index_tip.y * h)
                self.current_position = (cx_px, cy_px)
                
                # Calcular coordenadas
                current_cam_coords = self.pixel_to_camera(cx_px, cy_px)
                current_world_coords = self.camera_to_world(current_cam_coords)
                if current_world_coords:
                    self.last_valid_world_coords = current_world_coords

                # Detección de estado de la mano
                fingers_extended = self.count_extended_fingers(hand_landmarks)
                
                # Determinar estado de la mano
                if fingers_extended >= 4:
                    self.hand_state = "OPEN"
                elif fingers_extended <= 1:
                    self.hand_state = "CLOSED"
                else:
                    self.hand_state = "NO_HAND"

        # Manejar cambios de estado
        self.handle_state_transitions()
        
        # Dibujar la interfaz
        self.draw_ui(frame, current_cam_coords, current_world_coords)
        self.handle_stability()
        self.draw_status(frame)
        
        cv2.imshow("Finger Tracking", frame)
        cv2.waitKey(1)

    def count_extended_fingers(self, hand_landmarks):
        # Contar dedos extendidos
        fingers_extended = 0
        
        # Pulgar (comparación diferente)
        thumb_tip = hand_landmarks.landmark[4]
        thumb_ip = hand_landmarks.landmark[3]
        if thumb_tip.x < thumb_ip.x:  # Para mano derecha
            fingers_extended += 1
        
        # Índice
        if hand_landmarks.landmark[8].y < hand_landmarks.landmark[6].y:
            fingers_extended += 1
        
        # Medio
        if hand_landmarks.landmark[12].y < hand_landmarks.landmark[10].y:
            fingers_extended += 1
        
        # Anular
        if hand_landmarks.landmark[16].y < hand_landmarks.landmark[14].y:
            fingers_extended += 1
        
        # Meñique
        if hand_landmarks.landmark[20].y < hand_landmarks.landmark[18].y:
            fingers_extended += 1
        
        return fingers_extended

    def handle_state_transitions(self):
        # Reiniciar puntos si se detecta mano abierta después de completar
        if self.hand_state == "OPEN" and self.state == "COMPLETED":
            self.state = "WAITING_FIRST_POINT"
            self.first_point = None
            self.second_point = None
            self.get_logger().info("Sistema reiniciado por mano abierta")
        
        # Actualizar último estado válido
        if self.hand_state != "NO_HAND":
            self.last_hand_state = self.hand_state

    def pixel_to_camera(self, x_px, y_px):
        X = (x_px - self.cx) * (self.Z / self.fx)
        Y = (y_px - self.cy) * (self.Z / self.fy)
        return (-X, Y, self.Z)

    def camera_to_world(self, cam_coords):
        try:
            transform = self.tf_buffer.lookup_transform("world", "camera", rclpy.time.Time())
            point = PointStamped()
            point.header.frame_id = "camera"
            point.point.x = cam_coords[0]
            point.point.y = cam_coords[1]
            point.point.z = cam_coords[2]
            return do_transform_point(point, transform).point
        except Exception as e:
            self.get_logger().warn(f"Error TF: {str(e)}")
            return None

    def handle_stability(self):
        now = self.get_clock().now()
        
        # Solo actuar si hay mano cerrada detectada
        if self.hand_state == "CLOSED" and self.current_position and self.check_stability():
            if not self.stable_start_time:
                self.stable_start_time = now
                self.countdown = 3
            else:
                elapsed = (now - self.stable_start_time).nanoseconds / 1e9
                self.countdown = max(3 - int(elapsed), 0)
                if elapsed >= 3:
                    self.save_position()
                    self.stable_start_time = None
                    self.countdown = 0
        else:
            self.stable_start_time = None
            self.countdown = 0

    def check_stability(self, threshold=30):
        self.last_positions.append(self.current_position)
        if len(self.last_positions) > 5:
            self.last_positions.pop(0)
        if len(self.last_positions) < 2:
            return False
        movements = [np.linalg.norm(np.array(a) - np.array(b))
                     for a, b in zip(self.last_positions[:-1], self.last_positions[1:])]
        return np.mean(movements) < threshold

    def save_position(self):
        cam_coords = self.pixel_to_camera(*self.current_position)

        world_coords = self.camera_to_world(cam_coords)
        
        if self.state == "WAITING_FIRST_POINT":
            self.first_point = (cam_coords, world_coords)
            self.state = "WAITING_SECOND_POINT"
            self.get_logger().info(f"Punto 1 guardado - Mundo: {world_coords.x:.2f}, {world_coords.y:.2f}, {world_coords.z:.2f}")
        elif self.state == "WAITING_SECOND_POINT":
            self.second_point = (cam_coords, world_coords)
            self.state = "COMPLETED"
            self.publish_markers()
            self.get_logger().info(f"Punto 2 guardado - Mundo: {world_coords.x:.2f}, {world_coords.y:.2f}, {world_coords.z:.2f}")

    def publish_markers(self):
        marker = Marker()
        poses_msg = PoseArray()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "world"
        marker.ns = "finger_points"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.g = 1.0
      
        points = []
        for p in [self.first_point, self.second_point]:
            if p and p[1]:
                point = geometry_msgs.msg.Point()
                point.x = p[1].x
                point.y = p[1].y
                point.z = p[1].z
                points.append(point)
                
                pose = Pose()
                pose.position.x = p[1].x
                pose.position.y = p[1].y
                pose.position.z = p[1].z
                pose.orientation.w = 1.0
                poses_msg.poses.append(pose)
                
        self.poses_pub.publish(poses_msg)
        marker.points = points
        self.marker_pub.publish(marker)

    def draw_ui(self, frame, cam_coords, world_coords):
        detection_color = (0, 255, 0) if self.current_position else (200, 200, 200)
        
        # Texto de coordenadas
        coord_text = f"Cámara: X:{cam_coords[0]*100:.2f} Y:{cam_coords[1]*100:.2f}"
        world_text = (f"Mundo: X:{world_coords.x*100:.2f} Y:{world_coords.y*100:.2f}" 
                      if world_coords else "Mundo: N/A")
        
        cv2.putText(frame, coord_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, detection_color, 1)
        cv2.putText(frame, world_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, detection_color, 1)
        

        # Círculo y cuenta regresiva
        if self.current_position:
            cx_px, cy_px = self.current_position
            cv2.circle(frame, (cx_px, cy_px), 10, (0, 255, 0), -1)
            if self.countdown > 0:
                cv2.putText(frame, str(self.countdown), (cx_px+20, cy_px),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        
    # Dibujar líneas en los ejes X e Y desde el punto central (centro óptico)
        cv2.line(frame, (int(self.cx), 0), (int(self.cx), frame.shape[0]), (255, 0, 255), 1)  # Línea vertical (eje Y) en rosa
        cv2.line(frame, (0, int(self.cy)), (frame.shape[1], int(self.cy)), (255, 0, 255), 1)  # Línea horizontal (eje X) en rosa

        # Etiquetas para los ejes
        cv2.putText(frame, "Eje Y", (int(self.cx) + 10, int(frame.shape[0] / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        cv2.putText(frame, "Eje X", (int(frame.shape[1] / 2), int(self.cy) + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)


        # Estado de la mano
        state_colors = {
            "OPEN": (0, 255, 0),
            "CLOSED": (0, 0, 255),
            "NO_HAND": (200, 200, 200)
        }
        state_text = {
            "OPEN": "Mano Abierta: Lista para nuevos puntos",
            "CLOSED": "Mano Cerrada: Guardando posicion",
            "NO_HAND": "Mano No Detectada"
        }.get(self.hand_state, "Estado Desconocido")
        
        cv2.putText(frame, state_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                   state_colors.get(self.hand_state, (255, 255, 255)), 2)
        
        # Puntos guardados
        y_offset = 120
        for i, p in enumerate([self.first_point, self.second_point]):
            if p:
                text = f"P{i+1}  X:{p[1].x:.2f} Y:{p[1].y:.2f} Z:{p[1].z:.2f}"
                cv2.putText(frame, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
                y_offset += 30

    def draw_status(self, frame):
        status_text = f"Estado: {self.state.replace('_', ' ').title()}"
        cv2.putText(frame, status_text, (10, frame.shape[0]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

def main(args=None):
    rclpy.init(args=args)
    node = FingerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()