#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Pose, Point
import pytesseract  # Asegúrate de tener instalado pytesseract

class DigitCircleDetector(Node):
    def __init__(self):
        super().__init__('digit_circle_detector')

        # Suscriptor a la cámara
        self.subscriber = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )

        # Publicador de Marker para RViz (un único Marker con los 3 puntos)
        self.marker_pub = self.create_publisher(Marker, 'circle_marker', 10)

        # (Opcional) Publicar un arreglo de poses con las posiciones de los círculos
        self.poses_pub = self.create_publisher(PoseArray, 'circles_positions', 10)

        # Conversión entre ROS y OpenCV
        self.cv_bridge = CvBridge()

        # TF2: Buffer y Listener para obtener transformaciones
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Distancia (Z) estimada al objeto en metros (ajusta a tu entorno)
        self.Z = -0.25  # Por ejemplo, 25 cm frente a la cámara

        # Valores de calibración (hardcodeados, ajústalos a tu YAML)
        self.fx = 786.84289
        self.fy = 886.81534
        self.cx = 327.57656
        self.cy = 250.2981

        self.get_logger().info(f"Usando calibración: fx={self.fx:.2f}, fy={self.fy:.2f}, "
                               f"cx={self.cx:.2f}, cy={self.cy:.2f}")

    def get_camera_transform(self):
        """
        Intenta obtener la transformación desde 'camera' hasta 'world'.
        """
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                "world",   # Frame destino
                "camera",  # Frame origen
                rclpy.time.Time()
            )
            return transform_stamped
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Error obteniendo TF: {e}")
            return None

    def image_callback(self, msg):
        # Convertir imagen ROS a OpenCV
        frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        height, width, _ = frame.shape

        # Definir ROI central (opcional, 80% del ancho y alto)
        roi_w = int(width * 0.8)
        roi_h = int(height * 0.8)
        roi_x = (width - roi_w) // 2
        roi_y = (height - roi_h) // 2
        roi_frame = frame[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w].copy()

        # Convertir a gris y aplicar medianBlur para suavizar
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.medianBlur(gray, 5)

        # Detección de círculos con HoughCircles
        circles = cv2.HoughCircles(
            gray_blur,
            cv2.HOUGH_GRADIENT, 
            dp=1.2,         # Factor de resolución
            minDist=20,     # Distancia mínima entre centros de círculos
            param1=50,      # Umbral para Canny
            param2=30,      # Umbral para detección de centros
            minRadius=10,   # Radio mínimo
            maxRadius=100   # Radio máximo
        )

        # Dibujar la ROI en la imagen principal
        cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_w, roi_y + roi_h),
                      (255, 0, 0), 2)

        # Diccionario para almacenar: dígito -> (center_x, center_y, radio)
        digit_circles = {}

        if circles is not None:
            # circles tiene forma (1, N, 3) -> (x, y, r) en coordenadas de ROI
            circles = np.uint16(np.around(circles[0, :]))
            
            # Iteramos por cada círculo detectado (se pueden detectar más de 3 inicialmente)
            for (x, y, r) in circles:
                # Convertir a coordenadas de la imagen completa
                center_x = x + roi_x
                center_y = y + roi_y

                # Extraer la subimagen dentro del círculo (con un margen)
                margin = int(r * 0.6)
                x1 = max(x - margin, 0)
                y1 = max(y - margin, 0)
                x2 = min(x + margin, roi_frame.shape[1]-1)
                y2 = min(y + margin, roi_frame.shape[0]-1)
                circle_roi = roi_frame[y1:y2, x1:x2]

                # Preprocesar la imagen para OCR: convertir a escala de grises y aplicar threshold
                roi_gray = cv2.cvtColor(circle_roi, cv2.COLOR_BGR2GRAY)
                _, roi_thresh = cv2.threshold(roi_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

                # Configurar pytesseract para que reconozca solo dígitos (modo de página 10: un solo dígito)
                config = "--psm 10 -c tessedit_char_whitelist=123456789"
                digit_str = pytesseract.image_to_string(roi_thresh, config=config).strip()

                # Verificar si se reconoce un dígito válido
                if digit_str.isdigit():
                    digit = int(digit_str)
                    self.get_logger().info(f"Se reconoció el dígito {digit} en el círculo detectado")
                    # Si ya se tiene asignado el dígito, se omite o se puede reemplazar según tu criterio
                    if digit not in digit_circles:
                        digit_circles[digit] = (center_x, center_y, r)
                    # Dibujar sobre la imagen el número reconocido
                    cv2.putText(frame, f"{digit}", (center_x - 10, center_y + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                else:
                    # Si no se reconoce dígito, se puede omitir o registrar según convenga
                    self.get_logger().info("No se reconoció dígito en un círculo")

                # Dibujar el contorno del círculo y su centro (para depuración)
                cv2.circle(frame, (center_x, center_y), r, (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)

        # Si se reconocieron alguno de los dígitos 1, 2 y 3, procedemos
        expected_digits = [1, 2, 3]
        marker_points = []
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "world"  # o "camera" según convenga

        transform_stamped = self.get_camera_transform()

        for d in expected_digits:
            if d in digit_circles:
                cx, cy, r = digit_circles[d]

                # Mostrar posición en píxeles
                self.get_logger().info(f"C{d} -> px: X={cx}, Y={cy}")

                # Convertir a coordenadas en metros usando el modelo pinhole
                X_m = (cx - self.cx) * (self.Z / self.fx)
                Y_m = (cy - self.cy) * (self.Z / self.fy)

                # Si se dispone de TF, transformar al frame "world"
                if transform_stamped is not None:
                    point_in_camera = geometry_msgs.msg.PointStamped()
                    point_in_camera.header.frame_id = "camera"
                    point_in_camera.header.stamp = self.get_clock().now().to_msg()
                    # Ajusta los signos según la orientación de tu cámara
                    point_in_camera.point.x = -X_m
                    point_in_camera.point.y = -Y_m
                    point_in_camera.point.z = self.Z

                    point_in_world = do_transform_point(point_in_camera, transform_stamped)
                    world_point = Point(
                        x=point_in_world.point.x,
                        y=point_in_world.point.y,
                        z=point_in_world.point.z
                    )
                    self.get_logger().info(
                        f"C{d} en world: x={world_point.x:.2f}, "
                        f"y={world_point.y:.2f}, z={world_point.z:.2f}"
                    )
                else:
                    # Si no hay TF, se usan las coordenadas calculadas en cámara (o se omite)
                    world_point = Point(x=-X_m, y=-Y_m, z=self.Z)
                    self.get_logger().info(f"C{d} sin TF: X_m={X_m:.2f}, Y_m={Y_m:.2f}")

                marker_points.append(world_point)

                # Agregar a PoseArray (opcional)
                pose = Pose()
                pose.position = world_point
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)

        # Publicar el Marker único (si se detectaron al menos 1 de los 3 dígitos)
        if marker_points:
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "world"
            marker.ns = "digit_circle_points"
            marker.id = 0
            marker.type = Marker.SPHERE_LIST  # Tipo que permite visualizar varios puntos
            marker.action = Marker.ADD
            marker.points = marker_points
            # Tamaño de cada esfera
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            # Color (por ejemplo, verde)
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            self.marker_pub.publish(marker)
            self.poses_pub.publish(pose_array)

        # Mostrar la imagen con las anotaciones
        cv2.imshow("Digit Circle Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DigitCircleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
