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
import pytesseract  # Asegúrate de tener instalado pytesseract

class RectangleDetector(Node):
    def __init__(self):
        super().__init__('rectangle_detector')

        # Suscriptor a la cámara
        self.subscriber = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )
        # Publicador de markers para RViz
        self.marker_pub = self.create_publisher(Marker, 'rectangle_marker', 10)

        # Conversión entre ROS y OpenCV
        self.cv_bridge = CvBridge()

        # TF2: Buffer y Listener para obtener transformaciones
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Parámetros de filtrado de rectángulos
        self.min_rect_area = 1000       # Área mínima del bounding box (px^2)
        self.fill_ratio_min = 0.8       # Relación mínima contorno_area / boundingRect_area

        # Distancia estimada al objeto (en metros). Ajusta según tu setup.
        self.Z = -0.25  # Ejemplo: 25 cm

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
        """
        Callback para cada imagen recibida de la cámara.
        Se detectan cuadrados, se extrae el dígito de cada uno mediante OCR y se publican sus posiciones.
        """
        # Convertir imagen ROS a OpenCV
        frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        height, width, _ = frame.shape

        # Definir ROI central (80% del ancho y alto)
        roi_w = int(width * 0.8)
        roi_h = int(height * 0.8)
        roi_x = (width - roi_w) // 2
        roi_y = (height - roi_h) // 2
        roi_frame = frame[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w].copy()

        # Convertir a gris, aplicar blur y Canny para extraer bordes
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # Encontrar contornos en la ROI
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Lista para almacenar la información de cada cuadrado detectado:
        # Cada elemento es una tupla: (dígito, center_x, center_y)
        detected_rectangles = []

        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
            # Verificamos que el contorno tenga 4 vértices y sea convexo
            if len(approx) == 4 and cv2.isContourConvex(approx):
                area = cv2.contourArea(approx)
                if area < self.min_rect_area:
                    continue

                x, y, w, h = cv2.boundingRect(approx)
                fill_ratio = area / (w * h)
                if fill_ratio < self.fill_ratio_min:
                    continue

                # Recortar internamente para eliminar bordes (aumenta el margen si es necesario)
                margin = 10
                x1 = max(x + margin, 0)
                y1 = max(y + margin, 0)
                x2 = min(x + w - margin, roi_frame.shape[1] - 1)
                y2 = min(y + h - margin, roi_frame.shape[0] - 1)
                roi_digit = roi_frame[y1:y2, x1:x2]

                # Preprocesar la subimagen para OCR: convertir a gris y threshold
                roi_digit_gray = cv2.cvtColor(roi_digit, cv2.COLOR_BGR2GRAY)
                _, roi_thresh = cv2.threshold(roi_digit_gray, 0, 255,
                                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)

                # Operación morfológica de apertura para limpiar el borde
                kernel = np.ones((3, 3), np.uint8)
                roi_clean = cv2.morphologyEx(roi_thresh, cv2.MORPH_OPEN, kernel)

                # (Opcional) Invertir la imagen si el dígito es oscuro sobre fondo claro
                roi_inverted = cv2.bitwise_not(roi_clean)

                # Configurar pytesseract para reconocer un solo dígito
                config = "--psm 10 -c tessedit_char_whitelist=0123456789"
                digit_text = pytesseract.image_to_string(roi_inverted, config=config).strip()

                if digit_text.isdigit():
                    digit = int(digit_text)
                    # Calcular el centro del cuadrado (en coordenadas de la ROI)
                    center_x_roi = x + w // 2
                    center_y_roi = y + h // 2
                    # Convertir a coordenadas de la imagen completa
                    center_x = center_x_roi + roi_x
                    center_y = center_y_roi + roi_y
                    detected_rectangles.append((digit, center_x, center_y))
                    # Dibujar el cuadrado y el dígito sobre la imagen para depuración
                    cv2.rectangle(frame, (x + roi_x, y + roi_y),
                                  (x + w + roi_x, y + h + roi_y), (0, 255, 0), 2)
                    cv2.putText(frame, f"{digit}", (center_x + 10, center_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                    self.get_logger().info(f"Cuadrado con número {digit} detectado en (px): X={center_x}, Y={center_y}")
                else:
                    self.get_logger().info("No se reconoció dígito en un cuadrado")

        # Publicar un único Marker con todos los puntos detectados
        marker_points = []
        transform_stamped = self.get_camera_transform()
        for (digit, cx, cy) in detected_rectangles:
            # Convertir la posición en píxeles a coordenadas en metros (modelo pinhole)
            X_m = (cx - self.cx) * (self.Z / self.fx)
            Y_m = (cy - self.cy) * (self.Z / self.fy)
            if transform_stamped is not None:
                point_in_camera = geometry_msgs.msg.PointStamped()
                point_in_camera.header.frame_id = "camera"
                point_in_camera.header.stamp = self.get_clock().now().to_msg()
                point_in_camera.point.x = -X_m
                point_in_camera.point.y = -Y_m
                point_in_camera.point.z = self.Z

                point_in_world = do_transform_point(point_in_camera, transform_stamped)
                marker_points.append(point_in_world.point)
                self.get_logger().info(
                    f"Cuadrado {digit} en world: x={point_in_world.point.x:.2f}, "
                    f"y={point_in_world.point.y:.2f}, z={point_in_world.point.z:.2f}"
                )
            else:
                from geometry_msgs.msg import Point
                marker_points.append(Point(x=-X_m, y=-Y_m, z=self.Z))
                self.get_logger().info(f"Cuadrado {digit} (sin TF): X_m={X_m:.2f}, Y_m={Y_m:.2f}")

        if marker_points:
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "world"
            marker.ns = "rectangle_points"
            marker.id = 0
            marker.type = Marker.SPHERE_LIST  # Visualiza varios puntos en un único marker
            marker.action = Marker.ADD
            marker.points = marker_points
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            self.marker_pub.publish(marker)

        # Mostrar la imagen resultante
        cv2.imshow("Rectangle Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RectangleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
