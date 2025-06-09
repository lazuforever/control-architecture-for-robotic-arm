#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@Autor: 古月居(www.guyuehome.com)
@Descripción: Ejemplo de publicación de un tema en ROS2 - Publicación de imágenes
"""
import rclpy                        # Biblioteca de la interfaz de ROS2 para Python
from rclpy.node import Node         # Clase base para crear nodos en ROS2
from sensor_msgs.msg import Image   # Tipo de mensaje para imágenes en ROS2
from cv_bridge import CvBridge      # Clase para la conversión entre imágenes de OpenCV y ROS
import cv2                          # Biblioteca OpenCV para procesamiento de imágenes
"""
Definición de un nodo publicador
"""
class ImagePublisher(Node):

    def __init__(self, name):
        super().__init__(name)                                           # Inicialización de la clase base del nodo ROS2
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)  # Crea un publicador (tipo de mensaje, nombre del tema, longitud de la cola)
        self.timer = self.create_timer(0.1, self.timer_callback)         # Crea un temporizador que ejecuta la función de callback cada 0.1 segundos
        self.cap = cv2.VideoCapture(0)                                   # Inicia la captura de video desde la cámara (dispositivo de cámara por defecto)
        self.cv_bridge = CvBridge()                                      # Crea un objeto de conversión para transformar imágenes entre OpenCV y ROS

    def timer_callback(self):
        ret, frame = self.cap.read()                                     # Captura un fotograma desde la cámara
        
        if ret == True:                                                  # Si la captura es exitosa
            self.publisher_.publish(
                self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))             # Convierte la imagen de OpenCV a formato de ROS y la publica

        self.get_logger().info('Publicando fotograma de video')          # Muestra un mensaje en la terminal indicando que se ha publicado una imagen

def main(args=None):                                 # Función principal del nodo ROS2
    rclpy.init(args=args)                            # Inicializa la interfaz de ROS2 en Python
    node = ImagePublisher("topic_webcam_pub")        # Crea e inicia un nodo de ROS2 con el nombre especificado
    rclpy.spin(node)                                 # Mantiene el nodo en ejecución hasta que ROS2 se cierre
    node.destroy_node()                              # Destruye el nodo antes de cerrar
    rclpy.shutdown()                                 # Cierra la interfaz de ROS2 en Python
