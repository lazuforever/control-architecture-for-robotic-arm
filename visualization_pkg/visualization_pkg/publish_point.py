import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker

class LinePublisher(Node):
    def __init__(self):
        super().__init__('line_publisher')
        self.publisher_ = self.create_publisher(Marker, 'line_topic', 10)
        self.subscription = self.create_subscription(String, 'hand_direction', self.direction_callback, 10)
        self.timer_period = 1.0  # segundos
        self.timer = self.create_timer(self.timer_period, self.publish_line)
        self.x_position = -0.794  # Posición inicial en X
        self.x_increment = 0.1  # Incremento en X por cada iteración
        self.direction = "None"
        self.points = []  # Lista para almacenar la trayectoria
    
    def direction_callback(self, msg):
        self.direction = msg.data

    def publish_line(self):
        if self.direction == "Izquierda":
            self.x_position -= self.x_increment
        elif self.direction == "Derecha":
            self.x_position += self.x_increment

        new_point = Point()
        new_point.x = self.x_position
        new_point.y = 0.209
        new_point.z = 1.457
        self.points.append(new_point)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "world"
        marker.ns = "line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Tipo de marcador
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # Grosor de la línea
        marker.color.a = 1.0  # Opacidad
        marker.color.r = 1.0  # Rojo
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.points = self.points

        self.publisher_.publish(marker)
        self.get_logger().info(f'Publishing line with {len(self.points)} points | Direction: {self.direction}')

def main(args=None):
    rclpy.init(args=args)
    node = LinePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
