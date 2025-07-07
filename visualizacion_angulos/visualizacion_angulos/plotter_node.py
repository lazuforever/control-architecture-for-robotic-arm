import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import threading
import time
import copy

class Visualizador(Node):
    def __init__(self):
        super().__init__('visualizador_angulos')
        self.serial_time = []
        self.serial_data = []
        self.final_angle = [None] * 4  # Últimos ángulos recibidos
        self.final_time = None
        self.final_history = []  # Historial en forma (timestamp, [ángulos])
        self.lock = threading.Lock()

        self.create_subscription(
            JointState,
            '/serial_joint_states',
            self.serial_callback,
            10
        )
        self.create_subscription(
            JointState,
            '/final_angles',
            self.final_callback,
            10
        )

    def serial_callback(self, msg):
        with self.lock:
            if len(msg.position) >= 4:
                current_time = time.time()
                self.serial_time.append(current_time)
                self.serial_data.append(msg.position[:4])

    def final_callback(self, msg):
        with self.lock:
            if len(msg.position) >= 4:
                self.final_angle = msg.position[:4]
                self.final_time = time.time()
                self.final_history.append((self.final_time, list(self.final_angle)))
            base, shoulder_a, elbow, gripper = self.final_angle

            log_message = (
                f"Base = {int(base)}\n"
                f"ShoulderA = {int(shoulder_a)}\n"
                f"ElBow = {int(elbow)}\n"
                f"Gripper = {int(gripper)}"
            )

            self.get_logger().info(log_message)

def start_ros_spin(node):
    rclpy.spin(node)

def plot_live(node):
    fig, ax = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    fig.suptitle('Evolución de Ángulos en Tiempo Real')

    # Líneas para los ángulos serial y final
    lines = []
    final_lines = []
    annotations = []  # Lista para almacenar las anotaciones

    for i in range(4):
        line1, = ax[i].plot([], [], 'r-', label='Serial')  # Rojo = ángulos en tiempo real
        line2, = ax[i].plot([], [], 'b-', label='Final')   # Azul = ángulos finales (setpoint)
        lines.append(line1)
        final_lines.append(line2)

        ax[i].set_ylabel(f'Ángulo {["Base", "ShoulderA", "ElBow", "Gripper"][i]} (°)')
        ax[i].grid(True)
        ax[i].legend(loc='upper right')
        ax[i].set_ylim(-100, 300)

        # Crear anotación para este subplot
        annotation = ax[i].text(0.05, 0.9, '', transform=ax[i].transAxes, fontsize=12, color='blue')
        annotations.append(annotation)

    ax[-1].set_xlabel('Tiempo (s)')
    window_size = 10.0  # segundos

    def update_plot(frame):
        with node.lock:
            serial_time = node.serial_time.copy()
            serial_data = np.array(node.serial_data).copy() if node.serial_data else np.empty((0, 4))
            final_history = copy.deepcopy(node.final_history)

        current_time = time.time()
        xlim_left = current_time - window_size
        xlim_right = current_time

        # Filtrar datos de serial
        if len(serial_time) > 0:
            serial_mask = np.logical_and(np.array(serial_time) >= xlim_left, np.array(serial_time) <= xlim_right)
            serial_time_window = np.array(serial_time)[serial_mask]
            serial_data_window = serial_data[serial_mask]
        else:
            serial_time_window = np.array([])
            serial_data_window = np.empty((0, 4))

        # Obtener los últimos ángulos finales
        last_final_angles = None
        if final_history:
            last_final_angles = final_history[-1][1]

        for i in range(4):
            # Actualizar curva serial
            if len(serial_time_window) > 0:
                lines[i].set_data(serial_time_window, serial_data_window[:, i])
            else:
                lines[i].set_data([], [])

            # Dibujar línea escalonada de los ángulos finales (setpoints)
            x_vals = []
            y_vals = []
            for j in range(len(final_history) - 1):
                t1, angles1 = final_history[j]
                t2, _ = final_history[j + 1]
                if t2 >= xlim_left:
                    x_vals.extend([t1, t2])
                    y_vals.extend([angles1[i], angles1[i]])
            # Último valor hasta el presente
            if final_history:
                t_last, angles_last = final_history[-1]
                if t_last >= xlim_left:
                    x_vals.extend([t_last, current_time])
                    y_vals.extend([angles_last[i], angles_last[i]])

            final_lines[i].set_data(x_vals, y_vals)

            # Actualizar la anotación con el último ángulo final
            if last_final_angles is not None and i < len(last_final_angles):
                annotations[i].set_text(f'Valor: {int(last_final_angles[i])}°')
            else:
                annotations[i].set_text('Valor: -')

        # Limitar el eje X
        ax[-1].set_xlim(xlim_left, xlim_right)

        return lines + final_lines + annotations  # Incluir las anotaciones en la lista de elementos a actualizar

    ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=True)
    plt.tight_layout()
    plt.show()

    def update_plot(frame):
        with node.lock:
            serial_time = node.serial_time.copy()
            serial_data = np.array(node.serial_data).copy() if node.serial_data else np.empty((0, 4))
            final_history = copy.deepcopy(node.final_history)

        current_time = time.time()
        xlim_left = current_time - window_size
        xlim_right = current_time

        # Filtrar datos de serial
        if len(serial_time) > 0:
            serial_mask = np.logical_and(np.array(serial_time) >= xlim_left, np.array(serial_time) <= xlim_right)
            serial_time_window = np.array(serial_time)[serial_mask]
            serial_data_window = serial_data[serial_mask]
        else:
            serial_time_window = np.array([])
            serial_data_window = np.empty((0, 4))

        for i in range(4):
            # Actualizar curva serial
            if len(serial_time_window) > 0:
                lines[i].set_data(serial_time_window, serial_data_window[:, i])
            else:
                lines[i].set_data([], [])

            # Dibujar línea escalonada de los ángulos finales (setpoints)
            x_vals = []
            y_vals = []
            for j in range(len(final_history) - 1):
                t1, angles1 = final_history[j]
                t2, _ = final_history[j + 1]
                if t2 >= xlim_left:
                    x_vals.extend([t1, t2])
                    y_vals.extend([angles1[i], angles1[i]])
            # Último valor hasta el presente
            if final_history:
                t_last, angles_last = final_history[-1]
                if t_last >= xlim_left:
                    x_vals.extend([t_last, current_time])
                    y_vals.extend([angles_last[i], angles_last[i]])

            final_lines[i].set_data(x_vals, y_vals)

        # Limitar el eje X
        ax[-1].set_xlim(xlim_left, xlim_right)

        return lines + final_lines

    ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=True)
    plt.tight_layout()
    plt.show()

def main():
    rclpy.init()
    node = Visualizador()

    ros_thread = threading.Thread(target=start_ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        plot_live(node)
    except KeyboardInterrupt:
        print("\nDetenido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
