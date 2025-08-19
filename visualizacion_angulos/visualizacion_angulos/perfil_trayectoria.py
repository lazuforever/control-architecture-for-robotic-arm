#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import threading
import time

class TrajectoryStreamer(Node):
    def __init__(self):
        super().__init__('trajectory_streamer')
        self.lock = threading.Lock()

        self.joint_names = ['joint_link_1', 'joint_link_2', 'joint_link_3', 'joint_link_4']
        # Ahora almacenamos: (t_real, positions, velocities, accelerations)
        self.data = []  

        self.create_subscription(JointTrajectory, '/planned_trajectory', self.callback, 10)

    def callback(self, msg: JointTrajectory):
        threading.Thread(target=self._play, args=(msg,), daemon=True).start()

    def _play(self, msg: JointTrajectory):
        t0_wall = time.time()
        for pt in msg.points:
            delay = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            sleep_time = t0_wall + delay - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # Manejo seguro de datos faltantes
            positions = list(pt.positions)
            velocities = list(pt.velocities) if pt.velocities else [0.0] * len(positions)
            accelerations = list(pt.accelerations) if pt.accelerations else [0.0] * len(positions)
            
            with self.lock:
                self.data.append((time.time(), positions, velocities, accelerations))

# ------------------------------------------------------
def plot_realtime(node: TrajectoryStreamer):
    # Crear 3 subplots verticales
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    fig.suptitle("Trayectoria en Tiempo Real")
    
    # Configurar ejes y títulos
    y_labels = [
        "Posición (rad)",
        "Velocidad (rad/s)",
        "Aceleración (rad/s²)"
    ]
    
    for i, ax in enumerate(axs):
        ax.set_ylabel(y_labels[i])
        ax.grid(True)
        if i == 2:
            ax.set_xlabel("Tiempo real (s)")

    # Colores y líneas para cada articulación
    colors = plt.cm.tab10.colors
    plot_lines = [[], [], []]  # Para pos, vel, acc
    
    # Inicializar líneas en cada subplot
    for j, name in enumerate(node.joint_names):
        # Posición
        line_pos, = axs[0].plot([], [], color=colors[j], label=name)
        plot_lines[0].append(line_pos)
        
        # Velocidad
        line_vel, = axs[1].plot([], [], color=colors[j])
        plot_lines[1].append(line_vel)
        
        # Aceleración
        line_acc, = axs[2].plot([], [], color=colors[j])
        plot_lines[2].append(line_acc)
    
    axs[0].legend(loc='upper right')
    window = 10.0  # Ventana temporal deslizante

    def animate(_):
        with node.lock:
            if not node.data:
                return [line for sublist in plot_lines for line in sublist]
                
            t0 = node.data[0][0]
            t = [d[0] - t0 for d in node.data]
            positions = np.array([d[1] for d in node.data])
            velocities = np.array([d[2] for d in node.data])
            accelerations = np.array([d[3] for d in node.data])
            
        current_time = t[-1]
        xmin = max(0, current_time - window)
        xmax = current_time + 1
        
        # Actualizar datos de posición
        for j, line in enumerate(plot_lines[0]):
            line.set_data(t, positions[:, j])
            axs[0].set_xlim(xmin, xmax)
            axs[0].relim()
            axs[0].autoscale_view(scaley=True)
        
        # Actualizar datos de velocidad
        for j, line in enumerate(plot_lines[1]):
            line.set_data(t, velocities[:, j])
            axs[1].set_xlim(xmin, xmax)
            axs[1].relim()
            axs[1].autoscale_view(scaley=True)
        
        # Actualizar datos de aceleración
        for j, line in enumerate(plot_lines[2]):
            line.set_data(t, accelerations[:, j])
            axs[2].set_xlim(xmin, xmax)
            axs[2].relim()
            axs[2].autoscale_view(scaley=True)
        
        # Retornar todas las líneas para la animación
        return [line for sublist in plot_lines for line in sublist]

    ani = animation.FuncAnimation(fig, animate, interval=100, blit=False)
    plt.tight_layout(rect=[0, 0, 1, 0.96])  # Ajustar para el título principal
    plt.show()

# ------------------------------------------------------
def main():
    rclpy.init()
    node = TrajectoryStreamer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        plot_realtime(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()