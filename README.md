Robot Antropomórfico con Control Multimodal - ROS2
Sistema de control para robot antropomórfico de 4 grados de libertad que integra visión artificial, comandos de voz y planificación de trayectorias mediante ROS2 Humble.
🎯 Descripción General
Este proyecto implementa una arquitectura distribuida en ROS2 para el control de un robot manipulador mediante interacción multimodal, combinando:

Comandos de voz a través de Amazon Alexa
Visión artificial con detección de gestos mediante MediaPipe
Planificación de trayectorias usando MoveIt2
Control de hardware mediante interfaz personalizada ros2_control

✨ Características Principales

✅ Interfaz de hardware personalizada para comunicación serial con motores Dynamixel
✅ Detección de posición de mano en 3D usando MediaPipe Hands
✅ Integración con Alexa Skills Kit para control por voz
✅ Planificación automática de trayectorias con MoveIt2 y OMPL
✅ Visualización en tiempo real en RViz2
✅ Operaciones Pick & Place mediante gestos

🏗️ Arquitectura del Sistema
┌─────────────────────────────────────────────────┐
│           INTERACCIÓN MULTIMODAL                │
├──────────────────┬──────────────────────────────┤
│  Alexa Interface │    Finger Detector (Visión)  │
│   (Flask + ASK)  │    (MediaPipe + OpenCV)      │
└────────┬─────────┴──────────┬───────────────────┘
         │                    │
         ▼                    ▼
    ┌────────────────────────────────┐
    │      Task Server (C++)         │
    │    (Action Server ROS2)        │
    └────────────┬───────────────────┘
                 │
                 ▼
         ┌───────────────┐
         │    MoveIt2    │
         │  (Planning)   │
         └───────┬───────┘
                 │
                 ▼
         ┌───────────────┐
         │ ros2_control  │
         │  Controller   │
         │    Manager    │
         └───────┬───────┘
                 │
                 ▼
         ┌───────────────┐
         │  Rapling      │
         │  Interface    │
         │  (Hardware)   │
         └───────┬───────┘
                 │
                 ▼
         ┌───────────────┐
         │  ESP32 +      │
         │  Dynamixel    │
         │   AX-12A      │
         └───────────────┘
```

## 🛠️ Tecnologías Utilizadas

### Sistema Operativo y Framework
- **Ubuntu 22.04 LTS** - Plataforma base con soporte hasta 2027
- **ROS2 Humble Hawksbill** - Framework de robótica distribuida

### Control y Planificación
- **MoveIt2** - Planificación de trayectorias y cinemática inversa
- **ros2_control** - Framework de control de hardware
- **OMPL** - Librería de planificación de movimientos
- **Plugin personalizado** - Interfaz de hardware para comunicación serial

### Visión Artificial
- **MediaPipe Hands** - Detección de 21 landmarks de mano en tiempo real
- **OpenCV** - Procesamiento de imagen y calibración de cámara
- **TF2** - Transformaciones entre sistemas de coordenadas

### Interacción de Voz
- **Amazon Alexa Skills Kit** - Procesamiento de comandos de voz
- **Flask** - Servidor web para comunicación con Alexa
- **ngrok** - Túnel HTTPS para desarrollo local

### Hardware
- **ESP32** - Microcontrolador dual-core para gestión de motores
- **Dynamixel AX-12A** - Servomotores inteligentes con retroalimentación
- **Buffer 74LS241** - Adaptación de señal half-duplex TTL
- **Cámara Logitech C920s** - Sensor RGB para visión artificial

## 📦 Estructura de Paquetes
```
src/
├── rapling_description/       # Modelo URDF del robot
│   ├── urdf/                  # Archivos Xacro con geometría
│   ├── meshes/                # Modelos 3D (STL)
│   └── launch/                # Visualización en RViz
│
├── rapling_controller/        # Control de hardware
│   ├── include/               # Plugin de interfaz (HPP)
│   ├── src/                   # Implementación serial (CPP)
│   └── config/                # Controladores y límites
│
├── rapling_moveit_2/          # Configuración MoveIt2
│   ├── config/                # SRDF, cinemática, límites
│   └── launch/                # Move group + RViz
│
├── rapling_remote/            # Integración multimodal
│   ├── alexa_interface.py     # Servidor Flask + Alexa
│   └── task_server.cpp        # Action server de tareas
│
├── learning_topic/            # Visión artificial
│   ├── finger_detector.py     # Detección con MediaPipe
│   └── topic_webcam_pub.py    # Publicador de imagen
│
└── rapling_msgs/              # Mensajes personalizados
    └── action/                # Definición de acciones
```

## 🔧 Implementación Técnica

### 1. **Modelo Digital del Robot**
- Exportación desde SolidWorks usando plugin URDF Exporter
- Definición de 4 articulaciones rotacionales con límites
- Integración con ros2_control mediante tags Xacro
- Calibración de offsets entre cero simulado y cero físico

### 2. **Interfaz de Hardware (rapling_controller)**
- Plugin C++ basado en `hardware_interface::SystemInterface`
- Comunicación serial a 115200 baudios con ESP32
- Exportación de `StateInterface` y `CommandInterface` por articulación
- Conversión automática radianes ↔ grados con compensación de offsets
- Publicación de `JointState` para retroalimentación en tiempo real

### 3. **Sistema de Visión (finger_detector)**
- Captura de frames desde `/image_raw` (10 Hz)
- Detección de landmark 8 (punta del índice) con MediaPipe
- Conversión píxel → coordenadas 3D usando parámetros intrínsecos
- Transformación al frame `world` mediante TF2
- Detección de gesto: mano cerrada + posición estable (3s) = punto guardado
- Publicación de marcadores como `visualization_msgs/MarkerArray`

### 4. **Integración con Alexa (alexa_interface.py)**
- Servidor Flask en puerto 5000 expuesto mediante ngrok
- Handlers para intents: `LaunchRequest`, `PickIntent`, `WakeIntent`, `SleepIntent`
- Suscripción al tópico `/finger_poses` para obtener coordenadas
- Envío de goals al Action Server usando `rclpy.action.ActionClient`
- Ejecución en hilo separado para compatibilidad Flask + ROS2 spin

### 5. **Servidor de Tareas (task_server.cpp)**
- Action Server basado en `rclcpp_action`
- Recepción de goals con campo `task_number` (0-9)
- Interfaz con MoveIt2 mediante `MoveGroupInterface`
- Planificación con OMPL y parametrización temporal IPTP
- Ejecución mediante `arm_controller` (JointTrajectoryController)

### 6. **Comunicación Hardware**
- ESP32 con arquitectura dual-core:
  - **Core 0**: Lectura de encoders + envío de estados
  - **Core 1**: Procesamiento de comandos + control de motores
- Protocolo Dynamixel con daisy-chain en bus compartido
- Control de dirección half-duplex mediante GPIO4
- Compensación de motores acoplados (M2A/M2B en hombro)

## 🚀 Flujo de Operación

### Ejemplo: Pick & Place con Gestos

1. **Captura de Puntos**
```
   Usuario → Muestra índice → MediaPipe detecta → Convierte a 3D
   → Cierra puño 3s → Guarda punto 1 (Pick)
   → Repite → Guarda punto 2 (Place)
```

2. **Comando de Voz**
```
   Usuario: "Alexa, activar robot"
   Alexa → Flask → Obtiene puntos de /finger_poses
   
   Usuario: "Alexa, ejecutar movimiento"
   Alexa → Flask → Envía goal(task_number=1) → Task Server
```

3. **Planificación y Ejecución**
```
   Task Server → MoveIt2 planifica trayectoria
   → Genera waypoints articulares
   → IPTP parametriza tiempos/velocidades
   → arm_controller ejecuta
   → Rapling Interface convierte rad→grados+offset
   → ESP32 envía comandos a Dynamixels
📊 Resultados

✅ Conversión precisa entre cero simulado y físico de motores
✅ Detección estable de mano con precisión < 30px de desviación
✅ Trayectorias suaves con perfiles trapezoidales de velocidad
✅ Tiempo de respuesta < 2s desde comando de voz hasta inicio de movimiento
✅ Sincronización exitosa entre planificación y hardware real

💻 Requisitos

Ubuntu 22.04 LTS
ROS2 Humble
Python 3.10+ con: opencv-python, mediapipe, flask, ask-sdk-core
Hardware: ESP32, 4x Dynamixel AX-12A, Cámara USB, Alexa device

📖 Uso Rápido
bash# Visualización del modelo
ros2 launch rapling_description display.launch.py

# Sistema completo
ros2 launch rapling_controller controller.launch.py
ros2 launch rapling_moveit_2 moveit.launch.py
ros2 launch learning_topic vision_launch.py
python3 src/rapling_remote/alexa_interface.py
📝 Documentación
Este proyecto fue desarrollado como trabajo de grado en la Universidad de Pamplona, implementando metodologías modernas de robótica distribuida con interacción natural persona-robot.
📧 Contacto
Brayan Dayani

📧 Email: brayandayani@hotmail.com
📱 Teléfono: +57 312 364 4501
📍 Ubicación: Colombia

Para consultas, sugerencias o colaboraciones sobre el proyecto, no dudes en contactarme.

Desarrollado con ROS2 Humble | MoveIt2 | MediaPipe | Alexa Skills Kit
