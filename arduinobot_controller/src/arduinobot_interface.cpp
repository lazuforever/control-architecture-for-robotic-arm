#include "arduinobot_controller/arduinobot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

// Definición en el espacio de nombres del controlador
namespace arduinobot_controller
{

// Constructor de la clase ArduinobotInterface
ArduinobotInterface::ArduinobotInterface()
{
  // Inicialmente, no hacemos nada especial en el constructor
}

// Destructor: se asegura de cerrar la conexión serie si está abierta
ArduinobotInterface::~ArduinobotInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}

// Método on_init: Se inicializa la interfaz de hardware
CallbackReturn ArduinobotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  // Llamada a la inicialización base
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  // Se extrae el parámetro "port" de la configuración del hardware
  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("ArduinobotInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  // Se intenta leer un parámetro adicional "simulate_mode" para activar el modo simulación
  // Si no se proporciona, se asume false (modo real)
  try
  {
    std::string sim_str = info_.hardware_parameters.at("simulate_mode");
    simulate_mode_ = (sim_str == "true");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_WARN(rclcpp::get_logger("ArduinobotInterface"), "simulate_mode not provided, defaulting to false");
    simulate_mode_ = false;
  }

  // Reservar espacio para los vectores de comandos y estados, según la cantidad de juntas
  position_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  prev_position_commands_.reserve(info_.joints.size());

  return CallbackReturn::SUCCESS;
}

// Exporta las interfaces de estado (por ejemplo, posición) para cada junta
std::vector<hardware_interface::StateInterface> ArduinobotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Se proporciona únicamente la interfaz de posición
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}

// Exporta las interfaces de comando para cada junta (por ejemplo, enviar posiciones)
std::vector<hardware_interface::CommandInterface> ArduinobotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Se proporciona únicamente la interfaz de posición
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}

// on_activate se llama cuando se activa el hardware, para iniciar la conexión
CallbackReturn ArduinobotInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Starting robot hardware ...");

  // Reiniciar comandos y estados a cero para todas las juntas (aquí se asume 4 juntas)
  position_commands_ = { 0.0, 0.0, 0.0, 0.0 };
  prev_position_commands_ = { 0.0, 0.0, 0.0, 0.0 };
  position_states_ = { 0.0, 0.0, 0.0, 0.0 };

  // Si se activa el modo simulación, se salta la conexión real al hardware
  if (simulate_mode_)
  {
    RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"),
                "Simulate mode enabled. Skipping hardware connection.");
  }
  else
  {
    // Intentar abrir la conexión serie y configurar el baud rate
    try
    {
      arduino_.Open(port_);
      arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                          "Something went wrong while interacting with port " << port_);
      return CallbackReturn::FAILURE;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}

// on_deactivate se llama cuando se detiene el hardware
CallbackReturn ArduinobotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Stopping robot hardware ...");

  // Solo se cierra la conexión si no estamos en modo simulación y el puerto está abierto
  if (!simulate_mode_ && arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

// read: En control abierto, se asume que el estado actual es igual al comando enviado
hardware_interface::return_type ArduinobotInterface::read(const rclcpp::Time &time,
                                                          const rclcpp::Duration &period)
{
  position_states_ = position_commands_;
  return hardware_interface::return_type::OK;
}

// write: Envía comandos al hardware (o simula el envío en modo simulación)
hardware_interface::return_type ArduinobotInterface::write(const rclcpp::Time &time,
                                                           const rclcpp::Duration &period)
{
  // Si los comandos no han cambiado, no se envía nada
  if (position_commands_ == prev_position_commands_)
  {
    return hardware_interface::return_type::OK;
  }

  // Construir el mensaje de comando
  std::string msg;
  
  // Convertir la posición de la junta base de radianes a grados y agregar prefijo "b"
  int base = static_cast<int>(((position_commands_.at(0) + (M_PI / 2)) * 180) / M_PI);
  msg.append("b");
  msg.append(std::to_string(base));
  msg.append(",");

  // Convertir la posición de la junta del hombro, con ajuste, y agregar prefijo "s"
  int shoulder = 180 - static_cast<int>(((position_commands_.at(1) + (M_PI / 2)) * 180) / M_PI);
  msg.append("s");
  msg.append(std::to_string(shoulder));
  msg.append(",");

  // Convertir la posición de la junta del codo y agregar prefijo "e"
  int elbow = static_cast<int>(((position_commands_.at(2) + (M_PI / 2)) * 180) / M_PI);
  msg.append("e");
  msg.append(std::to_string(elbow));
  msg.append(",");

  // Convertir la posición del gripper y agregar prefijo "g"
  int gripper = static_cast<int>(((-position_commands_.at(3)) * 180) / (M_PI / 2));
  msg.append("g");
  msg.append(std::to_string(gripper));
  msg.append(",");

  // En modo simulación, solo se registra el mensaje sin enviarlo al hardware
  if (simulate_mode_)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Simulate mode: Command not sent, but simulated: " << msg);
  }
  else
  {
    // Si no estamos en modo simulación, se intenta enviar el mensaje real a través del puerto
    try
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Sending new command " << msg);
      arduino_.Write(msg);
    }
    catch (...)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                          "Something went wrong while sending the message "
                          << msg << " to the port " << port_);
      return hardware_interface::return_type::ERROR;
    }
  }

  // Actualizar los comandos previos para la siguiente iteración
  prev_position_commands_ = position_commands_;

  return hardware_interface::return_type::OK;
}

}  // namespace arduinobot_controller

// Exportar la clase como plugin para que ros2_control la pueda cargar dinámicamente
PLUGINLIB_EXPORT_CLASS(arduinobot_controller::ArduinobotInterface, hardware_interface::SystemInterface)
