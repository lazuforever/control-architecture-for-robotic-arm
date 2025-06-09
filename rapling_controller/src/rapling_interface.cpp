#include "rapling_controller/rapling_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace rapling_controller
{
RaplingInterface::RaplingInterface()
{
}


RaplingInterface::~RaplingInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("RaplingInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn RaplingInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("RaplingInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  position_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  prev_position_commands_.reserve(info_.joints.size());

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> RaplingInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> RaplingInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn RaplingInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("RaplingInterface"), "Starting robot hardware ...");

  // Reset commands and states
  position_commands_ = { 0.0, 0.0, 0.0, 0.0 };
  prev_position_commands_ = { 0.0, 0.0, 0.0, 0.0 };
  position_states_ = { 0.0, 0.0, 0.0, 0.0 };

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("RaplingInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn RaplingInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Stopping robot hardware ...");

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

  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type RaplingInterface::read(const rclcpp::Time &time,
                                                          const rclcpp::Duration &period)
{
  // Open Loop Control - assuming the robot is always where we command to be
  position_states_ = position_commands_;


  try {
    // 1) Bloquea hasta recibir '\n'
    std::string line;
    arduino_.ReadLine(line, '\n');   // line = "123,456,789,012\n"

    // 2) Tokeniza y convierte a enteros
    std::stringstream ss(line);
    std::string token;
    int idx = 0;
    while (std::getline(ss, token, ',') && idx < 4) {
      ANGULOSROBOT_[idx++] = std::stoi(token);
    }

    // 3) ¡Aquí imprimes en consola!
   /* RCLCPP_INFO(rclcpp::get_logger("RaplingInterface"),
                "Lectura serial -> Base: %d, Shoulder: %d, Elbow: %d, Gripper: %d",
                ANGULOSROBOT_[0],
                ANGULOSROBOT_[1],
                ANGULOSROBOT_[2],
                ANGULOSROBOT_[3]); */

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("RaplingInterface"),
                 "Error leyendo de serial: %s", e.what());
  }



  return hardware_interface::return_type::OK;
}



hardware_interface::return_type RaplingInterface::write(const rclcpp::Time &time,
                                                           const rclcpp::Duration &period)
{
  if (position_commands_ == prev_position_commands_)
  {
    // Nothing changed, do not send any command
    return hardware_interface::return_type::OK;
  }


  // M_PI / 2 = 90°, se usa como offset de referencia.
  std::string msg;
  int base = static_cast<int>(((position_commands_.at(0) + (M_PI / 2)) * 180) / M_PI);
  msg.append("b");
  msg.append(std::to_string(base));
  msg.append(",");
  int shoulder = 180 - static_cast<int>(((position_commands_.at(1) + (M_PI / 2)) * 180) / M_PI);
  msg.append("s");
  msg.append(std::to_string(shoulder));
  msg.append(",");
  int elbow = static_cast<int>(((position_commands_.at(2) + (M_PI / 2)) * 180) / M_PI);
  msg.append("e");
  msg.append(std::to_string(elbow));
  msg.append(",");
  int gripper = static_cast<int>(((-position_commands_.at(3)) * 180) / (M_PI / 2));
  msg.append("g");
  msg.append(std::to_string(gripper));
  msg.append(",");

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

  prev_position_commands_ = position_commands_;

  return hardware_interface::return_type::OK;
}
}  // namespace arduinobot_controller

PLUGINLIB_EXPORT_CLASS(rapling_controller::RaplingInterface, hardware_interface::SystemInterface)