#include "rapling_controller/rapling_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rapling_controller
{
RaplingInterface::RaplingInterface()
{
  // Crear el nodo para el publisher
  node_ = rclcpp::Node::make_shared("rapling_serial_publisher");
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

  // Provide only a position Interface
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

  // Provide only a position Interface
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

  // Crear el publisher para los ángulos leídos del serial
  serial_angles_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("serial_joint_states", 10);

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

  RCLCPP_INFO(rclcpp::get_logger("RaplingInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RaplingInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("RaplingInterface"), "Stopping robot hardware ...");

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

  RCLCPP_INFO(rclcpp::get_logger("RaplingInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

double RaplingInterface::degreesToRadians(int degrees)
{
  return (degrees * M_PI) / 180.0;
}

hardware_interface::return_type RaplingInterface::read(const rclcpp::Time &time,
                                                       const rclcpp::Duration &period)
{
  // Open Loop Control - assuming the robot is always where we command to be
  position_states_ = position_commands_;
  //  position_states_ = (ANGULOSROBOT_);


 /* 
     position_states_ = {ANGULOSROBOT_[0],
                ANGULOSROBOT_[1],
                ANGULOSROBOT_[2],
                ANGULOSROBOT_[4]};
 
 
 try {
    // 1) Bloquea hasta recibir '\n'
    std::string line;
    arduino_.ReadLine(line, '\n');   // line = "123,456,789,012\n"

    // 2) Tokeniza y convierte a enteros
    std::stringstream ss(line);
    std::string token;
    int idx = 0;
    while (std::getline(ss, token, ',') && idx < 5) {
      ANGULOSROBOT_[idx++] = std::stoi(token);
    }
    
    // 3) Imprimir en consola
   /* RCLCPP_INFO(rclcpp::get_logger("RaplingInterface"),
                "ANGULOS ACTUALES -> Base: %d, Shoulder: %d, Elbow: %d, Gripper: %d",
                ANGULOSROBOT_[0],
                ANGULOSROBOT_[1],
                ANGULOSROBOT_[2],
                ANGULOSROBOT_[3]); */

    // 4) Publicar el JointState con los valores del serial
     /* 
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = time;  // Usar el tiempo actual
    msg.name = {"base", "shoulder","shoulderB" "elbow", "gripper"};
    
    //
    msg.position = {
                ANGULOSROBOT_[0],
                ANGULOSROBOT_[1],
                ANGULOSROBOT_[2],
                ANGULOSROBOT_[3],
                ANGULOSROBOT_[4]
    };
    
    // Publicar el mensaje
    serial_angles_pub_->publish(msg);
    /*
    RCLCPP_INFO(rclcpp::get_logger("RaplingInterface"),
                "Publicado JointState con valores en radianes: [%.3f, %.3f, %.3f, %.3f]",
                msg.position[0], msg.position[1], msg.position[2], msg.position[3]); */
     /* 

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("RaplingInterface"),
                 "Error leyendo de serial: %s", e.what());
  } */

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


  std::string msg;
  int base = static_cast<int>(((position_commands_.at(0) )) * (180/M_PI)+23);
  msg.append("b");
  msg.append(std::to_string(base));
  msg.append(",");


// COMO LOS MOTORES ESTAN INVERTIDOS ENTONCES LOS ANGULOS SE DEBEN RESTAR Y SUMAR, ES DECIR SI EL CENTRO ES 150ª, PARA EL PRIMER MOTOR RECORRE HACIA ABAJO Y ARRIBA EL SEGUNDO ESO HACE QUE AL ESTAR INVERTIDOS EN EL  MONTAJE, AL INVERTIR LOS SENTIDOS ENTONCES GIREN EN LA MISMA DIRECCIÒN 

//-------------------------------------//---------------------------------//
  int shoulderA =  static_cast<int>((-1*(position_commands_.at(1) )) * (180/M_PI) + 240);
  msg.append("s");

  msg.append(std::to_string(shoulderA));
  msg.append(",");
  
  int shoulderB =  static_cast<int>((((position_commands_.at(1))) * (180/M_PI) ) + 60);
  msg.append("p ");

  msg.append(std::to_string(shoulderB));
  msg.append(",");
//-------------------------------------//----------------------------------//


  int elbow = static_cast<int>(((position_commands_.at(2) )) * (180/M_PI) + 150);
  msg.append("e");
  msg.append(std::to_string(elbow));
  msg.append(",");

  int gripper = static_cast<int>(((position_commands_.at(3) )) * (180/M_PI) + 150);
  msg.append("g");
  msg.append(std::to_string(gripper));
  msg.append(",");

  try
  {
    /*RCLCPP_INFO_STREAM(rclcpp::get_logger("RaplingInterface"),
                     "Comandos de posición (radianes): " 
                     << position_commands_.at(0) << ", " 
                     << position_commands_.at(1) << ", "
                     << shoulderA << ","
                     << position_commands_.at(2) << ", " 
                     << position_commands_.at(3));

    RCLCPP_INFO_STREAM(rclcpp::get_logger("RaplingInterface"), "Sending new command " << msg);*/

    arduino_.Write(msg);
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RaplingInterface"),
                        "Something went wrong while sending the message "
                            << msg << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  prev_position_commands_ = position_commands_;

  return hardware_interface::return_type::OK;
}

}  // namespace rapling_controller

PLUGINLIB_EXPORT_CLASS(rapling_controller::RaplingInterface, hardware_interface::SystemInterface)