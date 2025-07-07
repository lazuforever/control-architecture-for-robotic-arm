#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <cmath>

class VisualizacionAngulos : public rclcpp::Node
{
public:
  VisualizacionAngulos() : Node("visualizacion_angulos")
  {
    RCLCPP_INFO(this->get_logger(), "Iniciando nodo de visualización de ángulos...");

    // Crear suscriptores
    serial_joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/serial_joint_states", 
      10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->serialJointStatesCallback(msg);
      }
    );

    final_angles_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/final_angles", 
      10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->finalAnglesCallback(msg);
      }
    );

    RCLCPP_INFO(this->get_logger(), "Suscripciones creadas:");
    RCLCPP_INFO(this->get_logger(), "  - /serial_joint_states");
    RCLCPP_INFO(this->get_logger(), "  - /final_angles");
    RCLCPP_INFO(this->get_logger(), "Nodo listo para recibir mensajes...");
  }

private:
  void serialJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_serial_msg_ = msg;
    
    RCLCPP_INFO(this->get_logger(), "\n=== SERIAL JOINT STATES ===");
    printJointStates("serial_joint_states", msg);
    
    // Si tenemos ambos mensajes, mostrar comparación
    if (last_final_msg_ && last_serial_msg_) {
      RCLCPP_INFO(this->get_logger(), "\n=== COMPARACIÓN ===");
      for (size_t i = 0; i < std::min(msg->name.size(), last_final_msg_->name.size()); ++i) {
        if (i < msg->position.size() && i < last_final_msg_->position.size()) {
          double diff_rad = msg->position[i] - last_final_msg_->position[i];
          double diff_deg = radiansToDegrees(diff_rad);
          
          RCLCPP_INFO(this->get_logger(), 
            "%s - Diferencia: %.3f rad (%.1f°)", 
            msg->name[i].c_str(), diff_rad, diff_deg);
        }
      }
    }
  }

  void finalAnglesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_final_msg_ = msg;
    
    RCLCPP_INFO(this->get_logger(), "\n=== FINAL ANGLES ===");
    printJointStates("final_angles", msg);
  }

  double radiansToDegrees(double radians)
  {
    return (radians * 180.0) / M_PI;
  }

  void printJointStates(const std::string& topic_name, const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Timestamp: %d.%09d", 
      msg->header.stamp.sec, msg->header.stamp.nanosec);
    
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i < msg->position.size()) {
        double degrees = radiansToDegrees(msg->position[i]);
        
        RCLCPP_INFO(this->get_logger(), 
          "  %s: %.4f rad (%.2f°)", 
          msg->name[i].c_str(), 
          msg->position[i], 
          degrees);
      }
    }
    
    // Si hay velocidades, mostrarlas también
    if (!msg->velocity.empty()) {
      RCLCPP_INFO(this->get_logger(), "Velocidades:");
      for (size_t i = 0; i < std::min(msg->name.size(), msg->velocity.size()); ++i) {
        RCLCPP_INFO(this->get_logger(), 
          "  %s: %.4f rad/s", 
          msg->name[i].c_str(), 
          msg->velocity[i]);
      }
    }
  }

  // Suscriptores
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr serial_joint_states_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr final_angles_sub_;
  
  // Variables para almacenar los últimos valores recibidos
  sensor_msgs::msg::JointState::SharedPtr last_serial_msg_;
  sensor_msgs::msg::JointState::SharedPtr last_final_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<VisualizacionAngulos>();
  
  RCLCPP_INFO(node->get_logger(), "Nodo visualizacion_angulos iniciado");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}