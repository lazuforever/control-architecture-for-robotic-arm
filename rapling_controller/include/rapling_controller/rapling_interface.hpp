#ifndef ARDUINOBOT_INTERFACE_H
#define ARDUINOBOT_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <array>
#include <string>

namespace rapling_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RaplingInterface : public hardware_interface::SystemInterface
{
public:
  RaplingInterface();
  virtual ~RaplingInterface();

  // Lifecycle callbacks
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // Hardware interface
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;
  virtual hardware_interface::return_type write(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // Serial port
  LibSerial::SerialPort arduino_;
  std::string port_;

  // Joint command/state buffers
  std::vector<double> position_commands_;
  std::vector<double> prev_position_commands_;
  std::vector<double> position_states_;

  // Angles read back from Arduino
  std::array<int, 4> ANGULOSROBOT_;

  // Publisher for the set-point JointState message
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr setpoint_pub_;
};

}  // namespace rapling_controller

#endif  // ARDUINOBOT_INTERFACE_H
