#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "rapling_msgs/action/arduinobot_task.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <thread>
#include <vector>
#include <string>
#include <set>
#include <cmath>
using namespace std::placeholders;

namespace rapling_remote
{
  class TaskServer : public rclcpp::Node
  {
  public:
    explicit TaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("task_server", options)
    {
      RCLCPP_INFO(get_logger(), "Starting the Server");

      // 1) Action Server
      action_server_ = rclcpp_action::create_server<rapling_msgs::action::ArduinobotTask>(
          this, "task_server",
          std::bind(&TaskServer::goalCallback, this, _1, _2),
          std::bind(&TaskServer::cancelCallback, this, _1),
          std::bind(&TaskServer::acceptedCallback, this, _1));

      // 2) Suscripciones
      direction_subscriber_ = this->create_subscription<std_msgs::msg::String>(
          "hand_direction", 10, std::bind(&TaskServer::directionCallback, this, _1));

      finger_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
          "finger_poses", 10, std::bind(&TaskServer::fingerPoseCallback, this, _1));

      // 3) Cliente de acción (para pruebas internas)
      action_client_ = rclcpp_action::create_client<rapling_msgs::action::ArduinobotTask>(
          this, "task_server");

      // 4) Publicador de Marker
      marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
          "line_topic", 10);

      final_angles_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("final_angles", 10);
    }

  private:
    // ---- miembros ROS ----
    rclcpp_action::Server<rapling_msgs::action::ArduinobotTask>::SharedPtr action_server_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr direction_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr finger_pose_subscriber_;
    rclcpp_action::Client<rapling_msgs::action::ArduinobotTask>::SharedPtr action_client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr final_angles_pub_;

    // almacenamiento de la última PoseArray
    geometry_msgs::msg::PoseArray latest_finger_pose_array_;
    std::vector<std::string> trajectory_directions_;

    // ---- callbacks básicos ----
    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const rapling_msgs::action::ArduinobotTask::Goal> goal)
    {
      RCLCPP_INFO(get_logger(), "Received goal request with task number %d", goal->task_number);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<rapling_msgs::action::ArduinobotTask>>)
    {
      RCLCPP_INFO(get_logger(), "Received request to cancel goal");
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void acceptedCallback(
       const std::shared_ptr<rclcpp_action::ServerGoalHandle<rapling_msgs::action::ArduinobotTask>> goal_handle)
    {
      std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
    }

    void directionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
      static const std::set<std::string> valid = {"Izquierda","Derecha","Adelante","Atras","Arriba","Abajo"};
      if (trajectory_directions_.empty() && valid.count(msg->data))
      {
        trajectory_directions_.push_back(msg->data);
        RCLCPP_INFO(get_logger(), "Comando recibido: '%s'", msg->data.c_str());
        // envía task 0 si quieres reutilizar lógica de dirección
      }
    }

    void fingerPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
      latest_finger_pose_array_ = *msg;
      RCLCPP_INFO(get_logger(), "Recibida PoseArray con %zu poses", msg->poses.size());
    }

    // Función para convertir radianes a grados y aplicar ajustes específicos
    std::vector<double> convertRadiansToDegrees(const std::vector<double>& radians) {
        std::vector<double> degrees;
        degrees.reserve(radians.size() + 1);

        for (size_t i = 0; i < radians.size(); ++i) {
            double angle_deg = 0.0;

            switch (i) {
                case 0:
                    angle_deg = (radians[i]  * (180.0 / M_PI )) + 150;
                    break;
                case 1:
                    angle_deg = (-1*radians[i]  * (180.0 / M_PI )) + 240;
                    degrees.push_back(angle_deg); // Añadir el valor ajustado
                    angle_deg = (radians[i]  * (180.0 / M_PI )) + 60;
                    break;
                case 2:
                    angle_deg = (radians[i]  * (180.0 / M_PI )) + 150;
                    break;
                case 3:
                    angle_deg = (radians[i]  * (180.0 / M_PI )) + 150;
                    break;
                default:
                    angle_deg = (radians[i] * 180.0) / M_PI;
                    break;
            }
            degrees.push_back(angle_deg);
        }

        return degrees;
    }

    // ---- lógica principal ----
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<rapling_msgs::action::ArduinobotTask>> goal_handle)
    {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = this->now();

      RCLCPP_INFO(get_logger(), "Executing goal");
      auto result = std::make_shared<rapling_msgs::action::ArduinobotTask::Result>();
      auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");

      int task = goal_handle->get_goal()->task_number;

      if (task == 1)
      {
        std::vector<double> arm_joint_goal;

        // 1) Verificar dos poses
        if (latest_finger_pose_array_.poses.size() != 2)
        {
          RCLCPP_ERROR(get_logger(), "Se requieren 2 poses en 'finger_poses'");
          result->success = false;
          return goal_handle->abort(result);
        }

        // 2) Extraer puntos
        auto p1 = latest_finger_pose_array_.poses[0].position;
        auto p2 = latest_finger_pose_array_.poses[1].position;

        // 3) Visualizar trayectoria
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "world";
        m.header.stamp = now();
        m.ns = "finger_trajectory";
        m.id = 0;
        m.type = m.LINE_STRIP;
        m.action = m.ADD;
        m.scale.x = 0.01;  m.color.g = 1.0;  m.color.a = 1.0;
        m.points = { p1, p2 };
        marker_pub_->publish(m);

        // 4) Definir JointConstraint para "codo arriba"
        moveit_msgs::msg::JointConstraint elbow_up;
        elbow_up.joint_name      = "joint_link_3";  // AJUSTA al nombre real
        elbow_up.position        = 1.0;               // radianes, ángulo ejemplo
        elbow_up.tolerance_below = 0.05;
        elbow_up.tolerance_above = 0.05;
        elbow_up.weight          = 1.0;
        moveit_msgs::msg::Constraints path_c;
        path_c.joint_constraints.push_back(elbow_up);

        // --- Mover a posición 1 ---
        arm_move_group.setPositionTarget(p1.x, p1.y, p1.z);
        {
          moveit::planning_interface::MoveGroupInterface::Plan plan;

          if (arm_move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
          {
            RCLCPP_ERROR(get_logger(), "Plan failed para posición 1");
            arm_move_group.clearPathConstraints();
            result->success = false;
            return goal_handle->abort(result);
          }

          // Obtener los ángulos finales de la trayectoria
          const auto& trajectory_points = plan.trajectory_.joint_trajectory.points;
          if (!trajectory_points.empty())
          {
            const auto& final_angles = trajectory_points.back().positions;

            // Convertir radianes a grados y aplicar ajustes
            std::vector<double> angles_deg = convertRadiansToDegrees(final_angles);

            RCLCPP_INFO(get_logger(), "######################ANGULOS PARA PICK (radianes): Base=%.2f, Shoulder=%.2f, Elbow=%.2f, Gripper=%.2f ###########################",
                        final_angles[0], final_angles[1], final_angles[2], final_angles[3]);
            RCLCPP_INFO(get_logger(), "######################ANGULOS PARA PICK (Angular): Base=%.2f, Shoulder=%.2f, Elbow=%.2f, Gripper=%.2f ###########################",
                        angles_deg[0], angles_deg[1], angles_deg[2], angles_deg[3], angles_deg[4]);

            // Publicar el JointState con los ángulos en grados
            msg.header.stamp = now();
            msg.name = {"base", "shoulder", "shoulderB", "elbow", "gripper"};
            msg.position = angles_deg;
            final_angles_pub_->publish(msg);

            arm_joint_goal = {final_angles[0], final_angles[1], final_angles[2], final_angles[3]};
            arm_move_group.setJointValueTarget(arm_joint_goal);
            arm_move_group.move();
          }
        }

        // --- SEGUNDA ETAPA Moverse a la posición HOME ---
        {
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          std::vector<double> joint_goal = {-0.0, 0.0, -0.0, -0.0};
          if (!arm_move_group.setJointValueTarget(joint_goal)) {
              RCLCPP_ERROR(get_logger(), "Error al asignar joint goal");
          }

          if (arm_move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
              const auto& trajectory_points = plan.trajectory_.joint_trajectory.points;
              if (!trajectory_points.empty()) {
                  const auto& final_angles = trajectory_points.back().positions;

                  // Convertir radianes a grados y aplicar ajustes
                  std::vector<double> angles_deg = convertRadiansToDegrees(final_angles);

                  RCLCPP_INFO(get_logger(), "######################ANGULOS PARA HOME (radianes): Base=%.2f, Shoulder=%.2f, Elbow=%.2f, Gripper=%.2f ###########################",
                              final_angles[0], final_angles[1], final_angles[2], final_angles[3]);
                  RCLCPP_INFO(get_logger(), "######################ANGULOS PARA HOME (angulares): Base=%.2f, Shoulder=%.2f, Elbow=%.2f, Gripper=%.2f ###########################",
                              angles_deg[0], angles_deg[1], angles_deg[2], angles_deg[3], angles_deg[4]);

                  // Publicar el JointState con los ángulos en grados
                  msg.header.stamp = now();
                  msg.name = {"base", "shoulder", "shoulderB", "elbow", "gripper"};
                  msg.position = angles_deg;
                  final_angles_pub_->publish(msg);
              }

              arm_move_group.execute(plan);
          } 
          else {
              RCLCPP_ERROR(get_logger(), "Plan falló para joint_goal");
          }
        }

        // --- Mover a posición 2 ---
        arm_move_group.setPositionTarget(p2.x, p2.y, p2.z);
        {
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          if (arm_move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
          {
            RCLCPP_ERROR(get_logger(), "Plan failed para posición 2");
            arm_move_group.clearPathConstraints();
            result->success = false;
            return goal_handle->abort(result);
          }

          const auto& trajectory_points = plan.trajectory_.joint_trajectory.points;

          if (!trajectory_points.empty()) 
          {
            const auto& final_angles = trajectory_points.back().positions;

            // Convertir radianes a grados y aplicar ajustes
            std::vector<double> angles_deg = convertRadiansToDegrees(final_angles);

            RCLCPP_INFO(get_logger(), "######################ANGULOS PARA PLACE (radianes): Base=%.2f, Shoulder=%.2f, Elbow=%.2f, Gripper=%.2f ###########################",
                        final_angles[0], final_angles[1], final_angles[2], final_angles[3]);
            RCLCPP_INFO(get_logger(), "######################ANGULOS PARA PLACE (Angular): Base=%.2f, Shoulder=%.2f, Elbow=%.2f, Gripper=%.2f ###########################",
                        angles_deg[0], angles_deg[1], angles_deg[2], angles_deg[3], angles_deg[4]);

            // Publicar el JointState con los ángulos en grados
            msg.header.stamp = now();
            msg.name = {"base", "shoulder", "shoulderB", "elbow", "gripper"};
            msg.position = angles_deg;
            final_angles_pub_->publish(msg);

            arm_joint_goal = {final_angles[0], final_angles[1], final_angles[2], final_angles[3]};
            arm_move_group.setJointValueTarget(arm_joint_goal);   
            arm_move_group.move();
          }
        }

        // --- ÚLTIMA ETAPA Moverse a la posición HOME ---
        {
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          std::vector<double> joint_goal = {-0.0, 0.0, -0.0, -0.0};
          if (!arm_move_group.setJointValueTarget(joint_goal)) {
              RCLCPP_ERROR(get_logger(), "Error al asignar joint goal");
          }

          if (arm_move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
              const auto& trajectory_points = plan.trajectory_.joint_trajectory.points;
              if (!trajectory_points.empty()) {
                  const auto& final_angles = trajectory_points.back().positions;

                  // Convertir radianes a grados y aplicar ajustes para robot real.
                  std::vector<double> angles_deg = convertRadiansToDegrees(final_angles);

                  RCLCPP_INFO(get_logger(), "######################ANGULOS PARA home  (radianes): Base=%.2f, Shoulder=%.2f, Elbow=%.2f, Gripper=%.2f ###########################",
                              final_angles[0], final_angles[1], final_angles[2], final_angles[3]);
                  RCLCPP_INFO(get_logger(), "######################ANGULOS PARA HOME (angulares): Base=%.2f, Shoulder=%.2f, Elbow=%.2f, Gripper=%.2f ###########################",
                              angles_deg[0], angles_deg[1], angles_deg[2], angles_deg[3], angles_deg[4]);

                  // Publicar el JointState con los ángulos en grados
                  msg.header.stamp = now();
                  msg.name = {"base", "shoulder", "shoulderB", "elbow", "gripper"};
                  msg.position = angles_deg;
                  final_angles_pub_->publish(msg);
              }

              arm_move_group.execute(plan);
          } 
          else {
              RCLCPP_ERROR(get_logger(), "Plan falló para joint_goal");
          }


        }
        
        // Limpiar restricción
        arm_move_group.clearPathConstraints();
      }

      else
      {
        RCLCPP_ERROR(get_logger(), "Task number %d no implementado", task);
        result->success = false;
        goal_handle->abort(result);
        return;
      }

      // Terminar goal
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal succeeded");
    }
  };

} // namespace rapling_remote

RCLCPP_COMPONENTS_REGISTER_NODE(rapling_remote::TaskServer)