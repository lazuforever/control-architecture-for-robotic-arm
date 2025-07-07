#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "arduinobot_msgs/action/arduinobot_task.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <memory>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::placeholders;

namespace arduinobot_remote
{
  class TaskServer : public rclcpp::Node
  {
  public:
    explicit TaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("task_server", options)
    {
      RCLCPP_INFO(get_logger(), "Starting the Server");
      action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArduinobotTask>(
          this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
          std::bind(&TaskServer::cancelCallback, this, _1),
          std::bind(&TaskServer::acceptedCallback, this, _1));

          // Suscriptor para recibir la palabra "Izquierda"
          direction_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "hand_direction", 10, std::bind(&TaskServer::directionCallback, this, _1));

        // Cliente de acción para enviar Goal 0
          action_client_ = rclcpp_action::create_client<arduinobot_msgs::action::ArduinobotTask>(this, "task_server");
          marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("line_topic", 10);
 
        }
  private:

    rclcpp_action::Server<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_server_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr direction_subscriber_;
    rclcpp_action::Client<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;


    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const arduinobot_msgs::action::ArduinobotTask::Goal> goal)
    {
      RCLCPP_INFO(get_logger(), "Received GOALL request with id %d", goal->task_number);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
    {
      (void)goal_handle;
      RCLCPP_INFO(get_logger(), "Received request to cancel goal");
      auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
      auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");
      arm_move_group.stop();
      gripper_move_group.stop();
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void acceptedCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
    {
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&TaskServer::execute, this, _1), goal_handle}.detach();
    }

    void directionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string commands[] = {"Izquierda", "Derecha", "Arriba", "Abajo","Derecha_Arriba"};
        const int task_numbers[] = {3, 4, 5, 6, 7};
    
        int goal_value = -1;
    
        for (size_t i = 0; i < 5; ++i) {
            if (msg->data == commands[i]) {
                goal_value = task_numbers[i];
                break;
            }
        }
    
        if (goal_value != -1) {
            RCLCPP_INFO(get_logger(), "Recibido '%s', activando tarea %d.", msg->data.c_str(), goal_value);
    
            auto goal = arduinobot_msgs::action::ArduinobotTask::Goal();
            goal.task_number = goal_value;
    
            if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
                RCLCPP_ERROR(get_logger(), "Servidor de acción no disponible.");
                return;
            }
    
            auto send_goal_options = rclcpp_action::Client<arduinobot_msgs::action::ArduinobotTask>::SendGoalOptions();
            send_goal_options.result_callback = [](auto result) {
                RCLCPP_INFO(rclcpp::get_logger("TaskServer"), "Tarea completada con éxito.");
            };
    
            action_client_->async_send_goal(goal, send_goal_options);
        }
    }    

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
    {
      RCLCPP_INFO(get_logger(), "Executing goal");
      auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>();

      // MoveIt 2 Interface
      auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
      auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

      std::vector<double> arm_joint_goal;
      std::vector<double> gripper_joint_goal;
      std::vector<geometry_msgs::msg::Pose> waypoints;
      bool arm_within_bounds;

      auto joint_model_group = arm_move_group.getRobotModel()->getJointModelGroup("arm");

      const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
      for (const auto &joint_name : joint_names)
      {
        auto bounds = arm_move_group.getRobotModel()->getJointModel(joint_name)->getVariableBounds(joint_name);
        RCLCPP_INFO(get_logger(), "Límites de %s: [%f, %f]", joint_name.c_str(), bounds.min_position_, bounds.max_position_);
      }
      int task_number = goal_handle->get_goal()->task_number;



              // Publicar la trayectoria como marcador de línea
              visualization_msgs::msg::Marker marker;
              marker.header.frame_id = "world";  // Ajusta según el frame de referencia de tu robot
              marker.header.stamp = this->get_clock()->now();
              marker.ns = "trajectory";
              marker.id = 0;
              marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
              marker.action = visualization_msgs::msg::Marker::ADD;
              marker.scale.x = 0.02;  // Grosor de la línea
              marker.color.r = 1.0;
              marker.color.g = 0.0;
              marker.color.b = 0.0;
              marker.color.a = 1.0;   // Opacidad
      
      if (goal_handle->get_goal()->task_number == 0)

      {

        // Se asume que el goal 0 indica mover a la izquierda
        gripper_joint_goal = {0.0, 0.0};


        // Dentro de la función execute(), antes de obtener la pose actual:
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
        arm_move_group.setEndEffectorLink("gripper_tip");  // Ahora se usará el nuevo efector final
 

        // Obtener la pose actual del efector final
        geometry_msgs::msg::Pose current_pose = arm_move_group.getCurrentPose().pose;

        RCLCPP_INFO(
            get_logger(),
            "Posición inicial: x=%.2f, y=%.2f, z=%.2f",
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z);

            double desplazamiento_x = -0.25; // Ajusta este valor según la magnitud deseada
            double desplazamiento_z = 0.25;  // Movimiento extra en Z (opcional)
        
                // Crear un waypoint desplazado más a la izquierda
            geometry_msgs::msg::Pose pose1 = current_pose;
            pose1.position.x += desplazamiento_x;

            // Crear un segundo waypoint con un pequeño desplazamiento en Z
            geometry_msgs::msg::Pose pose2 = pose1;
            pose2.position.z += desplazamiento_z;

            for (const auto &joint_name : joint_names)
            {
              auto bounds = arm_move_group.getRobotModel()->getJointModel(joint_name)->getVariableBounds(joint_name);
              RCLCPP_INFO(get_logger(), "Límites de %s: [%f, %f]", joint_name.c_str(), bounds.min_position_, bounds.max_position_);
            }

            // Mostrar posición cartesiana
            RCLCPP_INFO(
                get_logger(),
                "Posición cartesiana: x=%.2f, y=%.2f, z=%.2f",
                current_pose.position.x, current_pose.position.y, current_pose.position.z);



        
              for (const auto &wp : waypoints) {
                  RCLCPP_INFO(get_logger(), "WaypointPOSE2: x=%.2f, y=%.2f, z=%.2f | Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                              wp.position.x, wp.position.y, wp.position.z,
                              wp.orientation.x, wp.orientation.y, wp.orientation.z, wp.orientation.w);
              }
    
            // Generar los waypoints
            waypoints.push_back(current_pose);
            waypoints.push_back(pose1);
            waypoints.push_back(pose2);


           // Convertir los waypoints en puntos y agregarlos al marcador
            for (const auto &wp : waypoints)
            {
              geometry_msgs::msg::Point p;
              p.x = wp.position.x;
              p.y = wp.position.y;
              p.z = wp.position.z;
              marker.points.push_back(p);
            }
            marker_pub_->publish(marker);
            RCLCPP_INFO(get_logger(), "Se publicó la trayectoria con %zu puntos", marker.points.size());

            // Calcular la trayectoria cartesiana
            const double eef_step = 0.01;      // Resolución de 1 cm entre puntos
            const double jump_threshold = 0.0; // Sin umbral de salto
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = arm_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

          
          if (fraction < 1.0)
          {
            RCLCPP_WARN(get_logger(), "No se pudo generar el 100%% de la trayectoria, fraction: %.2f", fraction);
            return;
          }
          else
          {
            RCLCPP_INFO(get_logger(), "Ejecutando movimiento Trayectoria creada" );
            arm_move_group.execute(trajectory);
          }
          // Actualizar la posición actual después del movimiento
          //current_pose = arm_move_group.getCurrentPose().pose;
          //arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
        
      }    
    // Verificar si el task_number está en el rango [3, 6]
      else if (task_number >= 3 && task_number <= 6)
    {
        RCLCPP_INFO(get_logger(), "Ejecutando tarea %d.", task_number);

        // Obtener la pose actual del efector final
        geometry_msgs::msg::Pose current_pose = arm_move_group.getCurrentPose().pose;

        RCLCPP_INFO(get_logger(), "Posición inicial: x=%.2f, y=%.2f, z=%.2f",
                    current_pose.position.x, current_pose.position.y, current_pose.position.z);

        // Definir los desplazamientos según el task_number
        double desplazamiento_x = (task_number == 3) ? -0.15 : (task_number == 4) ? 0.15 : 0.0;
        double desplazamiento_y = (task_number == 5) ? 0.15 : (task_number == 6) ? -0.15 : 0.0;
       
        // Crear nuevo objetivo con el desplazamiento aplicado
        geometry_msgs::msg::Pose target_pose = current_pose;
        target_pose.position.x += desplazamiento_x;
        target_pose.position.y += desplazamiento_y;
        gripper_joint_goal = {0.0, 0.0};
        arm_move_group.setPoseTarget(target_pose);

        // Generar un waypoint con la nueva pose
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        // Calcular la trayectoria cartesiana
        const double eef_step = 0.01;      // Resolución de 1 cm entre puntos
        const double jump_threshold = 0.0; // Sin umbral de salto
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        


        
        if (fraction < 1.0)
        {
          RCLCPP_WARN(get_logger(), "No se pudo generar el 100%% de la trayectoria, fraction: %.2f", fraction);
          return;
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Ejecutando movimiento " );
          arm_move_group.execute(trajectory);
        }
      
    }
      else if (task_number == 7)
      {
        RCLCPP_INFO(get_logger(), "Ejecutando tarea 7: Derecha_Arriba.");
      
        // Ajusta la apertura de la garra si procede
        gripper_joint_goal = {0.0, 0.0};
      
        // Obtener la pose actual del efector final
        geometry_msgs::msg::Pose current_pose = arm_move_group.getCurrentPose().pose;
      
        RCLCPP_INFO(
            get_logger(),
            "Posición inicial: x=%.2f, y=%.2f, z=%.2f",
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z);
      
        // Desplazamientos para "Derecha" y luego "Arriba"
        double desplazamiento_x = 0.25;  // Derecha
        double desplazamiento_z = 0.25;  // Arriba
      
        // Crear un primer waypoint (desplazamiento en X)
        geometry_msgs::msg::Pose pose1 = current_pose;
        pose1.position.x += desplazamiento_x;
      
        // Crear un segundo waypoint (desplazamiento en Z)
        geometry_msgs::msg::Pose pose2 = pose1;
        pose2.position.z += desplazamiento_z;
      
        // Generar los waypoints
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(current_pose);
        waypoints.push_back(pose1);
        waypoints.push_back(pose2);
      
        // Publicar la trayectoria como marcador de línea
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";  // Ajusta según tu TF
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.02;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
      
        for (const auto &wp : waypoints)
        {
          geometry_msgs::msg::Point p;
          p.x = wp.position.x;
          p.y = wp.position.y;
          p.z = wp.position.z;
          marker.points.push_back(p);
        }
        marker_pub_->publish(marker);
        RCLCPP_INFO(get_logger(), "Se publicó la trayectoria con %zu puntos", marker.points.size());
      
        // Calcular la trayectoria cartesiana
        const double eef_step = 0.01;      // Resolución de 1 cm entre puntos
        const double jump_threshold = 0.0; // Sin umbral de salto
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      
        if (fraction < 1.0)
        {
          RCLCPP_WARN(get_logger(), "No se pudo generar el 100%% de la trayectoria, fraction: %.2f", fraction);
          return;
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Ejecutando movimiento. Trayectoria creada." );
          arm_move_group.execute(trajectory);
        }
      }
  
      else if (goal_handle->get_goal()->task_number == 1)
      {
        arm_joint_goal = {-1.14, -0.6, -0.07};
        gripper_joint_goal = {-1, 1};
        arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
      }

      else if (goal_handle->get_goal()->task_number == 2)
      {
        arm_joint_goal = {-1.57, 0.0, -0.9};
        gripper_joint_goal = {-0.35, 0.35};
        arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Invalid Task Number");
        return;
      }

      bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);

      if (!arm_within_bounds | !gripper_within_bounds && goal_handle->get_goal()->task_number > 0)
      {
        RCLCPP_WARN(get_logger(),
                    "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
        return;
      }

      moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
      moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
      
      bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (arm_plan_success && gripper_plan_success  && goal_handle->get_goal()->task_number > 0 )
      {
        RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arme and the gripper");
        arm_move_group.move();
        gripper_move_group.move();
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "One or more planners failed!");
        return;
      }

      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal succeeded");
    }
  };
} // namespace arduinobot_remote

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::TaskServer)
