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
    rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID &,std::shared_ptr<const rapling_msgs::action::ArduinobotTask::Goal> goal)
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
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      rapling_msgs::action::ArduinobotTask>> goal_handle)
{
  // 1) Llevar el brazo a la pose llamada "home1"
  moveit::planning_interface::MoveGroupInterface arm_move_group(
      shared_from_this(), "arm");  // usa este nodo

  arm_move_group.setNamedTarget("home1");
  moveit::planning_interface::MoveGroupInterface::Plan home_plan;

  bool ok_home =
      (arm_move_group.plan(home_plan) ==
       moveit::core::MoveItErrorCode::SUCCESS);  // evita warning por "deprecated"

  if (ok_home) {
    arm_move_group.execute(home_plan);
    RCLCPP_INFO(get_logger(),
                "Pose 'home1' alcanzada. Lanzando hilo execute().");

    // 2) Si todo salió bien, lanza la ejecución normal
    std::thread(&TaskServer::execute, this, goal_handle).detach();
  } else {
    RCLCPP_ERROR(get_logger(),
                 "No se pudo planificar a 'home1'. Abortando goal.");

    // 3) Abortar el goal porque falló el pre‑movimiento
    auto result = std::make_shared<
        rapling_msgs::action::ArduinobotTask::Result>();
    result->success = false;
    goal_handle->abort(result);
  }
}

void directionCallback(const std_msgs::msg::String::SharedPtr msg)
{
  static const std::set<std::string> valid = {"Izquierda","Derecha","Adelante","Atras","Arriba","Abajo"};
  if (valid.count(msg->data))
  {
    trajectory_directions_.push_back(msg->data);
    RCLCPP_INFO(get_logger(), "Comando recibido: '%s'. Enviando goal task 2...", msg->data.c_str());

    auto goal_msg = rapling_msgs::action::ArduinobotTask::Goal();
    goal_msg.task_number = 2;
    action_client_->async_send_goal(goal_msg);
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
                    angle_deg = (radians[i]  * (180.0 / M_PI ))+23 ;
                    break;
                case 1:
                    //angle_deg = (-1*radians[i]  * (180.0 / M_PI )) + 240;
                    //degrees.push_back(angle_deg); // Añadir el valor ajustado
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

bool moveRelative(const std::string& direction, moveit::planning_interface::MoveGroupInterface& arm_move_group)
{
  geometry_msgs::msg::PoseStamped current_pose = arm_move_group.getCurrentPose();
  geometry_msgs::msg::Point p = current_pose.pose.position;

  const double step = 0.02; // 2 cm

  if (direction == "Derecha")        p.x += step;
  else if (direction == "Izquierda") p.x -= step;
  else if (direction == "Adelante")  p.y += step;
  else if (direction == "Atras")     p.y -= step;
  else if (direction == "Arriba")    p.z += step;
  else if (direction == "Abajo")     p.z -= step;
  else {
    RCLCPP_WARN(get_logger(), "Dirección no válida: %s", direction.c_str());
    return false;
  }

  arm_move_group.setPositionTarget(p.x, p.y, p.z);
  return arm_move_group.move() == moveit::core::MoveItErrorCode::SUCCESS;
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

     // ------------------------------------------------------------------
// TASK 1  →  Pick & Place con constraints y seed actual
// ------------------------------------------------------------------
if (task == 1)   // PICK AND PLACE
{
 
  
  sensor_msgs::msg::JointState msg;
  msg.name = {"base", "shoulder", "shoulderB", "elbow", "gripper"};

  //---------------------------------------------------------------
  // 1) Verificar que existan exactamente dos poses
  //---------------------------------------------------------------
  if (latest_finger_pose_array_.poses.size() != 2) {
    RCLCPP_ERROR(get_logger(), "Se requieren 2 poses en 'finger_poses'");
    result->success = false;
    return goal_handle->abort(result);
  }

  auto p1 = latest_finger_pose_array_.poses[0].position;
  auto p2 = latest_finger_pose_array_.poses[1].position;

  //---------------------------------------------------------------
  // 2) Visualizar línea entre p1 y p2
  //---------------------------------------------------------------
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "world";
  m.header.stamp    = now();
  m.ns   = "finger_trajectory";
  m.id   = 0;
  m.type = m.LINE_STRIP;
  m.action    = m.ADD;
  m.scale.x   = 0.01;
  m.color.g   = 1.0;
  m.color.a   = 1.0;
  m.points    = {p1, p2};
  marker_pub_->publish(m);

  //---------------------------------------------------------------
  // 3) Constraints: base ±90° y codo‑arriba
  //---------------------------------------------------------------
  const auto current_joints = arm_move_group.getCurrentJointValues();

  moveit_msgs::msg::JointConstraint base_limit;
  base_limit.joint_name      = "joint_link_1";          // ← ajusta al nombre real
  base_limit.position        = current_joints[0];       // posición actual
  base_limit.tolerance_above = M_PI / 2;                // +90°
  base_limit.tolerance_below = M_PI / 2;                // -90°
  base_limit.weight          = 1.0;

  moveit_msgs::msg::JointConstraint elbow_up;
  elbow_up.joint_name      = "joint_link_3";            // ← ajusta al nombre real
  elbow_up.position        = -0.47;                       // codo‑arriba aproximado
  elbow_up.tolerance_above = -2.0;                  // +30° (0.52 rad)
  elbow_up.tolerance_below = 0.30;
  elbow_up.weight          = 1.0;

  moveit_msgs::msg::Constraints path_constraints;
  path_constraints.joint_constraints = {base_limit, elbow_up};

  // Activar constraints para todo el ciclo Pick‑Place
  arm_move_group.setPathConstraints(path_constraints);

  //---------------------------------------------------------------
  // 4) MOVE → P1  (Pick)
  //---------------------------------------------------------------
  arm_move_group.setStartStateToCurrentState();
  arm_move_group.setPositionTarget(p1.x, p1.y, p1.z);

  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Plan falló para posición 1");
      arm_move_group.clearPathConstraints();
      result->success = false;
      return goal_handle->abort(result);
    }

    const auto& final_angles = plan.trajectory_.joint_trajectory.points.back().positions;
    std::vector<double> angles_deg = convertRadiansToDegrees(final_angles);

    msg.header.stamp = now();
    msg.position     = angles_deg;
    final_angles_pub_->publish(msg);

    arm_move_group.execute(plan);
  }

  //---------------------------------------------------------------
  // 5) MOVE → HOME intermedio
  //---------------------------------------------------------------
  arm_move_group.setStartStateToCurrentState();
  {
    std::vector<double> home = {1.57, 0.523, -0.47, -0.95};   // ajusta si es necesario
    arm_move_group.setJointValueTarget(home);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      const auto& final_angles = plan.trajectory_.joint_trajectory.points.back().positions;
      msg.header.stamp = now();
      msg.position     = convertRadiansToDegrees(final_angles);
      final_angles_pub_->publish(msg);

      arm_move_group.execute(plan);
    } else {
      RCLCPP_ERROR(get_logger(), "Plan falló para HOME intermedio");
    }
  }

  //---------------------------------------------------------------
  // 6) MOVE → P2  (Place)
  //---------------------------------------------------------------
  arm_move_group.setStartStateToCurrentState();
  arm_move_group.setPositionTarget(p2.x, p2.y, p2.z);

  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Plan falló para posición 2");
      arm_move_group.clearPathConstraints();
      result->success = false;
      return goal_handle->abort(result);
    }

    const auto& final_angles = plan.trajectory_.joint_trajectory.points.back().positions;
    std::vector<double> angles_deg = convertRadiansToDegrees(final_angles);

    msg.header.stamp = now();
    msg.position     = angles_deg;
    final_angles_pub_->publish(msg);

    arm_move_group.execute(plan);
  }
  arm_move_group.clearPathConstraints();
  //---------------------------------------------------------------
  // 7) MOVE → HOME final
  //---------------------------------------------------------------
  arm_move_group.setStartStateToCurrentState();
  {
    std::vector<double> home = {1.57, 0.523, -0.47, -0.95};   // mismo HOME
    arm_move_group.setJointValueTarget(home);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      const auto& final_angles = plan.trajectory_.joint_trajectory.points.back().positions;
      msg.header.stamp = now();
      msg.position     = convertRadiansToDegrees(final_angles);
      final_angles_pub_->publish(msg);

      arm_move_group.execute(plan);
    } else {
      RCLCPP_ERROR(get_logger(), "Plan falló para HOME final");
    }
  }

  //---------------------------------------------------------------
  // 8) Limpiar constraints y finalizar
  //---------------------------------------------------------------
  arm_move_group.clearPathConstraints();
}

      else if (task == 2) // TELEOPERACIÒN 
      {
        if (trajectory_directions_.empty())
        {
          RCLCPP_ERROR(get_logger(), "No hay dirección recibida. Abortando.");
          result->success = false;
          return goal_handle->abort(result);
        }

        std::string direction = trajectory_directions_.back();
        trajectory_directions_.clear(); // Limpiar para futuros usos

        RCLCPP_INFO(get_logger(), "Ejecutando movimiento hacia: %s", direction.c_str());

        if (!moveRelative(direction, arm_move_group))
        {
          result->success = false;
          return goal_handle->abort(result);
        }

        // Publicar ángulos finales
        auto final_state = arm_move_group.getCurrentJointValues();
        std::vector<double> final_deg = convertRadiansToDegrees(final_state);

        sensor_msgs::msg::JointState js;
        js.header.stamp = now();
        js.name = {"base", "shoulder", "shoulderB", "elbow", "gripper"};
        js.position = final_deg;
        final_angles_pub_->publish(js);

      }
      else if (task == 3) // UNIR PUNTOS CON TRAZADO CARTESIANO
      {
        // 1) Verificar dos poses
        if (latest_finger_pose_array_.poses.size() != 2) {
          RCLCPP_ERROR(get_logger(), "Se requieren 2 poses en 'finger_poses'");
          result->success = false;
          return goal_handle->abort(result);
        }

        // 2) Extraer puntos
        auto p1 = latest_finger_pose_array_.poses[0].position;
        auto p2 = latest_finger_pose_array_.poses[1].position;

        // 3) Visualizar trayectoria plana
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

        // 4) Llevar primero a posición HOME (opcional)
        {
          moveit::planning_interface::MoveGroupInterface::Plan plan_home;
          std::vector<double> home_joints = {1.57, 0.523, -0.47, -0.95};
          arm_move_group.setJointValueTarget(home_joints);
          if (arm_move_group.plan(plan_home) == moveit::core::MoveItErrorCode::SUCCESS) {
            arm_move_group.execute(plan_home);
          } else {
            RCLCPP_ERROR(get_logger(), "Plan falló para HOME inicial");
          }
        }

        // 5) Generar trayectoria cartesiana de p1 a p2
        std::vector<geometry_msgs::msg::Pose> waypoints;
        // partimos de la pose actual pero cambiamos sólo la posición:
        auto start_pose = arm_move_group.getCurrentPose().pose;
        start_pose.position = p1;
        waypoints.push_back(start_pose);

        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position = p2;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;      // resolución 1 cm
        const double jump_threshold = 0.0; // deshabilita salto de articulaciones
        double fraction = arm_move_group.computeCartesianPath(
          waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.99) {
          RCLCPP_ERROR(get_logger(), "Sólo se pudo computar %.2f%% de la trayectoria cartesiana", fraction * 100.0);
          result->success = false;
          return goal_handle->abort(result);
        }

        // 6) Ejecutar la trayectoria cartesiana completa
        arm_move_group.execute(trajectory);

        // 7) Publicar ángulos finales de PLACE
        {
          const auto& final_joints = arm_move_group.getCurrentJointValues();
          auto angles_deg = convertRadiansToDegrees(final_joints);

          RCLCPP_INFO(get_logger(),
            "Ángulos PLACE (rad): Base=%.2f, Sh=%.2f, El=%.2f, Gr=%.2f",
            final_joints[0], final_joints[1], final_joints[2], final_joints[3]);
          RCLCPP_INFO(get_logger(),
            "Ángulos PLACE (deg): Base=%.2f, Sh=%.2f, El=%.2f, Gr=%.2f",
            angles_deg[0], angles_deg[1], angles_deg[2], angles_deg[3]);

            sensor_msgs::msg::JointState js;
          js.header.stamp = now();
          js.name     = {"base", "shoulder", "shoulderB", "elbow", "gripper"};
          js.position = angles_deg;
          final_angles_pub_->publish(js);
        }

        // 8) Volver a HOME final (opcional)
        {
          moveit::planning_interface::MoveGroupInterface::Plan plan_end;
          std::vector<double> end_joints = {0.0, 0.0, 0.0, 0.0};
          arm_move_group.setJointValueTarget(end_joints);
          if (arm_move_group.plan(plan_end) == moveit::core::MoveItErrorCode::SUCCESS) {
            arm_move_group.execute(plan_end);
          } else {
            RCLCPP_ERROR(get_logger(), "Plan falló para HOME final");
          }
        }

        // 9) Limpiar cualquier restricción pendiente
        arm_move_group.clearPathConstraints();

        RCLCPP_INFO(get_logger(), "Task 3 completado con trayectoria cartesiana");
      }
else if (task == 4) // CÍRCULO VISUAL
{
  RCLCPP_INFO(get_logger(), "Dibujando un círculo en RViz...");

  visualization_msgs::msg::Marker circle;
  circle.header.frame_id = "world";
  circle.header.stamp = now();
  circle.ns = "circle_path";
  circle.id = 1;
  circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
  circle.action = visualization_msgs::msg::Marker::ADD;
  circle.scale.x = 0.005;
  circle.color.r = 0.0;
  circle.color.g = 0.0;
  circle.color.b = 1.0;
  circle.color.a = 1.0;

  const double radius = 0.05; // 5 cm
  const int num_points = 100;
  const double cx = -0.19; // centro X
  const double cy = -0.0; // centro Y
  const double cz = 0.05; // centro Z

  for (int i = 0; i <= num_points; ++i) {
    double theta = 2 * M_PI * i / num_points;
    geometry_msgs::msg::Point pt;
    pt.x = cx + radius * cos(theta);
    pt.y = cy + radius * sin(theta);
    pt.z = cz;
    circle.points.push_back(pt);
  }

  marker_pub_->publish(circle);
  std::vector<geometry_msgs::msg::Pose> waypoints;

geometry_msgs::msg::Pose start_pose = arm_move_group.getCurrentPose().pose;
for (int i = 0; i <= num_points; ++i) {
  double theta = 2 * M_PI * i / num_points;
  geometry_msgs::msg::Pose p = start_pose;
  p.position.x = cx + radius * cos(theta);
  p.position.y = cy + radius * sin(theta);
  p.position.z = cz;
  waypoints.push_back(p);
}

moveit_msgs::msg::RobotTrajectory trajectory;
const double eef_step = 0.005;
const double jump_threshold = 0.0;
double fraction = arm_move_group.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

if (fraction < 0.9) {
  RCLCPP_ERROR(get_logger(), "Trayectoria incompleta: %.2f%%", fraction * 100);
  result->success = false;
  return goal_handle->abort(result);
}

arm_move_group.execute(trajectory);
  result->success = true;
  goal_handle->succeed(result);
  return;

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