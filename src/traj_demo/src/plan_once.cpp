#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

static void timeScale(moveit_msgs::msg::RobotTrajectory &traj, double scale)
{
  auto &jt = traj.joint_trajectory;
  if (scale <= 0.0) return;

  for (auto &pt : jt.points) {
    // 1) 时间戳整体缩放
    auto t_ns = pt.time_from_start.sec * 1000000000LL + pt.time_from_start.nanosec;
    t_ns = static_cast<int64_t>(t_ns * scale);
    pt.time_from_start.sec     = static_cast<int32_t>(t_ns / 1000000000LL);
    pt.time_from_start.nanosec = static_cast<uint32_t>(t_ns % 1000000000LL);

    // 2) 速度/加速度按维度缩放（保持物理一致性；有些控制器会忽略它们，但写上更严谨）
    //   缩短一半时间 => 速度×(1/scale)=×2，加速度×(1/scale^2)=×4
    if (!pt.velocities.empty()) {
      for (auto &v : pt.velocities) v /= scale;
    }
    if (!pt.accelerations.empty()) {
      for (auto &a : pt.accelerations) a /= (scale * scale);
    }
  }
}



int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
      "plan_once",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // 按你的配置改这两个名字：
  const std::string GROUP = "arm_group";       // ← Planning Group 名（见 SRDF）
  const std::string BASE  = "base_link"; // ← 机器人基坐标系（见 URDF）
  
  
  RCLCPP_INFO(node->get_logger(), "Initializing MoveGroupInterface...");
  moveit::planning_interface::MoveGroupInterface mgi(node, GROUP);
  RCLCPP_INFO(node->get_logger(), "MoveGroupInterface initialized");

  mgi.setPlanningTime(5.0);
  mgi.setNumPlanningAttempts(5);
  mgi.setStartStateToCurrentState();  // 起点 = 当前状态（A）

  // 目标位姿（B）：示例值，按你臂的工作空间改
  // geometry_msgs::msg::PoseStamped target;
  // target.header.frame_id = BASE;
  // target.pose.position.x = 0.2;
  // target.pose.position.y = 0.3;
  // target.pose.position.z = 0.4;
  // target.pose.orientation.w = 1.0;   // 简单单位四元数
  // mgi.setPoseTarget(target);

  // mgi.setNamedTarget("home_pose");  // 目标位姿名称（见 SRDF）

  RCLCPP_INFO(node->get_logger(),"try to plan -------------------------");

  
 



  std::map<std::string, double> joints{
    {"joint_base", 0.5},
    {"joint_base_big_arm", 0},
    {"joint_big_arm_small_arm", 0.3},
    {"joint_small_arm_wrist", 0.3},
  };

  mgi.setJointValueTarget(joints);




  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto ret = mgi.plan(plan);

  timeScale(plan.trajectory_, 0.1);

  if (ret == moveit::core::MoveItErrorCode::SUCCESS) {
    const auto& jt = plan.trajectory_.joint_trajectory;
    RCLCPP_INFO(node->get_logger(), "Planned trajectory points: %zu", jt.points.size());
    
    // 打印每个轨迹点的 time / pos / vel / acc（默认已做时间参数化）
    for (size_t i = 0; i < jt.points.size(); ++i) {
      const auto& p = jt.points[i];
      double t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9;
      std::ostringstream oss;
      oss << "t=" << t << "s  pos=[";
      for (size_t j = 0; j < p.positions.size(); ++j) {
        oss << p.positions[j] << (j + 1 < p.positions.size() ? ", " : "");
      }
      oss << "]";
      if (!p.velocities.empty()) {
        oss << "  vel=[";
        for (size_t j = 0; j < p.velocities.size(); ++j) {
          oss << p.velocities[j] << (j + 1 < p.velocities.size() ? ", " : "");
        }
        oss << "]";
      }
      if (!p.accelerations.empty()) {
        oss << "  acc=[";
        for (size_t j = 0; j < p.accelerations.size(); ++j) {
          oss << p.accelerations[j] << (j + 1 < p.accelerations.size() ? ", " : "");
        }
        oss << "]";
      }
      RCLCPP_INFO(node->get_logger(), "%s", oss.str().c_str());
    }


    RCLCPP_INFO(node->get_logger(), "Planning successful, executing trajectory...");
    mgi.execute(plan); 

  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
  }

  rclcpp::shutdown();
  return 0;
}
