#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
      "plan_once",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // 按你的配置改这两个名字：
  const std::string GROUP = "arm_group";       // ← Planning Group 名（见 SRDF）
  const std::string BASE  = "base_link"; // ← 机器人基坐标系（见 URDF）

  moveit::planning_interface::MoveGroupInterface mgi(node, GROUP);
  mgi.setPlanningTime(5.0);
  mgi.setNumPlanningAttempts(5);
  mgi.setStartStateToCurrentState();  // 起点 = 当前状态（A）

  // 目标位姿（B）：示例值，按你臂的工作空间改
  // geometry_msgs::msg::PoseStamped target;
  // target.header.frame_id = BASE;
  // target.pose.position.x = -0.005;
  // target.pose.position.y = 0;
  // target.pose.position.z = 0.43;
  // target.pose.orientation.w = 1.0;   // 简单单位四元数
  // mgi.setPoseTarget(target);

  mgi.setNamedTarget("farest_pose");  // 目标位姿名称（见 SRDF）

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto ret = mgi.plan(plan);

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
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
  }

  rclcpp::shutdown();
  return 0;
}
