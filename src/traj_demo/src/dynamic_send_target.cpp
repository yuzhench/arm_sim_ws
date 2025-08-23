#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <mutex>
#include <map>
#include <vector>
using namespace std::chrono_literals;

class PlanFromTopic : public rclcpp::Node {
public:
  explicit PlanFromTopic(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : rclcpp::Node("dynamic_send_target", opts)
  {
    // 延后到下一拍执行，避免构造期 shared_from_this() 崩溃
    init_timer_ = this->create_wall_timer(1ms, [this]{
      init_timer_->cancel();        // 只运行一次
      init_moveit_and_sub();
    });
  }

private:
  void init_moveit_and_sub() {
    group_name_ = "arm_group";
    RCLCPP_INFO(get_logger(), "Init MoveGroupInterface...");
    mgi_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
              this->shared_from_this(), group_name_);
    mgi_->setPlanningTime(5.0);
    mgi_->setNumPlanningAttempts(5);

    joint_names_ = mgi_->getJointNames();
    rclcpp::QoS qos(10); qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    sub_goal_ = create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
      "/plan_goal_joints", qos,
      std::bind(&PlanFromTopic::onGoal, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Listening on /plan_goal_joints");
  }

  void onGoal(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg) {
    if (!mgi_) { RCLCPP_WARN(get_logger(), "MoveIt not ready yet."); return; }
    std::scoped_lock lk(mtx_);
    if (executing_) { RCLCPP_WARN(get_logger(), "Busy. Ignoring."); return; }
    if (msg->positions.size() != joint_names_.size()) {
      RCLCPP_ERROR(get_logger(), "positions=%zu, expected=%zu",
                   msg->positions.size(), joint_names_.size());
      return;
    }
    std::map<std::string,double> target;
    for (size_t i=0;i<joint_names_.size();++i) target[joint_names_[i]] = msg->positions[i];

    auto state = mgi_->getCurrentState(2.0);
    if (state) mgi_->setStartState(*state); else mgi_->setStartStateToCurrentState();
    mgi_->setJointValueTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mgi_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Planning failed."); return;
    }
    executing_ = true;
    auto exec_ret = mgi_->execute(plan);
    executing_ = false;
    if (exec_ret == moveit::core::MoveItErrorCode::SUCCESS)
      RCLCPP_INFO(get_logger(), "Execution finished");
    else
      RCLCPP_ERROR(get_logger(), "Execution failed");
  }

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
  std::vector<std::string> joint_names_;
  std::string group_name_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr sub_goal_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::mutex mtx_;
  bool executing_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<PlanFromTopic>(opts));
  rclcpp::shutdown();
  return 0;
}
