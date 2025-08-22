#pragma once
#include <vector>
#include <string>
#include <memory>   // <-- add
#include <chrono>   // <-- add
#include <deque>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "arm_sim_pkg/motor_controller.hpp"  // provides MotorController & MotorFeedback

namespace arm_sim_pkg {

class ArmHW : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmHW)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Configuration from URDF
  std::string device_;
  int baudrate_{0};
  std::vector<int> joint_ids_;
  std::vector<double> home_offset_;

  // Joint state management
  size_t njoints_{0};
  std::vector<double> pos_state_;   
  std::vector<double> vel_state_;   
  std::vector<double> pos_cmd_;      
  std::vector<double> vel_cmd_; 
  std::vector<double> acc_cmd_; 


  std::vector<double> last_sent_;   // Last sent positions for debouncing
  std::vector<bool> joint_enabled_; // Track enabled state of each joint

  // Motor controller
  std::unique_ptr<MotorController> ctrl_;
    
  // Motion parameters
  double default_acc_{10.0};
  double default_vel_{5.0};
  double angle_tolerance_{0.01};    // rad
  bool allow_offline_{false};       // Allow startup without device
  double send_rate_hz_{50.0};       // Command rate limiting
  std::chrono::steady_clock::time_point last_tx_time_;
    
  // Joint state tracking
  std::vector<MotorFeedback> joint_feedback_;  // Last feedback for each joint
  bool joints_initialized_{false};             // Track if joints are initialized


  // 轨迹分段执行：每关节一个 waypoint 队列
  std::vector<std::deque<double>> waypoint_queue_;
  std::vector<bool>   in_flight_;       // 是否有一段在执行
  std::vector<double> latched_target_;  // 当前段的目标角

  // 抽样/去抖参数
  double waypoint_min_delta_{1e-3};     // 相邻 waypoint 最小间隔（rad）
  double goal_tolerance_rad_{1e-3}; 
  size_t waypoint_max_queue_{200};      // 单关节队列最大长度，过长则丢弃旧点
  std::vector<bool> first_feedback_ok_;

  bool ready_{false};          // 是否允许发“第一枪”
  bool first_shot_done_{false}; // 是否已经发过一次
  std::ofstream* log_file_{nullptr};


};

} // namespace arm_sim_pkg
