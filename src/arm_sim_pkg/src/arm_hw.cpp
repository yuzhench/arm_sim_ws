// file: src/arm_hw.cpp
#include "arm_sim_pkg/arm_hw.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>



namespace arm_sim_pkg
{

// ---------- on_init ----------
hardware_interface::CallbackReturn
ArmHW::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  auto logger = rclcpp::get_logger("ArmHW");

 
  // The info_ member contains the hardware configuration 
  // information that loaded from your URDF/xacro file 
  // <ros2_control> tag. It includes:

  // Joint information
  // Hardware parameters
  // Interface types
  // Other configuration data
   
  try {
    device_   = info_.hardware_parameters.at("device");
    baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "Missing/invalid param 'device' or 'baudrate': %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // joint_ids (optional)
  if (auto it = info_.hardware_parameters.find("joint_ids");
      it != info_.hardware_parameters.end())
  {
    joint_ids_.clear();
    std::stringstream ss(it->second);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      if (tok.empty()) continue;
      tok.erase(0, tok.find_first_not_of(" \t"));
      tok.erase(tok.find_last_not_of(" \t") + 1);
      int id = (tok.rfind("0x", 0) == 0 || tok.rfind("0X", 0) == 0)
                ? std::stoi(tok, nullptr, 16)
                : std::stoi(tok);
      joint_ids_.push_back(id & 0xFF);
    }
  }

  // optional params
  if (auto it = info_.hardware_parameters.find("default_acc"); it != info_.hardware_parameters.end())
    default_acc_ = std::stod(it->second);
  if (auto it = info_.hardware_parameters.find("default_vel"); it != info_.hardware_parameters.end())
    default_vel_ = std::stod(it->second);
  if (auto it = info_.hardware_parameters.find("allow_offline"); it != info_.hardware_parameters.end())
    allow_offline_ = (it->second == "true" || it->second == "1");
  if (auto it = info_.hardware_parameters.find("send_rate_hz"); it != info_.hardware_parameters.end())
    send_rate_hz_ = std::max(1.0, std::stod(it->second));
  if (auto it = info_.hardware_parameters.find("angle_tolerance"); it != info_.hardware_parameters.end())
    angle_tolerance_ = std::stod(it->second);

  // init arrays
  njoints_ = info_.joints.size();
  pos_state_.assign(njoints_, 0.0);
  pos_cmd_.assign(njoints_, 0.0);
  last_sent_.assign(njoints_, 0.0);
  joint_enabled_.assign(njoints_, false);
  joint_feedback_.resize(njoints_);

  first_feedback_ok_.assign(njoints_, false);

  first_shot_done_ = false;  // 上电/激活后允许再次发一次
  ready_ = true;

  RCLCPP_INFO(logger,
    "ArmHW initialized with %zu joints (dev=%s baud=%d) acc=%.3f vel=%.3f tol=%.6f offline=%s rate=%.1fHz",
    njoints_, device_.c_str(), baudrate_, default_acc_, default_vel_, angle_tolerance_,
    (allow_offline_ ? "true" : "false"), send_rate_hz_);

  

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------- export_state_interfaces ----------
std::vector<hardware_interface::StateInterface>
ArmHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.reserve(njoints_);
  for (size_t i = 0; i < njoints_; ++i) {
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_state_[i]);
  }
  return si;
}

// ---------- export_command_interfaces ----------
std::vector<hardware_interface::CommandInterface>
ArmHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.reserve(njoints_);
  for (size_t i = 0; i < njoints_; ++i) {
    ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_cmd_[i]);
  }
  return ci;
}

// ---------- on_configure ----------
hardware_interface::CallbackReturn
ArmHW::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = rclcpp::get_logger("ArmHW");

  if (joint_ids_.empty()) {
    joint_ids_.push_back(0x01); // default one ID
  }
  if (joint_ids_.size() != njoints_) {
    RCLCPP_WARN(logger, "njoints(%zu) != joint_ids(%zu) — will use min()", njoints_, joint_ids_.size());
  }

  ctrl_ = std::make_unique<MotorController>();
  if (!ctrl_->open(device_, baudrate_)) {
    RCLCPP_ERROR(logger, "open(%s, %d) failed: %s",
                 device_.c_str(), baudrate_, std::strerror(errno));
    if (!allow_offline_) {
      ctrl_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    } else {
      RCLCPP_WARN(logger, "allow_offline=true → continue in offline mode (no actual TX).");
    }
  }

  last_tx_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(logger, "on_configure done. ready=%s",
              (ctrl_ && ctrl_->is_open()) ? "yes" : "no (offline)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------- on_activate ----------
hardware_interface::CallbackReturn
ArmHW::on_activate(const rclcpp_lifecycle::State &)
{
  auto logger = rclcpp::get_logger("ArmHW");
  pos_cmd_ = pos_state_;
  last_sent_ = pos_cmd_;
  last_tx_time_ = std::chrono::steady_clock::now();

  if (ctrl_ && ctrl_->is_open()) {
    const size_t n = std::min(njoints_, joint_ids_.size());
    for (size_t i = 0; i < n; ++i) {
      uint8_t id = static_cast<uint8_t>(joint_ids_[i]);

      if (!ctrl_->enable_motor(id)) {
        RCLCPP_WARN(logger, "enable_motor failed for 0x%02X", id);
        continue;
      }
      if (!ctrl_->set_angle_tolerance(static_cast<float>(angle_tolerance_))) {
        RCLCPP_WARN(logger, "set_angle_tolerance failed for 0x%02X", id);
      }

      MotorFeedback fb;
      if (ctrl_->request_feedback(id, fb)) {
        joint_feedback_[i] = fb;
        pos_state_[i] = fb.angle;
        pos_cmd_[i]   = fb.angle;
        last_sent_[i] = fb.angle;
        joint_enabled_[i] = true;
        first_feedback_ok_[i] = true;
      } else {
        joint_enabled_[i] = false;
      }
    }
    // one-time system time sync
    ctrl_->set_system_time_now();
  }


  waypoint_queue_.assign(njoints_, {});   // 清空每个关节队列
  in_flight_.assign(njoints_, false);
  latched_target_.assign(njoints_, 0.0);


  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------- on_deactivate ----------
hardware_interface::CallbackReturn
ArmHW::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Optionally disable here.
  // if (ctrl_ && ctrl_->is_open()) {
  //   const size_t n = std::min(njoints_, joint_ids_.size());
  //   for (size_t i = 0; i < n; ++i) {
  //     ctrl_->disable_motor(static_cast<uint8_t>(joint_ids_[i]));
  //   }
  // }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------- read ----------
hardware_interface::return_type
ArmHW::read(const rclcpp::Time &, const rclcpp::Duration &)
{

  //can't open the real motor
  if (!(ctrl_ && ctrl_->is_open())) {
    // mirror commands when offline
    for (size_t i = 0; i < njoints_; ++i) pos_state_[i] = pos_cmd_[i];
    return hardware_interface::return_type::OK;
  }

  //can open the real motor
  const size_t n = njoints_;

  //check the real motor and joint number 

  if (n != joint_ids_.size()) {
    RCLCPP_WARN(rclcpp::get_logger("ArmHW"),
                "njoints(%zu) != joint_ids(%zu) — please check the xacro file", njoints_, joint_ids_.size());
  }

  //go through each joint
  for (size_t i = 0; i < n; ++i) {

    if (!joint_enabled_[i]){
      RCLCPP_WARN(rclcpp::get_logger("ArmHW"),
                  "joint %zu is not enabled", i);
      continue;
    }

    MotorFeedback fb;
    if (ctrl_->request_feedback(static_cast<uint8_t>(joint_ids_[i]), fb)) {
      //get the feedback for each jointS
      joint_feedback_[i] = fb;
      pos_state_[i] = fb.angle;
      RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
                  "joint %zu feedback: angle=%.2f", i, fb.angle);
      first_feedback_ok_[i] = true;   // ← 成功读到一次反馈，记为 OK
    }
  }
  return hardware_interface::return_type::OK;
}






// ---------- write ----------
hardware_interface::return_type
ArmHW::write(const rclcpp::Time &, const rclcpp::Duration &)
{

  auto dubuglg = rclcpp::get_logger("!!!!!!!_debug_!!!!!!");
  auto lg = rclcpp::get_logger("ArmHW");


  // RCLCPP_INFO(dubuglg, "write is called");

  // 1) 底层未打开：不做任何事
  if (!(ctrl_ && ctrl_->is_open())) {
    // RCLCPP_WARN(lg, "ctrl not open, skip");
    return hardware_interface::return_type::OK;
  }

  // 2) 控制器未“武装”（尚未允许发第一枪）：直接返回
  if (!ready_) {
    return hardware_interface::return_type::OK;
  }

  // 3) 已经发过一次了：什么都不做（只发一次的关键）
  // if (first_shot_done_) {
  //   RCLCPP_INFO(lg, "directly leave --------");
  //   return hardware_interface::return_type::OK;
  // }

  // 4) 组装并一次性下发当前 pos_cmd_（视你的底层接口而定）
  const size_t n = std::min(njoints_, joint_ids_.size());
  // bool all_ok = true;

  // pos_cmd_[0] = 3.14;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t id  = static_cast<uint8_t>(joint_ids_[i]);
    const float   acc = static_cast<float>(default_acc_);   // 你已有的默认加速度
    const float   vel = static_cast<float>(default_vel_);   // 你已有的默认速度
    const float   pos = static_cast<float>(pos_cmd_[i]);    // 只用当前的目标值


    const bool ok = ctrl_->send_combined_command(id, acc, vel, pos, /*timeout_ms*/ 10000u);
    RCLCPP_INFO(lg, "motor index: %d, position pos is: %.2f", id, pos);

    if (!ok){
      RCLCPP_WARN(lg, "failed to send the command");

    }
    // if (!ok) {
    //   all_ok = false;
    //   RCLCPP_WARN(lg, "TX FAIL id=0x%02X pos=%.6f", id, pos);
    // } else {
    //   RCLCPP_INFO(lg,  "TX -> id=0x%02X acc=%.3f vel=%.3f pos=%.6f", id, acc, vel, pos);
    // }
  }
  //print a empty line
  RCLCPP_INFO(lg, "\n");


  // 5) 全部成功才标记“发过一次”
  // if (all_ok) {
  //   RCLCPP_INFO(lg, "write(): all commands sent successfully");
  //   first_shot_done_ = true;
  // }
  return hardware_interface::return_type::OK;
}


// debug version

// hardware_interface::return_type
// ArmHW::write(const rclcpp::Time &, const rclcpp::Duration &)
// {
//   if (!(ctrl_ && ctrl_->is_open())) {
//     RCLCPP_WARN(rclcpp::get_logger("ArmHW"), "write(): ctrl not open, skip");
//     return hardware_interface::return_type::OK;
//   }

//   const size_t n = std::min(njoints_, joint_ids_.size());
//   size_t sent = 0;

//   for (size_t i = 0; i < n; ++i) {
//     const uint8_t id  = static_cast<uint8_t>(joint_ids_[i]);
//     const float   pos = static_cast<float>(pos_cmd_[i]);

//     // 直接每周期都发（为调试用），并打印
//     const bool ok = ctrl_->send_combined_command(
//         id,
//         static_cast<float>(default_acc_),
//         static_cast<float>(default_vel_),
//         pos,
//         10000u);

//     if (ok) {
//       ++sent;
//       RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
//                   "TX -> id=0x%02X acc=%.3f vel=%.3f pos=%.6f",
//                   id, default_acc_, default_vel_, pos);
//     } else {
//       RCLCPP_WARN(rclcpp::get_logger("ArmHW"),
//                   "TX FAIL id=0x%02X pos=%.6f (send_combined_command=false)",
//                   id, pos);
//     }
//   }

//   if (sent == 0) {
//     RCLCPP_WARN(rclcpp::get_logger("ArmHW"),
//                 "write(): nothing sent (n=%zu, joint_ids=%zu)", n, joint_ids_.size());
//   }
//   return hardware_interface::return_type::OK;
// }


} // namespace arm_sim_pkg

PLUGINLIB_EXPORT_CLASS(arm_sim_pkg::ArmHW, hardware_interface::SystemInterface)
