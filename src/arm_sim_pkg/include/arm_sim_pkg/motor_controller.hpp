#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <sys/types.h>   // ssize_t
#include <termios.h>     // speed_t
#include <algorithm>     // std::reverse
#include <type_traits>   // std::is_trivially_copyable
#include <stdexcept>
#include <cmath>
#include <cstring>       // std::memcpy
#include <sstream>
#include <chrono>
#include <thread>
#include <memory>
#include <tuple>
#include <ctime>         // struct tm

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace arm_sim_pkg {

// Command constants (match Python)
enum : uint8_t {
  CMD_COMBINED_COMMAND              = 0x01,
  CMD_SIMULTANEOUS_COMMAND          = 0x02,
  CMD_REQUEST_FEEDBACK              = 0x03,
  CMD_ENABLE_MOTOR                  = 0x04,
  CMD_DISABLE_MOTOR                 = 0x05,
  CMD_SET_MECH_ZERO                 = 0x06,
  CMD_SET_ANGLE_TOLERANCE           = 0x07,
  CMD_MOTOR_STOP                    = 0x08,
  CMD_SET_TIME                      = 0x09,
  CMD_COMBINED_COMMAND_TIME_BASED   = 0x0A,
  CMD_SET_GLOBAL_MOVEMENT_TIME      = 0x0B,
  CMD_EXTERNAL_LOG                  = 0x0C,
  CMD_SET_TORQUE_LIMIT              = 0x0D,
};

// Motor IDs
enum : uint8_t {
  ROBSTRIDE_02_1_ID = 0x01,
  ROBSTRIDE_03_ID   = 0x02,
  ROBSTRIDE_02_2_ID = 0x03,
  ROBSTRIDE_02_3_ID = 0x04,
  ROBSTRIDE_00_ID   = 0x05
};

// Validation limits
constexpr double MAX_VELOCITY = 25.0;
constexpr double MIN_VELOCITY = -25.0;
constexpr double MAX_ACCELERATION = 40.0;
constexpr double MIN_ACCELERATION = 0.1;
constexpr double MAX_ANGLE = 6.4;
constexpr double MIN_ANGLE = -6.4;
constexpr double MAX_ANGLE_TOLERANCE = 5.0;
constexpr double MIN_ANGLE_TOLERANCE = 0.001;
constexpr double DEFAULT_ANGLE_TOLERANCE = 0.01;
constexpr int    MAX_MOTORS = 5;
constexpr double MAX_TORQUE_LIMIT = 60.0;
constexpr double MIN_TORQUE_LIMIT = 0.1;

// Python big-endian time header
constexpr uint16_t TIME_COMMAND_HEADER = 0xABCD;

struct MotorFeedback {
  int   mode_status{};
  int   fault_info{};
  int   motor_can_id{};
  int   host_can_id{};
  float angle{};
  float velocity{};
  float torque{};
  float temperature{};
  float current{};
  bool  current_valid{};
  bool  command_active{};
  bool  command_completed{};
  float target_angle{};
  float angle_tolerance{};
  double last_update_time{};

  bool  is_ready() const { return mode_status == 2 && fault_info == 0; }
  float get_angle_degrees() const { return angle * 180.0f / static_cast<float>(M_PI); }
  float get_angle_error() const { return std::abs(target_angle - angle); }   // optional: treat unset as 0
  bool  is_within_tolerance() const { return get_angle_error() < angle_tolerance; }
};

class MotorController {
public:
  MotorController() = default;
  ~MotorController();

  // Connection
  bool open(const std::string &device, int baudrate);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // Commands
  bool send_combined_command(uint8_t motor_id, float acc, float vel, float angle_rad, uint32_t timeout_ms);
  bool send_simultaneous(const std::vector<std::tuple<uint8_t,float,float,float>> &configs, uint32_t timeout_ms);
  bool send_combined_command_time_based(uint8_t motor_id, float target_angle_rad, uint32_t time_ms);
  bool send_combined_command_degrees(uint8_t motor_id, float acc, float vel, float angle_degrees, uint32_t timeout_ms);
  bool set_global_movement_time(uint32_t time_ms);
  bool set_angle_tolerance(float tol);
  bool set_angle_tolerance_degrees(float tol_degrees);
  bool set_torque_limit(uint8_t motor_id, float torque_nm);

  // Basics
  bool enable_motor(uint8_t motor_id);
  bool disable_motor(uint8_t motor_id);
  bool motor_stop(uint8_t motor_id);
  bool set_mech_zero(uint8_t motor_id);

  // Feedback
  bool request_feedback(uint8_t motor_id, MotorFeedback &feedback, double timeout_sec = 3.0);
  bool wait_for_completion(uint8_t motor_id, double timeout_sec = 30.0);

  // Time & logging
  bool set_system_time_now();
  bool set_system_time(const struct tm &time_info);
  bool send_external_log(const std::string &message);

private:
  int fd_{-1};

  // Low-level serial helpers
  static speed_t baud_to_termios_(int baudrate);
  bool configure_port_(int fd, int baudrate);
  bool write_all_(const uint8_t *data, size_t nbytes);
  ssize_t read_some_(uint8_t *buf, size_t maxlen, int timeout_ms);

  // Packing helpers
  template<typename T>
  static void appendLE(std::vector<uint8_t> &buf, T value) {
    static_assert(std::is_trivially_copyable<T>::value, "T must be POD");
    uint8_t tmp[sizeof(T)];
    std::memcpy(tmp, &value, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    std::reverse(tmp, tmp + sizeof(T));
#endif
    buf.insert(buf.end(), tmp, tmp + sizeof(T));
  }

  template<typename T>
  static void appendBE(std::vector<uint8_t> &buf, T value) {
    static_assert(std::is_trivially_copyable<T>::value, "T must be POD");
    uint8_t tmp[sizeof(T)];
    std::memcpy(tmp, &value, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    std::reverse(tmp, tmp + sizeof(T));
#endif
    buf.insert(buf.end(), tmp, tmp + sizeof(T));
  }
};

} // namespace arm_sim_pkg
