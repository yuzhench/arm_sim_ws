#include "arm_sim_pkg/motor_controller.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>
#include <iostream>   // optional for debugging
#include <string>

namespace arm_sim_pkg
{

// ------ low-level helpers ------

speed_t MotorController::baud_to_termios_(int baudrate)
{
  switch (baudrate) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:     return B115200;
  }
}

bool MotorController::configure_port_(int fd, int baudrate)
{
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) return false;

  cfsetospeed(&tty, baud_to_termios_(baudrate));
  cfsetispeed(&tty, baud_to_termios_(baudrate));

  // raw 8N1
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN]  = 0;   // non-blocking read
  tty.c_cc[VTIME] = 1;   // 0.1s read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  return (tcsetattr(fd, TCSANOW, &tty) == 0);
}

bool MotorController::write_all_(const uint8_t *data, size_t nbytes)
{
  size_t sent = 0;
  while (sent < nbytes) {
    ssize_t w = ::write(fd_, data + sent, nbytes - sent);
    if (w < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    sent += static_cast<size_t>(w);
  }
  return true;
}

ssize_t MotorController::read_some_(uint8_t *buf, size_t maxlen, int timeout_ms)
{
  if (fd_ < 0) return -1;

  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(fd_, &rfds);

  timeval tv{};
  tv.tv_sec  = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int rc = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
  if (rc < 0) {
    if (errno == EINTR) return 0;
    return -1;
  }
  if (rc == 0) return 0;  // timeout

  return ::read(fd_, buf, maxlen);
}

// ------ lifecycle ------

MotorController::~MotorController() { close(); }

bool MotorController::open(const std::string &device, int baudrate)
{
  close();
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) return false;
  if (!configure_port_(fd_, baudrate)) {
    close();
    return false;
  }
  // small settle
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return true;
}

void MotorController::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

// ------ commands ------

bool MotorController::send_combined_command(
    uint8_t motor_id, float acc, float vel, float angle_rad, uint32_t timeout_ms)
{
  if (fd_ < 0) return false;
  // <BB f f f I> (LE)
  std::vector<uint8_t> pkt;
  pkt.reserve(1 + 1 + 3*sizeof(float) + sizeof(uint32_t));
  pkt.push_back(CMD_COMBINED_COMMAND);
  pkt.push_back(motor_id);
  appendLE<float>(pkt, acc);
  appendLE<float>(pkt, vel);
  appendLE<float>(pkt, angle_rad);
  appendLE<uint32_t>(pkt, timeout_ms);
  return write_all_(pkt.data(), pkt.size());
}

bool MotorController::send_simultaneous(
    const std::vector<std::tuple<uint8_t,float,float,float>> &configs,
    uint32_t timeout_ms)
{
  if (fd_ < 0) return false;
  if (configs.empty() || configs.size() > static_cast<size_t>(MAX_MOTORS)) return false;

  std::vector<uint8_t> pkt;
  pkt.reserve(2 + configs.size() * (1 + 3*sizeof(float) + sizeof(uint32_t)));
  pkt.push_back(CMD_SIMULTANEOUS_COMMAND);
  pkt.push_back(static_cast<uint8_t>(configs.size()));
  for (const auto &c : configs) {
    uint8_t id; float a, v, ang;
    std::tie(id, a, v, ang) = c;
    pkt.push_back(id);
    appendLE<float>(pkt, a);
    appendLE<float>(pkt, v);
    appendLE<float>(pkt, ang);
    appendLE<uint32_t>(pkt, timeout_ms);
  }
  return write_all_(pkt.data(), pkt.size());
}

bool MotorController::send_combined_command_time_based(
    uint8_t motor_id, float target_angle_rad, uint32_t time_ms)
{
  if (fd_ < 0) return false;
  std::vector<uint8_t> pkt;
  pkt.reserve(1 + 1 + sizeof(float) + sizeof(uint32_t));
  pkt.push_back(CMD_COMBINED_COMMAND_TIME_BASED);
  pkt.push_back(motor_id);
  appendLE<float>(pkt, target_angle_rad);
  appendLE<uint32_t>(pkt, time_ms);
  return write_all_(pkt.data(), pkt.size());
}

bool MotorController::send_combined_command_degrees(
    uint8_t motor_id, float acc, float vel, float angle_degrees, uint32_t timeout_ms)
{
  const float rad = angle_degrees * static_cast<float>(M_PI / 180.0);
  return send_combined_command(motor_id, acc, vel, rad, timeout_ms);
}

bool MotorController::set_global_movement_time(uint32_t time_ms)
{
  if (fd_ < 0) return false;
  std::vector<uint8_t> pkt;
  pkt.reserve(1 + sizeof(uint32_t));
  pkt.push_back(CMD_SET_GLOBAL_MOVEMENT_TIME);
  appendLE<uint32_t>(pkt, time_ms);
  return write_all_(pkt.data(), pkt.size());
}

bool MotorController::set_angle_tolerance(float tol)
{
  if (fd_ < 0) return false;
  std::vector<uint8_t> pkt;
  pkt.reserve(1 + sizeof(float));
  pkt.push_back(CMD_SET_ANGLE_TOLERANCE);
  appendLE<float>(pkt, tol);
  return write_all_(pkt.data(), pkt.size());
}

bool MotorController::set_angle_tolerance_degrees(float tol_deg)
{
  return set_angle_tolerance(tol_deg * static_cast<float>(M_PI / 180.0));
}

bool MotorController::set_torque_limit(uint8_t motor_id, float torque_nm)
{
  if (fd_ < 0) return false;
  std::vector<uint8_t> pkt;
  pkt.reserve(1 + 1 + sizeof(float));
  pkt.push_back(CMD_SET_TORQUE_LIMIT);
  pkt.push_back(motor_id);
  appendLE<float>(pkt, torque_nm);
  return write_all_(pkt.data(), pkt.size());
}

bool MotorController::enable_motor(uint8_t motor_id)
{
  if (fd_ < 0) return false;
  uint8_t pkt[2] = { CMD_ENABLE_MOTOR, motor_id };
  return write_all_(pkt, sizeof(pkt));
}

bool MotorController::disable_motor(uint8_t motor_id)
{
  if (fd_ < 0) return false;
  uint8_t pkt[2] = { CMD_DISABLE_MOTOR, motor_id };
  return write_all_(pkt, sizeof(pkt));
}

bool MotorController::motor_stop(uint8_t motor_id)
{
  if (fd_ < 0) return false;
  uint8_t pkt[2] = { CMD_MOTOR_STOP, motor_id };
  return write_all_(pkt, sizeof(pkt));
}

bool MotorController::set_mech_zero(uint8_t motor_id)
{
  if (fd_ < 0) return false;
  uint8_t pkt[2] = { CMD_SET_MECH_ZERO, motor_id };
  return write_all_(pkt, sizeof(pkt));
}

// ------ feedback ------

bool MotorController::request_feedback(uint8_t motor_id, MotorFeedback &feedback, double timeout_sec)
{
  if (fd_ < 0) return false;

  // request
  {
    uint8_t pkt[2] = { CMD_REQUEST_FEEDBACK, motor_id };
    if (!write_all_(pkt, sizeof(pkt))) return false;
  }

  // read ASCII lines until "FEEDBACK:" arrives or timeout
  const auto t_end = std::chrono::steady_clock::now() +
                     std::chrono::duration<double>(timeout_sec);
  std::string resp;
  resp.reserve(256);

  while (std::chrono::steady_clock::now() < t_end) {
    uint8_t tmp[256];
    ssize_t n = read_some_(tmp, sizeof(tmp), 100);
    if (n < 0) return false;
    if (n == 0) continue;

    resp.append(reinterpret_cast<const char*>(tmp), static_cast<size_t>(n));

    // process by line
    for (;;) {
      auto pos = resp.find('\n');
      if (pos == std::string::npos) break;
      std::string line = resp.substr(0, pos);
      resp.erase(0, pos + 1);

      // expected: FEEDBACK:0xXX:mode,fault,motor_can_id,host_can_id,angle,velocity,torque,temp,current,current_valid,active,completed,target_angle,tolerance
      if (line.rfind("FEEDBACK:", 0) == 0) {
        auto colon2 = line.find(':', 9);
        if (colon2 == std::string::npos) continue;
        std::string payload = line.substr(colon2 + 1);

        // split by comma
        std::vector<std::string> values;
        values.reserve(14);
        std::stringstream ss(payload);
        std::string tok;
        while (std::getline(ss, tok, ',')) values.push_back(tok);

        if (values.size() == 14) {
          try {
            feedback.mode_status       = std::stoi(values[0]);
            feedback.fault_info        = std::stoi(values[1]);
            feedback.motor_can_id      = std::stoi(values[2]);
            feedback.host_can_id       = std::stoi(values[3]);
            feedback.angle             = std::stof(values[4]);
            feedback.velocity          = std::stof(values[5]);
            feedback.torque            = std::stof(values[6]);
            feedback.temperature       = std::stof(values[7]);
            feedback.current           = std::stof(values[8]);
            feedback.current_valid     = (std::stoi(values[9])  != 0);
            feedback.command_active    = (std::stoi(values[10]) != 0);
            feedback.command_completed = (std::stoi(values[11]) != 0);
            feedback.target_angle      = std::stof(values[12]);
            feedback.angle_tolerance   = std::stof(values[13]);
            feedback.last_update_time  =
              std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()
              ).count();
            return true;
          } catch (...) {
            return false;
          }
        }
      }
    }
  }
  return false; // timeout
}


 




bool MotorController::wait_for_completion(uint8_t motor_id, double timeout_sec)
{
  const auto t_end = std::chrono::steady_clock::now() +
                     std::chrono::duration<double>(timeout_sec);
  while (std::chrono::steady_clock::now() < t_end) {
    MotorFeedback fb{};
    if (request_feedback(motor_id, fb, 1.0)) {
      if (fb.command_completed) return true;
      if (!fb.command_active && !fb.command_completed) return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return false; // timeout
}

// ------ time & logging ------

bool MotorController::set_system_time_now()
{
  if (fd_ < 0) return false;

  // Build big-endian time packet (to match Python)
  auto now      = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tmv;
  localtime_r(&t, &tmv);

  uint32_t ts = static_cast<uint32_t>(t);

  std::vector<uint8_t> pkt;
  pkt.reserve(1 + 2 + 4 + 2 + 5); // cmd + header + ts + year + month..sec
  pkt.push_back(CMD_SET_TIME);
  appendBE<uint16_t>(pkt, TIME_COMMAND_HEADER);
  appendBE<uint32_t>(pkt, ts);
  appendBE<uint16_t>(pkt, static_cast<uint16_t>(tmv.tm_year + 1900));
  pkt.push_back(static_cast<uint8_t>(tmv.tm_mon + 1));
  pkt.push_back(static_cast<uint8_t>(tmv.tm_mday));
  pkt.push_back(static_cast<uint8_t>(tmv.tm_hour));
  pkt.push_back(static_cast<uint8_t>(tmv.tm_min));
  pkt.push_back(static_cast<uint8_t>(tmv.tm_sec));

  return write_all_(pkt.data(), pkt.size());
}

bool MotorController::set_system_time(const struct tm &tmv)
{
  if (fd_ < 0) return false;
  std::time_t t = std::mktime(const_cast<std::tm*>(&tmv));
  uint32_t ts = static_cast<uint32_t>(t);

  std::vector<uint8_t> pkt;
  pkt.reserve(1 + 2 + 4 + 2 + 5);
  pkt.push_back(CMD_SET_TIME);
  appendBE<uint16_t>(pkt, TIME_COMMAND_HEADER);
  appendBE<uint32_t>(pkt, ts);
  appendBE<uint16_t>(pkt, static_cast<uint16_t>(tmv.tm_year + 1900));
  pkt.push_back(static_cast<uint8_t>(tmv.tm_mon + 1));
  pkt.push_back(static_cast<uint8_t>(tmv.tm_mday));
  pkt.push_back(static_cast<uint8_t>(tmv.tm_hour));
  pkt.push_back(static_cast<uint8_t>(tmv.tm_min));
  pkt.push_back(static_cast<uint8_t>(tmv.tm_sec));

  return write_all_(pkt.data(), pkt.size());
}

bool MotorController::send_external_log(const std::string &message)
{
  if (fd_ < 0) return false;
  if (message.empty() || message.size() > 200) return false;

  std::vector<uint8_t> pkt;
  pkt.reserve(1 + 1 + message.size());
  pkt.push_back(CMD_EXTERNAL_LOG);
  pkt.push_back(static_cast<uint8_t>(message.size()));
  pkt.insert(pkt.end(), message.begin(), message.end());
  return write_all_(pkt.data(), pkt.size());
}

} // namespace arm_sim_pkg
