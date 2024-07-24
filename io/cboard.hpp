#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class CBoard
{
public:
  double bullet_speed;

  CBoard(const std::string & interface)
  : queue_(5000),
    bullet_speed(0),
    // 注意: callback的运行会早于Cboard构造函数的完成
    can_(interface, std::bind(&CBoard::callback, this, std::placeholders::_1))
  {
    // tools::logger()->info("[Cboard] Waiting for q...");
    // queue_.pop(data_ahead_);
    // queue_.pop(data_behind_);
    // tools::logger()->info("[Cboard] Opened.");
  }

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp)
  {
    if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

    while (true) {
      queue_.pop(data_behind_);
      if (data_behind_.timestamp > timestamp) break;
      data_ahead_ = data_behind_;
    }

    Eigen::Quaterniond q_a = data_ahead_.q;
    Eigen::Quaterniond q_b = data_behind_.q;
    auto t_a = data_ahead_.timestamp;
    auto t_b = data_behind_.timestamp;
    auto t_c = timestamp;
    std::chrono::duration<double> t_ab = t_b - t_a;
    std::chrono::duration<double> t_ac = t_c - t_a;

    // 四元数插值
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b);

    return q_c;
  }

  void send(Command command) const
  {
    can_frame frame;
    frame.can_id = 0x100;
    frame.can_dlc = 8;
    frame.data[0] = (int16_t)command.weights_x >> 8;
    frame.data[1] = (int16_t)command.weights_x;
    frame.data[2] = (int16_t)command.weights_y >> 8;
    frame.data[3] = (int16_t)command.weights_y;
    frame.data[4] = (int16_t)command.wood_x >> 8;
    frame.data[5] = (int16_t)command.wood_x;
    frame.data[6] = (int16_t)command.wood_y >> 8;
    frame.data[7] = (int16_t)command.wood_y;

    try {
      can_.write(&frame);
    } catch (const std::exception & e) {
      tools::logger()->warn("{}", e.what());
    }
  }

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData> queue_;  // 必须在can_之前初始化，否则存在死锁的可能
  SocketCAN can_;
  IMUData data_ahead_;
  IMUData data_behind_;

  void callback(const can_frame & frame)
  {
    auto timestamp = std::chrono::steady_clock::now();
    if (frame.can_id == 0x01) {
      auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
      auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;
      auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;
      auto w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;

      if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
        tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
        return;
      }

      queue_.push({{w, x, y, z}, timestamp});
    }

    else if (frame.can_id == 0x101) {
      bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
    }
  }
};

}  // namespace io

#endif  // IO__CBOARD_HPP