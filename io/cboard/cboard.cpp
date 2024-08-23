#include "cboard.hpp"

#include <functional>

#include "tools/logger.hpp"

namespace io
{
CBoard::CBoard(const std::string & interface, bool left)
: rx_id_(left ? 0x102 : 0x103),
  tx_id_(left ? 0x100 : 0x101),
  queue_(5000),
  // 注意: callback的运行会早于Cboard构造函数的完成
  can_(interface, std::bind(&CBoard::callback, this, std::placeholders::_1))
{
}

Eigen::Vector3d CBoard::odom_at(std::chrono::steady_clock::time_point t)
{
  if (data_behind_.t < t) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.t > t) break;
    data_ahead_ = data_behind_;
  }

  if (data_ahead_.t > t) tools::logger()->warn("[CBoard] ahead of time!");

  auto t_a = data_ahead_.t;
  auto t_b = data_behind_.t;
  auto t_c = t;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;
  Eigen::Vector3d xyz_a = data_ahead_.xyz;
  Eigen::Vector3d xyz_b = data_behind_.xyz;

  // 插值
  auto k = t_ac / t_ab;
  Eigen::Vector3d xyz_c = xyz_a + (xyz_b - xyz_a) * k;

  return xyz_c;
}

void CBoard::send(Command command) const
{
  command.y = -command.y;

  can_frame frame;
  frame.can_id = tx_id_;
  frame.can_dlc = 8;
  frame.data[0] = (int16_t)(command.x * 1e3) >> 8;
  frame.data[1] = (int16_t)(command.x * 1e3);
  frame.data[2] = (int16_t)(command.y * 1e3) >> 8;
  frame.data[3] = (int16_t)(command.y * 1e3);
  frame.data[4] = (int16_t)(command.z * 1e3) >> 8;
  frame.data[5] = (int16_t)(command.z * 1e3);
  frame.data[6] = (command.grip) ? 1 : 0;
  frame.data[7] = (command.slow) ? 1 : 0;

  try {
    can_.write(&frame);
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void CBoard::rotate(double dx) const
{
  can_frame frame;
  frame.can_id = 0x99;
  frame.can_dlc = 8;
  frame.data[0] = (int16_t)(dx * 1e3) >> 8;
  frame.data[1] = (int16_t)(dx * 1e3);

  try {
    can_.write(&frame);
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void CBoard::callback(const can_frame & frame)
{
  auto t = std::chrono::steady_clock::now();

  if (frame.can_id != rx_id_) return;

  auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e3;
  auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e3;
  auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e3;

  y = -y;
  this->start = (frame.data[6] == 0x01);

  queue_.push({{x, y, z}, t});
}

}  // namespace io
