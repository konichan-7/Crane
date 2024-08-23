#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Dense>
#include <chrono>

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class CBoard
{
public:
  CBoard(const std::string & interface, bool left);

  bool start = false;

  Eigen::Vector3d odom_at(std::chrono::steady_clock::time_point t);

  void send(Command command) const;

  void rotate(double dx) const;

private:
  struct OdomData
  {
    Eigen::Vector3d xyz;
    std::chrono::steady_clock::time_point t;
  };

  const uint16_t rx_id_;
  const uint16_t tx_id_;

  tools::ThreadSafeQueue<OdomData> queue_;  // 必须在can_之前初始化，否则存在死锁的可能
  SocketCAN can_;
  OdomData data_ahead_;
  OdomData data_behind_;

  void callback(const can_frame & frame);
};

}  // namespace io

#endif  // IO__CBOARD_HPP