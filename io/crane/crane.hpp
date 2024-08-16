#ifndef IO__CRANE_HPP
#define IO__CRANE_HPP

#include <string>

#include "io/cboard/cboard.hpp"
#include "io/command.hpp"

namespace io
{
class Crane
{
public:
  Crane(CBoard & left, CBoard & right, const std::string & config_path);

  Eigen::Vector3d odom_at(std::chrono::steady_clock::time_point t, bool left);

  void move_xy(double x_in_odom, double y_in_odom, bool left);

private:
  CBoard & left_;
  CBoard & right_;

  double y_left_in_odom_;
  double y_right_in_odom_;

  io::Command left_last_cmd_{0.0, 0.0, 0.0, false};
  io::Command right_last_cmd_{0.0, 0.0, 0.0, false};

  void cmd(Command command, bool left);
};

}  // namespace io

#endif  // IO__CRANE_HPP