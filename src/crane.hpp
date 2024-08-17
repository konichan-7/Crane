#ifndef SRC__CRANE_HPP
#define SRC__CRANE_HPP

#include "io/cboard/cboard.hpp"
#include "io/command.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_crane/matcher.hpp"
#include "tasks/auto_crane/solver.hpp"
#include "tasks/auto_crane/yolov8.hpp"

class Crane
{
public:
  Crane(const std::string & config_path);

  void get(int id1, int id2, bool left);
  void put(int id, bool left);

private:
  io::CBoard left_cboard_;
  io::CBoard right_cboard_;
  io::USBCamera left_cam_;
  io::USBCamera right_cam_;

  auto_crane::YOLOV8 yolo_;
  auto_crane::Solver solver_;
  auto_crane::Matcher matcher_;

  Eigen::Vector2d t_map_to_left_odom_;
  Eigen::Vector2d t_map_to_right_odom_;

  io::Command left_last_cmd_{0.0, 0.0, 0.0, false, false};
  io::Command right_last_cmd_{0.0, 0.0, 0.0, false, false};

  double last_x(bool left) const;
  double last_y(bool left) const;
  double last_z(bool left) const;

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & t, bool left);
  Eigen::Vector3d odom_at(std::chrono::steady_clock::time_point t, bool left);

  void cmd(io::Command command, bool left);
  void cmd(Eigen::Vector3d target_in_odom, bool left);

  void go(Eigen::Vector3d target_in_odom, bool left);
  void align(int id1, int id2, bool left);
  void grip(bool grip, bool left);
};

#endif  // SRC__CRANE_HPP