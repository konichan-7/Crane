#ifndef SRC__CRANE_HPP
#define SRC__CRANE_HPP

#include "io/cboard/cboard.hpp"
#include "io/command.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_crane/matcher.hpp"
#include "tasks/auto_crane/solver.hpp"
#include "tasks/auto_crane/yolov8.hpp"

constexpr double GET_Z = -0.26;
constexpr double HOLD_Z = -0.005;
constexpr double PUT_H_Z = -0.06;
constexpr double PUT_L_Z = -0.16;

class Crane
{
public:
  Crane(const std::string & config_path);

  void right_go_to_map(double y, double z);
  void left_go_to_map(double x, double y, double z);

  bool try_get(int id, bool left);
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

  double last_x() const;
  double last_y(bool left) const;
  double last_z(bool left) const;

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & t, bool left);
  Eigen::Vector3d odom_at(std::chrono::steady_clock::time_point t, bool left);

  void cmd(io::Command command, bool left);
  void cmd(Eigen::Vector3d target_in_odom, bool left);

  bool find_white(int id, bool left);
  void align(auto_crane::LandmarkName name, int id2, bool left);
  void grip(bool grip, bool left);
};

#endif  // SRC__CRANE_HPP