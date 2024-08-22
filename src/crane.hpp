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
constexpr double PUT_L_Z = -0.155;

class Crane
{
public:
  Crane(const std::string & config_path);

  void wait_to_start();

  void forward(auto_crane::LandmarkName name, int id, double z, bool left);
  bool try_get(int id, bool left);
  void put(int id, bool left);
  void puts(int id_l, int id_r);

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

  double x_left_gripper_offset_;
  double x_right_gripper_offset_;

  double y_left_get_offset_;
  double y_right_get_offset_;

  Eigen::Vector2d left_wood_offset_;
  Eigen::Vector2d right_wood_offset_;

  double y_left_put_offset_;
  double y_right_put_offset_;

  io::Command left_last_cmd_{0.0, 0.0, 0.0, false, false};
  io::Command right_last_cmd_{0.0, 0.0, 0.0, false, false};

  double last_x() const;
  double last_y(bool left) const;
  double last_z(bool left) const;

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & t, bool left);
  Eigen::Vector3d odom_at(std::chrono::steady_clock::time_point t, bool left);

  void cmd(io::Command command, bool left);

  void go_no_wait(Eigen::Vector3d target_in_odom, bool left);
  void go(Eigen::Vector3d target_in_odom, bool left);
  void go_both(Eigen::Vector3d l_in_odom, Eigen::Vector3d r_in_odom);

  bool find_white(int id, bool left);
  void align(auto_crane::LandmarkName name, int id, bool left);
  void align_woods(int id_l, int id_r);

  void grip(bool grip, bool left);
  void grip_both(bool grip_l, bool grip_r);
};

#endif  // SRC__CRANE_HPP