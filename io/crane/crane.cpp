#include "crane.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace io
{
Crane::Crane(CBoard & left, CBoard & right, const std::string & config_path)
: left_(left), right_(right)
{
  auto yaml = YAML::LoadFile(config_path);
  y_left_in_odom_ = yaml["y_left_in_odom"].as<double>();
  y_right_in_odom_ = yaml["y_right_in_odom"].as<double>();
}

Eigen::Vector3d Crane::odom_at(std::chrono::steady_clock::time_point t, bool left)
{
  Eigen::Vector3d left_gripper = left_.odom_at(t);
  Eigen::Vector3d right_gripper = right_.odom_at(t);

  // clang-format off
  Eigen::Vector3d gripper_in_odom{
    left_gripper[0], 
    left ? left_gripper[1] : right_gripper[1],
    left ? left_gripper[2] : right_gripper[2]
  };
  // clang-format on

  gripper_in_odom[1] += left ? y_left_in_odom_ : y_right_in_odom_;

  return gripper_in_odom;
}

void Crane::move_xy(double x_in_odom, double y_in_odom, bool left)
{
  auto command = left ? left_last_cmd_ : right_last_cmd_;
  command.x = x_in_odom;
  command.y = y_in_odom - (left ? y_left_in_odom_ : y_right_in_odom_);
  cmd(command, left);
}

void Crane::cmd(Command command, bool left)
{
  if (left) {
    left_.send(command);
    left_last_cmd_ = command;
    return;
  }

  auto left_cmd = left_last_cmd_;
  left_cmd.x = command.x;
  left_.send(left_cmd);
  left_last_cmd_ = left_cmd;

  right_.send(command);
  right_last_cmd_ = command;
}

}  // namespace io
