#include "matcher.hpp"

#include <yaml-cpp/yaml.h>

#include "../../tools/logger.hpp"

namespace auto_crane
{

Matcher::Matcher(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  x_cam2gripper_ = yaml["x_cam2gripper"].as<double>();
  y_cam2gripper_ = yaml["y_cam2gripper"].as<double>();
  judge_distance_ = yaml["judge_distance"].as<double>();
  landmark_points_ = generate_points(1200, 0, 375, 750);
}

Eigen::Vector2d Matcher::match(
  const Landmark & landmark, const Eigen::Vector2d & t_gripper2odo, Target & target)
{
  if (landmark.name == "invalid") return Eigen::Vector2d(0, 0);  // 此时没有识别到合适的路标

  Eigen::Vector2d t_landmark2odo;
  t_landmark2odo[0] = landmark.t_landmark2cam[0] + x_cam2gripper_ + t_gripper2odo[0];
  t_landmark2odo[1] = landmark.t_landmark2cam[1] + y_cam2gripper_ + t_gripper2odo[1];

  int count = 0;
  for (const auto & l : landmark_points_) {
    auto error_distance =
      std::pow(t_landmark2odo[0] - l.x, 2) + std::pow(t_landmark2odo[1] - l.y, 2);

    if (error_distance < judge_distance_) {
      target.t_target2map[0] = l.x;
      target.t_target2map[1] = l.y;
      target.name = "weights";
      ++count;
    }
  }

  // 此时匹配失败
  if (count == 0) {
    target.t_target2map << 0, 0;
    target.name = "invalid";
    tools::logger()->info("falied to match landmarks!");
    return Eigen::Vector2d{1e6, 1e6};
  }

  Eigen::Vector2d t_map2odo = t_landmark2odo - target.t_target2map;
  Eigen::Vector2d t_odo2map = {t_map2odo[1], t_map2odo[0]};
  return t_odo2map;  // t_odo2map
}

}  // namespace auto_crane