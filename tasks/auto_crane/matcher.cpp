#include "matcher.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_crane
{

Matcher::Matcher(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  x_cam2gripper_ = yaml["x_cam2gripper"].as<double>();
  y_cam2gripper_ = yaml["y_cam2gripper"].as<double>();
  landmark_points_ = generate_points(1200, 0, 375, 750);
}

Eigen::Vector2d Matcher::match(
  const Eigen::Vector2d & t_landmark2cam, const Eigen::Vector2d & t_gripper2odo)
{
  if (t_landmark2cam[0] == 0 && t_landmark2cam[1] == 0) return Eigen::Vector2d(0, 0);
  Eigen::Vector2d t_landmark2odo, t_landmark2map;
  t_landmark2odo[0] = t_landmark2cam[0] + x_cam2gripper_ + t_gripper2odo[0];
  t_landmark2odo[1] = t_landmark2cam[1] + y_cam2gripper_ + t_gripper2odo[1];
  double min_distance = 1e10;
  for (const auto & l : landmark_points_) {
    if (
      (t_landmark2odo[0] - l.x) * (t_landmark2odo[0] - l.x) +
        (t_landmark2odo[1] - l.y) * (t_landmark2odo[1] - l.y) <
      min_distance) {
      min_distance = (t_landmark2odo[0] - l.x) * (t_landmark2odo[0] - l.x) +
                     (t_landmark2odo[1] - l.y) * (t_landmark2odo[1] - l.y);
      t_landmark2map[0] = l.x;
      t_landmark2map[1] = l.y;
    }
  }
  return (t_landmark2odo - t_landmark2map);  //t_odo2map
}

}  // namespace auto_crane