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
  judge_distance_ = yaml["judge_distance"].as<double>();  // 单位m^2
  t_cam2gripper_ = {x_cam2gripper_, y_cam2gripper_};
  weights_landmark_points_ = generate_points(1.2, 0, 0.375, 0.75);
  wood_landmark_points_ = generate_points();
}

void Matcher::match(
  const std::vector<Landmark> & landmarks, const Eigen::Vector2d & t_gripper2odo,
  std::vector<Target> & targets, Eigen::Vector2d & t_odo2map)
{
  // 此时没有识别到合适的路标
  if (landmarks.size() == 0) {
    t_odo2map = {1e6, 1e6};
    targets = {};
    return;
  }

  int count = 0;
  Eigen::Vector2d t_landmark2odo_weights;
  Eigen::Vector2d t_landmark2odo_wood;
  Eigen::Vector2d t_landmark2odo_white;
  Eigen::Vector2d t_odo2map1{0, 0}, t_odo2map2{0, 0}, t_odo2map3{0, 0};

  for (const auto & landmark : landmarks) {
    if (landmark.name == LandmarkName::WEIGHTS) {
      // 计算路标在odo系下的坐标
      t_landmark2odo_weights = landmark.t_landmark2cam + t_cam2gripper_ + t_gripper2odo;

      // 通过遍历砝码可能的位置匹配出map系下的坐标
      for (const auto & l : weights_landmark_points_) {
        auto error_distance = (t_landmark2odo_weights - l).norm();

        if (error_distance < judge_distance_) {
          targets.push_back({l, TargetName::WEIGHT});
          t_odo2map1 = l - t_landmark2odo_weights;
          ++count;
          tools::logger()->info("{}weight matched, weight2map is:{:.3f},{:.3f}", count, l[0], l[1]);
          break;
        }
      }
    }

    else if (landmark.name == LandmarkName::WOOD) {
      t_landmark2odo_wood = landmark.t_landmark2cam + t_cam2gripper_ + t_gripper2odo;

      int i = 0;
      for (const auto & w : wood_landmark_points_) {
        auto error_distance = (t_landmark2odo_wood - w).norm();

        if (error_distance < judge_distance_) {
          Target target;
          target.t_target2map = w;

          target.name =
            (target.t_target2map[1] == 0) ? TargetName::SHORT_WOOD : TargetName::TALL_WOOD;

          targets.push_back(target);
          t_odo2map2 = target.t_target2map - t_landmark2odo_wood;
          ++count;
          tools::logger()->info("{}wood matched, wood2map is:{:.3f},{:.3f}", count, w[0], w[1]);
          break;
        }
      }
    }

    else if (landmark.name == LandmarkName::WHITE) {
      t_landmark2odo_white = landmark.t_landmark2cam + t_cam2gripper_ + t_gripper2odo;

      int i = 0;
      for (const auto & w : weights_landmark_points_) {
        auto error_distance = (t_landmark2odo_white - w).norm();
        if (error_distance < judge_distance_) {
          Target target;
          target.t_target2map = w;

          targets.push_back({w, TargetName::TWHITE});
          t_odo2map3 = target.t_target2map - t_landmark2odo_white;
          ++count;
          ++i;
          tools::logger()->info("{}white matched, white2map is:{:.3f},{:.3f}", count, w[0], w[1]);
          break;
        }
      }
    }
  }
  // 此时匹配失败
  if (count == 0) {
    targets = {};
    t_odo2map = {1e6, 1e6};
    tools::logger()->info("falied to match landmarks!");
    return;
  }

  t_odo2map = (t_odo2map1 + t_odo2map2 + t_odo2map3) / count;

  return;
}

}  // namespace auto_crane