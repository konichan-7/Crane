#include "solver.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_crane
{
Solver::Solver(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  weights_height_ = yaml["weights_height"].as<double>();
  wood_height_ = yaml["tall_wood_height"].as<double>();
  white_height_ = yaml["white_height"].as<double>();
}

std::vector<Landmark> Solver::solve(std::vector<Detection> & filtered_detections)
{
  if (filtered_detections.size() == 0) return std::vector<Landmark>{};

  std::vector<Landmark> landmarks;
  for (const auto & f : filtered_detections) {
    if (f.class_id == 0) {
      Landmark landmark;
      auto angle_h = (f.center.x - 960.0) / 1920.0 * 87.7 / 57.3;
      auto angle_v = (f.center.y - 540.0) / 1080.0 * 56.7 / 57.3;

      landmark.t_landmark2cam[0] = std::tan(angle_h) * weights_height_;  // 单位m
      landmark.t_landmark2cam[1] = -std::tan(angle_v) * weights_height_;  //根据坐标系定义，需要取反
      landmark.name = LandmarkName::WEIGHTS;
      landmarks.push_back(landmark);
    } else if (f.class_id == 1) {
      Landmark landmark;
      auto angle_h = (f.center.x - 960.0) / 1920.0 * 87.7 / 57.3;
      auto angle_v = (f.center.y - 540.0) / 1080.0 * 56.7 / 57.3;

      landmark.t_landmark2cam[0] = std::tan(angle_h) * white_height_;  // 单位m
      landmark.t_landmark2cam[1] = -std::tan(angle_v) * white_height_;  // 根据坐标系定义，需要取反
      landmark.name = LandmarkName::WHITE;
      landmarks.push_back(landmark);
    } else if (f.class_id == 2) {
      Landmark landmark;
      auto angle_h = (f.center.x - 960.0) / 1920.0 * 87.7 / 57.3;
      auto angle_v = (f.center.y - 540.0) / 1080.0 * 56.7 / 57.3;

      landmark.t_landmark2cam[0] = std::tan(angle_h) * wood_height_;  // 单位m
      landmark.t_landmark2cam[1] = -std::tan(angle_v) * wood_height_;  // 根据坐标系定义，需要取反
      landmark.name = LandmarkName::WOOD;
      landmarks.push_back(landmark);
    }
  }
  return landmarks;
}

}  // namespace auto_crane
