#include "solver.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_crane
{
Solver::Solver(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);

  z_cam2map_ = yaml["z_cam2map"].as<double>();
  white_height_ = yaml["white_height"].as<double>();
  weight_height_ = yaml["weight_height"].as<double>();
  tall_wood_height_ = yaml["tall_wood_height"].as<double>();
  short_wood_height_ = yaml["short_wood_height"].as<double>();
}

std::vector<Landmark> Solver::solve(
  const std::vector<Detection> & detections, Eigen::Vector2d t_cam2odom, bool left)
{
  std::vector<Landmark> landmarks;

  for (const auto & d : detections) {
    auto angle_x = (d.center.x - 960.0) / 1920.0 * 87.7 / 57.3;
    auto angle_y = (d.center.y - 540.0) / 1080.0 * 56.7 / 57.3;

    Landmark l;
    double z;

    if (d.class_id == 0) {
      z = z_cam2map_ - weight_height_;
      l.name = LandmarkName::WEIGHT;  // TODO WEIGHT
    }

    else if (d.class_id == 1) {
      z = z_cam2map_ - white_height_;
      l.name = LandmarkName::WHITE;
    }

    else if (d.class_id == 2) {
      z = z_cam2map_ - tall_wood_height_;  // 此时不考虑木桩高低的影响
      l.name = LandmarkName::TALL_WOOD;
    }

    l.in_cam = {std::tan(angle_x) * z, std::tan(angle_y) * z};

    Eigen::Matrix2d R_cam2odom = left ? R_cam2odom_left_ : R_cam2odom_right_;
    l.in_odom = R_cam2odom * l.in_cam + t_cam2odom;

    landmarks.push_back(l);
  }

  return landmarks;
}

void Solver::update_wood(std::vector<Landmark> & landmarks, Eigen::Vector2d t_cam2odom, bool left)
{
  for (auto & l : landmarks) {
    if (l.name != LandmarkName::SHORT_WOOD) continue;

    l.in_cam = l.in_cam * (z_cam2map_ - short_wood_height_) / (z_cam2map_ - tall_wood_height_);

    Eigen::Matrix2d R_cam2odom = left ? R_cam2odom_left_ : R_cam2odom_right_;
    l.in_odom = R_cam2odom * l.in_cam + t_cam2odom;
  }
}

}  // namespace auto_crane
