#include "weight_matcher.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_crane
{
WeightMatcher::WeightMatcher(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  auto x_cam2gripper = yaml["x_cam2gripper"].as<double>();
  auto y_cam2gripper = yaml["y_cam2gripper"].as<double>();
  max_match_error_ = yaml["max_match_error"].as<double>();
  t_cam2gripper_ = {x_cam2gripper, y_cam2gripper};

  // compute weights_in_map_
  auto cx = 1.2;
  auto cy = 0.0;
  auto r1 = 0.375;
  auto r2 = 0.75;
  for (int i = 0; i < 6; i++) {
    auto angle = tools::limit_rad(i * M_PI / 3);
    weights_in_map_.push_back({cx + r1 * std::cos(angle), cy + r1 * std::sin(angle)});
    weights_in_map_.push_back({cx + r2 * std::cos(angle), cy + r2 * std::sin(angle)});
  }
}

std::vector<MatchResult> WeightMatcher::match(
  const std::vector<Landmark> & landmarks, Eigen::Vector2d t_gripper2odom,
  Eigen::Vector2d t_odom2map)
{
  std::vector<MatchResult> match_results;

  for (const auto & landmark : landmarks) {
    if (landmark.name != auto_crane::LandmarkName::WEIGHTS) continue;

    Eigen::Vector2d weight_in_cam = landmark.t_landmark2cam;
    Eigen::Vector2d weight_in_odom = weight_in_cam + t_cam2gripper_ + t_gripper2odom;
    match_results.push_back(match(weight_in_odom, t_odom2map));
  }

  return match_results;
}

MatchResult WeightMatcher::match(Eigen::Vector2d weight_in_odom, Eigen::Vector2d t_odom2map)
{
  Eigen::Vector2d predicted = weight_in_odom + t_odom2map;

  auto min_match_error = 1e6;
  Eigen::Vector2d matched;
  for (const Eigen::Vector2d & weight : weights_in_map_) {
    auto match_error = (predicted - weight).norm();
    if (match_error < min_match_error) {
      min_match_error = match_error;
      matched = weight;
    }
  }

  if (min_match_error > max_match_error_) {
    tools::logger()->warn("[Matcher] large min_match_error: {:.3f}", min_match_error);
    return {false, {0.0, 0.0}};
  }

  return {true, matched};
}

}  // namespace auto_crane
