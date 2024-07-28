#include "matcher.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_crane
{
Matcher::Matcher(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  max_match_error_ = yaml["max_match_error"].as<double>();

  /// weights_in_map_ and whites_in_map_

  auto cx = 1.2, cy = 0.0;
  auto r1 = 0.375, r2 = 0.75;

  for (int i = 0; i < 6; i++) {
    auto angle = tools::limit_rad(i * M_PI / 3);

    whites_in_map_.push_back({cx + r1 * std::cos(angle), cy + r1 * std::sin(angle)});
    whites_in_map_.push_back({cx + r2 * std::cos(angle), cy + r2 * std::sin(angle)});

    weights_in_map_.push_back({cx + r1 * std::cos(angle), cy + r1 * std::sin(angle)});
    weights_in_map_.push_back({cx + r2 * std::cos(angle), cy + r2 * std::sin(angle)});
  }

  whites_in_map_.push_back({0.0, 0.0});

  /// woods_in_map

  woods_in_map_.push_back({2.205, 0.755});
  woods_in_map_.push_back({-1.305, 0.755});
  woods_in_map_.push_back({-1.305, -0.755});
  woods_in_map_.push_back({2.205, -0.755});
  woods_in_map_.push_back({1.2, 0});
}

void Matcher::match(std::vector<Landmark> & landmarks, Eigen::Vector2d t_odom2map)
{
  for (auto & landmark : landmarks) {
    if (landmark.name == LandmarkName::WEIGHT)
      match(landmark, weights_in_map_, t_odom2map);

    else if (landmark.name == LandmarkName::WHITE)
      match(landmark, whites_in_map_, t_odom2map);

    else if (landmark.name == LandmarkName::TALL_WOOD)
      match(landmark, woods_in_map_, t_odom2map);
  }
}

void Matcher::match(
  Landmark & landmark, const std::vector<Eigen::Vector2d> & landmarks_in_map,
  Eigen::Vector2d t_odom2map)
{
  Eigen::Vector2d predicted = landmark.in_odom + t_odom2map;

  auto matched = -1;
  auto min_match_error = 1e6;

  for (int i = 0; i < landmarks_in_map.size(); i++) {
    auto match_error = (predicted - landmarks_in_map[i]).norm();
    if (match_error < min_match_error) {
      min_match_error = match_error;
      matched = i;
    }
  }

  if (min_match_error > max_match_error_) {
    tools::logger()->warn(
      "[Matcher] [{}_{}] large min_match_error: {:.3f}", auto_crane::LANDMARK_NAMES[landmark.name],
      matched, min_match_error);
    landmark.name = LandmarkName::INVALID;
    return;
  }

  if (landmark.name == LandmarkName::TALL_WOOD && matched == 4) {
    landmark.name = LandmarkName::SHORT_WOOD;
  }

  landmark.in_map = landmarks_in_map[matched];
  landmark.id = matched;
}

}  // namespace auto_crane
