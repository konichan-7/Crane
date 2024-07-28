#ifndef AUTO_CRANE__LANDMARK_HPP
#define AUTO_CRANE__LANDMARK_HPP

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace auto_crane
{
enum LandmarkName
{
  WHITE,
  WEIGHT,
  TALL_WOOD,
  SHORT_WOOD,
  INVALID
};

const std::vector<std::string> LANDMARK_NAMES = {
  "WHITE", "WEIGHT", "TALL_WOOD", "SHORT_WOOD", "INVALID"};

struct Landmark
{
  Eigen::Vector2d in_cam;
  Eigen::Vector2d in_odom;
  Eigen::Vector2d in_map;
  LandmarkName name;
};

}  // namespace auto_crane

#endif