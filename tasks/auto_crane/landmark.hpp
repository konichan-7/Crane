#ifndef AUTO_CRANE__LANDMARK_HPP
#define AUTO_CRANE__LANDMARK_HPP

#include <Eigen/Dense>

namespace auto_crane
{
enum LandmarkName
{
  WEIGHTS,
  WOOD,
  WHITE,
  INVALID
};

enum TargetName
{
  WEIGHT,
  SHORT_WOOD,
  TALL_WOOD,
  CENTER
};

struct Landmark
{
  Eigen::Vector2d t_landmark2cam;  // TODO deprecate
  Eigen::Vector2d in_odom;
  Eigen::Vector2d in_map;
  LandmarkName name;
};

struct Target
{
  Eigen::Vector2d t_target2map;
  TargetName name;
};

}  // namespace auto_crane

#endif