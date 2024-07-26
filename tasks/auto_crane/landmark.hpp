#ifndef AUTO_CRANE__LANDMARK_HPP
#define AUTO_CRANE__LANDMARK_HPP

#include <Eigen/Dense>

namespace auto_crane
{
enum landmarkname
{
};
struct Landmark
{
  Eigen::Vector2d t_landmark2cam;
  std::string name;
};

struct Target
{
  Eigen::Vector2d t_target2map;
  std::string name;
};
}  // namespace auto_crane

#endif