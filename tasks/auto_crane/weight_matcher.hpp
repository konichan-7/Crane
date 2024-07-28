#ifndef AUTO_CRANE__WEIGHT_MATCHER_HPP
#define AUTO_CRANE__WEIGHT_MATCHER_HPP

#include <Eigen/Dense>
#include <vector>

#include "landmark.hpp"

namespace auto_crane
{

struct MatchResult
{
  bool success;
  Eigen::Vector2d weight_in_map;
};

class WeightMatcher
{
public:
  WeightMatcher(const std::string & config_path);

  std::vector<MatchResult> match(
    const std::vector<Landmark> & landmarks, Eigen::Vector2d t_gripper2odom,
    Eigen::Vector2d t_odom2map);
  MatchResult match(Eigen::Vector2d weight_in_odom, Eigen::Vector2d t_odom2map);

private:
  double max_match_error_;
  Eigen::Vector2d t_cam2gripper_;
  std::vector<Eigen::Vector2d> weights_in_map_;
};

}  // namespace auto_crane

#endif  // AUTO_CRANE__WEIGHT_MATCHER_HPP