#ifndef AUTO_CRANE__MATCHER_HPP
#define AUTO_CRANE__MATCHER_HPP

#include <Eigen/Dense>
#include <vector>

#include "landmark.hpp"

namespace auto_crane
{

class Matcher
{
public:
  Matcher(const std::string & config_path);

  void match(std::vector<Landmark> & landmarks, Eigen::Vector2d t_odom2map);

private:
  double max_match_error_;
  std::vector<Eigen::Vector2d> woods_in_map_;
  std::vector<Eigen::Vector2d> whites_in_map_;
  std::vector<Eigen::Vector2d> weights_in_map_;

  void match(
    Landmark & landmark, const std::vector<Eigen::Vector2d> & landmarks_in_map,
    Eigen::Vector2d t_odom2map);
};

}  // namespace auto_crane

#endif