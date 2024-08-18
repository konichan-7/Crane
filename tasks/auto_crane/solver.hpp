#ifndef AUTO_CRANE__SOLVER_HPP
#define AUTO_CRANE__SOLVER_HPP

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "yolov8.hpp"

namespace auto_crane
{
class Solver
{
public:
  Solver(const std::string & config_path);

  std::vector<Landmark> solve(
    const std::vector<Detection> & detections, Eigen::Vector2d t_cam2odom, bool left);

  void update_wood(std::vector<Landmark> & landmarks, Eigen::Vector2d t_cam2odom, bool left);

private:
  double z_cam2map_;
  double white_height_;
  double weight_height_;
  double tall_wood_height_;
  double short_wood_height_;
  Eigen::Matrix2d R_cam2odom_left_{{1, 0}, {0, -1}};
  Eigen::Matrix2d R_cam2odom_right_{{-1, 0}, {0, 1}};
};

}  // namespace auto_crane

#endif