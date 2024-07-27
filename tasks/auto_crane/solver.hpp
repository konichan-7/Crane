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

  std::vector<Landmark> solve(std::vector<Detection> & filtered_detections);

private:
  double weights_height_, wood_height_, white_height_;  // 此时不考虑木桩高低的影响
};
}  // namespace auto_crane

#endif