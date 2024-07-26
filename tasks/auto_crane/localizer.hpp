#ifndef AUTO_CRANE__LOCALIZER_HPP
#define AUTO_CRANE__LOCALIZER_HPP

#include <string>

#include "tools/extended_kalman_filter.hpp"

namespace auto_crane
{

class Localizer
{
public:
  Localizer(const std::string & config_path);

  Eigen::Vector2d update_coordinate_error(const Eigen::Vector2d & t_odo2map);

private:
  tools::ExtendedKalmanFilter ekf_;
  double t_odo2map_x0_, t_odo2map_y0_, state_cov0_, process_noise_, measurement_noise_;
  Eigen::Vector2d x0_;
  Eigen::MatrixXd p0_;
  Eigen::MatrixXd F_, Q_, R_, H_;
};
}  // namespace auto_crane

#endif