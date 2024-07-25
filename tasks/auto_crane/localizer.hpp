#ifndef AUTO_CRANE_LOCALIZER_HPP
#define AUTO_CRANE_LOCALIZER_HPP

#include <string>

#include "tools/extended_kalman_filter.hpp"

namespace auto_crane
{

class Localizer
{
public:
  Localizer(const std::string & config_path);

private:
  tools::ExtendedKalmanFilter efk_;
  double t_odo2map_x0_, t_odo2map_y0_, noise0_;
  Eigen::Vector2d x0_;
  Eigen::MatrixXd p0_;
};
}  // namespace auto_crane

#endif