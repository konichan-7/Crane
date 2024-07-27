#include "localizer.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_crane
{

Localizer::Localizer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  t_odo2map_x0_ = yaml["t_odo2map_x0"].as<double>();
  t_odo2map_y0_ = yaml["t_odo2map_y0"].as<double>();
  state_cov0_ = yaml["state_cov0"].as<double>();
  process_noise_ = yaml["process_noise"].as<double>();
  measurement_noise_ = yaml["measurement_noise"].as<double>();
  x0_ << t_odo2map_x0_, t_odo2map_y0_;

  p0_ << state_cov0_, 0.0, 0.0, state_cov0_;
  tools::ExtendedKalmanFilter efk_(x0_, p0_);

  F_ << 1.0, 0.0, 0.0, 1.0;
  Q_ << process_noise_, 0.0, 0.0, process_noise_;
  R_ << measurement_noise_, 0.0, 0.0, measurement_noise_;
  H_ << 1.0, 0.0, 0.0, 1.0;
}

Eigen::Vector2d Localizer::update_coordinate_error(const Eigen::Vector2d & t_odo2map)
{
  if (t_odo2map[0] == 1e6) return t_odo2map;  //此时无法匹配路标，不进行更新
  ekf_.predict(F_, Q_);
  ekf_.update(t_odo2map, H_, R_);
  return ekf_.x;
}

}  // namespace auto_crane