#include "localizer.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_crane
{

Localizer::Localizer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  t_odo2map_x0_ = yaml["t_odo2map_x0"].as<double>();
  t_odo2map_y0_ = yaml["t_odo2map_y0"].as<double>();
  noise0_ = yaml["noise0"].as<double>();
  x0_ = {t_odo2map_x0_, t_odo2map_y0_};
  p0_ = {noise0_, 0.0, 0.0, noise0_};
  tools::ExtendedKalmanFilter efk_(x0_, p0_);
}

}  // namespace auto_crane