#include "decider.hpp"

#include <yaml-cpp/yaml.h>

#include "../../tools/logger.hpp"
#include "yolov8.hpp"

namespace auto_crane
{

Decider::Decider(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  //   t_odo2map_x0_ = yaml["t_odo2map_x0"].as<double>()
}

}  // namespace auto_crane
