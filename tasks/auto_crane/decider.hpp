#ifndef AUTO_CRANE__DECIDER_HPP
#define AUTO_CRANE__DECIDER_HPP

#include <vector>

#include "yolov8.hpp"

namespace auto_crane
{
class Decider
{
public:
  Decider(const std::vector<std::string> & classes);

private:
};

}  // namespace auto_crane

#endif