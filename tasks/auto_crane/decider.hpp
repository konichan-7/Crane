#ifndef AUTO_CRANE__DECIDER_HPP
#define AUTO_CRANE__DECIDER_HPP

#include <vector>

#include "yolov8.hpp"

namespace auto_crane
{
class Decider
{
public:
  Decider(const std::vector<std::string> classes);
  std::vector<Detection> filter(const std::vector<Detection> & detections);
  void save_img(const cv::Mat & img, const std::vector<Detection> & targets);

private:
  std::string save_path_;
  std::vector<std::string> classes_;
};

}  // namespace auto_crane

#endif