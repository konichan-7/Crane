#include "decider.hpp"

#include <fmt/chrono.h>

#include <filesystem>
#include <vector>

#include "../../tools/logger.hpp"
#include "yolov8.hpp"

namespace auto_crane
{

Decider::Decider(const std::vector<std::string> & classes)
{
  classes_ = classes;
  save_path_ = "imgs";
  std::filesystem::create_directory("imgs");
}

std::vector<Detection> Decider::filter(const std::vector<Detection> & detections)
{
  if (detections.size() == 0) {
    return detections;
  }

  std::vector<Detection> targets;
  Detection wieghts_target, wood_target;
  int weights_min_distance = 0, wood_min_distance = 0;

  int weights_count = 0, wood_count = 0;

  for (auto d : detections) {
    if (d.class_id == 0) {
      ++weights_count;
      if (
        ((d.center.x - 960) * (d.center.x - 960) + (d.center.y - 540) * (d.center.y - 540)) <
        weights_min_distance) {
        wieghts_target = d;
        weights_min_distance =
          ((d.center.x - 960) * (d.center.x - 960) + (d.center.y - 540) * (d.center.y - 540));
      }
    }
    if (d.class_id == 1) {
      ++wood_count;
      if (
        ((d.center.x - 960) * (d.center.x - 960) + (d.center.y - 540) * (d.center.y - 540)) <
        wood_min_distance) {
        wood_target = d;
        wood_min_distance =
          ((d.center.x - 960) * (d.center.x - 960) + (d.center.y - 540) * (d.center.y - 540));
      }
    }
  }
  if (weights_count == 0) {
    targets.push_back(wood_target);
  }
  if (wood_count == 0) {
    targets.push_back(wieghts_target);
  }
  if (weights_count && wood_count) {
    targets.push_back(wieghts_target);
    targets.push_back(wood_target);
  }
  tools::logger()->debug("after filter the size of targets:{}", targets.size());
  return targets;
}

void Decider::save_img(const cv::Mat & img, const std::vector<Detection> & targets)
{
  for (const auto t : targets) {
    if (t.confidence < 0.85) {
      auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
      auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, classes_[t.class_id], file_name);
      cv::imwrite(img_path, img);
      return;
    }
  }
  return;
}
}  // namespace auto_crane
