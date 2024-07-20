#include "camera.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  auto camera_name = yaml["camera_name"].as<std::string>();
  auto exposure_ms = yaml["exposure_ms"].as<double>();

  if (camera_name == "mindvision") {
    auto gamma = yaml["gamma"].as<double>();
    auto vid_pid = yaml["vid_pid"].as<std::string>();
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto gain = yaml["gain"].as<double>();
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io
