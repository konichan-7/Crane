#ifndef IO__USBCamera_HPP
#define IO__USBCamera_HPP

#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class USBCamera : public CameraBase
{
public:
  USBCamera(const std::string & open_name,const std::string & config_path);
  ~USBCamera() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  cv::Mat read();
  std::string device_name;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  cv::VideoCapture cap_;
  cv::Mat img_;
  std::string open_name_;
  int usb_exposure_, usb_frame_rate_,sharpness_;
  double image_width_, image_height_;
  int usb_gamma_,usb_gain_;

  void open();
  void close();
};

}  // namespace io

#endif