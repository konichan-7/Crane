#ifndef YOLO__YOLOV8_HPP
#define YOLO__YOLOV8_HPP

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

namespace yolo
{
struct Detection
{
  int class_id;
  float confidence;
  cv::Rect box;
  cv::Point2i center;

  Detection(int class_id, float confidence, const cv::Rect & box)
  : class_id(class_id),
    confidence(confidence),
    box(box),
    center(box.x + box.width / 2.0f, box.y + box.height / 2.0f)
  {
  }
  Detection() {};
};

void draw_detections(
  cv::Mat & img, const std::vector<Detection> & detections,
  const std::vector<std::string> & classes = {});

class YOLOV8
{
public:
  YOLOV8(const std::string & model_path, int class_num, const std::string & device = "AUTO");

  std::vector<Detection> infer(const cv::Mat & bgr_img);

private:
  int class_num_;
  float nms_threshold_ = 0.3;
  float score_threshold_ = 0.7;
  ov::Core core_;
  ov::CompiledModel compiled_model_;

  std::vector<Detection> parse(double scale, cv::Mat & output) const;
};

}  // namespace yolo

#endif  // yolo__YOLOV8_HPP