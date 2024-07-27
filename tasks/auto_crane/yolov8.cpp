#include "yolov8.hpp"

#include <fmt/chrono.h>

#include <filesystem>
#include <random>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_crane
{
std::vector<cv::Scalar> class_colors;

static cv::Scalar get_color(int class_id)
{
  while (class_id >= class_colors.size()) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(100, 255);
    class_colors.emplace_back(cv::Scalar(dis(gen), dis(gen), dis(gen)));
  }
  return class_colors[class_id];
}

void draw_detections(
  cv::Mat & img, const std::vector<Detection> & detections,
  const std::vector<std::string> & classes)
{
  for (const auto & d : detections) {
    auto box = d.box;
    auto color = get_color(d.class_id);
    auto label = (classes.empty()) ? std::to_string(d.class_id) : classes[d.class_id];
    auto text = label + " " + std::to_string(d.confidence).substr(0, 4);
    auto text_size = cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
    auto text_box = cv::Rect(box.x, box.y - 40, text_size.width + 10, text_size.height + 20);

    cv::rectangle(img, box, color, 2);
    cv::rectangle(img, text_box, color, cv::FILLED);
    cv::putText(img, text, {box.x + 5, box.y - 10}, cv::FONT_HERSHEY_DUPLEX, 1, {0, 0, 0});
    tools::draw_point(img, d.center, color);
  }
}

YOLOV8::YOLOV8(
  const std::string & model_path, int class_num, const std::vector<std::string> & classes,
  const std::string & device)
: class_num_(class_num), classes_(classes)
{
  auto model = core_.read_model(model_path);

  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0);

  // TODO: ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
  save_path_ = "imgs";
  std::filesystem::create_directory("imgs");
}

std::vector<Detection> YOLOV8::infer(const cv::Mat & bgr_img)
{
  auto x_scale = static_cast<double>(640) / bgr_img.rows;
  auto y_scale = static_cast<double>(640) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  // preproces
  auto input = cv::Mat(640, 640, CV_8UC3);
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

  /// infer
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

  return parse(scale, output);
}

std::vector<Detection> YOLOV8::parse(double scale, cv::Mat & output) const
{
  // for each row: xywh + classess
  cv::transpose(output, output);

  std::vector<int> ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;

  for (int r = 0; r < output.rows; r++) {
    auto xywh = output.row(r).colRange(0, 4);
    auto scores = output.row(r).colRange(4, 4 + class_num_);

    double score;
    cv::Point max_point;
    cv::minMaxLoc(scores, nullptr, &score, nullptr, &max_point);

    if (score < score_threshold_) continue;

    auto x = xywh.at<float>(0);
    auto y = xywh.at<float>(1);
    auto w = xywh.at<float>(2);
    auto h = xywh.at<float>(3);
    auto left = static_cast<int>((x - 0.5 * w) / scale);
    auto top = static_cast<int>((y - 0.5 * h) / scale);
    auto width = static_cast<int>(w / scale);
    auto height = static_cast<int>(h / scale);

    ids.emplace_back(max_point.x);
    confidences.emplace_back(score);
    boxes.emplace_back(left, top, width, height);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::vector<Detection> detections;
  for (const auto & i : indices) detections.emplace_back(ids[i], confidences[i], boxes[i]);

  return detections;
}

std::vector<Detection> YOLOV8::filter(const std::vector<Detection> & detections)
{
  if (detections.size() == 0) {
    return detections;
  }

  std::vector<Detection> targets;
  Detection wieghts_target, wood_target;
  int weights_min_distance = 1e6, wood_min_distance = 1e6;

  int weights_count = 0, wood_count = 0;

  for (auto d : detections) {
    auto distance = std::pow(d.center.x - 960, 2) + std::pow(d.center.y - 540, 2);
    if (d.class_id == 0) {
      ++weights_count;
      if (distance < weights_min_distance) {
        wieghts_target = d;
        weights_min_distance = distance;
      }
    }
    if (d.class_id == 1) {
      ++wood_count;
      if (distance < wood_min_distance) {
        wood_target = d;
        wood_min_distance = distance;
      }
    }
  }

  if (weights_count == 0) {
    targets.push_back(wood_target);
  }

  if (wood_count == 0) {
    targets.push_back(wieghts_target);
  }

  if (weights_count > 0 && wood_count > 0) {
    targets.push_back(wieghts_target);
    targets.push_back(wood_target);
  }

  tools::logger()->debug("after filter the size of targets:{}", targets.size());
  return targets;
}

void YOLOV8::save_img(const cv::Mat & img, const std::vector<Detection> & targets)
{
  for (const auto & t : targets) {
    if (t.confidence < 0.85) {
      auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
      auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, classes_[t.class_id], file_name);
      cv::imwrite(img_path, img);
      return;
    }
  }
  return;
}

Landmark YOLOV8::pixel2cam(const std::vector<Detection> & landmarks)
{
  if (landmarks.size() == 0) return Landmark{Eigen::Vector2d{0.0, 0.0}, "invalid"};

  Detection landmark;
  Eigen::Vector2d t_landmark2cam;
  for (const auto & l : landmarks) {
    if (l.class_id == 1) {
      continue;
    }
    landmark = l;
  }

  auto angle_h = (landmark.center.x - 960.0) / 1920.0 * 87.7 / 57.3;
  auto angle_v = (landmark.center.y - 540.0) / 1080.0 * 56.7 / 57.3;

  t_landmark2cam[0] = std::tan(angle_h) * 0.365;   // 单位m
  t_landmark2cam[1] = -std::tan(angle_v) * 0.365;  //根据坐标系定义，需要取反

  return Landmark{t_landmark2cam, "weights"};
}

}  // namespace auto_crane
