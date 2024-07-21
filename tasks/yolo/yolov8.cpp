#include "yolov8.hpp"

#include <random>

#include "tools/img_tools.hpp"

namespace tasks
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

YOLOV8::YOLOV8(const std::string & model_path, int class_num, const std::string & device)
: class_num_(class_num)
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

}  // namespace tasks
