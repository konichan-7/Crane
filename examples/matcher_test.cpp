#include "tasks/auto_crane/matcher.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/cboard/cboard.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_crane/solver.hpp"
#include "tasks/auto_crane/yolov8.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{left l         |                        | bool }"
  "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help") || !cli.has("left")) {
    cli.printMessage();
    return 0;
  }

  auto left = cli.get<bool>("left");
  auto config_path = cli.get<std::string>(0);

  std::vector<std::string> classes = {"weights", "white", "wood"};

  io::USBCamera usbcam(left ? "video0" : "video2", config_path);
  io::CBoard cboard_left("can0", true);
  io::CBoard cboard_right("can0", false);

  tools::Exiter exiter;
  tools::Plotter plotter;

  auto_crane::YOLOV8 yolo("assets/openvino_model_v6/best.xml", classes.size(), classes, "AUTO");
  auto_crane::Solver solver(config_path);
  auto_crane::Matcher matcher(config_path);

  auto yaml = YAML::LoadFile(config_path);
  auto y_left_odom_in_map = yaml["y_left_odom_in_map"].as<double>();
  auto y_right_odom_in_map = yaml["y_right_odom_in_map"].as<double>();
  Eigen::Vector2d t_odom2map{0.0, (left ? y_left_odom_in_map : y_right_odom_in_map)};

  while (!exiter.exit()) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    usbcam.read(img, t);
    Eigen::Vector3d cam_in_odom_left = cboard_left.odom_at(t);
    Eigen::Vector3d cam_in_odom_right = cboard_right.odom_at(t);

    // clang-format off
    Eigen::Vector3d cam_in_odom{
      cam_in_odom_left[0], 
      left ? cam_in_odom_left[1] : cam_in_odom_right[1],
      left ? cam_in_odom_left[2] : cam_in_odom_right[2]
    };
    // clang-format on

    Eigen::Vector2d t_cam2odom = cam_in_odom.head<2>();

    auto detections = yolo.infer(img);
    yolo.save_img(img, detections);

    auto landmarks = solver.solve(detections, t_cam2odom, left);

    tools::logger()->info("-------------------- Match Result --------------------");
    matcher.match(landmarks, t_odom2map);

    auto has_landmark = false;
    for (const auto & landmark : landmarks) {
      if (landmark.name == auto_crane::LandmarkName::INVALID) continue;

      has_landmark = true;
      auto match_error = (landmark.in_map - (landmark.in_odom + t_odom2map)).norm();
      tools::logger()->info(
        "[{}_{}] map=({:.3f}, {:.3f}) odom=({:.3f}, {:.3f}) error={:.3f}",
        auto_crane::LANDMARK_NAMES[landmark.name], landmark.id, landmark.in_map[0],
        landmark.in_map[1], landmark.in_odom[0], landmark.in_odom[1], match_error);
    }

    if (!has_landmark) tools::logger()->info("landmark not found");

    // -------------------- 调试输出 --------------------

    nlohmann::json data;
    data["x_cam_in_odom"] = cam_in_odom[0];
    data["y_cam_in_odom"] = cam_in_odom[1];
    data["z_cam_in_odom"] = cam_in_odom[2];
    plotter.plot(data);

    auto_crane::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("press q to quit", img);
    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}
