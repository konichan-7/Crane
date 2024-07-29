#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/cboard/cboard.hpp"
#include "io/command.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_crane/localizer.hpp"
#include "tasks/auto_crane/matcher.hpp"
#include "tasks/auto_crane/solver.hpp"
#include "tasks/auto_crane/yolov8.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{name n         | video0                 | 端口名称 }"
  "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto device_name = cli.get<std::string>("name");
  auto config_path = cli.get<std::string>(0);

  auto yaml = YAML::LoadFile(config_path);
  auto x_cam2gripper = yaml["x_cam2gripper"].as<double>();
  auto y_cam2gripper = yaml["y_cam2gripper"].as<double>();
  Eigen::Vector2d t_cam2gripper = {x_cam2gripper, y_cam2gripper};

  std::vector<std::string> classes = {"weights", "white", "wood"};

  io::USBCamera usbcam(device_name, config_path);
  io::CBoard cboard("can0");

  tools::Exiter exiter;
  tools::Plotter plotter;

  auto_crane::YOLOV8 yolo("assets/openvino_model_v5/best.xml", classes.size(), classes, "AUTO");
  auto_crane::Solver solver(config_path);
  auto_crane::Matcher matcher(config_path);
  auto_crane::Localizer localizer(config_path);

  Eigen::Vector2d t_odom2map{0.0, 0.0};

  // 去67之间
  for (int i = 0; i < 200; i++) {
    cboard.send({0.6375, 0.0, 0.0, false});
    std::this_thread::sleep_for(10ms);
  }

  // 找6或7
  for (int i = 0; i < 50; i++) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    usbcam.read(img, t);
    Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);
    Eigen::Vector2d t_gripper2odom = gripper_in_odom.head<2>();

    auto detections = yolo.infer(img);
    yolo.save_img(img, detections);

    auto landmarks = solver.solve(detections, t_gripper2odom);
    matcher.match(landmarks, t_odom2map);
    solver.update_wood(landmarks, t_gripper2odom);

    for (const auto & l : landmarks) {
      if (l.name != auto_crane::LandmarkName::WEIGHT) continue;
      if (l.id != 6 && l.id != 7) continue;

      cboard.send({l.in_odom[0], l.in_odom[1], 0, false});
      tools::logger()->info("send");
      break;
    }

    auto_crane::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("press q to quit", img);
    if (cv::waitKey(1) == 'q') break;
  }

  // 降
  for (int i = 0; i < 200; i++) {
    auto t = std::chrono::steady_clock::now();
    Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);
    cboard.send({gripper_in_odom[0], gripper_in_odom[1], -0.25, false});
    std::this_thread::sleep_for(10ms);
  }

  // 取
  for (int i = 0; i < 200; i++) {
    auto t = std::chrono::steady_clock::now();
    Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);
    cboard.send({gripper_in_odom[0], gripper_in_odom[1], -0.25, true});
    std::this_thread::sleep_for(10ms);
  }

  // 抬
  for (int i = 0; i < 200; i++) {
    auto t = std::chrono::steady_clock::now();
    Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);
    cboard.send({gripper_in_odom[0], gripper_in_odom[1], 0.0, true});
    std::this_thread::sleep_for(10ms);
  }

  // 去小木桩
  for (int i = 0; i < 200; i++) {
    cboard.send({1.2, 0.0, 0.0, true});
    std::this_thread::sleep_for(10ms);
  }

  // 找小木桩
  for (int i = 0; i < 50; i++) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    usbcam.read(img, t);
    Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);
    Eigen::Vector2d t_gripper2odom = gripper_in_odom.head<2>();

    auto detections = yolo.infer(img);
    yolo.save_img(img, detections);

    auto landmarks = solver.solve(detections, t_gripper2odom);
    matcher.match(landmarks, t_odom2map);
    solver.update_wood(landmarks, t_gripper2odom);

    for (const auto & l : landmarks) {
      if (l.name != auto_crane::LandmarkName::SHORT_WOOD) continue;

      cboard.send({l.in_odom[0], l.in_odom[1], 0, true});
      tools::logger()->info("send");
      break;
    }

    auto_crane::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("press q to quit", img);
    if (cv::waitKey(1) == 'q') break;
  }

  // 降
  for (int i = 0; i < 200; i++) {
    auto t = std::chrono::steady_clock::now();
    Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);
    cboard.send({gripper_in_odom[0], gripper_in_odom[1], -0.14, true});
    std::this_thread::sleep_for(10ms);
  }

  // 放
  for (int i = 0; i < 200; i++) {
    auto t = std::chrono::steady_clock::now();
    Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);
    cboard.send({gripper_in_odom[0], gripper_in_odom[1], -0.14, false});
    std::this_thread::sleep_for(10ms);
  }

  // 抬
  for (int i = 0; i < 200; i++) {
    auto t = std::chrono::steady_clock::now();
    Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);
    cboard.send({gripper_in_odom[0], gripper_in_odom[1], 0.0, false});
    std::this_thread::sleep_for(10ms);
  }

  return 0;
}
