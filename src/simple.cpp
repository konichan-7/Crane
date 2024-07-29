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
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

constexpr double EPS = 0.005;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{name n         | video0                 | 端口名称 }"
  "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }";

const std::vector<std::string> classes = {"weights", "white", "wood"};

void go(io::USBCamera & cam, io::CBoard & cboard, Eigen::Vector3d target_in_odom, bool grip)
{
  auto reach = 0;

  while (true) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    cam.read(img, t);
    Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);

    if ((gripper_in_odom - target_in_odom).norm() < EPS)
      reach++;
    else
      reach = 0;

    if (reach > 10) break;

    cboard.send({target_in_odom[0], target_in_odom[1], target_in_odom[2], grip});

    tools::draw_text(img, "go", {50, 50});
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);
  }
}

void lift(io::USBCamera & cam, io::CBoard & cboard, double z, bool grip)
{
  auto t = std::chrono::steady_clock::now();
  Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);
  go(cam, cboard, {gripper_in_odom[0], gripper_in_odom[1], z}, grip);
}

void align_weight(
  io::USBCamera & cam, io::CBoard & cboard, auto_crane::YOLOV8 & yolo, auto_crane::Solver & solver,
  auto_crane::Matcher & matcher, int id1, int id2)
{
  int found = 0;

  while (true) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    cam.read(img, t);
    Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);
    Eigen::Vector2d t_gripper2odom = gripper_in_odom.head<2>();

    auto detections = yolo.infer(img);
    yolo.save_img(img, detections);

    auto landmarks = solver.solve(detections, t_gripper2odom);
    matcher.match(landmarks, {0.0, 0.0});
    solver.update_wood(landmarks, t_gripper2odom);

    for (const auto & l : landmarks) {
      if (l.name != auto_crane::LandmarkName::WEIGHT) continue;
      if (l.id != id1 && l.id != id2) continue;

      cboard.send({l.in_odom[0], l.in_odom[1], 0, false});

      if ((l.in_odom - gripper_in_odom.head<2>()).norm() < EPS)
        found++;
      else
        found = 0;

      break;
    }

    if (found > 10) break;

    auto_crane::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);
  }
}

void align_wood(
  io::USBCamera & cam, io::CBoard & cboard, auto_crane::YOLOV8 & yolo, auto_crane::Solver & solver,
  auto_crane::Matcher & matcher, int wood_id)
{
  int reach_cnt = 0;
  bool wood_found = false;
  Eigen::Vector2d wood_in_odom;

  while (true) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    cam.read(img, t);
    Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);
    Eigen::Vector2d t_gripper2odom = gripper_in_odom.head<2>();

    auto detections = yolo.infer(img);
    yolo.save_img(img, detections);

    auto landmarks = solver.solve(detections, t_gripper2odom);
    matcher.match(landmarks, {0.0, 0.0});
    solver.update_wood(landmarks, t_gripper2odom);

    for (const auto & l : landmarks) {
      if (
        l.name != auto_crane::LandmarkName::SHORT_WOOD &&
        l.name != auto_crane::LandmarkName::TALL_WOOD)
        continue;

      if (l.id != wood_id) continue;

      wood_found = true;
      wood_in_odom = l.in_odom;
      break;
    }

    if (reach_cnt > 10) break;

    if (wood_found) cboard.send({wood_in_odom[0], wood_in_odom[1], 0, true});

    if (wood_found && (wood_in_odom - gripper_in_odom.head<2>()).norm() < EPS) reach_cnt++;

    auto_crane::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);
  }
}

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

  io::USBCamera usbcam(device_name, config_path);
  io::CBoard cboard("can0");

  auto_crane::YOLOV8 yolo("assets/openvino_model_v5/best.xml", classes.size(), classes, "AUTO");
  auto_crane::Solver solver(config_path);
  auto_crane::Matcher matcher(config_path);
  auto_crane::Localizer localizer(config_path);

  Eigen::Vector2d t_odom2map{0.0, 0.0};

  // 去砝码2和砝码3之间
  go(usbcam, cboard, {1.48125, 0.487125 - 0.05, 0.0}, false);

  // 找砝码2或砝码3
  align_weight(usbcam, cboard, yolo, solver, matcher, 2, 3);

  // 降
  lift(usbcam, cboard, -0.26, false);

  // 取
  lift(usbcam, cboard, -0.26, true);
  std::this_thread::sleep_for(500ms);

  // 抬
  lift(usbcam, cboard, 0.0, true);

  // 去木桩0
  go(usbcam, cboard, {2.205, 0.755 - 0.05, 0.0}, true);

  // 找木桩0
  align_wood(usbcam, cboard, yolo, solver, matcher, 0);

  // 降
  lift(usbcam, cboard, -0.06, true);

  // 放
  lift(usbcam, cboard, -0.06, false);
  std::this_thread::sleep_for(500ms);

  // 抬
  lift(usbcam, cboard, 0, false);

  return 0;
}
