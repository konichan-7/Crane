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

constexpr int REACH_CNT = 10;
constexpr double EPS = 0.01;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{name n         | video0                 | 端口名称 }"
  "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }";

const std::vector<std::string> classes = {"weights", "white", "wood"};

void go(
  io::USBCamera & cam, io::CBoard & cboard, Eigen::Vector3d target_in_odom, bool grip, bool slow)
{
  auto reach_cnt = 0;

  while (true) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    cam.read(img, t);
    Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);

    if ((gripper_in_odom - target_in_odom).norm() < EPS)
      reach_cnt++;
    else
      reach_cnt = 0;

    if (reach_cnt > REACH_CNT) break;

    cboard.send({target_in_odom[0], target_in_odom[1], target_in_odom[2], grip, slow});

    tools::draw_text(
      img,
      fmt::format(
        "[go] gripper=({:.3f}, {:.3f}, {:.3f}) target=({:.3f}, {:.3f}, {:.3f})", gripper_in_odom[0],
        gripper_in_odom[1], gripper_in_odom[2], target_in_odom[0], target_in_odom[1],
        target_in_odom[2]),
      {50, 50});

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);
  }
}

void lift(io::USBCamera & cam, io::CBoard & cboard, double z, bool grip, bool slow)
{
  auto t = std::chrono::steady_clock::now();
  Eigen::Vector3d gripper_in_odom = cboard.odom_at(t);
  go(cam, cboard, {gripper_in_odom[0], gripper_in_odom[1], z}, grip, slow);
}

void align_weight(
  io::USBCamera & cam, io::CBoard & cboard, auto_crane::YOLOV8 & yolo, auto_crane::Solver & solver,
  auto_crane::Matcher & matcher, int id1, int id2)
{
  int reach_cnt = 0;

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

      cboard.send({l.in_odom[0], l.in_odom[1], 0, false, false});

      if ((l.in_odom - gripper_in_odom.head<2>()).norm() < EPS)
        reach_cnt++;
      else
        reach_cnt = 0;

      break;
    }

    if (reach_cnt > REACH_CNT) break;

    auto_crane::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);
  }
}

Eigen::Vector2d align_wood(
  io::USBCamera & cam, io::CBoard & cboard, auto_crane::YOLOV8 & yolo, auto_crane::Solver & solver,
  auto_crane::Matcher & matcher, int wood_id)
{
  int reach_cnt = 0;
  bool wood_found = false;
  Eigen::Vector2d wood_in_odom;
  Eigen::Vector2d align_xy;

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
      wood_in_odom[0] = l.in_odom[0];
      wood_in_odom[1] = l.in_odom[1] - 0.04;
      align_xy = wood_in_odom;
      break;
    }

    if (reach_cnt > REACH_CNT) break;

    if (wood_found) cboard.send({wood_in_odom[0], wood_in_odom[1], 0, true, false});

    if (wood_found && (wood_in_odom - gripper_in_odom.head<2>()).norm() < EPS) reach_cnt++;

    auto_crane::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);
  }

  return align_xy;
}

void get(
  io::USBCamera & cam, io::CBoard & cboard, auto_crane::YOLOV8 & yolo, auto_crane::Solver & solver,
  auto_crane::Matcher & matcher, int id1, int id2)
{
  // 去砝码[id1]和砝码[id2]之中点
  Eigen::Vector2d w1 = matcher.weight_in_map(id1);
  Eigen::Vector2d w2 = matcher.weight_in_map(id2);
  Eigen::Vector2d center = (w1 + w2) * 0.5;

  tools::logger()->info("go");
  go(cam, cboard, {center[0], center[1] - 0.05, 0.0}, false, false);

  // 对齐砝码[id1]或砝码[id2]
  tools::logger()->info("align_weight");
  align_weight(cam, cboard, yolo, solver, matcher, id1, id2);

  // 降
  tools::logger()->info("lift down");
  lift(cam, cboard, -0.26, false, false);

  // 取
  tools::logger()->info("grip");
  lift(cam, cboard, -0.26, true, false);
  std::this_thread::sleep_for(500ms);

  // 抬
  tools::logger()->info("lift up");
  lift(cam, cboard, 0.0, true, false);
}

void put(
  io::USBCamera & cam, io::CBoard & cboard, auto_crane::YOLOV8 & yolo, auto_crane::Solver & solver,
  auto_crane::Matcher & matcher, int id)
{
  // 去木桩[id]
  Eigen::Vector2d w = matcher.wood_in_map(id);
  auto y_offset = (id != 2 && id != 3) ? -0.05 : -0.01;
  auto x_offset = (id == 2 || id == 1) ? 0.15 : 0;
  auto z = (id == 4) ? -0.155 : -0.06;

  tools::logger()->info("go");
  go(cam, cboard, {w[0] + x_offset, w[1] + y_offset, 0.0}, true, false);

  // 找木桩[id]
  tools::logger()->info("align_wood");
  Eigen::Vector2d align_xy = align_wood(cam, cboard, yolo, solver, matcher, id);

  tools::logger()->info("go wood");

  // Eigen::Vector3d gripper_in_odom = cboard.odom_at(std::chrono::steady_clock::now());
  go(cam, cboard, {align_xy[0], align_xy[1] + 0.055, 0.0}, true, true);
  // go(cam, cboard, {gripper_in_odom[0], gripper_in_odom[1] + 0.055, 0.0}, true, true);
  // std::this_thread::sleep_for(3000ms);

  // 降
  tools::logger()->info("lift down");
  lift(cam, cboard, z, true, false);

  // 放
  tools::logger()->info("grip");
  lift(cam, cboard, z, false, false);
  std::this_thread::sleep_for(500ms);

  // 抬
  tools::logger()->info("lift up");
  lift(cam, cboard, 0, false, false);

  if(id == 2) {
    go(cam, cboard, {align_xy[0]+0.15, align_xy[1] + 0.055, 0.0}, false, false);
  }

  if(id == 3) {
    go(cam, cboard, {align_xy[0]-0.15, align_xy[1] + 0.055, 0.0}, false, false);
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

  auto_crane::YOLOV8 yolo("assets/openvino_model_v6/best.xml", classes.size(), classes, "AUTO");
  auto_crane::Solver solver(config_path);
  auto_crane::Matcher matcher(config_path);
  auto_crane::Localizer localizer(config_path);

  get(usbcam, cboard, yolo, solver, matcher, 2, 3);
  put(usbcam, cboard, yolo, solver, matcher, 0);

  get(usbcam, cboard, yolo, solver, matcher, 0, 1);
  put(usbcam, cboard, yolo, solver, matcher, 3);

  get(usbcam, cboard, yolo, solver, matcher, 10, 11);
  put(usbcam, cboard, yolo, solver, matcher, 4);

  get(usbcam, cboard, yolo, solver, matcher, 8, 9);
  put(usbcam, cboard, yolo, solver, matcher, 2);

  get(usbcam, cboard, yolo, solver, matcher, 6, 7);
  put(usbcam, cboard, yolo, solver, matcher, 1);

  return 0;
}
