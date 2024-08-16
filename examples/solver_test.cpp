#include "tasks/auto_crane/solver.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/cboard/cboard.hpp"
#include "io/crane/crane.hpp"
#include "io/usbcamera/usbcamera.hpp"
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
  io::Crane crane(cboard_left, cboard_right);

  tools::Exiter exiter;
  tools::Plotter plotter;

  auto_crane::YOLOV8 yolo("assets/openvino_model_v6/best.xml", classes.size(), classes, "AUTO");
  auto_crane::Solver solver(config_path);

  while (!exiter.exit()) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    usbcam.read(img, t);
    Eigen::Vector3d gripper_in_odom_left = cboard_left.odom_at(t);
    Eigen::Vector3d gripper_in_odom_right = cboard_right.odom_at(t);

    // clang-format off
    Eigen::Vector3d gripper_in_odom{
      gripper_in_odom_left[0], 
      left ? gripper_in_odom_left[1] : gripper_in_odom_right[1],
      left ? gripper_in_odom_left[2] : gripper_in_odom_right[2]
    };
    // clang-format on

    Eigen::Vector2d t_gripper2odom = gripper_in_odom.head<2>();

    auto detections = yolo.infer(img);
    yolo.save_img(img, detections);

    auto landmarks = solver.solve(detections, t_gripper2odom, left);

    Eigen::Vector2d target_in_odom = gripper_in_odom.head<2>();

    for (const auto & landmark : landmarks) {
      if (landmark.name != auto_crane::LandmarkName::WEIGHT) continue;

      target_in_odom = landmark.in_odom;
      break;
    }

    crane.cmd({target_in_odom[0], target_in_odom[1], gripper_in_odom_left[2], false}, left);

    // -------------------- 调试输出 --------------------

    nlohmann::json data;
    data["x_gripper_in_odom"] = gripper_in_odom[0];
    data["y_gripper_in_odom"] = gripper_in_odom[1];
    data["z_gripper_in_odom"] = gripper_in_odom[2];
    data["x_target_in_odom"] = target_in_odom[0];
    data["y_target_in_odom"] = target_in_odom[1];
    plotter.plot(data);

    auto_crane::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("press q to quit", img);
    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}
