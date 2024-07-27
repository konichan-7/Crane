#include "tasks/auto_crane/localizer.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/cboard/cboard.hpp"
#include "io/command.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_crane/decider.hpp"
#include "tasks/auto_crane/matcher.hpp"
#include "tasks/auto_crane/solver.hpp"
#include "tasks/auto_crane/yolov8.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{name n         |video0| 端口名称 }"
  "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }"
  "{output-folder o | negative/   | 输出文件夹路径   }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto device_name = cli.get<std::string>("name");
  auto output_folder = cli.get<std::string>("output-folder");
  auto config_path = cli.get<std::string>(0);
  std::vector<std::string> classes = {"weights", "wood"};

  io::USBCamera usbcam(device_name, config_path);
  io::CBoard cboard("can0");

  tools::Exiter exiter;
  tools::Plotter plotter;

  auto_crane::YOLOV8 yolo("assets/openvino_model_v3/best.xml", classes.size(), classes, "AUTO");
  auto_crane::Decider decider(config_path);
  auto_crane::Solver solver(config_path);
  auto_crane::Matcher matcher(config_path);
  auto_crane::Localizer localizer(config_path);

  while (!exiter.exit()) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    usbcam.read(img, t);
    Eigen::Vector3d odom = cboard.odom_at(t);

    auto detections = yolo.infer(img);
    yolo.save_img(img, detections);

    auto landmarks = solver.solve(detections);

    std::vector<auto_crane::Target> targets;
    Eigen::Vector2d t_odo2map;
    matcher.match(landmarks, odom.head<2>(), targets, t_odo2map);

    // -------------------- 调试输出 --------------------

    nlohmann::json data;
    data["gripper_x_in_odom"] = odom[0];
    data["gripper_y_in_odom"] = odom[1];
    data["gripper_z_in_odom"] = odom[2];

    // if (landmark.name != auto_crane::LandmarkName::INVALID) {
    //   data["landmark_x_in_cam"] = landmark.t_landmark2cam[0];
    //   data["landmark_y_in_cam"] = landmark.t_landmark2cam[1];
    // }
    plotter.plot(data);

    auto_crane::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("press q to quit", img);
    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}
