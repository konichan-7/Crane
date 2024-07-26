#include <Eigen/Dense>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/cboard/cboard.hpp"
#include "io/command.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_crane/decider.hpp"
#include "tasks/auto_crane/localizer.hpp"
#include "tasks/auto_crane/matcher.hpp"
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
  io::Command command;
  tools::Exiter exiter;
  tools::Plotter plotter;
  auto_crane::YOLOV8 yolo("assets/openvino_model_v3/best.xml", classes.size(), classes, "AUTO");
  auto_crane::Decider decider(classes);
  auto_crane::Matcher matcher(config_path);
  auto_crane::Localizer localizer(config_path);

  Eigen::Vector2d t_landmark2cam, t_gripper2odo, t_odo2map, t_target2map;

  auto_crane::Landmark landmark;
  auto_crane::Target target;

  auto sum = 0.0;
  auto count = 0;
  auto sample_count = 1;

  while (!exiter.exit()) {
    cv::Mat img;
    std::vector<auto_crane::Detection> landmarks;

    auto start = std::chrono::steady_clock::now();

    usbcam.read(img, start);

    t_gripper2odo = (cboard.odom_at(start)).head<2>() / 1e3;  //提取xy坐标

    auto detections = yolo.infer(img);

    auto end = std::chrono::steady_clock::now();

    auto span = std::chrono::duration<double>(end - start).count();
    std::cout << "infer + read =: " << span * 1e3 << "ms\n";

    sum += span;
    count += 1;

    landmarks = yolo.filter(detections);

    yolo.save_img(img, landmarks);

    landmark = yolo.pixel2cam(landmarks);

    t_odo2map = matcher.match(landmark, t_gripper2odo, target);

    t_odo2map = localizer.update_coordinate_error(t_odo2map);

    auto_crane::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("press q to quit", img);

    if (auto key = cv::waitKey(1) == 'q') break;
  }

  std::cout << "avg: " << count / sum << "fps\n";

  return 0;
}
