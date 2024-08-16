#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "io/cboard/cboard.hpp"
#include "io/crane/crane.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_crane/matcher.hpp"
#include "tasks/auto_crane/solver.hpp"
#include "tasks/auto_crane/yolov8.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }";

Eigen::Vector2d t_map2odom{0.0, 0.0};

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>(0);

  std::vector<std::string> classes = {"weights", "white", "wood"};

  io::USBCamera cam_left("video0", config_path);
  io::USBCamera cam_right("video2", config_path);
  io::CBoard cboard_left("can0", true);
  io::CBoard cboard_right("can0", false);
  io::Crane crane(cboard_left, cboard_right, config_path);

  auto_crane::YOLOV8 yolo("assets/openvino_model_v6/best.xml", classes.size(), classes, "AUTO");
  auto_crane::Solver solver(config_path);
  auto_crane::Matcher matcher(config_path);

  // 去砝码[id1]和砝码[id2]之中点
  auto id1 = 2, id2 = 3;
  Eigen::Vector2d w1 = matcher.weight_in_map(id1);
  Eigen::Vector2d w2 = matcher.weight_in_map(id2);
  Eigen::Vector2d center = (w1 + w2) * 0.5;
  crane.move_xy(center[0], center[1], true);

  tools::logger()->info("{} {}", center[0], center[1]);

  return 0;
}
