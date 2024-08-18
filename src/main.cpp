#include <opencv2/opencv.hpp>

#include "crane.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>(0);

  Crane crane(config_path);
  auto_crane::Matcher map(config_path);

  Eigen::Vector2d wt2 = map.weight_in_map(2);
  Eigen::Vector2d wt10 = map.weight_in_map(10);

  crane.right_go_to_map(wt10[1], GET_Z);        // no wait
  crane.left_go_to_map(wt2[0], wt2[1], GET_Z);  // no wait

  bool got2 = crane.try_get(2, true);
  bool got10 = crane.try_get(10, false);

  if (!got2) crane.try_get(3, true);
  if (!got10) crane.try_get(11, true);

  Eigen::Vector2d wd0 = map.wood_in_map(0);
  Eigen::Vector2d wd3 = map.wood_in_map(3);

  crane.right_go_to_map(wd3[1], HOLD_Z);         // no wait
  crane.left_go_to_map(wd0[0], wd0[1], HOLD_Z);  // no wait

  crane.put(0, true);
  crane.put(3, true);

  return 0;
}