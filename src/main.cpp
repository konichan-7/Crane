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

  crane.wait_to_start();

  Eigen::Vector2d wt2 = map.weight_in_map(2);
  Eigen::Vector2d wt3 = map.weight_in_map(3);
  Eigen::Vector2d wt10 = map.weight_in_map(10);
  Eigen::Vector2d wt11 = map.weight_in_map(11);

  crane.right_go_to_map(wt10[1], GET_Z);
  crane.left_go_to_map(wt2[0], wt2[1], GET_Z);

  bool got2 = crane.try_get(2, true);
  bool got10 = crane.try_get(10, false);

  if (!got2) crane.left_go_to_map(wt3[0], wt3[1], GET_Z);
  if (!got10) crane.right_go_to_map(wt11[1], GET_Z);

  if (!got2) crane.try_get(3, true);
  if (!got10) crane.try_get(11, false);

  Eigen::Vector2d wd0 = map.wood_in_map(0);
  Eigen::Vector2d wd3 = map.wood_in_map(3);

  crane.right_go_to_map(wd3[1], HOLD_Z);
  crane.left_go_to_map(wd0[0], wd0[1], HOLD_Z);

  crane.put(0, true);
  crane.put(3, false);

  Eigen::Vector2d wt0 = map.weight_in_map(0);
  Eigen::Vector2d wt1 = map.weight_in_map(1);

  // 先去[0], 防止撞到轴
  crane.left_go_to_map(wt0[0], wt0[1], GET_Z);

  bool got0 = crane.try_get(0, true);
  if (!got0) {
    crane.left_go_to_map(wt1[0], wt1[1], GET_Z);
    crane.try_get(1, true);
  }

  Eigen::Vector2d wd4 = map.wood_in_map(4);
  crane.left_go_to_map(wd4[0], wd4[1], HOLD_Z);
  crane.put(4, true);

  Eigen::Vector2d wt4 = map.weight_in_map(4);
  Eigen::Vector2d wt5 = map.weight_in_map(5);
  Eigen::Vector2d wt8 = map.weight_in_map(8);
  Eigen::Vector2d wt9 = map.weight_in_map(9);

  crane.right_go_to_map(wt8[1], GET_Z);
  crane.left_go_to_map(wt4[0], wt4[1], GET_Z);

  bool got4 = crane.try_get(4, true);
  bool got8 = crane.try_get(8, false);

  if (!got4) crane.left_go_to_map(wt5[0], wt5[1], GET_Z);
  if (!got8) crane.right_go_to_map(wt9[1], GET_Z);

  if (!got4) crane.try_get(5, true);
  if (!got8) crane.try_get(9, false);

  Eigen::Vector2d wd1 = map.wood_in_map(1);
  Eigen::Vector2d wd2 = map.wood_in_map(2);

  crane.right_go_to_map(wd2[1], HOLD_Z);
  crane.left_go_to_map(wd1[0], wd1[1], HOLD_Z);

  crane.put(1, true);
  crane.put(2, false);

  return 0;
}