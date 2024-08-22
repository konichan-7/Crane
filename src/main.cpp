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

  crane.wait_to_start();

  // -------------------- front --------------------

  crane.forward(auto_crane::WEIGHT, 2, GET_Z, true);
  crane.forward(auto_crane::WEIGHT, 10, GET_Z, false);

  bool got2 = crane.try_get(2, true);
  bool got10 = crane.try_get(10, false);

  if (!got2) crane.forward(auto_crane::WEIGHT, 3, GET_Z, true);
  if (!got10) crane.forward(auto_crane::WEIGHT, 11, GET_Z, false);

  if (!got2) crane.try_get(3, true);
  if (!got10) crane.try_get(11, false);

  crane.forward(auto_crane::TALL_WOOD, 0, HOLD_Z, true);
  crane.forward(auto_crane::TALL_WOOD, 3, HOLD_Z, false);

  // crane.put(0, true);
  // crane.put(3, false);
  crane.puts(0, 3);

  // -------------------- middle --------------------

  crane.forward(auto_crane::WEIGHT, 0, GET_Z, true);  // 先去[0], 防止撞到轴

  bool got0 = crane.try_get(0, true);
  if (!got0) crane.forward(auto_crane::WEIGHT, 1, GET_Z, true);
  if (!got0) crane.try_get(1, true);

  crane.forward(auto_crane::SHORT_WOOD, 4, HOLD_Z, true);
  crane.put(4, true);

  // -------------------- back --------------------

  crane.forward(auto_crane::WEIGHT, 4, GET_Z, true);
  crane.forward(auto_crane::WEIGHT, 8, GET_Z, false);

  bool got4 = crane.try_get(4, true);
  bool got8 = crane.try_get(8, false);

  if (!got4) crane.forward(auto_crane::WEIGHT, 5, GET_Z, true);
  if (!got8) crane.forward(auto_crane::WEIGHT, 9, GET_Z, false);

  if (!got4) crane.try_get(5, true);
  if (!got8) crane.try_get(9, false);

  crane.forward(auto_crane::TALL_WOOD, 1, HOLD_Z, true);
  crane.forward(auto_crane::TALL_WOOD, 2, HOLD_Z, false);

  // crane.put(1, true);
  // crane.put(2, false);
  crane.puts(1, 2);

  return 0;
}