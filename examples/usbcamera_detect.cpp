#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/classifier.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/perception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{name n         | left | 端口名称 }"
  "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto device_name = cli.get<std::string>("name");
  auto config_path = cli.get<std::string>(0);

  io::USBCamera usbcam(device_name, config_path);

  auto_aim::Detector detector(config_path);
  perception::Decider decider(config_path);

  cv::Mat img;
  Eigen::Vector2d delta_anlge;  //degree
  int state, mode = 1;
  std::list<auto_aim::Armor> armors;

  for (int frame_count = 0; !exiter.exit(); frame_count++) {
    auto perception_start = std::chrono::steady_clock::now();
    img = usbcam.read();
    auto detect_begin = std::chrono::steady_clock::now();
    auto armors = detector.detect(img, frame_count);
    auto detect_end = std::chrono::steady_clock::now();
    auto empty = decider.armor_filter(armors);
    // if(empty) continue;
    decider.set_priority(armors, 1);
    delta_anlge = decider.delta_angle(armors, device_name);
    auto perception_end = std::chrono::steady_clock::now();
    tools::logger()->info(
      "single camera detect time:{:.1f}ms", abs(tools::delta_time(detect_begin, detect_end) * 1e3));
    tools::logger()->info(
      "perception time:{:.1f}ms", abs(tools::delta_time(perception_start, perception_end) * 1e3));
    tools::logger()->info(
      "read time:{:.2f}ms", abs(tools::delta_time(perception_start, detect_begin) * 1e3));

    tools::draw_text(
      img,
      fmt::format(
        "[1 cam detect time:{:.2f}ms]", abs(tools::delta_time(detect_begin, detect_end) * 1e3)),
      {10, 30}, {255, 255, 255});

    nlohmann::json data;

    // 云台响应情况
    data["delta_yaw"] = delta_anlge[0];
    data["pixel x:"] = armors.front().center.x;
    armors.clear();

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);

    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  return 0;
}
