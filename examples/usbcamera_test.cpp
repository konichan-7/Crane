#include "io/usbcamera/usbcamera.hpp"

#include <opencv2/opencv.hpp>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{name n         |video0| 端口名称 }"
  "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>(0);
  auto device_name = cli.get<std::string>("name");
  auto display = cli.has("display");

  tools::Exiter exiter;
  io::USBCamera usbcam(device_name, config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_stamp = std::chrono::steady_clock::now();
  while (!exiter.exit()) {
    usbcam.read(img, timestamp);

    auto dt = tools::delta_time(timestamp, last_stamp);
    last_stamp = timestamp;

    tools::logger()->info("{:.2f} fps", 1 / dt);

    if (!display) continue;
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') break;
  }

  cv::destroyAllWindows();

  return 0;
}
