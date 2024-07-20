#include "io/usbcamera/usbcamera.hpp"

#include <opencv2/opencv.hpp>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{name n         |video0| 端口名称 }"
  "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>(0);

  auto device_name = cli.get<std::string>("name");

  tools::Exiter exiter;
  io::USBCamera usbcam(device_name, config_path);

  for (int frame_count = 0; !exiter.exit(); frame_count++) {
    cv::Mat img = usbcam.read();
    cv::imshow("USBCamera", img);
    char key = cv::waitKey(1);
    if (key == 'q') break;
  }

  cv::destroyAllWindows();

  return 0;
}
