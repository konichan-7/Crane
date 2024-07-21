#include <chrono>
#include <opencv2/opencv.hpp>

#include "tasks/yolo/yolov8.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tools/exiter.hpp"
#include "tools/plotter.hpp"
#include "tools/logger.hpp"

const std::string keys =
    "{help h usage ? |                        | 输出命令行参数说明 }"
    "{name n         |video0| 端口名称 }"
    "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }"
    "{output-folder o | negative/   | 输出文件夹路径   }";

int main(int argc, char *argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help"))
  {
    cli.printMessage();
    return 0;
  }

  auto device_name = cli.get<std::string>("name");
  auto output_folder = cli.get<std::string>("output-folder");
  auto config_path = cli.get<std::string>(0);

  io::USBCamera usbcam(device_name, config_path);

  std::vector<std::string> classes = {"weights", "wood"};

  tasks::YOLOV8 yolo("assets/openvino_model/best.xml", classes.size(), "AUTO");

  auto sum = 0.0;
  auto count = 0;
  auto sample_count = 1;

  while (!exiter.exit())
  {
    cv::Mat img;
    img = usbcam.read();
    if (img.empty())
    {
      tools::logger()->warn("failed to read img from camera!");
      break;
    }

    auto start = std::chrono::steady_clock::now();

    auto detections = yolo.infer(img);

    auto end = std::chrono::steady_clock::now();
    auto span = std::chrono::duration<double>(end - start).count();
    std::cout << "total: " << span * 1e3 << "ms\n";

    sum += span;
    count += 1;

    auto key = cv::waitKey(1);
    if (key == 's')
    {
      auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
      cv::imwrite(img_path, img);
      tools::logger()->info("negative samples [{}] Saved in {}", sample_count, output_folder);
      ++sample_count;
    }

    tasks::draw_detections(img, detections, classes);

    cv::imshow("press q to quit", img);

    if (key == 'q')
      break;
  }

  std::cout << "avg: " << count / sum << "fps\n";

  return 0;
}