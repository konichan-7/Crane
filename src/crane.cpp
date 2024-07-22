#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/yolo/yolov8.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{name n         |video0| 端口名称 }"
  "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }"
  "{output-folder o | negative/   | 输出文件夹路径   }";

static io::Command filter(const std::vector<yolo::Detection> & detections);

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
  yolo::YOLOV8 yolo("assets/openvino_model_v1/best.xml", classes.size(), "AUTO");

  auto sum = 0.0;
  auto count = 0;
  auto sample_count = 1;

  while (!exiter.exit()) {
    cv::Mat img;
    img = usbcam.read();

    auto start = std::chrono::steady_clock::now();

    auto detections = yolo.infer(img);

    auto end = std::chrono::steady_clock::now();
    auto span = std::chrono::duration<double>(end - start).count();
    std::cout << "total: " << span * 1e3 << "ms\n";

    sum += span;
    count += 1;

    command = filter(detections);
    cboard.send(command);

    auto key = cv::waitKey(1);
    if (key == 's') {
      auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
      cv::imwrite(img_path, img);
      tools::logger()->info("negative samples [{}] Saved in {}", sample_count, output_folder);
      ++sample_count;
    }

    yolo::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("press q to quit", img);

    if (key == 'q') break;
  }

  std::cout << "avg: " << count / sum << "fps\n";

  return 0;
}

static io::Command filter(const std::vector<yolo::Detection> & detections)
{
  if (detections.size() == 0) {
    io::Command command{1, 0, 0};
    return command;
  }

  yolo::Detection target;
  int min_distance = 0;

  for (auto d : detections) {
    if (
      ((d.center.x - 960) * (d.center.x - 960) + (d.center.y - 540) * (d.center.y - 540)) >
      min_distance) {
      target = d;
      min_distance =
        ((d.center.x - 960) * (d.center.x - 960) + (d.center.y - 540) * (d.center.y - 540));
    }
  }
  io::Command command = {1, target.center.x - 960, target.center.y - 540};
  tools::logger()->debug("send data:x:{},y:{}", target.center.x - 960, target.center.y - 540);
  return command;
}