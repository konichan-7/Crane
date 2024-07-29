#include <Eigen/Dense>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/cboard/cboard.hpp"
#include "io/command.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_crane/decider.hpp"
#include "tasks/auto_crane/localizer.hpp"
#include "tasks/auto_crane/matcher.hpp"
#include "tasks/auto_crane/solver.hpp"
#include "tasks/auto_crane/yolov8.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ?  |                        | 输出命令行参数说明 }"
  "{name n          |         video0         | 端口名称 }"
  "{@config-path    | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }"
  "{output-folder o | negative/              | 输出文件夹路径   }"
  "{auto a          |         false          | 自动挡}";

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
  auto auto_mode = cli.get<bool>("auto");
  std::vector<std::string> classes = {"weights", "white", "wood"};

  io::USBCamera usbcam(device_name, config_path);
  io::CBoard cboard("can0");

  tools::Exiter exiter;
  tools::Plotter plotter;

  auto_crane::YOLOV8 yolo("assets/openvino_model_v5/best.xml", classes.size(), classes, "AUTO");
  auto_crane::Solver solver(config_path);
  auto_crane::Matcher matcher(config_path);
  auto_crane::Localizer localizer(config_path);
  auto_crane::Decider decider(config_path);

  auto sum = 0.0;
  auto count = 0;
  auto sample_count = 1;

  while (!exiter.exit()) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    Eigen::Vector2d t_gripper2odo, t_odo2map, t_odo2map_update;
    Eigen::Vector3d p_gripper2odo;
    std::vector<auto_crane::Target> targets;
    bool servo_state;

    auto start = std::chrono::steady_clock::now();

    usbcam.read(img, t);

    p_gripper2odo = cboard.odom_at(t);
    servo_state = cboard.servo();
    t_gripper2odo = p_gripper2odo.head<2>();  // 提取xy坐标

    auto detections = yolo.infer(img);

    auto end = std::chrono::steady_clock::now();

    auto span = std::chrono::duration<double>(end - start).count();
    // std::cout << "infer + read =: " << span * 1e3 << "ms\n";

    sum += span;
    count += 1;

    yolo.save_img(img, detections);

    // auto filtered_detections = yolo.filter(detections);

    auto landmarks = solver.solve(detections);

    matcher.match(landmarks, t_gripper2odo, targets, t_odo2map);

    t_odo2map_update = localizer.localize(t_odo2map);

    auto command = decider.decide(p_gripper2odo, t_odo2map_update, targets, servo_state);

    if (command.x != 0)
      tools::logger()->debug(
        "command is:{:.4f},{:.4f},{:.4f},{}.state is {}", command.x, command.y, command.z,
        command.grip, decider.state());

    auto_crane::draw_detections(img, detections, classes);
    tools::draw_text(img, decider.state(), cv::Point(50, 50), {0, 0, 255}, 2);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("press q to quit", img);

    if (cv::waitKey(1) == 'q') break;

    if (t_odo2map_update[0] != 1e6) {
      nlohmann::json data;
      data["x_odo2map_update"] = t_odo2map_update[0];
      data["y_odo2map_update"] = t_odo2map_update[1];
      plotter.plot(data);
    }

    if (!auto_mode) continue;

    if (command.x != 0) cboard.send(command);
  }

  std::cout << "avg: " << count / sum << "fps\n";

  return 0;
}
