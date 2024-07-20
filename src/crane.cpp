#include <fmt/core.h>

#include <chrono>
#include <future>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/usbcamera/usbcamera.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono;

std::string rec_data_async(io::Client &client) { return client.recdata(); }

const std::string keys =
    "{help h usage ? |      | 输出命令行参数说明}"
    "{can c          | can0 | can端口名称     }"
    "{debug d        |      | 输出调试画面和信息 }"
    "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }";

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
  auto debug = cli.has("debug");
  auto can = cli.get<std::string>("can");
  auto config_path = cli.get<std::string>(0);

  io::CBoard cboard(can);
  io::Camera camera(config_path);
  io::Client client;
  io::USBCamera usbcam1("video0", config_path);
  io::USBCamera usbcam2("video2", config_path);
  io::USBCamera usbcam3("video4", config_path);

  auto_aim::Detector detector(config_path, debug);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  perception::Decider decider(config_path);

  cv::Mat img;
  cv::Mat usb_img1, usb_img2, usb_img3;
  int state;
  const int mode = 1;
  Eigen::Vector2d delta_angle;
  Eigen::Vector3d gimbal_pos;
  std::string send_msg;
  std::string armor_omit = "0,";
  // 创建一个 std::future 对象，用于在另一个线程中异步获取 client.recdata() 的返回值
  std::future<std::string> rec_data =
      std::async(std::launch::async, rec_data_async, std::ref(client));

  std::chrono::steady_clock::time_point timestamp;

  std::chrono::steady_clock::time_point perception_start, detect_begin, detect_end, perception_end,
      record_start;
  int i = 0;
  for (int frame_count = 0; !exiter.exit(); frame_count++)
  {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
    // recorder.record(img, q, timestamp);

    /// 自瞄核心逻辑

    solver.set_R_gimbal2world(q);

    gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img, frame_count);

    decider.armor_filter(armors, armor_omit);

    decider.set_priority(armors, mode);

    auto targets = tracker.track(armors, timestamp);

    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑

    if (tracker.state() == "lost")
    {
      perception_start = std::chrono::steady_clock::now();
      usb_img1 = usbcam1.read();
      record_start = std::chrono::steady_clock::now();
      detect_begin = std::chrono::steady_clock::now();
      auto armors_left = detector.detect(usb_img1, frame_count);
      detect_end = std::chrono::steady_clock::now();
      auto left_empty = decider.armor_filter(armors_left, armor_omit);
      if (!left_empty)
      {
        // left_recorder.record(usb_img1, timestamp);
        delta_angle = decider.delta_angle(armors_left, usbcam1.device_name);
        tools::logger()->debug(
            "delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}", delta_angle[0],
            delta_angle[1], armors_left.size(), auto_aim::ARMOR_NAMES[armors_left.front().name]);
        command = {
            true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
            tools::limit_rad(delta_angle[1] / 57.3)};
      }
      else
      {
        usb_img2 = usbcam2.read();
        auto armors_right = detector.detect(usb_img2, frame_count);
        auto right_empty = decider.armor_filter(armors_right, armor_omit);
        if (!right_empty)
        {
          // right_recorder.record(usb_img2, timestamp);
          delta_angle = decider.delta_angle(armors_right, usbcam2.device_name);
          tools::logger()->debug(
              "delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}", delta_angle[0],
              delta_angle[1], armors_right.size(), auto_aim::ARMOR_NAMES[armors_right.front().name]);
          command = {
              true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
              tools::limit_rad(delta_angle[1] / 57.3)};
        }
        else
        {
          usb_img3 = usbcam3.read();
          auto armors_back = detector.detect(usb_img3, frame_count);
          auto back_empty = decider.armor_filter(armors_back, armor_omit);
          if (!back_empty)
          {
            // back_recorder.record(usb_img3, timestamp);
            delta_angle = decider.delta_angle(armors_back, usbcam3.device_name);
            tools::logger()->debug(
                "delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}", delta_angle[0],
                delta_angle[1], armors_back.size(), auto_aim::ARMOR_NAMES[armors_back.front().name]);
            command = {
                true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
                tools::limit_rad(delta_angle[1] / 57.3)};
          }
        }
      }
      perception_end = std::chrono::steady_clock::now();
    }
    else
    {
      command = aimer.aim(targets, timestamp, cboard.bullet_speed);
    }

    if (command.control && tracker.state() == "tracking")
      command.shoot = true;
    cboard.send(command);

    /// tcp导航通信部分

    if (client.con_state == 0)
    {
      client.setdata("0,0,0,0");
      client.senddata();
      // tools::logger()->info("client send invalid data success");
    }
    else
      client.try_connect();

    // 检查 rec_data 是否已经准备好了
    if (
        client.con_state == 0 &&
        rec_data.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      // 如果准备好了，获取数据
      armor_omit = rec_data.get();
      if (armor_omit != "0,")
        tools::logger()->debug(fmt::format("recieve valid data from server:{}", armor_omit));
      // std::this_thread::sleep_for(std::chrono::milliseconds(250));
      rec_data = std::async(std::launch::async, rec_data_async, std::ref(client));
    }
    else
      memset(client.rec_buf, 0, sizeof(client.rec_buf));

    /// 调试输出
    if (!debug)
      continue;

    tools::logger()->info(
        "detect time:{:.2f}ms", abs(tools::delta_time(detect_begin, detect_end) * 1e3));
    tools::logger()->info(
        "perception time:{:.2f}ms", abs(tools::delta_time(perception_start, perception_end) * 1e3));
    tools::logger()->info(
        "record time:{:.2f}ms", abs(tools::delta_time(detect_begin, record_start) * 1e3));

    tools::draw_text(
        img, fmt::format("[{}] [{}]", frame_count, tracker.state()), {10, 30}, {255, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty())
    {
      const auto &armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
    }

    if (tracker.state() != "lost")
    {
      auto target = targets.front();
      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d &xyza : armor_xyza_list)
      {
        auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
          solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["a"] = x[5] * 57.3;
      data["w"] = x[6];
      data["r"] = x[7];
      data["l"] = x[8];
      data["h"] = x[9];
      data["last_id"] = target.last_id;
    }

    // 云台响应情况
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = -ypr[1] * 57.3;

    if (command.control)
    {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["delta_yaw"] = delta_angle[0];
      data["delta_pitch"] = delta_angle[1];
    }

    plotter.plot(data);
    for (auto &armor : armors)
    {
      tools::draw_points(img, armor.points);
      solver.solve(armor);
      auto xyz = armor.xyz_in_world;
      auto ypr = armor.ypr_in_world * 57.3;

      tools::draw_text(
          img, fmt::format("x{:.2f} y{:.2f} z{:.2f}", xyz[0], xyz[1], xyz[2]), {30, 60});
      tools::draw_text(
          img, fmt::format("r{:.1f} p{:.1f} y{:.1f}", ypr[0], ypr[1], ypr[2]), {30, 100});
    }
    cv::resize(img, img, {}, 0.5, 0.5); // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);

    auto key = cv::waitKey(33);
    if (key == 'q')
      break;
  }

  return 0;
}