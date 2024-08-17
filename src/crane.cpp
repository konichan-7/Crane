#include "crane.hpp"

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

constexpr int REACH_CNT = 10;
constexpr double EPS = 0.01;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }";

const std::vector<std::string> classes = {"weights", "white", "wood"};

Crane::Crane(const std::string & config_path)
: left_cboard_(config_path, true),
  right_cboard_(config_path, false),
  left_cam_("video0", config_path),
  right_cam_("video2", config_path),
  yolo_("assets/openvino_model_v6/best.xml", classes.size(), classes, "AUTO"),
  solver_(config_path),
  matcher_(config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  y_left_in_odom_ = yaml["y_left_in_odom"].as<double>();
  y_right_in_odom_ = yaml["y_right_in_odom"].as<double>();
}

void Crane::get(int id1, int id2, bool left)
{
  auto & last_cmd = left ? left_last_cmd_ : right_last_cmd_;

  Eigen::Vector2d w1 = matcher_.weight_in_map(id1);
  Eigen::Vector2d w2 = matcher_.weight_in_map(id2);
  Eigen::Vector2d center = (w1 + w2) * 0.5;

  this->go({center[0], center[1], last_cmd.z}, left);
  this->align_weight(id1, id2, left);
  this->go({last_cmd.x, last_cmd.y, -0.26}, left);
}

void Crane::put(int id, bool left) {}

void Crane::read(cv::Mat & img, std::chrono::steady_clock::time_point & t, bool left)
{
  if (left)
    left_cam_.read(img, t);
  else
    right_cam_.read(img, t);
}

Eigen::Vector3d Crane::odom_at(std::chrono::steady_clock::time_point t, bool left)
{
  Eigen::Vector3d left_gripper = left_cboard_.odom_at(t);
  Eigen::Vector3d right_gripper = right_cboard_.odom_at(t);

  // clang-format off
  Eigen::Vector3d gripper_in_odom{
    left_gripper[0], 
    left ? left_gripper[1] : right_gripper[1],
    left ? left_gripper[2] : right_gripper[2]
  };
  // clang-format on

  gripper_in_odom[1] += left ? y_left_in_odom_ : y_right_in_odom_;

  return gripper_in_odom;
}

void Crane::cmd(io::Command command, bool left)
{
  if (left) {
    left_cboard_.send(command);
    left_last_cmd_ = command;
    return;
  }

  auto left_cmd = left_last_cmd_;
  left_cmd.x = command.x;
  left_cboard_.send(left_cmd);
  left_last_cmd_ = left_cmd;

  right_cboard_.send(command);
  right_last_cmd_ = command;
}

void Crane::cmd(Eigen::Vector3d target_in_odom, bool left)
{
  auto command = left ? left_last_cmd_ : right_last_cmd_;
  command.x = target_in_odom[0];
  command.y = target_in_odom[1] - (left ? y_left_in_odom_ : y_right_in_odom_);
  command.z = target_in_odom[2];
  this->cmd(command, left);
}

void Crane::go(Eigen::Vector3d target_in_odom, bool left)
{
  int reach_cnt = 0;

  while (true) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    this->read(img, t, left);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);

    Eigen::Vector3d gripper_in_odom = this->odom_at(t, left);

    this->cmd(target_in_odom, left);
    if ((gripper_in_odom - target_in_odom).norm() < EPS)
      reach_cnt++;
    else
      reach_cnt = 0;

    if (reach_cnt > REACH_CNT) break;
  }
}

void Crane::align_weight(int id1, int id2, bool left)
{
  auto & last_cmd = left ? left_last_cmd_ : right_last_cmd_;
  int reach_cnt = 0;

  while (true) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    this->read(img, t, left);

    auto detections = yolo_.infer(img);
    yolo_.save_img(img, detections);
    auto_crane::draw_detections(img, detections, classes);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);

    Eigen::Vector3d gripper_in_odom = this->odom_at(t, left);
    Eigen::Vector2d t_gripper2odom = gripper_in_odom.head<2>();

    auto landmarks = solver_.solve(detections, t_gripper2odom, left);
    matcher_.match(landmarks, -t_map2odom_);

    bool found = false;
    Eigen::Vector3d target_in_odom;

    for (const auto & l : landmarks) {
      if (l.name != auto_crane::LandmarkName::WEIGHT) continue;
      if (l.id != id1 && l.id != id2) continue;
      found = true;
      target_in_odom = {l.in_odom[0], l.in_odom[1], last_cmd.z};
      break;
    }

    if (!found) {
      tools::logger()->warn("[Crane] weight not found!");
      continue;
    }

    this->cmd(target_in_odom, left);
    if ((gripper_in_odom - target_in_odom).norm() < EPS)
      reach_cnt++;
    else
      reach_cnt = 0;

    if (reach_cnt > REACH_CNT) break;
  }
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>(0);
  Crane crane(config_path);

  crane.get(2, 3, true);
  crane.get(10, 11, false);
  crane.put(0, true);
  crane.put(3, false);

  crane.get(0, 1, true);
  crane.put(4, true);

  crane.get(4, 5, true);
  crane.get(8, 9, false);
  crane.put(1, true);
  crane.put(2, false);

  return 0;
}