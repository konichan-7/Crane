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
: left_cboard_("can0", true),
  right_cboard_("can0", false),
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
  Eigen::Vector2d in_map = (matcher_.weight_in_map(id2) + matcher_.weight_in_map(id1)) * 0.5;
  Eigen::Vector2d in_odom = in_map + t_map2odom_;
  in_odom[1] += left ? -0.15 : 0.15;

  this->go({in_odom[0], in_odom[1], this->last_z(left)}, left);
  this->align(id1, id2, left);

  this->go({this->last_x(left), this->last_y(left), -0.26}, left);
  this->grip(true, left);
  this->go({this->last_x(left), this->last_y(left), -0.005}, left);
}

void Crane::put(int id, bool left)
{
  Eigen::Vector2d in_odom = matcher_.wood_in_map(id) + t_map2odom_;
  in_odom[1] += left ? -0.15 : 0.15;
  auto z = (id == 4) ? -0.155 : -0.06;

  this->go({in_odom[0], in_odom[1], this->last_z(left)}, left);
  this->align(id, -1, left);

  auto & last_cmd = left ? left_last_cmd_ : right_last_cmd_;
  last_cmd.slow = true;
  this->go(
    {this->last_x(left), this->last_y(left) + (left ? 0.055 : -0.055), this->last_z(left)}, left);
  last_cmd.slow = false;

  this->go({this->last_x(left), this->last_y(left), z}, left);
  this->grip(false, left);
  this->go({this->last_x(left), this->last_y(left), -0.005}, left);
}

double Crane::last_x(bool left) const
{
  auto last_cmd = left ? left_last_cmd_ : right_last_cmd_;
  return last_cmd.x;
}

double Crane::last_y(bool left) const
{
  auto last_cmd = left ? left_last_cmd_ : right_last_cmd_;
  last_cmd.y += left ? y_left_in_odom_ : y_right_in_odom_;
  return last_cmd.y;
}

double Crane::last_z(bool left) const
{
  auto last_cmd = left ? left_last_cmd_ : right_last_cmd_;
  return last_cmd.z;
}

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
  tools::logger()->info("[Crane] go {}", left ? "left" : "right");

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

void Crane::align(int id1, int id2, bool left)
{
  tools::logger()->info("[Crane] align {}", left ? "left" : "right");

  auto reach_cnt = 0;
  auto is_wood = (id2 == -1);
  auto name = !is_wood ? auto_crane::LandmarkName::WEIGHT
                       : ((id1 != 4) ? auto_crane::LandmarkName::TALL_WOOD
                                     : auto_crane::LandmarkName::SHORT_WOOD);

  auto found = false;
  auto_crane::Landmark target;

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
    solver_.update_wood(landmarks, t_gripper2odom, left);

    for (const auto & l : landmarks) {
      if (l.name != name) continue;
      if (l.id != id1 && l.id != id2) continue;
      found = true;
      target = l;
      break;
    }

    if (!found) continue;

    Eigen::Vector3d target_in_odom{target.in_odom[0], target.in_odom[1], this->last_z(left)};
    if (is_wood) target_in_odom[1] += left ? -0.04 : 0.04;

    this->cmd(target_in_odom, left);
    if ((gripper_in_odom - target_in_odom).norm() < EPS)
      reach_cnt++;
    else
      reach_cnt = 0;

    if (reach_cnt > REACH_CNT) break;
  }

  // if (!is_wood) t_map2odom_ = target.in_odom - target.in_map;
}

void Crane::grip(bool grip, bool left)
{
  tools::logger()->info("[Crane] grip {}", left ? "left" : "right");

  auto command = left ? left_last_cmd_ : right_last_cmd_;
  command.grip = grip;

  for (int i = 0; i < 20; i++) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    this->read(img, t, left);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);

    this->cmd(command, left);
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

  // crane.get(0, 1, true);
  // crane.put(4, true);

  // crane.get(4, 5, true);
  // crane.get(8, 9, false);
  // crane.put(1, true);
  // crane.put(2, false);

  return 0;
}