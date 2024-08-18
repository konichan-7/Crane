#include "crane.hpp"

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

constexpr int REACH_CNT = 10;
constexpr double EPS = 0.01;

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
  auto y_left_odom_in_map = yaml["y_left_odom_in_map"].as<double>();
  auto y_right_odom_in_map = yaml["y_right_odom_in_map"].as<double>();
  t_map_to_left_odom_ = {0.0, -y_left_odom_in_map};
  t_map_to_right_odom_ = {0.0, -y_right_odom_in_map};
}

void Crane::right_go_to_map(double y, double z, bool wait)
{
  auto right_cmd = right_last_cmd_;
  right_cmd.y = y + t_map_to_right_odom_[1];
  right_cmd.z = z;

  if (wait)
    this->go({this->last_x(), right_cmd.y, right_cmd.z}, false);
  else
    this->cmd(right_cmd, false);
}

void Crane::left_go_to_map(double x, double y, double z, bool wait)
{
  auto left_cmd = left_last_cmd_;
  left_cmd.x = x + t_map_to_left_odom_[0];
  left_cmd.y = y + t_map_to_left_odom_[1];
  left_cmd.z = z;

  if (wait)
    this->go({left_cmd.x, left_cmd.y, left_cmd.z}, true);
  else
    this->cmd(left_cmd, true);
}

bool Crane::try_get(int id, bool left)
{
  if (this->find_white(id, left)) return false;

  this->align(auto_crane::WEIGHT, id, left);
  this->grip(true, left);

  if (left)
    this->left_go_to_map(this->last_x(), this->last_y(left), HOLD_Z, false);
  else
    this->right_go_to_map(this->last_y(left), HOLD_Z, false);

  return true;
}

void Crane::put(int id, bool left)
{
  // Eigen::Vector2d & t_map2odom = left ? t_map_to_left_odom_ : t_map_to_right_odom_;

  // Eigen::Vector2d in_odom = matcher_.wood_in_map(id) + t_map2odom;
  // in_odom[1] += left ? -0.15 : 0.15;
  // auto z = (id == 4) ? -0.155 : -0.06;

  // this->go({in_odom[0], in_odom[1], this->last_z(left)}, left);
  // this->align(id, -1, left);

  // auto & last_cmd = left ? left_last_cmd_ : right_last_cmd_;
  // last_cmd.slow = true;
  // this->go(
  //   {this->last_x(left), this->last_y(left) + (left ? 0.055 : -0.055), this->last_z(left)}, left);
  // last_cmd.slow = false;

  // this->go({this->last_x(left), this->last_y(left), z}, left);
  // this->grip(false, left);
  // this->go({this->last_x(left), this->last_y(left), -0.005}, left);
}

double Crane::last_x() const { return left_last_cmd_.x; }

double Crane::last_y(bool left) const { return left ? left_last_cmd_.y : right_last_cmd_.y; }

double Crane::last_z(bool left) const { return left ? left_last_cmd_.z : right_last_cmd_.z; }

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

  return gripper_in_odom;
}

void Crane::cmd(io::Command command, bool left)
{
  if (left) {
    left_cboard_.send(command);
    left_last_cmd_ = command;
  }
  else {
    right_cboard_.send(command);
    right_last_cmd_ = command;
  }
}

void Crane::cmd(Eigen::Vector3d target_in_odom, bool left)
{
  auto left_cmd = left_last_cmd_;

  if (left) {
    left_cmd.y = target_in_odom[1];
    left_cmd.z = target_in_odom[2];
  }
  else {
    auto right_cmd = right_last_cmd_;
    right_cmd.y = target_in_odom[1];
    right_cmd.z = target_in_odom[2];
    this->cmd(right_cmd, false);
  }

  left_cmd.x = target_in_odom[0];
  this->cmd(left_cmd, true);
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

bool Crane::find_white(int id, bool left)
{
  while (true) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    this->read(img, t, left);

    Eigen::Vector3d cam_in_odom = this->odom_at(t, left);
    Eigen::Vector2d t_cam2odom = cam_in_odom.head<2>();

    auto detections = yolo_.infer(img);
    auto landmarks = solver_.solve(detections, t_cam2odom, left);
    matcher_.match(landmarks, -(left ? t_map_to_left_odom_ : t_map_to_right_odom_));

    auto found_white = false;
    auto found_weight = false;

    for (const auto & l : landmarks) {
      if (l.name == auto_crane::WHITE && l.id == id) {
        found_white = true;
        break;
      }
      if (l.name == auto_crane::WEIGHT && l.id == id) {
        found_weight = true;
        break;
      }
    }

    if (found_white || found_weight) return found_white;

    auto_crane::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);
  }
}

void Crane::align(auto_crane::LandmarkName name, int id, bool left)
{
  auto is_wood = (name == auto_crane::TALL_WOOD || name == auto_crane::SHORT_WOOD);
  Eigen::Vector2d & t_map2odom = left ? t_map_to_left_odom_ : t_map_to_right_odom_;

  auto reach_cnt = 0;

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
    matcher_.match(landmarks, -t_map2odom);
    solver_.update_wood(landmarks, t_gripper2odom, left);

    for (const auto & l : landmarks) {
      if (l.name != name && l.id != id) continue;
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

  // if (!is_wood) t_map2odom = target.in_odom - target.in_map;
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
