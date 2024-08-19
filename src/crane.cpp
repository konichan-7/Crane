#include "crane.hpp"

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

constexpr int REACH_CNT = 10;
constexpr int FOUND_CNT = 3;
constexpr double EPS = 0.01;

const std::vector<std::string> classes = {"weights", "white", "wood"};

Crane::Crane(const std::string & config_path)
: left_cboard_("can0", true),
  right_cboard_("can0", false),
  left_cam_("video0", config_path),
  right_cam_("video2", config_path),
  yolo_("assets/int8/quantized_model.xml", classes.size(), classes, "AUTO"),
  solver_(config_path),
  matcher_(config_path)
{
  auto yaml = YAML::LoadFile(config_path);

  auto y_left_odom_in_map = yaml["y_left_odom_in_map"].as<double>();
  auto y_right_odom_in_map = yaml["y_right_odom_in_map"].as<double>();
  t_map_to_left_odom_ = {0.0, -y_left_odom_in_map};
  t_map_to_right_odom_ = {0.0, -y_right_odom_in_map};

  x_left_gripper_offset_ = yaml["x_left_gripper_offset"].as<double>();
  x_right_gripper_offset_ = yaml["x_right_gripper_offset"].as<double>();

  y_left_get_offset_ = yaml["y_left_get_offset"].as<double>();
  y_right_get_offset_ = yaml["y_right_get_offset"].as<double>();

  y_left_put_offset_ = yaml["y_left_put_offset"].as<double>();
  y_right_put_offset_ = yaml["y_right_put_offset"].as<double>();
}

void Crane::wait_to_start()
{
  tools::logger()->info("[Crane] waiting to start...");
  while (!left_cboard_.start) std::this_thread::sleep_for(10ms);
}

void Crane::forward(auto_crane::LandmarkName name, int id, double z, bool left)
{
  auto is_weight = (name == auto_crane::WEIGHT);

  Eigen::Vector2d m = is_weight ? matcher_.weight_in_map(id) : matcher_.wood_in_map(id);
  Eigen::Vector2d o = m + (left ? t_map_to_left_odom_ : t_map_to_right_odom_);

  auto command = left ? left_last_cmd_ : right_last_cmd_;
  command.x = o[0];
  command.y = o[1];
  command.z = z;

  this->cmd(command, left);
}

bool Crane::try_get(int id, bool left)
{
  if (this->find_white(id, left)) {
    tools::logger()->info("[Crane] try_get {} not found", id);
    return false;
  }
  tools::logger()->info("[Crane] try_get {} found", id);

  this->align(auto_crane::WEIGHT, id, left);
  this->grip(true, left);
  this->go_no_wait({this->last_x(), this->last_y(left), HOLD_Z}, left);

  return true;
}

void Crane::put(int id, bool left)
{
  this->align((id != 4) ? auto_crane::TALL_WOOD : auto_crane::SHORT_WOOD, id, left);

  auto & last_cmd = left ? left_last_cmd_ : right_last_cmd_;
  last_cmd.slow = true;
  this->go(
    {this->last_x(), this->last_y(left) + (left ? y_left_put_offset_ : y_right_put_offset_),
     HOLD_Z},
    left);
  last_cmd.slow = false;

  this->go({this->last_x(), this->last_y(left), (id == 4) ? PUT_L_Z : PUT_H_Z}, left);
  this->grip(false, left);
  this->go({this->last_x(), this->last_y(left), HOLD_Z}, left);
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

void Crane::go_no_wait(Eigen::Vector3d target_in_odom, bool left)
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
  tools::logger()->info("[Crane] go {}", left ? "left" : "right");

  int reach_cnt = 0;

  while (true) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    this->read(img, t, left);

    Eigen::Vector3d gripper_in_odom = this->odom_at(t, left);

    this->go_no_wait(target_in_odom, left);
    if ((gripper_in_odom - target_in_odom).norm() < EPS)
      reach_cnt++;
    else
      reach_cnt = 0;

    if (reach_cnt > REACH_CNT) break;

    // clang-format off
    tools::draw_text(
      img,
      fmt::format(
        "({:.3f}, {:.3f}, {:.3f}) -> ({:.3f}, {:.3f}, {:.3f})",
        gripper_in_odom[0],
        gripper_in_odom[1],
        gripper_in_odom[2],
        target_in_odom[0],
        target_in_odom[1],
        target_in_odom[2]
      ),
      {100, 100}
    );
    // clang-format on

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);
  }
}

bool Crane::find_white(int id, bool left)
{
  tools::logger()->info("[Crane] find_white {} {}", id, left ? "left" : "right");

  auto found_white_cnt = 0;
  auto found_weight_cnt = 0;

  while (true) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    this->read(img, t, left);

    Eigen::Vector3d cam_in_odom = this->odom_at(t, left);
    Eigen::Vector2d t_cam2odom = cam_in_odom.head<2>();

    auto detections = yolo_.infer(img);
    auto landmarks = solver_.solve(detections, t_cam2odom, left);
    matcher_.match(landmarks, -(left ? t_map_to_left_odom_ : t_map_to_right_odom_));

    for (const auto & l : landmarks) {
      if (l.name == auto_crane::WHITE && l.id == id) {
        found_white_cnt++;
        break;
      }
      if (l.name == auto_crane::WEIGHT && l.id == id) {
        found_weight_cnt++;
        break;
      }
    }

    if (found_white_cnt > FOUND_CNT) return true;
    if (found_weight_cnt > FOUND_CNT) return false;

    auto_crane::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);
  }
}

void Crane::align(auto_crane::LandmarkName name, int id, bool left)
{
  tools::logger()->info("[Crane] align {}", left ? "left" : "right");

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

    Eigen::Vector3d cam_in_odom = this->odom_at(t, left);
    Eigen::Vector2d t_cam2odom = cam_in_odom.head<2>();

    auto landmarks = solver_.solve(detections, t_cam2odom, left);
    matcher_.match(landmarks, -t_map2odom);
    solver_.update_wood(landmarks, t_cam2odom, left);

    for (const auto & l : landmarks) {
      if (l.name != name || l.id != id) continue;
      found = true;
      target = l;
      break;
    }

    if (!found) continue;

    Eigen::Vector3d target_in_odom{target.in_odom[0], target.in_odom[1], this->last_z(left)};

    target_in_odom[0] += left ? x_left_gripper_offset_ : x_right_gripper_offset_;

    if (!is_wood) {
      target_in_odom[1] += left ? y_left_get_offset_ : y_right_get_offset_;
    }

    this->go_no_wait(target_in_odom, left);
    if ((cam_in_odom - target_in_odom).norm() < EPS)
      reach_cnt++;
    else
      reach_cnt = 0;

    if (reach_cnt > REACH_CNT) break;

    // clang-format off
    tools::draw_text(
      img,
      fmt::format(
        "({:.3f}, {:.3f}, {:.3f}) -> ({:.3f}, {:.3f}, {:.3f})",
        cam_in_odom[0],
        cam_in_odom[1],
        cam_in_odom[2],
        target_in_odom[0],
        target_in_odom[1],
        target_in_odom[2]
      ),
      {100, 100}
    );
    // clang-format on

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);
  }

  if (!is_wood) t_map2odom = target.in_odom - target.in_map;
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
