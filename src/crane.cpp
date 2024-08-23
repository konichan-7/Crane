#include "crane.hpp"

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

constexpr int GRIP_CNT = 30;
constexpr int REACH_CNT = 5;
constexpr int FOUND_CNT = 3;
constexpr double EPS = 0.0075;

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

  auto x_left_odom_in_map = yaml["x_left_odom_in_map"].as<double>();
  auto y_left_odom_in_map = yaml["y_left_odom_in_map"].as<double>();
  auto x_right_odom_in_map = yaml["x_right_odom_in_map"].as<double>();
  auto y_right_odom_in_map = yaml["y_right_odom_in_map"].as<double>();
  t_map_to_left_odom_ = {-x_left_odom_in_map, -y_left_odom_in_map};
  t_map_to_right_odom_ = {-x_right_odom_in_map, -y_right_odom_in_map};

  x_left_gripper_offset_ = yaml["x_left_gripper_offset"].as<double>();
  x_right_gripper_offset_ = yaml["x_right_gripper_offset"].as<double>();

  y_left_get_offset_ = yaml["y_left_get_offset"].as<double>();
  y_right_get_offset_ = yaml["y_right_get_offset"].as<double>();

  auto x_left_wood_offset = yaml["x_left_wood_offset"].as<double>();
  auto y_left_wood_offset = yaml["y_left_wood_offset"].as<double>();
  auto x_right_wood_offset = yaml["x_right_wood_offset"].as<double>();
  auto y_right_wood_offset = yaml["y_right_wood_offset"].as<double>();
  left_wood_offset_ = {x_left_wood_offset, y_left_wood_offset};
  right_wood_offset_ = {x_right_wood_offset, y_right_wood_offset};

  y_left_put_offset_ = yaml["y_left_put_offset"].as<double>();
  y_right_put_offset_ = yaml["y_right_put_offset"].as<double>();
  x_align_woods_offset_ = yaml["x_align_woods_offset"].as<double>();
}

void Crane::wait_to_start()
{
  tools::logger()->info("[Crane] waiting to start...");
  while (!left_cboard_.start || !right_cboard_.start) std::this_thread::sleep_for(10ms);
}

void Crane::forward(auto_crane::LandmarkName name, int id, double z, bool left)
{
  auto is_weight = (name == auto_crane::WEIGHT);

  Eigen::Vector2d m = is_weight ? matcher_.weight_in_map(id) : matcher_.wood_in_map(id);
  Eigen::Vector2d o = m + (left ? t_map_to_left_odom_ : t_map_to_right_odom_);

  auto command = left ? left_last_cmd_ : right_last_cmd_;
  command.x = o[0];
  command.y = o[1] + (left ? -0.04 : 0.04);
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

void Crane::puts(int id_l, int id_r)
{
  this->align_woods(id_l, id_r);
  this->rotate(id_l, id_r);

  left_last_cmd_.slow = true;
  right_last_cmd_.slow = true;
  this->go_both(
    {this->last_x(), this->last_y(true) + y_left_put_offset_, HOLD_Z},
    {this->last_x(), this->last_y(false) + y_right_put_offset_, HOLD_Z});
  left_last_cmd_.slow = false;
  right_last_cmd_.slow = false;

  this->go_both(
    {this->last_x(), this->last_y(true), PUT_H_Z}, {this->last_x(), this->last_y(false), PUT_H_Z});
  this->grip_both(false, false);
  this->go_both(
    {this->last_x(), this->last_y(true), HOLD_Z}, {this->last_x(), this->last_y(false), HOLD_Z});
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

void Crane::go_both(Eigen::Vector3d l_in_odom, Eigen::Vector3d r_in_odom)
{
  tools::logger()->info("[Crane] go both");

  int reach_cnt = 0;

  while (true) {
    cv::Mat img_l;
    std::chrono::steady_clock::time_point t_l;
    this->read(img_l, t_l, true);

    cv::Mat img_r;
    std::chrono::steady_clock::time_point t_r;
    this->read(img_r, t_r, false);

    Eigen::Vector3d cam_in_odom_l;
    Eigen::Vector3d cam_in_odom_r;

    if (t_r > t_l) {
      cam_in_odom_l = this->odom_at(t_l, true);
      cam_in_odom_r = this->odom_at(t_r, false);
    }
    else {
      cam_in_odom_r = this->odom_at(t_r, false);
      cam_in_odom_l = this->odom_at(t_l, true);
    }

    this->go_no_wait(l_in_odom, true);
    this->go_no_wait(r_in_odom, false);

    if ((cam_in_odom_l - l_in_odom).norm() < EPS && (cam_in_odom_r - r_in_odom).norm() < EPS)
      reach_cnt++;
    else
      reach_cnt = 0;

    if (reach_cnt > REACH_CNT) break;

    cv::resize(img_l, img_l, {}, 0.5, 0.5);
    cv::imshow("img", img_l);
    cv::resize(img_r, img_r, {}, 0.5, 0.5);
    cv::imshow("img2", img_r);
    cv::waitKey(1);
  }
}

bool Crane::find_white(int id, bool left)
{
  tools::logger()->info("[Crane] find_white {} {}", id, left ? "left" : "right");

  Eigen::Vector2d & t_map2odom = left ? t_map_to_left_odom_ : t_map_to_right_odom_;

  auto found_white_cnt = 0;
  auto found_weight_cnt = 0;
  auto_crane::Landmark target;

  while (true) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    this->read(img, t, left);

    Eigen::Vector3d cam_in_odom = this->odom_at(t, left);
    Eigen::Vector2d t_cam2odom = cam_in_odom.head<2>();

    auto detections = yolo_.infer(img);
    auto landmarks = solver_.solve(detections, t_cam2odom, left);
    matcher_.match(landmarks, -t_map2odom);

    for (const auto & l : landmarks) {
      if (l.name == auto_crane::WHITE && l.id == id) {
        found_white_cnt++;
        target = l;
        break;
      }
      if (l.name == auto_crane::WEIGHT && l.id == id) {
        found_weight_cnt++;
        target = l;
        break;
      }
    }

    if (found_white_cnt > FOUND_CNT || found_weight_cnt > FOUND_CNT) break;

    auto_crane::draw_detections(img, detections, classes);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);
  }

  if (found_white_cnt > FOUND_CNT) {
    t_map2odom = target.in_odom - target.in_map;
    return true;
  }

  return false;
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
    else {
      target_in_odom[1] += left ? -0.02 : 0.02;
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

void Crane::align_woods(int id_l, int id_r)
{
  tools::logger()->info("[Crane] align woods {} {}", id_l, id_r);

  auto reach_cnt = 0;

  while (true) {
    cv::Mat img_l;
    std::chrono::steady_clock::time_point t_l;
    this->read(img_l, t_l, true);
    auto detections_l = yolo_.infer(img_l);
    auto_crane::draw_detections(img_l, detections_l, classes);

    cv::Mat img_r;
    std::chrono::steady_clock::time_point t_r;
    this->read(img_r, t_r, false);
    auto detections_r = yolo_.infer(img_r);
    auto_crane::draw_detections(img_r, detections_r, classes);

    Eigen::Vector3d cam_in_odom_l;
    Eigen::Vector3d cam_in_odom_r;

    if (t_r > t_l) {
      cam_in_odom_l = this->odom_at(t_l, true);
      cam_in_odom_r = this->odom_at(t_r, false);
    }
    else {
      cam_in_odom_r = this->odom_at(t_r, false);
      cam_in_odom_l = this->odom_at(t_l, true);
    }

    auto found_l = false;
    auto_crane::Landmark wood_l;
    Eigen::Vector2d t_cam2odom_l = cam_in_odom_l.head<2>();
    auto landmarks_l = solver_.solve(detections_l, t_cam2odom_l, true);
    matcher_.match(landmarks_l, -t_map_to_left_odom_);
    for (const auto & l : landmarks_l) {
      if (l.name != auto_crane::TALL_WOOD || l.id != id_l) continue;
      found_l = true;
      wood_l = l;
      break;
    }

    auto found_r = false;
    auto_crane::Landmark wood_r;
    Eigen::Vector2d t_cam2odom_r = cam_in_odom_r.head<2>();
    auto landmarks_r = solver_.solve(detections_r, t_cam2odom_r, false);
    matcher_.match(landmarks_r, -t_map_to_right_odom_);
    for (const auto & l : landmarks_r) {
      if (l.name != auto_crane::TALL_WOOD || l.id != id_r) continue;
      found_r = true;
      wood_r = l;
      break;
    }

    if (found_l && found_r) {
      Eigen::Vector3d l_in_odom{wood_l.in_odom[0], wood_l.in_odom[1] - 0.02, HOLD_Z};
      Eigen::Vector3d r_in_odom{wood_r.in_odom[0], wood_r.in_odom[1] + 0.02, HOLD_Z};

      auto x = (l_in_odom[0] + r_in_odom[0]) * 0.5 + x_align_woods_offset_;
      l_in_odom[0] = x;
      r_in_odom[0] = x;

      this->go_no_wait(l_in_odom, true);
      this->go_no_wait(r_in_odom, false);

      if ((cam_in_odom_l - l_in_odom).norm() < EPS && (cam_in_odom_r - r_in_odom).norm() < EPS)
        reach_cnt++;
      else
        reach_cnt = 0;
    }

    if (reach_cnt > REACH_CNT) break;

    cv::resize(img_l, img_l, {}, 0.5, 0.5);
    cv::imshow("img", img_l);
    cv::resize(img_r, img_r, {}, 0.5, 0.5);
    cv::imshow("img2", img_r);
    cv::waitKey(1);
  }
}

void Crane::rotate(int id_l, int id_r)
{
  tools::logger()->info("[Crane] rotate {} {}", id_l, id_r);

  auto dx = 0.0;

  while (true) {
    cv::Mat img_l;
    std::chrono::steady_clock::time_point t_l;
    this->read(img_l, t_l, true);
    auto detections_l = yolo_.infer(img_l);
    auto_crane::draw_detections(img_l, detections_l, classes);

    cv::Mat img_r;
    std::chrono::steady_clock::time_point t_r;
    this->read(img_r, t_r, false);
    auto detections_r = yolo_.infer(img_r);
    auto_crane::draw_detections(img_r, detections_r, classes);

    Eigen::Vector3d cam_in_odom_l;
    Eigen::Vector3d cam_in_odom_r;

    if (t_r > t_l) {
      cam_in_odom_l = this->odom_at(t_l, true);
      cam_in_odom_r = this->odom_at(t_r, false);
    }
    else {
      cam_in_odom_r = this->odom_at(t_r, false);
      cam_in_odom_l = this->odom_at(t_l, true);
    }

    auto found_l = false;
    auto_crane::Landmark wood_l;
    Eigen::Vector2d t_cam2odom_l = cam_in_odom_l.head<2>();
    auto landmarks_l = solver_.solve(detections_l, t_cam2odom_l, true);
    matcher_.match(landmarks_l, -t_map_to_left_odom_);
    for (const auto & l : landmarks_l) {
      if (l.name != auto_crane::TALL_WOOD || l.id != id_l) continue;
      found_l = true;
      wood_l = l;
      break;
    }

    auto found_r = false;
    auto_crane::Landmark wood_r;
    Eigen::Vector2d t_cam2odom_r = cam_in_odom_r.head<2>();
    auto landmarks_r = solver_.solve(detections_r, t_cam2odom_r, false);
    matcher_.match(landmarks_r, -t_map_to_right_odom_);
    for (const auto & l : landmarks_r) {
      if (l.name != auto_crane::TALL_WOOD || l.id != id_r) continue;
      found_r = true;
      wood_r = l;
      break;
    }

    if (!found_l || !found_r) continue;

    dx = 0.03 - (wood_l.in_odom[0] - wood_r.in_odom[0]);
    tools::logger()->debug("dx={:.3f}", dx);
    break;

    cv::resize(img_l, img_l, {}, 0.5, 0.5);
    cv::imshow("img", img_l);
    cv::resize(img_r, img_r, {}, 0.5, 0.5);
    cv::imshow("img2", img_r);
    cv::waitKey(1);
  }

  left_cboard_.rotate(dx);
}

void Crane::grip(bool grip, bool left)
{
  tools::logger()->info("[Crane] grip {}", left ? "left" : "right");

  auto command = left ? left_last_cmd_ : right_last_cmd_;
  command.grip = grip;

  for (int i = 0; i < GRIP_CNT; i++) {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    this->read(img, t, left);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("img", img);
    cv::waitKey(1);

    this->cmd(command, left);
  }
}

void Crane::grip_both(bool grip_l, bool grip_r)
{
  tools::logger()->info("[Crane] grip both");

  auto command_l = left_last_cmd_;
  auto command_r = right_last_cmd_;
  command_l.grip = grip_l;
  command_r.grip = grip_r;

  for (int i = 0; i < GRIP_CNT; i++) {
    cv::Mat img_l;
    cv::Mat img_r;
    std::chrono::steady_clock::time_point t;
    this->read(img_l, t, true);
    this->read(img_r, t, false);

    cv::resize(img_l, img_l, {}, 0.5, 0.5);
    cv::imshow("img", img_l);
    cv::resize(img_r, img_r, {}, 0.5, 0.5);
    cv::imshow("img2", img_r);
    cv::waitKey(1);

    this->cmd(command_l, true);
    this->cmd(command_r, false);
  }
}
