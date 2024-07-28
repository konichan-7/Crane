#include "decider.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>

#include "../../tools/logger.hpp"

namespace auto_crane
{

Decider::Decider(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  safe_height_ = yaml["safe_height"].as<double>();
  crawl_height_ = yaml["crawl_height"].as<double>();
  short_place_height_ = yaml["short_place_height"].as<double>();
  tall_place_height_ = yaml["tall_place_height"].as<double>();
  state_ = State::FOR_APPROX;
  min_shift_count_ = 5;
  shift_count_ = 0;
  circle_count_ = 1;
  servo_count_ = 0;
}

bool Decider::judge(
  const Eigen::Vector2d & t_gripper2odo, const Eigen::Vector2d & t_target2odo,
  const bool & servo_state, double judge_distance = 0.5)
{
  double error = (t_gripper2odo - t_target2odo).norm();

  if (
    state_ == State::BEFORE_CRAWL || state_ == State::CRAWLING || state_ == State::AFTER_CRAWL ||
    state_ == State::BEFORE_PLACE || state_ == State::PLACING || state_ == State::AFTER_PLACE) {
    ++servo_count_;
    if (servo_count_ > 100 && servo_state == true) {
      servo_count_ = 0;
      return true;
    }
    return false;
  }

  if (error <= judge_distance) return true;
  return false;
}

std::string Decider::state() { return StateNames[state_]; }

Target Decider::choose_target(const std::vector<Target> & targets)
{
  if (state_ == State::FOR_APPROX) {
    switch (circle_count_) {
      case 1:
        return Target{Eigen::Vector2d{0.6375, 0}, TargetName::CENTER};
      case 2:
        return Target{Eigen::Vector2d{0.9188, -0.4874}, TargetName::CENTER};
      default:
        return Target{};  // 默认返回值
    }
  } else if (
    state_ == State::FOR_WEIGHTS || state_ == State::BEFORE_CRAWL || state_ == State::CRAWLING ||
    state_ == State::AFTER_CRAWL) {
    auto it = std::find_if(targets.begin(), targets.end(), [](const Target & t) {
      return t.name == TargetName::WEIGHT;
    });
    if (it != targets.end()) {
      return *it;
    }
    return Target{};  // 未找到目标时的默认返回值
  } else {
    switch (circle_count_) {
      case 1:
        return Target{Eigen::Vector2d{1.2, 0}, TargetName::SHORT_WOOD};
      case 2:
        return Target{Eigen::Vector2d{2.205, -0.755}, TargetName::TALL_WOOD};
      default:
        return Target{};  // 默认返回值
    }
  }
}

io::Command Decider::decide(
  const Eigen::Vector2d & t_gripper2odo, const Eigen::Vector2d & t_odo2map,
  const std::vector<Target> & targets, const bool & servo_state)
{
  auto target = choose_target(targets);

  Eigen::Vector2d t_target2odo = target.t_target2map - t_odo2map;

  bool shift = judge(t_gripper2odo, t_target2odo, servo_state);

  state_machine(shift);

  double place_height =
    (target.name == TargetName::SHORT_WOOD) ? short_place_height_ : tall_place_height_;

  if (state_ == State::FOR_APPROX) {
    return io::Command{target.t_target2map[0], target.t_target2map[1], safe_height_, 0};
  } else if (state_ == State::FOR_WEIGHTS) {
    if (!shift) return io::Command{0, 0, 0, 0};
    return io::Command{t_target2odo[0], t_target2odo[1], safe_height_, 0};
  } else if (state_ == State::BEFORE_CRAWL) {
    if (!shift) return io::Command{0, 0, 0, 0};
    return io::Command{t_target2odo[0], t_target2odo[1], crawl_height_, 0};
  } else if (state_ == State::CRAWLING) {
    if (!shift) return io::Command{0, 0, 0, 0};
    return io::Command{t_target2odo[0], t_target2odo[1], crawl_height_, 1};
  } else if (state_ == State::AFTER_CRAWL) {
    if (!shift) return io::Command{0, 0, 0, 0};
    return io::Command{t_target2odo[0], t_target2odo[1], safe_height_, 1};
  } else if (state_ == State::FOR_WOOD) {
    if (!shift) return io::Command{0, 0, 0, 0};
    return io::Command{t_target2odo[0], t_target2odo[1], safe_height_, 1};
  } else if (state_ == State::BEFORE_PLACE) {
    if (!shift) return io::Command{0, 0, 0, 0};
    return io::Command{t_target2odo[0], t_target2odo[1], place_height, 1};
  } else if (state_ == State::PLACING) {
    if (!shift) return io::Command{0, 0, 0, 0};
    return io::Command{t_target2odo[0], t_target2odo[1], place_height, 0};
  } else {
    if (!shift) return io::Command{0, 0, 0, 0};
    return io::Command{t_target2odo[0], t_target2odo[1], safe_height_, 0};
  }
}

void Decider::state_machine(bool & judge)
{
  if (!judge) return;

  ++shift_count_;
  if (shift_count_ > min_shift_count_) {
    shift_count_ = 0;
    switch (state_) {
      case State::FOR_APPROX:
        state_ = State::FOR_WEIGHTS;
        ++circle_count_;
        break;
      case State::FOR_WEIGHTS:
        state_ = State::BEFORE_CRAWL;
        break;
      case State::BEFORE_CRAWL:
        state_ = State::CRAWLING;
        break;
      case State::CRAWLING:
        state_ = State::AFTER_CRAWL;
        break;
      case State::AFTER_CRAWL:
        state_ = State::FOR_WOOD;
        break;
      case State::FOR_WOOD:
        state_ = State::BEFORE_PLACE;
        break;
      case State::BEFORE_PLACE:
        state_ = State::PLACING;
        break;
      case State::PLACING:
        state_ = State::AFTER_PLACE;
        break;
      case State::AFTER_PLACE:
        state_ = State::FOR_APPROX;
        break;
    }
  }
}

}  // namespace auto_crane
