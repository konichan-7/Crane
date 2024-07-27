#include "decider.hpp"

#include <yaml-cpp/yaml.h>

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
  judge_distance_ = yaml["judge_distance"].as<double>();  // 单位m^2
  state_ = State::FOR_APPROX;
  min_shift_count_ = 5;
  shift_count_ = 0;
}

static bool judge(
  const Eigen::Vector2d & t_gripper2odo, const Eigen::Vector2d & t_target2odo,
  double judge_distance = 0.02)
{
  double error = (t_gripper2odo - t_target2odo).squaredNorm();
  if (error <= judge_distance) return true;
  return false;
}

io::Command Decider::decide(
  const Eigen::Vector2d & t_gripper2odo, const Eigen::Vector2d & t_odo2map,
  const Eigen::Vector2d & t_target2map)
{
  Eigen::Vector2d t_target2odo = t_target2map - t_odo2map;

  bool shift = judge(t_gripper2odo, t_target2odo);

  state_machine(shift);

  if (state_ == State::FOR_APPROX) {
    return io::Command{0.6375, 0, safe_height_, 0};
  } else if (state_ == State::FOR_WEIGHTS) {
    return io::Command{t_target2map[0], t_target2map[1], safe_height_, 0};
  } else if (state_ == State::BEFORE_CRAWL) {
    return io::Command{t_target2map[0], t_target2map[1], crawl_height_, 0};
  } else if (state_ == State::CRAWLING) {
    return io::Command{t_target2map[0], t_target2map[1], crawl_height_, 1};
  } else if (state_ == State::AFTER_CRAWL) {
    return io::Command{t_target2map[0], t_target2map[1], safe_height_, 1};
  } else if (state_ == State::FOR_WOOD) {
    return io::Command{0, 0, 0, 0};
  } else if (state_ == State::BEFORE_PLACE) {
    return io::Command{0, 0, 0, 0};
  } else if (state_ == State::PLACING) {
    return io::Command{0, 0, 0, 0};
  } else {
    return io::Command{0, 0, 0, 0};
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
