#ifndef AUTO_CRANE__DECIDER_HPP
#define AUTO_CRANE__DECIDER_HPP

#include <Eigen/Dense>
#include <string>

#include "io/command.hpp"

namespace auto_crane
{
class Decider
{
public:
  Decider(const std::string & config_path);

  void state_machine(bool & judge);

  io::Command decide(
    const Eigen::Vector2d & t_gripper2odo, const Eigen::Vector2d & t_odo2map,
    const Eigen::Vector2d & t_target2map);

private:
  double safe_height_, crawl_height_, short_place_height_, tall_place_height_;
  double judge_distance_;
  int shift_count_, min_shift_count_;

  enum class State
  {
    FOR_APPROX,
    FOR_WEIGHTS,
    BEFORE_CRAWL,
    CRAWLING,
    AFTER_CRAWL,
    FOR_WOOD,
    BEFORE_PLACE,
    PLACING,
    AFTER_PLACE
  };

  State state_;
};

static bool judge(
  const Eigen::Vector2d & t_gripper2odo, const Eigen::Vector2d & t_target2odo,
  double judge_distance);

}  // namespace auto_crane

#endif