#ifndef AUTO_CRANE__DECIDER_HPP
#define AUTO_CRANE__DECIDER_HPP

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "landmark.hpp"

namespace auto_crane
{

enum State
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

const std::vector<std::string> StateNames = {"for_approx",   "for_weights", "before_crawl",
                                             "crawling",     "after_crawl", "for_wood",
                                             "before_place", "placing",     "after_place"};

class Decider
{
public:
  Decider(const std::string & config_path);

  void state_machine(bool & judge);

  std::string state();

  double get_target_z(const Target & target);

  io::Command decide(
    const Eigen::Vector3d & p_gripper2odo, const Eigen::Vector2d & t_odo2map,
    const std::vector<Target> & targets, const bool & servo_state);

  Target choose_target(const std::vector<Target> & targets);

  bool judge(
    const Eigen::Vector3d & p_gripper2odo, const Eigen::Vector3d & p_target2odo,
    const bool & servo_state, double judge_distance);

private:
  double safe_height_, crawl_height_, short_place_height_, tall_place_height_;
  double judge_distance_;
  int shift_count_, min_shift_count_;
  int circle_count_;
  int servo_count_;
  double x_bias_, y_bias_;
  Eigen::Vector2d bias_;

  State state_;
};

}  // namespace auto_crane

#endif