#ifndef AUTO_CRANE__MATCHER_HPP
#define AUTO_CRANE__MATCHER_HPP

#include <Eigen/Dense>
#include <string>

#include "landmark.hpp"

namespace auto_crane
{

class Matcher
{
public:
  Matcher(const std::string & config_path);
  void match(
    const std::vector<Landmark> & landmarks, const Eigen::Vector2d & t_gripper2odo,
    std::vector<Target> & targets, Eigen::Vector2d & t_odo2map);

private:
  double x_cam2gripper_;
  double y_cam2gripper_;
  double judge_distance_;
  Eigen::Vector2d t_cam2gripper_;

  std::array<Eigen::Vector2d, 12> weights_landmark_points_;
  std::array<Eigen::Vector2d, 5> wood_landmark_points_;

  constexpr double pi() { return std::atan(1) * 4; }
  constexpr double deg_to_rad(double degrees) { return degrees * pi() / 180.0; }

  std::array<Eigen::Vector2d, 12> generate_points(
    double center_x, double center_y, double radius1, double radius2)
  {
    std::array<Eigen::Vector2d, 12> points = {};
    for (int i = 0; i < 6; ++i) {
      double angle = deg_to_rad(60 * i);
      points[i] =
        Eigen::Vector2d{center_x + radius1 * std::cos(angle), center_y + radius1 * std::sin(angle)};
    }
    for (int i = 6; i < 12; ++i) {
      double angle = deg_to_rad(60 * (i - 6));
      points[i] =
        Eigen::Vector2d{center_x + radius2 * std::cos(angle), center_y + radius2 * std::sin(angle)};
    }
    return points;
  }

  std::array<Eigen::Vector2d, 5> generate_points()
  {
    std::array<Eigen::Vector2d, 5> points = {};
    points[0] = {1.2, 0};
    points[1] = {2.205, 0.755};
    points[2] = {2.205, -0.755};
    points[3] = {-1.305, 0.755};
    points[4] = {-1.305, -0.755};
    return points;
  }
};

}  // namespace auto_crane

#endif