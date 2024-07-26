#ifndef AUTO_CRANE__MATCHER_HPP
#define AUTO_CRANE__MATCHER_HPP

#include <Eigen/Dense>
#include <string>

namespace auto_crane
{

class Matcher
{
public:
  Matcher(const std::string & config_path);
  Eigen::Vector2d match(
    const Eigen::Vector2d & t_landmark2cam, const Eigen::Vector2d & t_gripper2odo,
    Eigen::Vector2d & t_landmark2map);

private:
  struct Point_
  {
    double x;
    double y;
  };

  double x_cam2gripper_;
  double y_cam2gripper_;
  double judge_distance_;
  std::array<Point_, 12> landmark_points_;
  constexpr double pi() { return std::atan(1) * 4; }
  constexpr double deg_to_rad(double degrees) { return degrees * pi() / 180.0; }

  constexpr std::array<Point_, 12> generate_points(
    double center_x, double center_y, double radius1, double radius2)
  {
    std::array<Point_, 12> points = {};
    for (int i = 0; i < 6; ++i) {
      double angle = deg_to_rad(60 * i);
      points[i] =
        Point_{center_x + radius1 * std::cos(angle), center_y + radius1 * std::sin(angle)};
    }
    for (int i = 6; i < 12; ++i) {
      double angle = deg_to_rad(60 * (i - 6));
      points[i] =
        Point_{center_x + radius2 * std::cos(angle), center_y + radius2 * std::sin(angle)};
    }
    return points;
  }
};

}  // namespace auto_crane

#endif