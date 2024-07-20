#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <Eigen/Geometry>
#include <chrono>

namespace tools
{
// 将弧度值限制在(-pi, pi]
double limit_rad(double angle);

// 四元数转欧拉角
// x = 0, y = 1, z = 2
// e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
// 参考：https://github.com/evbernardes/quaternion_to_euler
Eigen::Vector3d eulers(
  Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false);

// 旋转矩阵转欧拉角
// x = 0, y = 1, z = 2
// e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false);

// 直角坐标系转球坐标系
// ypd为yaw、pitch、distance的缩写
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz);

// 直角坐标系转球坐标系转换函数对xyz的雅可比矩阵
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz);

// 计算时间差a - b，单位：s
double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b);

}  // namespace tools

#endif  // TOOLS__MATH_TOOLS_HPP