#ifndef EIGEN_TYPE_HPP
#define EIGEN_TYPE_HPP
#include "Eigen/Dense"
// double
using Vec2D = typename Eigen::Matrix<double, 2, 1>;
using Vec3D = typename Eigen::Matrix<double, 3, 1>;
using Vec4D = typename Eigen::Matrix<double, 4, 1>;
using Vec6D = typename Eigen::Matrix<double, 6, 1>;
using Vec12D = typename Eigen::Matrix<double, 12, 1>;
using Vec18D = typename Eigen::Matrix<double, 18, 1>;
using VecXD = typename Eigen::Matrix<double, Eigen::Dynamic, 1>;
using rot_mat3x3D = typename Eigen::Matrix<double, 3, 3>;
using homo_mat4x4D = typename Eigen::Matrix<double, 4, 4>;
using Mat2x2D = typename Eigen::Matrix<double, 2, 2>;
using Mat3x3D = typename Eigen::Matrix<double, 3, 3>;
using Mat4x4D = typename Eigen::Matrix<double, 4, 4>;
using Mat6x6D = typename Eigen::Matrix<double, 6, 6>;
using Mat12x12D = typename Eigen::Matrix<double, 12, 12>;
using Mat18x18D = typename Eigen::Matrix<double, 18, 18>;
using MatXD = typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using Mat3x4D = typename Eigen::Matrix<double, 3, 4>;
using Mat4x5D = typename Eigen::Matrix<double, 4, 5>;
// int
using Vec3I = typename Eigen::Matrix<int, 3, 1>;
using Vec4I = typename Eigen::Matrix<int, 4, 1>;
// char
using Vec4C = typename Eigen::Matrix<char, 4, 1>;
// float
using Vec2F = typename Eigen::Matrix<float, 2, 1>;
using Vec3F = typename Eigen::Matrix<float, 3, 1>;
using Vec4F = typename Eigen::Matrix<float, 4, 1>;
using Vec5F = typename Eigen::Matrix<float, 5, 1>;
using Vec6F = typename Eigen::Matrix<float, 6, 1>;
using Vec8F = typename Eigen::Matrix<float, 8, 1>;
using Vec10F = typename Eigen::Matrix<float, 10, 1>;
using Vec12F = typename Eigen::Matrix<float, 12, 1>;
using Vec18F = typename Eigen::Matrix<float, 18, 1>;
using VecXF = typename Eigen::Matrix<float, Eigen::Dynamic, 1>;
using Mat2x3F = typename Eigen::Matrix<float, 2, 3>;
using Mat2x2F = typename Eigen::Matrix<float, 2, 2>;
using Mat3x3F = typename Eigen::Matrix<float, 3, 3>;
using Mat3x4F = typename Eigen::Matrix<float, 3, 4>;
using Mat4x2F = typename Eigen::Matrix<float, 4, 2>;
using Mat4x3F = typename Eigen::Matrix<float, 4, 3>;
using Mat4x4F = typename Eigen::Matrix<float, 4, 4>;
using Mat4x5F = typename Eigen::Matrix<float, 4, 5>;
using Mat4x6F = typename Eigen::Matrix<float, 4, 6>;
using Mat4x7F = typename Eigen::Matrix<float, 4, 7>;
using Mat4x9F = typename Eigen::Matrix<float, 4, 9>;
using Mat4x10F = typename Eigen::Matrix<float, 4, 10>;
using Mat4x11F = typename Eigen::Matrix<float, 4, 11>;
using Mat6x4F = typename Eigen::Matrix<float, 6, 4>;
using Mat6x6F = typename Eigen::Matrix<float, 6, 6>;
// 控制系统专用矩阵类型
using Mat4x4F = typename Eigen::Matrix<float, 4, 4>;
using Mat12x1F = typename Eigen::Matrix<float, 12, 1>;
using Mat12x12F = typename Eigen::Matrix<float, 12, 12>;
using Mat12x3F = typename Eigen::Matrix<float, 12, 3>;
using Mat3x1F = typename Eigen::Matrix<float, 3, 1>;
using MatXF = typename Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using MatXxYF = typename Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;

// 四足机器人专用矩阵类型
using LegParamMatF = typename Eigen::Matrix<float, 4, 12>;   // 4条腿×12种参数
using GaitPhaseMatF = typename Eigen::Matrix<float, 4, 4>;   // 4条腿×4种相位参数
using MotorParamMatF = typename Eigen::Matrix<float, 12, 2>; // 12个电机×2种参数(P/D)
/******** Matrix ********/
#define I3 Eigen::MatrixXd::Identity(3, 3)
#define I12 Eigen::MatrixXd::Identity(12, 12)
#define I18 Eigen::MatrixXd::Identity(18, 18)
using Quat4x1D = typename Eigen::Matrix<double, 4, 1>;
/****** Functions *******/
inline Mat3x4D VecDouble12ToMatDouble34(const Vec12D &vec1x12)
{
  Mat3x4D vec3x4;
  for (int i(0); i < 4; ++i)
  {
    vec3x4.col(i) = vec1x12.segment(3 * i, 3);
  }
  return vec3x4;
}
inline Vec12D VecDouble34ToVecDouble12(const Mat3x4D &vec3x4)
{
  Vec12D vec1x12;
  for (int i(0); i < 4; ++i)
  {
    vec1x12.segment(3 * i, 3) = vec3x4.col(i);
  }
  return vec1x12;
}

#endif
