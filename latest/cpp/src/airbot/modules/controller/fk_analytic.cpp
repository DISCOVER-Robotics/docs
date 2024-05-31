#include "airbot/modules/controller/fk_analytic.hpp"
namespace arm {

template <>
Frame AnalyticFKSolver<6>::JntToCart(const Joints<6>& joints) {
  Eigen::Matrix4d T[6];
  T[0] << cos(joints[0] - bias[0]), -sin(joints[0] - bias[0]), 0, 0, sin(joints[0] - bias[0]), cos(joints[0] - bias[0]),
      0, 0, 0, 0, 1, a1, 0, 0, 0, 1;
  T[1] << cos(joints[1] - bias[1]), -sin(joints[1] - bias[1]), 0, 0, 0, 0, -1, 0, sin(joints[1] - bias[1]),
      cos(joints[1] - bias[1]), 0, 0, 0, 0, 0, 1;
  T[2] << cos(joints[2] - bias[2]), -sin(joints[2] - bias[2]), 0, a3, sin(joints[2] - bias[2]),
      cos(joints[2] - bias[2]), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  T[3] << 0, 0, 1, a4, sin(joints[3] - bias[3]), cos(joints[3] - bias[3]), 0, 0, -cos(joints[3] - bias[3]),
      sin(joints[3] - bias[3]), 0, 0, 0, 0, 0, 1;
  T[4] << 0, 0, -1, 0, sin(joints[4] - bias[4]), cos(joints[4] - bias[4]), 0, 0, cos(joints[4] - bias[4]),
      -sin(joints[4]) - bias[4], 0, 0, 0, 0, 0, 1;
  T[5] << 0, 0, 1, a6, sin(joints[5] - bias[5]), cos(joints[5] - bias[5]), 0, 0, -cos(joints[5] - bias[5]),
      sin(joints[5] - bias[5]), 0, 0, 0, 0, 0, 1;
  Eigen::Matrix4d T_0_6 = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
  KDL::Frame frame;
  frame.p = KDL::Vector(T_0_6(0, 3), T_0_6(1, 3), T_0_6(2, 3));
  frame.M = KDL::Rotation(T_0_6(0, 0), T_0_6(0, 1), T_0_6(0, 2), T_0_6(1, 0), T_0_6(1, 1), T_0_6(1, 2), T_0_6(2, 0),
                          T_0_6(2, 1), T_0_6(2, 2));
  return frame2array(frame);
}
template <>
AnalyticFKSolver<6>::AnalyticFKSolver(const std::string& model_path) {
  load_model(model_path);
}
}  // namespace arm
