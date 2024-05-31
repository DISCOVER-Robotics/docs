#include "airbot/modules/controller/ik_analytic.hpp"

#include <iostream>
namespace arm {
/**
 * FIXME HARDCODED PARAMETERS
 */
template <>
AnalyticIKSolver<6>::AnalyticIKSolver(const std::string& model_path) {
  load_model(model_path);
}

template <std::size_t DOF>
inline Joints<DOF> add_bias(Joints<DOF> angle) {
  Joints<DOF> ret;
  for (int i = 0; i < ret.size(); i++) {
    angle[i] += bias[i];
    while (angle[i] > M_PI) angle[i] -= 2 * M_PI;
    while (angle[i] < -M_PI) angle[i] += 2 * M_PI;
    ret[i] = angle[i];
  }
  return ret;
}
inline array<double, 3> move_joint6_2_joint5(array<double, 3> pos, KDL::Rotation ori) {
  array<double, 3> ret;
  ret[0] = -ori(0, 2) * a6 + pos[0];
  ret[1] = -ori(1, 2) * a6 + pos[1];
  ret[2] = -ori(2, 2) * a6 + pos[2];
  return ret;
}
template <>
vector<Joints<6>> AnalyticIKSolver<6>::CartToJnt(const Frame& pose) {
  auto target_pose = array2frame(pose);
  vector<Joints<6>> ret;
  array<double, 6> angle;
  array<double, 3> pos;
  pos = move_joint6_2_joint5(pose.first, target_pose.M);
  Eigen::Matrix3d ori;
  for (int i1 : {1, -1}) {
    angle[0] = atan2(i1 * pos[1], i1 * pos[0]);
    double c3 = (pos[0] * pos[0] + pos[1] * pos[1] + (pos[2] - a1) * (pos[2] - a1) - a3 * a3 - a4 * a4) / (2 * a3 * a4);
    if (c3 > 1 || c3 < -1) {
      throw InvalidTarget("Fail to solve inverse kinematics");
      continue;
    }
    for (int i2 : {1, -1}) {
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) ori(i, j) = target_pose.M(i, j);
      double s3 = i2 * sqrt(1 - c3 * c3);
      angle[2] = atan2(s3, c3);
      double k1 = a3 + a4 * c3, k2 = a4 * s3;
      angle[1] = atan2(k1 * (pos[2] - a1) - i1 * k2 * sqrt(pos[0] * pos[0] + pos[1] * pos[1]),
                       i1 * k1 * sqrt(pos[0] * pos[0] + pos[1] * pos[1]) + k2 * (pos[2] - a1));
      Eigen::Matrix3d R;
      R << cos(angle[0]) * cos(angle[1] + angle[2]), -cos(angle[0]) * sin(angle[1] + angle[2]), sin(angle[0]),
          sin(angle[0]) * cos(angle[1] + angle[2]), -sin(angle[0]) * sin(angle[1] + angle[2]), -cos(angle[0]),
          sin(angle[1] + angle[2]), cos(angle[1] + angle[2]), 0;
      ori = R.inverse() * ori;
      for (int i5 : {1, -1}) {
        angle[3] = atan2(i5 * ori(2, 2), i5 * ori(1, 2));
        angle[4] = atan2(i5 * (sqrt(ori(2, 2) * ori(2, 2) + ori(1, 2) * ori(1, 2))), ori(0, 2));
        angle[5] = atan2(-i5 * ori(0, 0), -i5 * ori(0, 1));
        ret.push_back(add_bias(angle));
      }
    }
  }
  return ret;
}
}  // namespace arm
