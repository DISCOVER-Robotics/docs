#include "airbot/modules/controller/ikv_chain.hpp"
namespace arm {
template <>
ChainIKVSolver<6>::ChainIKVSolver(const std::string& model_path) {
  load_model(model_path);
  joints_last_ = KDL::JntArray(num_joints_);
  joints_last_(1) = -M_PI / 2;
  joints_last_(2) = M_PI / 2;
  ik_solver_ = new KDL::ChainIkSolverVel_pinv(chain_);
}
template <>
ChainIKVSolver<6>::~ChainIKVSolver() {
  delete ik_solver_;
}
template <>
Joints<6> ChainIKVSolver<6>::CartToJnt(const Joints<6>& pos, const Twist& twist) {
  KDL::Twist vel(KDL::Vector(twist.first[0], twist.first[1], twist.first[2]),
                 KDL::Vector(twist.second[0], twist.second[1], twist.second[2]));
  KDL::JntArray output;
  auto ret = ik_solver_->CartToJnt(joints2jntarray(pos), vel, output);
  if (ret < 0)
    throw InvalidTarget("Fail to solve inverse kinematics");
  else
    return jntarray2joints<6>(output);
}
}  // namespace arm
