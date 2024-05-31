#include "airbot/modules/controller/id_chain_rne.hpp"

template <>
arm::ChainIDSolver<6>::ChainIDSolver(const std::string& model_path, const std::string& direction) {
  load_model(model_path);
  if (direction == "right")
    id_solver_ = new KDL::ChainIdSolver_RNE(chain_, KDL::Vector(0, -9.801, 0));
  else if (direction == "left")
    id_solver_ = new KDL::ChainIdSolver_RNE(chain_, KDL::Vector(0, 9.801, 0));
  else if (direction == "down")
    id_solver_ = new KDL::ChainIdSolver_RNE(chain_, KDL::Vector(0, 0, -9.801));
  else
    throw "Not Implemented";
}

template <>
arm::ChainIDSolver<6>::~ChainIDSolver() {
  delete id_solver_;
}

template <>
Joints<6> arm::ChainIDSolver<6>::CartToJnt(const Joints<6>& pos, const Joints<6>& spd, const Wrench& wrench) {
  KDL::JntArray joints(num_joints_);
  // TODO consider this wrench as a means of force feedback. currently wrench is not used.
  auto ret = id_solver_->CartToJnt(joints2jntarray(pos), joints2jntarray(spd), KDL::JntArray(num_joints_),
                                   KDL::Wrenches(num_segments_), joints);
  if (ret < 0) {
    throw InvalidTarget("Fail to solve inverse dynamics");
  } else {
    return jntarray2joints<6>(joints);
  }
}
