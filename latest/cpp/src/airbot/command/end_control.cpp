#include "airbot/command/command_base.hpp"
template <std::size_t DOF>
bool arm::Robot<DOF>::set_target_pose(const Frame& target_pose, bool use_planning, double vel, bool blocking) {
  if (arm_mode_.load(std::memory_order::memory_order_relaxed) != ArmMode::ONLINE) {
    logger_->warn("current state is {}, set_target_pose is NOT permitted",
                  ArmModeStr[arm_mode_.load(std::memory_order::memory_order_relaxed)]);
    return false;
  }
  logger_->debug("Setting target pose: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}", target_pose.first[0],
                 target_pose.first[1], target_pose.first[2], target_pose.second[0], target_pose.second[1],
                 target_pose.second[2]);
  std::vector<Joints<DOF>> results;
  try {
    results = ik_solver_->CartToJnt(target_pose);
  } catch (const arm::InvalidTarget& e) {
    logger_->error(e.what());
    return false;
  }
  auto current_joints = get_current_joint_q();
  auto result = *std::min_element(results.begin(), results.end(),
                                  [&current_joints, this](const Joints<DOF>& a, const Joints<DOF>& b) {
                                    if (!valid_joint_q(a))
                                      return false;
                                    else if (!valid_joint_q(b))
                                      return true;
                                    return (l1norm(a, current_joints) < l1norm(b, current_joints));
                                  });
  if (!valid_joint_q(result)) {
    logger_->error("No valid target");
    return false;
  }
  logger_->info("Calculated target joint position: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}", result[0], result[1],
                result[2], result[3], result[4], result[5]);
  auto ret = _plan_target_joint_q(result, use_planning, vel);
  if (blocking) {
    for (int i = 0; i < (double)BLOCK_TIMEOUT / BLOCK_SPIN_TIME; i++)
      if (l1norm(get_current_joint_q(), result) <= BLOCK_THRESHOLD)
        return true;
      else
        std::this_thread::sleep_for(std::chrono::milliseconds(BLOCK_SPIN_TIME));
    auto current_joint_q = get_current_joint_q();
    logger_->warn(
        "set_target_pose blocking timeout, current joint q: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} target joint q: "
        "{:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}",
        current_joint_q[0], current_joint_q[1], current_joint_q[2], current_joint_q[3], current_joint_q[4],
        current_joint_q[5], result[0], result[1], result[2], result[3], result[4], result[5]);
    return false;
  } else {
    return ret;
  }
}
template <std::size_t DOF>
bool arm::Robot<DOF>::set_target_pose(const Translation& target_translation, const Rotation& target_rotation,
                                      bool use_planning, double vel, bool blocking) {
  return set_target_pose({target_translation, target_rotation}, use_planning, vel, blocking);
}
template <std::size_t DOF>
bool arm::Robot<DOF>::set_target_translation(const Translation& target_translation, bool use_planning, double vel,
                                             bool blocking) {
  return set_target_pose({target_translation, get_current_rotation()}, use_planning, vel, blocking);
}
template <std::size_t DOF>
bool arm::Robot<DOF>::set_target_rotation(const Rotation& target_rotation, bool use_planning, double vel,
                                          bool blocking) {
  return set_target_pose({get_current_translation(), target_rotation}, use_planning, vel, blocking);
}
template <std::size_t DOF>
bool arm::Robot<DOF>::add_target_translation(const Translation& target_d_translation, bool use_planning, double vel,
                                             bool blocking) {
  Joints<DOF> target_joint_q;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    target_joint_q = robot_cmd_data_.plan_target_joint_q;
  }
  auto plan_target_pose = fk_solver_->JntToCart(target_joint_q);
  auto target_translation = Translation{plan_target_pose.first[0] + target_d_translation[0],
                                        plan_target_pose.first[1] + target_d_translation[1],
                                        plan_target_pose.first[2] + target_d_translation[2]};

  return set_target_pose({target_translation, plan_target_pose.second}, use_planning, vel, blocking);
}
template <std::size_t DOF>
bool arm::Robot<DOF>::add_target_relative_translation(const Translation& target_d_translation, bool use_planning,
                                                      double vel, bool blocking) {
  Joints<DOF> target_joint_q;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    target_joint_q = robot_cmd_data_.plan_target_joint_q;
  }
  auto plan_target_pose = array2frame(fk_solver_->JntToCart(target_joint_q));
  plan_target_pose.p =
      plan_target_pose.M * KDL::Vector(target_d_translation[0], target_d_translation[1], target_d_translation[2]) +
      plan_target_pose.p;
  return set_target_pose(frame2array(plan_target_pose), use_planning, vel, blocking);
}
template <std::size_t DOF>
bool arm::Robot<DOF>::add_target_relative_rotation(const Rotation& target_d_rotation, bool use_planning, double vel,
                                                   bool blocking) {
  Joints<DOF> target_joint_q;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    target_joint_q = robot_cmd_data_.plan_target_joint_q;
  }
  auto plan_target_pose = array2frame(fk_solver_->JntToCart(target_joint_q));
  plan_target_pose.M = plan_target_pose.M * KDL::Rotation::Quaternion(target_d_rotation[0], target_d_rotation[1],
                                                                      target_d_rotation[2], target_d_rotation[3]);
  return set_target_pose(frame2array(plan_target_pose), use_planning, vel, blocking);
}
template <std::size_t DOF>
bool arm::Robot<DOF>::set_target_vel(const Twist& target_vel) {
  if (arm_mode_.load(std::memory_order::memory_order_relaxed) != ArmMode::ONLINE) {
    logger_->warn("current state is {}, set_target_joint_mit is NOT permitted",
                  ArmModeStr[arm_mode_.load(std::memory_order::memory_order_relaxed)]);
    return false;
  }
  try {
    auto current_joint_q = get_current_joint_q();
    auto result = ikv_solver_->CartToJnt(current_joint_q, target_vel);
    return set_target_joint_v(result);
  } catch (const arm::InvalidTarget& e) {
    logger_->error(e.what());
    return false;
  }
}
