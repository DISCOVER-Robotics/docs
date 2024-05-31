#include "airbot/command/command_base.hpp"
template <std::size_t DOF>
bool arm::Robot<DOF>::_plan_target_joint_q(const Joints<DOF>& target_joint_q_, bool use_planning, double vel,
                                           bool commit) {
  Joints<DOF> target_joint_q;
  if (joint_safe_detect_.load() && !valid_joint_q(target_joint_q_)) {
    // logger_->warn("Planning: Target joint position out of boundary!");
    // return false;
    for (int i = 0; i < target_joint_q_.size(); i++) {
      target_joint_q[i] =
          std::clamp(target_joint_q_[i], motor_driver_[i]->MotorBoundary()[0], motor_driver_[i]->MotorBoundary()[1]);
    }
  } else {
    target_joint_q = target_joint_q_;
  }
  bool perform_planning = use_planning;
  double distance = 0.;
  Joints<DOF> current_joint_q, current_joint_v;
  if (use_planning) {
    {
      std::lock_guard<std::shared_mutex> lock(fb_mutex_);
      current_joint_q = robot_fb_data_.current_joint_q;
      current_joint_v = robot_fb_data_.current_joint_v;
    }
    auto current_pose = fk_solver_->JntToCart(current_joint_q);
    auto target_pose = fk_solver_->JntToCart(target_joint_q);
    distance = diff(current_pose, target_pose);
    if (distance < 0.002) {
      logger_->info("Planning: Target joint position is too close to current joint position, no need to plan!");
      perform_planning = false;
    }
  }
  if (perform_planning) {
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (commit) {
        robot_cmd_data_.cmd_state = MotorControlState::JOINT_POS;
        robot_cmd_data_.plan_target_joint_q = target_joint_q;
        robot_cmd_data_.target_joint_v = joint_vel_limit_;
      }
      robot_plan_data_.plan_params =
          calc_plan(current_joint_q, current_joint_v, target_joint_q, {0., 0., 0., 0., 0., 0.});
      robot_plan_data_.plan_execute_time = 1000000 * distance / vel;
      robot_plan_data_.plan_start_timestamp = get_microsecond_now();
    }
    logger_->info(
        "Planning: Trajectory generated with target joint positions: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ",
        target_joint_q[0], target_joint_q[1], target_joint_q[2], target_joint_q[3], target_joint_q[4],
        target_joint_q[5]);
    logger_->info("Planning: Total steps: {}, start timestamp: {}", robot_plan_data_.plan_execute_time,
                  robot_plan_data_.plan_start_timestamp);
  } else {
    if (commit) {
      std::unique_lock<std::mutex> lock(cmd_mutex_);
      robot_cmd_data_.cmd_state = MotorControlState::JOINT_POS;
      robot_cmd_data_.target_joint_q = target_joint_q;
      robot_cmd_data_.plan_target_joint_q = target_joint_q;
      for (int i = 0; i < target_joint_q.size(); i++) robot_cmd_data_.target_joint_v[i] = vel;
    } else {
      logger_->warn(
          "Invoked _plan_target_joint_q with use_planning = false and commit = false. Nothing would happen :/");
    }
    logger_->debug(
        "Planning: Moving without planning, target joint positions: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ",
        target_joint_q[0], target_joint_q[1], target_joint_q[2], target_joint_q[3], target_joint_q[4],
        target_joint_q[5]);
  }
  use_planning_.store(use_planning, std::memory_order_seq_cst);
  return true;
}
template <std::size_t DOF>
bool arm::Robot<DOF>::set_target_joint_q(const Joints<DOF>& target_joint_q, bool use_planning, double time,
                                         bool blocking) {
  if (arm_mode_.load(std::memory_order::memory_order_relaxed) != ArmMode::ONLINE) {
    logger_->warn("current state is {}, set_target_joint_q is NOT permitted",
                  ArmModeStr[arm_mode_.load(std::memory_order::memory_order_relaxed)]);
    return false;
  }
  auto ret = _plan_target_joint_q(target_joint_q, use_planning, time);
  logger_->debug("Setting target joint positions: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_joint_q[0],
                 target_joint_q[1], target_joint_q[2], target_joint_q[3], target_joint_q[4], target_joint_q[5]);
  if (blocking) {
    for (int i = 0; i < (double)BLOCK_TIMEOUT / BLOCK_SPIN_TIME; i++) {
      if (l1norm(get_current_joint_q(), target_joint_q) <= BLOCK_THRESHOLD)
        return true;
      else
        std::this_thread::sleep_for(std::chrono::milliseconds(BLOCK_SPIN_TIME));
    }
    auto current_joint_q = get_current_joint_q();
    logger_->warn(
        "set_target_joint_q blocking timeout, current joint q: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}, target joint "
        "q: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}",
        current_joint_q[0], current_joint_q[1], current_joint_q[2], current_joint_q[3], current_joint_q[4],
        current_joint_q[5], target_joint_q[0], target_joint_q[1], target_joint_q[2], target_joint_q[3],
        target_joint_q[4], target_joint_q[5]);
    return false;
  } else {
    return ret;
  }
}
template <std::size_t DOF>
bool arm::Robot<DOF>::set_target_end(const double& target_end, bool blocking) {
  if (arm_mode_.load(std::memory_order::memory_order_relaxed) != ArmMode::ONLINE) {
    logger_->warn("current state is {}, set_target_end is NOT permitted",
                  ArmModeStr[arm_mode_.load(std::memory_order::memory_order_relaxed)]);
    return false;
  }
  if (target_end < 0. || target_end > 1.) {
    logger_->warn("Target end effector position out of boundary!");
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(cmd_mutex_);
    robot_cmd_data_.target_end = target_end;
    logger_->debug("Setting target end effector position: {:.5f}", target_end);
  }
  if (blocking) {
    for (int i = 0; i < (double)BLOCK_TIMEOUT / BLOCK_SPIN_TIME; i++)
      if (get_current_end() - target_end <= BLOCK_THRESHOLD)
        return true;
      else
        std::this_thread::sleep_for(std::chrono::milliseconds(BLOCK_SPIN_TIME));
    auto current_end = get_current_end();
    logger_->warn("set_target_end blocking timeout, current end effector position: {:.5f}", current_end);
    return false;
  } else {
    return true;
  }
}
template <std::size_t DOF>
bool arm::Robot<DOF>::set_target_joint_v(const Joints<DOF>& target_joint_v) {
  if (arm_mode_.load(std::memory_order::memory_order_relaxed) != ArmMode::ONLINE) {
    logger_->warn("current state is {}, set_target_joint_v is NOT permitted",
                  ArmModeStr[arm_mode_.load(std::memory_order::memory_order_relaxed)]);
    return false;
  }
  if (!valid_joint_v(target_joint_v)) {
    logger_->warn("Target joint velocity out of boundary!");
    return false;
  }
  {
    std::lock_guard<std::mutex> write_lock(cmd_mutex_);
    robot_cmd_data_.target_joint_v = target_joint_v;
    robot_cmd_data_.cmd_state = MotorControlState::JOINT_VEL;
  }
  logger_->debug("Setting target joint velocities: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_joint_v[0],
                 target_joint_v[1], target_joint_v[2], target_joint_v[3], target_joint_v[4], target_joint_v[5]);
  return true;
}
template <std::size_t DOF>
bool arm::Robot<DOF>::add_target_joint_q(const Joints<DOF>& target_d_joint_q, bool use_planning, double time,
                                         bool blocking) {
  KDL::JntArray output;
  auto plan_target_joint_q = robot_cmd_data_.plan_target_joint_q;
  KDL::Add(joints2jntarray(plan_target_joint_q), joints2jntarray(target_d_joint_q), output);
  logger_->info("Adding target joint positions: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_d_joint_q[0],
                target_d_joint_q[1], target_d_joint_q[2], target_d_joint_q[3], target_d_joint_q[4],
                target_d_joint_q[5]);
  logger_->info("Target joint positions: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", jntarray2joints<DOF>(output)[0],
                jntarray2joints<DOF>(output)[1], jntarray2joints<DOF>(output)[2], jntarray2joints<DOF>(output)[3],
                jntarray2joints<DOF>(output)[4], jntarray2joints<DOF>(output)[5]);
  return set_target_joint_q(jntarray2joints<DOF>(output), use_planning, time, blocking);
}
template <std::size_t DOF>
bool arm::Robot<DOF>::add_target_joint_v(const Joints<DOF>& target_d_joint_v) {
  KDL::JntArray output;
  std::unique_lock<std::mutex> lock(cmd_mutex_);
  KDL::Add(joints2jntarray(robot_cmd_data_.target_joint_v), joints2jntarray(target_d_joint_v), output);
  logger_->info("Adding target joint velocities: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_d_joint_v[0],
                target_d_joint_v[1], target_d_joint_v[2], target_d_joint_v[3], target_d_joint_v[4],
                target_d_joint_v[5]);
  logger_->info("Target joint velocities: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", jntarray2joints<DOF>(output)[0],
                jntarray2joints<DOF>(output)[1], jntarray2joints<DOF>(output)[2], jntarray2joints<DOF>(output)[3],
                jntarray2joints<DOF>(output)[4], jntarray2joints<DOF>(output)[5]);
  return set_target_joint_v(jntarray2joints<DOF>(output));
}

template <std::size_t DOF>
bool arm::Robot<DOF>::set_target_joint_mit(const Joints<DOF>& target_joint_q, const Joints<DOF>& target_joint_v,
                                           const Joints<DOF>& target_joint_kp, const Joints<DOF>& target_joint_kd,
                                           const Joints<DOF>& target_joint_t) {
  if (arm_mode_.load(std::memory_order::memory_order_relaxed) != ArmMode::ONLINE) {
    logger_->warn("current state is {}, set_target_joint_mit is NOT permitted",
                  ArmModeStr[arm_mode_.load(std::memory_order::memory_order_relaxed)]);
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(cmd_mutex_);
    robot_cmd_data_.target_joint_q = target_joint_q;
    robot_cmd_data_.target_joint_v = target_joint_v;
    robot_cmd_data_.target_joint_kp = target_joint_kp;
    robot_cmd_data_.target_joint_kd = target_joint_kd;
    robot_cmd_data_.target_joint_t = target_joint_t;
    robot_cmd_data_.cmd_state = MotorControlState::JOINT_MIT;
  }
  logger_->debug("Setting target joint positions: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_joint_q[0],
                 target_joint_q[1], target_joint_q[2], target_joint_q[3], target_joint_q[4], target_joint_q[5]);
  logger_->debug("Setting target joint velocities: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_joint_v[0],
                 target_joint_v[1], target_joint_v[2], target_joint_v[3], target_joint_v[4], target_joint_v[5]);
  logger_->debug("Setting target joint kp: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_joint_kp[0],
                 target_joint_kp[1], target_joint_kp[2], target_joint_kp[3], target_joint_kp[4], target_joint_kp[5]);
  logger_->debug("Setting target joint kd: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_joint_kd[0],
                 target_joint_kd[1], target_joint_kd[2], target_joint_kd[3], target_joint_kd[4], target_joint_kd[5]);
  logger_->debug("Setting target joint t: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_joint_t[0],
                 target_joint_t[1], target_joint_t[2], target_joint_t[3], target_joint_t[4], target_joint_t[5]);
  return true;
}

template <std::size_t DOF>
bool arm::Robot<DOF>::set_target_joint_mit(const Joints<DOF>& target_joint_q, const Joints<DOF>& target_joint_v,
                                           const Joints<DOF>& target_joint_kp, const Joints<DOF>& target_joint_kd) {
  if (arm_mode_.load(std::memory_order::memory_order_relaxed) != ArmMode::ONLINE) {
    logger_->warn("current state is {}, set_target_joint_mit is NOT permitted",
                  ArmModeStr[arm_mode_.load(std::memory_order::memory_order_relaxed)]);
    return false;
  }
  Joints<DOF> current_joint_q = get_current_joint_q();
  Joints<DOF> current_joint_v = get_current_joint_v();
  {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    current_joint_q = robot_fb_data_.current_joint_q;
    current_joint_v = robot_fb_data_.current_joint_v;
  }
  auto target_joint_t = id_solver_->CartToJnt(current_joint_q, current_joint_v, Wrench{{0, 0, 0}, {0, 0, 0}});
  logger_->info("Automatically calculating target joint torques according to current joint positions and velocities");
  {
    std::unique_lock<std::mutex> lock(cmd_mutex_);
    robot_cmd_data_.target_joint_q = target_joint_q;
    robot_cmd_data_.target_joint_v = target_joint_v;
    robot_cmd_data_.target_joint_kp = target_joint_kp;
    robot_cmd_data_.target_joint_kd = target_joint_kd;
    robot_cmd_data_.target_joint_t = target_joint_t;
    robot_cmd_data_.cmd_state = MotorControlState::JOINT_MIT;
  }
  logger_->debug("Setting target joint positions: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_joint_q[0],
                 target_joint_q[1], target_joint_q[2], target_joint_q[3], target_joint_q[4], target_joint_q[5]);
  logger_->debug("Setting target joint velocities: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_joint_v[0],
                 target_joint_v[1], target_joint_v[2], target_joint_v[3], target_joint_v[4], target_joint_v[5]);
  logger_->debug("Setting target joint kp: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_joint_kp[0],
                 target_joint_kp[1], target_joint_kp[2], target_joint_kp[3], target_joint_kp[4], target_joint_kp[5]);
  logger_->debug("Setting target joint kd: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_joint_kd[0],
                 target_joint_kd[1], target_joint_kd[2], target_joint_kd[3], target_joint_kd[4], target_joint_kd[5]);
  logger_->debug("Setting target joint t: {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} ", target_joint_t[0],
                 target_joint_t[1], target_joint_t[2], target_joint_t[3], target_joint_t[4], target_joint_t[5]);
  return true;
}
