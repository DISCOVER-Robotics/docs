#include "airbot/command/command_base.hpp"

/**
 * NOTE: the following sleep is necessary to avoid sending too many
 * commands to the motor driver, which will cause the motor driver to
 * ignore some commands. 1000 microseconds is a carefully chosen and
 * tested value.
 */
#define STOP_VEL 0.1
#define FREQ_LOG_INTERVAL 10
constexpr const Joints<6> MIT_KD = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
constexpr const Joints<6> SLOW_MIT_KD = {10., 10., 10., 0.8, 0.8, 0.8};

template <std::size_t DOF>
void arm::Robot<DOF>::_update_once() {
  /**
   * Collect command from critical section
   */
  RobotCmdData<DOF> cmd_data;
  {
    std::lock_guard<std::mutex> cmd_lock(cmd_mutex_);
    cmd_data = robot_cmd_data_;
  }

  /**
   * Update control frequency
   */
  counter_ += 1;
  auto time_stamp = get_microsecond_now();
  if (time_stamp - reported_time_.load() > 1000000 * FREQ_LOG_INTERVAL) {
    logger_->info("robot is running, update frame rate: {}", (double)counter_ / FREQ_LOG_INTERVAL);
    if (logging_freq_.load(std::memory_order_relaxed) != 0)
      logger_->info("Logging queue size: {}", logging_queue_.was_size());
    counter_.store(0);
    reported_time_.store(time_stamp);
  }

  /**
   * Collect feedback data from motor drivers and calculate current pose
   */
  RobotFeedbackData<DOF> fb_tmp_data;
  {
    std::lock_guard<std::shared_mutex> fb_lock(fb_mutex_);
    fb_tmp_data = robot_fb_data_;
  }
  int index = 0;
  fb_tmp_data.time_stamp = time_stamp;
  for (auto&& motor : motor_driver_) {
    fb_tmp_data.current_joint_q[index] = motor->get_motor_pos();
    fb_tmp_data.current_joint_v[index] = motor->get_motor_spd();
    fb_tmp_data.current_joint_t[index] = motor->get_motor_current();
    fb_tmp_data.current_joint_temp[index] = motor->get_motor_temperature();
    fb_tmp_data.current_joint_err[index] = motor->get_motor_error_id();
    fb_tmp_data.response_cnt[index] = motor->get_response_count();
    index++;
  }
  if (end_motor_driver_) fb_tmp_data.current_end = e2i(end_motor_driver_->get_motor_pos(), end_effector_type_);
  fb_tmp_data.current_pose = fk_solver_->JntToCart(fb_tmp_data.current_joint_q);

  // check arm_mode changes
  ArmMode arm_mode = arm_mode_.load(std::memory_order_seq_cst);
  // check robot is stopped
  auto difference = l1norm(fb_tmp_data.current_joint_v, {0., 0., 0., 0., 0., 0.});
  auto last_is_stopped = fb_tmp_data.is_robot_stopped;
  fb_tmp_data.is_robot_stopped = difference < STOP_VEL;
  // validate the joint state
  // if (joint_safe_detect_.load() && arm_mode != ArmMode::DEMONSTRATE && arm_mode != ArmMode::RECORDING &&
  //     !valid_joint_q(fb_tmp_data.current_joint_q) && arm_mode != ArmMode::ERROR && arm_mode != ArmMode::ERROR) {
  //   logger_->error("joint out of limit, into ERROR state");
  //   auto ret = _on_mode_change(arm_mode, ArmMode::ERROR, cmd_data);
  //   if (!ret) logger_->error("failed to change mode to ERROR");
  //   _write_fb_data(fb_tmp_data);
  //   return;
  // }
  // check error id
  for (size_t i = 0; i < fb_tmp_data.current_joint_err.size(); i++) {
    if (fb_tmp_data.current_joint_err[i] != 0 && arm_mode != ArmMode::ERROR) {
      logger_->error("motor{} is error, error code is {}", i, fb_tmp_data.current_joint_err[i]);
      auto ret = _on_mode_change(arm_mode, ArmMode::ERROR, cmd_data);
      if (!ret) logger_->error("failed to change mode to ERROR");
      _write_fb_data(fb_tmp_data);
      return;
    }
  }
  // update last_valid_joint_q
  if (arm_mode != ArmMode::ERROR && safe_joint_q(fb_tmp_data.current_joint_q))
    fb_tmp_data.last_valid_joint_q = fb_tmp_data.current_joint_q;

  /**
   * Automatically triggered mode changes
   */
  auto target_arm_mode = arm_mode;
  switch (arm_mode) {
    case ArmMode::ERROR:
      if (cmd_data.recover_request && fb_tmp_data.is_robot_stopped && safe_joint_q(fb_tmp_data.current_joint_q)) {
        logger_->info("robot is recovered to last valid state");
        target_arm_mode = ArmMode::ONLINE;
        cmd_data.recover_request = false;
      }
      break;
    case ArmMode::OFFLINE:
      if (cmd_data.replay_request) {
        cmd_data.replay_request = false;
        target_arm_mode = ArmMode::REPLAY_REACHING;
      }
      break;
    case ArmMode::REPLAY_REACHING:
      if (l1norm(fb_tmp_data.current_joint_q, recorded_data_.q_records[0]) < 0.01) {
        target_arm_mode = ArmMode::REPLAY_WAITING;
      }
      break;
    case ArmMode::REPLAY_WAITING:
      if (fb_tmp_data.time_stamp - replay_inplace_time_ > 1000000) {
        target_arm_mode = ArmMode::REPLAYING;
      }
    case ArmMode::REPLAYING:
      if (replay_index_.load() == replay_data_.q_records.size()) {
        logger_->info("replay finished");
        target_arm_mode = ArmMode::OFFLINE;
      }
      break;
    default:
      break;
  }

  if (_check_mode_change(arm_mode, target_arm_mode) && _on_mode_change(arm_mode, target_arm_mode, cmd_data)) {
    _write_fb_data(fb_tmp_data);
    return;
  }

  /**
   * Snap triggered mode changes
   */
  auto end_snap_mode = interface_board_end_->get_snap_mode();
  if (end_snap_mode != SnapMode::SNAP_RELEASE) logger_->info("end snap mode: {}", SnapModeStr[end_snap_mode]);
  auto base_snap_mode = interface_board_base_->get_snap_mode();
  if (base_snap_mode != SnapMode::SNAP_RELEASE) logger_->info("base snap mode: {}", SnapModeStr[base_snap_mode]);

  if (arm_mode != ArmMode::ERROR && end_snap_mode != SnapMode::SNAP_RELEASE) switch (arm_mode | end_snap_mode) {
      // Long press or double press end snap to change arm mode
      case ArmMode::ONLINE | SnapMode::SNAP_DOUBLE_PRESS:
      case ArmMode::DEMONSTRATE | SnapMode::SNAP_DOUBLE_PRESS:
      case ArmMode::RECORDING | SnapMode::SNAP_DOUBLE_PRESS:
        target_arm_mode = ArmMode::OFFLINE;
        break;
      case ArmMode::ONLINE | SnapMode::SNAP_LONG_PRESS:
      case ArmMode::OFFLINE | SnapMode::SNAP_LONG_PRESS:
        target_arm_mode = ArmMode::DEMONSTRATE;
        break;
      case ArmMode::OFFLINE | SnapMode::SNAP_DOUBLE_PRESS:
      case ArmMode::DEMONSTRATE | SnapMode::SNAP_LONG_PRESS:
      case ArmMode::RECORDING | SnapMode::SNAP_LONG_PRESS:
        target_arm_mode = ArmMode::ONLINE;
        break;

      // Short press end snap to change end effector state
      case ArmMode::ONLINE | SnapMode::SNAP_SHORT_PRESS:
      case ArmMode::DEMONSTRATE | SnapMode::SNAP_SHORT_PRESS:
      case ArmMode::RECORDING | SnapMode::SNAP_SHORT_PRESS: {
        // TODO fail to locate why target_end_ is not updated
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        robot_cmd_data_.target_end = fabs(fb_tmp_data.current_end) > 0.5 ? 0. : 1.;
      }
        logger_->info("end effector ({}) state switched from {} to {}", end_effector_type_, fb_tmp_data.current_end,
                      cmd_data.target_end);
        break;
      case ArmMode::OFFLINE | SnapMode::SNAP_SHORT_PRESS:
        logger_->warn("offline mode does not support end effector control");
        break;

      default:
        logger_->error("In arm mode {}, action {} is invalid", ArmModeStr[arm_mode], SnapModeStr[end_snap_mode]);
        break;
    }
  else if (arm_mode == ArmMode::ERROR && end_snap_mode != SnapMode::SNAP_RELEASE) {
    logger_->warn("robot is in ERROR state, ignore end snap signal {}", end_snap_mode);
  }

  if (arm_mode != ArmMode::ERROR && base_snap_mode != SnapMode::SNAP_RELEASE) switch (arm_mode | base_snap_mode) {
      case ArmMode::ONLINE | SnapMode::SNAP_DOUBLE_PRESS:
      case ArmMode::DEMONSTRATE | SnapMode::SNAP_DOUBLE_PRESS:
      case ArmMode::RECORDING | SnapMode::SNAP_DOUBLE_PRESS:
      case ArmMode::OFFLINE | SnapMode::SNAP_DOUBLE_PRESS:
      case ArmMode::ONLINE | SnapMode::SNAP_LONG_PRESS:
      case ArmMode::DEMONSTRATE | SnapMode::SNAP_LONG_PRESS:
      case ArmMode::RECORDING | SnapMode::SNAP_LONG_PRESS:
      case ArmMode::OFFLINE | SnapMode::SNAP_LONG_PRESS:
      case ArmMode::ONLINE | SnapMode::SNAP_SHORT_PRESS:
        break;
      case ArmMode::DEMONSTRATE | SnapMode::SNAP_SHORT_PRESS:
        target_arm_mode = ArmMode::RECORDING;
        break;
      case ArmMode::RECORDING | SnapMode::SNAP_SHORT_PRESS:
        target_arm_mode = ArmMode::DEMONSTRATE;
        break;
      case ArmMode::OFFLINE | SnapMode::SNAP_SHORT_PRESS:
        cmd_data.replay_request = true;
        {
          std::lock_guard<std::mutex> lock(cmd_mutex_);
          robot_cmd_data_.replay_request = true;
        }
        break;
      default:
        logger_->error("never be here 2");
        break;
    }
  else if (arm_mode == ArmMode::ERROR && base_snap_mode != SnapMode::SNAP_RELEASE) {
    logger_->warn("robot is in ERROR state, ignore base snap signal {}", base_snap_mode);
  }

  if (_check_mode_change(arm_mode, target_arm_mode) && _on_mode_change(arm_mode, target_arm_mode, cmd_data)) {
    _write_fb_data(fb_tmp_data);
    return;
  }

  /**
   * External observations and actions have been properly handled, now proceed to actions
   */

  // check motorcontrol state change
  if (cmd_data.cmd_state != fb_tmp_data.current_state) {
    logger_->info("control mode changed, from: {} to: {}", MotorControlStateStr[fb_tmp_data.current_state],
                  MotorControlStateStr[cmd_data.cmd_state]);
    for (auto&& i : motor_driver_) {
      switch (cmd_data.cmd_state) {
        case MotorControlState::JOINT_POS:
          i->set_motor_control_mode(MotorDriver::MotorControlMode_e::POS);
          break;
        case MotorControlState::JOINT_VEL:
          i->set_motor_control_mode(MotorDriver::MotorControlMode_e::SPD);
          break;
        case MotorControlState::JOINT_MIT:
          i->set_motor_control_mode(MotorDriver::MotorControlMode_e::MIT);
          break;
        default:
          logger_->error("never be here 3 ");
      }
      std::this_thread::sleep_for(std::chrono::microseconds(MAGIC_DELAY));
    }
    fb_tmp_data.current_state = cmd_data.cmd_state;
    // start next round, not sync
    _write_fb_data(fb_tmp_data);
    return;
  }

  // Control state in cmd and feedback are the same
  MotorControlState control_state = fb_tmp_data.current_state;
  bool use_planning = use_planning_.load(std::memory_order_relaxed);

  /**
   * If planning is enabled and plan_params are valid, execute the planned trajectory
   * cmd_data will be updated according to the planned trajectory
   */
  if (control_state == MotorControlState::JOINT_POS and use_planning) {
    if ((get_microsecond_now()) - robot_plan_data_.plan_start_timestamp >=
        robot_plan_data_.plan_execute_time - LOOK_AHEAD) {
      use_planning_.store(false, std::memory_order_relaxed);
      logger_->info("execution with simple planning in JOINT_POS is finished");
    } else {
      auto results = plan_infer(robot_plan_data_.plan_params,
                                ((double)(get_microsecond_now()) - robot_plan_data_.plan_start_timestamp + LOOK_AHEAD) /
                                    robot_plan_data_.plan_execute_time);
      cmd_data.target_joint_q = results;
      // NOTE: planned data will be used, to keep consistency with normal mode, robot_cmd_data_ should be updated
      _write_cmd_data(cmd_data);
    }
  }

  /**
   * Calculating required torque for gravity compensation
   */

  auto id_solver_coef = std::vector<double>{0.85, 0.85, 0.75, 0.9, 0.9, 0.9};
  if (end_effector_type_ == "newteacher" || end_effector_type_ == "teacherv2") {
    id_solver_coef[3] = 1.5;
    id_solver_coef[4] = 1.5;
  }
  // generate torque command when control_state is TORQUE
  if (arm_mode == ArmMode::DEMONSTRATE || arm_mode == ArmMode::RECORDING) {
    if (control_state != MotorControlState::JOINT_MIT) {
      logger_->error(
          "arm mode DEMONSTRATE and RECORDING should not run with states other than MotorControlState::JOINT_MIT");
      _write_fb_data(fb_tmp_data);
      return;
    }
    auto ts =
        id_solver_->CartToJnt(fb_tmp_data.current_joint_q, fb_tmp_data.current_joint_v, Wrench{{0, 0, 0}, {0, 0, 0}});
    for (int i = 0; i < ts.size(); i++) cmd_data.target_joint_t[i] = id_solver_coef[i] * ts[i];
  }

  /**
   * Logging before executing the command
   */
  if (logging_freq_.load(std::memory_order_relaxed) != 0 &&
      time_stamp - last_logging_time_.load() > 1000000 / logging_freq_.load()) {
    logging_queue_.push(LoggingData<6>{get_sn(), cmd_data, fb_tmp_data});
    last_logging_time_ = time_stamp;
  }
  if (!is_init_.load()) {
    _write_fb_data(fb_tmp_data);
    _write_cmd_data(cmd_data);
    return;
  }
  /**
   * Actual command execution
   */
  if (end_motor_driver_) {
    if (end_effector_type_ == "gripper") switch (arm_mode) {
        case ArmMode::ONLINE:
        case ArmMode::RECORDING:
        case ArmMode::DEMONSTRATE:
        case ArmMode::OFFLINE:
        case ArmMode::REPLAY_REACHING:
        case ArmMode::REPLAY_WAITING:
          end_motor_driver_->MotorMitModeCmd(i2e(cmd_data.target_end, "gripper"), 0, 2, 0.1, 0);
          break;
        case ArmMode::REPLAYING:
          end_motor_driver_->MotorMitModeCmd(i2e(recorded_data_.end_records[replay_index_.load()], "gripper"), 0, 2,
                                             0.1, 0);
          break;
        case ArmMode::ERROR:
          end_motor_driver_->MotorMitModeCmd(0, 0, 0, 0, 0);
          break;
      }
    else if (end_effector_type_ == "teacher" || end_effector_type_ == "newteacher" || end_effector_type_ == "teacherv2")
      switch (arm_mode) {
        case ArmMode::ONLINE:
        case ArmMode::RECORDING:
        case ArmMode::DEMONSTRATE:
        case ArmMode::OFFLINE:
        case ArmMode::REPLAYING:
        case ArmMode::REPLAY_REACHING:
        case ArmMode::REPLAY_WAITING:
        case ArmMode::ERROR:
          end_motor_driver_->MotorMitModeCmd(0, 0, 0, 0, 0);
          break;
      }
    else if (end_effector_type_ == "yinshi")
      switch (arm_mode) {
        case ArmMode::ONLINE:
        case ArmMode::RECORDING:
        case ArmMode::DEMONSTRATE:
        case ArmMode::OFFLINE:
        case ArmMode::REPLAY_REACHING:
        case ArmMode::REPLAY_WAITING:
          end_motor_driver_->MotorPosModeCmd(i2e(cmd_data.target_end, "gripper"), 1.);  // TODO 1. is a temporary value
          break;
        case ArmMode::REPLAYING:
          end_motor_driver_->MotorPosModeCmd(i2e(recorded_data_.end_records[replay_index_.load()], "gripper"),
                                             1.);  // TODO 1. is a temporary value
          break;
        case ArmMode::ERROR:
          end_motor_driver_->MotorMitModeCmd(0, 0, 0, 0, 0);
          break;
      }
    std::this_thread::sleep_for(std::chrono::microseconds(MAGIC_DELAY));
  }
  switch (arm_mode) {
    case ArmMode::ONLINE: {
      int _idx = 0;
      for (auto&& i : motor_driver_) {
        switch (control_state) {
          case MotorControlState::INIT:
            // no incomming command, do nothing
            break;
          case MotorControlState::JOINT_POS:
            i->MotorPosModeCmd(cmd_data.target_joint_q[_idx], cmd_data.target_joint_v[_idx]);
            break;
          case MotorControlState::JOINT_VEL:
            i->MotorSpdModeCmd(cmd_data.target_joint_v[_idx]);
            break;
          case MotorControlState::JOINT_MIT:
            i->MotorMitModeCmd(cmd_data.target_joint_q[_idx], cmd_data.target_joint_v[_idx],
                               cmd_data.target_joint_kp[_idx], cmd_data.target_joint_kd[_idx],
                               cmd_data.target_joint_t[_idx]);
            break;
          default:
            logger_->error("never be here 4");
        }
        _idx++;
        std::this_thread::sleep_for(std::chrono::microseconds(MAGIC_DELAY));
      }
      if (last_is_stopped && !fb_tmp_data.is_robot_stopped) {
        interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                      (uint32_t)COLOR_GREEN_FLASHING);
      } else if (!last_is_stopped && fb_tmp_data.is_robot_stopped) {
        interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                      (uint32_t)COLOR_GREEN_CONSTANT);
      }
    } break;
    case ArmMode::RECORDING:
      recorded_data_.q_records.push_back(fb_tmp_data.current_joint_q);
      recorded_data_.time_records.push_back(fb_tmp_data.time_stamp);
      recorded_data_.end_records.push_back(fb_tmp_data.current_end);
    case ArmMode::DEMONSTRATE:
      if (control_state != MotorControlState::JOINT_MIT) {
        logger_->error(
            "arm mode DEMONSTRATE or RECORDING should not run with states other than MotorControlState::JOINT_MIT");
        _write_fb_data(fb_tmp_data);
        return;
      } else {
        int _idx = 0;
        auto slow = slow_joint_q(fb_tmp_data.current_joint_q);
        bool all_slow = false;
        for (auto&& i : slow)
          if (i) all_slow = true;

        for (auto&& motor : motor_driver_) {
          if (all_slow) {
            motor->MotorMitModeCmd(0, 0, 0, SLOW_MIT_KD[_idx], cmd_data.target_joint_t[_idx]);
          } else {
            motor->MotorMitModeCmd(0, 0, 0, MIT_KD[_idx], cmd_data.target_joint_t[_idx]);
          }
          _idx++;
          std::this_thread::sleep_for(std::chrono::microseconds(MAGIC_DELAY));
        }
      }
      break;
    case ArmMode::OFFLINE:
      if (control_state != MotorControlState::JOINT_POS) {
        logger_->error("arm mode OFFLINE should not run with states other than MotorControlState::JOINT_POS");
        _write_fb_data(fb_tmp_data);
        return;
      } else {
        int _idx = 0;
        for (auto&& i : motor_driver_) {
          i->MotorPosModeCmd(cmd_data.target_joint_q[_idx], joint_vel_limit_[_idx]);
          _idx++;
          std::this_thread::sleep_for(std::chrono::microseconds(MAGIC_DELAY));
        }
      }
      break;
    case ArmMode::REPLAY_REACHING:
    case ArmMode::REPLAY_WAITING: {
      int _idx = 0;
      for (auto&& i : motor_driver_) {
        i->MotorPosModeCmd(cmd_data.target_joint_q[_idx], joint_vel_limit_[_idx]);
        _idx++;
        std::this_thread::sleep_for(std::chrono::microseconds(MAGIC_DELAY));
      }
    } break;

    case ArmMode::REPLAYING:
      if (control_state != MotorControlState::JOINT_POS) {
        logger_->error("arm mode OFFLINE should not run with states other than MotorControlState::JOINT_POS");
        _write_fb_data(fb_tmp_data);
        return;
      } else {
        int _idx = 0;
        for (auto&& i : motor_driver_) {
          i->MotorPosModeCmd(replay_data_.q_records[replay_index_.load()][_idx], joint_vel_limit_[_idx]);
          _idx++;
          std::this_thread::sleep_for(std::chrono::microseconds(MAGIC_DELAY));
        }
        replay_index_ += 1;
      }
      break;
    case ArmMode::ERROR: {
      if (cmd_data.recover_request) {
        int _idx = 0;
        for (auto&& i : motor_driver_) {
          i->MotorPosModeCmd(fb_tmp_data.last_valid_joint_q[_idx], joint_vel_limit_[_idx] * RECOVERY_SPD_RATIO);
          _idx++;
          std::this_thread::sleep_for(std::chrono::microseconds(MAGIC_DELAY));
        }
      }
    } break;
  }
  _write_fb_data(fb_tmp_data);
}
