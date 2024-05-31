#include "airbot/command/command_base.hpp"

#include "airbot/modules/motors/od_motor_driver.hpp"

template <std::size_t DOF>
std::unordered_map<std::string, std::pair<double, double>> arm::Robot<DOF>::end_limits = {
    {"teacher", {-0.15, -4}},
    {"gripper", {0.15, -1.39}},
    {"yinshi", {0.0, 100.0}},
    {"newteacher", {-0.15, -4}},
    {"teacherv2", {0, -2.744292}}};
template <std::size_t DOF>
arm::Robot<DOF>::Robot(std::string urdf_path, std::string interface_s, std::string direction, double vel_limit,
                       std::string end_mode, std::string forearm, bool factory)
    : fk_solver_(std::make_unique<arm::AnalyticFKSolver<DOF>>(urdf_path)),
      ik_solver_(std::make_unique<arm::AnalyticIKSolver<DOF>>(urdf_path)),
      ikv_solver_(std::make_unique<arm::ChainIKVSolver<DOF>>(urdf_path)),
      id_solver_(std::make_unique<arm::ChainIDSolver<DOF>>(urdf_path, direction)),
      is_running_(true),
      logging_freq_(0),
      joint_safe_detect_(false),
      last_logging_time_(0),
      is_init_(false),
      logging_queue_(QUEUE_SIZE),
      replay_inplace_time_(0),
      use_planning_(true) {
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      (std::string("logs/airbot_play-") + get_timestring() + ".log").c_str(), 1024 * 1024, 10, false));
  logger_ = setup_logger(sinks);
  spdlog::flush_every(std::chrono::seconds(1));
  spdlog::flush_on(spdlog::level::info);
  logger_->set_level(spdlog::level::info);
  logger_->info("================== AIRBOT PLAY ==================");

  auto interface = interface_s.c_str();
  bool is_vcan = interface[0] == 'v';
  if (is_vcan) {
    // TODO: will be removed, currently it turn "vcanX" to "canX", why net device can recognize it
    interface += 1;
  }

  assert(fk_solver_->get_num_joints() == ik_solver_->get_num_joints());
  robot_cmd_data_.init(ik_solver_->get_num_joints());
  robot_fb_data_.init(ik_solver_->get_num_joints());
  recorded_data_.init();
  replay_data_.init();
  //  for (auto& motor : motor_driver_) response_cnt.push_back(motor->get_response_count());

  if (vel_limit > MAX_VEL_LIMIT) {
    logger_->warn("joint velocity limit {} is too high, set to pi", vel_limit);
    vel_limit = MAX_VEL_LIMIT;
  }
  for (auto&& i : joint_vel_limit_) i = vel_limit;

  _set_state(MotorControlState::JOINT_POS);

  // Initialize motors. end_motor_driver_ is responsible for end effector
  // (demonstrator or gripper)
  interface_board_base_ = std::make_unique<arm::InterfaceBoardBase>(000, interface);

  auto status = true;
  if (!is_vcan) status = interface_board_base_->Init();
  if (!status) {
    logger_->error("Base interface board initialization failed");
    throw std::runtime_error("Base interface board initialization failed");
  }
  arm_mode_.store(ArmMode::ONLINE, std::memory_order_relaxed);
  for (size_t i = 0; i < fk_solver_->get_num_joints(); i++) {
    if (i < 3) {
      motor_driver_[i] = MotorDriver::MotorCreate(i + 1, interface, "OD");
      motor_driver_[i]->MotorMitModeCmd(0, 0, 0, 10, 0);  // TODO FIXME this is a hack to initialize motor info
    } else {
      if (is_vcan)
        motor_driver_[i] = MotorDriver::MotorCreate(i + 1, interface, "OD");
      else {
        motor_driver_[i] = MotorDriver::MotorCreate(i + 1, interface, forearm);
        if (forearm == "OD") {
          motor_driver_[i]->MotorMitModeCmd(0, 0, 0, 1, 0);  // TODO FIXME this is a hack to initialize motor info
        }
      }
    }
    if (!is_vcan) status = motor_driver_[i]->MotorInit();
    if (!status) {
      for (int j = 0; j < i; j++) motor_driver_[j]->MotorDeInit();
      interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                    (uint32_t)COLOR_RED_CONSTANT);
      logger_->error("Motor {} initialization failed", i + 1);
      throw std::runtime_error("Motor " + std::to_string(i + 1) + " initialization failed");
    }
  }
  end_effector_type_ = end_mode;
  if (end_effector_type_ == "newteacher" || end_effector_type_ == "teacherv2") {
    end_motor_driver_ = MotorDriver::MotorCreate(7, interface, "OD");
  } else if (end_effector_type_ == "gripper" || end_effector_type_ == "teacher") {
    end_motor_driver_ = MotorDriver::MotorCreate(7, interface, "DM");
  } else if (end_effector_type_ == "yinshi") {
    end_motor_driver_ = MotorDriver::MotorCreate(7, interface, "485");
  }
  if (!is_vcan && end_motor_driver_) status = end_motor_driver_->MotorInit();

  if (!status) {
    for (auto&& i : motor_driver_) i->MotorDeInit();
    interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                  (uint32_t)COLOR_RED_CONSTANT);
    logger_->error("End motor initialization failed");
    throw std::runtime_error("End motor initialization failed");
  }

  // the mode of end_motor_driver_ is set to MIT mode after init
  // to ensure end effector **always** work in MIT mode
  if (end_motor_driver_) end_motor_driver_->set_motor_control_mode(MotorDriver::MotorControlMode_e::POS);

  interface_board_end_ = std::make_unique<arm::InterfaceBoardEnd>(0x008, interface);
  if (!is_vcan) status = interface_board_end_->Init();
  if (!status) {
    for (int i = 0; i < motor_driver_.size(); i++) motor_driver_[i]->MotorDeInit();
    if (end_motor_driver_) end_motor_driver_->MotorDeInit();
    interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                  (uint32_t)COLOR_RED_CONSTANT);
    logger_->error("End interface board initialization failed");
    throw std::runtime_error("End interface board initialization failed");
  }
  std::string arm_sn_code = interface_board_base_->get_arm_sn_code();
  for (uint8_t i = 0; i < 3; i++) {
    status &= (motor_driver_[i]->get_arm_sn_code() == arm_sn_code);
  }
  status &= (interface_board_end_->get_arm_sn_code() == arm_sn_code);
  if (!status) {
    for (int i = 0; i < motor_driver_.size(); i++) motor_driver_[i]->MotorDeInit();
    if (end_motor_driver_) end_motor_driver_->MotorDeInit();
    interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                  (uint32_t)COLOR_RED_CONSTANT);
    logger_->error("Arm sn code not match");
    throw std::runtime_error("Arm sn code not match");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Initialize targets with zeros
  // Abort program if any joint is out of range
  for (int motor_id = 1; motor_id <= motor_driver_.size(); motor_id++) {
    auto current_joint_q = robot_fb_data_.current_joint_q;
    if (current_joint_q[motor_id - 1] > motor_driver_[motor_id - 1]->MotorBoundary()[1] ||
        current_joint_q[motor_id - 1] < motor_driver_[motor_id - 1]->MotorBoundary()[0]) {
      for (int i = 0; i < motor_id; i++) motor_driver_[i]->MotorDeInit();
      if (end_motor_driver_) end_motor_driver_->MotorDeInit();
      interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                    (uint32_t)COLOR_RED_CONSTANT);
      logger_->error("Joint {} is out of range", motor_id);
      throw std::runtime_error("Joint " + std::to_string(motor_id) + " is out of range");
    }
  }
  interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                (uint32_t)COLOR_GREEN_CONSTANT);

  // Update current joint and end effector motor information and
  // trigger motor state update
  main_update_thread_ = std::thread([this]() {
    while (is_running_.load(std::memory_order_relaxed)) {
      this->_update_once();
    }
  });

  auto read_flag = false;
  for (int i = 0; i < 100; i++)
    if (get_current_joint_q()[0] == 0 && get_current_joint_q()[1] == 0 && get_current_joint_q()[2] == 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    else
      read_flag = true;
  if (!read_flag) {
    auto current_joint_q = get_current_joint_q();
    for (int i = 0; i < motor_driver_.size(); i++) motor_driver_[i]->MotorDeInit();
    if (end_motor_driver_) end_motor_driver_->MotorDeInit();
    interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                  (uint32_t)COLOR_RED_CONSTANT);

    logger_->error("Current joint positions: {}, {}, {}", current_joint_q[0], current_joint_q[1], current_joint_q[2]);
    logger_->error("Failed to read joint position");
    is_running_.store(false, std::memory_order_relaxed);
    main_update_thread_.join();
    throw std::runtime_error("Failed to read joint position");
  }

  is_init_.store(true);
  if (!factory) {
    auto ret = set_target_joint_q({0, 0, 0, 0, 0, 0}, true, 0.2, true);
    if (!ret) {
      for (int i = 0; i < motor_driver_.size(); i++) motor_driver_[i]->MotorDeInit();
      if (end_motor_driver_) end_motor_driver_->MotorDeInit();
      interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                    (uint32_t)COLOR_RED_CONSTANT);
      logger_->error("Failed to move to initial position");
      throw std::runtime_error("Failed to move to initial position");
    }
  }
  if (!factory) joint_safe_detect_.store(true);

  logging_thread_ = std::thread([this]() {
    while (is_running_.load(std::memory_order_relaxed)) {
      if (logging_freq_.load(std::memory_order_relaxed) != 0) {
        if (this->logging_queue_.was_full()) logger_->warn("Logging queue is full");
        if (!this->logging_queue_.was_empty()) this->kibana_logger_.push_remote_log_once(this->logging_queue_.pop());
        std::this_thread::sleep_for(std::chrono::milliseconds(uint32_t(1. / logging_freq_.load() * 1000)));
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  });
  logger_->info("================== Start finished ==================");
}
template <std::size_t DOF>
arm::Robot<DOF>::~Robot() {
  logger_->info("Robot shutting down");
  change_mode(ArmMode::ONLINE);

  set_target_joint_q({0, 0, 0, 0, 0, 0}, true, 0.2, true);
  joint_safe_detect_.store(false);
  set_max_current({1, 2, 3, 1, 1, 1});
  set_target_joint_q({0, motor_driver_[1]->MotorBoundary()[1], motor_driver_[2]->MotorBoundary()[0], 0, 0, 0}, true,
                     0.2, true);

  for (int i = 0; i < 3; i++) motor_driver_[i]->MotorLock();
  is_running_.store(false, std::memory_order_relaxed);
  main_update_thread_.join();
  logging_thread_.join();
  interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                (uint32_t)COLOR_WHITE_BREATHING);
  if (end_motor_driver_) end_motor_driver_->MotorDeInit();
  for (auto&& i : motor_driver_) i->MotorDeInit();
}
template <std::size_t DOF>
bool arm::Robot<DOF>::valid_target_pose(const Frame& target_pose) const {
  try {
    auto results = ik_solver_->CartToJnt(target_pose);
    std::any_of(results.begin(), results.end(), [this](auto& i) { return this->valid_joint_q(i); });
    return false;
  } catch (const arm::InvalidTarget& e) {
    logger_->warn(e.what());
    return false;
  }
}

template <std::size_t DOF>
bool arm::Robot<DOF>::safe_joint_q(const Joints<DOF>& target_joint_q) const {
  for (uint8_t i = 0; i < motor_driver_.size(); i++) {
    auto boundaries = motor_driver_[i]->MotorBoundary();
    if (target_joint_q[i] < boundaries[0] + 2 * M_PI / 180 || target_joint_q[i] > boundaries[1] - 2 * M_PI / 180) {
      return false;
    }
  }
  return true;
}

template <std::size_t DOF>
array<bool, 6> arm::Robot<DOF>::slow_joint_q(const Joints<DOF>& target_joint_q) const {
  return {(target_joint_q[0] < motor_driver_[0]->MotorBoundary()[0] + 10 * M_PI / 180 ||
           target_joint_q[0] > motor_driver_[0]->MotorBoundary()[1] - 10 * M_PI / 180),
          (target_joint_q[1] < motor_driver_[1]->MotorBoundary()[0] + 10 * M_PI / 180 ||
           target_joint_q[1] > motor_driver_[1]->MotorBoundary()[1] - 2 * M_PI / 180),
          (target_joint_q[2] < motor_driver_[2]->MotorBoundary()[0] + 2 * M_PI / 180 ||
           target_joint_q[2] > motor_driver_[2]->MotorBoundary()[1] - 45 * M_PI / 180),
          (target_joint_q[3] < motor_driver_[3]->MotorBoundary()[0] + 30 * M_PI / 180 ||
           target_joint_q[3] > motor_driver_[3]->MotorBoundary()[1] - 30 * M_PI / 180),
          (target_joint_q[4] < motor_driver_[4]->MotorBoundary()[0] + 30 * M_PI / 180 ||
           target_joint_q[4] > motor_driver_[4]->MotorBoundary()[1] - 30 * M_PI / 180),
          (target_joint_q[5] < motor_driver_[5]->MotorBoundary()[0] + 30 * M_PI / 180 ||
           target_joint_q[5] > motor_driver_[5]->MotorBoundary()[1] - 30 * M_PI / 180)};
}

template <std::size_t DOF>
bool arm::Robot<DOF>::valid_joint_q(const Joints<DOF>& target_joint_q) const {
  for (uint8_t i = 0; i < motor_driver_.size(); i++) {
    auto boundaries = motor_driver_[i]->MotorBoundary();
    if (target_joint_q[i] < boundaries[0] + 1 * M_PI / 180 || target_joint_q[i] > boundaries[1] - 1 * M_PI / 180) {
      return false;
    }
  }
  return true;
}

template <std::size_t DOF>
bool arm::Robot<DOF>::valid_joint_v(const Joints<DOF>& target_joint_v) const {
  for (uint8_t i = 0; i < motor_driver_.size(); i++)
    if (std::abs(target_joint_v[i]) > joint_vel_limit_[i]) return false;
  return true;
}

template <std::size_t DOF>
bool arm::Robot<DOF>::_check_mode_change(const ArmMode& current_mode, const ArmMode& cmd_mode) {
  if (current_mode == cmd_mode) return true;
  switch (current_mode >> cmd_mode) {
    // --> ONLINE
    case ArmMode::DEMONSTRATE >> ArmMode::ONLINE:
    case ArmMode::OFFLINE >> ArmMode::ONLINE:
      return true;

    // --> DEMONSTRATE
    case ArmMode::RECORDING >> ArmMode::DEMONSTRATE:
    case ArmMode::ONLINE >> ArmMode::DEMONSTRATE:
    case ArmMode::OFFLINE >> ArmMode::DEMONSTRATE:
      return true;

    // --> OFFLINE
    case ArmMode::ONLINE >> ArmMode::OFFLINE:
    case ArmMode::DEMONSTRATE >> ArmMode::OFFLINE:
      if (recorded_data_.q_records.size() > 0)
        return true;
      else {
        logger_->warn("no recorded data, cannot change mode from {} to {}", ArmModeStr[current_mode],
                      ArmModeStr[cmd_mode]);
        interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                      (uint32_t)COLOR_PURPLE_FLASHING);
        std::thread([this]() {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                        (uint32_t)COLOR_GREEN_CONSTANT);
        }).detach();
        return false;
      }
    case ArmMode::REPLAYING >> ArmMode::OFFLINE:
      return true;

    // --> RECORDING
    case ArmMode::DEMONSTRATE >> ArmMode::RECORDING:
      return true;

    // From recording
    case ArmMode::RECORDING >> ArmMode::ONLINE:
    case ArmMode::RECORDING >> ArmMode::OFFLINE:
      logger_->warn("Recording, cannot change mode from {} to {}", ArmModeStr[current_mode], ArmModeStr[cmd_mode]);
      interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                    (uint32_t)COLOR_BLUE_FLASHING);
      std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                      (uint32_t)COLOR_BLUE_CONSTANT);
      }).detach();
      return false;

    // Replay cases
    case ArmMode::OFFLINE >> ArmMode::REPLAY_REACHING:
      return true;
    case ArmMode::REPLAY_REACHING >> ArmMode::REPLAY_WAITING:
      return l1norm(get_current_joint_q(), recorded_data_.q_records[0]) < 0.01;
    case ArmMode::REPLAY_WAITING >> ArmMode::REPLAYING:
      return get_microsecond_now() - replay_inplace_time_ > 1000000;

    // Error cases
    case ArmMode::ERROR >> ArmMode::ONLINE: {
      std::shared_lock<std::shared_mutex> lock(fb_mutex_);
      if (safe_joint_q(robot_fb_data_.current_joint_q))
        return true;
      else
        return false;
      break;
    }
    case ArmMode::DEMONSTRATE >> ArmMode::ERROR:
    case ArmMode::RECORDING >> ArmMode::ERROR:
    case ArmMode::OFFLINE >> ArmMode::ERROR:
    case ArmMode::REPLAY_REACHING >> ArmMode::ERROR:
    case ArmMode::REPLAYING >> ArmMode::ERROR:
    case ArmMode::ONLINE >> ArmMode::ERROR:
      return true;

    default:
      logger_->error("changing mode is not allowed: {} >> {}", ArmModeStr[current_mode], ArmModeStr[cmd_mode]);
      return false;
  }
}

template <std::size_t DOF>
bool arm::Robot<DOF>::_on_mode_change(const ArmMode& current_mode, const ArmMode& cmd_mode,
                                      RobotCmdData<DOF>& robot_cmd_data) {
  if (current_mode == cmd_mode) {
    return false;
  } else {
    logger_->info("changing mode from {} to {}", ArmModeStr[current_mode], ArmModeStr[cmd_mode]);
    auto current_joint_q = get_current_joint_q();
    switch (current_mode >> cmd_mode) {
      // --> ONLINE
      case ArmMode::DEMONSTRATE >> ArmMode::ONLINE:
      case ArmMode::OFFLINE >> ArmMode::ONLINE:
        robot_cmd_data.cmd_state = MotorControlState::JOINT_POS;
        robot_cmd_data.plan_target_joint_q = current_joint_q;
        robot_cmd_data.target_joint_q = current_joint_q;
        robot_cmd_data.target_joint_v = joint_vel_limit_;
        interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                      (uint32_t)COLOR_GREEN_CONSTANT);
        break;

      // --> DEMONSTRATE
      case ArmMode::RECORDING >> ArmMode::DEMONSTRATE:
        replay_data_ = recorded_data_;
      case ArmMode::ONLINE >> ArmMode::DEMONSTRATE:
      case ArmMode::OFFLINE >> ArmMode::DEMONSTRATE:
        robot_cmd_data.cmd_state = MotorControlState::JOINT_MIT;
        interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                      (uint32_t)COLOR_CYAN_CONSTANT);
        break;

      // --> OFFLINE
      case ArmMode::ONLINE >> ArmMode::OFFLINE:
      case ArmMode::DEMONSTRATE >> ArmMode::OFFLINE:
      case ArmMode::REPLAYING >> ArmMode::OFFLINE:
        robot_cmd_data.cmd_state = MotorControlState::JOINT_POS;
        robot_cmd_data.plan_target_joint_q = current_joint_q;
        robot_cmd_data.target_joint_q = current_joint_q;
        robot_cmd_data.target_joint_v = joint_vel_limit_;
        interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                      (uint32_t)COLOR_PURPLE_CONSTANT);
        break;

      // --> RECORDING
      case ArmMode::DEMONSTRATE >> ArmMode::RECORDING:
        recorded_data_.init();
        interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                      (uint32_t)COLOR_BLUE_BREATHING);
        robot_cmd_data.cmd_state = MotorControlState::JOINT_MIT;
        break;

      // Replay cases
      case ArmMode::OFFLINE >> ArmMode::REPLAY_REACHING:
        robot_cmd_data.plan_target_joint_q = recorded_data_.q_records[0];
        robot_cmd_data.cmd_state = MotorControlState::JOINT_POS;
        _plan_target_joint_q(recorded_data_.q_records[0], true, 1.0, false);
        interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                      (uint32_t)COLOR_PURPLE_WAVE);
        break;
      case ArmMode::REPLAY_REACHING >> ArmMode::REPLAY_WAITING:
        replay_inplace_time_ = get_microsecond_now();
        replay_index_ = 0;
        use_planning_.store(false, std::memory_order_relaxed);
        interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                      (uint32_t)COLOR_PURPLE_FLASHING);
        break;
      case ArmMode::REPLAY_WAITING >> ArmMode::REPLAYING:
        interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                      (uint32_t)COLOR_PURPLE_BREATHING);
        break;

      case ArmMode::ONLINE >> ArmMode::ERROR:
      case ArmMode::DEMONSTRATE >> ArmMode::ERROR:
      case ArmMode::OFFLINE >> ArmMode::ERROR:
      case ArmMode::RECORDING >> ArmMode::ERROR:
      case ArmMode::REPLAYING >> ArmMode::ERROR:
        robot_cmd_data.cmd_state = MotorControlState::JOINT_POS;
        robot_cmd_data.plan_target_joint_q = current_joint_q;
        robot_cmd_data.target_joint_q = current_joint_q;
        robot_cmd_data.target_joint_v = joint_vel_limit_;
        use_planning_.store(false, std::memory_order_relaxed);
        robot_cmd_data.recover_request = false;
        interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                      (uint32_t)COLOR_RED_FLASHING);
        break;

      case ArmMode::ERROR >> ArmMode::ONLINE:
        logger_->info("changing mode from ERROR to ONLINE, recovered");
        robot_cmd_data.cmd_state = MotorControlState::JOINT_POS;
        robot_cmd_data.plan_target_joint_q = current_joint_q;
        robot_cmd_data.target_joint_q = current_joint_q;
        robot_cmd_data.target_joint_v = joint_vel_limit_;
        robot_cmd_data.recover_request = false;
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        interface_board_base_->SetCmd(arm::InterfaceBoardBase::CMD_LED_MODE, arm::InterfaceBoardBase::FRAME_1,
                                      (uint32_t)COLOR_GREEN_CONSTANT);
        break;

      default:
        logger_->error("changing mode is not allowed: {} >> {}", ArmModeStr[current_mode], ArmModeStr[cmd_mode]);
        return false;
    }
  }
  arm_mode_.store(cmd_mode, std::memory_order_relaxed);
  _write_cmd_data(robot_cmd_data);
  return true;
}

template <std::size_t DOF>
void arm::Robot<DOF>::set_frame(const Joints<DOF>& read_meters) {
  is_init_.store(false);
  for (int i = 0; i < read_meters.size(); i++) {
    try {
      dynamic_cast<arm::OdMotorDriver&>(*motor_driver_[i]).MotorSetPosBias(read_meters[i]);
    } catch (std::bad_cast& e) {
      logger_->warn("Motor {} is not an OdMotorDriver", i);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(MAGIC_DELAY));
  }
  is_init_.store(true);
}
