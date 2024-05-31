// DmMotorDriver.cpp
#include "airbot/modules/motors/dm_motor_driver.hpp"

namespace arm {

// extern std::shared_ptr<spdlog::logger> motor_logger;

DmMotorDriver::DmMotorDriver(uint16_t motor_id, std::string can_interface)
    : MotorDriver(), can_(SocketCAN::get(can_interface)) {
  board_id_ = motor_id;
  board_type_ = "DM motor";
  CanCbkCondition can_condition = std::bind(
      [motor_id](const can_frame& frame) {
        return (bool)((frame.can_id == motor_id) ||
                      ((frame.can_id == 0 || frame.can_id == 0x700 || frame.can_id == 0x701 || frame.can_id == 0x702 ||
                        frame.can_id == 0x703) &&
                       frame.data[0] == motor_id));
      },
      std::placeholders::_1);
  CanCbkFunc can_callback = std::bind(&DmMotorDriver::CanRxMsgCallback, this, std::placeholders::_1);
  can_->add_can_callback(
      std::make_tuple(std::string("motor#") + std::to_string(board_id_), can_condition, can_callback));
}

DmMotorDriver::~DmMotorDriver() { can_->remove_can_callback(std::string("motor#") + std::to_string(board_id_)); }

void DmMotorDriver::MotorLock() {
  can_frame tx_frame;
  tx_frame.can_id = board_id_;  // change according to the mode
  tx_frame.can_dlc = 0x08;

  tx_frame.data[0] = 0xFF;
  tx_frame.data[1] = 0xFF;
  tx_frame.data[2] = 0xFF;
  tx_frame.data[3] = 0xFF;
  tx_frame.data[4] = 0xFF;
  tx_frame.data[5] = 0xFF;
  tx_frame.data[6] = 0xFF;
  tx_frame.data[7] = 0xFC;

  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void DmMotorDriver::MotorUnlock() {
  can_frame tx_frame;
  tx_frame.can_id = board_id_;  // change according to the mode
  tx_frame.can_dlc = 0x08;

  tx_frame.data[0] = 0xFF;
  tx_frame.data[1] = 0xFF;
  tx_frame.data[2] = 0xFF;
  tx_frame.data[3] = 0xFF;
  tx_frame.data[4] = 0xFF;
  tx_frame.data[5] = 0xFF;
  tx_frame.data[6] = 0xFF;
  tx_frame.data[7] = 0xFD;
  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

bool DmMotorDriver::MotorInit() {
  // send disable command to enter read mode
  DmMotorDriver::MotorUnlock();
  Timer::ThreadSleepFor(normal_sleep_time);
  // send get firmware version command
  DmMotorDriver::MotorGetParam(10);
  Timer::ThreadSleepFor(normal_sleep_time);
  // send enable command to enter contorl mode
  DmMotorDriver::MotorLock();
  Timer::ThreadSleepFor(normal_sleep_time);
  DmMotorDriver::DmLed('G', 100);
  Timer::ThreadSleepFor(normal_sleep_time);
  // motor_logger->info("motor_id: {}\tversion: {}\ttarget version: {}\t",
  //                    board_id_, get_motor_msg()->firmware_version,
  //                    kFirmWareVersion);
  if (firmware_version_ == "") {
    logger_->warn("motor_id: {} communication error , init failed!", board_id_);
    return false;
  }
  logger_->info("motor_id: {0}\tversion: {1}\ttarget version: {2}", board_id_, firmware_version_, "3163 or 5013");
  if (firmware_version_ != "3163" && firmware_version_ != "5013") {
    logger_->warn("firmware version error, init failed!");
    return false;
  } else {
    logger_->info("firmware version match, init success!");
    return true;
  }
}

void DmMotorDriver::MotorDeInit() {
  DmMotorDriver::MotorUnlock();
  Timer::ThreadSleepFor(normal_sleep_time);
  DmMotorDriver::DmLed('G', 0);
}

bool DmMotorDriver::MotorWriteFlash() { return true; }

bool DmMotorDriver::MotorSetZero() {
  // send set zero command
  DmMotorDriver::DmMotorSetZero();
  Timer::ThreadSleepFor(500);  // wait for motor to set zero
  // motor_logger->info("motor_id: %d\tposition: %f\t", board_id_,
  //                    get_motor_pos());
  logger_->info("motor_id: {0}\tposition: {1}\t", board_id_, get_motor_pos());
  DmMotorDriver::MotorUnlock();
  if (get_motor_pos() > judgment_accuracy_threshold || get_motor_pos() < -judgment_accuracy_threshold) {
    logger_->warn("set zero error");
    return false;
  } else {
    logger_->info("set zero success");
    return false;
  }
  // disable motor
}

void DmMotorDriver::CanRxMsgCallback(const can_frame& rx_frame) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count--;
  }
  union32_t rv_type_convert;
  if (rx_frame.can_id == board_id_) {
    uint16_t board_id_t = 0;
    uint16_t pos_int = 0;
    uint16_t spd_int = 0;
    uint16_t t_int = 0;
    pos_int = rx_frame.data[1] << 8 | rx_frame.data[2];
    spd_int = rx_frame.data[3] << 4 | (rx_frame.data[4] & 0xF0) >> 4;
    t_int = (rx_frame.data[4] & 0x0F) << 8 | rx_frame.data[5];
    board_id_t = (rx_frame.data[0] & 0x0F);
    if ((rx_frame.data[0] & 0xF0) >> 4 > 7) {  // error code range from 8 to 15
      error_id_ = (rx_frame.data[0] & 0xF0) >> 4;
    }
    motor_pos_ = range_map(pos_int, uint16_t(0), bitmax<uint16_t>(16), kPMin, kPMax);
    motor_spd_ = range_map(spd_int, uint16_t(0), bitmax<uint16_t>(12), kSpdMin, kSpdMax);
    motor_current_ = range_map(t_int, uint16_t(0), bitmax<uint16_t>(12), kTorqueMin, kTorqueMax);
    mos_temperature_ = rx_frame.data[6];
    motor_temperature_ = rx_frame.data[7];

    heartbeat_detect_counter_ = 0;
    ret_id_ = board_id_t;
  } else if (rx_frame.can_id == 0x700) {
    uint16_t motor_id = 0;
    uint8_t reg_id = 0;
    motor_id = (rx_frame.data[1] << 8 | rx_frame.data[0]);
    reg_id = rx_frame.data[3];
    rv_type_convert.buf[0] = rx_frame.data[4];
    rv_type_convert.buf[1] = rx_frame.data[5];
    rv_type_convert.buf[2] = rx_frame.data[6];
    rv_type_convert.buf[3] = rx_frame.data[7];
    heartbeat_detect_counter_ = 0;
    switch (reg_id) {
      case 0:
        under_voltage_ = rv_type_convert.f;
        break;
      case 1:
        motor_kd_spd = rv_type_convert.f;
        break;
      case 2:
        current_limit_ = rv_type_convert.f;
        break;
      case 3:
        motor_acceleration_ = rv_type_convert.f;
        break;
      case 4:
        dec_ = rv_type_convert.f;
        break;
      case 5:
        max_speed_ = rv_type_convert.f;
        break;
      case 6:
        master_id_ = rv_type_convert.i;
        break;
      case 7:
        ret_id_ = rv_type_convert.i;
        break;
      case 8:
        timeout_ = rv_type_convert.u;
        break;
      case 9:
        ctrl_mode_ = rv_type_convert.i;
        break;
      case 10:
        firmware_version_ = std::to_string((int)(rv_type_convert.buf[0] * 1000 + rv_type_convert.buf[1] * 100 +
                                                 rv_type_convert.buf[2] * 10 + rv_type_convert.buf[3]));
        break;
      case 11:
        gear_ratio_ = rv_type_convert.f;
        break;
      case 12:
        pos_max_ = rv_type_convert.f;
        break;
      case 13:
        vel_max_ = rv_type_convert.f;
        break;
      case 14:
        torque_max_ = rv_type_convert.f;
        break;
      case 15:
        current_bandwidth_ = rv_type_convert.f;
        break;
      case 16:
        motor_kp_spd = rv_type_convert.f;
        break;
      case 17:
        motor_ki_spd = rv_type_convert.f;
        break;
      case 18:
        motor_kp_pos = rv_type_convert.f;
        break;
      case 19:
        motor_ki_pos = rv_type_convert.f;
        break;
      case 20:
        over_voltage_ = rv_type_convert.f;
        break;
      case 21:
        gear_torque_coefficient_ = rv_type_convert.f;
        break;
      case 22:
        motor_kd_pos = rv_type_convert.f;
        break;
      default:
        break;
    }
  } else if (rx_frame.can_id == 0x701) {
    uint16_t motor_id = 0;
    int rid = rx_frame.data[3];
    param_cmd_flag_[rid] = true;
    if (rid == 9) {
      motor_control_mode_ = rx_frame.data[4];
    }

    write_para_res_ = rx_frame.data[2];
  } else if (rx_frame.can_id == 0x702) {
    uint16_t motor_id = 0;
    write_para_res_ = rx_frame.data[3];
    heartbeat_detect_counter_ = 0;
  } else if (rx_frame.can_id == 0x703) {
    write_para_res_ = rx_frame.data[3];
  }
}

void DmMotorDriver::MotorGetParam(uint8_t param_cmd) {
  can_frame tx_frame;
  tx_frame.can_id = 0x7FF;
  tx_frame.can_dlc = 0x08;

  tx_frame.data[0] = board_id_ & 0xFF;
  tx_frame.data[1] = board_id_ >> 8;
  tx_frame.data[2] = 0x33;
  tx_frame.data[3] = param_cmd;

  tx_frame.data[4] = 0xFF;
  tx_frame.data[5] = 0xFF;
  tx_frame.data[6] = 0xFF;
  tx_frame.data[7] = 0xFF;
  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void DmMotorDriver::MotorPosModeCmd(float pos, float spd, bool ignore_limit) {
  if (motor_control_mode_ != POS) {
    set_motor_control_mode(POS);
    return;
  }
  if (pos < joint_lower_bounder_[board_id_ - 1] || pos > joint_upper_bounder_[board_id_ - 1]) {
    logger_->warn("motor {0} pos {1} is out of jointspace: {2} {3}", board_id_, pos,
                  joint_lower_bounder_[board_id_ - 1], joint_upper_bounder_[board_id_ - 1]);
    return;
  }
  can_frame tx_frame;
  tx_frame.can_id = board_id_;
  tx_frame.can_dlc = 0x08;
  uint8_t *pbuf, *vbuf;

  spd = limit(spd, kSpdMin, kSpdMax);
  pos = limit(pos, kPMin, kPMax);

  pbuf = (uint8_t*)&pos;
  vbuf = (uint8_t*)&spd;

  tx_frame.data[0] = *pbuf;
  tx_frame.data[1] = *(pbuf + 1);
  tx_frame.data[2] = *(pbuf + 2);
  tx_frame.data[3] = *(pbuf + 3);
  tx_frame.data[4] = *vbuf;
  tx_frame.data[5] = *(vbuf + 1);
  tx_frame.data[6] = *(vbuf + 2);
  tx_frame.data[7] = *(vbuf + 3);

  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void DmMotorDriver::MotorSpdModeCmd(float spd) {
  if (motor_control_mode_ != SPD) {
    set_motor_control_mode(SPD);
    return;
  }
  can_frame tx_frame;
  tx_frame.can_id = board_id_;
  tx_frame.can_dlc = 0x04;

  union32_t rv_type_convert;
  rv_type_convert.f = spd;
  tx_frame.data[0] = rv_type_convert.buf[0];
  tx_frame.data[1] = rv_type_convert.buf[1];
  tx_frame.data[2] = rv_type_convert.buf[2];
  tx_frame.data[3] = rv_type_convert.buf[3];

  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

// Transmit MIT-mDme control(hybrid) package. Called in canTask.
void DmMotorDriver::MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
  if (motor_control_mode_ != MIT) {
    set_motor_control_mode(MIT);
    return;
  }
  uint16_t p, v, kp, kd, t;
  can_frame tx_frame;

  f_p = limit(f_p, kPMin, kPMax);
  f_v = limit(f_v, kSpdMin, kSpdMax);
  f_kp = limit(f_kp, kKpMin, kKpMax);
  f_kd = limit(f_kd, kKdMin, kKdMax);
  f_t = limit(f_t, kTorqueMin, kTorqueMax);

  p = range_map(f_p, kPMin, kPMax, uint16_t(0), bitmax<uint16_t>(16));
  v = range_map(f_v, kSpdMin, kSpdMax, uint16_t(0), bitmax<uint16_t>(12));
  kp = range_map(f_kp, kKpMin, kKpMax, uint16_t(0), bitmax<uint16_t>(12));
  kd = range_map(f_kd, kKdMin, kKdMax, uint16_t(0), bitmax<uint16_t>(12));
  t = range_map(f_t, kTorqueMin, kTorqueMax, uint16_t(0), bitmax<uint16_t>(12));

  tx_frame.can_id = board_id_;
  tx_frame.can_dlc = 0x08;

  tx_frame.data[0] = p >> 8;
  tx_frame.data[1] = p & 0xFF;
  tx_frame.data[2] = v >> 4;
  tx_frame.data[3] = (v & 0x0F) << 4 | kp >> 8;
  tx_frame.data[4] = kp & 0xFF;
  tx_frame.data[5] = kd >> 4;
  tx_frame.data[6] = (kd & 0x0F) << 4 | t >> 8;
  tx_frame.data[7] = t & 0xFF;

  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

// todo
void DmMotorDriver::MotorSetPosParam(float kp, float kd) {}

void DmMotorDriver::MotorSetSpdParam(float kp, float ki) {}

void DmMotorDriver::MotorSetFilterParam(float position_kd_filter, float kd_spd) {}

void DmMotorDriver::set_motor_id(uint8_t motor_id) {
  DmWriteRegister(7, motor_id);
  Timer::ThreadSleepFor(normal_sleep_time);
  board_id_ = motor_id;
  DmSaveRegister(7);
  Timer::ThreadSleepFor(normal_sleep_time);
  DmWriteRegister(6, motor_id);
  Timer::ThreadSleepFor(normal_sleep_time);
  DmSaveRegister(6);
  Timer::ThreadSleepFor(normal_sleep_time);
}

void DmMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) { DmWriteRegister(9, motor_control_mode); }

void DmMotorDriver::DmMotorSetZero() {
  can_frame tx_frame;
  tx_frame.can_id = board_id_;  // change according to the mode
  tx_frame.can_dlc = 0x08;

  tx_frame.data[0] = 0xFF;
  tx_frame.data[1] = 0xFF;
  tx_frame.data[2] = 0xFF;
  tx_frame.data[3] = 0xFF;
  tx_frame.data[4] = 0xFF;
  tx_frame.data[5] = 0xFF;
  tx_frame.data[6] = 0xFF;
  tx_frame.data[7] = 0xFE;
  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void DmMotorDriver::DmMotorClearError() {
  can_frame tx_frame;
  tx_frame.can_id = board_id_;  // change according to the mode
  tx_frame.can_dlc = 0x08;

  tx_frame.data[0] = 0xFF;
  tx_frame.data[1] = 0xFF;
  tx_frame.data[2] = 0xFF;
  tx_frame.data[3] = 0xFF;
  tx_frame.data[4] = 0xFF;
  tx_frame.data[5] = 0xFF;
  tx_frame.data[6] = 0xFF;
  tx_frame.data[7] = 0xFB;
  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void DmMotorDriver::DmWriteRegister(uint8_t rid, float value) {
  param_cmd_flag_[rid] = false;
  can_frame tx_frame;
  tx_frame.can_id = 0x7FF;
  tx_frame.can_dlc = 0x08;

  uint8_t* vbuf;
  vbuf = (uint8_t*)&value;

  tx_frame.data[0] = board_id_ & 0xFF;
  tx_frame.data[1] = board_id_ >> 8;
  tx_frame.data[2] = 0x55;
  tx_frame.data[3] = rid;

  tx_frame.data[4] = *vbuf;
  tx_frame.data[5] = *(vbuf + 1);
  tx_frame.data[6] = *(vbuf + 2);
  tx_frame.data[7] = *(vbuf + 3);
  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void DmMotorDriver::DmWriteRegister(uint8_t rid, int32_t value) {
  param_cmd_flag_[rid] = false;
  can_frame tx_frame;
  tx_frame.can_id = 0x7FF;
  tx_frame.can_dlc = 0x08;

  uint8_t* vbuf;
  vbuf = (uint8_t*)&value;

  tx_frame.data[0] = board_id_ & 0xFF;
  tx_frame.data[1] = board_id_ >> 8;
  tx_frame.data[2] = 0x55;
  tx_frame.data[3] = rid;

  tx_frame.data[4] = *vbuf;
  tx_frame.data[5] = *(vbuf + 1);
  tx_frame.data[6] = *(vbuf + 2);
  tx_frame.data[7] = *(vbuf + 3);
  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void DmMotorDriver::DmSaveRegister(uint8_t rid) {
  can_frame tx_frame;
  tx_frame.can_id = 0x7FF;
  tx_frame.can_dlc = 0x08;

  tx_frame.data[0] = board_id_ & 0xFF;
  tx_frame.data[1] = board_id_ >> 8;
  tx_frame.data[2] = 0xAA;
  tx_frame.data[3] = rid;

  tx_frame.data[4] = 0xFF;
  tx_frame.data[5] = 0xFF;
  tx_frame.data[6] = 0xFF;
  tx_frame.data[7] = 0xFF;
  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void DmMotorDriver::DmLed(uint8_t lid, uint8_t freq) {
  can_frame tx_frame;
  tx_frame.can_id = 0x7FF;
  tx_frame.can_dlc = 0x08;

  tx_frame.data[0] = board_id_ & 0xFF;
  tx_frame.data[1] = board_id_ >> 8;
  tx_frame.data[2] = 0xDD;
  tx_frame.data[3] = 0xFF;

  tx_frame.data[4] = lid;
  tx_frame.data[5] = 0x00;
  tx_frame.data[6] = 0x00;
  tx_frame.data[7] = freq;
  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

bool DmMotorDriver::Init() {
  if (!MotorInit()) {
    return false;
  }
  return true;
}

}  // namespace arm
