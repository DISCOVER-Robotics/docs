// OdMotorDriver.cpp
#include "airbot/modules/motors/od_motor_driver.hpp"

#include <functional>
#include <iostream>

#define OD_MOTOR_MIN_FIRMWARE_VERSION "V2.7.0"

namespace arm {

OdMotorDriver::OdMotorDriver(uint16_t motor_id, std::string can_interface)
    : MotorDriver(), can_(SocketCAN::get(can_interface)) {
  board_id_ = motor_id;
  board_id_ = motor_id;
  board_type_ = "OD motor";
  int ret_cmd_id = board_id_ | RET_CMD << 7;
  CanCbkCondition can_condition = std::bind(
      [motor_id, ret_cmd_id](const can_frame& frame) {
        return (bool)((frame.can_id == motor_id) || frame.can_id == 0X7FF || (frame.can_id == ret_cmd_id));
      },
      std::placeholders::_1);
  CanCbkFunc can_callback = std::bind(&OdMotorDriver::CanRxMsgCallback, this, std::placeholders::_1);
  can_->add_can_callback(
      std::make_tuple(std::string("motor#") + std::to_string(board_id_), can_condition, can_callback));
}

OdMotorDriver::~OdMotorDriver() { can_->remove_can_callback(std::string("motor#") + std::to_string(board_id_)); }

bool OdMotorDriver::Init() {
  bool ret = BoardDriver::Init();
  GetCmd(CMD_MOTOR_BODY_SN_CODE);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  GetCmd(CMD_MOTOR_SN_CODE);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  logger_->info("\t\tmotor_body_sn_code: {0}\t motor_sn_code: {1}", body_sn_code_, whole_sn_code_);
  if (firmware_version_ < OD_MOTOR_MIN_FIRMWARE_VERSION) {
    logger_->error("board {0} firmware version lower than min version {1}!!!", DEVICE_ID,
                   OD_MOTOR_MIN_FIRMWARE_VERSION);
    return false;
  }
  return ret;
}

void OdMotorDriver::MotorLock() {
  OdMotorDriver::ConfigOdMotor(0x08);
  Timer::ThreadSleepFor(10);
}

void OdMotorDriver::MotorUnlock() {
  OdMotorDriver::ConfigOdMotor(0x07);
  Timer::ThreadSleepFor(10);
}

bool OdMotorDriver::MotorInit() { return Init() && CheckId(); }

void OdMotorDriver::MotorDeInit() { OdMotorDriver::MotorLock(); }

bool OdMotorDriver::MotorSetZero() {
  // send set zero command
  OdMotorDriver::ConfigOdMotor(0x03);
  Timer::ThreadSleepFor(500);  // wait for motor to set zero
  // send read position command
  OdMotorDriver::MotorGetParam(0x01);
  Timer::ThreadSleepFor(1);

  logger_->info("motor_id: {0}\tposition: {1}\t", board_id_, get_motor_pos());
  if (get_motor_pos() > judgment_accuracy_threshold || get_motor_pos() < -judgment_accuracy_threshold) {
    logger_->warn("set zero error");
    return false;
  } else {
    logger_->info("set zero success");
    return true;
  }
}

bool OdMotorDriver::MotorSetPosBias(float pos_bias) {
  SetCmd(CMD_POS_BIAS, FRAME_1, pos_bias);
  Timer::ThreadSleepFor(500);
  // TODO: check if set pos bias success
  return true;
}

bool OdMotorDriver::MotorWriteFlash() {
  OdMotorDriver::ConfigOdMotor(0x06);
  return true;
}

void OdMotorDriver::CanRxMsgCallback(const can_frame& rx_frame) {
  // for (int i = 0; i < rx_frame.can_dlc; i++) {
  //   motor_logger->info("%d ", rx_frame.data[i]);
  // }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count--;
  }

  uint8_t board_id_t = 0;
  uint8_t ack_status = 0;
  uint16_t pos_int = 0;
  uint16_t spd_int = 0;
  uint16_t cur_int = 0;
  int outpos_int = 0;
  if (rx_frame.can_id == RET_CMD_ID) {
    BoardDriver::CanRxMsgCallback(rx_frame);
    return;
  } else if (rx_frame.can_id == 0x7FF) {
    if (rx_frame.data[2] != 0x01) return;
    if ((rx_frame.data[0] == 0xFF) && (rx_frame.data[1] == 0xFF)) {
      board_id_t = (rx_frame.data[3] << 8 | rx_frame.data[4]);
      communication_mode_ = 0x01;
      heartbeat_detect_counter_ = 0;
      if (board_id_t != board_id_ && ret_id_ + 1) return;
    } else {
      board_id_t = (rx_frame.data[0] << 8 | rx_frame.data[1]);
      ret_id_ = board_id_t;
      communication_mode_ = rx_frame.data[3];
      heartbeat_detect_counter_ = 0;
    }
  } else {
    ack_status = rx_frame.data[0] >> 5;
    board_id_t = rx_frame.can_id;
    error_id_ = rx_frame.data[0] & 0x1F;
    heartbeat_detect_counter_ = 0;
    union32_t rv_type_convert;
    if (ack_status == 1)  // response frame 1
    {
      pos_int = rx_frame.data[1] << 8 | rx_frame.data[2];
      spd_int = rx_frame.data[3] << 4 | (rx_frame.data[4] & 0xF0) >> 4;
      cur_int = (rx_frame.data[4] & 0x0F) << 8 | rx_frame.data[5];
      motor_pos_ = range_map(pos_int, (uint16_t)0, bitmax<uint16_t>(16), kPosMin, kPosMax);
      motor_spd_ = range_map(spd_int, (uint16_t)0, bitmax<uint16_t>(12), kSpdMin, kSpdMax);
      motor_current_ = range_map(cur_int, (uint16_t)0, bitmax<uint16_t>(12), kIMin, kIMax);
      motor_temperature_ = (rx_frame.data[6] - 50) / 2;
    } else if (ack_status == 2)  // response frame 2
    {
      rv_type_convert.buf[0] = rx_frame.data[4];
      rv_type_convert.buf[1] = rx_frame.data[3];
      rv_type_convert.buf[2] = rx_frame.data[2];
      rv_type_convert.buf[3] = rx_frame.data[1];

      motor_pos_ = rv_type_convert.f * M_PI / 180.0f;
      motor_temperature_ = (rx_frame.data[7] - 50) / 2;
      motor_current_ = (int16_t)(rx_frame.data[5] << 8 | rx_frame.data[6]) / 100.0f;
    } else if (ack_status == 3)  // response frame 3
    {
      rv_type_convert.buf[0] = rx_frame.data[4];
      rv_type_convert.buf[1] = rx_frame.data[3];
      rv_type_convert.buf[2] = rx_frame.data[2];
      rv_type_convert.buf[3] = rx_frame.data[1];

      motor_spd_ = rv_type_convert.f * M_PI / 30.0f;
      motor_temperature_ = (rx_frame.data[7] - 50) / 2;
      motor_current_ = (int16_t)(rx_frame.data[5] << 8 | rx_frame.data[6]) / 100.0f;
    } else if (ack_status == 4)  // response frame 4
    {
      if (rx_frame.can_dlc != 3) return;
      write_para_res_ = rx_frame.data[2];
    } else if (ack_status == 5)  // response frame 5
    {
      uint16_t read_para = rx_frame.data[1];
      // if (read_para == 0 & rx_frame.can_dlc == 4) {
      //   can_motor_.firmware_version = rx_frame.data[2] << 8 | rx_frame.data[3];
      // }
      if (read_para == 1 & rx_frame.can_dlc == 6)  // get position
      {
        rv_type_convert.buf[0] = rx_frame.data[5];
        rv_type_convert.buf[1] = rx_frame.data[4];
        rv_type_convert.buf[2] = rx_frame.data[3];
        rv_type_convert.buf[3] = rx_frame.data[2];
        motor_pos_ = rv_type_convert.f * M_PI / 180.0f;
      } else if (read_para == 2 & rx_frame.can_dlc == 6)  // get speed
      {
        rv_type_convert.buf[0] = rx_frame.data[5];
        rv_type_convert.buf[1] = rx_frame.data[4];
        rv_type_convert.buf[2] = rx_frame.data[3];
        rv_type_convert.buf[3] = rx_frame.data[2];
        motor_spd_ = rv_type_convert.f * M_PI / 30.0f;
      } else if (read_para == 3 & rx_frame.can_dlc == 6)  // get current
      {
        rv_type_convert.buf[0] = rx_frame.data[5];
        rv_type_convert.buf[1] = rx_frame.data[4];
        rv_type_convert.buf[2] = rx_frame.data[3];
        rv_type_convert.buf[3] = rx_frame.data[2];
        motor_current_ = rv_type_convert.f;
      } else if (read_para == 4 & rx_frame.can_dlc == 6)  // get power
      {
        rv_type_convert.buf[0] = rx_frame.data[5];
        rv_type_convert.buf[1] = rx_frame.data[4];
        rv_type_convert.buf[2] = rx_frame.data[3];
        rv_type_convert.buf[3] = rx_frame.data[2];
        power_ = rv_type_convert.f;
      } else if (read_para == 5 & rx_frame.can_dlc == 4)  // get acceleration
      {
        motor_acceleration_ = float(rx_frame.data[2] << 8 | rx_frame.data[3]);
      } else if (read_para == 6 & rx_frame.can_dlc == 4)  // get ki_spd
      {
        motor_ki_spd = float(rx_frame.data[2] << 8 | rx_frame.data[3]) / 10000;
      } else if (read_para == 7 & rx_frame.can_dlc == 4)  // get kp_spd
      {
        motor_kp_spd = float(rx_frame.data[2] << 8 | rx_frame.data[3]) / 10000;
      } else if (read_para == 8 & rx_frame.can_dlc == 4)  // get kp_pos
      {
        motor_kp_pos = float(rx_frame.data[2] << 8 | rx_frame.data[3]) / 10000;
      } else if (read_para == 9 & rx_frame.can_dlc == 4)  // get kd_pos
      {
        motor_kd_pos = float(rx_frame.data[2] << 8 | rx_frame.data[3]) / 100000;
      } else if (read_para == 10 & rx_frame.can_dlc == 4)  // get position_kd_filter
      {
        motor_kd_pos_filter = float(rx_frame.data[2] << 8 | rx_frame.data[3]) / 10000;
      } else if (read_para == 11 & rx_frame.can_dlc == 4)  // get kd_spd
      {
        motor_kd_spd = float(rx_frame.data[2] << 8 | rx_frame.data[3]) / 1000;
      } else if (read_para == 0x0C & rx_frame.can_dlc == 6)  // get ki_pos
      {
        rv_type_convert.buf[0] = rx_frame.data[5];
        rv_type_convert.buf[1] = rx_frame.data[4];
        rv_type_convert.buf[2] = rx_frame.data[3];
        rv_type_convert.buf[3] = rx_frame.data[2];
        motor_ki_pos = rv_type_convert.f;
      } else if (read_para == 0x0D & rx_frame.can_dlc == 6)  // get gear_ratio
      {
        rv_type_convert.buf[0] = rx_frame.data[5];
        rv_type_convert.buf[1] = rx_frame.data[4];
        rv_type_convert.buf[2] = rx_frame.data[3];
        rv_type_convert.buf[3] = rx_frame.data[2];
        gear_ratio_ = rv_type_convert.f;
      } else if (read_para == 0x0E & rx_frame.can_dlc == 6)  // get p_pid_ang_div
      {
        rv_type_convert.buf[0] = rx_frame.data[5];
        rv_type_convert.buf[1] = rx_frame.data[4];
        rv_type_convert.buf[2] = rx_frame.data[3];
        rv_type_convert.buf[3] = rx_frame.data[2];
        p_pid_ang_div = rv_type_convert.f;
      } else if (read_para == 0x0F & rx_frame.can_dlc == 6)  // get m_ntc_motor_beta
      {
        rv_type_convert.buf[0] = rx_frame.data[5];
        rv_type_convert.buf[1] = rx_frame.data[4];
        rv_type_convert.buf[2] = rx_frame.data[3];
        rv_type_convert.buf[3] = rx_frame.data[2];
        m_ntc_motor_beta = rv_type_convert.f;
      } else if (read_para == 0x10 & rx_frame.can_dlc == 3)  // get si_motor_poles
      {
        si_motor_poles = rx_frame.data[2];
      } else if (read_para == 0x11 & rx_frame.can_dlc == 6)  // get foc_motor_r
      {
        rv_type_convert.buf[0] = rx_frame.data[5];
        rv_type_convert.buf[1] = rx_frame.data[4];
        rv_type_convert.buf[2] = rx_frame.data[3];
        rv_type_convert.buf[3] = rx_frame.data[2];
        foc_motor_r = rv_type_convert.f;
      } else if (read_para == 0x12 & rx_frame.can_dlc == 6)  // get foc_motor_l
      {
        rv_type_convert.buf[0] = rx_frame.data[5];
        rv_type_convert.buf[1] = rx_frame.data[4];
        rv_type_convert.buf[2] = rx_frame.data[3];
        rv_type_convert.buf[3] = rx_frame.data[2];
        foc_motor_l = rv_type_convert.f;
      } else if (read_para == 0x13 & rx_frame.can_dlc == 6)  // get timeout
      {
        rv_type_convert.buf[0] = rx_frame.data[5];
        rv_type_convert.buf[1] = rx_frame.data[4];
        rv_type_convert.buf[2] = rx_frame.data[3];
        rv_type_convert.buf[3] = rx_frame.data[2];
        timeout_ = rv_type_convert.u;
      }
    }
  }
  ret_id_ = board_id_t;
}

void OdMotorDriver::MotorGetParam(uint8_t param_cmd) {
  can_frame tx_frame;
  tx_frame.can_id = board_id_;
  tx_frame.can_dlc = 0x02;
  tx_frame.data[0] = 0xE0;
  tx_frame.data[1] = param_cmd;

  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void OdMotorDriver::MotorPosModeCmd(float pos, float spd, bool ignore_limit) {
  if (!ignore_limit) {
    if (pos < joint_lower_bounder_[board_id_ - 1] || pos > joint_upper_bounder_[board_id_ - 1]) {
      // motor_logger->warn(
      //     "motor {} pos {:.3f} is out of jointspace: {:.3f} {:.3f}",
      //     board_id_, pos, joint_lower_bounder_[board_id_ - 1],
      //     joint_upper_bounder_[board_id_ - 1]);
      logger_->warn("motor {0} pos {1} is out of jointspace: {2} {3}", board_id_, pos,
                    joint_lower_bounder_[board_id_ - 1], joint_upper_bounder_[board_id_ - 1]);
      return;
    }
  }
  float cur = max_current_;
  uint8_t ack_status = OdMotorDriver::ack_status_;
  can_frame tx_frame;
  tx_frame.can_id = board_id_;
  tx_frame.can_dlc = 0x08;

  if (ack_status > 3) return;

  // spd = limitMax(fabs(spd), 343.1f);
  if (spd > 343.1f) {
    spd = 343.1f;
  } else if (spd < -343.1f) {
    spd = -343.1f;
  } else {
  }
  cur = limit(cur, 409.5f);

  union32_t rv_type_convert;
  rv_type_convert.f = pos * 180.0f / M_PI;
  uint16_t tmp_spd = 0, tmp_cur = 0;
  tmp_spd = (uint16_t)floorf(spd * 60.0f / M_PI / 2.0f * 10.0f);
  tmp_cur = (uint16_t)floorf(cur * 10.0f);

  tx_frame.data[0] = 0x20 | (rv_type_convert.buf[3] >> 3);
  tx_frame.data[1] = (rv_type_convert.buf[3] << 5) | (rv_type_convert.buf[2] >> 3);
  tx_frame.data[2] = (rv_type_convert.buf[2] << 5) | (rv_type_convert.buf[1] >> 3);
  tx_frame.data[3] = (rv_type_convert.buf[1] << 5) | (rv_type_convert.buf[0] >> 3);
  tx_frame.data[4] = (rv_type_convert.buf[0] << 5) | (tmp_spd >> 10);
  tx_frame.data[5] = (tmp_spd & 0x3FC) >> 2;
  tx_frame.data[6] = (tmp_spd & 0x03) << 6 | (tmp_cur >> 6);
  tx_frame.data[7] = (tmp_cur & 0x3F) << 2 | ack_status;

  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void OdMotorDriver::MotorSpdModeCmd(float spd) {
  float cur = max_current_;
  uint8_t ack_status = OdMotorDriver::ack_status_;
  can_frame tx_frame;
  tx_frame.can_id = board_id_;
  tx_frame.can_dlc = 0x07;

  if (ack_status > 3) return;

  spd = limit(spd, 343.1f);
  cur = limit(cur, 409.5f);

  static uint16_t tmp_cur = 0;
  union32_t rv_type_convert;
  rv_type_convert.f = spd * 60.0f / M_PI / 2.0f;
  tmp_cur = (uint16_t)floorf(cur * 10.0f);

  tx_frame.data[0] = 0x40 | ack_status;
  tx_frame.data[1] = rv_type_convert.buf[3];
  tx_frame.data[2] = rv_type_convert.buf[2];
  tx_frame.data[3] = rv_type_convert.buf[1];
  tx_frame.data[4] = rv_type_convert.buf[0];
  tx_frame.data[5] = tmp_cur >> 8;
  tx_frame.data[6] = tmp_cur & 0xff;

  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}
// Transmit MIT-mode control(hybrid) package. Called in canTask.
void OdMotorDriver::MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
  uint16_t p, v, kp, kd, t;
  can_frame tx_frame;

  f_p = limit(f_p, kPosMin, kPosMax);
  f_v = limit(f_v, kSpdMin, kSpdMax);
  f_kp = limit(f_kp, kKpMin, kKpMax);
  f_kd = limit(f_kd, kKdMin, kKdMax);
  f_t = limit(f_t, kTorqueMin, kTorqueMax);

  p = range_map(f_p, kPosMin, kPosMax, uint16_t(0), bitmax<uint16_t>(16));
  v = range_map(f_v, kSpdMin, kSpdMax, uint16_t(0), bitmax<uint16_t>(12));
  kp = range_map(f_kp, kKpMin, kKpMax, uint16_t(0), bitmax<uint16_t>(12));
  kd = range_map(f_kd, kKdMin, kKdMax, uint16_t(0), bitmax<uint16_t>(9));
  t = range_map(f_t, kTorqueMin, kTorqueMax, uint16_t(0), bitmax<uint16_t>(12));

  tx_frame.can_id = board_id_;
  tx_frame.can_dlc = 0x08;

  tx_frame.data[0] = kp >> 7;
  tx_frame.data[1] = ((kp & 0x7F) << 1) | ((kd & 0x100) >> 8);
  tx_frame.data[2] = kd & 0xFF;
  tx_frame.data[3] = p >> 8;
  tx_frame.data[4] = p & 0xFF;
  tx_frame.data[5] = v >> 4;
  tx_frame.data[6] = (v & 0x0F) << 4 | (t >> 8);
  tx_frame.data[7] = t & 0xff;

  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void OdMotorDriver::MotorSetPosParam(float kp, float kd) {
  uint16_t fdbKP, fdbKD;
  fdbKP = range_map(kp, kKpMin, kKpMax, uint16_t(0), bitmax<uint16_t>(12));
  fdbKD = range_map(kd, kKdMin, kKdMax, uint16_t(0), bitmax<uint16_t>(9));
  OdMotorDriver::OdSetPosParam(fdbKP, fdbKD, ack_status_);
  Timer::ThreadSleepFor(setup_sleep_time);
}

void OdMotorDriver::MotorSetSpdParam(float kp, float ki) {
  uint16_t linkage, speedKI;
  linkage = range_map(kp, kKpMin, kKpMax, uint16_t(0), bitmax<uint16_t>(12));
  speedKI = range_map(ki, kKdMin, kKdMax, uint16_t(0), bitmax<uint16_t>(9));
  OdMotorDriver::OdSetSpdParam(linkage, speedKI, ack_status_);
  Timer::ThreadSleepFor(setup_sleep_time);
}

void OdMotorDriver::MotorSetFilterParam(float position_kd_filter, float kd_spd) {
  uint16_t position_kd_filter_int, kd_spd_int;
  position_kd_filter_int = range_map(position_kd_filter, kKpMin, kKpMax, uint16_t(0), bitmax<uint16_t>(12));
  kd_spd_int = range_map(kd_spd, kKdMin, kKdMax, uint16_t(0), bitmax<uint16_t>(9));
  OdMotorDriver::OdSetFilterParam(position_kd_filter_int, kd_spd_int, ack_status_);
  Timer::ThreadSleepFor(setup_sleep_time);
}

void OdMotorDriver::MotorResetID() {
  can_frame tx_frame;
  tx_frame.can_id = 0x7FF;
  tx_frame.can_dlc = 0x06;
  tx_frame.data[0] = 0x7f;
  tx_frame.data[1] = 0x7f;
  tx_frame.data[2] = 0x00;
  tx_frame.data[3] = 0x05;
  tx_frame.data[4] = 0x7f;
  tx_frame.data[5] = 0x7f;
  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void OdMotorDriver::set_motor_id(uint8_t motor_id) {
  can_frame tx_frame;
  tx_frame.can_id = 0x7FF;
  tx_frame.can_dlc = 0x06;
  tx_frame.data[0] = board_id_ >> 8;
  tx_frame.data[1] = board_id_ & 0xff;
  tx_frame.data[2] = 0x00;
  tx_frame.data[3] = 0x04;
  tx_frame.data[4] = motor_id >> 8;
  tx_frame.data[5] = motor_id & 0xff;
  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
  board_id_ = motor_id;
}

void OdMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) { motor_control_mode_ = motor_control_mode; }

void OdMotorDriver::ConfigOdMotor(uint8_t cmd) {
  can_frame config_frame;
  config_frame.can_dlc = 0x04;
  config_frame.can_id = 0x7FF;
  if (cmd == 0x00) {
    return;
  }

  /*
  cmd = 0x01: automatic telegram
  cmd = 0x02: answer mode
  cmd = 0x03: set zero
  */
  config_frame.data[0] = board_id_ >> 8;
  config_frame.data[1] = board_id_ & 0xff;
  config_frame.data[2] = 0x00;
  config_frame.data[3] = cmd;

  // transmit can bag
  can_->transmit(config_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

// Configure the kp,kd of position loop
void OdMotorDriver::OdSetPosParam(uint16_t fdbKP, uint16_t fdbKD, uint8_t ack_status) {
  can_frame tx_frame;
  tx_frame.can_id = board_id_;
  tx_frame.can_dlc = 6;

  if (ack_status > 2) return;

  fdbKP = limit(fdbKP, (uint16_t)10000);
  fdbKD = limit(fdbKD, (uint16_t)10000);

  tx_frame.data[0] = 0xC0 | ack_status;
  tx_frame.data[1] = 0x03;
  tx_frame.data[2] = fdbKP >> 8;
  tx_frame.data[3] = fdbKP & 0xff;
  tx_frame.data[4] = fdbKD >> 8;
  tx_frame.data[5] = fdbKD & 0xff;

  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

// Configure the kp,kd of speed loop
void OdMotorDriver::OdSetSpdParam(uint16_t linkage, uint16_t speedKI, uint8_t ack_status) {
  can_frame tx_frame;
  tx_frame.can_id = board_id_;
  tx_frame.can_dlc = 6;

  if (ack_status > 2) return;

  linkage = limit(linkage, (uint16_t)10000);
  speedKI = limit(speedKI, (uint16_t)10000);

  tx_frame.data[0] = 0xC0 | ack_status;
  tx_frame.data[1] = 0x02;
  tx_frame.data[2] = speedKI >> 8;
  tx_frame.data[3] = speedKI & 0xff;
  tx_frame.data[4] = linkage >> 8;
  tx_frame.data[5] = linkage & 0xff;

  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void OdMotorDriver::OdSetFilterParam(uint16_t position_kd_filter, uint16_t kd_spd, uint8_t ack_status) {
  can_frame tx_frame;
  tx_frame.can_id = board_id_;
  tx_frame.can_dlc = 6;

  if (ack_status > 2) return;

  position_kd_filter = limit(position_kd_filter, (uint16_t)10000);
  kd_spd = limit(kd_spd, (uint16_t)10000);

  tx_frame.data[0] = 0xC0 | ack_status;
  tx_frame.data[1] = 0x04;
  tx_frame.data[2] = position_kd_filter >> 8;
  tx_frame.data[3] = position_kd_filter & 0xff;
  tx_frame.data[4] = kd_spd >> 8;
  tx_frame.data[5] = kd_spd & 0xff;

  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
}

void OdMotorDriver::OdSetCommunicationMode(uint8_t communication_mode) {
  OdMotorDriver::ConfigOdMotor(communication_mode);
  Timer::ThreadSleepFor(setup_sleep_time);
}

bool OdMotorDriver::CheckId() {
  can_frame tx_frame;
  tx_frame.can_id = 0x7FF;
  tx_frame.can_dlc = 0x04;
  tx_frame.data[0] = 0xFF;
  tx_frame.data[1] = 0xFF;
  tx_frame.data[2] = 0x00;
  tx_frame.data[3] = 0x82;
  can_->transmit(tx_frame);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_count++;
  }
  Timer::ThreadSleepFor(normal_sleep_time);
  if (get_ret_id() == board_id_) {
    return true;
  } else {
    logger_->warn("motor_id: {0}\tread id {1}\n", board_id_, get_ret_id());
    return false;
  }
}
void OdMotorDriver::CanSendMsg(const can_frame& tx_frame) { can_->transmit(tx_frame); }

uint32_t OdMotorDriver::MotorKeyParamValid() {
  uint32_t ret = 0;
  for (int i = 0x06; i <= 0x13; i++) {
    OdMotorDriver::MotorGetParam(i);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (timeout_ != kTIMEOUT) {
    logger_->warn("motor_id: {0}\ttimeout error!\n", board_id_);
    logger_->warn("motor_id: {0}\ttimeout: {1}\n", board_id_, timeout_);
    ret |= 1 << TIMEOUT;
  }
  if (fabs(m_ntc_motor_beta - kM_NTC_MOTOR_BETA) > kERROR) {
    logger_->warn("motor_id: {0}\tm_ntc_motor_beta error!\n", board_id_);
    logger_->warn("motor_id: {0}\tm_ntc_motor_beta: {1}\n", board_id_, m_ntc_motor_beta);
    ret |= 1 << M_NTC_MOTOR_BETA;
  }
  if (si_motor_poles != kSI_MOTOR_POLES) {
    logger_->warn("motor_id: {0}\tsi_motor_poles error!\n", board_id_);
    logger_->warn("motor_id: {0}\tsi_motor_poles: {1}\n", board_id_, si_motor_poles);
    ret |= 1 << SI_MOTOR_POLES;
  }
  if (gear_ratio_ != p_pid_ang_div) {
    logger_->warn("motor_id: {0}\tgear_ratio error!\n", board_id_);
    logger_->warn("motor_id: {0}\tgear_ratio: {1}\n", board_id_, gear_ratio_);
    logger_->warn("motor_id: {0}\tp_pid_ang_div: {1}\n", board_id_, p_pid_ang_div);
    ret |= 1 << GEAR_RATIO;
  }
  if (gear_ratio_ == 1) {
    if (fabs(motor_kp_spd - kS_PID_KP_1) > kERROR) {
      logger_->warn("motor_id: {0}\tkp_spd error!\n", board_id_);
      logger_->warn("motor_id: {0}\tkp_spd: {1}\n", board_id_, motor_kp_spd);
      ret |= 1 << S_PID_KP;
    }
    if (fabs(motor_ki_spd - kS_PID_KI_1) > kERROR) {
      logger_->warn("motor_id: {0}\tki_spd error!\n", board_id_);
      logger_->warn("motor_id: {0}\tki_spd: {1}\n", board_id_, motor_ki_spd);
      ret |= 1 << S_PID_KI;
    }
    if (fabs(motor_kd_spd - kS_PID_KD_1) > kERROR) {
      logger_->warn("motor_id: {0}\tkd_spd error!\n", board_id_);
      logger_->warn("motor_id: {0}\tkd_spd: {1}\n", board_id_, motor_kd_spd);
      ret |= 1 << S_PID_KD;
    }
    if (fabs(motor_kp_pos - kP_PID_KP_1) > kERROR) {
      logger_->warn("motor_id: {0}\tkp_pos error!\n", board_id_);
      logger_->warn("motor_id: {0}\tkp_pos: {1}\n", board_id_, motor_kp_pos);
      ret |= 1 << P_PID_KP;
    }
    if (fabs(motor_ki_pos - kP_PID_KI_1) > kERROR) {
      logger_->warn("motor_id: {0}\tki_pos error!\n", board_id_);
      logger_->warn("motor_id: {0}\tki_pos: {1}\n", board_id_, motor_ki_pos);
      ret |= 1 << P_PID_KI;
    }
    if (fabs(motor_kd_pos - kP_PID_KD_1) > kERROR) {
      logger_->warn("motor_id: {0}\tkd_pos error!\n", board_id_);
      logger_->warn("motor_id: {0}\tkd_pos: {1}\n", board_id_, motor_kd_pos);
      ret |= 1 << P_PID_KD;
    }
  } else if (gear_ratio_ == 10) {
    if (fabs(motor_kp_spd - kS_PID_KP_10) > kERROR) {
      logger_->warn("motor_id: {0}\tkp_spd error!\n", board_id_);
      logger_->warn("motor_id: {0}\tkp_spd: {1}\n", board_id_, motor_kp_spd);
      ret |= 1 << S_PID_KP;
    }
    if (fabs(motor_ki_spd - kS_PID_KI_10) > kERROR) {
      logger_->warn("motor_id: {0}\tki_spd error!\n", board_id_);
      logger_->warn("motor_id: {0}\tki_spd: {1}\n", board_id_, motor_ki_spd);
      ret |= 1 << S_PID_KI;
    }
    if (fabs(motor_kd_spd - kS_PID_KD_10) > kERROR) {
      logger_->warn("motor_id: {0}\tkd_spd error!\n", board_id_);
      logger_->warn("motor_id: {0}\tkd_spd: {1}\n", board_id_, motor_kd_spd);
      ret |= 1 << S_PID_KD;
    }
    if (fabs(motor_kp_pos - kP_PID_KP_10) > kERROR) {
      logger_->warn("motor_id: {0}\tkp_pos error!\n", board_id_);
      logger_->warn("motor_id: {0}\tkp_pos: {1}\n", board_id_, motor_kp_pos);
      ret |= 1 << P_PID_KP;
    }
    if (fabs(motor_ki_pos - kP_PID_KI_10) > kERROR) {
      logger_->warn("motor_id: {0}\tki_pos error!\n", board_id_);
      logger_->warn("motor_id: {0}\tki_pos: {1}\n", board_id_, motor_ki_pos);
      ret |= 1 << P_PID_KI;
    }
    if (fabs(motor_kd_pos - kP_PID_KD_10) > kERROR) {
      logger_->warn("motor_id: {0}\tkd_pos error!\n", board_id_);
      logger_->warn("motor_id: {0}\tkd_pos: {1}\n", board_id_, motor_kd_pos);
      ret |= 1 << P_PID_KD;
    }
  } else if (gear_ratio_ == 36) {
    if (fabs(motor_kp_spd - kS_PID_KP_36) > kERROR) {
      logger_->warn("motor_id: {0}\tkp_spd error!\n", board_id_);
      logger_->warn("motor_id: {0}\tkp_spd: {1}\n", board_id_, motor_kp_spd);
      ret |= 1 << S_PID_KP;
    }
    if (fabs(motor_ki_spd - kS_PID_KI_36) > kERROR) {
      logger_->warn("motor_id: {0}\tki_spd error!\n", board_id_);
      logger_->warn("motor_id: {0}\tki_spd: {1}\n", board_id_, motor_ki_spd);
      ret |= 1 << S_PID_KI;
    }
    if (fabs(motor_kd_spd - kS_PID_KD_36) > kERROR) {
      logger_->warn("motor_id: {0}\tkd_spd error!\n", board_id_);
      logger_->warn("motor_id: {0}\tkd_spd: {1}\n", board_id_, motor_kd_spd);
      ret |= 1 << S_PID_KD;
    }
    if (fabs(motor_kp_pos - kP_PID_KP_36) > kERROR) {
      logger_->warn("motor_id: {0}\tkp_pos error!\n", board_id_);
      logger_->warn("motor_id: {0}\tkp_pos: {1}\n", board_id_, motor_kp_pos);
      ret |= 1 << P_PID_KP;
    }
    if (fabs(motor_ki_pos - kP_PID_KI_36) > kERROR) {
      logger_->warn("motor_id: {0}\tki_pos error!\n", board_id_);
      logger_->warn("motor_id: {0}\tki_pos: {1}\n", board_id_, motor_ki_pos);
      ret |= 1 << P_PID_KI;
    }
    if (fabs(motor_kd_pos - kP_PID_KD_36) > kERROR) {
      logger_->warn("motor_id: {0}\tkd_pos error!\n", board_id_);
      logger_->warn("motor_id: {0}\tkd_pos: {1}\n", board_id_, motor_kd_pos);
      ret |= 1 << P_PID_KD;
    }
  } else {
    logger_->warn("motor_id: {0}\tgear_ratio error!\n", board_id_);
    logger_->warn("motor_id: {0}\tgear_ratio: {1}\n", board_id_, gear_ratio_);
    ret |= 1 << GEAR_RATIO;
  }
  if (foc_motor_r < kMIN_RESISTANCE || foc_motor_r > kMAX_RESISTANCE) {
    logger_->warn("motor_id: {0}\tfoc_motor_r error!\n", board_id_);
    logger_->warn("motor_id: {0}\tfoc_motor_r: {1}\n", board_id_, foc_motor_r);
    ret |= 1 << FOC_MOTOR_R;
  }
  if (foc_motor_l < kMIN_INDUCTANCE || foc_motor_l > kMAX_INDUCTANCE) {
    logger_->warn("motor_id: {0}\tfoc_motor_l error!\n", board_id_);
    logger_->warn("motor_id: {0}\tfoc_motor_l: {1}\n", board_id_, foc_motor_l);
    ret |= 1 << FOC_MOTOR_L;
  }
  return ret;
}
}  // namespace arm
