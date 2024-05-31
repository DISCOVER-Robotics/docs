#include "airbot/modules/motors/oid_motor_driver.hpp"

namespace arm {
OidMotorDriver::OidMotorDriver(uint16_t motor_id, std::string can_interface)
    : MotorDriver(), can_(SocketCAN::get(can_interface)) {
  board_id_ = motor_id;
  CanCbkCondition can_condition =
      std::bind([motor_id](const can_frame& frame) { return (bool)(frame.can_id == motor_id); }, std::placeholders::_1);
  CanCbkFunc can_callback = std::bind([this](const can_frame& frame) {}, std::placeholders::_1);

  can_->add_can_callback(
      std::make_tuple(std::string("motor#") + std::to_string(board_id_), can_condition, can_callback));
}

OidMotorDriver::~OidMotorDriver() { can_->remove_can_callback(std::string("motor#") + std::to_string(board_id_)); }

bool OidMotorDriver::MotorInit() { return true; }

void OidMotorDriver::MotorPosModeCmd(float pos, float spd, bool ignore_limit) {
  can_frame tx_frame;
  tx_frame.can_id = board_id_;
  tx_frame.can_dlc = 0x04;
  tx_frame.data[0] = 0x04;
  tx_frame.data[1] = board_id_;
  tx_frame.data[2] = 0x01;
  tx_frame.data[3] = 0x00;
  can_->transmit(tx_frame);
}

void OidMotorDriver::MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
  can_frame tx_frame;
  tx_frame.can_id = board_id_;
  tx_frame.can_dlc = 0x04;
  tx_frame.data[0] = 0x04;
  tx_frame.data[1] = board_id_;
  tx_frame.data[2] = 0x01;
  tx_frame.data[3] = 0x00;
  can_->transmit(tx_frame);
}

void OidMotorDriver::CanRxMsgCallback(const can_frame& frame, uint8_t index) {
  int value = frame.data[3] | frame.data[4] << 8 | frame.data[5] << 16 | frame.data[6] << 24;
  double poss = value * 360.0 / 32768;
  if (poss > 180) {
    poss -= 360;
  }
  if (board_id_ == 2) poss = -poss;
  motor_pos_ = poss / 180 * 3.14;
}
}  // namespace arm
