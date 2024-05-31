#include "airbot/modules/motors/gripper485_motor_driver.hpp"

namespace arm {
Gripper485MotorDriver::Gripper485MotorDriver(uint16_t motor_id, std::string can_interface)
    : MotorDriver(), can_(SocketCAN::get(can_interface)) {
  board_id_ = motor_id;

  CanCbkCondition can_condition =
      std::bind([motor_id](const can_frame& frame) { return (bool)(frame.can_id == motor_id); }, std::placeholders::_1);
  CanCbkFunc can_callback = std::bind(&Gripper485MotorDriver::CanRxMsgCallback, this, std::placeholders::_1);

  can_->add_can_callback(
      std::make_tuple(std::string("motor#") + std::to_string(board_id_), can_condition, can_callback));
}

Gripper485MotorDriver::~Gripper485MotorDriver() {
  can_->remove_can_callback(std::string("motor#") + std::to_string(board_id_));
}

bool Gripper485MotorDriver::MotorInit() { return true; }

void Gripper485MotorDriver::MotorPosModeCmd(float pos, float spd, bool ignore_limit) {
  can_frame tx_frame;
  tx_frame.can_id = 0x088;
  tx_frame.can_dlc = 0x06;
  tx_frame.data[0] = 0x11;
  tx_frame.data[1] = 0x01;
  uint8_t* buf = (uint8_t*)&pos;
  tx_frame.data[2] = buf[0];
  tx_frame.data[3] = buf[1];
  tx_frame.data[4] = buf[2];
  tx_frame.data[5] = buf[3];
  can_->transmit(tx_frame);
  motor_pos_ = pos;
}

void Gripper485MotorDriver::MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
  can_frame tx_frame;
  tx_frame.can_id = 0x088;
  tx_frame.can_dlc = 0x06;
  tx_frame.data[0] = 0x11;
  tx_frame.data[1] = 0x01;
  uint8_t* buf = (uint8_t*)&f_p;
  tx_frame.data[2] = buf[0];
  tx_frame.data[3] = buf[1];
  tx_frame.data[4] = buf[2];
  tx_frame.data[5] = buf[3];
  can_->transmit(tx_frame);
  motor_pos_ = f_p;
}

}  // namespace arm
