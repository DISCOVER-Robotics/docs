#include "airbot/modules/boards/interface_board_base.hpp"

#include <iostream>
#define BASE_MIN_FIRMWARE_VERSION "V2.6.1"
namespace arm {
InterfaceBoardBase::InterfaceBoardBase(uint16_t board_id, std::string can_interface)
    : can_(SocketCAN::get(can_interface)) {
  board_id_ = board_id;
  board_type_ = "Base board";
  int ret_cmd_id = board_id_ | RET_CMD << 7;
  CanCbkCondition can_condition = std::bind(
      [ret_cmd_id](const can_frame& frame) { return (bool)(frame.can_id == ret_cmd_id); }, std::placeholders::_1);
  CanCbkFunc can_callback = std::bind(&InterfaceBoardBase::CanRxMsgCallback, this, std::placeholders::_1);
  can_->add_can_callback(
      std::make_tuple(std::string("board#") + std::to_string(board_id_), can_condition, can_callback));
}
InterfaceBoardBase::~InterfaceBoardBase() {
  can_->remove_can_callback(std::string("board#") + std::to_string(board_id_));
}
void InterfaceBoardBase::CanRxMsgCallback(const can_frame& rx_frame) { BoardDriver::CanRxMsgCallback(rx_frame); }
void InterfaceBoardBase::CanSendMsg(const can_frame& tx_frame) { can_->transmit(tx_frame); }
bool InterfaceBoardBase::Init() {
  bool ret = BoardDriver::Init();
  if (firmware_version_ < BASE_MIN_FIRMWARE_VERSION) {
    logger_->error("board {} firmware version lower than min version {}!!!", board_id_, BASE_MIN_FIRMWARE_VERSION);
    return false;
  }
  return ret;
}
}  // namespace arm
