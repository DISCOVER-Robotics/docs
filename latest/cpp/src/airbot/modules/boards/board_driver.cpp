#include "airbot/modules/boards/board_driver.hpp"

#include "airbot/airbot.hpp"
namespace arm {
BoardDriver::BoardDriver() {
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_st>());
  logger_ = setup_logger(sinks);
}
bool BoardDriver::Init() {
  DEVICE_ID = get_board_id();
  GET_CMD_ID = (DEVICE_ID | GET_CMD << 7);
  SET_CMD_ID = (DEVICE_ID | SET_CMD << 7);
  RET_CMD_ID = (DEVICE_ID | RET_CMD << 7);
  GetCmd(CMD_DEVICE_ID);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  GetCmd(CMD_FIRMWARE_VERSION);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  GetCmd(CMD_BOARD_SN_CODE);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  GetCmd(CMD_ARM_SN_CODE);
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  logger_->info(
      "board_id: {}\tboard_type: {}\tfirmware_version: {}\thardware_version: {}\tboard_sn_code: {}\t arm_sn_code: "
      "{}",
      DEVICE_ID, board_type_, firmware_version_, hardware_version_, board_sn_code_, arm_sn_code_);
  if (ret_id_ != DEVICE_ID) {
    logger_->error("board {} id error or device lost connection!!!", DEVICE_ID);
    return false;
  }

  if (firmware_version_ > ("V" + std::string(AIRBOT_VERSION)).substr(0, 6)) {
    logger_->error("board {} firmware version higher than airbot version {}!!!", DEVICE_ID,
                   ("V" + std::string(AIRBOT_VERSION)).substr(0, 6).c_str());
    return false;
  }

  if (board_sn_code_ < "0000000000000000" || board_sn_code_ > "zzzzzzzzzzzzzzzz") {
    logger_->error("board {} sn_code error!!!", DEVICE_ID);
    return false;
  }
  if (arm_sn_code_ < "0000000000000000" || arm_sn_code_ > "zzzzzzzzzzzzzzzz") {
    logger_->error("board {} arm_sn_code error!!!", DEVICE_ID);
    return false;
  }
  return true;
}

void BoardDriver::GetCmd(uint8_t cmd_id) {
  can_frame tx_frame;
  tx_frame.can_id = GET_CMD_ID;
  tx_frame.can_dlc = 2;
  tx_frame.data[0] = cmd_id;
  tx_frame.data[1] = FRAME_1;
  CanSendMsg(tx_frame);
}

void BoardDriver::SetCmd(uint8_t cmd_id, uint8_t framd_id, uint8_t* data) {
  can_frame tx_frame;
  tx_frame.can_id = SET_CMD_ID;
  tx_frame.can_dlc = 8;
  tx_frame.data[0] = cmd_id;
  tx_frame.data[1] = framd_id;
  for (int i = 0; i < 4; i++) {
    tx_frame.data[i + 2] = data[i];
  }
  CanSendMsg(tx_frame);
}

void BoardDriver::SetCmd(uint8_t cmd_id, uint8_t framd_id, uint32_t data) {
  can_frame tx_frame;
  tx_frame.can_id = SET_CMD_ID;
  tx_frame.can_dlc = 8;
  tx_frame.data[0] = cmd_id;
  tx_frame.data[1] = framd_id;
  for (int i = 0; i < 4; i++) {
    tx_frame.data[i + 2] = (uint8_t)(data >> (i * 8));
  }
  CanSendMsg(tx_frame);
}

void BoardDriver::SetCmd(uint8_t cmd_id, uint8_t framd_id, float data) {
  can_frame tx_frame;
  tx_frame.can_id = SET_CMD_ID;
  tx_frame.can_dlc = 8;
  tx_frame.data[0] = cmd_id;
  tx_frame.data[1] = framd_id;
  uint32_t data_int = *(uint32_t*)(&data);
  for (int i = 0; i < 4; i++) {
    tx_frame.data[i + 2] = (uint8_t)(data_int >> (i * 8));
  }
  CanSendMsg(tx_frame);
}

void BoardDriver::SetCmd(uint8_t cmd_id, uint8_t framd_id, double data) { SetCmd(cmd_id, framd_id, (float)data); }

BoardDriver::snap_mode_e BoardDriver::get_snap_mode() {
  auto res = snap_mode_;
  snap_mode_ = SNAP_RELEASE;
  return res;
}
bool BoardDriver::IsSnCodeValid(const std::string& sn_code) {
  if (sn_code.size() != 16) return false;
  for (int i = 0; i < 16; i++) {
    if (sn_code[i] < '0' || sn_code[i] > 'Z' || (sn_code[i] > '9' && sn_code[i] < 'A')) return false;
  }
  int sum = 0;
  for (int i = 0; i < 15; i++) {
    sum ^= sn_code[i];
  }
  sum %= 36;
  if (sn_code[15] != (sum < 10 ? sum + '0' : sum - 10 + 'A')) return false;
  return true;
}
void BoardDriver::CanRxMsgCallback(const can_frame& rx_frame) {
  ret_id_ = board_id_;
  uint8_t cmd_id = rx_frame.data[0];
  uint8_t frame_id = rx_frame.data[1];
  switch (cmd_id) {
    case CMD_DEVICE_ID:
      ret_id_ = rx_frame.data[2] | (rx_frame.data[3] << 8) | (rx_frame.data[4] << 16) | (rx_frame.data[5] << 24);
      break;
    case CMD_HARDWARE_VERSION:
      hardware_version_ = std::string("V" + std::to_string(rx_frame.data[3]) + "." + std::to_string(rx_frame.data[4]) +
                                      "." + std::to_string(rx_frame.data[5]));
      break;
    case CMD_FIRMWARE_VERSION:
      firmware_version_ = std::string("V" + std::to_string(rx_frame.data[3]) + "." + std::to_string(rx_frame.data[4]) +
                                      "." + std::to_string(rx_frame.data[5]));
      break;
    case CMD_BOARD_SN_CODE:
      switch (rx_frame.data[1]) {
        case FRAME_1:
          for (int i = 0; i < 4; i++) board_sn_code_[i] = rx_frame.data[i + 2];
          break;
        case FRAME_2:
          for (int i = 0; i < 4; i++) board_sn_code_[i + 4] = rx_frame.data[i + 2];
          break;
        case FRAME_3:
          for (int i = 0; i < 4; i++) board_sn_code_[i + 8] = rx_frame.data[i + 2];
          break;
        case FRAME_4:
          for (int i = 0; i < 4; i++) board_sn_code_[i + 12] = rx_frame.data[i + 2];
          break;
        default:
          break;
      }
      break;
    case CMD_ARM_SN_CODE:
      switch (rx_frame.data[1]) {
        case FRAME_1:
          for (int i = 0; i < 4; i++) arm_sn_code_[i] = rx_frame.data[i + 2];
          break;
        case FRAME_2:
          for (int i = 0; i < 4; i++) arm_sn_code_[i + 4] = rx_frame.data[i + 2];
          break;
        case FRAME_3:
          for (int i = 0; i < 4; i++) arm_sn_code_[i + 8] = rx_frame.data[i + 2];
          break;
        case FRAME_4:
          for (int i = 0; i < 4; i++) arm_sn_code_[i + 12] = rx_frame.data[i + 2];
          break;
        default:
          break;
      }
      break;
    case CMD_MOTOR_BODY_SN_CODE:
      switch (rx_frame.data[1]) {
        case FRAME_1:
          for (int i = 0; i < 4; i++) body_sn_code_[i] = rx_frame.data[i + 2];
          break;
        case FRAME_2:
          for (int i = 0; i < 4; i++) body_sn_code_[i + 4] = rx_frame.data[i + 2];
          break;
        case FRAME_3:
          for (int i = 0; i < 4; i++) body_sn_code_[i + 8] = rx_frame.data[i + 2];
          break;
        case FRAME_4:
          for (int i = 0; i < 4; i++) body_sn_code_[i + 12] = rx_frame.data[i + 2];
          break;
        default:
          break;
      }
      break;
    case CMD_MOTOR_SN_CODE:
      switch (rx_frame.data[1]) {
        case FRAME_1:
          for (int i = 0; i < 4; i++) whole_sn_code_[i] = rx_frame.data[i + 2];
          break;
        case FRAME_2:
          for (int i = 0; i < 4; i++) whole_sn_code_[i + 4] = rx_frame.data[i + 2];
          break;
        case FRAME_3:
          for (int i = 0; i < 4; i++) whole_sn_code_[i + 8] = rx_frame.data[i + 2];
          break;
        case FRAME_4:
          for (int i = 0; i < 4; i++) whole_sn_code_[i + 12] = rx_frame.data[i + 2];
          break;
        default:
          break;
      }
      break;
    case CMD_SNAP_SIGNAL:
      snap_mode_ = static_cast<snap_mode_e>(rx_frame.data[2] | (rx_frame.data[3] << 8) | (rx_frame.data[4] << 16) |
                                            (rx_frame.data[5] << 24));
      break;
    case CMD_PRODUCT_FLAG:
      product_flag_ = rx_frame.data[2] | (rx_frame.data[3] << 8) | (rx_frame.data[4] << 16) | (rx_frame.data[5] << 24);
      break;
    default:
      break;
  }
}
}  // namespace arm
