

#include "airbot/modules/motors/motor_driver.hpp"

#include "airbot/modules/motors/dm_motor_driver.hpp"
#include "airbot/modules/motors/gripper485_motor_driver.hpp"
#include "airbot/modules/motors/od_motor_driver.hpp"
#include "airbot/modules/motors/oid_motor_driver.hpp"

namespace arm {
uint8_t MotorDriver::motor_error_type_ = NONE_ERROR;

std::unique_ptr<MotorDriver> MotorDriver::MotorCreate(uint16_t motor_id, const char* interface,
                                                      const std::string type) {
  if (motor_id <= 0 || motor_id > 7) {
    throw std::runtime_error("Motor ID out of range");
  }
  if (type == "OD" || type == "newteacher" || type == "teacherv2") {
    return std::make_unique<OdMotorDriver>(motor_id, interface);
  } else if (type == "OID") {
    return std::make_unique<OidMotorDriver>(motor_id, interface);
  } else if (type == "DM" || type == "gripper" || type == "teacher") {
    return std::make_unique<DmMotorDriver>(motor_id, interface);
  } else if (type == "485" || type == "yinshi") {
    return std::make_unique<Gripper485MotorDriver>(motor_id, interface);
  } else {
    throw std::runtime_error("Motor type not supported");
  }
}

bool MotorDriver::MotorCurrentDetect() {
  static int detect_current_counter[6] = {0};
  if (get_motor_current() < -10.0f) {
    return false;
  }
  if (abs(get_motor_current()) > 25.0f) {
    detect_current_counter[board_id_ - 1]++;
  } else {
    detect_current_counter[board_id_ - 1] = 0;
  }
  if (detect_current_counter[board_id_ - 1] > 60) {
    logger_->info("motor {} is over current\n", board_id_);
    motor_error_type_ = OVER_CURRENT;
    return true;
  }
  return false;
}

bool MotorDriver::MotorCommunicationDetect() {
  static int detect_communication_counter[6] = {0};
  if (heartbeat_detect_counter_ != 0) {
    detect_communication_counter[board_id_ - 1]++;
  } else {
    heartbeat_detect_counter_++;
    detect_communication_counter[board_id_ - 1] = 0;
  }
  if (detect_communication_counter[board_id_ - 1] > 1e5) {
    logger_->warn("motor {} is communication error\n", board_id_);
    motor_error_type_ = COMMUNICATION_ERROR;
    return true;
  }
  return false;
}

bool MotorDriver::MotorTemperatureDetect() {
  static int detect_temperature_counter[6] = {0};
  if (motor_temperature_ > 80.0f) {
    detect_temperature_counter[board_id_ - 1]++;
  } else {
    detect_temperature_counter[board_id_ - 1] = 0;
  }
  if (detect_temperature_counter[board_id_ - 1] > 60) {
    logger_->warn("motor {} is over temperature\n", board_id_);
    motor_error_type_ = OVER_TEMPERATURE;
    return true;
  }
  return false;
}

bool MotorDriver::MotorErrorDetect() {
  return MotorCurrentDetect() || MotorCommunicationDetect() || MotorTemperatureDetect();
}

void MotorDriver::MotorErrorModeCmd() {
  switch (motor_error_type_) {
    case OVER_CURRENT:
      MotorPosModeCmd(0.0f, 0.5f);
      break;
    case OVER_TEMPERATURE:
      MotorPosModeCmd(0.0f, 0.5f);
      break;
    case COMMUNICATION_ERROR:
      MotorPosModeCmd(get_motor_pos(), 0.5f);
      break;
    default:
      break;
  }
}

// MotorDriver* motor[6];

}  // namespace arm
