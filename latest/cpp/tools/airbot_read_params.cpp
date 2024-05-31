#include <airbot/airbot.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include "airbot/utils.hpp"
#include "argparse/argparse.hpp"

enum flag {
  UNFLAG = 0,
  TESTED = 1,
  PASSED = 3,
};
const std::string red("\033[0;31m");
const std::string green("\033[0;32m");
const std::string yellow("\033[0;33m");
const std::string blue("\033[0;34m");
const std::string cyan("\033[0;36m");
const std::string reset("\033[0m");

std::map<std::string, int> status_map = {
    {"unflag", UNFLAG},
    {"tested", TESTED},
    {"passed", PASSED},
};

int main(int argc, char **argv) {
  argparse::ArgumentParser program("airbot_read_params", AIRBOT_VERSION);
  program.add_description("A simple program to read the parameters of the boards on the arm.");
  program.add_argument("-m", "--master")
      .required()
      .default_value("can0")
      .help("Can device interface of the master arm. Default: can0");
  program.add_argument("-e", "--master-end-mode")
      .default_value("none")
      .choices("teacher", "gripper", "yinshi", "newteacher", "none", "teacherv2")
      .help(
          "The mode of the master arm end effector. Available choices: \n"
          "\"teacher\": The demonstrator equipped with Damiao motor \n"
          "\"gripper\": The gripper equipped with Damiao motor \n"
          "\"yinshi\": The Yinshi two-finger gripper \n"
          "\"newteacher\": The demonstrator equipped with self-developed "
          "motor \n"
          "\"teacherv2\": The V2 version of demonstrator equipped with self-developed motor \n"
          "\"none\": The arm is not equipped with end effector.");
  program.add_argument("--forearm-type")
      .default_value("DM")
      .choices("DM", "OD")
      .help("The type of forearm. Available choices: \"DM\": Damiao motor, \"OD\": Self-developed motors");
  program.add_argument("--verbose")
      .help("Display more detailed information about motors")
      .default_value(false)
      .implicit_value(true);

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }
  // setup logger
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      (std::string("logs/tool_read_logs") + get_timestring() + ".log").c_str(), 1024 * 1024, 10, false));
  std::shared_ptr<spdlog::logger> logger = setup_logger(sinks);
  spdlog::flush_every(std::chrono::seconds(1));
  logger->set_level(spdlog::level::info);

  std::string interface = program.get<std::string>("--master");
  std::string gripper_type = program.get<std::string>("--master-end-mode");
  std::string forearm_type = program.get<std::string>("--forearm-type");
  bool verbose = program.get<bool>("--verbose");

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::vector<std::shared_ptr<arm::MotorDriver>> motor_driver_;
  for (uint8_t i = 0; i < 3; i++) {
    motor_driver_.push_back(arm::MotorDriver::MotorCreate(i + 1, interface.c_str(), "OD"));
  }
  for (uint8_t i = 3; i < 6; i++) {
    motor_driver_.push_back(arm::MotorDriver::MotorCreate(i + 1, interface.c_str(), forearm_type));
  }
  if (gripper_type != "none") {
    motor_driver_.push_back(arm::MotorDriver::MotorCreate(7, interface.c_str(), gripper_type));
  }
  std::unique_ptr<arm::InterfaceBoardBase> interface_board_base_ =
      std::make_unique<arm::InterfaceBoardBase>(0, interface.c_str());
  std::unique_ptr<arm::InterfaceBoardEnd> interface_board_end_ =
      std::make_unique<arm::InterfaceBoardEnd>(8, interface.c_str());
  // while (1) {
  // deinit dm motors
  if (forearm_type == "DM") {
    for (uint8_t motor_id = 3; motor_id <= 6; motor_id++) {
      motor_driver_[motor_id - 1]->MotorUnlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  if (gripper_type == "teacher" || gripper_type == "gripper") {
    motor_driver_[6]->MotorUnlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  // send get motor param commands
  for (uint8_t motor_id = 1; motor_id <= (forearm_type == "OD" ? 6 : 3); motor_id++) {
    for (uint8_t cmd_id : {1, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}) {
      motor_driver_[motor_id - 1]->MotorGetParam(cmd_id);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  if (forearm_type == "DM") {
    for (uint8_t motor_id = 4; motor_id <= 6; motor_id++) {
      for (uint8_t cmd_id : {1, 7, 10, 16, 17, 18, 19, 22}) {
        motor_driver_[motor_id - 1]->MotorGetParam(cmd_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }
  if (gripper_type == "newteacher" || gripper_type == "teacherv2") {
    for (uint8_t cmd_id : {1, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}) {
      motor_driver_[6]->MotorGetParam(cmd_id);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  } else if (gripper_type == "teacher" || gripper_type == "gripper") {
    for (uint8_t cmd_id : {1, 7, 10, 16, 17, 18, 19, 22}) {
      motor_driver_[6]->MotorGetParam(cmd_id);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  // init motors
  if (forearm_type == "DM") {
    for (uint8_t motor_id = 4; motor_id <= 6; motor_id++) {
      motor_driver_[motor_id - 1]->MotorLock();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  if (gripper_type == "teacher" || gripper_type == "gripper") {
    motor_driver_[6]->MotorLock();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  interface_board_base_->Init();
  interface_board_base_->GetCmd(arm::BoardDriver::CMD_PRODUCT_FLAG);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  printf("Board#%d:%15s%8s%s%18s%s%s%18s%s  ", interface_board_base_->get_board_id(),
         interface_board_base_->get_board_type().c_str(), interface_board_base_->get_firmware_version().c_str(),
         cyan.c_str(), interface_board_base_->get_sn_code().c_str(), reset.c_str(), green.c_str(),
         interface_board_base_->get_arm_sn_code().c_str(), reset.c_str());
  uint32_t current_flag = interface_board_base_->get_product_flag();
  if (current_flag == -1) current_flag = 0;
  for (int i = 0; i < 6; i++)
    if ((current_flag & (PASSED << (2 * i))) == (PASSED << (2 * i)))
      std::cout << green << "Y" << reset;
    else
      std::cout << red << "N" << reset;
  printf("\n");

  for (uint8_t motor_id = 0; motor_id < (gripper_type != "none" ? 7 : 6); motor_id++) {
    auto motor_driver = motor_driver_[motor_id];
    motor_driver->MotorInit();
    motor_driver->GetCmd(arm::MotorDriver::CMD_PRODUCT_FLAG);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (motor_driver->get_firmware_version() == "") {
      printf("Board#%d:%s%15s%s\n", motor_driver->get_board_id(), red.c_str(), "Disconnected", reset.c_str());
      continue;
    }
    printf("Board#%d:%15s%8s%s%18s%s%s%18s%s  ", motor_driver->get_board_id(), motor_driver->get_board_type().c_str(),
           motor_driver->get_firmware_version().c_str(), cyan.c_str(), motor_driver->get_sn_code().c_str(),
           reset.c_str(), green.c_str(), motor_driver->get_arm_sn_code().c_str(), reset.c_str());
    for (int i = 0; i < 6; i++)
      if ((current_flag & (PASSED << (2 * i))) == (PASSED << (2 * i)))
        std::cout << green << "Y" << reset;
      else
        std::cout << red << "N" << reset;
    printf("  pos: % 2.4f vel: % 2.4f cur: % 2.4f", motor_driver->get_motor_pos(), motor_driver->get_motor_spd(),
           motor_driver->get_motor_current());
    printf("\n");
    if (verbose) {
      if (auto motor_driver = dynamic_cast<arm::OdMotorDriver *>(motor_driver_[motor_id].get())) {
        printf(
            "\t\tgear_ratio: %f\tfoc_motor_r: %f\tfoc_motor_l: %f\ttimeout: "
            "%d\tntc_motor_beta: %f\tsi_motor_poles: %d\n",
            motor_driver->get_gear_ratio(), motor_driver->get_foc_motor_r(), motor_driver->get_foc_motor_l(),
            motor_driver->get_timeout(), motor_driver->get_m_ntc_motor_beta(), motor_driver->get_si_motor_poles());
        printf(
            "\t\tv_kp: %f\tv_ki: %f\tv_kd: %f\tp_kp: %f\tp_ki: %f\tp_kd: "
            "%f\tp_kd_filter: %f\n",
            motor_driver->get_motor_kp_spd(), motor_driver->get_motor_ki_spd(), motor_driver->get_motor_kd_spd(),
            motor_driver->get_motor_kp_pos(), motor_driver->get_motor_ki_pos(), motor_driver->get_motor_kd_pos(),
            motor_driver->get_motor_kd_pos_filter());
      } else if (auto motor_driver = dynamic_cast<arm::DmMotorDriver *>(motor_driver_[motor_id].get())) {
        printf("\t\tv_kp: %f\tv_ki: %f\tv_kd: %f\tp_kp: %f\tp_ki:%f\tp_kd: %f\n", motor_driver->get_motor_kp_spd(),
               motor_driver->get_motor_ki_spd(), motor_driver->get_motor_kd_spd(), motor_driver->get_motor_kp_pos(),
               motor_driver->get_motor_ki_pos(), motor_driver->get_motor_kd_pos());
      }
      printf("\n");
    }
    current_flag = motor_driver->get_product_flag();
    if (current_flag == -1) current_flag = 0;
  }

  interface_board_end_->Init();
  interface_board_end_->GetCmd(arm::BoardDriver::CMD_PRODUCT_FLAG);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  printf("Board#%d:%15s%8s%s%18s%s%s%18s%s  ", interface_board_end_->get_board_id(),
         interface_board_end_->get_board_type().c_str(), interface_board_end_->get_firmware_version().c_str(),
         cyan.c_str(), interface_board_end_->get_sn_code().c_str(), reset.c_str(), green.c_str(),
         interface_board_end_->get_arm_sn_code().c_str(), reset.c_str());
  current_flag = interface_board_end_->get_product_flag();
  if (current_flag == -1) current_flag = 0;
  for (int i = 0; i < 6; i++)
    if ((current_flag & (PASSED << (2 * i))) == (PASSED << (2 * i)))
      std::cout << green << "Y" << reset;
    else
      std::cout << red << "N" << reset;
  printf("\n");

  return 0;
}
