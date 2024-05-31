#include <ncurses.h>

#include <airbot/airbot.hpp>

#include "argparse/argparse.hpp"

const double REF_POS[3] = {MotorDriver::joint_lower_bounder_[0] * 180 / M_PI,
                           MotorDriver::joint_upper_bounder_[1] * 180 / M_PI,
                           MotorDriver::joint_lower_bounder_[2] * 180 / M_PI};

int main(int argc, char **argv) {
  // argparse
  argparse::ArgumentParser program("airbot_auto_set_zero", AIRBOT_VERSION);
  program.add_description("A simple program to set zero point of AIRBOT Play.");
  program.add_argument("-m", "--master")
      .required()
      .default_value("can0")
      .help("Can device interface of the master arm.");
  program.add_argument("-u", "--urdf")
      .default_value(std::string())
      .help("Manually provided URDF path to override default paths.");
  program.add_argument("--forearm-type")
      .default_value("DM")
      .choices("DM", "OD")
      .help("The type of forearm. Available choices: \"DM\": Damiao motor, \"OD\": Self-developed motors");

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }
  std::string urdf_path = program.get<std::string>("--urdf");
  std::string master_can = program.get<std::string>("--master");
  std::string forearm_type = program.get<std::string>("--forearm-type");
  if (urdf_path == "") urdf_path = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf";

  // Synchronization
  std::unique_ptr<arm::Robot<6>> robot;
  try {
    robot = std::make_unique<arm::Robot<6>>(urdf_path, master_can, "down", M_PI / 2, "none", forearm_type, true);
  } catch (const std::runtime_error &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  auto current_pos = robot->get_current_joint_q();
  robot->set_target_joint_q({current_pos[0], current_pos[1], current_pos[2], 0, 0, 0}, false, 0.1, true);

  double joint_pos[6];
  double read_past, read_now;
  robot->set_max_current({1, 2, 5, 10, 10, 10});

  read_past = -20000;
  read_now = -2000;
  robot->set_target_joint_v({-0.5, 0, 0, 0, 0, 0});
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while (std::abs(read_now - read_past) > 0.001) {
    read_past = read_now;
    read_now = robot->get_current_joint_q()[0];
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  joint_pos[0] = read_now;

  read_past = -20000;
  read_now = -2000;
  robot->set_target_joint_v({0, 0.5, 0, 0, 0, 0});
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while (std::abs(read_now - read_past) > 0.001) {
    read_past = read_now;
    read_now = robot->get_current_joint_q()[1];
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  joint_pos[1] = read_now;

  read_past = -20000;
  read_now = -2000;
  robot->set_max_current({1, 1, 2, 10, 10, 10});
  robot->set_target_joint_v({0, 0, -0.5, 0, 0, 0});
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while (std::abs(read_now - read_past) > 0.001) {
    read_past = read_now;
    read_now = robot->get_current_joint_q()[2];
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  joint_pos[2] = read_now;

  std::cerr << "Joint position: " << joint_pos[0] << ", " << joint_pos[1] << ", " << joint_pos[2] << ", "
            << joint_pos[3] << ", " << joint_pos[4] << ", " << joint_pos[5] << std::endl;

  robot->set_frame({REF_POS[0], REF_POS[1], REF_POS[2], 0, 0, 0});

  auto current_joint_q = robot->get_current_joint_q();
  std::cerr << "Joint positions:" << current_joint_q[0] << ", " << current_joint_q[1] << ", " << current_joint_q[2]
            << ", " << current_joint_q[3] << ", " << current_joint_q[4] << ", " << current_joint_q[5] << std::endl;
  robot->set_max_current({10, 10, 10, 10, 10, 10});
  robot->set_target_joint_q({0, 0, 0, 0, 0, 0}, false, 0.5, true);
  return 0;
}
