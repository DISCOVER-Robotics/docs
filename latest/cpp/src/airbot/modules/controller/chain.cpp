#include "airbot/modules/controller/chain.hpp"

// #include <spdlog/spdlog.h>

#include <iostream>
#include <kdl_parser/kdl_parser.hpp>

namespace arm {

double bias[6] = {0.0, -2.75, 2.75, 1.57, 0.0, 0.0};
double a1 = 0.1127, a3 = 0.27009588894968695, a4 = 0.308557122050828, a6 = 0.08653867207135546;

ChainDescriptor::ChainDescriptor() {
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      (std::string("logs/solver-") + get_timestring() + ".log").c_str(), 1024 * 1024, 10, false));
  logger_ = setup_logger(sinks, "solver");
  spdlog::flush_every(std::chrono::seconds(1));
  logger_->set_level(spdlog::level::info);
}

void ChainDescriptor::load_model(const std::string& model_path) {
  try {
    kdl_parser::treeFromFile(model_path, tree_);
    // link6 is supposed to be the end-effector
    // TODO replace link6 with configurable end-effector to support IK to any link
    tree_.getChain("base_link", "link6", chain_);
  } catch (...) {
    logger_->error("Fail to load model from {}", model_path);
  }

  num_joints_ = chain_.getNrOfJoints();
  num_segments_ = chain_.getNrOfSegments();
}
}  // namespace arm
