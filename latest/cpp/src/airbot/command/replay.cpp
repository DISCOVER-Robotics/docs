#include <nlohmann/json.hpp>

#include "airbot/command/command_base.hpp"
template <std::size_t DOF>
void arm::Robot<DOF>::record_save(const std::string& filename) {
  // condition 1: invalid arm mode
  if (arm_mode_.load(std::memory_order_relaxed) == ArmMode::RECORDING) {
    logger_->warn("Still recording");
    return;
  }
  // condition 2: robot is stopped
  bool is_stopped;
  {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    is_stopped = robot_fb_data_.is_robot_stopped;
  }
  if (!is_stopped) {
    logger_->warn("robot is still moving");
    return;
  }

  if (!fs::exists("records")) {
    try {
      if (fs::create_directories("records")) {
        logger_->info("folder created");
      }
    } catch (const fs::filesystem_error& e) {
      logger_->error("folder creat failed");
      return;
    }
  } else {
    logger_->error("Records directory already exists");
    return;
  }
  RobotRecordData<DOF> temp_data;
  {
    std::unique_lock<std::mutex> lock(record_mutex_);
    temp_data = recorded_data_;
    recorded_data_.init();
  }
  std::ofstream f(filename);

  nlohmann::json data;
  data["q"] = nlohmann::json::array();
  data["v"] = nlohmann::json::array();
  data["pose"] = nlohmann::json::array();
  data["end"] = nlohmann::json::array();
  data["time"] = nlohmann::json::array();
  for (auto&& i : temp_data.q_records) data["q"].push_back({i[0], i[1], i[2], i[3], i[4], i[5]});
  for (auto&& i : temp_data.v_records) data["v"].push_back({i[0], i[1], i[2], i[3], i[4], i[5]});
  for (auto&& i : temp_data.wp_records)
    data["pose"].push_back(
        {{i.first[0], i.first[1], i.first[2]}, {i.second[0], i.second[1], i.second[2], i.second[3]}});
  for (auto&& i : temp_data.end_records) data["end"].push_back(i);
  for (auto&& i : temp_data.time_records) data["time"].push_back(i);
  f << data.dump(2);
  f.close();
}
template <std::size_t DOF>
void arm::Robot<DOF>::record_load(const std::string& filename) {
  // condition 1: invalid arm mode
  if (arm_mode_.load(std::memory_order_relaxed) == ArmMode::RECORDING) {
    logger_->warn("Still recording");
    return;
  }

  if (arm_mode_.load(std::memory_order_relaxed) == ArmMode::REPLAYING) {
    logger_->warn("Still replaying");
    return;
  }

  // condition 2: file is not empty
  if (filename.empty()) {
    logger_->warn("record file is empty, file name wrong?");
    return;
  }
  // condition 3: robot is not replaying record data
  // bool is_replaying;
  // {
  //   std::unique_lock<std::shared_mutex> lock(fb_mutex_);
  //   is_replaying = robot_fb_data_.replay_state;
  // }
  // if (is_replaying) {
  //   logger_->warn("Still replaying");
  //   return;
  // }

  std::ifstream f(filename.c_str());
  if (!f.good()) {
    logger_->error("File not found");
    return;
  }
  nlohmann::json data = nlohmann::json::parse(f);
  replay_data_.init();
  for (auto&& i : data["end"]) replay_data_.end_records.push_back(i.get<double>());
  for (auto&& i : data["time"]) replay_data_.time_records.push_back(i.get<double>());
  for (auto&& i : data["pose"]) replay_data_.wp_records.push_back(i.get<Frame>());
  for (auto&& i : data["q"]) replay_data_.q_records.push_back(i.get<Joints<DOF>>());
  for (auto&& i : data["v"]) replay_data_.v_records.push_back(i.get<Joints<DOF>>());

  f.close();
}
