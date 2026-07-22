#pragma once

#include <cstdint>
#include <string>

namespace multi_agv_control {

enum class ExperimentPhase : std::uint8_t {
  kIdle = 0U,
  kArmed = 1U,
  kRunningNominal = 2U,
  kDeratingDown = 3U,
  kDerated = 4U,
  kRestoring = 5U,
  kFinished = 6U,
  kAborted = 7U,
};

struct SupervisorConfig {
  double activation_progress{0.3};
  double restoration_progress{0.6};
  double target_speed_ratio_left{0.4};
  double target_speed_ratio_right{0.4};
  double target_acceleration_ratio_left{0.5};
  double target_acceleration_ratio_right{0.5};
  double target_deceleration_ratio_left{0.6};
  double target_deceleration_ratio_right{0.6};
  double ramp_down_time{0.5};
  double ramp_up_time{0.5};
  std::uint8_t derating_mode{1U};
};

struct DeratingTarget {
  std::uint32_t sequence{1U};
  std::uint8_t mode{0U};
  bool active{false};
  double speed_ratio_left{1.0};
  double speed_ratio_right{1.0};
  double acceleration_ratio_left{1.0};
  double acceleration_ratio_right{1.0};
  double deceleration_ratio_left{1.0};
  double deceleration_ratio_right{1.0};
  double ramp_down_time{0.0};
  double ramp_up_time{0.0};
};

struct SupervisorSnapshot {
  ExperimentPhase phase{ExperimentPhase::kIdle};
  DeratingTarget target;
  bool run_active{false};
  bool evaluation_active{false};
  bool manual_abort{false};
  std::string abort_reason;
};

class ExperimentSupervisor {
 public:
  explicit ExperimentSupervisor(const SupervisorConfig& config);

  bool arm();
  bool start(double now_seconds);
  bool updateActualProgress(double actual_progress, bool valid,
                            double now_seconds);
  bool tick(double now_seconds);
  bool finish();
  bool abort(const std::string& reason);
  bool reset();

  const SupervisorSnapshot& snapshot() const noexcept { return snapshot_; }

 private:
  void activateDerating(double now_seconds);
  void beginRestoration(double now_seconds);
  void setPhase(ExperimentPhase phase, double now_seconds);
  bool runPhase() const noexcept;

  SupervisorConfig config_;
  SupervisorSnapshot snapshot_;
  double phase_start_time_{0.0};
  bool has_actual_progress_{false};
  double previous_actual_progress_{0.0};
};

}  // namespace multi_agv_control
