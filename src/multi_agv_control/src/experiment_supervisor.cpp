#include "multi_agv_control/experiment_supervisor.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace multi_agv_control {
namespace {

bool validRatio(double value) {
  return std::isfinite(value) && value > 0.0 && value <= 1.0;
}

}  // namespace

ExperimentSupervisor::ExperimentSupervisor(const SupervisorConfig& config)
    : config_(config) {
  if (!std::isfinite(config_.activation_progress) ||
      !std::isfinite(config_.restoration_progress) ||
      config_.activation_progress < 0.0 ||
      config_.restoration_progress <= config_.activation_progress ||
      !std::isfinite(config_.evaluation_start_progress) ||
      !std::isfinite(config_.evaluation_end_progress) ||
      config_.evaluation_start_progress < 0.0 ||
      config_.evaluation_end_progress <=
          config_.evaluation_start_progress ||
      !validRatio(config_.target_speed_ratio_left) ||
      !validRatio(config_.target_speed_ratio_right) ||
      !validRatio(config_.target_acceleration_ratio_left) ||
      !validRatio(config_.target_acceleration_ratio_right) ||
      !validRatio(config_.target_deceleration_ratio_left) ||
      !validRatio(config_.target_deceleration_ratio_right) ||
      !std::isfinite(config_.ramp_down_time) || config_.ramp_down_time <= 0.0 ||
      !std::isfinite(config_.ramp_up_time) || config_.ramp_up_time <= 0.0) {
    throw std::invalid_argument("invalid experiment supervisor configuration");
  }
}

bool ExperimentSupervisor::arm() {
  if (snapshot_.phase != ExperimentPhase::kIdle) {
    return false;
  }
  snapshot_.phase = ExperimentPhase::kArmed;
  snapshot_.run_active = true;
  snapshot_.evaluation_active = false;
  snapshot_.manual_abort = false;
  snapshot_.abort_reason.clear();
  has_actual_progress_ = false;
  evaluation_started_ = false;
  return true;
}

bool ExperimentSupervisor::start(double now_seconds) {
  if (snapshot_.phase != ExperimentPhase::kArmed ||
      !std::isfinite(now_seconds)) {
    return false;
  }
  setPhase(ExperimentPhase::kRunningNominal, now_seconds);
  snapshot_.evaluation_active = false;
  return true;
}

void ExperimentSupervisor::setPhase(ExperimentPhase phase,
                                    double now_seconds) {
  snapshot_.phase = phase;
  phase_start_time_ = now_seconds;
}

void ExperimentSupervisor::activateDerating(double now_seconds) {
  ++snapshot_.target.sequence;
  snapshot_.target.mode = config_.derating_mode;
  snapshot_.target.active = true;
  snapshot_.target.speed_ratio_left = config_.target_speed_ratio_left;
  snapshot_.target.speed_ratio_right = config_.target_speed_ratio_right;
  snapshot_.target.acceleration_ratio_left =
      config_.target_acceleration_ratio_left;
  snapshot_.target.acceleration_ratio_right =
      config_.target_acceleration_ratio_right;
  snapshot_.target.deceleration_ratio_left =
      config_.target_deceleration_ratio_left;
  snapshot_.target.deceleration_ratio_right =
      config_.target_deceleration_ratio_right;
  snapshot_.target.ramp_down_time = config_.ramp_down_time;
  snapshot_.target.ramp_up_time = config_.ramp_up_time;
  setPhase(ExperimentPhase::kDeratingDown, now_seconds);
}

void ExperimentSupervisor::beginRestoration(double now_seconds) {
  ++snapshot_.target.sequence;
  snapshot_.target.mode = config_.derating_mode;
  snapshot_.target.active = false;
  snapshot_.target.speed_ratio_left = 1.0;
  snapshot_.target.speed_ratio_right = 1.0;
  snapshot_.target.acceleration_ratio_left = 1.0;
  snapshot_.target.acceleration_ratio_right = 1.0;
  snapshot_.target.deceleration_ratio_left = 1.0;
  snapshot_.target.deceleration_ratio_right = 1.0;
  snapshot_.target.ramp_down_time = config_.ramp_down_time;
  snapshot_.target.ramp_up_time = config_.ramp_up_time;
  setPhase(ExperimentPhase::kRestoring, now_seconds);
}

bool ExperimentSupervisor::updateActualProgress(double actual_progress,
                                                bool valid,
                                                double now_seconds) {
  if (!valid || !std::isfinite(actual_progress) ||
      !std::isfinite(now_seconds) || !runPhase()) {
    return false;
  }
  previous_actual_progress_ = actual_progress;
  has_actual_progress_ = true;
  if (!evaluation_started_ &&
      actual_progress >= config_.evaluation_start_progress &&
      actual_progress < config_.evaluation_end_progress) {
    evaluation_started_ = true;
    snapshot_.evaluation_active = true;
  }
  if (evaluation_started_ &&
      actual_progress >= config_.evaluation_end_progress) {
    snapshot_.evaluation_active = false;
    if (!config_.finish_after_restoration &&
        snapshot_.phase == ExperimentPhase::kPostRestoration) {
      setPhase(ExperimentPhase::kFinished, now_seconds);
      snapshot_.run_active = false;
      return true;
    }
  }
  if (snapshot_.phase == ExperimentPhase::kRunningNominal &&
      actual_progress >= config_.activation_progress) {
    activateDerating(now_seconds);
    return true;
  }
  if ((snapshot_.phase == ExperimentPhase::kDeratingDown ||
       snapshot_.phase == ExperimentPhase::kDerated) &&
      actual_progress >= config_.restoration_progress) {
    beginRestoration(now_seconds);
    return true;
  }
  return false;
}

bool ExperimentSupervisor::tick(double now_seconds) {
  if (!std::isfinite(now_seconds)) {
    return false;
  }
  if (snapshot_.phase == ExperimentPhase::kDeratingDown &&
      now_seconds - phase_start_time_ >= config_.ramp_down_time) {
    setPhase(ExperimentPhase::kDerated, now_seconds);
    return true;
  }
  if (snapshot_.phase == ExperimentPhase::kRestoring &&
      now_seconds - phase_start_time_ >= config_.ramp_up_time) {
    if (config_.finish_after_restoration) {
      setPhase(ExperimentPhase::kFinished, now_seconds);
      snapshot_.run_active = false;
      snapshot_.evaluation_active = false;
    } else {
      setPhase(ExperimentPhase::kPostRestoration, now_seconds);
    }
    return true;
  }
  return false;
}

bool ExperimentSupervisor::finish() {
  if (!runPhase() || snapshot_.target.active) {
    return false;
  }
  snapshot_.phase = ExperimentPhase::kFinished;
  snapshot_.run_active = false;
  snapshot_.evaluation_active = false;
  return true;
}

bool ExperimentSupervisor::abort(const std::string& reason) {
  if (!runPhase() && snapshot_.phase != ExperimentPhase::kArmed) {
    return false;
  }
  if (snapshot_.target.active) {
    beginRestoration(phase_start_time_);
  }
  snapshot_.phase = ExperimentPhase::kAborted;
  snapshot_.run_active = false;
  snapshot_.evaluation_active = false;
  snapshot_.manual_abort = true;
  snapshot_.abort_reason = reason;
  return true;
}

bool ExperimentSupervisor::reset() {
  if (snapshot_.run_active || snapshot_.target.active ||
      (snapshot_.phase != ExperimentPhase::kIdle &&
       snapshot_.phase != ExperimentPhase::kFinished &&
       snapshot_.phase != ExperimentPhase::kAborted)) {
    return false;
  }
  snapshot_ = SupervisorSnapshot{};
  phase_start_time_ = 0.0;
  has_actual_progress_ = false;
  evaluation_started_ = false;
  previous_actual_progress_ = 0.0;
  return true;
}

bool ExperimentSupervisor::runPhase() const noexcept {
  return snapshot_.phase == ExperimentPhase::kRunningNominal ||
         snapshot_.phase == ExperimentPhase::kDeratingDown ||
         snapshot_.phase == ExperimentPhase::kDerated ||
         snapshot_.phase == ExperimentPhase::kRestoring ||
         snapshot_.phase == ExperimentPhase::kPostRestoration;
}

}  // namespace multi_agv_control
