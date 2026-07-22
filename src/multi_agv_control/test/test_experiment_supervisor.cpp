#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "multi_agv_control/experiment_supervisor.hpp"

using multi_agv_control::ExperimentPhase;
using multi_agv_control::ExperimentSupervisor;
using multi_agv_control::SupervisorConfig;

TEST(ExperimentSupervisor, RejectsInvalidConfiguration) {
  SupervisorConfig config;
  config.restoration_progress = config.activation_progress;
  EXPECT_THROW({ const ExperimentSupervisor supervisor(config); },
               std::invalid_argument);
}

TEST(ExperimentSupervisor, TriggerUsesOnlyActualProgressAtCrossing) {
  ExperimentSupervisor supervisor(SupervisorConfig{});
  ASSERT_TRUE(supervisor.arm());
  ASSERT_TRUE(supervisor.start(1.0));
  EXPECT_FALSE(supervisor.updateActualProgress(100.0, false, 1.1));
  EXPECT_EQ(supervisor.snapshot().phase, ExperimentPhase::kRunningNominal);
  EXPECT_FALSE(supervisor.updateActualProgress(0.299, true, 1.2));
  const auto before = supervisor.snapshot().target.sequence;
  EXPECT_TRUE(supervisor.updateActualProgress(0.3, true, 1.3));
  EXPECT_EQ(supervisor.snapshot().phase, ExperimentPhase::kDeratingDown);
  EXPECT_TRUE(supervisor.snapshot().target.active);
  EXPECT_EQ(supervisor.snapshot().target.sequence, before + 1U);
}

TEST(ExperimentSupervisor, RepeatsTargetWithoutIncrementingSequence) {
  ExperimentSupervisor supervisor(SupervisorConfig{});
  ASSERT_TRUE(supervisor.arm());
  ASSERT_TRUE(supervisor.start(0.0));
  ASSERT_TRUE(supervisor.updateActualProgress(0.3, true, 0.1));
  const auto sequence = supervisor.snapshot().target.sequence;
  EXPECT_FALSE(supervisor.updateActualProgress(0.4, true, 0.2));
  EXPECT_FALSE(supervisor.tick(0.3));
  EXPECT_EQ(supervisor.snapshot().target.sequence, sequence);
  EXPECT_TRUE(supervisor.tick(0.7));
  EXPECT_EQ(supervisor.snapshot().phase, ExperimentPhase::kDerated);
  EXPECT_EQ(supervisor.snapshot().target.sequence, sequence);
}

TEST(ExperimentSupervisor, RestoresOnSecondActualCrossing) {
  ExperimentSupervisor supervisor(SupervisorConfig{});
  ASSERT_TRUE(supervisor.arm());
  ASSERT_TRUE(supervisor.start(0.0));
  ASSERT_TRUE(supervisor.updateActualProgress(0.3, true, 0.1));
  const auto derating_sequence = supervisor.snapshot().target.sequence;
  EXPECT_TRUE(supervisor.updateActualProgress(0.6, true, 0.2));
  EXPECT_EQ(supervisor.snapshot().phase, ExperimentPhase::kRestoring);
  EXPECT_FALSE(supervisor.snapshot().target.active);
  EXPECT_EQ(supervisor.snapshot().target.sequence, derating_sequence + 1U);
  EXPECT_TRUE(supervisor.tick(0.8));
  EXPECT_EQ(supervisor.snapshot().phase, ExperimentPhase::kFinished);
}

TEST(ExperimentSupervisor, ResetAllowedOnlyWhenInactive) {
  ExperimentSupervisor supervisor(SupervisorConfig{});
  ASSERT_TRUE(supervisor.arm());
  EXPECT_FALSE(supervisor.reset());
  ASSERT_TRUE(supervisor.start(0.0));
  EXPECT_FALSE(supervisor.reset());
  ASSERT_TRUE(supervisor.finish());
  EXPECT_TRUE(supervisor.reset());
  EXPECT_EQ(supervisor.snapshot().phase, ExperimentPhase::kIdle);
}

TEST(ExperimentSupervisor, NonFiniteAndBackwardReferenceCannotTrigger) {
  ExperimentSupervisor supervisor(SupervisorConfig{});
  ASSERT_TRUE(supervisor.arm());
  ASSERT_TRUE(supervisor.start(0.0));
  EXPECT_FALSE(supervisor.updateActualProgress(
      std::numeric_limits<double>::quiet_NaN(), true, 0.1));
  EXPECT_EQ(supervisor.snapshot().phase, ExperimentPhase::kRunningNominal);
}
