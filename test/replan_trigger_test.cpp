/**
 * @file replan_trigger_test.cpp
 * @brief Local Goal segment 重規劃 trigger 的純 C++ 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 */

#include <gtest/gtest.h>

#include "ego_swarm/ReplanTrigger.hpp"

namespace
{

using taiphoon::ego_swarm::ReplanTriggerInput;
using taiphoon::ego_swarm::ReplanTriggerReason;
using taiphoon::ego_swarm::evaluate_replan_trigger;

TEST(ReplanTriggerTest, ProgressAtThresholdAfterMinimumIntervalTriggers)
{
    const ReplanTriggerInput input{
        true,   // Local Goal enabled
        true,   // has active plan
        true,   // Final Target unchanged
        true,   // active trajectory safe
        7.0,    // elapsed time
        10.0,   // duration
        8.0,    // now
        true,   // has last replan time
        7.5,    // last replan time
        0.70,   // progress threshold
        0.5};   // minimum interval

    const auto decision = evaluate_replan_trigger(input);

    EXPECT_TRUE(decision.should_replan);
    EXPECT_EQ(decision.reason, ReplanTriggerReason::SEGMENT_PROGRESS);
}

TEST(ReplanTriggerTest, ProgressAtThresholdBeforeMinimumIntervalKeepsActive)
{
    const ReplanTriggerInput input{
        true, true, true, true, 7.0, 10.0, 7.4, true, 7.0, 0.70, 0.5};

    const auto decision = evaluate_replan_trigger(input);

    EXPECT_FALSE(decision.should_replan);
    EXPECT_EQ(decision.reason, ReplanTriggerReason::KEEP_ACTIVE);
}

TEST(ReplanTriggerTest, DisabledLocalGoalKeepsActiveAtProgressThreshold)
{
    const ReplanTriggerInput input{
        false, true, true, true, 9.0, 10.0, 20.0, true, 1.0, 0.70, 0.5};

    const auto decision = evaluate_replan_trigger(input);

    EXPECT_FALSE(decision.should_replan);
    EXPECT_EQ(decision.reason, ReplanTriggerReason::KEEP_ACTIVE);
}

TEST(ReplanTriggerTest, ChangedFinalTargetTriggersImmediately)
{
    const ReplanTriggerInput input{
        true, true, false, true, 2.0, 10.0, 2.0, true, 1.9, 0.70, 0.5};

    const auto decision = evaluate_replan_trigger(input);

    EXPECT_TRUE(decision.should_replan);
    EXPECT_EQ(decision.reason, ReplanTriggerReason::FINAL_TARGET_CHANGED);
}

TEST(ReplanTriggerTest, UnsafeActiveTrajectoryTriggersImmediately)
{
    const ReplanTriggerInput input{
        true, true, true, false, 1.0, 10.0, 1.0, true, 0.9, 0.70, 0.5};

    const auto decision = evaluate_replan_trigger(input);

    EXPECT_TRUE(decision.should_replan);
    EXPECT_EQ(
        decision.reason, ReplanTriggerReason::ACTIVE_TRAJECTORY_UNSAFE);
}

TEST(ReplanTriggerTest, SegmentEndTriggersImmediately)
{
    const ReplanTriggerInput input{
        true, true, true, true, 10.0, 10.0, 2.0, true, 1.9, 0.70, 5.0};

    const auto decision = evaluate_replan_trigger(input);

    EXPECT_TRUE(decision.should_replan);
    EXPECT_EQ(decision.reason, ReplanTriggerReason::SEGMENT_ENDED);
}

TEST(ReplanTriggerTest, FutureStartKeepsActivePlan)
{
    const ReplanTriggerInput input{
        true, true, true, true, -0.2, 10.0, 4.8, true, 4.0, 0.70, 0.5};

    const auto decision = evaluate_replan_trigger(input);

    EXPECT_FALSE(decision.should_replan);
    EXPECT_EQ(decision.reason, ReplanTriggerReason::KEEP_ACTIVE);
}

}  // namespace
