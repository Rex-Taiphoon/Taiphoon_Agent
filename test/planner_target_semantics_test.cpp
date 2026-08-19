/**
 * @file planner_target_semantics_test.cpp
 * @brief Planner target 語意判斷的純 C++ 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 */

#include <gtest/gtest.h>

#include "ego_swarm/PlannerTargetSemantics.hpp"

namespace
{

using taiphoon::ego_swarm::Vector3;
using taiphoon::ego_swarm::active_plan_matches_final_target;

TEST(PlannerTargetSemanticsTest, ActivePlanMatchesItsFinalTarget)
{
    EXPECT_TRUE(active_plan_matches_final_target(
        true,
        Vector3{30.0, 0.0, 0.0},
        Vector3{30.0, 0.0, 0.0}));
}

TEST(PlannerTargetSemanticsTest, PlanningSegmentDoesNotDefineTaskCompletion)
{
    // 目前 active segment 只到 10 m，但它所屬任務的 Final Target 是 30 m。
    // Final Target 相同時，segment endpoint 不應被拿來判定任務已改變。
    const Vector3 active_plan_target{10.0, 0.0, 0.0};
    const Vector3 active_plan_final_target{30.0, 0.0, 0.0};
    const Vector3 current_final_target{30.0, 0.0, 0.0};

    (void)active_plan_target;
    EXPECT_TRUE(active_plan_matches_final_target(
        true, active_plan_final_target, current_final_target));
}

TEST(PlannerTargetSemanticsTest, ChangedFinalTargetInvalidatesActivePlan)
{
    EXPECT_FALSE(active_plan_matches_final_target(
        true,
        Vector3{30.0, 0.0, 0.0},
        Vector3{40.0, 0.0, 0.0}));
}

TEST(PlannerTargetSemanticsTest, MissingActivePlanNeverMatches)
{
    EXPECT_FALSE(active_plan_matches_final_target(
        false,
        Vector3{30.0, 0.0, 0.0},
        Vector3{30.0, 0.0, 0.0}));
}

}  // namespace
