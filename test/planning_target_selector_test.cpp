/**
 * @file planning_target_selector_test.cpp
 * @brief Planning Target 選擇的純 C++ 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 */

#include <gtest/gtest.h>

#include <limits>

#include "ego_swarm/PlanningTargetSelector.hpp"

namespace
{

using taiphoon::ego_swarm::LocalGoalConfig;
using taiphoon::ego_swarm::PathConstraint;
using taiphoon::ego_swarm::select_planning_target;
using taiphoon::ego_swarm::Vector3;

TEST(PlanningTargetSelectorTest, DisabledFeaturePreservesFinalTarget)
{
    const auto result = select_planning_target(
        false,
        Vector3{0.0, 0.0, 0.0},
        Vector3{30.0, 0.0, 0.0},
        PathConstraint::FREE_3D,
        LocalGoalConfig{10.0});

    ASSERT_TRUE(result.is_valid);
    EXPECT_TRUE(result.is_final_goal);
    EXPECT_NEAR(result.goal.at(0), 30.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(1), 0.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(2), 0.0, 1.0e-9);
}

TEST(PlanningTargetSelectorTest, EnabledFeatureSelectsLocalGoal)
{
    const auto result = select_planning_target(
        true,
        Vector3{0.0, 0.0, 0.0},
        Vector3{30.0, 0.0, 0.0},
        PathConstraint::FREE_3D,
        LocalGoalConfig{10.0});

    ASSERT_TRUE(result.is_valid);
    EXPECT_FALSE(result.is_final_goal);
    EXPECT_NEAR(result.goal.at(0), 10.0, 1.0e-9);
}

TEST(PlanningTargetSelectorTest, EnabledFeatureKeepsPathConstraint)
{
    const auto result = select_planning_target(
        true,
        Vector3{0.0, 0.0, 5.0},
        Vector3{30.0, 0.0, 0.0},
        PathConstraint::FIXED_Z,
        LocalGoalConfig{10.0});

    ASSERT_TRUE(result.is_valid);
    EXPECT_NEAR(result.goal.at(0), 10.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(2), 0.0, 1.0e-9);
}

TEST(PlanningTargetSelectorTest, InvalidFinalTargetFailsClosed)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto result = select_planning_target(
        true,
        Vector3{0.0, 0.0, 0.0},
        Vector3{nan, 0.0, 0.0},
        PathConstraint::FREE_3D,
        LocalGoalConfig{10.0});

    EXPECT_FALSE(result.is_valid);
}

}  // namespace
