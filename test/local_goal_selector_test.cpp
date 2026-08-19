/**
 * @file local_goal_selector_test.cpp
 * @brief Local Goal Selector 純 C++ 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 */

#include <gtest/gtest.h>

#include <limits>

#include "ego_swarm/LocalGoalSelector.hpp"

namespace
{

using taiphoon::ego_swarm::LocalGoalConfig;
using taiphoon::ego_swarm::compute_local_goal;
using taiphoon::ego_swarm::PathConstraint;
using taiphoon::ego_swarm::Vector3;

TEST(LocalGoalSelectorTest, UsesFinalTargetWithinHorizon)
{
    const LocalGoalConfig config{10.0};
    const auto result = compute_local_goal(
        Vector3{0.0, 0.0, 0.0},
        Vector3{5.0, 0.0, 0.0},
        PathConstraint::FREE_3D,
        config);

    ASSERT_TRUE(result.is_valid);
    EXPECT_TRUE(result.is_final_goal);
    EXPECT_NEAR(result.goal.at(0), 5.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(1), 0.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(2), 0.0, 1.0e-9);
}

TEST(LocalGoalSelectorTest, StopsAtHorizonBeforeFinalTarget)
{
    const LocalGoalConfig config{10.0};
    const auto result = compute_local_goal(
        Vector3{0.0, 0.0, 0.0},
        Vector3{30.0, 0.0, 0.0},
        PathConstraint::FREE_3D,
        config);

    ASSERT_TRUE(result.is_valid);
    EXPECT_FALSE(result.is_final_goal);
    EXPECT_NEAR(result.goal.at(0), 10.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(1), 0.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(2), 0.0, 1.0e-9);
}

TEST(LocalGoalSelectorTest, PreservesArbitraryTargetDirection)
{
    const LocalGoalConfig config{10.0};
    const auto result = compute_local_goal(
        Vector3{0.0, 0.0, 0.0},
        Vector3{30.0, 40.0, 0.0},
        PathConstraint::FREE_3D,
        config);

    ASSERT_TRUE(result.is_valid);
    EXPECT_FALSE(result.is_final_goal);
    EXPECT_NEAR(result.goal.at(0), 6.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(1), 8.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(2), 0.0, 1.0e-9);
}

TEST(LocalGoalSelectorTest, FixedZProjectsPlanningCurrentBeforeHorizon)
{
    const LocalGoalConfig config{10.0};
    const auto result = compute_local_goal(
        Vector3{0.0, 0.0, 5.0},
        Vector3{30.0, 0.0, 0.0},
        PathConstraint::FIXED_Z,
        config);

    ASSERT_TRUE(result.is_valid);
    EXPECT_FALSE(result.is_final_goal);
    EXPECT_NEAR(result.goal.at(0), 10.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(1), 0.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(2), 0.0, 1.0e-9);
}

TEST(LocalGoalSelectorTest, LinearZInterpolatesAlongFinalTargetPath)
{
    const LocalGoalConfig config{10.0};
    const auto result = compute_local_goal(
        Vector3{0.0, 0.0, 0.0},
        Vector3{30.0, 0.0, 10.0},
        PathConstraint::LINEAR_Z,
        config);

    ASSERT_TRUE(result.is_valid);
    EXPECT_FALSE(result.is_final_goal);
    EXPECT_NEAR(result.goal.at(0), 9.4868329805, 1.0e-9);
    EXPECT_NEAR(result.goal.at(1), 0.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(2), 3.1622776602, 1.0e-9);
}

TEST(LocalGoalSelectorTest, RejectsNonFiniteInput)
{
    const LocalGoalConfig config{10.0};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();

    const auto nan_result = compute_local_goal(
        Vector3{nan, 0.0, 0.0},
        Vector3{5.0, 0.0, 0.0},
        PathConstraint::FREE_3D,
        config);
    const auto infinity_result = compute_local_goal(
        Vector3{0.0, 0.0, 0.0},
        Vector3{infinity, 0.0, 0.0},
        PathConstraint::FREE_3D,
        config);

    EXPECT_FALSE(nan_result.is_valid);
    EXPECT_FALSE(infinity_result.is_valid);
}

TEST(LocalGoalSelectorTest, RejectsNonPositiveHorizon)
{
    const auto result = compute_local_goal(
        Vector3{0.0, 0.0, 0.0},
        Vector3{5.0, 0.0, 0.0},
        PathConstraint::FREE_3D,
        LocalGoalConfig{0.0});

    EXPECT_FALSE(result.is_valid);
}

}  // namespace
