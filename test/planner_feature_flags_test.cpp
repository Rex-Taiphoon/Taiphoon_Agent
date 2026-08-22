/**
 * @file planner_feature_flags_test.cpp
 * @brief Planner feature flags 純 C++ 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 */

#include <gtest/gtest.h>

#include <limits>

#include "ego_swarm/PlannerFeatureFlags.hpp"

namespace
{

using taiphoon::ego_swarm::PlannerFeatureFlags;
using taiphoon::ego_swarm::normalize_planner_feature_flags;

TEST(PlannerFeatureFlagsTest, EnablesLocalGoalByDefaultAndKeepsReservationOff)
{
    const PlannerFeatureFlags flags{};

    EXPECT_TRUE(flags.enable_local_goal_planning);
    EXPECT_FALSE(flags.enable_generic_trajectory_reservation);
    EXPECT_DOUBLE_EQ(flags.local_goal_horizon_m, 10.0);
    EXPECT_DOUBLE_EQ(flags.local_goal_replan_ratio, 0.70);
    EXPECT_DOUBLE_EQ(flags.minimum_replan_interval_s, 0.5);
}

TEST(PlannerFeatureFlagsTest, PreservesValidOverrides)
{
    const PlannerFeatureFlags requested{
        true,
        true,
        20.0,
        0.8,
        1.25};

    const PlannerFeatureFlags normalized =
        normalize_planner_feature_flags(requested);

    EXPECT_TRUE(normalized.enable_local_goal_planning);
    EXPECT_TRUE(normalized.enable_generic_trajectory_reservation);
    EXPECT_DOUBLE_EQ(normalized.local_goal_horizon_m, 20.0);
    EXPECT_DOUBLE_EQ(normalized.local_goal_replan_ratio, 0.8);
    EXPECT_DOUBLE_EQ(normalized.minimum_replan_interval_s, 1.25);
}

TEST(PlannerFeatureFlagsTest, RestoresDefaultsForInvalidNumericOverrides)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    const PlannerFeatureFlags requested{
        true,
        false,
        -1.0,
        infinity,
        nan};

    const PlannerFeatureFlags normalized =
        normalize_planner_feature_flags(requested);

    EXPECT_TRUE(normalized.enable_local_goal_planning);
    EXPECT_FALSE(normalized.enable_generic_trajectory_reservation);
    EXPECT_DOUBLE_EQ(normalized.local_goal_horizon_m, 10.0);
    EXPECT_DOUBLE_EQ(normalized.local_goal_replan_ratio, 0.70);
    EXPECT_DOUBLE_EQ(normalized.minimum_replan_interval_s, 0.5);
}

TEST(PlannerFeatureFlagsTest, RejectsOutOfRangeReplanRatio)
{
    const PlannerFeatureFlags below_zero{
        false,
        false,
        10.0,
        0.0,
        0.5};
    const PlannerFeatureFlags above_one{
        false,
        false,
        10.0,
        1.1,
        0.5};

    EXPECT_DOUBLE_EQ(
        normalize_planner_feature_flags(below_zero).local_goal_replan_ratio,
        0.70);
    EXPECT_DOUBLE_EQ(
        normalize_planner_feature_flags(above_one).local_goal_replan_ratio,
        0.70);
}

}  // namespace
