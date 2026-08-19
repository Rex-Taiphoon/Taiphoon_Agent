/**
 * @file trajectory_reservation_policy_test.cpp
 * @brief Generic trajectory reservation policy 純 C++ 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 */

#include <gtest/gtest.h>

#include "ego_swarm/PlannerFeatureFlags.hpp"
#include "ego_swarm/TrajectoryReservationPolicy.hpp"

namespace
{

using taiphoon::ego_swarm::PlannerFeatureFlags;
using taiphoon::ego_swarm::required_peer_separation_m;
using taiphoon::ego_swarm::should_reserve_moving_trajectory;

TEST(TrajectoryReservationPolicyTest, KeepsFormationReservationEnabled)
{
    const PlannerFeatureFlags flags{};

    EXPECT_TRUE(should_reserve_moving_trajectory(true, flags));
    EXPECT_FALSE(should_reserve_moving_trajectory(false, flags));
}

TEST(TrajectoryReservationPolicyTest, EnablesReservationForMoveWhenFlagIsOn)
{
    PlannerFeatureFlags flags{};
    flags.enable_generic_trajectory_reservation = true;

    EXPECT_TRUE(should_reserve_moving_trajectory(false, flags));
}

TEST(TrajectoryReservationPolicyTest, UsesPeerDistanceForMove)
{
    EXPECT_DOUBLE_EQ(
        required_peer_separation_m(false, 5.0, 3.0),
        3.0);
    EXPECT_DOUBLE_EQ(
        required_peer_separation_m(true, 5.0, 3.0),
        5.0);
}

}  // namespace
