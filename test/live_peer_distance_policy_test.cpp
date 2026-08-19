/**
 * @file live_peer_distance_policy_test.cpp
 * @brief 即時同儕距離碰撞判定測試
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 */

#include <gtest/gtest.h>

#include "ego_swarm/LivePeerDistancePolicy.hpp"

namespace
{

using taiphoon::ego_swarm::LivePeerDistanceResult;
using taiphoon::ego_swarm::live_peer_collision_detected;

TEST(LivePeerDistancePolicyTest, EmptyPeerSetDoesNotTriggerCollision)
{
    const LivePeerDistanceResult result{};

    EXPECT_FALSE(result.has_peer);
    EXPECT_TRUE(result.data_valid);
    EXPECT_FALSE(live_peer_collision_detected(result, 3.0));
}

TEST(LivePeerDistancePolicyTest, CloseValidPeerTriggersCollision)
{
    const LivePeerDistanceResult result{true, true, 2.5};

    EXPECT_TRUE(live_peer_collision_detected(result, 3.0));
}

TEST(LivePeerDistancePolicyTest, InvalidPeerDataFailsClosed)
{
    const LivePeerDistanceResult result{true, false, 0.0};

    EXPECT_TRUE(live_peer_collision_detected(result, 3.0));
}

}  // namespace
