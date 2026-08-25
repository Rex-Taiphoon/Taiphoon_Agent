/**
 * @file peer_deconfliction_planner_test.cpp
 * @brief 近距離 peer recovery planner 純 C++ 測試
 */

#include <gtest/gtest.h>

#include "ego_swarm/PeerDeconflictionPlanner.hpp"
#include "perception/RawPointObstacleMap.hpp"

namespace
{

using taiphoon::ego_swarm::PeerDeconflictionAction;
using taiphoon::ego_swarm::PeerDeconflictionConfig;
using taiphoon::ego_swarm::PeerDeconflictionInput;
using taiphoon::ego_swarm::PeerDeconflictionPeer;
using taiphoon::ego_swarm::PeerDeconflictionPlanner;
using taiphoon::ego_swarm::PlannerConfig;
using taiphoon::ego_swarm::Vector3;
using taiphoon::perception::RawPointObstacleMap;

TEST(PeerDeconflictionPlannerTest, LowestMovableAgentOwnsEscape)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(10.0));

    PeerDeconflictionInput input;
    input.self_agent_id = 1;
    input.self_position = Vector3{0.0, 0.0, 0.0};
    input.final_target = Vector3{10.0, 0.0, 0.0};
    input.now_s = 10.0;
    input.peers_are_fresh = true;
    input.obstacle_map = &map;
    input.planner_config = PlannerConfig{};
    input.peers.push_back(PeerDeconflictionPeer{
        2, Vector3{1.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}, false, true});

    PeerDeconflictionPlanner planner;
    const auto result = planner.decide(input);

    EXPECT_EQ(result.action, PeerDeconflictionAction::ESCAPE);
    EXPECT_TRUE(result.is_valid);
    EXPECT_EQ(result.owner_agent_id, 1);
    EXPECT_TRUE(result.plan.trajectory.is_valid());
    EXPECT_LT(result.escape_target.at(0), 0.0);
    const auto validation = planner.validate_remaining(
        result.plan.trajectory, 0.0, input, true);
    EXPECT_TRUE(validation.is_safe) << validation.reason;
    EXPECT_GE(validation.minimum_peer_distance_m, 1.0);
}

TEST(PeerDeconflictionPlannerTest, HigherAgentHoldsForLowerMovableAgent)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(10.0));

    PeerDeconflictionInput input;
    input.self_agent_id = 2;
    input.self_position = Vector3{1.0, 0.0, 0.0};
    input.final_target = Vector3{10.0, 0.0, 0.0};
    input.now_s = 10.0;
    input.peers_are_fresh = true;
    input.obstacle_map = &map;
    input.planner_config = PlannerConfig{};
    input.peers.push_back(PeerDeconflictionPeer{
        1, Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}, false, true});

    const auto result = PeerDeconflictionPlanner{}.decide(input);

    EXPECT_EQ(result.action, PeerDeconflictionAction::HOLD);
    EXPECT_TRUE(result.is_valid);
    EXPECT_EQ(result.owner_agent_id, 1);
}

TEST(PeerDeconflictionPlannerTest, TerminalHoldPeerDoesNotBecomeMover)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(10.0));

    PeerDeconflictionInput input;
    input.self_agent_id = 2;
    input.self_position = Vector3{1.0, 0.0, 0.0};
    input.final_target = Vector3{10.0, 0.0, 0.0};
    input.now_s = 10.0;
    input.peers_are_fresh = true;
    input.obstacle_map = &map;
    input.planner_config = PlannerConfig{};
    input.peers.push_back(PeerDeconflictionPeer{
        1, Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}, false, false});

    const auto result = PeerDeconflictionPlanner{}.decide(input);

    EXPECT_EQ(result.action, PeerDeconflictionAction::ESCAPE);
    EXPECT_EQ(result.owner_agent_id, 2);
}

TEST(PeerDeconflictionPlannerTest, MovingConflictingPeerMustBrakeFirst)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(10.0));

    PeerDeconflictionInput input;
    input.self_agent_id = 1;
    input.self_position = Vector3{0.0, 0.0, 0.0};
    input.final_target = Vector3{10.0, 0.0, 0.0};
    input.now_s = 10.0;
    input.peers_are_fresh = true;
    input.obstacle_map = &map;
    input.planner_config = PlannerConfig{};
    input.peers.push_back(PeerDeconflictionPeer{
        2, Vector3{1.0, 0.0, 0.0}, Vector3{1.0, 0.0, 0.0}, true, true});

    const auto result = PeerDeconflictionPlanner{}.decide(input);

    EXPECT_EQ(result.action, PeerDeconflictionAction::HOLD);
    EXPECT_EQ(result.reason, "waiting for conflicting peer to brake");
}

TEST(PeerDeconflictionPlannerTest, ObstacleAlongEscapeRayBlocksRecovery)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.update({
        Vector3{-2.0, 0.0, 0.0}, Vector3{2.0, 0.0, 0.0},
        Vector3{0.0, -2.0, 0.0}, Vector3{0.0, 2.0, 0.0}}, 10.0));

    PeerDeconflictionInput input;
    input.self_agent_id = 1;
    input.self_position = Vector3{0.0, 0.0, 0.0};
    input.final_target = Vector3{10.0, 0.0, 0.0};
    input.now_s = 10.0;
    input.peers_are_fresh = true;
    input.obstacle_map = &map;
    input.planner_config = PlannerConfig{};
    input.peers.push_back(PeerDeconflictionPeer{
        2, Vector3{1.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}, false, true});

    const auto result = PeerDeconflictionPlanner{}.decide(input);

    EXPECT_EQ(result.action, PeerDeconflictionAction::BLOCKED);
}

TEST(PeerDeconflictionPlannerTest, RecoveryRejectsDistanceWorsening)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(10.0));

    PeerDeconflictionInput input;
    input.self_agent_id = 1;
    input.self_position = Vector3{0.0, 0.0, 0.0};
    input.final_target = Vector3{10.0, 0.0, 0.0};
    input.now_s = 10.0;
    input.peers_are_fresh = true;
    input.obstacle_map = &map;
    input.planner_config = PlannerConfig{};
    input.peers.push_back(PeerDeconflictionPeer{
        2, Vector3{1.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}, false, true});
    const taiphoon::ego_swarm::BsplineTrajectory toward_peer(
        {Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0},
         Vector3{2.0, 0.0, 0.0}, Vector3{2.0, 0.0, 0.0}},
        4.0);

    const auto validation = PeerDeconflictionPlanner{}.validate_remaining(
        toward_peer, 0.0, input, true);

    EXPECT_FALSE(validation.is_safe);
    EXPECT_EQ(validation.reason, "recovery distance is worsening");
}

}  // namespace
