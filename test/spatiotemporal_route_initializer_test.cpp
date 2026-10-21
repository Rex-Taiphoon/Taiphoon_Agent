/**
 * @file spatiotemporal_route_initializer_test.cpp
 * @brief FIXED_Z 時空 route initializer 的純 C++ 測試
 */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "ego_swarm/SpatiotemporalRouteInitializer.hpp"
#include "perception/RawPointObstacleMap.hpp"

namespace
{

using taiphoon::ego_swarm::BsplineTrajectory;
using taiphoon::ego_swarm::PathConstraint;
using taiphoon::ego_swarm::PeerTrajectory;
using taiphoon::ego_swarm::PlannerConfig;
using taiphoon::ego_swarm::PlanningSnapshot;
using taiphoon::ego_swarm::SpatiotemporalRouteInitializer;
using taiphoon::ego_swarm::SpatiotemporalRouteResult;
using taiphoon::ego_swarm::Vector3;
using taiphoon::perception::RawPointObstacleMap;

/// 正確性測試使用的寬鬆牆鐘預算（毫秒）：A* 由 ROUTE_MAX_EXPANDED_NODES 等
/// 決定性條件終止，結果不隨 CPU 負載變動；實際耗時改由 SITL 日誌觀察。
constexpr double DETERMINISTIC_TIME_BUDGET_MS = 5000.0;

RawPointObstacleMap make_map(const std::vector<Vector3>& points)
{
    RawPointObstacleMap map;
    EXPECT_TRUE(map.update(points, 0.0));
    return map;
}

PlanningSnapshot make_snapshot(RawPointObstacleMap& map)
{
    PlanningSnapshot snapshot;
    snapshot.self_state.position = Vector3{0.0, 0.0, 0.0};
    snapshot.self_state.timestamp_s = 0.0;
    snapshot.local_goal = Vector3{10.0, 0.0, 0.0};
    snapshot.planning_time_s = 0.0;
    snapshot.trajectory_start_time_s = 0.5;
    snapshot.frame_revision = 1U;
    snapshot.self_agent_id = 1;
    snapshot.path_constraint = PathConstraint::FIXED_Z;
    snapshot.fixed_z_m = 0.0;
    snapshot.obstacle_map = &map;
    snapshot.peers_are_fresh = true;
    return snapshot;
}

std::vector<Vector3> make_straight_seed()
{
    return {
        Vector3{0.0, 0.0, 0.0},
        Vector3{2.0, 0.0, 0.0},
        Vector3{4.0, 0.0, 0.0},
        Vector3{6.0, 0.0, 0.0},
        Vector3{8.0, 0.0, 0.0},
        Vector3{10.0, 0.0, 0.0}};
}

PeerTrajectory make_terminal_hold(
    const int agent_id,
    const Vector3& position)
{
    PeerTrajectory peer;
    peer.agent_id = agent_id;
    peer.trajectory_id = 1U;
    peer.frame_revision = 1U;
    peer.message_time_s = 0.0;
    peer.start_time_s = 0.5;
    peer.receive_time_s = 0.0;
    peer.end_time_s = 1.5;
    peer.terminal_hold = true;
    peer.trajectory = BsplineTrajectory(
        {position, position, position, position}, 1.0);
    return peer;
}

PeerTrajectory make_crossing_peer(const int agent_id)
{
    PeerTrajectory peer;
    peer.agent_id = agent_id;
    peer.trajectory_id = 1U;
    peer.frame_revision = 1U;
    peer.message_time_s = 0.0;
    peer.start_time_s = 0.5;
    peer.receive_time_s = 0.0;
    peer.end_time_s = 10.5;
    peer.terminal_hold = false;
    peer.trajectory = BsplineTrajectory(
        {Vector3{5.0, -5.0, 0.0}, Vector3{5.0, -1.7, 0.0},
         Vector3{5.0, 1.7, 0.0}, Vector3{5.0, 5.0, 0.0}}, 10.0);
    return peer;
}

double distance_to_peer(
    const Vector3& position,
    const PeerTrajectory& peer)
{
    return taiphoon::ego_swarm::distance_between_centers(
        position, peer.trajectory.evaluate_state(0.0).position);
}

TEST(SpatiotemporalRouteInitializerTest, ClearLineUsesFastPath)
{
    RawPointObstacleMap map = make_map({});
    PlanningSnapshot snapshot = make_snapshot(map);
    PlannerConfig config;
    SpatiotemporalRouteInitializer initializer;

    const SpatiotemporalRouteResult result = initializer.initialize(
        snapshot, config, make_straight_seed());

    EXPECT_TRUE(result.is_valid);
    EXPECT_FALSE(result.used_search);
    EXPECT_EQ(result.route.size(), 0U);
    EXPECT_NEAR(result.trajectory_start_time_s, 0.5, 1.0e-9);
}

TEST(SpatiotemporalRouteInitializerTest, FixedWallProducesLateralRoute)
{
    std::vector<Vector3> wall;
    for (int index = -8; index <= 8; ++index)
    {
        wall.push_back(Vector3{5.0, static_cast<double>(index) * 0.5, 0.0});
    }
    RawPointObstacleMap map = make_map(wall);
    PlanningSnapshot snapshot = make_snapshot(map);
    PlannerConfig config;
    config.optimizer_time_budget_ms = DETERMINISTIC_TIME_BUDGET_MS;
    SpatiotemporalRouteInitializer initializer;

    const SpatiotemporalRouteResult result = initializer.initialize(
        snapshot, config, make_straight_seed());

    ASSERT_TRUE(result.is_valid)
        << "termination=" << static_cast<int>(result.termination)
        << " expanded=" << result.expanded_nodes
        << " search_ms=" << result.search_time_ms;
    ASSERT_TRUE(result.used_search);
    ASSERT_GE(result.route.size(), 2U);
    bool has_lateral_offset = false;
    for (const auto& point : result.route)
    {
        EXPECT_NEAR(point.position.at(2), snapshot.fixed_z_m, 1.0e-9);
        EXPECT_TRUE(map.is_position_safe(
            point.position, config.obstacle_clearance_m));
        has_lateral_offset |= std::abs(point.position.at(1)) > 1.0;
    }
    EXPECT_TRUE(has_lateral_offset);
}

TEST(SpatiotemporalRouteInitializerTest, PointCloudWallAtTakeoffHeightRoutes)
{
    std::vector<Vector3> wall;
    for (const double x : {4.75, 5.25})
    {
        for (int index = -2; index <= 2; ++index)
        {
            for (const double z : {-0.25, 0.25})
            {
                wall.push_back(Vector3{
                    x, static_cast<double>(index), z});
            }
        }
    }
    RawPointObstacleMap map = make_map(wall);
    PlanningSnapshot snapshot = make_snapshot(map);
    PlannerConfig config;
    config.optimizer_time_budget_ms = DETERMINISTIC_TIME_BUDGET_MS;
    SpatiotemporalRouteInitializer initializer;

    const SpatiotemporalRouteResult result = initializer.initialize(
        snapshot, config, make_straight_seed());

    ASSERT_TRUE(result.is_valid)
        << "termination=" << static_cast<int>(result.termination)
        << " expanded=" << result.expanded_nodes
        << " search_ms=" << result.search_time_ms;
    ASSERT_TRUE(result.used_search);
    bool has_lateral_offset = false;
    for (const auto& point : result.route)
    {
        EXPECT_TRUE(map.is_position_safe(
            point.position, config.obstacle_clearance_m));
        has_lateral_offset |= std::abs(point.position.at(1)) > 1.0;
    }
    EXPECT_TRUE(has_lateral_offset);
}

TEST(SpatiotemporalRouteInitializerTest, OffsetTakeoffPositionRoutesPastWall)
{
    std::vector<Vector3> wall;
    for (int index = -2; index <= 2; ++index)
    {
        for (const double z : {-8.0, -7.75})
        {
            wall.push_back(Vector3{
                8.0, static_cast<double>(index), z});
        }
    }
    RawPointObstacleMap map = make_map(wall);
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_state.position = Vector3{1.75, 2.35, -8.0};
    snapshot.local_goal = Vector3{11.75, 2.35, -8.0};
    snapshot.fixed_z_m = -8.0;
    PlannerConfig config;
    config.optimizer_time_budget_ms = DETERMINISTIC_TIME_BUDGET_MS;
    SpatiotemporalRouteInitializer initializer;

    const SpatiotemporalRouteResult result = initializer.initialize(
        snapshot, config, make_straight_seed());

    ASSERT_TRUE(result.is_valid)
        << "termination=" << static_cast<int>(result.termination)
        << " expanded=" << result.expanded_nodes
        << " search_ms=" << result.search_time_ms;
    ASSERT_TRUE(result.used_search);
}

TEST(SpatiotemporalRouteInitializerTest, StaggeredTerminalHoldsAllowMiddleRoute)
{
    RawPointObstacleMap map = make_map({});
    PlanningSnapshot snapshot = make_snapshot(map);
    const PeerTrajectory lower = make_terminal_hold(
        2, Vector3{3.0, -2.9, 0.0});
    const PeerTrajectory upper = make_terminal_hold(
        3, Vector3{7.0, 2.9, 0.0});
    snapshot.peers = {lower, upper};
    PlannerConfig config;
    // 此回歸場景以 3 m 門檻阻塞直線，不依賴部署預設距離。
    config.peer_collision_min_distance_m = 3.0;
    config.optimizer_time_budget_ms = DETERMINISTIC_TIME_BUDGET_MS;
    SpatiotemporalRouteInitializer initializer;

    const SpatiotemporalRouteResult result = initializer.initialize(
        snapshot, config, make_straight_seed());

    ASSERT_TRUE(result.is_valid);
    ASSERT_TRUE(result.used_search);
    bool has_positive_side = false;
    bool has_negative_side = false;
    for (const auto& point : result.route)
    {
        EXPECT_GE(distance_to_peer(point.position, lower),
                  config.peer_collision_min_distance_m - 1.0e-6);
        EXPECT_GE(distance_to_peer(point.position, upper),
                  config.peer_collision_min_distance_m - 1.0e-6);
        has_positive_side |= point.position.at(1) > 0.1;
        has_negative_side |= point.position.at(1) < -0.1;
    }
    EXPECT_TRUE(has_positive_side);
    EXPECT_TRUE(has_negative_side);
}

TEST(SpatiotemporalRouteInitializerTest, ClosedWallReportsNoFeasiblePath)
{
    std::vector<Vector3> wall;
    for (int index = -24; index <= 24; ++index)
    {
        wall.push_back(Vector3{5.0, static_cast<double>(index) * 0.5, 0.0});
    }
    RawPointObstacleMap map = make_map(wall);
    PlanningSnapshot snapshot = make_snapshot(map);
    PlannerConfig config;
    config.optimizer_time_budget_ms = 1000.0;
    config.optimizer_search_radius_m = 10.0;
    SpatiotemporalRouteInitializer initializer;

    const SpatiotemporalRouteResult result = initializer.initialize(
        snapshot, config, make_straight_seed());

    EXPECT_FALSE(result.is_valid);
    EXPECT_TRUE(
        result.termination ==
            taiphoon::ego_swarm::RouteInitializerTerminationReason::NO_FEASIBLE_PATH ||
        result.termination ==
            taiphoon::ego_swarm::RouteInitializerTerminationReason::TIMEOUT);
}

TEST(SpatiotemporalRouteInitializerTest, LowerPriorityMovingPeerTriggersRouteSearch)
{
    RawPointObstacleMap map = make_map({});
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 2;
    snapshot.peers.push_back(make_crossing_peer(1));
    PlannerConfig config;
    config.optimizer_time_budget_ms = DETERMINISTIC_TIME_BUDGET_MS;
    SpatiotemporalRouteInitializer initializer;

    const SpatiotemporalRouteResult result = initializer.initialize(
        snapshot, config, make_straight_seed());

    ASSERT_TRUE(result.is_valid)
        << "termination=" << static_cast<int>(result.termination)
        << " expanded=" << result.expanded_nodes
        << " search_ms=" << result.search_time_ms;
    EXPECT_TRUE(result.used_search);
}

TEST(SpatiotemporalRouteInitializerTest, HigherPriorityMovingPeerDoesNotBlockSeed)
{
    RawPointObstacleMap map = make_map({});
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 1;
    snapshot.peers.push_back(make_crossing_peer(2));
    PlannerConfig config;
    SpatiotemporalRouteInitializer initializer;

    const SpatiotemporalRouteResult result = initializer.initialize(
        snapshot, config, make_straight_seed());

    EXPECT_TRUE(result.is_valid);
    EXPECT_FALSE(result.used_search);
}

TEST(SpatiotemporalRouteInitializerTest, StaleMapIsRejected)
{
    RawPointObstacleMap map = make_map({});
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.planning_time_s = 2.0;
    PlannerConfig config;
    SpatiotemporalRouteInitializer initializer;

    const SpatiotemporalRouteResult result = initializer.initialize(
        snapshot, config, make_straight_seed());

    EXPECT_FALSE(result.is_valid);
    EXPECT_EQ(
        result.termination,
        taiphoon::ego_swarm::RouteInitializerTerminationReason::INVALID_INPUT);
}

}  // namespace
