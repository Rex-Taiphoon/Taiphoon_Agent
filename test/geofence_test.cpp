/**
 * @file geofence_test.cpp
 * @brief 水平地理圍籬解析、幾何與各規劃階段整合的純 C++ 測試
 * @author Taiphoon Agent Team
 * @date 2026-11-05
 */

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "ego_swarm/ContinuousSafetyValidator.hpp"
#include "ego_swarm/Geofence.hpp"
#include "ego_swarm/LocalGoalSelector.hpp"
#include "ego_swarm/OptimizedEgoPlanner.hpp"
#include "ego_swarm/PeerTrajectoryTable.hpp"
#include "perception/RawPointObstacleMap.hpp"

namespace
{

using taiphoon::ego_swarm::BsplineTrajectory;
using taiphoon::ego_swarm::ContinuousSafetyValidator;
using taiphoon::ego_swarm::GeoCoordinate;
using taiphoon::ego_swarm::Geofence;
using taiphoon::ego_swarm::LocalGoalConfig;
using taiphoon::ego_swarm::LocalGoalEnvironment;
using taiphoon::ego_swarm::LocalNedFrame;
using taiphoon::ego_swarm::OptimizedEgoPlanner;
using taiphoon::ego_swarm::PathConstraint;
using taiphoon::ego_swarm::PeerTrajectory;
using taiphoon::ego_swarm::PeerTrajectoryTable;
using taiphoon::ego_swarm::PlannerConfig;
using taiphoon::ego_swarm::PlanningSnapshot;
using taiphoon::ego_swarm::SafetyFailureReason;
using taiphoon::ego_swarm::SafetyValidationContext;
using taiphoon::ego_swarm::Vector3;
using taiphoon::ego_swarm::VehiclePlanningState;
using taiphoon::ego_swarm::compute_safe_local_goal;
using taiphoon::ego_swarm::parse_geofence_polygon;
using taiphoon::perception::RawPointObstacleMap;

/// 測試場域半寬（m），對應實際 8×8 m 場域。
constexpr double FIELD_HALF_M = 4.0;
/// 測試用硬性 margin（m）。
constexpr double FIELD_MARGIN_M = 0.5;
/// 測試正確性時使用的寬鬆牆鐘預算（ms），避免結果隨 CPU 負載變動。
constexpr double DETERMINISTIC_TIME_BUDGET_MS = 5000.0;

Geofence make_square_fence(const double half_m, const double margin_m)
{
    Geofence fence;
    EXPECT_TRUE(fence.set_polygon(
        {Vector3{-half_m, -half_m, 0.0}, Vector3{half_m, -half_m, 0.0},
         Vector3{half_m, half_m, 0.0}, Vector3{-half_m, half_m, 0.0}},
        margin_m));
    return fence;
}

BsplineTrajectory make_line(const Vector3& start, const Vector3& end)
{
    // 6 個控制點、每段 2 s：速度遠低於預設動力學上限，只測圍籬。
    constexpr std::size_t CONTROL_POINT_COUNT = 6U;
    std::vector<Vector3> points;
    for (std::size_t index = 0U; index < CONTROL_POINT_COUNT; ++index)
    {
        const double ratio = static_cast<double>(index) /
            static_cast<double>(CONTROL_POINT_COUNT - 1U);
        points.push_back(Vector3{
            start.at(0) + ratio * (end.at(0) - start.at(0)),
            start.at(1) + ratio * (end.at(1) - start.at(1)),
            start.at(2) + ratio * (end.at(2) - start.at(2))});
    }
    return BsplineTrajectory(points, 2.0);
}

PeerTrajectory make_terminal_hold_peer(const int agent_id, const Vector3& position)
{
    PeerTrajectory peer;
    peer.agent_id = agent_id;
    peer.trajectory_id = 1U;
    peer.frame_revision = 1U;
    peer.message_time_s = 0.0;
    peer.start_time_s = 0.5;
    peer.receive_time_s = 0.0;
    peer.end_time_s = peer.start_time_s + 1.0;
    peer.terminal_hold = true;
    peer.trajectory = BsplineTrajectory(
        {position, position, position, position}, 1.0);
    return peer;
}

SafetyValidationContext make_context(
    const Vector3& start,
    const PeerTrajectoryTable& peers,
    const RawPointObstacleMap& map,
    const Geofence* fence)
{
    SafetyValidationContext context;
    context.self_state.position = start;
    context.self_state.timestamp_s = 0.0;
    context.now_s = 0.0;
    context.trajectory_start_time_s = 0.0;
    context.frame_revision = 1U;
    context.trajectory_frame_revision = 1U;
    context.self_agent_id = 1;
    context.peer_table = &peers;
    context.obstacle_map = &map;
    context.geofence = fence;
    return context;
}

TEST(GeofenceParseTest, AcceptsVerticesAndEmptyMeansDisabled)
{
    std::vector<GeoCoordinate> vertices;
    EXPECT_TRUE(parse_geofence_polygon("", vertices));
    EXPECT_TRUE(vertices.empty());
    EXPECT_TRUE(parse_geofence_polygon("   ", vertices));
    EXPECT_TRUE(vertices.empty());

    ASSERT_TRUE(parse_geofence_polygon(
        "24.88,121.06; 24.881 , 121.06 ;24.881,121.061;", vertices));
    ASSERT_EQ(vertices.size(), 3U);
    EXPECT_DOUBLE_EQ(vertices.at(1).latitude_deg, 24.881);
    EXPECT_DOUBLE_EQ(vertices.at(2).longitude_deg, 121.061);
}

TEST(GeofenceParseTest, RejectsMalformedInputFailClosed)
{
    std::vector<GeoCoordinate> vertices;
    for (const std::string text :
         {"24.88,121.06;24.881,121.06", "24.88,121.06;24.881;24.881,121.061",
          "24.88,abc;24.881,121.06;24.881,121.061",
          "24.88,121.06x;24.881,121.06;24.881,121.061",
          "91.0,121.06;24.881,121.06;24.881,121.061",
          "24.88 121.06;24.881,121.06;24.881,121.061"})
    {
        EXPECT_FALSE(parse_geofence_polygon(text, vertices)) << text;
        EXPECT_TRUE(vertices.empty()) << text;
    }
}

TEST(GeofenceGeometryTest, ClearanceIsSignedDistanceToNearestEdge)
{
    const Geofence fence = make_square_fence(FIELD_HALF_M, FIELD_MARGIN_M);
    EXPECT_NEAR(fence.clearance_m(Vector3{0.0, 0.0, -2.0}), 4.0, 1.0e-9);
    EXPECT_NEAR(fence.clearance_m(Vector3{3.0, 0.0, 0.0}), 1.0, 1.0e-9);
    EXPECT_NEAR(fence.clearance_m(Vector3{5.0, 0.0, 0.0}), -1.0, 1.0e-9);
    EXPECT_TRUE(fence.contains_with_margin(Vector3{3.5, 0.0, 0.0}));
    EXPECT_FALSE(fence.contains_with_margin(Vector3{3.6, 0.0, 0.0}));
    EXPECT_FALSE(fence.contains_with_margin(
        Vector3{std::nan(""), 0.0, 0.0}));
}

TEST(GeofenceGeometryTest, ConcavePolygonSegmentCannotCutTheNotch)
{
    Geofence fence;
    // L 形：右上角 (2..4, 2..4) 不在圍籬內。
    ASSERT_TRUE(fence.set_polygon(
        {Vector3{0.0, 0.0, 0.0}, Vector3{4.0, 0.0, 0.0},
         Vector3{4.0, 2.0, 0.0}, Vector3{2.0, 2.0, 0.0},
         Vector3{2.0, 4.0, 0.0}, Vector3{0.0, 4.0, 0.0}},
        0.0));
    EXPECT_LT(fence.clearance_m(Vector3{3.0, 3.0, 0.0}), 0.0);
    EXPECT_GT(fence.clearance_m(Vector3{1.0, 3.0, 0.0}), 0.0);
    EXPECT_GT(fence.clearance_m(Vector3{3.0, 1.0, 0.0}), 0.0);
    EXPECT_FALSE(fence.segment_inside(
        Vector3{3.5, 1.0, 0.0}, Vector3{1.0, 3.5, 0.0}, 0.0));
    EXPECT_TRUE(fence.segment_inside(
        Vector3{1.0, 1.0, 0.0}, Vector3{1.0, 3.0, 0.0}, 0.5));
    EXPECT_FALSE(fence.segment_inside(
        Vector3{1.0, 1.0, 0.0}, Vector3{1.0, 3.0, 0.0}, 1.5));
}

TEST(GeofenceGeometryTest, RejectsSelfIntersectingDegenerateAndBadMargin)
{
    Geofence fence;
    EXPECT_FALSE(fence.set_polygon(
        {Vector3{0.0, 0.0, 0.0}, Vector3{2.0, 2.0, 0.0},
         Vector3{2.0, 0.0, 0.0}, Vector3{0.0, 2.0, 0.0}},
        0.5));
    EXPECT_FALSE(fence.is_valid());
    EXPECT_FALSE(fence.set_polygon(
        {Vector3{0.0, 0.0, 0.0}, Vector3{1.0, 0.0, 0.0},
         Vector3{2.0, 0.0, 0.0}},
        0.5));
    EXPECT_FALSE(fence.set_polygon(
        {Vector3{0.0, 0.0, 0.0}, Vector3{4.0, 0.0, 0.0},
         Vector3{4.0, 4.0, 0.0}},
        -0.1));
    // 無效圍籬一律視為在圍籬外。
    EXPECT_FALSE(fence.contains(Vector3{1.0, 1.0, 0.0}, 0.0));
}

TEST(GeofenceGeometryTest, BuildsFromWgs84AroundFieldCenter)
{
    LocalNedFrame frame;
    ASSERT_TRUE(frame.set_origin(
        GeoCoordinate{24.881051576424703, 121.0696306771756, 50.0}));
    std::vector<GeoCoordinate> vertices;
    for (const Vector3& corner :
         {Vector3{-4.0, -4.0, 0.0}, Vector3{4.0, -4.0, 0.0},
          Vector3{4.0, 4.0, 0.0}, Vector3{-4.0, 4.0, 0.0}})
    {
        GeoCoordinate vertex{};
        ASSERT_TRUE(frame.from_local_ned(corner, vertex));
        vertices.push_back(vertex);
    }
    Geofence fence;
    ASSERT_TRUE(Geofence::from_geodetic(frame, vertices, 0.5, fence));
    EXPECT_NEAR(fence.clearance_m(Vector3{0.0, 0.0, 0.0}), 4.0, 1.0e-3);

    // 約 11 km 外的頂點視為誤填。
    vertices.back().latitude_deg += 0.1;
    EXPECT_FALSE(Geofence::from_geodetic(frame, vertices, 0.5, fence));
    EXPECT_FALSE(fence.is_valid());
}

TEST(GeofenceGeometryTest, RequiredClearanceAllowsOnlyInwardRecovery)
{
    const Geofence fence = make_square_fence(FIELD_HALF_M, FIELD_MARGIN_M);
    EXPECT_NEAR(fence.required_clearance_from_start(2.0), 0.5, 1.0e-12);
    EXPECT_NEAR(fence.required_clearance_from_start(0.2), 0.15, 1.0e-12);
    EXPECT_NEAR(fence.required_clearance_from_start(-1.0), -1.05, 1.0e-12);
}

TEST(GeofenceValidatorTest, RejectsSamplesOutsideMarginAndAllowsInwardRecovery)
{
    const PlannerConfig config;
    ContinuousSafetyValidator validator(config);
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(0.0));
    PeerTrajectoryTable peers;
    const Geofence fence = make_square_fence(FIELD_HALF_M, FIELD_MARGIN_M);

    const Vector3 center{0.0, 0.0, -2.0};
    const auto outward = validator.validate(
        make_line(center, Vector3{3.8, 0.0, -2.0}),
        make_context(center, peers, map, &fence));
    EXPECT_FALSE(outward.is_safe);
    EXPECT_EQ(outward.reason, SafetyFailureReason::GEOFENCE_VIOLATION);
    EXPECT_EQ(ContinuousSafetyValidator::reason_string(outward.reason),
              "geofence_violation");

    const auto inside = validator.validate(
        make_line(center, Vector3{3.0, 0.0, -2.0}),
        make_context(center, peers, map, &fence));
    EXPECT_TRUE(inside.is_safe);
    EXPECT_NEAR(inside.minimum_geofence_clearance_m, 1.0, 1.0e-6);

    // 未啟用圍籬時行為與既有完全相同。
    EXPECT_TRUE(validator.validate(
        make_line(center, Vector3{3.8, 0.0, -2.0}),
        make_context(center, peers, map, nullptr)).is_safe);

    const Vector3 near_edge{3.8, 0.0, -2.0};
    EXPECT_TRUE(validator.validate(
        make_line(near_edge, Vector3{2.0, 0.0, -2.0}),
        make_context(near_edge, peers, map, &fence)).is_safe);
    EXPECT_EQ(validator.validate(
                  make_line(near_edge, Vector3{3.9, 0.0, -2.0}),
                  make_context(near_edge, peers, map, &fence)).reason,
              SafetyFailureReason::GEOFENCE_VIOLATION);

    const Geofence invalid_fence;
    EXPECT_EQ(validator.validate(
                  make_line(center, Vector3{1.0, 0.0, -2.0}),
                  make_context(center, peers, map, &invalid_fence)).reason,
              SafetyFailureReason::GEOFENCE_VIOLATION);
}

TEST(GeofenceLocalGoalTest, DetourCandidateStaysInsideSmallField)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(0.0));
    const Geofence fence = make_square_fence(FIELD_HALF_M, FIELD_MARGIN_M);
    LocalGoalEnvironment environment;
    environment.now_s = 0.0;
    environment.candidate_start_time_s = 0.5;
    environment.self_agent_id = 1;
    environment.peers_are_fresh = true;
    environment.peers.push_back(
        make_terminal_hold_peer(2, Vector3{0.0, 3.0, -2.0}));
    environment.obstacle_map = &map;
    environment.peer_collision_min_distance_m = 2.0;
    const LocalGoalConfig config{10.0, 1.0, 4.5};
    const Vector3 current{0.0, -3.0, -2.0};
    const Vector3 target{0.0, 3.0, -2.0};

    const auto unfenced = compute_safe_local_goal(
        current, target, PathConstraint::FIXED_Z, config, environment);
    ASSERT_TRUE(unfenced.is_valid);
    EXPECT_FALSE(fence.contains_with_margin(unfenced.goal));

    environment.geofence = &fence;
    const auto fenced = compute_safe_local_goal(
        current, target, PathConstraint::FIXED_Z, config, environment);
    ASSERT_TRUE(fenced.is_valid);
    EXPECT_TRUE(fence.contains_with_margin(fenced.goal))
        << fenced.goal.at(0) << "," << fenced.goal.at(1);
    EXPECT_GE(fenced.minimum_peer_distance_m, 2.0);
}

class GeofenceOptimizerTest : public ::testing::TestWithParam<bool>
{
};

TEST_P(GeofenceOptimizerTest, DetourAroundHoldingPeerStaysInsideFence)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(0.0));
    const Geofence fence = make_square_fence(FIELD_HALF_M, FIELD_MARGIN_M);
    PlanningSnapshot snapshot;
    snapshot.self_state.position = Vector3{0.0, -3.0, -2.0};
    snapshot.self_state.timestamp_s = 0.0;
    snapshot.local_goal = Vector3{0.0, 3.0, -2.0};
    snapshot.planning_time_s = 0.0;
    snapshot.trajectory_start_time_s = 0.5;
    snapshot.frame_revision = 1U;
    snapshot.self_agent_id = 1;
    snapshot.path_constraint = PathConstraint::FIXED_Z;
    snapshot.fixed_z_m = -2.0;
    snapshot.obstacle_map = &map;
    snapshot.geofence = &fence;
    snapshot.peers_are_fresh = true;
    snapshot.peers.push_back(
        make_terminal_hold_peer(2, Vector3{0.0, 0.0, -2.0}));

    PlannerConfig config;
    config.optimizer_time_budget_ms = DETERMINISTIC_TIME_BUDGET_MS;
    config.enable_spatiotemporal_route_planning = GetParam();
    OptimizedEgoPlanner planner(config);
    const auto result = planner.plan(snapshot);
    ASSERT_TRUE(result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            result.termination);

    PeerTrajectoryTable peers;
    ASSERT_TRUE(peers.update(snapshot.peers.front()));
    SafetyValidationContext context = make_context(
        snapshot.self_state.position, peers, map, &fence);
    context.trajectory_start_time_s = result.trajectory_start_time_s;
    context.max_future_start_delay_s = 10.0;
    ContinuousSafetyValidator validator(config);
    const auto safety = validator.validate(result.trajectory, context);
    EXPECT_TRUE(safety.is_safe)
        << ContinuousSafetyValidator::reason_string(safety.reason);
    EXPECT_GE(safety.minimum_geofence_clearance_m, FIELD_MARGIN_M - 1.0e-6);
    EXPECT_GE(safety.minimum_peer_distance_m,
              config.peer_collision_min_distance_m);
}

INSTANTIATE_TEST_SUITE_P(
    WithAndWithoutRouteSearch, GeofenceOptimizerTest,
    ::testing::Values(false, true));

/// 8×8 m 交叉情境收尾：繞過中間機後停在目標側邊，目標靠近圍籬軟性緩衝帶。
TEST_P(GeofenceOptimizerTest, FinalApproachAfterDetourReachesGoalNearFence)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(0.0));
    const Geofence fence = make_square_fence(FIELD_HALF_M, FIELD_MARGIN_M);
    for (const double side : {1.05, -1.76})
    {
        PlanningSnapshot snapshot;
        snapshot.self_state.position = Vector3{-2.97, side, -2.0};
        snapshot.self_state.timestamp_s = 0.0;
        snapshot.local_goal = Vector3{-3.19, 0.14, -2.0};
        snapshot.planning_time_s = 0.0;
        snapshot.trajectory_start_time_s = 0.5;
        snapshot.frame_revision = 1U;
        snapshot.self_agent_id = 3;
        snapshot.path_constraint = PathConstraint::FIXED_Z;
        snapshot.fixed_z_m = -2.0;
        snapshot.obstacle_map = &map;
        snapshot.geofence = &fence;
        snapshot.peers_are_fresh = true;
        snapshot.peers.push_back(
            make_terminal_hold_peer(1, Vector3{3.03, 0.14, -2.0}));
        snapshot.peers.push_back(
            make_terminal_hold_peer(2, Vector3{0.1, -0.15, -2.0}));

        PlannerConfig config;
        config.optimizer_time_budget_ms = DETERMINISTIC_TIME_BUDGET_MS;
        config.enable_spatiotemporal_route_planning = GetParam();
        OptimizedEgoPlanner planner(config);
        const auto result = planner.plan(snapshot);
        ASSERT_TRUE(result.is_valid)
            << "side=" << side << " "
            << taiphoon::ego_swarm::planner_termination_reason_string(
                result.termination);
        const auto end = result.trajectory.evaluate_state(
            result.trajectory.duration_s());
        EXPECT_NEAR(end.position.at(0), -3.19, 1.0e-6) << "side=" << side;
        EXPECT_NEAR(end.position.at(1), 0.14, 1.0e-6) << "side=" << side;
    }
}

/// 場域座標（u 長軸、v 短軸）依長軸方位轉成 NED 水平座標。
Vector3 field_to_ned(const double u, const double v, const double bearing_deg)
{
    const double bearing_rad = bearing_deg * M_PI / 180.0;
    return Vector3{
        u * std::cos(bearing_rad) - v * std::sin(bearing_rad),
        u * std::sin(bearing_rad) + v * std::cos(bearing_rad),
        -2.0};
}

/// 8×8 m 交叉情境、1.5 m 間距：中間機與已到位的對向機擋住內側，
/// 只剩中間機與圍籬之間的窄通道可以通過；場域長軸可與格點斜交。
TEST_P(GeofenceOptimizerTest, NarrowCorridorBetweenPeerAndFenceAtReducedSpacing)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(0.0));
    for (const double bearing_deg : {0.0, 143.0})
    {
        Geofence fence;
        ASSERT_TRUE(fence.set_polygon(
            {field_to_ned(-FIELD_HALF_M, -FIELD_HALF_M, bearing_deg),
             field_to_ned(FIELD_HALF_M, -FIELD_HALF_M, bearing_deg),
             field_to_ned(FIELD_HALF_M, FIELD_HALF_M, bearing_deg),
             field_to_ned(-FIELD_HALF_M, FIELD_HALF_M, bearing_deg)},
            FIELD_MARGIN_M));
        PlanningSnapshot snapshot;
        snapshot.self_state.position = field_to_ned(2.65, -2.0, bearing_deg);
        snapshot.self_state.timestamp_s = 0.0;
        snapshot.local_goal = field_to_ned(-3.18, 0.02, bearing_deg);
        snapshot.planning_time_s = 0.0;
        snapshot.trajectory_start_time_s = 0.5;
        snapshot.frame_revision = 1U;
        snapshot.self_agent_id = 3;
        snapshot.path_constraint = PathConstraint::FIXED_Z;
        snapshot.fixed_z_m = -2.0;
        snapshot.obstacle_map = &map;
        snapshot.geofence = &fence;
        snapshot.peers_are_fresh = true;
        snapshot.peers.push_back(make_terminal_hold_peer(
            1, field_to_ned(2.99, 0.24, bearing_deg)));
        snapshot.peers.push_back(make_terminal_hold_peer(
            2, field_to_ned(0.1, -0.3, bearing_deg)));

        PlannerConfig config;
        config.optimizer_time_budget_ms = DETERMINISTIC_TIME_BUDGET_MS;
        config.peer_collision_min_distance_m = 1.5;
        config.enable_spatiotemporal_route_planning = GetParam();
        OptimizedEgoPlanner planner(config);
        const auto result = planner.plan(snapshot);
        ASSERT_TRUE(result.is_valid)
            << "bearing=" << bearing_deg << " "
            << taiphoon::ego_swarm::planner_termination_reason_string(
                result.termination)
            << " min_peer=" << result.minimum_peer_distance_m
            << " route_used=" << result.route_search_used;

        PeerTrajectoryTable peers;
        for (const auto& peer : snapshot.peers)
        {
            ASSERT_TRUE(peers.update(peer));
        }
        SafetyValidationContext context = make_context(
            snapshot.self_state.position, peers, map, &fence);
        context.trajectory_start_time_s = result.trajectory_start_time_s;
        context.max_future_start_delay_s = 10.0;
        ContinuousSafetyValidator validator(config);
        const auto safety = validator.validate(result.trajectory, context);
        EXPECT_TRUE(safety.is_safe)
            << "bearing=" << bearing_deg << " "
            << ContinuousSafetyValidator::reason_string(safety.reason);
        EXPECT_GE(safety.minimum_peer_distance_m,
                  config.peer_collision_min_distance_m)
            << "bearing=" << bearing_deg;
    }
}

}  // namespace
