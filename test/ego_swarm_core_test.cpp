/**
 * @file ego_swarm_core_test.cpp
 * @brief EGO-Swarm 純 C++ 核心單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * 測試核心模組的 WGS84 與 receiver-local NED、非連續 Agent ID 槽位、B-spline、
 * 同儕軌跡生命週期、機體中心距離與 deterministic 候選規劃行為。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <array>
#include <utility>
#include <vector>

#include "ego_swarm/BsplineTrajectory.hpp"
#include "ego_swarm/EgoSwarmPlanner.hpp"
#include "ego_swarm/EgoSwarmTypes.hpp"
#include "ego_swarm/FormationSlotPlanner.hpp"
#include "ego_swarm/FormationAssignment.hpp"
#include "ego_swarm/FormationReentryPolicy.hpp"
#include "ego_swarm/FormationTransition.hpp"
#include "ego_swarm/MovementFrame.hpp"
#include "ego_swarm/PeerTrajectoryTable.hpp"
#include "ego_swarm/PositionArrival.hpp"
#include "ego_swarm/LocalNedFrame.hpp"
#include "ego_swarm/TrajectoryPriority.hpp"
#include "perception/RawPointObstacleMap.hpp"
#include "utils/CommandTypes.hpp"

namespace
{

using taiphoon::ego_swarm::BsplineTrajectory;
using taiphoon::ego_swarm::EgoSwarmPlanner;
using taiphoon::ego_swarm::FormationConfig;
using taiphoon::ego_swarm::FormationSlot;
using taiphoon::ego_swarm::FormationSlotPlanner;
using taiphoon::ego_swarm::FormationAssignment;
using taiphoon::ego_swarm::FormationAssignmentResult;
using taiphoon::ego_swarm::FormationTransition;
using taiphoon::ego_swarm::FormationTransitionObservation;
using taiphoon::ego_swarm::FormationType;
using taiphoon::ego_swarm::GeoCoordinate;
using taiphoon::ego_swarm::PeerTrajectory;
using taiphoon::ego_swarm::PeerTrajectoryStatus;
using taiphoon::ego_swarm::PeerTrajectoryTable;
using taiphoon::ego_swarm::PlannerConfig;
using taiphoon::ego_swarm::LocalNedFrame;
using taiphoon::ego_swarm::Vector3;
using taiphoon::ego_swarm::distance_between_centers;
using taiphoon::ego_swarm::has_trajectory_priority;
using taiphoon::perception::RawPointObstacleMap;

/**
 * @brief 建立同儕軌跡表測試用的有限 B-spline
 *
 * @param start_x 軌跡控制點的 North 座標
 * @return 測試用 B-spline 軌跡
 */
BsplineTrajectory make_test_trajectory(const double start_x)
{
    const std::vector<Vector3> control_points{
        Vector3{start_x, -2.0, 0.0},
        Vector3{start_x, -1.0, 0.0},
        Vector3{start_x, 1.0, 0.0},
        Vector3{start_x, 2.0, 0.0}};
    return BsplineTrajectory(control_points, 1.0);
}

PeerTrajectory make_peer_trajectory(
    const int agent_id,
    const uint32_t trajectory_id,
    const double start_time_s,
    const double receive_time_s,
    const double start_x)
{
    PeerTrajectory peer;
    peer.agent_id = agent_id;
    peer.trajectory_id = trajectory_id;
    peer.frame_revision = 1U;
    peer.message_time_s = receive_time_s;
    peer.start_time_s = start_time_s;
    peer.receive_time_s = receive_time_s;
    peer.trajectory = make_test_trajectory(start_x);
    peer.end_time_s = peer.start_time_s + peer.trajectory.duration_s();
    return peer;
}

/**
 * @brief 建立測試用 fresh explicit empty obstacle map
 * @return 可供 planner 使用的空障礙物地圖
 */
RawPointObstacleMap make_empty_obstacle_map()
{
    RawPointObstacleMap map;
    const bool updated = map.set_explicit_empty(0.0);
    EXPECT_TRUE(updated);
    return map;
}

/**
 * @brief 建立測試用 fresh raw point obstacle map
 * @param points 障礙物點
 * @return 可供 planner 使用的障礙物地圖
 */
RawPointObstacleMap make_obstacle_map(const std::vector<Vector3>& points)
{
    RawPointObstacleMap map;
    const bool updated = map.update(points, 0.0);
    EXPECT_TRUE(updated);
    return map;
}

TEST(LocalNedFrameTest, ConvertsWgs84ToLocalNed)
{
    LocalNedFrame frame;
    ASSERT_TRUE(frame.set_origin(GeoCoordinate{25.000000, 121.000000, 100.0}));

    Vector3 ned{};
    ASSERT_TRUE(frame.to_local_ned(
        GeoCoordinate{25.000009, 121.000010, 95.0}, ned));

    EXPECT_NEAR(ned.at(0), 1.0, 0.05);
    EXPECT_NEAR(ned.at(1), 1.0, 0.05);
    EXPECT_NEAR(ned.at(2), 5.0, 1.0e-9);
}

TEST(LocalNedFrameTest, ConvertsReceiverLocalNedBackToWgs84)
{
    LocalNedFrame frame;
    ASSERT_TRUE(frame.set_origin(GeoCoordinate{25.000000, 121.000000, 100.0}));

    const Vector3 expected_ned{12.0, -7.0, 3.5};
    GeoCoordinate global{};
    ASSERT_TRUE(frame.from_local_ned(expected_ned, global));

    Vector3 round_trip{};
    ASSERT_TRUE(frame.to_local_ned(global, round_trip));
    EXPECT_NEAR(round_trip.at(0), expected_ned.at(0), 1.0e-6);
    EXPECT_NEAR(round_trip.at(1), expected_ned.at(1), 1.0e-6);
    EXPECT_NEAR(round_trip.at(2), expected_ned.at(2), 1.0e-9);
}

TEST(LocalNedFrameTest, RejectsNonFiniteGps)
{
    LocalNedFrame frame;
    ASSERT_TRUE(frame.set_origin(GeoCoordinate{25.0, 121.0, 100.0}));

    Vector3 ned{4.0, 5.0, 6.0};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(frame.to_local_ned(
        GeoCoordinate{nan, 121.0, 100.0}, ned));
    EXPECT_NEAR(ned.at(0), 0.0, 1.0e-12);
    EXPECT_NEAR(ned.at(1), 0.0, 1.0e-12);
    EXPECT_NEAR(ned.at(2), 0.0, 1.0e-12);
}

TEST(LocalNedFrameTest, RejectsDifferentFrameOrigins)
{
    const GeoCoordinate origin{25.0, 121.0, 100.0};
    const GeoCoordinate compatible_origin{25.0, 121.0, 100.0005};
    const GeoCoordinate different_origin{25.0001, 121.0, 100.0};

    EXPECT_TRUE(LocalNedFrame::are_origins_compatible(
        origin, compatible_origin));
    EXPECT_FALSE(LocalNedFrame::are_origins_compatible(
        origin, different_origin));
}

TEST(LocalNedFrameTest, FreezesOriginAndRevision)
{
    LocalNedFrame frame;
    const GeoCoordinate origin{25.0, 121.0, 100.0};
    const GeoCoordinate different_origin{25.0001, 121.0, 100.0};

    ASSERT_TRUE(frame.set_origin(origin));
    EXPECT_TRUE(frame.has_origin());
    EXPECT_EQ(frame.frame_revision(), 1U);
    EXPECT_FALSE(frame.set_origin(different_origin));
    EXPECT_TRUE(LocalNedFrame::are_origins_compatible(frame.origin(), origin));
    EXPECT_EQ(frame.frame_revision(), 1U);
}

TEST(LocalNedFrameTest, DoesNotInventOriginWhenOriginIsUnavailable)
{
    LocalNedFrame frame;
    Vector3 ned{1.0, 2.0, 3.0};

    EXPECT_FALSE(frame.has_origin());
    EXPECT_EQ(frame.frame_revision(), 0U);
    EXPECT_FALSE(frame.to_local_ned(
        GeoCoordinate{25.0, 121.0, 100.0}, ned));
    const Vector3 zero{0.0, 0.0, 0.0};
    EXPECT_EQ(ned, zero);
}

TEST(FormationSlotPlannerTest, SupportsNonContiguousAgentIds)
{
    const std::vector<int> agent_ids{42, 3, 10};
    FormationConfig config;
    config.type = FormationType::HORI_LINEAR;
    config.spacing_m = 2.0;

    std::vector<FormationSlot> slots;
    ASSERT_TRUE(FormationSlotPlanner::compute_all_slots(
        agent_ids, config, slots));
    ASSERT_EQ(slots.size(), 3U);

    EXPECT_EQ(slots.at(0).agent_id, 3);
    EXPECT_EQ(slots.at(1).agent_id, 10);
    EXPECT_EQ(slots.at(2).agent_id, 42);
    EXPECT_NEAR(slots.at(0).offset_ned.at(1), -2.0, 1.0e-9);
    EXPECT_NEAR(slots.at(1).offset_ned.at(1), 0.0, 1.0e-9);
    EXPECT_NEAR(slots.at(2).offset_ned.at(1), 2.0, 1.0e-9);
}

TEST(FormationSlotPlannerTest, ComputesCircleAndVerticalSlots)
{
    const std::vector<int> agent_ids{10, 20, 30, 40};
    FormationConfig circle_config;
    circle_config.type = FormationType::CIRCLE;
    circle_config.radius_m = 5.0;

    FormationSlot circle_slot;
    ASSERT_TRUE(FormationSlotPlanner::compute_slot(
        20, agent_ids, circle_config, circle_slot));
    EXPECT_NEAR(circle_slot.offset_ned.at(0), 0.0, 1.0e-9);
    EXPECT_NEAR(circle_slot.offset_ned.at(1), 5.0, 1.0e-9);

    FormationConfig vertical_config;
    vertical_config.type = FormationType::VERT_LINEAR;
    vertical_config.spacing_m = 1.5;
    ASSERT_TRUE(FormationSlotPlanner::compute_slot(
        30, agent_ids, vertical_config, circle_slot));
    EXPECT_NEAR(circle_slot.offset_ned.at(2), 0.75, 1.0e-9);
}

TEST(MovementFrameTest, ConvertsBodyForwardAndRightUsingCommandYaw)
{
    Vector3 offset{};
    ASSERT_TRUE(taiphoon::ego_swarm::body_horizontal_offset_to_ned(
        0.0, 5.0, 0.0, offset));
    EXPECT_NEAR(offset.at(0), 5.0, 1.0e-9);
    EXPECT_NEAR(offset.at(1), 0.0, 1.0e-9);

    ASSERT_TRUE(taiphoon::ego_swarm::body_horizontal_offset_to_ned(
        taiphoon::ego_swarm::EGO_SWARM_PI / 2.0, 5.0, 0.0, offset));
    EXPECT_NEAR(offset.at(0), 0.0, 1.0e-9);
    EXPECT_NEAR(offset.at(1), 5.0, 1.0e-9);
}

TEST(MovementFrameTest, ConvertsNedAxisDirections)
{
    using taiphoon::ego_swarm::NedMoveDirection;
    using taiphoon::ego_swarm::ned_axis_offset;

    const std::array<std::pair<NedMoveDirection, Vector3>, 6> expected = {{
        {NedMoveDirection::FORWARD, {2.0, 0.0, 0.0}},
        {NedMoveDirection::BACKWARD, {-2.0, 0.0, 0.0}},
        {NedMoveDirection::LEFT, {0.0, -2.0, 0.0}},
        {NedMoveDirection::RIGHT, {0.0, 2.0, 0.0}},
        {NedMoveDirection::UP, {0.0, 0.0, -2.0}},
        {NedMoveDirection::DOWN, {0.0, 0.0, 2.0}},
    }};

    for (const auto& item : expected)
    {
        Vector3 offset{};
        ASSERT_TRUE(ned_axis_offset(item.first, 2.0, offset));
        EXPECT_DOUBLE_EQ(offset.at(0), item.second.at(0));
        EXPECT_DOUBLE_EQ(offset.at(1), item.second.at(1));
        EXPECT_DOUBLE_EQ(offset.at(2), item.second.at(2));
    }
}

TEST(MovementFrameTest, RejectsNonFiniteNedDistance)
{
    Vector3 offset{1.0, 2.0, 3.0};
    EXPECT_FALSE(taiphoon::ego_swarm::ned_axis_offset(
        taiphoon::ego_swarm::NedMoveDirection::FORWARD,
        std::numeric_limits<double>::quiet_NaN(), offset));
    EXPECT_FALSE(taiphoon::ego_swarm::ned_axis_offset(
        taiphoon::ego_swarm::NedMoveDirection::RIGHT,
        std::numeric_limits<double>::infinity(), offset));
}

TEST(FormationReentryPolicyTest, ReactivatesExistingEpochAndClearsSoloGoal)
{
    taiphoon::ego_swarm::FormationReentryState state;
    state.has_goal = true;
    state.has_position_target = true;
    state.preserve_altitude = true;
    state.formation_command_pending = true;
    state.formation_membership_change_pending = true;

    ASSERT_TRUE(taiphoon::ego_swarm::reactivate_existing_formation(
        state, true, true));
    EXPECT_TRUE(state.has_formation);
    EXPECT_FALSE(state.has_goal);
    EXPECT_FALSE(state.has_position_target);
    EXPECT_FALSE(state.preserve_altitude);
    EXPECT_FALSE(state.formation_command_pending);
    EXPECT_FALSE(state.formation_membership_change_pending);
    EXPECT_TRUE(state.formation_initial_plan_pending);
}

TEST(FormationReentryPolicyTest, DoesNotReactivateActiveOrUnmatchedEpoch)
{
    taiphoon::ego_swarm::FormationReentryState active;
    active.has_formation = true;
    EXPECT_FALSE(taiphoon::ego_swarm::reactivate_existing_formation(
        active, true, true));

    taiphoon::ego_swarm::FormationReentryState unmatched;
    EXPECT_FALSE(taiphoon::ego_swarm::reactivate_existing_formation(
        unmatched, false, false));
    EXPECT_FALSE(unmatched.has_formation);
}

TEST(MovementFrameTest, RejectsNonFiniteAttitude)
{
    std::array<float, 4> quaternion{1.0F, 0.0F, 0.0F,
        std::numeric_limits<float>::quiet_NaN()};
    double yaw = 0.0;
    EXPECT_FALSE(taiphoon::ego_swarm::quaternion_to_ned_yaw(quaternion, yaw));
}

TEST(CommandParserTest, ParsesVerticalFormationSpacingAsSingleParameter)
{
    const auto parsed = taiphoon::utils::parse_command_string(
        "FORMATION_VERT_LINEAR 3");

    EXPECT_EQ(parsed.command,
              taiphoon::utils::AgentCommand::FORMATION_VERT_LINEAR);
    EXPECT_TRUE(parsed.has_param);
    EXPECT_FALSE(parsed.has_multi_params);
    EXPECT_FLOAT_EQ(parsed.param_value, 3.0F);
}

TEST(CommandParserTest, ParsesVerticalMovementAliases)
{
    const auto up = taiphoon::utils::parse_command_string("UP 1");
    EXPECT_EQ(up.command, taiphoon::utils::AgentCommand::MOVE_UP);
    EXPECT_TRUE(up.has_param);
    EXPECT_FLOAT_EQ(up.param_value, 1.0F);

    const auto down = taiphoon::utils::parse_command_string("DOWN 2");
    EXPECT_EQ(down.command, taiphoon::utils::AgentCommand::MOVE_DOWN);
    EXPECT_TRUE(down.has_param);
    EXPECT_FLOAT_EQ(down.param_value, 2.0F);
}

TEST(CommandParserTest, ParsesCircleAndHorizontalFormationParameters)
{
    const auto circle = taiphoon::utils::parse_command_string(
        "FORMATION_CIRCLE 5 8");
    EXPECT_TRUE(circle.has_param);
    EXPECT_TRUE(circle.has_multi_params);
    EXPECT_FLOAT_EQ(circle.param_value, 5.0F);
    EXPECT_FLOAT_EQ(circle.param2, 8.0F);

    const auto horizontal = taiphoon::utils::parse_command_string(
        "FORMATION_HORI_LINEAR 4 8");
    EXPECT_TRUE(horizontal.has_param);
    EXPECT_TRUE(horizontal.has_multi_params);
    EXPECT_FLOAT_EQ(horizontal.param_value, 4.0F);
    EXPECT_FLOAT_EQ(horizontal.param2, 8.0F);
}

TEST(FormationAssignmentTest, UsesMinimumCostOneToOneMapping)
{
    const std::vector<int> ids{42, 3, 10};
    const std::vector<Vector3> positions{
        Vector3{0.0, 2.0, 0.0}, Vector3{0.0, -2.0, 0.0},
        Vector3{0.0, 0.0, 0.0}};
    FormationConfig config;
    config.type = FormationType::HORI_LINEAR;
    config.spacing_m = 2.0;
    FormationAssignmentResult result;
    ASSERT_TRUE(FormationAssignment::assign(
        ids, positions, config, {}, 2.0, result));
    ASSERT_TRUE(result.is_valid);
    ASSERT_EQ(result.agent_to_slot.size(), 3U);
    EXPECT_EQ(result.agent_to_slot.at(3), 0U);
    EXPECT_EQ(result.agent_to_slot.at(10), 1U);
    EXPECT_EQ(result.agent_to_slot.at(42), 2U);
}

TEST(FormationAssignmentTest, SwitchPenaltyKeepsPreviousOwnership)
{
    const std::vector<int> ids{1, 2};
    const std::vector<Vector3> positions{
        Vector3{0.0, -1.0, 0.0}, Vector3{0.0, 1.0, 0.0}};
    FormationConfig config;
    config.type = FormationType::HORI_LINEAR;
    config.spacing_m = 2.0;
    FormationAssignmentResult result;
    ASSERT_TRUE(FormationAssignment::assign(
        ids, positions, config, {{1, 0U}, {2, 1U}}, 2.0, result));
    EXPECT_EQ(result.agent_to_slot.at(1), 0U);
    EXPECT_EQ(result.agent_to_slot.at(2), 1U);
}

TEST(FormationAssignmentTest, RejectsInvalidInputAsAWhole)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    FormationConfig config;
    FormationAssignmentResult result;
    ASSERT_FALSE(FormationAssignment::assign(
        {1, 1}, {Vector3{0.0, 0.0, 0.0}, Vector3{nan, 0.0, 0.0}},
        config, {}, 2.0, result));
    EXPECT_FALSE(result.is_valid);
    EXPECT_TRUE(result.agent_to_slot.empty());
}

TEST(FormationTransitionTest, AllowsStationaryFreshPredecessorWithoutTerminalFlag)
{
    const FormationTransitionObservation predecessor{
        1, Vector3{5.0, 0.0, -8.0}, Vector3{5.0, 0.0, -8.0},
        Vector3{0.05, 0.0, 0.0}, true};
    EXPECT_TRUE(FormationTransition::predecessors_ready(
        {1, 2, 3, 4}, 2, {predecessor}, 1.0, 0.2));
}

TEST(FormationTransitionTest, RejectsPredecessorThatIsMovingOrNotAtTarget)
{
    const FormationTransitionObservation predecessor{
        1, Vector3{3.0, 0.0, -8.0}, Vector3{5.0, 0.0, -8.0},
        Vector3{0.3, 0.0, 0.0}, true};
    EXPECT_FALSE(FormationTransition::predecessors_ready(
        {1, 2}, 2, {predecessor}, 1.0, 0.2));
}

TEST(FormationTransitionTest, MissingPredecessorFailsClosed)
{
    EXPECT_FALSE(FormationTransition::predecessors_ready(
        {1, 2}, 2, {}, 1.0, 0.2));
}

TEST(FormationTransitionTest, ChoosesSafeGeometryOrderInsteadOfAgentIdOrder)
{
    // 初始機體沿 East 間隔排列；最低 ID 的目標會靠近 Agent 2，
    // 因此必須先釋放 Agent 2，才能讓整個 CIRCLE 轉換保持安全距離。
    const std::vector<int> members{1, 2, 3, 4};
    const std::vector<FormationTransitionObservation> observations{
        {1, Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 7.0, 0.0}, {}, true},
        {2, Vector3{0.0, 8.0, 0.0}, Vector3{-5.0, 12.0, 0.0}, {}, true},
        {3, Vector3{0.0, 16.0, 0.0}, Vector3{0.0, 17.0, 0.0}, {}, true},
        {4, Vector3{0.0, 24.0, 0.0}, Vector3{5.0, 12.0, 0.0}, {}, true}};
    std::vector<int> order;

    ASSERT_TRUE(FormationTransition::compute_safe_order(
        members, observations, 3.0, order));
    ASSERT_EQ(order.size(), 4U);
    EXPECT_EQ(order.at(0), 2);
    EXPECT_EQ(order.at(1), 1);
    EXPECT_EQ(order.at(2), 4);
    EXPECT_EQ(order.at(3), 3);
}

TEST(FormationTransitionTest, RejectsWhenNoSingleSafeReleaseExists)
{
    const std::vector<FormationTransitionObservation> observations{
        {1, Vector3{-1.0, 0.0, 0.0}, Vector3{1.0, 0.0, 0.0}, {}, true},
        {2, Vector3{1.0, 0.0, 0.0}, Vector3{-1.0, 0.0, 0.0}, {}, true}};
    std::vector<int> order;

    EXPECT_FALSE(FormationTransition::compute_safe_order(
        {1, 2}, observations, 3.0, order));
    EXPECT_TRUE(order.empty());
}

TEST(FormationTransitionTest, AvoidsCompletedSlotAndBacktracksWhenNeeded)
{
    // Agent 2 可以先移動，但其新槽位會落在 Agent 3 的後續路徑上；
    // deterministic search 必須回溯並選擇 Agent 3、Agent 2 的順序。
    const std::vector<FormationTransitionObservation> observations{
        {2, Vector3{0.0, 20.0, 0.0}, Vector3{0.0, 10.0, 0.0}, {}, true},
        {3, Vector3{5.0, 10.0, 0.0}, Vector3{-5.0, 10.0, 0.0}, {}, true}};
    std::vector<int> order;

    ASSERT_TRUE(FormationTransition::compute_safe_order(
        {2, 3}, observations, 3.0, order));
    ASSERT_EQ(order.size(), 2U);
    EXPECT_EQ(order.at(0), 3);
    EXPECT_EQ(order.at(1), 2);
}

TEST(FormationTransitionTest, CircleSnapshotReleasesSafePredecessorFirst)
{
    // 這是四機沿 East 間隔 8 m、CIRCLE radius=5 m 的實際 SITL
    // snapshot。Agent 1 的 West 槽位靠近 Agent 2 的原始位置，故必須
    // 先讓 Agent 2 前往 North 槽位，再釋放 Agent 1；反過來會把
    // Agent 2 的直線路徑導向已佔用的 West 槽位。
    const std::vector<FormationTransitionObservation> observations{
        {1, Vector3{0.105419, -11.734577, 0.0}, Vector3{0.0, -5.0, 0.0}, {}, true},
        {2, Vector3{0.105419, -4.039772, 0.0}, Vector3{5.0, 0.0, 0.0}, {}, true},
        {3, Vector3{-0.105419, 3.655032, 0.0}, Vector3{-5.0, 0.0, 0.0}, {}, true},
        {4, Vector3{-0.105419, 12.119317, 0.0}, Vector3{0.0, 5.0, 0.0}, {}, true}};
    std::vector<int> order;

    ASSERT_TRUE(FormationTransition::compute_safe_order(
        {1, 2, 3, 4}, observations, 3.0, order));
    ASSERT_EQ(order.size(), 4U);
    EXPECT_EQ(order.at(0), 2);
    EXPECT_EQ(order.at(1), 1);
    EXPECT_EQ(order.at(2), 3);
    EXPECT_EQ(order.at(3), 4);
}

TEST(FormationTransitionTest, UsesHorizontalDetourBeforeWaiting)
{
    const std::vector<FormationTransitionObservation> observations{
        {1, Vector3{0.0, -2.0, -8.0}, Vector3{10.0, -2.0, -8.0}, {}, true},
        {2, Vector3{5.0, 0.0, -8.0}, Vector3{5.0, 10.0, -8.0}, {}, true}};
    std::vector<int> order;

    ASSERT_TRUE(FormationTransition::compute_safe_order(
        {1, 2}, observations, 3.0, 4.0, order));
    ASSERT_EQ(order.size(), 2U);
    EXPECT_EQ(order.at(0), 1);
}

TEST(BsplineTrajectoryTest, EvaluatesEndpointsAndFiniteValues)
{
    const std::vector<Vector3> control_points{
        Vector3{0.0, 0.0, 0.0},
        Vector3{1.0, 0.0, 0.0},
        Vector3{2.0, 1.0, 0.0},
        Vector3{3.0, 1.0, 0.0}};
    const BsplineTrajectory trajectory(control_points, 2.0);

    ASSERT_TRUE(trajectory.is_valid());
    EXPECT_NEAR(trajectory.duration_s(), 2.0, 1.0e-9);
    const auto start = trajectory.evaluate(0.0);
    const auto middle = trajectory.evaluate(1.0);
    const auto end = trajectory.evaluate(2.0);
    ASSERT_TRUE(start.is_valid);
    ASSERT_TRUE(middle.is_valid);
    ASSERT_TRUE(end.is_valid);
    EXPECT_NEAR(start.position.at(0), 0.0, 1.0e-9);
    EXPECT_NEAR(end.position.at(0), 3.0, 1.0e-9);
    EXPECT_TRUE(std::isfinite(middle.position.at(0)));
    EXPECT_TRUE(std::isfinite(middle.position.at(1)));
}

TEST(BsplineTrajectoryTest, EvaluatesAnalyticCubicDerivatives)
{
    // 這組控制點形成 x(t) = t^3 的 cubic Bezier 曲線，方便驗證
    // position、velocity、acceleration 與 jerk 的解析導數。
    const std::vector<Vector3> control_points{
        Vector3{0.0, 0.0, 0.0},
        Vector3{0.0, 0.0, 0.0},
        Vector3{0.0, 0.0, 0.0},
        Vector3{1.0, 0.0, 0.0}};
    const BsplineTrajectory trajectory(control_points, 1.0);

    const auto middle = trajectory.evaluate(0.5);
    ASSERT_TRUE(middle.is_valid);
    EXPECT_NEAR(middle.position.at(0), 0.125, 1.0e-9);
    EXPECT_NEAR(middle.velocity.at(0), 0.75, 1.0e-9);
    EXPECT_NEAR(middle.acceleration.at(0), 3.0, 1.0e-9);
    EXPECT_NEAR(middle.jerk.at(0), 6.0, 1.0e-9);

    const auto start = trajectory.evaluate(0.0);
    const auto end = trajectory.evaluate(1.0);
    ASSERT_TRUE(start.is_valid);
    ASSERT_TRUE(end.is_valid);
    EXPECT_NEAR(start.velocity.at(0), 0.0, 1.0e-9);
    EXPECT_NEAR(start.acceleration.at(0), 0.0, 1.0e-9);
    EXPECT_NEAR(start.jerk.at(0), 6.0, 1.0e-9);
    EXPECT_NEAR(end.velocity.at(0), 3.0, 1.0e-9);
    EXPECT_NEAR(end.acceleration.at(0), 6.0, 1.0e-9);
    EXPECT_NEAR(end.jerk.at(0), 6.0, 1.0e-9);
}

TEST(BsplineTrajectoryTest, DerivativesAreContinuousAtInteriorKnot)
{
    const std::vector<Vector3> control_points{
        Vector3{0.0, 0.0, 0.0},
        Vector3{1.0, 2.0, 0.0},
        Vector3{2.0, -1.0, 0.0},
        Vector3{3.0, 3.0, 0.0},
        Vector3{4.0, 0.0, 0.0},
        Vector3{5.0, 1.0, 0.0}};
    const BsplineTrajectory trajectory(control_points, 1.0);

    const auto before = trajectory.evaluate(1.0 - 1.0e-7);
    const auto after = trajectory.evaluate(1.0 + 1.0e-7);
    ASSERT_TRUE(before.is_valid);
    ASSERT_TRUE(after.is_valid);
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        EXPECT_NEAR(
            before.position.at(axis), after.position.at(axis), 1.0e-5);
        EXPECT_NEAR(
            before.velocity.at(axis), after.velocity.at(axis), 1.0e-5);
        EXPECT_NEAR(
            before.acceleration.at(axis), after.acceleration.at(axis), 1.0e-5);
    }
}

TEST(EgoSwarmPlannerTest, DurationScalesWithDistance)
{
    PlannerConfig config;
    config.knot_interval_s = 1.0;
    config.max_velocity_m_s = 2.0;
    config.max_acceleration_m_s2 = 1.0;
    config.candidate_offset_m = 0.0;
    EgoSwarmPlanner planner(config);
    PeerTrajectoryTable peers;
    const auto empty_map = make_empty_obstacle_map();

    const auto short_plan = planner.plan(
        Vector3{0.0, 0.0, 0.0}, Vector3{1.0, 0.0, 0.0}, empty_map, peers,
        0.0, 1, true);
    const auto long_plan = planner.plan(
        Vector3{0.0, 0.0, 0.0}, Vector3{10.0, 0.0, 0.0}, empty_map, peers,
        0.0, 1, true);

    ASSERT_TRUE(short_plan.trajectory.is_valid());
    ASSERT_TRUE(long_plan.trajectory.is_valid());
    EXPECT_NEAR(short_plan.trajectory.duration_s(), 3.0, 1.0e-9);
    EXPECT_GT(long_plan.trajectory.duration_s(),
              short_plan.trajectory.duration_s());
    EXPECT_GE(long_plan.trajectory.duration_s(), 5.0);
    EXPECT_GE(long_plan.trajectory.duration_s(),
              2.0 * std::sqrt(10.0));
}

TEST(EgoSwarmPlannerTest, ExactHoldLocksEveryControlPointToTarget)
{
    PlannerConfig config;
    config.knot_interval_s = 0.5;
    config.control_point_count = 6U;
    EgoSwarmPlanner planner(config);

    const Vector3 target{12.345678, -6.789012, -15.25};
    const auto result = planner.make_exact_hold_result(target);

    ASSERT_TRUE(result.is_valid);
    ASSERT_TRUE(result.is_hold);
    ASSERT_TRUE(result.trajectory.is_valid());
    ASSERT_EQ(result.trajectory.control_points().size(), 6U);
    for (const auto& point : result.trajectory.control_points())
    {
        EXPECT_NEAR(point.at(0), target.at(0), 1.0e-12);
        EXPECT_NEAR(point.at(1), target.at(1), 1.0e-12);
        EXPECT_NEAR(point.at(2), target.at(2), 1.0e-12);
    }
}

TEST(PositionArrivalTest, RequiresPx4LocalAndPlannerPositions)
{
    const Vector3 current_local{10.0, 20.0, -5.0};
    const Vector3 target_local{10.2, 20.2, -5.1};
    const Vector3 current_planner{1.0, 2.0, 3.0};
    const Vector3 target_planner{1.2, 2.2, 3.1};

    const auto reached = taiphoon::ego_swarm::evaluate_position_arrival(
        current_local, target_local, current_planner, target_planner,
        0.5, 0.5);
    EXPECT_TRUE(reached.local_valid);
    EXPECT_TRUE(reached.planner_valid);
    EXPECT_TRUE(reached.reached);

    const auto local_missed =
        taiphoon::ego_swarm::evaluate_position_arrival(
            Vector3{11.0, 20.0, -5.0}, target_local,
            current_planner, target_planner, 0.5, 0.5);
    EXPECT_FALSE(local_missed.reached);

    const auto planner_missed =
        taiphoon::ego_swarm::evaluate_position_arrival(
            current_local, target_local,
            Vector3{2.0, 2.0, 3.0}, target_planner, 0.5, 0.5);
    EXPECT_FALSE(planner_missed.reached);
}

TEST(BsplineTrajectoryTest, RejectsNonFiniteControlPoint)
{
    const double inf = std::numeric_limits<double>::infinity();
    const std::vector<Vector3> control_points{
        Vector3{0.0, 0.0, 0.0},
        Vector3{1.0, 0.0, 0.0},
        Vector3{inf, 1.0, 0.0},
        Vector3{3.0, 1.0, 0.0}};
    const BsplineTrajectory trajectory(control_points, 1.0);

    EXPECT_FALSE(trajectory.is_valid());
    EXPECT_FALSE(trajectory.evaluate(0.5).is_valid);
}

TEST(BsplineTrajectoryTest, RejectsInvalidInputAndNonFiniteEvaluationTime)
{
    const std::vector<Vector3> control_points{
        Vector3{0.0, 0.0, 0.0},
        Vector3{1.0, 0.0, 0.0},
        Vector3{2.0, 1.0, 0.0},
        Vector3{3.0, 1.0, 0.0}};
    const double nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(BsplineTrajectory(control_points, 0.0).is_valid());
    EXPECT_FALSE(BsplineTrajectory(
        std::vector<Vector3>(3U, Vector3{0.0, 0.0, 0.0}), 1.0).is_valid());

    const BsplineTrajectory trajectory(control_points, 1.0);
    EXPECT_FALSE(trajectory.evaluate(nan).is_valid);
}

TEST(BsplineTrajectoryTest, ClampsEvaluationTimeToFiniteEndpoints)
{
    const std::vector<Vector3> control_points{
        Vector3{0.0, 0.0, 0.0},
        Vector3{1.0, 0.0, 0.0},
        Vector3{2.0, 1.0, 0.0},
        Vector3{3.0, 1.0, 0.0}};
    const BsplineTrajectory trajectory(control_points, 1.0);

    const auto before_start = trajectory.evaluate(-1.0);
    const auto after_end = trajectory.evaluate(2.0);
    ASSERT_TRUE(before_start.is_valid);
    ASSERT_TRUE(after_end.is_valid);
    EXPECT_EQ(before_start.position, control_points.front());
    EXPECT_EQ(after_end.position, control_points.back());
}

TEST(PeerTrajectoryTableTest, PreservesLastValidTrajectory)
{
    PeerTrajectoryTable table;
    PeerTrajectory first = make_peer_trajectory(17, 1U, 10.0, 10.0, 2.0);
    ASSERT_TRUE(table.update(first));

    PeerTrajectory result;
    ASSERT_TRUE(table.get_last_valid(17, result));
    EXPECT_NEAR(
        result.trajectory.control_points().at(0).at(0), 2.0, 1.0e-9);

    PeerTrajectory invalid = first;
    invalid.trajectory = BsplineTrajectory();
    EXPECT_FALSE(table.update(invalid));

    PeerTrajectory after_invalid_update;
    ASSERT_TRUE(table.get_last_valid(17, after_invalid_update));
    EXPECT_NEAR(
        after_invalid_update.trajectory.control_points().at(0).at(0),
        2.0,
        1.0e-9);
    EXPECT_FALSE(table.get_last_valid(99, result));
}

TEST(PeerTrajectoryTableTest, DistinguishesActiveStaleMissingAndInvalid)
{
    PeerTrajectoryTable table;
    const PeerTrajectory peer = make_peer_trajectory(2, 1U, 0.0, 0.0, 2.0);
    ASSERT_TRUE(table.update(peer));

    EXPECT_EQ(table.get_status(2, 0.5), PeerTrajectoryStatus::ACTIVE);
    EXPECT_TRUE(table.is_fresh(2, 0.5));
    EXPECT_EQ(table.get_status(2, 2.0), PeerTrajectoryStatus::STALE);
    EXPECT_FALSE(table.is_fresh(2, 2.0));
    EXPECT_EQ(table.get_status(99, 0.0), PeerTrajectoryStatus::MISSING);

    PeerTrajectory invalid = peer;
    invalid.agent_id = 3;
    invalid.start_time_s = 10.0;
    invalid.receive_time_s = 0.0;
    invalid.message_time_s = 0.0;
    invalid.end_time_s = invalid.start_time_s +
        invalid.trajectory.duration_s();
    EXPECT_FALSE(table.update(invalid));
    EXPECT_EQ(table.get_status(3, 0.0), PeerTrajectoryStatus::INVALID);
}

TEST(PeerTrajectoryTableTest, PruneTurnsExpiredPeerIntoMissing)
{
    PeerTrajectoryTable table;
    ASSERT_TRUE(table.update(
        make_peer_trajectory(2, 1U, 0.0, 0.0, 2.0)));

    EXPECT_EQ(table.prune(2.0), 1U);
    EXPECT_EQ(table.get_status(2, 2.0), PeerTrajectoryStatus::MISSING);
    EXPECT_FALSE(table.contains(2));
}

TEST(PeerTrajectoryTableTest, KeepsTerminalHoldAfterTrajectoryEnd)
{
    PeerTrajectoryTable table(
        taiphoon::ego_swarm::PeerTrajectoryTableConfig{10.0, 1.0});
    PeerTrajectory peer = make_peer_trajectory(2, 1U, 0.0, 0.0, 2.0);
    peer.terminal_hold = true;
    ASSERT_TRUE(table.update(peer));

    EXPECT_EQ(table.get_status(2, 10.0), PeerTrajectoryStatus::TERMINAL_HOLD);
    EXPECT_TRUE(table.is_fresh(2, 10.0));

    std::vector<PeerTrajectory> active;
    table.get_active(10.0, active);
    ASSERT_EQ(active.size(), 1U);
    EXPECT_TRUE(active.front().terminal_hold);

    EXPECT_EQ(table.prune(10.0), 0U);
    EXPECT_EQ(table.prune(11.1), 1U);
    EXPECT_EQ(table.get_status(2, 11.1), PeerTrajectoryStatus::MISSING);
}

TEST(PeerTrajectoryTableTest, MissingExpectedPeerFailsClosed)
{
    PeerTrajectoryTable table;
    EXPECT_TRUE(table.all_fresh(0.0));
    EXPECT_FALSE(table.all_fresh(0.0, std::vector<int>{2}));
}

TEST(PeerTrajectoryTableTest, RejectsFutureAndReplayedTrajectory)
{
    PeerTrajectoryTable table;
    PeerTrajectory future = make_peer_trajectory(2, 1U, 2.0, 0.0, 2.0);
    EXPECT_FALSE(table.update(future));
    EXPECT_EQ(table.get_status(2, 0.0), PeerTrajectoryStatus::INVALID);

    ASSERT_TRUE(table.update(
        make_peer_trajectory(2, 4U, 0.0, 0.0, 2.0)));
    PeerTrajectory replay = make_peer_trajectory(2, 3U, 0.0, 0.1, 2.0);
    EXPECT_FALSE(table.update(replay));
    PeerTrajectory active;
    ASSERT_TRUE(table.get_fresh(2, 0.1, active));
    EXPECT_EQ(active.trajectory_id, 4U);
}

TEST(PeerTrajectoryTableTest, AllowsNewTrajectorySequenceAfterPeerErase)
{
    PeerTrajectoryTable table;
    ASSERT_TRUE(table.update(
        make_peer_trajectory(2, 4U, 0.0, 0.0, 2.0)));
    ASSERT_TRUE(table.erase(2));

    EXPECT_TRUE(table.update(
        make_peer_trajectory(2, 1U, 0.0, 0.0, 3.0)));
    PeerTrajectory active;
    ASSERT_TRUE(table.get_fresh(2, 0.0, active));
    EXPECT_EQ(active.trajectory_id, 1U);
}

TEST(EgoSwarmGeometryTest, UsesDirectCenterToCenterDistance)
{
    const Vector3 first{0.0, 0.0, 0.0};
    const Vector3 second{1.0, 2.0, 2.0};

    EXPECT_NEAR(distance_between_centers(first, second), 3.0, 1.0e-9);
}

TEST(TrajectoryPriorityTest, EarlierStartTimeWinsAndAgentIdBreaksTies)
{
    EXPECT_TRUE(has_trajectory_priority(1.0, 5, 2.0, 1));
    EXPECT_FALSE(has_trajectory_priority(2.0, 5, 1.0, 1));
    EXPECT_TRUE(has_trajectory_priority(1.0, 1, 1.0, 2));
    EXPECT_FALSE(has_trajectory_priority(1.0, 2, 1.0, 1));
}

TEST(EgoSwarmPlannerTest, SelectsDeterministicObstacleAvoidingCandidate)
{
    PlannerConfig config;
    config.candidate_offset_m = 3.0;
    config.obstacle_clearance_m = 1.0;
    config.peer_collision_min_distance_m = 1.5;
    EgoSwarmPlanner planner(config);
    PeerTrajectoryTable peers;
    const auto obstacle_map = make_obstacle_map({Vector3{5.0, 0.0, 0.0}});

    const Vector3 current{0.0, 0.0, 0.0};
    const Vector3 target{10.0, 0.0, 0.0};
    const auto result = planner.plan(
        current, target, obstacle_map, peers, 0.0, 1);

    ASSERT_TRUE(result.is_valid);
    EXPECT_FALSE(result.is_hold);
    EXPECT_TRUE(result.has_finite_output());
    ASSERT_FALSE(result.trajectory.control_points().empty());
    EXPECT_GT(std::abs(
        result.trajectory.control_points().at(2).at(1)), 0.1);
    EXPECT_GE(result.minimum_obstacle_distance_m, 1.0);
}

TEST(EgoSwarmPlannerTest, RejectsCandidateCollidingWithPeerFutureTrajectory)
{
    PlannerConfig config;
    config.peer_collision_min_distance_m = 3.0;
    EgoSwarmPlanner planner(config);
    PeerTrajectoryTable peers;
    const auto empty_map = make_empty_obstacle_map();

    PeerTrajectory peer = make_peer_trajectory(2, 1U, 0.0, 0.0, 0.0);
    ASSERT_TRUE(peers.update(peer));

    const auto result = planner.plan(
        Vector3{0.0, 0.0, 0.0},
        Vector3{10.0, 0.0, 0.0},
        empty_map,
        peers,
        0.0,
        3);

    EXPECT_FALSE(result.is_valid);
    EXPECT_TRUE(result.is_hold);
    EXPECT_TRUE(result.trajectory.is_valid());
    EXPECT_TRUE(result.has_finite_output());
}

TEST(EgoSwarmPlannerTest, EarlierRouteHasPriorityOverConflictingPeer)
{
    PlannerConfig config;
    config.peer_collision_min_distance_m = 3.0;
    EgoSwarmPlanner planner(config);
    PeerTrajectoryTable peers;
    const auto empty_map = make_empty_obstacle_map();
    ASSERT_TRUE(peers.update(
        make_peer_trajectory(2, 1U, 0.0, 0.0, 0.0)));

    const auto result = planner.plan(
        Vector3{0.0, 0.0, 0.0},
        Vector3{10.0, 0.0, 0.0},
        empty_map,
        peers,
        0.0,
        1);

    EXPECT_TRUE(result.is_valid);
    EXPECT_FALSE(result.is_hold);
}

TEST(EgoSwarmPlannerTest, DoesNotUsePeerEndpointAfterTrajectoryEnd)
{
    EgoSwarmPlanner planner;
    PeerTrajectoryTable peers;
    const auto empty_map = make_empty_obstacle_map();
    const PeerTrajectory peer = make_peer_trajectory(2, 1U, 0.0, 0.0, 10.0);
    ASSERT_TRUE(peers.update(peer));

    const auto result = planner.plan(
        Vector3{0.0, 0.0, 0.0}, Vector3{10.0, 0.0, 0.0}, empty_map, peers,
        0.0, 1);

    EXPECT_TRUE(result.is_valid);
    EXPECT_FALSE(result.is_hold);
}

TEST(EgoSwarmPlannerTest, EmptyObstacleInputIsAcceptedAsCurrentBaseline)
{
    EgoSwarmPlanner planner;
    PeerTrajectoryTable peers;
    const auto empty_map = make_empty_obstacle_map();

    const auto result = planner.plan(
        Vector3{0.0, 0.0, 0.0},
        Vector3{5.0, 0.0, 0.0},
        empty_map,
        peers,
        0.0,
        1);

    ASSERT_TRUE(result.is_valid);
    EXPECT_FALSE(result.is_hold);
    EXPECT_DOUBLE_EQ(result.minimum_obstacle_distance_m, 0.0);
    EXPECT_TRUE(result.has_finite_output());
}

TEST(EgoSwarmPlannerTest, RejectsMissingOrStaleObstacleMap)
{
    EgoSwarmPlanner planner;
    PeerTrajectoryTable peers;
    RawPointObstacleMap map(1.0);

    const auto missing_result = planner.plan(
        Vector3{0.0, 0.0, 0.0}, Vector3{5.0, 0.0, 0.0}, map, peers,
        0.0, 1);
    EXPECT_FALSE(missing_result.is_valid);

    ASSERT_TRUE(map.set_explicit_empty(0.0));
    const auto stale_result = planner.plan(
        Vector3{0.0, 0.0, 0.0}, Vector3{5.0, 0.0, 0.0}, map, peers,
        2.0, 1);
    EXPECT_FALSE(stale_result.is_valid);
}

TEST(EgoSwarmPlannerTest, HorizontalPlanPreservesAltitude)
{
    EgoSwarmPlanner planner;
    PeerTrajectoryTable peers;
    const auto empty_map = make_empty_obstacle_map();
    const Vector3 current{0.0, 0.0, -8.0};
    const Vector3 target{0.0, -15.0, -8.0};

    const auto result = planner.plan(
        current, target, empty_map, peers, 0.0, 1, true);

    ASSERT_TRUE(result.is_valid);
    ASSERT_FALSE(result.is_hold);
    for (const Vector3& point : result.trajectory.control_points())
    {
        EXPECT_NEAR(point.at(2), -8.0, 1.0e-9);
    }
}

TEST(EgoSwarmPlannerTest, FormationPlanDoesNotUseVerticalDetour)
{
    EgoSwarmPlanner planner;
    PeerTrajectoryTable peers;
    const auto obstacle_map = make_obstacle_map({Vector3{5.0, 0.0, -9.0}});
    const Vector3 current{0.0, 0.0, -8.0};
    const Vector3 target{10.0, 0.0, -10.0};

    const auto result = planner.plan(
        current, target, obstacle_map, peers, 0.0, 1, true);

    ASSERT_TRUE(result.is_valid);
    ASSERT_FALSE(result.is_hold);
    const auto& control_points = result.trajectory.control_points();
    ASSERT_FALSE(control_points.empty());
    for (std::size_t index = 0U; index < control_points.size(); ++index)
    {
        const double progress = static_cast<double>(index) /
            static_cast<double>(control_points.size() - 1U);
        const double expected_z = current.at(2) +
            (target.at(2) - current.at(2)) * progress;
        EXPECT_NEAR(control_points.at(index).at(2), expected_z, 1.0e-9);
    }
}

}  // namespace
