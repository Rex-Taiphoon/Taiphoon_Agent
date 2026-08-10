/**
 * @file ego_swarm_core_test.cpp
 * @brief EGO-Swarm 純 C++ 核心單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * 測試核心模組的 GPS 共同座標、非連續 Agent ID 槽位、B-spline、
 * 最後有效軌跡、機體中心距離與 deterministic 候選規劃行為。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "ego_swarm/BsplineTrajectory.hpp"
#include "ego_swarm/EgoSwarmPlanner.hpp"
#include "ego_swarm/EgoSwarmTypes.hpp"
#include "ego_swarm/FormationSlotPlanner.hpp"
#include "ego_swarm/PeerTrajectoryTable.hpp"
#include "ego_swarm/SwarmFrame.hpp"

namespace
{

using taiphoon::ego_swarm::BsplineTrajectory;
using taiphoon::ego_swarm::EgoSwarmPlanner;
using taiphoon::ego_swarm::FormationConfig;
using taiphoon::ego_swarm::FormationSlot;
using taiphoon::ego_swarm::FormationSlotPlanner;
using taiphoon::ego_swarm::FormationType;
using taiphoon::ego_swarm::GeoCoordinate;
using taiphoon::ego_swarm::PeerTrajectory;
using taiphoon::ego_swarm::PeerTrajectoryTable;
using taiphoon::ego_swarm::PlannerConfig;
using taiphoon::ego_swarm::SwarmFrame;
using taiphoon::ego_swarm::Vector3;
using taiphoon::ego_swarm::distance_between_centers;

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

TEST(SwarmFrameTest, ConvertsGpsToCommonNed)
{
    SwarmFrame frame;
    ASSERT_TRUE(frame.set_origin(GeoCoordinate{25.000000, 121.000000, 100.0}));

    Vector3 ned{};
    ASSERT_TRUE(frame.to_swarm_ned(
        GeoCoordinate{25.000009, 121.000010, 95.0}, ned));

    EXPECT_NEAR(ned.at(0), 1.0, 0.05);
    EXPECT_NEAR(ned.at(1), 1.0, 0.05);
    EXPECT_NEAR(ned.at(2), 5.0, 1.0e-9);
}

TEST(SwarmFrameTest, RejectsNonFiniteGps)
{
    SwarmFrame frame;
    ASSERT_TRUE(frame.set_origin(GeoCoordinate{25.0, 121.0, 100.0}));

    Vector3 ned{4.0, 5.0, 6.0};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(frame.to_swarm_ned(
        GeoCoordinate{nan, 121.0, 100.0}, ned));
    EXPECT_NEAR(ned.at(0), 0.0, 1.0e-12);
    EXPECT_NEAR(ned.at(1), 0.0, 1.0e-12);
    EXPECT_NEAR(ned.at(2), 0.0, 1.0e-12);
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

TEST(PeerTrajectoryTableTest, PreservesLastValidTrajectory)
{
    PeerTrajectoryTable table;
    PeerTrajectory first;
    first.agent_id = 17;
    first.start_time_s = 10.0;
    first.trajectory = make_test_trajectory(2.0);
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

TEST(EgoSwarmGeometryTest, UsesDirectCenterToCenterDistance)
{
    const Vector3 first{0.0, 0.0, 0.0};
    const Vector3 second{1.0, 2.0, 2.0};

    EXPECT_NEAR(distance_between_centers(first, second), 3.0, 1.0e-9);
}

TEST(EgoSwarmPlannerTest, SelectsDeterministicObstacleAvoidingCandidate)
{
    PlannerConfig config;
    config.candidate_offset_m = 3.0;
    config.obstacle_clearance_m = 1.0;
    config.peer_collision_min_distance_m = 1.5;
    EgoSwarmPlanner planner(config);
    PeerTrajectoryTable peers;

    const Vector3 current{0.0, 0.0, 0.0};
    const Vector3 target{10.0, 0.0, 0.0};
    const std::vector<Vector3> obstacles{Vector3{5.0, 0.0, 0.0}};
    const auto result = planner.plan(
        current, target, obstacles, peers, 0.0, 1);

    ASSERT_TRUE(result.is_valid);
    EXPECT_FALSE(result.is_hold);
    EXPECT_TRUE(result.has_finite_output());
    ASSERT_FALSE(result.trajectory.control_points().empty());
    EXPECT_GT(std::abs(
        result.trajectory.control_points().at(2).at(1)), 0.1);
    EXPECT_GE(result.minimum_obstacle_distance_m, 1.0);
}

}  // namespace
