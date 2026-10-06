/**
 * @file dynamic_safety_test.cpp
 * @brief Dynamic safety 與 emergency brake 純 C++ 測試
 * @author Taiphoon Agent Team
 * @date 2026-08-12
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "ego_swarm/ContinuousSafetyValidator.hpp"
#include "ego_swarm/EmergencyBrakePlanner.hpp"
#include "ego_swarm/EgoSwarmTypes.hpp"
#include "ego_swarm/PeerTrajectoryTable.hpp"
#include "perception/RawPointObstacleMap.hpp"

namespace
{

using taiphoon::ego_swarm::BsplineTrajectory;
using taiphoon::ego_swarm::ContinuousSafetyValidator;
using taiphoon::ego_swarm::EmergencyBrakePlanner;
using taiphoon::ego_swarm::EgoSwarmPlanner;
using taiphoon::ego_swarm::PeerTrajectory;
using taiphoon::ego_swarm::PeerTrajectoryStatus;
using taiphoon::ego_swarm::PeerTrajectoryTable;
using taiphoon::ego_swarm::PlannerConfig;
using taiphoon::ego_swarm::SafetyFailureReason;
using taiphoon::ego_swarm::SafetyValidationContext;
using taiphoon::ego_swarm::Vector3;
using taiphoon::ego_swarm::VehiclePlanningState;
using taiphoon::perception::RawPointObstacleMap;

PeerTrajectory make_peer(
    const int agent_id,
    const double start_time_s,
    const double receive_time_s,
    const std::vector<Vector3>& control_points,
    const double knot_interval_s)
{
    PeerTrajectory peer;
    peer.agent_id = agent_id;
    peer.trajectory_id = 1U;
    peer.frame_revision = 1U;
    peer.message_time_s = receive_time_s;
    peer.start_time_s = start_time_s;
    peer.receive_time_s = receive_time_s;
    peer.trajectory = BsplineTrajectory(control_points, knot_interval_s);
    peer.end_time_s = start_time_s + peer.trajectory.duration_s();
    return peer;
}

SafetyValidationContext make_context(
    const VehiclePlanningState& state,
    const PeerTrajectoryTable& peers,
    const RawPointObstacleMap& map,
    const double now_s)
{
    SafetyValidationContext context;
    context.self_state = state;
    context.now_s = now_s;
    context.trajectory_start_time_s = 0.0;
    context.frame_revision = 1U;
    context.trajectory_frame_revision = 1U;
    context.self_agent_id = 1;
    context.peer_table = &peers;
    context.obstacle_map = &map;
    return context;
}

double speed_at(
    const BsplineTrajectory& trajectory,
    const double time_s)
{
    const auto sample = trajectory.evaluate_state(time_s);
    return std::sqrt(taiphoon::ego_swarm::squared_norm(sample.velocity));
}

TEST(ContinuousSafetyValidatorTest, EnforcesTwoMeterDefaultAtBoundary)
{
    const PlannerConfig config;
    ContinuousSafetyValidator validator(config);
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(0.0));
    VehiclePlanningState state;
    state.position = Vector3{0.0, 0.0, -2.0};
    const BsplineTrajectory hold(
        std::vector<Vector3>(config.control_point_count, state.position), 1.0);

    for (const double distance_m : {1.99, 2.0, 2.01})
    {
        PeerTrajectoryTable peers;
        auto peer = make_peer(
            2, 0.0, 0.0,
            std::vector<Vector3>(config.control_point_count,
                                 Vector3{0.0, distance_m, -2.0}),
            1.0);
        peer.terminal_hold = true;
        ASSERT_TRUE(peers.update(peer));
        const auto result = validator.validate(
            hold, make_context(state, peers, map, 0.0));
        if (distance_m < 2.0)
        {
            EXPECT_FALSE(result.is_safe);
            EXPECT_EQ(result.reason, SafetyFailureReason::PEER_COLLISION);
        }
        else
        {
            EXPECT_TRUE(result.is_safe);
        }
    }
}

TEST(VehiclePlanningStateTest, RejectsNonFiniteState)
{
    VehiclePlanningState state;
    EXPECT_TRUE(state.is_finite());
    state.velocity.at(0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(state.is_finite());
}

TEST(EmergencyBrakePlannerTest, StationaryVehicleUsesStaticHold)
{
    PlannerConfig config;
    config.max_velocity_mps = 5.0;
    config.max_acceleration_mps2 = 2.0;
    config.max_jerk_mps3 = 4.0;
    EmergencyBrakePlanner planner(config);
    VehiclePlanningState state;
    state.position = Vector3{2.0, -1.0, -3.0};

    const auto result = planner.plan(state);
    ASSERT_TRUE(result.is_valid);
    ASSERT_TRUE(result.is_hold);
    ASSERT_EQ(result.trajectory.control_points().size(),
              config.control_point_count);
    for (const auto& point : result.trajectory.control_points())
    {
        EXPECT_EQ(point, state.position);
    }
}

TEST(EmergencyBrakePlannerTest, ThreeMetersPerSecondBrakeReducesSpeed)
{
    PlannerConfig config;
    config.max_velocity_mps = 5.0;
    config.max_acceleration_mps2 = 2.0;
    config.max_jerk_mps3 = 4.0;
    config.sample_interval_s = 0.05;
    EmergencyBrakePlanner planner(config);
    VehiclePlanningState state;
    state.velocity = Vector3{3.0, 0.0, 0.0};

    const auto result = planner.plan(state);
    ASSERT_TRUE(result.is_valid);
    ASSERT_FALSE(result.trajectory.control_points().front() ==
                 result.trajectory.control_points().back());
    EXPECT_NEAR(speed_at(result.trajectory, 0.0), 3.0, 1.0e-6);
    EXPECT_LT(speed_at(result.trajectory, result.trajectory.duration_s()),
              1.0e-6);

    double previous_speed = speed_at(result.trajectory, 0.0);
    for (std::size_t index = 1U; index <= 20U; ++index)
    {
        const double ratio = static_cast<double>(index) / 20.0;
        const double speed = speed_at(
            result.trajectory, result.trajectory.duration_s() * ratio);
        EXPECT_LE(speed, previous_speed + 1.0e-5);
        previous_speed = speed;
    }
}

TEST(EmergencyBrakePlannerTest, HighSpeedBrakeRemainsFinite)
{
    PlannerConfig config;
    config.max_velocity_mps = 5.0;
    config.max_acceleration_mps2 = 3.0;
    config.max_jerk_mps3 = 6.0;
    EmergencyBrakePlanner planner(config);
    VehiclePlanningState state;
    state.velocity = Vector3{10.0, -2.0, 0.0};

    const auto result = planner.plan(state);
    ASSERT_TRUE(result.is_valid);
    EXPECT_TRUE(result.has_finite_output());
    for (std::size_t index = 0U; index <= 20U; ++index)
    {
        const double ratio = static_cast<double>(index) / 20.0;
        const auto sample = result.trajectory.evaluate_state(
            result.trajectory.duration_s() * ratio);
        ASSERT_TRUE(sample.is_valid);
        EXPECT_TRUE(std::isfinite(sample.position.at(0)));
        EXPECT_TRUE(std::isfinite(sample.velocity.at(0)));
        EXPECT_TRUE(std::isfinite(sample.acceleration.at(0)));
    }
}

TEST(EmergencyBrakePlannerTest, NoisyAccelerationStillProducesBrake)
{
    PlannerConfig config;
    config.max_velocity_mps = 5.0;
    config.max_acceleration_mps2 = 2.0;
    config.max_jerk_mps3 = 4.0;
    EmergencyBrakePlanner planner(config);
    VehiclePlanningState state;
    state.velocity = Vector3{3.0, 0.0, 0.0};
    state.acceleration = Vector3{500.0, -200.0, 100.0};

    const auto result = planner.plan(state);
    ASSERT_TRUE(result.is_valid);
    EXPECT_TRUE(result.has_finite_output());
    EXPECT_NEAR(speed_at(result.trajectory, 0.0), 3.0, 1.0e-6);
    EXPECT_LT(speed_at(result.trajectory, result.trajectory.duration_s()),
              1.0e-6);
}

TEST(ContinuousSafetyValidatorTest, RejectsHeadOnPeerConflict)
{
    PlannerConfig config;
    config.max_velocity_mps = 20.0;
    config.max_acceleration_mps2 = 20.0;
    config.max_jerk_mps3 = 100.0;
    config.peer_collision_min_distance_m = 1.0;
    config.sample_interval_s = 0.05;
    ContinuousSafetyValidator validator(config);
    PeerTrajectoryTable peers;
    ASSERT_TRUE(peers.update(make_peer(
        2, 0.0, 0.0,
        {Vector3{10.0, 0.0, 0.0}, Vector3{10.0, 0.0, 0.0},
         Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}},
        2.0)));
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(0.0));
    VehiclePlanningState state;
    auto context = make_context(state, peers, map, 0.0);
    context.self_agent_id = 3;
    const BsplineTrajectory candidate(
        {Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0},
         Vector3{10.0, 0.0, 0.0}, Vector3{10.0, 0.0, 0.0}},
        2.0);

    const auto result = validator.validate(candidate, context);
    EXPECT_FALSE(result.is_safe);
    EXPECT_EQ(result.reason, SafetyFailureReason::PEER_COLLISION);
}

TEST(ContinuousSafetyValidatorTest, RejectsSuddenObstacle)
{
    PlannerConfig config;
    config.max_velocity_mps = 20.0;
    config.max_acceleration_mps2 = 20.0;
    config.max_jerk_mps3 = 100.0;
    config.obstacle_clearance_m = 1.0;
    config.sample_interval_s = 0.05;
    ContinuousSafetyValidator validator(config);
    PeerTrajectoryTable peers;
    RawPointObstacleMap map;
    ASSERT_TRUE(map.update({Vector3{5.0, 0.0, 0.0}}, 0.0));
    VehiclePlanningState state;
    const auto context = make_context(state, peers, map, 0.0);
    const BsplineTrajectory candidate(
        {Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0},
         Vector3{10.0, 0.0, 0.0}, Vector3{10.0, 0.0, 0.0}},
        2.0);

    const auto result = validator.validate(candidate, context);
    EXPECT_FALSE(result.is_safe);
    EXPECT_EQ(result.reason, SafetyFailureReason::OBSTACLE_COLLISION);
}

TEST(ContinuousSafetyValidatorTest, RejectsStalePeerAndMap)
{
    PlannerConfig config;
    config.max_velocity_mps = 20.0;
    config.max_acceleration_mps2 = 20.0;
    config.max_jerk_mps3 = 100.0;
    ContinuousSafetyValidator validator(config);
    PeerTrajectoryTable peers;
    ASSERT_TRUE(peers.update(make_peer(
        2, 0.0, 0.0,
        {Vector3{20.0, 0.0, 0.0}, Vector3{20.0, 0.0, 0.0},
         Vector3{20.0, 0.0, 0.0}, Vector3{20.0, 0.0, 0.0}},
        1.0)));
    RawPointObstacleMap stale_map;
    ASSERT_TRUE(stale_map.set_explicit_empty(0.0));
    VehiclePlanningState state;
    state.timestamp_s = 2.0;
    const auto context = make_context(state, peers, stale_map, 2.0);
    const BsplineTrajectory candidate(
        {Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0},
         Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}},
        3.0);

    const auto result = validator.validate(candidate, context);
    EXPECT_FALSE(result.is_safe);
    EXPECT_EQ(result.reason, SafetyFailureReason::STALE_MAP);

    RawPointObstacleMap fresh_map;
    ASSERT_TRUE(fresh_map.set_explicit_empty(2.0));
    const auto stale_peer_context = make_context(
        state, peers, fresh_map, 2.0);
    const auto stale_peer_result = validator.validate(
        candidate, stale_peer_context);
    EXPECT_FALSE(stale_peer_result.is_safe);
    EXPECT_EQ(stale_peer_result.reason, SafetyFailureReason::STALE_PEER);
    EXPECT_EQ(peers.get_status(2, 2.0), PeerTrajectoryStatus::STALE);
}

TEST(ContinuousSafetyValidatorTest, RejectsPlannerFailureAndReportsFiniteMetrics)
{
    PlannerConfig config;
    config.max_velocity_mps = 20.0;
    config.max_acceleration_mps2 = 20.0;
    config.max_jerk_mps3 = 100.0;
    ContinuousSafetyValidator validator(config);
    PeerTrajectoryTable peers;
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(0.0));
    VehiclePlanningState state;
    const auto context = make_context(state, peers, map, 0.0);
    const BsplineTrajectory invalid_trajectory;

    const auto result = validator.validate(invalid_trajectory, context);
    EXPECT_FALSE(result.is_safe);
    EXPECT_EQ(result.reason, SafetyFailureReason::INVALID_TIME);
    EXPECT_TRUE(result.has_finite_metrics());
}

TEST(ContinuousSafetyValidatorTest, RejectsDynamicInfeasibility)
{
    PlannerConfig config;
    config.max_velocity_mps = 2.0;
    config.max_acceleration_mps2 = 1.0;
    config.max_jerk_mps3 = 2.0;
    ContinuousSafetyValidator validator(config);
    PeerTrajectoryTable peers;
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(0.0));
    VehiclePlanningState state;
    const auto context = make_context(state, peers, map, 0.0);
    const BsplineTrajectory candidate(
        {Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0},
         Vector3{0.0, 0.0, 0.0}, Vector3{1.0, 0.0, 0.0}},
        1.0);

    const auto result = validator.validate(candidate, context);
    EXPECT_FALSE(result.is_safe);
    EXPECT_EQ(result.reason, SafetyFailureReason::DYNAMIC_INFEASIBLE);
}

TEST(ContinuousSafetyValidatorTest, AcceptsDeterministicPlannerOutput)
{
    PlannerConfig config;
    config.max_velocity_mps = 2.0;
    config.max_acceleration_mps2 = 1.0;
    config.max_jerk_mps3 = 2.0;
    config.sample_interval_s = 0.05;
    EgoSwarmPlanner planner(config);
    ContinuousSafetyValidator validator(config);
    PeerTrajectoryTable peers;
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(0.0));
    const Vector3 current{0.0, 0.0, -8.0};
    const Vector3 target{5.0, 0.0, -8.0};
    const auto planned = planner.plan(
        current, target, map, peers, 0.0, 1, true);
    ASSERT_TRUE(planned.is_valid);

    VehiclePlanningState state;
    state.position = current;
    const auto context = make_context(state, peers, map, 0.0);
    const auto result = validator.validate(planned.trajectory, context);
    EXPECT_TRUE(result.is_safe)
        << ContinuousSafetyValidator::reason_string(result.reason)
        << " max_v=" << result.maximum_velocity_mps
        << " max_a=" << result.maximum_acceleration_mps2
        << " max_j=" << result.maximum_jerk_mps3;
}

}  // namespace
