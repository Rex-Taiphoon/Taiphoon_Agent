/**
 * @file optimized_planner_test.cpp
 * @brief Planner seam、optimizer、reservation 與 convergence 測試
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "ego_swarm/ContinuousSafetyValidator.hpp"
#include "ego_swarm/EmergencyBrakePlanner.hpp"
#include "ego_swarm/FormationConvergence.hpp"
#include "ego_swarm/FormationReservation.hpp"
#include "ego_swarm/OptimizedEgoPlanner.hpp"
#include "ego_swarm/ProjectedGradientSolver.hpp"
#include "perception/RawPointObstacleMap.hpp"

namespace
{

using taiphoon::ego_swarm::BsplineTrajectory;
using taiphoon::ego_swarm::ContinuousSafetyValidator;
using taiphoon::ego_swarm::EmergencyBrakePlanner;
using taiphoon::ego_swarm::FormationConvergenceEvaluator;
using taiphoon::ego_swarm::FormationConvergenceObservation;
using taiphoon::ego_swarm::FormationReservationPlanner;
using taiphoon::ego_swarm::IOptimizationObjective;
using taiphoon::ego_swarm::OptimizationEvaluation;
using taiphoon::ego_swarm::OptimizedEgoPlanner;
using taiphoon::ego_swarm::PathConstraint;
using taiphoon::ego_swarm::PeerTrajectory;
using taiphoon::ego_swarm::PlannerConfig;
using taiphoon::ego_swarm::PlannerTerminationReason;
using taiphoon::ego_swarm::PlanningSnapshot;
using taiphoon::ego_swarm::ProjectedGradientSolver;
using taiphoon::ego_swarm::ProjectedGradientSolverConfig;
using taiphoon::ego_swarm::SafetyFailureReason;
using taiphoon::ego_swarm::SafetyValidationContext;
using taiphoon::ego_swarm::PeerTrajectoryTable;
using taiphoon::ego_swarm::SolverTerminationReason;
using taiphoon::ego_swarm::Vector3;
using taiphoon::perception::RawPointObstacleMap;

RawPointObstacleMap make_empty_map()
{
    RawPointObstacleMap map;
    EXPECT_TRUE(map.set_explicit_empty(0.0));
    return map;
}

PlanningSnapshot make_snapshot(RawPointObstacleMap& map)
{
    PlanningSnapshot snapshot;
    snapshot.self_state.position = Vector3{0.0, 0.0, 0.0};
    snapshot.self_state.timestamp_s = 0.0;
    snapshot.local_goal = Vector3{8.0, 0.0, 0.0};
    snapshot.planning_time_s = 0.0;
    snapshot.trajectory_start_time_s = 0.5;
    snapshot.frame_revision = 1U;
    snapshot.self_agent_id = 2;
    snapshot.fixed_z_m = 0.0;
    snapshot.obstacle_map = &map;
    snapshot.peers_are_fresh = true;
    return snapshot;
}

BsplineTrajectory make_peer_path(
    const Vector3& start,
    const Vector3& end)
{
    const Vector3 delta{
        (end.at(0) - start.at(0)) / 3.0,
        (end.at(1) - start.at(1)) / 3.0,
        (end.at(2) - start.at(2)) / 3.0};
    return BsplineTrajectory(
        {start,
         Vector3{start.at(0) + delta.at(0), start.at(1) + delta.at(1),
                 start.at(2) + delta.at(2)},
         Vector3{start.at(0) + 2.0 * delta.at(0),
                 start.at(1) + 2.0 * delta.at(1),
                 start.at(2) + 2.0 * delta.at(2)},
         end},
        1.0);
}

PeerTrajectory make_terminal_hold_peer(
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
    peer.end_time_s = peer.start_time_s + 1.0;
    peer.terminal_hold = true;
    peer.trajectory = BsplineTrajectory(
        {position, position, position, position}, 1.0);
    return peer;
}

struct QuadraticObjective final : IOptimizationObjective
{
    OptimizationEvaluation evaluate(
        const std::vector<Vector3>& points) const override
    {
        OptimizationEvaluation result;
        result.is_valid = !points.empty();
        for (const Vector3& point : points)
        {
            result.total_cost += point.at(0) * point.at(0);
        }
        return result;
    }
};

struct NonfinitePerturbationObjective final : IOptimizationObjective
{
    OptimizationEvaluation evaluate(
        const std::vector<Vector3>& points) const override
    {
        OptimizationEvaluation result;
        result.is_valid = true;
        result.total_cost = 0.0;
        if (points.size() > 1U && std::abs(points.at(1).at(0) - 1.0) > 1.0e-9)
        {
            result.total_cost = std::numeric_limits<double>::quiet_NaN();
        }
        return result;
    }
};

TEST(OptimizedSolverTest, NominalAndFailureReasonsAreExplicit)
{
    ProjectedGradientSolver solver;
    ProjectedGradientSolverConfig config;
    config.max_iterations = 20;
    config.time_budget_ms = 100.0;
    const std::vector<Vector3> seed{
        Vector3{0.0, 0.0, 0.0}, Vector3{1.0, 0.0, 0.0},
        Vector3{1.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}};
    const ProjectedGradientSolver::Projection identity =
        [](std::vector<Vector3>&) { return true; };

    const auto nominal = solver.solve(seed, QuadraticObjective{}, config, identity);
    EXPECT_TRUE(nominal.is_valid);
    EXPECT_EQ(nominal.termination, SolverTerminationReason::CONVERGED);
    EXPECT_NEAR(nominal.control_points.at(1).at(0), 0.0, 0.05);

    const auto nonfinite = solver.solve(
        seed, NonfinitePerturbationObjective{}, config, identity);
    EXPECT_EQ(nonfinite.termination, SolverTerminationReason::NONFINITE_GRADIENT);

    config.max_iterations = 1;
    const auto iteration_limit = solver.solve(
        seed, QuadraticObjective{}, config, identity);
    EXPECT_EQ(iteration_limit.termination, SolverTerminationReason::ITERATION_LIMIT);

    config.max_iterations = 20;
    config.time_budget_ms = 0.001;
    const auto timeout = solver.solve(seed, QuadraticObjective{}, config, identity);
    EXPECT_EQ(timeout.termination, SolverTerminationReason::TIMEOUT);
}

TEST(OptimizedPlannerTest, OptimizedPlannerConvergesInOpenSpace)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    PlannerConfig config;
    config.optimizer_time_budget_ms = 20.0;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    ASSERT_TRUE(result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            result.termination)
        << " iterations=" << result.iteration_count
        << " planning_ms=" << result.planning_time_ms;
    EXPECT_EQ(result.backend,
              taiphoon::ego_swarm::PlannerBackendKind::OPTIMIZED);
    EXPECT_EQ(result.termination, PlannerTerminationReason::SUCCESS);
    EXPECT_TRUE(result.trajectory.is_valid());
    EXPECT_TRUE(result.has_finite_output());
    const auto endpoint = result.trajectory.evaluate(
        result.trajectory.duration_s());
    ASSERT_TRUE(endpoint.is_valid);
    EXPECT_NEAR(endpoint.position.at(0), snapshot.local_goal.at(0), 1.0e-6);
}

TEST(OptimizedPlannerTest, LongHorizontalMoveReachesThirtyMetersWithinTwentySeconds)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 1;
    snapshot.local_goal = Vector3{0.0, 30.0, 0.0};
    snapshot.path_constraint = PathConstraint::FIXED_Z;
    snapshot.fixed_z_m = 0.0;

    PlannerConfig config;
    config.optimizer_time_budget_ms = 20.0;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    ASSERT_TRUE(result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            result.termination)
        << " duration_s=" << result.trajectory.duration_s()
        << " planning_ms=" << result.planning_time_ms;
    EXPECT_LT(result.trajectory.duration_s(), 20.0);
    EXPECT_TRUE(result.has_finite_output());

    for (std::size_t index = 0U; index <= 100U; ++index)
    {
        const double time_s = result.trajectory.duration_s() *
            static_cast<double>(index) / 100.0;
        const auto state = result.trajectory.evaluate_state(time_s);
        ASSERT_TRUE(state.is_valid);
        EXPECT_LE(std::sqrt(taiphoon::ego_swarm::squared_norm(state.velocity)),
            config.max_velocity_mps + 1.0e-6);
        EXPECT_LE(std::sqrt(
                taiphoon::ego_swarm::squared_norm(state.acceleration)),
            config.max_acceleration_mps2 + 1.0e-6);
        EXPECT_LE(std::sqrt(taiphoon::ego_swarm::squared_norm(state.jerk)),
            config.max_jerk_mps3 + 1.0e-6);
    }
}

TEST(OptimizedPlannerTest, ClearSeedSkipsSolverWithNonEmptyFarObstacleMap)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.update({Vector3{100.0, 100.0, 0.0}}, 0.0));
    PlanningSnapshot snapshot = make_snapshot(map);
    PlannerConfig config;
    config.optimizer_time_budget_ms = 20.0;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    ASSERT_TRUE(result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            result.termination);
    EXPECT_EQ(result.termination, PlannerTerminationReason::SUCCESS);
    EXPECT_LT(result.planning_time_ms, config.optimizer_time_budget_ms);
}

TEST(OptimizedPlannerTest, LongMoveWithPeerOccupancyFitsDefaultBudget)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 1;
    snapshot.local_goal = Vector3{0.0, 40.0, 0.0};
    snapshot.peers.push_back(
        make_terminal_hold_peer(2, Vector3{10.0, 0.0, 0.0}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(3, Vector3{-10.0, 0.0, 0.0}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(4, Vector3{0.0, -10.0, 0.0}));

    PlannerConfig config;
    config.optimizer_time_budget_ms = 20.0;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    ASSERT_TRUE(result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            result.termination)
        << " iterations=" << result.iteration_count
        << " planning_ms=" << result.planning_time_ms
        << " obstacle_cost=" << result.obstacle_cost
        << " peer_cost=" << result.peer_cost
        << " min_peer=" << result.minimum_peer_distance_m;
    EXPECT_LT(result.planning_time_ms, config.optimizer_time_budget_ms * 1.5);
}

TEST(OptimizedPlannerTest, HorizontalMoveAroundTerminalPeerLineStaysSafe)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 1;
    snapshot.self_state.position = Vector3{0.0, 0.0, 0.0};
    snapshot.local_goal = Vector3{0.0, 20.0, 0.0};
    snapshot.path_constraint = PathConstraint::FIXED_Z;
    snapshot.fixed_z_m = 0.0;
    snapshot.peers.push_back(
        make_terminal_hold_peer(2, Vector3{0.0, 8.0, 0.0}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(3, Vector3{0.0, 16.0, 0.0}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(4, Vector3{0.0, 24.0, 0.0}));

    PlannerConfig config;
    config.optimizer_time_budget_ms = 20.0;
    config.optimizer_max_iterations = 40;
    config.peer_weight = 100.0;
    config.smoothness_weight = 0.01;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    ASSERT_TRUE(result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            result.termination)
        << " iterations=" << result.iteration_count
        << " planning_ms=" << result.planning_time_ms
        << " min_peer=" << result.minimum_peer_distance_m;
    EXPECT_GE(result.minimum_peer_distance_m,
              config.peer_collision_min_distance_m);
    bool has_lateral_departure = false;
    const auto& control_points = result.trajectory.control_points();
    for (std::size_t index = 0U; index < control_points.size(); ++index)
    {
        const double ratio = static_cast<double>(index) /
            static_cast<double>(control_points.size() - 1U);
        const double direct_x = snapshot.self_state.position.at(0) +
            ratio * (snapshot.local_goal.at(0) -
            snapshot.self_state.position.at(0));
        if (std::abs(control_points.at(index).at(0) - direct_x) > 2.0)
        {
            has_lateral_departure = true;
        }
        EXPECT_DOUBLE_EQ(control_points.at(index).at(2), snapshot.fixed_z_m);
    }
    EXPECT_TRUE(has_lateral_departure);
}

TEST(OptimizedPlannerTest, HorizontalMoveWithYawResidualStillFindsSidePath)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 1;
    snapshot.self_state.position = Vector3{0.423, 0.0, -7.834};
    snapshot.local_goal = Vector3{0.896, 19.994, -7.834};
    snapshot.path_constraint = PathConstraint::FIXED_Z;
    snapshot.fixed_z_m = -7.834;
    snapshot.peers.push_back(
        make_terminal_hold_peer(2, Vector3{0.0, 7.699, -7.834}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(3, Vector3{0.0, 15.399, -7.834}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(4, Vector3{0.0, 23.868, -7.834}));

    PlannerConfig config;
    config.optimizer_time_budget_ms = 20.0;
    config.optimizer_max_iterations = 40;
    config.peer_weight = 100.0;
    config.smoothness_weight = 0.01;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    ASSERT_TRUE(result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            result.termination)
        << " iterations=" << result.iteration_count
        << " planning_ms=" << result.planning_time_ms
        << " min_peer=" << result.minimum_peer_distance_m;
    EXPECT_GE(result.minimum_peer_distance_m,
              config.peer_collision_min_distance_m);
    for (const auto& point : result.trajectory.control_points())
    {
        EXPECT_DOUBLE_EQ(point.at(2), snapshot.fixed_z_m);
    }
}

TEST(OptimizedPlannerTest, HorizontalMoveNearPeerUsesEarlySideClearance)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 1;
    snapshot.self_state.position = Vector3{-0.423, 2.310, -6.573};
    snapshot.local_goal = Vector3{-0.299, 22.309, -6.573};
    snapshot.path_constraint = PathConstraint::FIXED_Z;
    snapshot.fixed_z_m = -6.573;
    snapshot.peers.push_back(
        make_terminal_hold_peer(2, Vector3{0.211, 6.160, -7.783}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(3, Vector3{-0.423, 18.479, -6.640}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(4, Vector3{-1.268, 25.408, -7.125}));

    PlannerConfig config;
    config.optimizer_time_budget_ms = 20.0;
    config.optimizer_max_iterations = 40;
    config.peer_weight = 100.0;
    config.smoothness_weight = 0.01;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    ASSERT_TRUE(result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            result.termination)
        << " iterations=" << result.iteration_count
        << " planning_ms=" << result.planning_time_ms
        << " min_peer=" << result.minimum_peer_distance_m;
    EXPECT_GE(result.minimum_peer_distance_m,
              config.peer_collision_min_distance_m);
    for (const auto& point : result.trajectory.control_points())
    {
        EXPECT_DOUBLE_EQ(point.at(2), snapshot.fixed_z_m);
    }
}

TEST(OptimizedPlannerTest, LongHorizontalMoveAroundEastLineStaysFeasible)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 1;
    snapshot.self_state.position = Vector3{0.0, 0.0, 0.0};
    snapshot.local_goal = Vector3{0.0, 30.0, 0.0};
    snapshot.path_constraint = PathConstraint::FIXED_Z;
    snapshot.fixed_z_m = 0.0;
    // 以 SITL 機群避障功能1 實測的相對幾何建立長距離 crossing case：
    // UAV1 朝東移動，其他 UAV 在東向路徑上形成 terminal occupancy。
    snapshot.peers.push_back(
        make_terminal_hold_peer(2, Vector3{-3.2, 4.0, 0.0}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(3, Vector3{-0.8, 8.2, 0.0}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(4, Vector3{-2.4, 21.4, 0.0}));

    PlannerConfig config;
    config.optimizer_time_budget_ms = 20.0;
    config.optimizer_max_iterations = 40;
    config.peer_weight = 100.0;
    config.smoothness_weight = 0.01;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    ASSERT_TRUE(result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            result.termination)
        << " iterations=" << result.iteration_count
        << " planning_ms=" << result.planning_time_ms
        << " min_peer=" << result.minimum_peer_distance_m;
    EXPECT_GE(result.minimum_peer_distance_m,
              config.peer_collision_min_distance_m);
    EXPECT_GE(result.minimum_peer_distance_m,
              config.peer_collision_min_distance_m +
                  config.optimizer_peer_soft_margin_m - 1.0e-6);
    EXPECT_LT(result.trajectory.duration_s(), 20.0);
    for (const auto& point : result.trajectory.control_points())
    {
        EXPECT_DOUBLE_EQ(point.at(2), snapshot.fixed_z_m);
    }
}

TEST(OptimizedPlannerTest, LongHorizontalMoveMatchesSitlEastLineSnapshot)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 1;
    snapshot.self_state.position = Vector3{0.0, 0.0, 0.0};
    snapshot.local_goal = Vector3{0.0, 30.0, 0.0};
    snapshot.path_constraint = PathConstraint::FIXED_Z;
    snapshot.fixed_z_m = 0.0;
    // 取自 4 機 SITL 的西端 UAV 與東向一字列相對位置。
    snapshot.peers.push_back(
        make_terminal_hold_peer(2, Vector3{0.0, 6.8, 0.0}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(3, Vector3{0.0, 14.3, 0.0}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(4, Vector3{-1.0, 23.0, 0.0}));

    PlannerConfig config;
    config.optimizer_time_budget_ms = 20.0;
    config.optimizer_max_iterations = 40;
    config.peer_weight = 100.0;
    config.smoothness_weight = 0.01;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    ASSERT_TRUE(result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            result.termination)
        << " iterations=" << result.iteration_count
        << " planning_ms=" << result.planning_time_ms
        << " min_peer=" << result.minimum_peer_distance_m;
    EXPECT_GE(result.minimum_peer_distance_m,
              config.peer_collision_min_distance_m +
                  config.optimizer_peer_soft_margin_m - 1.0e-6);
    EXPECT_LT(result.trajectory.duration_s(), 20.0);
    for (const auto& point : result.trajectory.control_points())
    {
        EXPECT_DOUBLE_EQ(point.at(2), snapshot.fixed_z_m);
    }
}

TEST(OptimizedPlannerTest, ReverseLongHorizontalMoveMatchesSitlEastLineSnapshot)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 1;
    snapshot.self_state.position = Vector3{0.0, 30.0, 0.0};
    snapshot.local_goal = Vector3{0.0, 0.0, 0.0};
    snapshot.path_constraint = PathConstraint::FIXED_Z;
    snapshot.fixed_z_m = 0.0;
    snapshot.peers.push_back(
        make_terminal_hold_peer(2, Vector3{0.0, 6.8, 0.0}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(3, Vector3{0.0, 14.3, 0.0}));
    snapshot.peers.push_back(
        make_terminal_hold_peer(4, Vector3{-1.0, 23.0, 0.0}));

    PlannerConfig config;
    config.optimizer_time_budget_ms = 20.0;
    config.optimizer_max_iterations = 40;
    config.peer_weight = 100.0;
    config.smoothness_weight = 0.01;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    ASSERT_TRUE(result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            result.termination)
        << " iterations=" << result.iteration_count
        << " planning_ms=" << result.planning_time_ms
        << " min_peer=" << result.minimum_peer_distance_m;
    EXPECT_GE(result.minimum_peer_distance_m,
              config.peer_collision_min_distance_m +
                  config.optimizer_peer_soft_margin_m - 1.0e-6);
    EXPECT_LT(result.trajectory.duration_s(), 20.0);
    for (const auto& point : result.trajectory.control_points())
    {
        EXPECT_DOUBLE_EQ(point.at(2), snapshot.fixed_z_m);
    }
}

TEST(OptimizedPlannerTest, AltitudeProjectionHonorsFixedAndLinearPaths)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_state.position = Vector3{0.0, 0.0, 5.0};
    snapshot.local_goal = Vector3{8.0, 0.0, 5.0};
    snapshot.fixed_z_m = 5.0;
    PlannerConfig config;
    config.optimizer_time_budget_ms = 20.0;
    OptimizedEgoPlanner planner(config);

    snapshot.path_constraint = PathConstraint::FIXED_Z;
    const auto fixed_result = planner.plan(snapshot);
    ASSERT_TRUE(fixed_result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            fixed_result.termination)
        << " iterations=" << fixed_result.iteration_count
        << " planning_ms=" << fixed_result.planning_time_ms;
    for (const auto& point : fixed_result.trajectory.control_points())
    {
        EXPECT_DOUBLE_EQ(point.at(2), 5.0);
    }

    snapshot.self_state.position = Vector3{0.0, 0.0, 0.0};
    snapshot.local_goal = Vector3{8.0, 0.0, 4.0};
    snapshot.path_constraint = PathConstraint::LINEAR_Z;
    const auto linear_result = planner.plan(snapshot);
    ASSERT_TRUE(linear_result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            linear_result.termination);
    const auto& points = linear_result.trajectory.control_points();
    for (std::size_t index = 0U; index < points.size(); ++index)
    {
        const double ratio = static_cast<double>(index) /
            static_cast<double>(points.size() - 1U);
        EXPECT_NEAR(points.at(index).at(2), 4.0 * ratio, 1.0e-9);
    }
}

TEST(OptimizedSafetyTest, HorizontalEmergencyBrakeKeepsFixedAltitude)
{
    PlannerConfig config;
    EmergencyBrakePlanner planner(config);
    taiphoon::ego_swarm::VehiclePlanningState state;
    state.position = Vector3{0.0, 0.0, 10.0};
    state.velocity = Vector3{1.0, 0.0, 1.0};
    state.acceleration = Vector3{0.0, 0.0, 0.0};
    state.timestamp_s = 0.0;

    const auto result = planner.plan(state, PathConstraint::FIXED_Z);
    ASSERT_TRUE(result.is_valid);
    for (const auto& point : result.trajectory.control_points())
    {
        EXPECT_DOUBLE_EQ(point.at(2), 10.0);
    }
}

TEST(OptimizedPlannerTest, PointObstacleProducesAFeasibleLateralOffset)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.update({Vector3{4.0, 0.5, 0.0}}, 0.0));
    PlanningSnapshot snapshot = make_snapshot(map);
    PlannerConfig config;
    config.optimizer_time_budget_ms = 2000.0;
    config.obstacle_clearance_m = 1.0;
    config.smoothness_weight = 0.0;
    config.obstacle_weight = 1000.0;
    config.optimizer_max_iterations = 200;
    config.optimizer_initial_step_m = 0.05;
    config.optimizer_minimum_step_m = 1.0e-5;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    ASSERT_TRUE(result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            result.termination)
        << " iterations=" << result.iteration_count
        << " min_obstacle=" << result.minimum_obstacle_distance_m
        << " obstacle_cost=" << result.obstacle_cost;
    EXPECT_GE(result.minimum_obstacle_distance_m,
              config.obstacle_clearance_m);
    bool has_lateral_offset = false;
    for (const auto& point : result.trajectory.control_points())
    {
        if (std::abs(point.at(1)) > 0.05)
        {
            has_lateral_offset = true;
        }
    }
    EXPECT_TRUE(has_lateral_offset);
}

TEST(OptimizedPlannerTest, DenseWallUsesExpandedHorizontalSearchEnvelope)
{
    RawPointObstacleMap map;
    std::vector<Vector3> wall_points;
    for (int index = 0; index <= 8; ++index)
    {
        wall_points.push_back(
            Vector3{4.0, static_cast<double>(index) * 0.125, 0.0});
    }
    ASSERT_TRUE(map.update(wall_points, 0.0));
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.local_goal = Vector3{8.0, 0.0, 0.0};
    snapshot.path_constraint = PathConstraint::FIXED_Z;
    snapshot.fixed_z_m = 0.0;
    PlannerConfig config;
    config.optimizer_time_budget_ms = 5000.0;
    config.optimizer_max_iterations = 1000;
    // 此案例專注驗證 dense obstacle 的水平搜尋 envelope；動力學限制
    // 由其他 planner／safety 測試覆蓋，避免將幾何繞行與極端轉彎混在一起。
    config.smoothness_weight = 0.0;
    config.obstacle_weight = 1000.0;
    config.dynamic_weight = 0.0;
    config.max_velocity_mps = 100.0;
    config.max_acceleration_mps2 = 100.0;
    config.max_jerk_mps3 = 100.0;
    config.optimizer_initial_step_m = 0.50;
    config.optimizer_minimum_step_m = 1.0e-5;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    ASSERT_TRUE(result.is_valid)
        << taiphoon::ego_swarm::planner_termination_reason_string(
            result.termination)
        << " iterations=" << result.iteration_count
        << " min_obstacle=" << result.minimum_obstacle_distance_m;
    EXPECT_GE(result.minimum_obstacle_distance_m,
              config.obstacle_clearance_m);
    bool has_wall_clearance_offset = false;
    for (const auto& point : result.trajectory.control_points())
    {
        if (std::abs(point.at(1)) > 0.05)
        {
            has_wall_clearance_offset = true;
        }
    }
    EXPECT_TRUE(has_wall_clearance_offset);
}

TEST(OptimizedPlannerTest, TerminalHoldIsNeverIgnoredByPriority)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 1;
    snapshot.peers.push_back(
        make_terminal_hold_peer(2, Vector3{4.0, 0.5, 0.0}));
    PlannerConfig config;
    config.smoothness_weight = 0.0;
    config.peer_weight = 100.0;
    config.optimizer_max_iterations = 100;
    config.optimizer_time_budget_ms = 100.0;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    EXPECT_TRUE(!result.is_valid ||
                result.minimum_peer_distance_m >=
                    config.peer_collision_min_distance_m);
}

TEST(OptimizedPlannerTest, TerminalHoldRemainsHardOccupancyAfterShortHeartbeat)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 1;
    // The terminal hold's short B-spline heartbeat has already ended in
    // absolute time, but the record is still fresh and must occupy its
    // endpoint until a newer trajectory replaces it.
    snapshot.trajectory_start_time_s = 5.0;
    snapshot.local_goal = Vector3{8.0, 0.0, 0.0};
    snapshot.peers.push_back(
        make_terminal_hold_peer(2, Vector3{4.0, 0.0, 0.0}));

    PlannerConfig config;
    config.smoothness_weight = 0.0;
    config.peer_weight = 100.0;
    config.optimizer_max_iterations = 80;
    config.optimizer_time_budget_ms = 100.0;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    EXPECT_TRUE(!result.is_valid ||
                result.minimum_peer_distance_m >=
                    config.peer_collision_min_distance_m);
}

TEST(OptimizedPlannerTest, OptimizerCanUseExpandedHorizontalSearchEnvelope)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 2;
    snapshot.peers.push_back(
        make_terminal_hold_peer(1, Vector3{4.0, 0.5, 0.0}));
    PlannerConfig config;
    config.smoothness_weight = 0.0;
    config.peer_weight = 100.0;
    config.optimizer_max_iterations = 200;
    config.optimizer_time_budget_ms = 500.0;
    OptimizedEgoPlanner planner(config);

    const auto result = planner.plan(snapshot);
    EXPECT_TRUE(!result.is_valid ||
                result.minimum_peer_distance_m >=
                    config.peer_collision_min_distance_m);
    if (result.is_valid)
    {
        bool has_large_lateral_offset = false;
        for (const auto& point : result.trajectory.control_points())
        {
            if (std::abs(point.at(1)) > config.candidate_offset_m + 0.1)
            {
                has_large_lateral_offset = true;
            }
        }
        EXPECT_TRUE(has_large_lateral_offset);
    }
}

TEST(OptimizedReservationTest, TerminalHoldCannotBeBypassedByPriority)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    snapshot.self_agent_id = 1;
    snapshot.peers.push_back(
        make_terminal_hold_peer(2, Vector3{4.0, 0.0, 0.0}));
    const BsplineTrajectory candidate = make_peer_path(
        Vector3{0.0, 0.0, 0.0}, Vector3{8.0, 0.0, 0.0});

    const auto decision = FormationReservationPlanner::reserve(
        candidate, snapshot, 3.0, 0.5, 0.5, 5.0);
    EXPECT_FALSE(decision.is_reserved);
}

TEST(OptimizedSafetyTest, TerminalHoldCannotBeBypassedByPriority)
{
    RawPointObstacleMap map = make_empty_map();
    const PeerTrajectory peer = make_terminal_hold_peer(
        2, Vector3{4.0, 0.0, 0.0});
    PeerTrajectoryTable peer_table;
    ASSERT_TRUE(peer_table.update(peer));

    PlannerConfig config;
    config.peer_collision_min_distance_m = 3.0;
    config.max_velocity_mps = 20.0;
    config.max_acceleration_mps2 = 20.0;
    config.max_jerk_mps3 = 20.0;
    ContinuousSafetyValidator validator(config);
    SafetyValidationContext context;
    context.self_state.position = Vector3{0.0, 0.0, 0.0};
    context.self_state.timestamp_s = 0.0;
    context.now_s = 0.0;
    context.trajectory_start_time_s = 0.5;
    context.max_time_skew_s = 1.0;
    context.max_future_start_delay_s = 2.0;
    context.frame_revision = 1U;
    context.trajectory_frame_revision = 1U;
    context.self_agent_id = 1;
    context.peer_table = &peer_table;
    context.obstacle_map = &map;

    const auto result = validator.validate(
        make_peer_path(Vector3{0.0, 0.0, 0.0}, Vector3{8.0, 0.0, 0.0}),
        context);
    EXPECT_FALSE(result.is_safe);
    EXPECT_EQ(result.reason, SafetyFailureReason::PEER_COLLISION);
}

TEST(OptimizedPlannerTest, OptimizedFailureHasNoExecutableFallback)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.update({Vector3{4.0, 0.0, 0.0}}, 0.0));
    PlanningSnapshot snapshot = make_snapshot(map);
    PlannerConfig config;
    config.optimizer_time_budget_ms = 0.001;
    const OptimizedEgoPlanner optimized(config);

    const auto primary = optimized.plan(snapshot);
    EXPECT_FALSE(primary.is_valid);
    EXPECT_EQ(primary.termination, PlannerTerminationReason::TIMEOUT);
    EXPECT_EQ(primary.backend,
              taiphoon::ego_swarm::PlannerBackendKind::OPTIMIZED);
    EXPECT_FALSE(primary.trajectory.is_valid());
}

TEST(OptimizedReservationTest, ConflictingPathsAreTimeSeparated)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    taiphoon::ego_swarm::PeerTrajectory peer;
    peer.agent_id = 1;
    peer.trajectory_id = 1U;
    peer.frame_revision = 1U;
    peer.message_time_s = 0.0;
    peer.start_time_s = 0.5;
    peer.receive_time_s = 0.0;
    peer.trajectory = make_peer_path(
        Vector3{4.0, -4.0, 0.0}, Vector3{4.0, 4.0, 0.0});
    peer.end_time_s = peer.start_time_s + peer.trajectory.duration_s();
    snapshot.peers.push_back(peer);
    const BsplineTrajectory candidate = make_peer_path(
        Vector3{0.0, -4.0, 0.0}, Vector3{8.0, 4.0, 0.0});

    const auto decision = FormationReservationPlanner::reserve(
        candidate, snapshot, 3.0, 0.5, 0.5, 5.0);
    ASSERT_TRUE(decision.is_reserved);
    EXPECT_GT(decision.start_time_s, snapshot.trajectory_start_time_s);
}

TEST(OptimizedReservationTest, ReservationDoesNotModifyCandidateGeometry)
{
    RawPointObstacleMap map = make_empty_map();
    PlanningSnapshot snapshot = make_snapshot(map);
    const BsplineTrajectory candidate = make_peer_path(
        Vector3{0.0, 0.0, 0.0}, Vector3{8.0, 2.0, 0.0});
    const auto original_control_points = candidate.control_points();

    const auto decision = FormationReservationPlanner::reserve(
        candidate, snapshot, 3.0, 0.5, 0.5, 5.0);

    ASSERT_TRUE(decision.is_reserved);
    EXPECT_EQ(candidate.control_points(), original_control_points);
}

TEST(OptimizedConvergenceTest, UsesPositionAndDataValidityWithoutVelocityGate)
{
    std::vector<FormationConvergenceObservation> observations(2U);
    for (std::size_t index = 0U; index < observations.size(); ++index)
    {
        observations.at(index).agent_id = static_cast<int>(index + 1U);
        observations.at(index).position = Vector3{0.0, 0.0, 0.0};
        observations.at(index).target = Vector3{0.0, 0.0, 0.0};
        observations.at(index).velocity = Vector3{5.0, -3.0, 2.0};
        observations.at(index).has_fresh_state = true;
        observations.at(index).has_fresh_occupancy = true;
        observations.at(index).has_same_epoch = true;
    }
    EXPECT_TRUE(FormationConvergenceEvaluator::evaluate(
        observations, 1.0).is_converged);
    observations.at(1U).has_fresh_occupancy = false;
    EXPECT_FALSE(FormationConvergenceEvaluator::evaluate(
        observations, 1.0).is_converged);
}

}  // namespace
