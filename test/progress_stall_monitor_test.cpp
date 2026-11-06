/**
 * @file progress_stall_monitor_test.cpp
 * @brief 依進度重新規劃的停滯偵測單元測試
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "ego_swarm/Geofence.hpp"
#include "ego_swarm/OptimizedEgoPlanner.hpp"
#include "ego_swarm/ProgressStallMonitor.hpp"
#include "ego_swarm/ReplanTrigger.hpp"
#include "perception/RawPointObstacleMap.hpp"

namespace
{

using taiphoon::ego_swarm::BsplineTrajectory;
using taiphoon::ego_swarm::Geofence;
using taiphoon::ego_swarm::OptimizedEgoPlanner;
using taiphoon::ego_swarm::PathConstraint;
using taiphoon::ego_swarm::PeerTrajectory;
using taiphoon::ego_swarm::PlannerConfig;
using taiphoon::ego_swarm::MIN_CANDIDATE_PATH_PROGRESS_RATIO;
using taiphoon::ego_swarm::PlannerResult;
using taiphoon::ego_swarm::PlanningSnapshot;
using taiphoon::ego_swarm::ProgressStallContext;
using taiphoon::ego_swarm::ProgressStallMonitor;
using taiphoon::ego_swarm::ReplanTriggerInput;
using taiphoon::ego_swarm::ReplanTriggerReason;
using taiphoon::ego_swarm::candidate_lacks_path_progress;
using taiphoon::ego_swarm::evaluate_replan_trigger;
using taiphoon::ego_swarm::progress_stall_applies;
using taiphoon::ego_swarm::Vector3;
using taiphoon::ego_swarm::trajectory_path_progress_ratio;
using taiphoon::perception::RawPointObstacleMap;

constexpr double FIELD_HALF_M = 4.0;
constexpr double FIELD_MARGIN_M = 0.5;
constexpr double FIELD_BEARING_DEG = 143.0;
/// launch 預設的 local_goal_replan_ratio。
constexpr double REPLAN_RATIO = 0.30;
/// 測試正確性時使用的寬鬆牆鐘預算（ms），避免結果隨 CPU 負載變動。
constexpr double DETERMINISTIC_TIME_BUDGET_MS = 5000.0;

Vector3 field_to_ned(const double u, const double v)
{
    const double bearing_rad = FIELD_BEARING_DEG * M_PI / 180.0;
    return Vector3{
        u * std::cos(bearing_rad) - v * std::sin(bearing_rad),
        u * std::sin(bearing_rad) + v * std::cos(bearing_rad),
        -2.0};
}

Geofence make_field_fence()
{
    Geofence fence;
    EXPECT_TRUE(fence.set_polygon(
        {field_to_ned(-FIELD_HALF_M, -FIELD_HALF_M),
         field_to_ned(FIELD_HALF_M, -FIELD_HALF_M),
         field_to_ned(FIELD_HALF_M, FIELD_HALF_M),
         field_to_ned(-FIELD_HALF_M, FIELD_HALF_M)},
        FIELD_MARGIN_M));
    return fence;
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

struct PlanCase
{
    const char* name;
    Vector3 start;
    Vector3 goal;
    std::vector<PeerTrajectory> peers;
    bool fenced;
    double separation_m;
};

PlannerResult plan_case(
    const PlanCase& item,
    const RawPointObstacleMap& map,
    const Geofence& fence)
{
    PlanningSnapshot snapshot;
    snapshot.self_state.position = item.start;
    snapshot.self_state.timestamp_s = 0.0;
    snapshot.local_goal = item.goal;
    snapshot.planning_time_s = 0.0;
    snapshot.trajectory_start_time_s = 0.5;
    snapshot.frame_revision = 1U;
    snapshot.self_agent_id = 3;
    snapshot.path_constraint = PathConstraint::FIXED_Z;
    snapshot.fixed_z_m = -2.0;
    snapshot.obstacle_map = &map;
    snapshot.geofence = item.fenced ? &fence : nullptr;
    snapshot.peers_are_fresh = true;
    snapshot.peers = item.peers;
    PlannerConfig config;
    config.optimizer_time_budget_ms = DETERMINISTIC_TIME_BUDGET_MS;
    config.peer_collision_min_distance_m = item.separation_m;
    config.enable_spatiotemporal_route_planning = true;
    OptimizedEgoPlanner planner(config);
    return planner.plan(snapshot);
}

ProgressStallContext make_eligible_context()
{
    ProgressStallContext context;
    context.has_active_plan = true;
    context.active_plan_running = true;
    context.active_target_is_final = true;
    return context;
}

TEST(ProgressStallMonitorTest, ConstantDistanceStallsAfterWindow)
{
    ProgressStallMonitor monitor;
    EXPECT_FALSE(monitor.update(10.0, 2.0));
    EXPECT_FALSE(monitor.update(11.0, 1.95));
    EXPECT_FALSE(monitor.update(12.9, 1.9));
    EXPECT_TRUE(monitor.update(13.0, 1.85));
    EXPECT_TRUE(monitor.is_stalled());
}

TEST(ProgressStallMonitorTest, ImprovementOfMinimumDistanceRestartsWindow)
{
    ProgressStallMonitor monitor;
    EXPECT_FALSE(monitor.update(0.0, 2.0));
    EXPECT_FALSE(monitor.update(2.5, 1.8));
    EXPECT_FALSE(monitor.update(5.0, 1.75));
    EXPECT_TRUE(monitor.update(5.5, 1.75));
    // 停滯後再有足夠進展即解除。
    EXPECT_FALSE(monitor.update(6.0, 1.5));
    EXPECT_FALSE(monitor.is_stalled());
}

TEST(ProgressStallMonitorTest, SlowSteadyApproachIsNotStalled)
{
    ProgressStallMonitor monitor;
    for (int step = 0; step <= 200; ++step)
    {
        const double now_s = 0.1 * static_cast<double>(step);
        // 0.1 m/s：每 2 s 改善 0.2 m，時間窗內一定有進展。
        EXPECT_FALSE(monitor.update(now_s, 5.0 - 0.1 * now_s)) << now_s;
    }
}

TEST(ProgressStallMonitorTest, JitterAroundFixedPointStillStalls)
{
    ProgressStallMonitor monitor;
    bool stalled = false;
    for (int step = 0; step <= 100; ++step)
    {
        const double now_s = 0.1 * static_cast<double>(step);
        // 8×8 m 交叉停滯時實測的 ±0.3 m 位置抖動。
        const double distance_m = 1.7 + 0.3 * std::sin(2.0 * M_PI * now_s);
        stalled = monitor.update(now_s, distance_m);
    }
    EXPECT_TRUE(stalled);
}

TEST(ProgressStallMonitorTest, InvalidInputAndTimeReversalRestart)
{
    ProgressStallMonitor monitor;
    EXPECT_FALSE(monitor.update(0.0, 2.0));
    EXPECT_TRUE(monitor.update(3.5, 2.0));
    EXPECT_FALSE(monitor.update(1.0, 2.0));
    EXPECT_FALSE(monitor.update(3.9, 2.0));
    EXPECT_TRUE(monitor.update(4.0, 2.0));
    EXPECT_FALSE(monitor.update(4.1, std::nan("")));
    EXPECT_FALSE(monitor.update(4.2, -1.0));
    EXPECT_FALSE(monitor.update(4.3, 2.0));
}

TEST(ProgressStallMonitorTest, ExcludedStatesAndClearingConditions)
{
    EXPECT_TRUE(progress_stall_applies(make_eligible_context()));
    const std::vector<void (*)(ProgressStallContext&)> exclusions{
        [](ProgressStallContext& c) { c.has_active_plan = false; },
        // segment 已結束或 Final Target 改變。
        [](ProgressStallContext& c) { c.active_plan_running = false; },
        // terminal hold、waiting hold。
        [](ProgressStallContext& c) { c.active_plan_is_hold = true; },
        [](ProgressStallContext& c) { c.emergency_brake_active = true; },
        // local route arbitration 與規劃失敗後的等待。
        [](ProgressStallContext& c) { c.waiting_for_replan = true; },
        // 終點只是中途 Local Goal。
        [](ProgressStallContext& c) { c.active_target_is_final = false; }};
    for (std::size_t index = 0U; index < exclusions.size(); ++index)
    {
        ProgressStallContext context = make_eligible_context();
        exclusions.at(index)(context);
        EXPECT_FALSE(progress_stall_applies(context)) << index;

        // 停滯後遇到排除狀態必須清除，回到適用狀態時重新計時。
        ProgressStallMonitor monitor;
        EXPECT_FALSE(monitor.track(make_eligible_context(), 1U, 0.0, 2.0));
        EXPECT_TRUE(monitor.track(make_eligible_context(), 1U, 3.0, 2.0));
        EXPECT_FALSE(monitor.track(context, 1U, 3.1, 2.0)) << index;
        EXPECT_FALSE(monitor.is_stalled()) << index;
        EXPECT_FALSE(monitor.track(make_eligible_context(), 1U, 3.2, 2.0));
        EXPECT_FALSE(monitor.track(make_eligible_context(), 1U, 6.1, 2.0));
        EXPECT_TRUE(monitor.track(make_eligible_context(), 1U, 6.2, 2.0));
    }
}

TEST(ProgressStallMonitorTest, FrameRevisionChangeAndResetClear)
{
    ProgressStallMonitor monitor;
    EXPECT_FALSE(monitor.track(make_eligible_context(), 1U, 0.0, 2.0));
    EXPECT_TRUE(monitor.track(make_eligible_context(), 1U, 3.0, 2.0));
    EXPECT_FALSE(monitor.track(make_eligible_context(), 2U, 3.2, 2.0));
    EXPECT_TRUE(monitor.track(make_eligible_context(), 2U, 6.2, 2.0));
    // 軌跡被作廢、到達目標等由 Node 呼叫 reset。
    monitor.reset();
    EXPECT_FALSE(monitor.is_stalled());
    EXPECT_FALSE(monitor.track(make_eligible_context(), 2U, 6.3, 2.0));
}

TEST(ReplanTriggerStallTest, StallOnlyCancelsProgressReplan)
{
    ReplanTriggerInput input{
        true, true, true, true, 7.0, 10.0, 8.0, true, 7.0, 0.30, 0.5, true};
    auto decision = evaluate_replan_trigger(input);
    EXPECT_FALSE(decision.should_replan);
    EXPECT_EQ(decision.reason, ReplanTriggerReason::PROGRESS_STALLED);

    input.progress_stalled = false;
    decision = evaluate_replan_trigger(input);
    EXPECT_TRUE(decision.should_replan);
    EXPECT_EQ(decision.reason, ReplanTriggerReason::SEGMENT_PROGRESS);

    // 安全、Final Target 與 segment 結束的觸發不受停滯影響。
    input.progress_stalled = true;
    input.active_trajectory_safe = false;
    EXPECT_EQ(evaluate_replan_trigger(input).reason,
              ReplanTriggerReason::ACTIVE_TRAJECTORY_UNSAFE);
    input.active_trajectory_safe = true;
    input.final_target_unchanged = false;
    EXPECT_EQ(evaluate_replan_trigger(input).reason,
              ReplanTriggerReason::FINAL_TARGET_CHANGED);
    input.final_target_unchanged = true;
    input.elapsed_s = 10.0;
    EXPECT_EQ(evaluate_replan_trigger(input).reason,
              ReplanTriggerReason::SEGMENT_ENDED);
}

TEST(CandidatePathProgressTest, StraightTrajectoryHasProgress)
{
    const BsplineTrajectory line(
        {Vector3{0.0, 0.0, -2.0}, Vector3{1.0, 0.0, -2.0},
         Vector3{2.0, 0.0, -2.0}, Vector3{3.0, 0.0, -2.0},
         Vector3{4.0, 0.0, -2.0}, Vector3{5.0, 0.0, -2.0}},
        1.0);
    double ratio = 0.0;
    ASSERT_TRUE(trajectory_path_progress_ratio(line, REPLAN_RATIO, ratio));
    EXPECT_GT(ratio, MIN_CANDIDATE_PATH_PROGRESS_RATIO);
    EXPECT_FALSE(candidate_lacks_path_progress(line, REPLAN_RATIO));
    ASSERT_TRUE(trajectory_path_progress_ratio(line, 1.0, ratio));
    EXPECT_NEAR(ratio, 1.0, 1.0e-9);
}

TEST(CandidatePathProgressTest, StationaryOrInvalidInputDoesNotBlock)
{
    const Vector3 point{1.0, 2.0, -2.0};
    const BsplineTrajectory hold({point, point, point, point}, 1.0);
    double ratio = 0.0;
    EXPECT_FALSE(trajectory_path_progress_ratio(hold, REPLAN_RATIO, ratio));
    EXPECT_FALSE(candidate_lacks_path_progress(hold, REPLAN_RATIO));
    EXPECT_FALSE(trajectory_path_progress_ratio(hold, -0.1, ratio));
    EXPECT_FALSE(trajectory_path_progress_ratio(hold, 1.1, ratio));
    EXPECT_FALSE(trajectory_path_progress_ratio(BsplineTrajectory{}, 0.3, ratio));
}

/// 8×8 m 交叉測試中造成停滯的軌跡：目標在圍籬軟性緩衝帶內，最佳化器把前段
/// 壓在起點附近。第一層判斷必須認出它，正常的直線、繞行與窄通道則不可誤判。
TEST(CandidatePathProgressTest, SeparatesStalledFromNormalPlannerTrajectories)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.set_explicit_empty(0.0));
    const Geofence fence = make_field_fence();
    const std::vector<PlanCase> stalled{
        {"stall_soft_band", field_to_ned(-2.95, -1.5), field_to_ned(-3.20, 0.05),
         {make_terminal_hold_peer(1, field_to_ned(3.02, 0.15)),
          make_terminal_hold_peer(2, field_to_ned(0.13, -0.13))}, true, 1.5},
        {"stall_soft_band_wide", field_to_ned(-2.95, -2.2),
         field_to_ned(-3.20, 0.05),
         {make_terminal_hold_peer(1, field_to_ned(3.02, 0.15)),
          make_terminal_hold_peer(2, field_to_ned(0.13, -0.13))}, true, 1.5}};
    const std::vector<PlanCase> normal{
        {"open_straight_5m", Vector3{0.0, 0.0, -2.0}, Vector3{5.0, 0.0, -2.0},
         {}, false, 2.0},
        {"open_straight_1m", Vector3{0.0, 0.0, -2.0}, Vector3{1.0, 0.0, -2.0},
         {}, false, 2.0},
        {"crossing_start_1_5m", field_to_ned(3.02, 0.02),
         field_to_ned(-3.20, 0.02),
         {make_terminal_hold_peer(2, field_to_ned(0.13, -0.15))}, true, 1.5},
        {"crossing_start_2_0m", field_to_ned(3.02, 0.02),
         field_to_ned(-3.20, 0.02),
         {make_terminal_hold_peer(2, field_to_ned(0.13, -0.15))}, true, 2.0},
        {"narrow_corridor", field_to_ned(2.65, -2.0), field_to_ned(-3.18, 0.02),
         {make_terminal_hold_peer(1, field_to_ned(2.99, 0.24)),
          make_terminal_hold_peer(2, field_to_ned(0.1, -0.3))}, true, 1.5},
        {"detour_open", Vector3{0.0, -3.0, -2.0}, Vector3{0.0, 3.0, -2.0},
         {make_terminal_hold_peer(2, Vector3{0.0, 0.0, -2.0})}, false, 2.0}};
    for (const auto& item : stalled)
    {
        const auto result = plan_case(item, map, fence);
        ASSERT_TRUE(result.is_valid) << item.name;
        EXPECT_TRUE(candidate_lacks_path_progress(
            result.trajectory, REPLAN_RATIO)) << item.name;
    }
    for (const auto& item : normal)
    {
        const auto result = plan_case(item, map, fence);
        ASSERT_TRUE(result.is_valid) << item.name;
        EXPECT_FALSE(candidate_lacks_path_progress(
            result.trajectory, REPLAN_RATIO)) << item.name;
    }
}

}  // namespace
