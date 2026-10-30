/**
 * @file waypoint_mission_executor_test.cpp
 * @brief 航點任務驗證、WGS84／PX4 local 高度換算與狀態機測試
 * @author Taiphoon Agent Team
 * @date 2026-10-30
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "ego_swarm/WaypointMissionExecutor.hpp"

namespace
{

using taiphoon::ego_swarm::GeoCoordinate;
using taiphoon::ego_swarm::LocalNedFrame;
using taiphoon::ego_swarm::MissionWaypointInput;
using taiphoon::ego_swarm::Vector3;
using taiphoon::ego_swarm::WAYPOINT_ALTITUDE_PX4_LOCAL;
using taiphoon::ego_swarm::WaypointMissionDecision;
using taiphoon::ego_swarm::WaypointMissionExecutor;
using taiphoon::ego_swarm::WaypointMissionLeg;
using taiphoon::ego_swarm::WaypointMissionLimits;
using taiphoon::ego_swarm::WaypointMissionReason;
using taiphoon::ego_swarm::WaypointMissionStartConditions;
using taiphoon::ego_swarm::WaypointMissionState;
using taiphoon::ego_swarm::check_waypoint_mission_range;
using taiphoon::ego_swarm::convert_waypoints_to_planner_legs;
using taiphoon::ego_swarm::validate_waypoint_mission_plan;

constexpr uint32_t MISSION_ID = 42U;
constexpr double HOME_LATITUDE_DEG = 24.0;
constexpr double HOME_LONGITUDE_DEG = 121.0;
constexpr double HOME_ALTITUDE_M = 50.0;
// 緯度 24 度附近 0.0001 度約 11 m，用於產生距離可預期的航點。
constexpr double SMALL_DELTA_DEG = 0.0001;
constexpr double POSITION_TOLERANCE_M = 1.0e-6;
constexpr double NAN_YAW = std::numeric_limits<double>::quiet_NaN();
constexpr double START_TIME_S = 100.0;

MissionWaypointInput make_waypoint(const double latitude_deg,
                                   const double longitude_deg,
                                   const double altitude_m,
                                   const double dwell_s = 0.0)
{
    MissionWaypointInput waypoint;
    waypoint.latitude_deg = latitude_deg;
    waypoint.longitude_deg = longitude_deg;
    waypoint.altitude_m = altitude_m;
    waypoint.dwell_s = dwell_s;
    waypoint.yaw_deg = NAN_YAW;
    return waypoint;
}

std::vector<MissionWaypointInput> make_valid_waypoints()
{
    return {
        make_waypoint(HOME_LATITUDE_DEG + SMALL_DELTA_DEG, HOME_LONGITUDE_DEG, 10.0, 2.0),
        make_waypoint(HOME_LATITUDE_DEG, HOME_LONGITUDE_DEG + SMALL_DELTA_DEG, 15.0, 0.0)};
}

std::vector<WaypointMissionLeg> make_legs()
{
    WaypointMissionLeg first;
    first.target_planner_ned = Vector3{10.0, 0.0, -10.0};
    first.dwell_s = 2.0;
    WaypointMissionLeg second;
    second.target_planner_ned = Vector3{10.0, 10.0, -10.0};
    second.dwell_s = 0.0;
    return {first, second};
}

WaypointMissionStartConditions make_ready_conditions()
{
    WaypointMissionStartConditions conditions;
    conditions.can_plan = true;
    conditions.is_hovering = true;
    conditions.is_px4_offboard = true;
    conditions.has_formation = false;
    return conditions;
}

void expect_rejected(const WaypointMissionDecision& decision,
                     const WaypointMissionReason reason,
                     const uint16_t index = 0U)
{
    EXPECT_FALSE(decision.is_accepted);
    EXPECT_EQ(decision.reason, reason);
    EXPECT_EQ(decision.index, index);
}

void expect_vector_near(const Vector3& actual, const Vector3& expected)
{
    EXPECT_NEAR(actual.at(0), expected.at(0), POSITION_TOLERANCE_M);
    EXPECT_NEAR(actual.at(1), expected.at(1), POSITION_TOLERANCE_M);
    EXPECT_NEAR(actual.at(2), expected.at(2), POSITION_TOLERANCE_M);
}

/// 載入並開始一個兩點任務，回傳第一段目標。
Vector3 start_two_leg_mission(WaypointMissionExecutor& executor)
{
    EXPECT_TRUE(executor.load(MISSION_ID, make_legs(), false, START_TIME_S).is_accepted);
    EXPECT_TRUE(executor.start(MISSION_ID, make_ready_conditions(),
                               Vector3{0.0, 0.0, -10.0}, START_TIME_S).is_accepted);
    const std::optional<Vector3> target = executor.due_target(START_TIME_S);
    EXPECT_TRUE(target.has_value());
    return target.value_or(Vector3{0.0, 0.0, 0.0});
}

// --- 上傳驗證 ---

TEST(WaypointMissionValidationTest, AcceptsValidPlan)
{
    const auto decision = validate_waypoint_mission_plan(
        MISSION_ID, WAYPOINT_ALTITUDE_PX4_LOCAL, make_valid_waypoints(),
        WaypointMissionLimits{});
    EXPECT_TRUE(decision.is_accepted);
}

TEST(WaypointMissionValidationTest, RejectsZeroIdFrameAndCount)
{
    const WaypointMissionLimits limits{};
    expect_rejected(validate_waypoint_mission_plan(
        0U, WAYPOINT_ALTITUDE_PX4_LOCAL, make_valid_waypoints(), limits),
        WaypointMissionReason::INVALID_PLAN);
    // altitude_frame 0 視為未設定
    expect_rejected(validate_waypoint_mission_plan(
        MISSION_ID, 0U, make_valid_waypoints(), limits),
        WaypointMissionReason::INVALID_PLAN);
    expect_rejected(validate_waypoint_mission_plan(
        MISSION_ID, WAYPOINT_ALTITUDE_PX4_LOCAL, {}, limits),
        WaypointMissionReason::INVALID_PLAN);

    WaypointMissionLimits small_limits{};
    small_limits.max_waypoints = 1U;
    expect_rejected(validate_waypoint_mission_plan(
        MISSION_ID, WAYPOINT_ALTITUDE_PX4_LOCAL, make_valid_waypoints(), small_limits),
        WaypointMissionReason::INVALID_PLAN);
}

TEST(WaypointMissionValidationTest, ReportsFirstInvalidWaypointIndex)
{
    const WaypointMissionLimits limits{};
    auto waypoints = make_valid_waypoints();
    waypoints.at(1).latitude_deg = 91.0;
    expect_rejected(validate_waypoint_mission_plan(
        MISSION_ID, WAYPOINT_ALTITUDE_PX4_LOCAL, waypoints, limits),
        WaypointMissionReason::INVALID_PLAN, 1U);

    waypoints = make_valid_waypoints();
    waypoints.at(1).longitude_deg = std::numeric_limits<double>::infinity();
    expect_rejected(validate_waypoint_mission_plan(
        MISSION_ID, WAYPOINT_ALTITUDE_PX4_LOCAL, waypoints, limits),
        WaypointMissionReason::INVALID_PLAN, 1U);
}

TEST(WaypointMissionValidationTest, RejectsAltitudeDwellAndYawOutOfRange)
{
    const WaypointMissionLimits limits{};
    auto waypoints = make_valid_waypoints();
    waypoints.at(0).altitude_m = limits.min_altitude_m - 0.1;
    expect_rejected(validate_waypoint_mission_plan(
        MISSION_ID, WAYPOINT_ALTITUDE_PX4_LOCAL, waypoints, limits),
        WaypointMissionReason::INVALID_PLAN, 0U);

    waypoints = make_valid_waypoints();
    waypoints.at(1).altitude_m = limits.max_altitude_m + 0.1;
    expect_rejected(validate_waypoint_mission_plan(
        MISSION_ID, WAYPOINT_ALTITUDE_PX4_LOCAL, waypoints, limits),
        WaypointMissionReason::INVALID_PLAN, 1U);

    waypoints = make_valid_waypoints();
    waypoints.at(0).dwell_s = -1.0;
    expect_rejected(validate_waypoint_mission_plan(
        MISSION_ID, WAYPOINT_ALTITUDE_PX4_LOCAL, waypoints, limits),
        WaypointMissionReason::INVALID_PLAN, 0U);

    waypoints = make_valid_waypoints();
    waypoints.at(0).dwell_s = limits.max_dwell_s + 1.0;
    expect_rejected(validate_waypoint_mission_plan(
        MISSION_ID, WAYPOINT_ALTITUDE_PX4_LOCAL, waypoints, limits),
        WaypointMissionReason::INVALID_PLAN, 0U);

    // 本版只接受 NaN 航向
    waypoints = make_valid_waypoints();
    waypoints.at(1).yaw_deg = 90.0;
    expect_rejected(validate_waypoint_mission_plan(
        MISSION_ID, WAYPOINT_ALTITUDE_PX4_LOCAL, waypoints, limits),
        WaypointMissionReason::INVALID_PLAN, 1U);
}

TEST(WaypointMissionValidationTest, AcceptsBoundaryValues)
{
    const WaypointMissionLimits limits{};
    auto waypoints = make_valid_waypoints();
    waypoints.at(0).altitude_m = limits.min_altitude_m;
    waypoints.at(0).dwell_s = limits.max_dwell_s;
    waypoints.at(1).altitude_m = limits.max_altitude_m;
    waypoints.at(1).dwell_s = 0.0;
    EXPECT_TRUE(validate_waypoint_mission_plan(
        MISSION_ID, WAYPOINT_ALTITUDE_PX4_LOCAL, waypoints, limits).is_accepted);
}

// --- 座標換算 ---

TEST(WaypointMissionGeometryTest, ConvertsAltitudeWithPx4LocalOffset)
{
    LocalNedFrame planner_frame;
    ASSERT_TRUE(planner_frame.set_origin(
        GeoCoordinate{HOME_LATITUDE_DEG, HOME_LONGITUDE_DEG, HOME_ALTITUDE_M}));
    // PX4 local = planner-local + offset；Agent 在空中重啟時 PX4 原點低於 Home 2 m。
    const Vector3 offset{1.5, -2.0, 2.0};
    std::vector<WaypointMissionLeg> legs;
    ASSERT_TRUE(convert_waypoints_to_planner_legs(
        make_valid_waypoints(), planner_frame, offset, legs));
    ASSERT_EQ(legs.size(), 2U);

    // z_planner = -altitude_m - offset.z，加回 offset 後即為 PX4 local 的 -altitude_m。
    EXPECT_NEAR(legs.at(0).target_planner_ned.at(2), -10.0 - 2.0, POSITION_TOLERANCE_M);
    EXPECT_NEAR(legs.at(0).target_planner_ned.at(2) + offset.at(2), -10.0,
                POSITION_TOLERANCE_M);
    EXPECT_NEAR(legs.at(1).target_planner_ned.at(2), -15.0 - 2.0, POSITION_TOLERANCE_M);
    EXPECT_DOUBLE_EQ(legs.at(0).dwell_s, 2.0);

    // 水平直接由 WGS84 換算，不受 PX4 local offset 影響。
    Vector3 expected_first{};
    ASSERT_TRUE(planner_frame.to_local_ned(
        GeoCoordinate{HOME_LATITUDE_DEG + SMALL_DELTA_DEG, HOME_LONGITUDE_DEG,
                      HOME_ALTITUDE_M}, expected_first));
    EXPECT_NEAR(legs.at(0).target_planner_ned.at(0), expected_first.at(0),
                POSITION_TOLERANCE_M);
    EXPECT_NEAR(legs.at(0).target_planner_ned.at(1), expected_first.at(1),
                POSITION_TOLERANCE_M);
    EXPECT_GT(legs.at(0).target_planner_ned.at(0), 10.0);
    EXPECT_NEAR(legs.at(0).target_planner_ned.at(1), 0.0, 1.0e-3);
    EXPECT_GT(legs.at(1).target_planner_ned.at(1), 9.0);
}

TEST(WaypointMissionGeometryTest, FailsWithoutOriginOrFiniteOffset)
{
    std::vector<WaypointMissionLeg> legs{WaypointMissionLeg{}};
    LocalNedFrame no_origin;
    EXPECT_FALSE(convert_waypoints_to_planner_legs(
        make_valid_waypoints(), no_origin, Vector3{0.0, 0.0, 0.0}, legs));
    EXPECT_TRUE(legs.empty());

    LocalNedFrame planner_frame;
    ASSERT_TRUE(planner_frame.set_origin(
        GeoCoordinate{HOME_LATITUDE_DEG, HOME_LONGITUDE_DEG, HOME_ALTITUDE_M}));
    EXPECT_FALSE(convert_waypoints_to_planner_legs(
        make_valid_waypoints(), planner_frame,
        Vector3{0.0, 0.0, std::numeric_limits<double>::quiet_NaN()}, legs));
    EXPECT_TRUE(legs.empty());
}

TEST(WaypointMissionGeometryTest, RangeCheckUsesHorizontalDistanceOnly)
{
    std::vector<WaypointMissionLeg> legs(2U);
    legs.at(0).target_planner_ned = Vector3{0.0, 0.0, -100.0};
    legs.at(1).target_planner_ned = Vector3{60.0, 80.0, -10.0};

    EXPECT_TRUE(check_waypoint_mission_range(
        legs, Vector3{0.0, 0.0, 0.0}, 100.0).is_accepted);
    const auto rejected = check_waypoint_mission_range(
        legs, Vector3{0.0, 0.0, 0.0}, 99.9);
    EXPECT_FALSE(rejected.is_accepted);
    EXPECT_EQ(rejected.reason, WaypointMissionReason::OUT_OF_RANGE);
    EXPECT_EQ(rejected.index, 1U);

    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    expect_rejected(check_waypoint_mission_range(
        legs, Vector3{unavailable, 0.0, 0.0}, 100.0),
        WaypointMissionReason::NOT_READY);
}

// --- 狀態機 ---

TEST(WaypointMissionExecutorTest, StartsIdle)
{
    const WaypointMissionExecutor executor;
    const auto status = executor.snapshot();
    EXPECT_EQ(status.state, WaypointMissionState::IDLE);
    EXPECT_EQ(status.mission_id, 0U);
    EXPECT_EQ(status.reason, WaypointMissionReason::NONE);
    EXPECT_FALSE(executor.is_active());
}

TEST(WaypointMissionExecutorTest, LoadDoesNotProduceTarget)
{
    WaypointMissionExecutor executor;
    ASSERT_TRUE(executor.load(MISSION_ID, make_legs(), false, START_TIME_S).is_accepted);
    const auto status = executor.snapshot();
    EXPECT_EQ(status.state, WaypointMissionState::LOADED);
    EXPECT_EQ(status.mission_id, MISSION_ID);
    EXPECT_EQ(status.waypoint_count, 2U);
    EXPECT_TRUE(executor.is_active());
    EXPECT_FALSE(executor.is_running());
    EXPECT_FALSE(executor.due_target(START_TIME_S + 1.0).has_value());
}

TEST(WaypointMissionExecutorTest, LoadRejectedDuringFormationKeepsState)
{
    WaypointMissionExecutor executor;
    expect_rejected(executor.load(MISSION_ID, make_legs(), true, START_TIME_S),
                    WaypointMissionReason::BUSY);
    EXPECT_EQ(executor.snapshot().state, WaypointMissionState::IDLE);
}

TEST(WaypointMissionExecutorTest, NewUploadReplacesLoadedMission)
{
    WaypointMissionExecutor executor;
    ASSERT_TRUE(executor.load(MISSION_ID, make_legs(), false, START_TIME_S).is_accepted);
    ASSERT_TRUE(executor.load(MISSION_ID + 1U, make_legs(), false, START_TIME_S).is_accepted);
    EXPECT_EQ(executor.snapshot().mission_id, MISSION_ID + 1U);
    expect_rejected(executor.start(MISSION_ID, make_ready_conditions(),
                                   Vector3{0.0, 0.0, -10.0}, START_TIME_S),
                    WaypointMissionReason::ID_MISMATCH);
}

TEST(WaypointMissionExecutorTest, LoadExpiresToIdle)
{
    WaypointMissionExecutor executor;
    const double load_timeout_s = executor.limits().load_timeout_s;
    ASSERT_TRUE(executor.load(MISSION_ID, make_legs(), false, START_TIME_S).is_accepted);
    EXPECT_FALSE(executor.expire_load_if_due(START_TIME_S + load_timeout_s - 0.1));
    EXPECT_TRUE(executor.expire_load_if_due(START_TIME_S + load_timeout_s));
    const auto status = executor.snapshot();
    EXPECT_EQ(status.state, WaypointMissionState::IDLE);
    EXPECT_EQ(status.mission_id, 0U);
    EXPECT_EQ(status.reason, WaypointMissionReason::NONE);

    // 逾時後的 MISSION_START 以識別碼不符拒絕。
    ASSERT_TRUE(executor.load(MISSION_ID, make_legs(), false, START_TIME_S).is_accepted);
    expect_rejected(executor.start(MISSION_ID, make_ready_conditions(),
                                   Vector3{0.0, 0.0, -10.0},
                                   START_TIME_S + load_timeout_s + 1.0),
                    WaypointMissionReason::ID_MISMATCH);
    EXPECT_EQ(executor.snapshot().state, WaypointMissionState::IDLE);
}

TEST(WaypointMissionExecutorTest, StartRequiresHoveringOffboardAndNoFormation)
{
    WaypointMissionExecutor executor;
    ASSERT_TRUE(executor.load(MISSION_ID, make_legs(), false, START_TIME_S).is_accepted);
    const Vector3 current{0.0, 0.0, -10.0};

    auto conditions = make_ready_conditions();
    conditions.can_plan = false;
    expect_rejected(executor.start(MISSION_ID, conditions, current, START_TIME_S),
                    WaypointMissionReason::NOT_HOVERING);
    conditions = make_ready_conditions();
    conditions.is_hovering = false;
    expect_rejected(executor.start(MISSION_ID, conditions, current, START_TIME_S),
                    WaypointMissionReason::NOT_HOVERING);
    conditions = make_ready_conditions();
    conditions.is_px4_offboard = false;
    expect_rejected(executor.start(MISSION_ID, conditions, current, START_TIME_S),
                    WaypointMissionReason::NOT_HOVERING);
    conditions = make_ready_conditions();
    conditions.has_formation = true;
    expect_rejected(executor.start(MISSION_ID, conditions, current, START_TIME_S),
                    WaypointMissionReason::BUSY);

    // 拒絕不改變目前狀態
    EXPECT_EQ(executor.snapshot().state, WaypointMissionState::LOADED);
    EXPECT_TRUE(executor.start(MISSION_ID, make_ready_conditions(), current,
                               START_TIME_S).is_accepted);
}

TEST(WaypointMissionExecutorTest, StartRechecksRangeFromCurrentPosition)
{
    WaypointMissionExecutor executor;
    ASSERT_TRUE(executor.load(MISSION_ID, make_legs(), false, START_TIME_S).is_accepted);
    const double max_range_m = executor.limits().max_range_m;
    const auto decision = executor.start(
        MISSION_ID, make_ready_conditions(),
        Vector3{max_range_m + 100.0, 0.0, -10.0}, START_TIME_S);
    expect_rejected(decision, WaypointMissionReason::OUT_OF_RANGE, 0U);
    EXPECT_EQ(executor.snapshot().state, WaypointMissionState::LOADED);
}

TEST(WaypointMissionExecutorTest, RunsDwellsAndCompletes)
{
    WaypointMissionExecutor executor;
    const Vector3 first_target = start_two_leg_mission(executor);
    expect_vector_near(first_target, make_legs().at(0).target_planner_ned);
    EXPECT_EQ(executor.snapshot().state, WaypointMissionState::RUNNING);
    EXPECT_EQ(executor.snapshot().current_index, 0U);
    // 目標只交付一次
    EXPECT_FALSE(executor.due_target(START_TIME_S + 0.1).has_value());

    // 其他來源的 Final Target 不算航點到達
    EXPECT_FALSE(executor.on_target_reached(Vector3{0.0, 0.0, -10.0}, START_TIME_S + 5.0));
    EXPECT_EQ(executor.snapshot().state, WaypointMissionState::RUNNING);

    ASSERT_TRUE(executor.on_target_reached(first_target, START_TIME_S + 5.0));
    EXPECT_EQ(executor.snapshot().state, WaypointMissionState::DWELLING);
    EXPECT_EQ(executor.snapshot().current_index, 0U);
    // 停留中不前進
    EXPECT_FALSE(executor.due_target(START_TIME_S + 6.9).has_value());

    const std::optional<Vector3> second_target = executor.due_target(START_TIME_S + 7.0);
    ASSERT_TRUE(second_target.has_value());
    expect_vector_near(*second_target, make_legs().at(1).target_planner_ned);
    EXPECT_EQ(executor.snapshot().state, WaypointMissionState::RUNNING);
    EXPECT_EQ(executor.snapshot().current_index, 1U);

    // 最後一點到達即完成，不停留
    ASSERT_TRUE(executor.on_target_reached(*second_target, START_TIME_S + 20.0));
    const auto status = executor.snapshot();
    EXPECT_EQ(status.state, WaypointMissionState::COMPLETED);
    EXPECT_EQ(status.reason, WaypointMissionReason::NONE);
    EXPECT_EQ(status.current_index, 1U);
    EXPECT_FALSE(executor.is_active());
    EXPECT_FALSE(executor.due_target(START_TIME_S + 100.0).has_value());
}

TEST(WaypointMissionExecutorTest, ArrivalBeforeTargetDeliveredIsIgnored)
{
    WaypointMissionExecutor executor;
    ASSERT_TRUE(executor.load(MISSION_ID, make_legs(), false, START_TIME_S).is_accepted);
    ASSERT_TRUE(executor.start(MISSION_ID, make_ready_conditions(),
                               Vector3{0.0, 0.0, -10.0}, START_TIME_S).is_accepted);
    EXPECT_FALSE(executor.on_target_reached(
        make_legs().at(0).target_planner_ned, START_TIME_S));
    EXPECT_EQ(executor.snapshot().state, WaypointMissionState::RUNNING);
}

TEST(WaypointMissionExecutorTest, StopReasonsAbortRunningMission)
{
    const WaypointMissionReason reasons[] = {
        WaypointMissionReason::OPERATOR_STOP,
        WaypointMissionReason::COMMAND_OVERRIDE,
        WaypointMissionReason::FLIGHT_STATE,
        WaypointMissionReason::LANDED,
        WaypointMissionReason::LEG_TIMEOUT};
    for (const WaypointMissionReason reason : reasons)
    {
        WaypointMissionExecutor executor;
        (void)start_two_leg_mission(executor);
        EXPECT_TRUE(executor.stop(reason, "HOLD"));
        const auto status = executor.snapshot();
        EXPECT_EQ(status.state, WaypointMissionState::ABORTED);
        EXPECT_EQ(status.reason, reason);
        EXPECT_EQ(status.detail, "HOLD");
        EXPECT_EQ(status.mission_id, MISSION_ID);
        EXPECT_FALSE(executor.is_active());
        EXPECT_FALSE(executor.due_target(START_TIME_S + 100.0).has_value());
    }
}

TEST(WaypointMissionExecutorTest, StopDuringDwellAborts)
{
    WaypointMissionExecutor executor;
    const Vector3 first_target = start_two_leg_mission(executor);
    ASSERT_TRUE(executor.on_target_reached(first_target, START_TIME_S + 5.0));
    EXPECT_TRUE(executor.stop(WaypointMissionReason::COMMAND_OVERRIDE, "YAW"));
    EXPECT_EQ(executor.snapshot().state, WaypointMissionState::ABORTED);
    EXPECT_FALSE(executor.due_target(START_TIME_S + 100.0).has_value());
}

TEST(WaypointMissionExecutorTest, StopClearsLoadedMissionWithoutAbort)
{
    WaypointMissionExecutor executor;
    ASSERT_TRUE(executor.load(MISSION_ID, make_legs(), false, START_TIME_S).is_accepted);
    EXPECT_FALSE(executor.stop(WaypointMissionReason::OPERATOR_STOP, "MISSION_STOP"));
    const auto status = executor.snapshot();
    EXPECT_EQ(status.state, WaypointMissionState::IDLE);
    EXPECT_EQ(status.mission_id, 0U);
    EXPECT_EQ(status.reason, WaypointMissionReason::NONE);
    EXPECT_EQ(status.waypoint_count, 0U);
}

TEST(WaypointMissionExecutorTest, CompletedAndAbortedStayUntilNextUpload)
{
    WaypointMissionExecutor executor;
    (void)start_two_leg_mission(executor);
    ASSERT_TRUE(executor.stop(WaypointMissionReason::LEG_TIMEOUT, "leg timeout"));
    // 已結束的任務不受 STOP 影響
    EXPECT_FALSE(executor.stop(WaypointMissionReason::OPERATOR_STOP, "MISSION_STOP"));
    EXPECT_EQ(executor.snapshot().state, WaypointMissionState::ABORTED);
    EXPECT_EQ(executor.snapshot().reason, WaypointMissionReason::LEG_TIMEOUT);

    ASSERT_TRUE(executor.load(MISSION_ID + 1U, make_legs(), false, START_TIME_S).is_accepted);
    EXPECT_EQ(executor.snapshot().state, WaypointMissionState::LOADED);
    EXPECT_EQ(executor.snapshot().reason, WaypointMissionReason::NONE);
}

TEST(WaypointMissionExecutorTest, UploadRejectedWhileRunning)
{
    WaypointMissionExecutor executor;
    (void)start_two_leg_mission(executor);
    expect_rejected(executor.load(MISSION_ID + 1U, make_legs(), false, START_TIME_S),
                    WaypointMissionReason::BUSY);
    EXPECT_EQ(executor.snapshot().state, WaypointMissionState::RUNNING);
    EXPECT_EQ(executor.snapshot().mission_id, MISSION_ID);
}

TEST(WaypointMissionExecutorTest, LegTimeoutScalesWithLegLengthAndExcludesDwell)
{
    WaypointMissionLimits limits{};
    limits.leg_base_timeout_s = 30.0;
    limits.min_expected_speed_mps = 0.5;
    WaypointMissionExecutor executor(limits);
    // 第一段從 (0,0,-10) 到 (10,0,-10)，長 10 m：逾時 30 + 10 / 0.5 = 50 s
    const Vector3 first_target = start_two_leg_mission(executor);
    EXPECT_FALSE(executor.is_leg_timed_out(START_TIME_S + 50.0));
    EXPECT_TRUE(executor.is_leg_timed_out(START_TIME_S + 50.1));

    // 停留不計入逾時
    ASSERT_TRUE(executor.on_target_reached(first_target, START_TIME_S + 10.0));
    EXPECT_FALSE(executor.is_leg_timed_out(START_TIME_S + 1000.0));

    // 第二段從第一點起算，長 10 m
    ASSERT_TRUE(executor.due_target(START_TIME_S + 12.0).has_value());
    EXPECT_FALSE(executor.is_leg_timed_out(START_TIME_S + 62.0));
    EXPECT_TRUE(executor.is_leg_timed_out(START_TIME_S + 62.1));
}

TEST(WaypointMissionExecutorTest, ZeroDwellAdvancesOnNextTick)
{
    std::vector<WaypointMissionLeg> legs = make_legs();
    legs.at(0).dwell_s = 0.0;
    WaypointMissionExecutor executor;
    ASSERT_TRUE(executor.load(MISSION_ID, legs, false, START_TIME_S).is_accepted);
    ASSERT_TRUE(executor.start(MISSION_ID, make_ready_conditions(),
                               Vector3{0.0, 0.0, -10.0}, START_TIME_S).is_accepted);
    const auto first_target = executor.due_target(START_TIME_S);
    ASSERT_TRUE(first_target.has_value());
    ASSERT_TRUE(executor.on_target_reached(*first_target, START_TIME_S + 5.0));
    EXPECT_TRUE(executor.due_target(START_TIME_S + 5.0).has_value());
    EXPECT_EQ(executor.snapshot().current_index, 1U);
}

}  // namespace
