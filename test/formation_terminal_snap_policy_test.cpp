/**
 * @file formation_terminal_snap_policy_test.cpp
 * @brief 編隊直接到位直線段同儕淨空判定測試
 * @author Taiphoon Agent Team
 * @date 2026-10-30
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "ego_swarm/FormationTerminalSnapPolicy.hpp"
#include "ego_swarm/FormationTransition.hpp"

namespace
{

using taiphoon::ego_swarm::BsplineTrajectory;
using taiphoon::ego_swarm::FormationSnapPeerSegment;
using taiphoon::ego_swarm::FORMATION_TERMINAL_SNAP_MAX_PEER_SAMPLES;
using taiphoon::ego_swarm::FORMATION_TERMINAL_SNAP_PEER_LOOKAHEAD_S;
using taiphoon::ego_swarm::FORMATION_TERMINAL_SNAP_PEER_SAMPLE_STEP_S;
using taiphoon::ego_swarm::PeerTrajectory;
using taiphoon::ego_swarm::Vector3;
using taiphoon::ego_swarm::append_formation_snap_peer_path;
using taiphoon::ego_swarm::distance_between_segments;
using taiphoon::ego_swarm::distance_to_segment;
using taiphoon::ego_swarm::is_formation_terminal_snap_clear;

/// 測試用碰撞距離（公尺），與預設 peer_collision_min_distance_m 相同。
constexpr double TEST_CLEARANCE_M = 2.0;
/// 測試共同時間（秒）。
constexpr double TEST_NOW_S = 100.0;
/// 距離比較容許誤差（公尺）。
constexpr double TEST_DISTANCE_TOLERANCE_M = 1.0e-9;

/**
 * @brief 建立停在同一點的 hold 軌跡
 * @param position hold 位置
 * @param start_time_s 軌跡起始時間（秒）
 * @return 同儕軌跡
 */
PeerTrajectory make_hold_trajectory(const Vector3& position, const double start_time_s)
{
    PeerTrajectory peer;
    peer.agent_id = 2;
    peer.start_time_s = start_time_s;
    peer.terminal_hold = true;
    peer.trajectory = BsplineTrajectory(
        std::vector<Vector3>{position, position, position, position}, 1.0);
    peer.end_time_s = start_time_s + peer.trajectory.duration_s();
    return peer;
}

/**
 * @brief 建立沿 East 等速移動的軌跡
 *
 * 控制點等距共線，位置沿 East 單調增加；中段速度約為 speed_mps。
 *
 * @param start_east_m 第一個控制點的 East 座標
 * @param speed_mps 速度（公尺／秒）
 * @param start_time_s 軌跡起始時間（秒）
 * @return 同儕軌跡
 */
PeerTrajectory make_east_moving_trajectory(
    const double start_east_m,
    const double speed_mps,
    const double start_time_s)
{
    constexpr std::size_t CONTROL_POINT_COUNT = 20U;
    constexpr double KNOT_INTERVAL_S = 1.0;
    std::vector<Vector3> control_points;
    for (std::size_t index = 0U; index < CONTROL_POINT_COUNT; ++index)
    {
        control_points.push_back(Vector3{
            0.0, start_east_m + speed_mps * KNOT_INTERVAL_S * static_cast<double>(index), -10.0});
    }
    PeerTrajectory peer;
    peer.agent_id = 3;
    peer.start_time_s = start_time_s;
    peer.trajectory = BsplineTrajectory(control_points, KNOT_INTERVAL_S);
    peer.end_time_s = start_time_s + peer.trajectory.duration_s();
    return peer;
}

/**
 * @brief 以預設取樣參數加入同儕路徑
 * @param live_position 同儕即時位置
 * @param has_trajectory 是否有 fresh trajectory
 * @param trajectory 同儕軌跡
 * @param segments 輸出清單
 * @return append_formation_snap_peer_path 的結果
 */
bool append_default_path(
    const Vector3& live_position,
    const bool has_trajectory,
    const PeerTrajectory& trajectory,
    std::vector<FormationSnapPeerSegment>& segments)
{
    return append_formation_snap_peer_path(
        live_position, has_trajectory, trajectory, TEST_NOW_S,
        FORMATION_TERMINAL_SNAP_PEER_LOOKAHEAD_S,
        FORMATION_TERMINAL_SNAP_PEER_SAMPLE_STEP_S, segments);
}

TEST(FormationTerminalSnapSegmentDistanceTest, CrossingSegmentsHaveZeroDistance)
{
    EXPECT_NEAR(
        distance_between_segments(
            Vector3{-1.0, 0.0, 0.0}, Vector3{1.0, 0.0, 0.0},
            Vector3{0.0, -1.0, 0.0}, Vector3{0.0, 1.0, 0.0}),
        0.0, TEST_DISTANCE_TOLERANCE_M);
}

TEST(FormationTerminalSnapSegmentDistanceTest, SkewSegmentsUseVerticalGap)
{
    EXPECT_NEAR(
        distance_between_segments(
            Vector3{-1.0, 0.0, 0.0}, Vector3{1.0, 0.0, 0.0},
            Vector3{0.0, -1.0, 3.0}, Vector3{0.0, 1.0, 3.0}),
        3.0, TEST_DISTANCE_TOLERANCE_M);
}

TEST(FormationTerminalSnapSegmentDistanceTest, ParallelAndCollinearSegments)
{
    EXPECT_NEAR(
        distance_between_segments(
            Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 4.0, 0.0},
            Vector3{1.5, 1.0, 0.0}, Vector3{1.5, 6.0, 0.0}),
        1.5, TEST_DISTANCE_TOLERANCE_M);
    EXPECT_NEAR(
        distance_between_segments(
            Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 1.0, 0.0},
            Vector3{0.0, 3.5, 0.0}, Vector3{0.0, 5.0, 0.0}),
        2.5, TEST_DISTANCE_TOLERANCE_M);
}

TEST(FormationTerminalSnapSegmentDistanceTest, ClosestPointsClampToEndpoints)
{
    // 兩條線段的延長線相交，但交點落在兩條線段之外。
    EXPECT_NEAR(
        distance_between_segments(
            Vector3{0.0, 0.0, 0.0}, Vector3{1.0, 0.0, 0.0},
            Vector3{4.0, 3.0, 0.0}, Vector3{4.0, 1.0, 0.0}),
        std::sqrt(10.0), TEST_DISTANCE_TOLERANCE_M);
}

TEST(FormationTerminalSnapSegmentDistanceTest, DegenerateSegmentsMatchPointDistance)
{
    const Vector3 start{0.0, -1.0, 0.0};
    const Vector3 end{0.0, 3.0, 0.0};
    const Vector3 point{1.2, 1.0, 0.5};
    EXPECT_NEAR(
        distance_between_segments(start, end, point, point),
        distance_to_segment(point, start, end), TEST_DISTANCE_TOLERANCE_M);
    EXPECT_NEAR(
        distance_between_segments(point, point, start, end),
        distance_to_segment(point, start, end), TEST_DISTANCE_TOLERANCE_M);
    EXPECT_NEAR(
        distance_between_segments(start, start, end, end),
        4.0, TEST_DISTANCE_TOLERANCE_M);
}

TEST(FormationTerminalSnapSegmentDistanceTest, NonFiniteInputFailsClosedAsZero)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_DOUBLE_EQ(
        distance_between_segments(
            Vector3{nan, 0.0, 0.0}, Vector3{1.0, 0.0, 0.0},
            Vector3{10.0, 0.0, 0.0}, Vector3{11.0, 0.0, 0.0}),
        0.0);
    EXPECT_DOUBLE_EQ(
        distance_between_segments(
            Vector3{0.0, 0.0, 0.0}, Vector3{1.0, 0.0, 0.0},
            Vector3{10.0, inf, 0.0}, Vector3{11.0, 0.0, 0.0}),
        0.0);
}

TEST(FormationTerminalSnapPolicyTest, NoPeerIsClear)
{
    EXPECT_TRUE(is_formation_terminal_snap_clear(
        Vector3{0.0, 0.0, -10.0}, Vector3{0.0, -3.0, -10.0}, {}, TEST_CLEARANCE_M));
}

TEST(FormationTerminalSnapPolicyTest, PeerOnStraightPathBlocksSnap)
{
    // 重現 SITL：一字 2.5 m 時 uav_2 的燕行槽位在西方 3 m，直線穿過 uav_1。
    const Vector3 uav2_position{0.0, 2.5, -10.0};
    const Vector3 uav2_slot{0.0, -0.5, -10.0};
    const Vector3 uav1_position{0.0, 0.0, -10.0};
    EXPECT_FALSE(is_formation_terminal_snap_clear(
        uav2_position, uav2_slot,
        {FormationSnapPeerSegment{uav1_position, uav1_position}}, TEST_CLEARANCE_M));
}

TEST(FormationTerminalSnapPolicyTest, PeerBesidePathUsesInclusiveClearance)
{
    const Vector3 current{0.0, 0.0, -10.0};
    const Vector3 slot{0.0, -3.0, -10.0};
    const Vector3 at_clearance{TEST_CLEARANCE_M, -1.5, -10.0};
    const Vector3 inside_clearance{TEST_CLEARANCE_M - 0.01, -1.5, -10.0};
    EXPECT_TRUE(is_formation_terminal_snap_clear(
        current, slot, {FormationSnapPeerSegment{at_clearance, at_clearance}},
        TEST_CLEARANCE_M));
    EXPECT_FALSE(is_formation_terminal_snap_clear(
        current, slot, {FormationSnapPeerSegment{inside_clearance, inside_clearance}},
        TEST_CLEARANCE_M));
}

TEST(FormationTerminalSnapPolicyTest, PeerNearEitherEndpointBlocksSnap)
{
    const Vector3 current{0.0, 0.0, -10.0};
    const Vector3 slot{0.0, -3.0, -10.0};
    const Vector3 beyond_slot{0.0, -4.5, -10.0};
    const Vector3 behind_current{0.0, 1.5, -10.0};
    const Vector3 far_beyond_slot{0.0, -5.5, -10.0};
    EXPECT_FALSE(is_formation_terminal_snap_clear(
        current, slot, {FormationSnapPeerSegment{beyond_slot, beyond_slot}},
        TEST_CLEARANCE_M));
    EXPECT_FALSE(is_formation_terminal_snap_clear(
        current, slot, {FormationSnapPeerSegment{behind_current, behind_current}},
        TEST_CLEARANCE_M));
    EXPECT_TRUE(is_formation_terminal_snap_clear(
        current, slot, {FormationSnapPeerSegment{far_beyond_slot, far_beyond_slot}},
        TEST_CLEARANCE_M));
}

TEST(FormationTerminalSnapPolicyTest, AnyBlockingSegmentBlocksSnap)
{
    const Vector3 current{0.0, 0.0, -10.0};
    const Vector3 slot{0.0, -3.0, -10.0};
    const Vector3 far_peer{20.0, 0.0, -10.0};
    const Vector3 near_peer{0.5, -1.0, -10.0};
    EXPECT_FALSE(is_formation_terminal_snap_clear(
        current, slot,
        {FormationSnapPeerSegment{far_peer, far_peer},
         FormationSnapPeerSegment{near_peer, near_peer}},
        TEST_CLEARANCE_M));
}

TEST(FormationTerminalSnapPolicyTest, InvalidInputFailsClosed)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const Vector3 current{0.0, 0.0, -10.0};
    const Vector3 slot{0.0, -3.0, -10.0};
    const Vector3 far_peer{20.0, 0.0, -10.0};
    const std::vector<FormationSnapPeerSegment> far_segments{
        FormationSnapPeerSegment{far_peer, far_peer}};

    EXPECT_FALSE(is_formation_terminal_snap_clear(
        Vector3{nan, 0.0, -10.0}, slot, far_segments, TEST_CLEARANCE_M));
    EXPECT_FALSE(is_formation_terminal_snap_clear(
        current, Vector3{0.0, inf, -10.0}, far_segments, TEST_CLEARANCE_M));
    EXPECT_FALSE(is_formation_terminal_snap_clear(current, slot, far_segments, nan));
    EXPECT_FALSE(is_formation_terminal_snap_clear(current, slot, far_segments, inf));
    EXPECT_FALSE(is_formation_terminal_snap_clear(current, slot, far_segments, 0.0));
    EXPECT_FALSE(is_formation_terminal_snap_clear(current, slot, far_segments, -1.0));
    EXPECT_FALSE(is_formation_terminal_snap_clear(
        current, slot,
        {FormationSnapPeerSegment{far_peer, Vector3{20.0, nan, -10.0}}},
        TEST_CLEARANCE_M));
}

TEST(FormationTerminalSnapPeerPathTest, WithoutTrajectoryAddsLivePositionOnly)
{
    const Vector3 live{3.0, 4.0, -10.0};
    std::vector<FormationSnapPeerSegment> segments;

    ASSERT_TRUE(append_default_path(live, false, PeerTrajectory{}, segments));
    ASSERT_EQ(segments.size(), 1U);
    EXPECT_EQ(segments.at(0).start, live);
    EXPECT_EQ(segments.at(0).end, live);
}

TEST(FormationTerminalSnapPeerPathTest, HoldPeerPathStartsAtLivePosition)
{
    const Vector3 live{0.0, 2.0, -10.0};
    const Vector3 hold{0.0, 5.0, -10.0};
    std::vector<FormationSnapPeerSegment> segments;

    ASSERT_TRUE(append_default_path(
        live, true, make_hold_trajectory(hold, TEST_NOW_S - 1.0), segments));
    ASSERT_FALSE(segments.empty());
    EXPECT_EQ(segments.front().start, live);
    EXPECT_NEAR(
        taiphoon::ego_swarm::distance_between_centers(segments.front().end, hold),
        0.0, TEST_DISTANCE_TOLERANCE_M);
    EXPECT_NEAR(
        taiphoon::ego_swarm::distance_between_centers(segments.back().end, hold),
        0.0, TEST_DISTANCE_TOLERANCE_M);
}

TEST(FormationTerminalSnapPeerPathTest, CrossingSimultaneousSnapsBlockEachOther)
{
    // 兩台同時直接到位、路徑呈 X 交叉；兩端點都離本機直線超過碰撞距離，
    // 只有「即時位置 → hold 槽位」線段能偵測到交叉。
    const Vector3 current{-1.5, 0.0, -10.0};
    const Vector3 slot{1.5, 0.0, -10.0};
    const Vector3 peer_live{0.0, -2.5, -10.0};
    const Vector3 peer_hold{0.0, 2.5, -10.0};
    std::vector<FormationSnapPeerSegment> segments;

    ASSERT_TRUE(append_default_path(
        peer_live, true, make_hold_trajectory(peer_hold, TEST_NOW_S), segments));
    EXPECT_TRUE(is_formation_terminal_snap_clear(
        current, slot,
        {FormationSnapPeerSegment{peer_live, peer_live},
         FormationSnapPeerSegment{peer_hold, peer_hold}},
        TEST_CLEARANCE_M));
    EXPECT_FALSE(is_formation_terminal_snap_clear(current, slot, segments, TEST_CLEARANCE_M));
}

TEST(FormationTerminalSnapPeerPathTest, MovingPeerCrossingWithinLookaheadBlocksSnap)
{
    // 同儕以 2 m/s 由西向東經過本機直線（East = 0），4 s 內會穿越。
    const Vector3 current{-1.5, 0.0, -10.0};
    const Vector3 slot{1.5, 0.0, -10.0};
    const PeerTrajectory peer = make_east_moving_trajectory(-6.0, 2.0, TEST_NOW_S);
    const auto start_sample = peer.trajectory.evaluate(0.0);
    ASSERT_TRUE(start_sample.is_valid);
    std::vector<FormationSnapPeerSegment> segments;

    ASSERT_TRUE(append_default_path(start_sample.position, true, peer, segments));
    EXPECT_FALSE(is_formation_terminal_snap_clear(current, slot, segments, TEST_CLEARANCE_M));
}

TEST(FormationTerminalSnapPeerPathTest, MovingPeerBeyondLookaheadDoesNotBlockSnap)
{
    // 同儕 4 s 內只到 East 約 -12 m，離本機直線仍遠。
    const Vector3 current{-1.5, 0.0, -10.0};
    const Vector3 slot{1.5, 0.0, -10.0};
    const PeerTrajectory peer = make_east_moving_trajectory(-22.0, 2.0, TEST_NOW_S);
    const auto start_sample = peer.trajectory.evaluate(0.0);
    ASSERT_TRUE(start_sample.is_valid);
    std::vector<FormationSnapPeerSegment> segments;

    ASSERT_TRUE(append_default_path(start_sample.position, true, peer, segments));
    EXPECT_EQ(
        segments.size(),
        static_cast<std::size_t>(std::ceil(
            FORMATION_TERMINAL_SNAP_PEER_LOOKAHEAD_S /
            FORMATION_TERMINAL_SNAP_PEER_SAMPLE_STEP_S)) + 1U);
    EXPECT_TRUE(is_formation_terminal_snap_clear(current, slot, segments, TEST_CLEARANCE_M));
}

TEST(FormationTerminalSnapPeerPathTest, FutureStartTrajectoryUsesStartPosition)
{
    const PeerTrajectory peer = make_east_moving_trajectory(-6.0, 2.0, TEST_NOW_S + 10.0);
    const auto start_sample = peer.trajectory.evaluate(0.0);
    ASSERT_TRUE(start_sample.is_valid);
    std::vector<FormationSnapPeerSegment> segments;

    ASSERT_TRUE(append_default_path(start_sample.position, true, peer, segments));
    for (const auto& segment : segments)
    {
        EXPECT_NEAR(
            taiphoon::ego_swarm::distance_between_centers(segment.end, start_sample.position),
            0.0, TEST_DISTANCE_TOLERANCE_M);
    }
}

TEST(FormationTerminalSnapPeerPathTest, InvalidInputFailsClosedWithoutPartialOutput)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Vector3 live{0.0, 2.0, -10.0};
    const PeerTrajectory hold = make_hold_trajectory(live, TEST_NOW_S);
    const double too_long_lookahead_s = FORMATION_TERMINAL_SNAP_PEER_SAMPLE_STEP_S *
        static_cast<double>(FORMATION_TERMINAL_SNAP_MAX_PEER_SAMPLES);
    PeerTrajectory nan_start = hold;
    nan_start.start_time_s = nan;
    std::vector<FormationSnapPeerSegment> segments;

    EXPECT_FALSE(append_default_path(Vector3{nan, 0.0, 0.0}, false, hold, segments));
    EXPECT_FALSE(append_default_path(live, true, PeerTrajectory{}, segments));
    EXPECT_FALSE(append_default_path(live, true, nan_start, segments));
    EXPECT_FALSE(append_formation_snap_peer_path(
        live, true, hold, nan, FORMATION_TERMINAL_SNAP_PEER_LOOKAHEAD_S,
        FORMATION_TERMINAL_SNAP_PEER_SAMPLE_STEP_S, segments));
    EXPECT_FALSE(append_formation_snap_peer_path(
        live, true, hold, TEST_NOW_S, -1.0,
        FORMATION_TERMINAL_SNAP_PEER_SAMPLE_STEP_S, segments));
    EXPECT_FALSE(append_formation_snap_peer_path(
        live, true, hold, TEST_NOW_S, FORMATION_TERMINAL_SNAP_PEER_LOOKAHEAD_S,
        0.0, segments));
    EXPECT_FALSE(append_formation_snap_peer_path(
        live, true, hold, TEST_NOW_S, FORMATION_TERMINAL_SNAP_PEER_LOOKAHEAD_S,
        nan, segments));
    EXPECT_FALSE(append_formation_snap_peer_path(
        live, true, hold, TEST_NOW_S, too_long_lookahead_s,
        FORMATION_TERMINAL_SNAP_PEER_SAMPLE_STEP_S, segments));
    EXPECT_TRUE(segments.empty());
}

}  // namespace
