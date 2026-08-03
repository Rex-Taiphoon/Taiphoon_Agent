/**
 * @file trajectory_planner_test.cpp
 * @brief TrajectoryPlanner 單元測試（gtest）
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 測試 TrajectoryPlanner 的路徑生成、到達時間計算與安全檢查邏輯。
 * 需在 colcon 環境中編譯執行：colcon test --packages-select taiphoon_agent
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "trajectory/TrajectoryPlanner.hpp"

using taiphoon::trajectory::TrajectoryPlanner;
using taiphoon::trajectory::Waypoint;

// ==================== generate_path 測試 ====================

TEST(TrajectoryPlannerTest, GeneratePathStraightLine)
{
    // 從 (0,0,0) 到 (10,0,0)，間距 2m
    std::array<float, 3> current = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target  = {10.0F, 0.0F, 0.0F};

    auto waypoints = TrajectoryPlanner::generate_path(current, target, 2.0F);

    // 預期：0, 2, 4, 6, 8, 10 → 6 個 waypoint
    ASSERT_EQ(waypoints.size(), 6u);
    EXPECT_NEAR(waypoints[0].position_ned[0], 0.0F, 1e-4F);
    EXPECT_NEAR(waypoints[5].position_ned[0], 10.0F, 1e-4F);
    EXPECT_NEAR(waypoints[2].position_ned[0], 4.0F, 1e-4F);
}

TEST(TrajectoryPlannerTest, GeneratePathNoMovement)
{
    std::array<float, 3> pos = {5.0F, -3.0F, 10.0F};
    auto waypoints = TrajectoryPlanner::generate_path(pos, pos, 1.0F);

    ASSERT_EQ(waypoints.size(), 1u);
    EXPECT_NEAR(waypoints[0].position_ned[0], 5.0F, 1e-6F);
    EXPECT_NEAR(waypoints[0].position_ned[1], -3.0F, 1e-6F);
}

// ==================== compute_arrival_times 測試 ====================

TEST(TrajectoryPlannerTest, ComputeArrivalTimes)
{
    // 5 個 waypoint，間距 2m，速度 2 m/s
    std::array<float, 3> a = {0, 0, 0};
    std::array<float, 3> b = {8, 0, 0};
    auto waypoints = TrajectoryPlanner::generate_path(a, b, 2.0F);

    TrajectoryPlanner::compute_arrival_times(waypoints, 0.0, 2.0);

    // 每個 waypoint 間隔 2m / 2m/s = 1 秒
    ASSERT_GE(waypoints.size(), 5u);
    EXPECT_NEAR(waypoints[0].arrival_time_s, 0.0, 1e-6);
    EXPECT_NEAR(waypoints[1].arrival_time_s, 1.0, 1e-6);
    EXPECT_NEAR(waypoints[4].arrival_time_s, 4.0, 1e-6);
}

// ==================== is_waypoint_safe 測試 ====================

TEST(TrajectoryPlannerTest, IsWaypointSafeFar)
{
    std::array<float, 3> wp = {5.0F, 0.0F, 0.0F};
    std::vector<std::array<float, 3>> peers = {{10.0F, 0.0F, 0.0F}}; // 距離 5m

    EXPECT_TRUE(TrajectoryPlanner::is_waypoint_safe(wp, peers, 3.0F));
}

TEST(TrajectoryPlannerTest, IsWaypointSafeClose)
{
    std::array<float, 3> wp = {5.0F, 0.0F, 0.0F};
    std::vector<std::array<float, 3>> peers = {{6.5F, 0.0F, 0.0F}}; // 距離 1.5m

    EXPECT_FALSE(TrajectoryPlanner::is_waypoint_safe(wp, peers, 3.0F));
}

TEST(TrajectoryPlannerTest, IsWaypointSafeExactThreshold)
{
    std::array<float, 3> wp = {0.0F, 0.0F, 0.0F};
    std::vector<std::array<float, 3>> peers = {{3.0F, 0.0F, 0.0F}}; // exactly 3m

    // 距離 ≥ 安全邊界才算安全
    EXPECT_TRUE(TrajectoryPlanner::is_waypoint_safe(wp, peers, 3.0F));
}

// ==================== find_unsafe_waypoints 測試 ====================

TEST(TrajectoryPlannerTest, FindUnsafeWaypoints)
{
    std::array<float, 3> current = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target  = {10.0F, 0.0F, 0.0F};
    auto waypoints = TrajectoryPlanner::generate_path(current, target, 2.0F);

    // 同儕位於 (5, 0, 0)，距離 5m → 在路徑上
    std::vector<std::array<float, 3>> peers = {{5.0F, 0.0F, 0.0F}};

    auto unsafe = TrajectoryPlanner::find_unsafe_waypoints(
        waypoints, peers, 3.0F);

    // idx 0: x=0, dist=5 → safe
    // idx 1: x=2, dist=3 → safe (= threshold)
    // idx 2: x=4, dist=1 → unsafe
    // idx 3: x=6, dist=1 → unsafe
    // idx 4: x=8, dist=3 → safe
    // idx 5: x=10, dist=5 → safe
    ASSERT_EQ(unsafe.size(), 2u);

    bool found_idx2 = false;
    bool found_idx3 = false;
    for (size_t i : unsafe)
    {
        if (i == 2) found_idx2 = true;
        if (i == 3) found_idx3 = true;
    }
    EXPECT_TRUE(found_idx2);
    EXPECT_TRUE(found_idx3);
}

TEST(TrajectoryPlannerTest, NoUnsafeWaypoints)
{
    std::array<float, 3> current = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target  = {10.0F, 0.0F, 0.0F};
    auto waypoints = TrajectoryPlanner::generate_path(current, target, 2.0F);

    // 同儕在遠方
    std::vector<std::array<float, 3>> peers = {{100.0F, 100.0F, 100.0F}};
    auto unsafe = TrajectoryPlanner::find_unsafe_waypoints(
        waypoints, peers, 3.0F);

    EXPECT_TRUE(unsafe.empty());
}

TEST(TrajectoryPlannerTest, NoPeers)
{
    std::array<float, 3> current = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target  = {5.0F, 0.0F, 0.0F};
    auto waypoints = TrajectoryPlanner::generate_path(current, target, 1.0F);

    std::vector<std::array<float, 3>> peers;
    auto unsafe = TrajectoryPlanner::find_unsafe_waypoints(
        waypoints, peers, 3.0F);

    EXPECT_TRUE(unsafe.empty());
}

TEST(TrajectoryPlannerTest, WaypointDistance3d)
{
    std::array<float, 3> current = {0.0F, 0.0F, -5.0F};
    std::array<float, 3> target  = {0.0F, 0.0F, 5.0F};
    auto waypoints = TrajectoryPlanner::generate_path(current, target, 2.0F);

    // 總距離 10m，間距 2m → 6 個 waypoint
    ASSERT_EQ(waypoints.size(), 6u);
    EXPECT_NEAR(waypoints[0].position_ned[2], -5.0F, 1e-4F);
    EXPECT_NEAR(waypoints[5].position_ned[2], 5.0F, 1e-4F);
}
