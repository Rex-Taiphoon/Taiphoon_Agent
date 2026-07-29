/**
 * @file trajectory_planner_test.cpp
 * @brief TrajectoryPlanner 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 */

#include "trajectory/TrajectoryPlanner.hpp"

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <vector>

static int g_test_count = 0;
static int g_pass_count = 0;
static int g_fail_count = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        g_test_count++; \
        if (!(cond)) { \
            std::printf("  FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
            g_fail_count++; \
        } else { \
            g_pass_count++; \
        } \
    } while(0)

#define TEST_ASSERT_FLOAT_EQ(actual, expected, tolerance, msg) \
    do { \
        g_test_count++; \
        if (std::abs((actual) - (expected)) > (tolerance)) { \
            std::printf("  FAIL: %s — expected %.6f, got %.6f (%s:%d)\n", \
                        msg, static_cast<double>(expected), \
                        static_cast<double>(actual), __FILE__, __LINE__); \
            g_fail_count++; \
        } else { \
            g_pass_count++; \
        } \
    } while(0)

using taiphoon::trajectory::TrajectoryPlanner;
using taiphoon::trajectory::Waypoint;

void test_generate_path_straight_line()
{
    std::printf("\n--- test_generate_path_straight_line ---\n");

    // 從 (0,0,0) 到 (10,0,0)，間距 2m
    std::array<float, 3> current = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target  = {10.0F, 0.0F, 0.0F};

    auto waypoints = TrajectoryPlanner::generate_path(current, target, 2.0F);

    // 預期：0, 2, 4, 6, 8, 10 → 6 個 waypoint
    TEST_ASSERT(waypoints.size() == 6,
                "10m / 2.0m spacing → 6 waypoints (including start)");
    if (waypoints.size() >= 6)
    {
        TEST_ASSERT_FLOAT_EQ(waypoints[0].position_ned[0], 0.0F, 1e-4F,
                             "waypoint[0].x = 0 (start)");
        TEST_ASSERT_FLOAT_EQ(waypoints[5].position_ned[0], 10.0F, 1e-4F,
                             "waypoint[5].x = 10 (target)");
        TEST_ASSERT_FLOAT_EQ(waypoints[2].position_ned[0], 4.0F, 1e-4F,
                             "waypoint[2].x = 4");
    }
}

void test_generate_path_no_movement()
{
    std::printf("\n--- test_generate_path_no_movement ---\n");

    std::array<float, 3> pos = {5.0F, -3.0F, 10.0F};
    auto waypoints = TrajectoryPlanner::generate_path(pos, pos, 1.0F);

    TEST_ASSERT(waypoints.size() == 1,
                "Same current/target → 1 waypoint");
    if (!waypoints.empty())
    {
        TEST_ASSERT_FLOAT_EQ(waypoints[0].position_ned[0], 5.0F, 1e-6F, "");
        TEST_ASSERT_FLOAT_EQ(waypoints[0].position_ned[1], -3.0F, 1e-6F, "");
    }
}

void test_compute_arrival_times()
{
    std::printf("\n--- test_compute_arrival_times ---\n");

    // 5 個 waypoint，間距 2m，速度 2 m/s
    std::array<float, 3> a = {0, 0, 0};
    std::array<float, 3> b = {8, 0, 0};
    auto waypoints = TrajectoryPlanner::generate_path(a, b, 2.0F);

    TrajectoryPlanner::compute_arrival_times(waypoints, 0.0, 2.0);

    // 每個 waypoint 間隔 2m / 2m/s = 1 秒
    TEST_ASSERT(waypoints.size() >= 5, "At least 5 waypoints");
    if (waypoints.size() >= 5)
    {
        TEST_ASSERT_FLOAT_EQ(waypoints[0].arrival_time_s, 0.0, 1e-6,
                             "first waypoint at time 0");
        TEST_ASSERT_FLOAT_EQ(waypoints[1].arrival_time_s, 1.0, 1e-6,
                             "second waypoint at time 1s");
        TEST_ASSERT_FLOAT_EQ(waypoints[4].arrival_time_s, 4.0, 1e-6,
                             "last waypoint at time 4s");
    }
}

void test_is_waypoint_safe_far()
{
    std::printf("\n--- test_is_waypoint_safe_far ---\n");

    std::array<float, 3> wp = {5.0F, 0.0F, 0.0F};
    std::vector<std::array<float, 3>> peers = {{10.0F, 0.0F, 0.0F}}; // 距離 5m

    TEST_ASSERT(TrajectoryPlanner::is_waypoint_safe(wp, peers, 3.0F),
                "Peer at 5m with margin 3m → safe");
}

void test_is_waypoint_safe_close()
{
    std::printf("\n--- test_is_waypoint_safe_close ---\n");

    std::array<float, 3> wp = {5.0F, 0.0F, 0.0F};
    std::vector<std::array<float, 3>> peers = {{6.5F, 0.0F, 0.0F}}; // 距離 1.5m

    TEST_ASSERT(!TrajectoryPlanner::is_waypoint_safe(wp, peers, 3.0F),
                "Peer at 1.5m with margin 3m → unsafe");
}

void test_is_waypoint_safe_exact_threshold()
{
    std::printf("\n--- test_is_waypoint_safe_exact_threshold ---\n");

    std::array<float, 3> wp = {0.0F, 0.0F, 0.0F};
    std::vector<std::array<float, 3>> peers = {{3.0F, 0.0F, 0.0F}}; // exactly 3m

    // 距離 ≥ 安全邊界才算安全
    TEST_ASSERT(TrajectoryPlanner::is_waypoint_safe(wp, peers, 3.0F),
                "Peer at exactly 3m with margin 3m → safe (≥)");
}

void test_find_unsafe_waypoints()
{
    std::printf("\n--- test_find_unsafe_waypoints ---\n");

    std::array<float, 3> current = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target  = {10.0F, 0.0F, 0.0F};
    auto waypoints = TrajectoryPlanner::generate_path(current, target, 2.0F);

    // 同儕位於 (5, 0, 0)，距離 5m → 在路徑上
    std::vector<std::array<float, 3>> peers = {{5.0F, 0.0F, 0.0F}};

    auto unsafe = TrajectoryPlanner::find_unsafe_waypoints(
        waypoints, peers, 3.0F);

    // WP at x=4 (idx 2): distance from (5,0,0) = 1m → unsafe
    // WP at x=6 (idx 3): distance from (5,0,0) = 1m → unsafe
    // WP at x=5 (idx 2.5 — doesn't exist at 2m spacing)

    // Let's check what we have:
    // idx 0: x=0, dist=5 → safe
    // idx 1: x=2, dist=3 → safe (= threshold)
    // idx 2: x=4, dist=1 → unsafe
    // idx 3: x=6, dist=1 → unsafe
    // idx 4: x=8, dist=3 → safe
    // idx 5: x=10, dist=5 → safe

    TEST_ASSERT(unsafe.size() == 2, "2 waypoints should be unsafe (x=4, x=6)");
    bool found_idx2 = false;
    bool found_idx3 = false;
    for (size_t i : unsafe)
    {
        if (i == 2) found_idx2 = true;
        if (i == 3) found_idx3 = true;
    }
    TEST_ASSERT(found_idx2, "idx=2 (x=4) should be unsafe");
    TEST_ASSERT(found_idx3, "idx=3 (x=6) should be unsafe");
}

void test_no_unsafe_waypoints()
{
    std::printf("\n--- test_no_unsafe_waypoints ---\n");

    std::array<float, 3> current = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target  = {10.0F, 0.0F, 0.0F};
    auto waypoints = TrajectoryPlanner::generate_path(current, target, 2.0F);

    // 同儕在遠方
    std::vector<std::array<float, 3>> peers = {{100.0F, 100.0F, 100.0F}};
    auto unsafe = TrajectoryPlanner::find_unsafe_waypoints(
        waypoints, peers, 3.0F);

    TEST_ASSERT(unsafe.empty(), "No waypoints should be unsafe");
}

void test_no_peers()
{
    std::printf("\n--- test_no_peers ---\n");

    std::array<float, 3> current = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target  = {5.0F, 0.0F, 0.0F};
    auto waypoints = TrajectoryPlanner::generate_path(current, target, 1.0F);

    std::vector<std::array<float, 3>> peers;
    auto unsafe = TrajectoryPlanner::find_unsafe_waypoints(
        waypoints, peers, 3.0F);

    TEST_ASSERT(unsafe.empty(), "No peers → no unsafe waypoints");
}

void test_waypoint_distance_3d()
{
    std::printf("\n--- test_waypoint_distance_3d ---\n");

    std::array<float, 3> current = {0.0F, 0.0F, -5.0F};
    std::array<float, 3> target  = {0.0F, 0.0F, 5.0F};
    auto waypoints = TrajectoryPlanner::generate_path(current, target, 2.0F);

    // 總距離 10m，間距 2m → 6 個 waypoint
    TEST_ASSERT(waypoints.size() == 6,
                "Vertical movement: 10m / 2.0m → 6 waypoints");
    if (waypoints.size() >= 6)
    {
        TEST_ASSERT_FLOAT_EQ(waypoints[0].position_ned[2], -5.0F, 1e-4F, "");
        TEST_ASSERT_FLOAT_EQ(waypoints[5].position_ned[2], 5.0F, 1e-4F, "");
    }
}

// ==================== 主程式 ====================

int main()
{
    std::printf("=== TrajectoryPlanner 單元測試 ===\n");

    test_generate_path_straight_line();
    test_generate_path_no_movement();
    test_compute_arrival_times();
    test_is_waypoint_safe_far();
    test_is_waypoint_safe_close();
    test_is_waypoint_safe_exact_threshold();
    test_find_unsafe_waypoints();
    test_no_unsafe_waypoints();
    test_no_peers();
    test_waypoint_distance_3d();

    std::printf("\n=== 測試結果: %d/%d 通過, %d 失敗 ===\n",
                g_pass_count, g_test_count, g_fail_count);

    return (g_fail_count == 0) ? 0 : 1;
}
