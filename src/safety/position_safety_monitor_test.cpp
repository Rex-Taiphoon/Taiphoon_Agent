/**
 * @file position_safety_monitor_test.cpp
 * @brief PositionSafetyMonitor 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 測試 PositionSafetyMonitor 的歐氏距離計算與偏差閾值檢查。
 * 使用簡易 assert 巨集，不依賴外部測試框架。
 */

#include "safety/PositionSafetyMonitor.hpp"

#include <cstdio>
#include <cmath>
#include <cstdlib>

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

// ==================== 測試案例 ====================

void test_compute_distance_origin()
{
    std::printf("\n--- test_compute_distance_origin ---\n");

    taiphoon::safety::PositionSafetyMonitor monitor(2.0F);

    // 原點到 (3, 4, 0) 的歐氏距離 = 5.0
    std::array<float, 3> origin = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target = {3.0F, 4.0F, 0.0F};

    float dist = monitor.compute_distance(origin, target);
    TEST_ASSERT_FLOAT_EQ(dist, 5.0F, 1e-6F,
                         "Distance from origin to (3,4,0) should be 5.0");
}

void test_compute_distance_same_point()
{
    std::printf("\n--- test_compute_distance_same_point ---\n");

    taiphoon::safety::PositionSafetyMonitor monitor(2.0F);

    std::array<float, 3> a = {1.5F, -2.3F, 10.0F};
    std::array<float, 3> b = {1.5F, -2.3F, 10.0F};

    float dist = monitor.compute_distance(a, b);
    TEST_ASSERT_FLOAT_EQ(dist, 0.0F, 1e-6F,
                         "Distance between identical points should be 0.0");
}

void test_compute_distance_3d()
{
    std::printf("\n--- test_compute_distance_3d ---\n");

    taiphoon::safety::PositionSafetyMonitor monitor(2.0F);

    // (1, 2, 3) 到 (4, 6, 3) 的距離 = sqrt(9+16+0) = 5
    std::array<float, 3> a = {1.0F, 2.0F, 3.0F};
    std::array<float, 3> b = {4.0F, 6.0F, 3.0F};

    float dist = monitor.compute_distance(a, b);
    TEST_ASSERT_FLOAT_EQ(dist, 5.0F, 1e-6F,
                         "3D distance (1,2,3)→(4,6,3) should be 5.0");
}

void test_deviation_not_exceeded()
{
    std::printf("\n--- test_deviation_not_exceeded ---\n");

    // 閾值 2.0m，偏差 1.5m → 不觸發
    taiphoon::safety::PositionSafetyMonitor monitor(2.0F);

    std::array<float, 3> current = {0.0F, 0.0F, 3.0F};
    std::array<float, 3> target  = {1.0F, 1.0F, 3.0F}; // 距離 sqrt(2) ≈ 1.414

    bool exceeded = monitor.is_deviation_exceeded(current, target);
    TEST_ASSERT(!exceeded,
                "Deviation 1.414m should NOT exceed 2.0m threshold");
}

void test_deviation_exceeded()
{
    std::printf("\n--- test_deviation_exceeded ---\n");

    // 閾值 2.0m，偏差 3.0m → 觸發
    taiphoon::safety::PositionSafetyMonitor monitor(2.0F);

    std::array<float, 3> current = {0.0F, 0.0F, 3.0F};
    std::array<float, 3> target  = {3.0F, 0.0F, 3.0F}; // 距離 3.0

    bool exceeded = monitor.is_deviation_exceeded(current, target);
    TEST_ASSERT(exceeded,
                "Deviation 3.0m should exceed 2.0m threshold");
}

void test_deviation_exactly_at_threshold()
{
    std::printf("\n--- test_deviation_exactly_at_threshold ---\n");

    // 閾值 2.0m，偏差恰好 2.0m → 應觸發（≥ 閾值）
    taiphoon::safety::PositionSafetyMonitor monitor(2.0F);

    std::array<float, 3> current = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target  = {2.0F, 0.0F, 0.0F}; // 距離 2.0

    bool exceeded = monitor.is_deviation_exceeded(current, target);
    TEST_ASSERT(exceeded,
                "Deviation exactly 2.0m should exceed 2.0m threshold (≥)");
}

void test_custom_threshold()
{
    std::printf("\n--- test_custom_threshold ---\n");

    // 自訂閾值 5.0m
    taiphoon::safety::PositionSafetyMonitor monitor(5.0F);

    std::array<float, 3> current = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target  = {4.0F, 0.0F, 0.0F}; // 距離 4.0

    bool exceeded = monitor.is_deviation_exceeded(current, target);
    TEST_ASSERT(!exceeded,
                "Deviation 4.0m should NOT exceed 5.0m custom threshold");

    // 確認取得閾值
    TEST_ASSERT_FLOAT_EQ(monitor.get_threshold(), 5.0F, 1e-6F,
                         "get_threshold() should return 5.0");
}

void test_negative_coordinates()
{
    std::printf("\n--- test_negative_coordinates ---\n");

    taiphoon::safety::PositionSafetyMonitor monitor(3.0F);

    // 含負值的 NED 座標
    std::array<float, 3> current = {-10.0F, 5.0F, -2.0F};
    std::array<float, 3> target  = {-6.0F, 2.0F, -2.0F}; // dx=4, dy=-3, dz=0 → dist=5

    float dist = monitor.compute_distance(current, target);
    TEST_ASSERT_FLOAT_EQ(dist, 5.0F, 1e-6F,
                         "Distance with negative coords should be 5.0");

    bool exceeded = monitor.is_deviation_exceeded(current, target);
    TEST_ASSERT(exceeded,
                "Deviation 5.0m should exceed 3.0m threshold");
}

// ==================== 主程式 ====================

int main()
{
    std::printf("=== PositionSafetyMonitor 單元測試 ===\n");

    test_compute_distance_origin();
    test_compute_distance_same_point();
    test_compute_distance_3d();
    test_deviation_not_exceeded();
    test_deviation_exceeded();
    test_deviation_exactly_at_threshold();
    test_custom_threshold();
    test_negative_coordinates();

    std::printf("\n=== 測試結果: %d/%d 通過, %d 失敗 ===\n",
                g_pass_count, g_test_count, g_fail_count);

    return (g_fail_count == 0) ? 0 : 1;
}
