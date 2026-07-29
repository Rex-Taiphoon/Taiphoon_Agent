/**
 * @file formation_calculator_test.cpp
 * @brief FormationCalculator 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 */

#include "formation/FormationCalculator.hpp"

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

using taiphoon::formation::FormationCalculator;
using taiphoon::formation::FormationType;
using taiphoon::formation::FormationParams;
using taiphoon::formation::PeerPosition;

void test_count_online_all_offline()
{
    std::printf("\n--- test_count_online_all_offline ---\n");
    std::vector<PeerPosition> peers = {
        {{0,0,0}, false},
        {{0,0,0}, false},
    };
    TEST_ASSERT(FormationCalculator::count_online(peers) == 0,
                "All offline → count=0");
}

void test_count_online_mixed()
{
    std::printf("\n--- test_count_online_mixed ---\n");
    std::vector<PeerPosition> peers = {
        {{0,0,0}, true},
        {{0,0,0}, false},
        {{0,0,0}, true},
    };
    TEST_ASSERT(FormationCalculator::count_online(peers) == 2,
                "2 online → count=2");
}

void test_compute_centroid_single_peer()
{
    std::printf("\n--- test_compute_centroid_single_peer ---\n");
    std::vector<PeerPosition> peers = {
        {{10.0F, 20.0F, -3.0F}, true},
    };
    auto c = FormationCalculator::compute_centroid(peers);
    TEST_ASSERT_FLOAT_EQ(c[0], 10.0F, 1e-6F, "centroid.x = 10");
    TEST_ASSERT_FLOAT_EQ(c[1], 20.0F, 1e-6F, "centroid.y = 20");
    TEST_ASSERT_FLOAT_EQ(c[2], -3.0F, 1e-6F, "centroid.z = -3");
}

void test_compute_centroid_multiple_peers()
{
    std::printf("\n--- test_compute_centroid_multiple_peers ---\n");
    std::vector<PeerPosition> peers = {
        {{0.0F, 0.0F, 0.0F}, true},
        {{10.0F, 10.0F, 10.0F}, true},
        {{20.0F, 20.0F, 20.0F}, true},
    };
    auto c = FormationCalculator::compute_centroid(peers);
    TEST_ASSERT_FLOAT_EQ(c[0], 10.0F, 1e-6F, "centroid.x = 10");
    TEST_ASSERT_FLOAT_EQ(c[1], 10.0F, 1e-6F, "centroid.y = 10");
    TEST_ASSERT_FLOAT_EQ(c[2], 10.0F, 1e-6F, "centroid.z = 10");
}

void test_compute_centroid_skips_offline()
{
    std::printf("\n--- test_compute_centroid_skips_offline ---\n");
    std::vector<PeerPosition> peers = {
        {{0.0F, 0.0F, 0.0F}, true},
        {{100.0F, 100.0F, 100.0F}, false},
        {{20.0F, 20.0F, 20.0F}, true},
    };
    auto c = FormationCalculator::compute_centroid(peers);
    TEST_ASSERT_FLOAT_EQ(c[0], 10.0F, 1e-6F, "centroid.x = 10 (skip offline)");
    TEST_ASSERT_FLOAT_EQ(c[1], 10.0F, 1e-6F, "centroid.y = 10");
    TEST_ASSERT_FLOAT_EQ(c[2], 10.0F, 1e-6F, "centroid.z = 10");
}

void test_compute_centroid_empty()
{
    std::printf("\n--- test_compute_centroid_empty ---\n");
    std::vector<PeerPosition> peers;
    auto c = FormationCalculator::compute_centroid(peers);
    TEST_ASSERT_FLOAT_EQ(c[0], 0.0F, 1e-6F, "empty → centroid.x = 0");
    TEST_ASSERT_FLOAT_EQ(c[1], 0.0F, 1e-6F, "empty → centroid.y = 0");
    TEST_ASSERT_FLOAT_EQ(c[2], 0.0F, 1e-6F, "empty → centroid.z = 0");
}

void test_formation_circle()
{
    std::printf("\n--- test_formation_circle ---\n");
    // 3 架無人機，環形半徑 5m，高度 4m
    // 自身 index=0 → 角度 0°
    std::array<float, 3> centroid = {0.0F, 0.0F, 0.0F};
    FormationParams params;
    params.param1 = 5.0F;  // 半徑
    params.param2 = 4.0F;  // 高度

    auto t = FormationCalculator::compute_formation_target(
        centroid, FormationType::CIRCLE, params, 0, 3);

    TEST_ASSERT_FLOAT_EQ(t[0], 5.0F, 1e-4F, "circle index0: x = radius");
    TEST_ASSERT_FLOAT_EQ(t[1], 0.0F, 1e-4F, "circle index0: y = 0");
    TEST_ASSERT_FLOAT_EQ(t[2], -4.0F, 1e-4F, "circle index0: z = -altitude (NED)");
    TEST_ASSERT(std::isnan(t[3]), "circle: yaw should be NaN");
}

void test_formation_circle_index1()
{
    std::printf("\n--- test_formation_circle_index1 ---\n");
    // 3 架無人機，半徑 5m，自身 index=1 → 角度 120°
    std::array<float, 3> centroid = {0.0F, 0.0F, 0.0F};
    FormationParams params;
    params.param1 = 5.0F;
    params.param2 = 4.0F;

    auto t = FormationCalculator::compute_formation_target(
        centroid, FormationType::CIRCLE, params, 1, 3);

    constexpr float PI = 3.14159265358979323846F;
    float angle = 2.0F * PI / 3.0F;
    TEST_ASSERT_FLOAT_EQ(t[0], 5.0F * std::cos(angle), 1e-4F,
                         "circle index1: x = r*cos(120deg)");
    TEST_ASSERT_FLOAT_EQ(t[1], 5.0F * std::sin(angle), 1e-4F,
                         "circle index1: y = r*sin(120deg)");
    TEST_ASSERT_FLOAT_EQ(t[2], -4.0F, 1e-4F, "circle index1: z = -altitude");
}

void test_formation_hori_linear()
{
    std::printf("\n--- test_formation_hori_linear ---\n");
    // 3 架無人機，水平直列，間距 3m，高度 5m
    // index=0(mid=-1): offset = 3*(-1) = -3, index=1(mid=0): offset=0, index=2(mid=1): offset=3
    std::array<float, 3> centroid = {10.0F, 20.0F, 30.0F};
    FormationParams params;
    params.param1 = 3.0F;  // 間距
    params.param2 = 5.0F;  // 高度

    // index 0
    auto t0 = FormationCalculator::compute_formation_target(
        centroid, FormationType::HORI_LINEAR, params, 0, 3);
    TEST_ASSERT_FLOAT_EQ(t0[0], 10.0F, 1e-4F, "hori index0: x = centroid.x");
    TEST_ASSERT_FLOAT_EQ(t0[1], 17.0F, 1e-4F, "hori index0: y = centroid.y - 3");
    TEST_ASSERT_FLOAT_EQ(t0[2], -5.0F, 1e-4F, "hori index0: z = -altitude");
    TEST_ASSERT(std::isnan(t0[3]), "hori: yaw should be NaN");

    // index 1
    auto t1 = FormationCalculator::compute_formation_target(
        centroid, FormationType::HORI_LINEAR, params, 1, 3);
    TEST_ASSERT_FLOAT_EQ(t1[1], 20.0F, 1e-4F, "hori index1: y = centroid.y");

    // index 2
    auto t2 = FormationCalculator::compute_formation_target(
        centroid, FormationType::HORI_LINEAR, params, 2, 3);
    TEST_ASSERT_FLOAT_EQ(t2[1], 23.0F, 1e-4F, "hori index2: y = centroid.y + 3");
}

void test_formation_vert_linear()
{
    std::printf("\n--- test_formation_vert_linear ---\n");
    // 2 架無人機，垂直直列，間距 2m
    // index=0(mid=-0.5): offset = 2*(-0.5) = -1
    // index=1(mid=0.5):  offset = 2*(0.5) = 1
    std::array<float, 3> centroid = {5.0F, 5.0F, -10.0F};
    FormationParams params;
    params.param1 = 2.0F;  // 間距
    params.param2 = 0.0F;  // (unused for VERT_LINEAR)

    auto t0 = FormationCalculator::compute_formation_target(
        centroid, FormationType::VERT_LINEAR, params, 0, 2);
    TEST_ASSERT_FLOAT_EQ(t0[0], 5.0F, 1e-4F, "vert index0: x = centroid.x");
    TEST_ASSERT_FLOAT_EQ(t0[1], 5.0F, 1e-4F, "vert index0: y = centroid.y");
    TEST_ASSERT_FLOAT_EQ(t0[2], -11.0F, 1e-4F, "vert index0: z = centroid.z - 1");
    TEST_ASSERT(std::isnan(t0[3]), "vert: yaw should be NaN");

    auto t1 = FormationCalculator::compute_formation_target(
        centroid, FormationType::VERT_LINEAR, params, 1, 2);
    TEST_ASSERT_FLOAT_EQ(t1[2], -9.0F, 1e-4F, "vert index1: z = centroid.z + 1");
}

void test_formation_none()
{
    std::printf("\n--- test_formation_none ---\n");
    std::array<float, 3> centroid = {1.0F, 2.0F, 3.0F};
    FormationParams params;
    auto t = FormationCalculator::compute_formation_target(
        centroid, FormationType::NONE, params, 0, 3);
    TEST_ASSERT_FLOAT_EQ(t[0], 1.0F, 1e-6F, "NONE → target = centroid");
    TEST_ASSERT_FLOAT_EQ(t[1], 2.0F, 1e-6F, "");
    TEST_ASSERT_FLOAT_EQ(t[2], 3.0F, 1e-6F, "");
    TEST_ASSERT(std::isnan(t[3]), "NONE → yaw = NaN");
}

// ==================== 主程式 ====================

int main()
{
    std::printf("=== FormationCalculator 單元測試 ===\n");

    test_count_online_all_offline();
    test_count_online_mixed();
    test_compute_centroid_single_peer();
    test_compute_centroid_multiple_peers();
    test_compute_centroid_skips_offline();
    test_compute_centroid_empty();
    test_formation_circle();
    test_formation_circle_index1();
    test_formation_hori_linear();
    test_formation_vert_linear();
    test_formation_none();

    std::printf("\n=== 測試結果: %d/%d 通過, %d 失敗 ===\n",
                g_pass_count, g_test_count, g_fail_count);

    return (g_fail_count == 0) ? 0 : 1;
}
