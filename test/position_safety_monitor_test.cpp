/**
 * @file position_safety_monitor_test.cpp
 * @brief PositionSafetyMonitor 單元測試（gtest）
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 測試 PositionSafetyMonitor 的歐氏距離計算與偏差閾值檢查。
 * 需在 colcon 環境中編譯執行：colcon test --packages-select taiphoon_agent
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "safety/PositionSafetyMonitor.hpp"

using taiphoon::safety::PositionSafetyMonitor;

// ==================== compute_distance 測試 ====================

TEST(PositionSafetyMonitorTest, ComputeDistanceOrigin)
{
    PositionSafetyMonitor monitor(2.0F);

    // 原點到 (3, 4, 0) 的歐氏距離 = 5.0
    std::array<float, 3> origin = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target = {3.0F, 4.0F, 0.0F};

    float dist = monitor.compute_distance(origin, target);
    EXPECT_NEAR(dist, 5.0F, 1e-6F);
}

TEST(PositionSafetyMonitorTest, ComputeDistanceSamePoint)
{
    PositionSafetyMonitor monitor(2.0F);

    std::array<float, 3> a = {1.5F, -2.3F, 10.0F};
    std::array<float, 3> b = {1.5F, -2.3F, 10.0F};

    float dist = monitor.compute_distance(a, b);
    EXPECT_NEAR(dist, 0.0F, 1e-6F);
}

TEST(PositionSafetyMonitorTest, ComputeDistance3d)
{
    PositionSafetyMonitor monitor(2.0F);

    // (1, 2, 3) 到 (4, 6, 3) 的距離 = sqrt(9+16+0) = 5
    std::array<float, 3> a = {1.0F, 2.0F, 3.0F};
    std::array<float, 3> b = {4.0F, 6.0F, 3.0F};

    float dist = monitor.compute_distance(a, b);
    EXPECT_NEAR(dist, 5.0F, 1e-6F);
}

// ==================== is_deviation_exceeded 測試 ====================

TEST(PositionSafetyMonitorTest, DeviationNotExceeded)
{
    // 閾值 2.0m，偏差 1.5m → 不觸發
    PositionSafetyMonitor monitor(2.0F);

    std::array<float, 3> current = {0.0F, 0.0F, 3.0F};
    std::array<float, 3> target  = {1.0F, 1.0F, 3.0F}; // 距離 sqrt(2) ≈ 1.414

    bool exceeded = monitor.is_deviation_exceeded(current, target);
    EXPECT_FALSE(exceeded);
}

TEST(PositionSafetyMonitorTest, DeviationExceeded)
{
    // 閾值 2.0m，偏差 3.0m → 觸發
    PositionSafetyMonitor monitor(2.0F);

    std::array<float, 3> current = {0.0F, 0.0F, 3.0F};
    std::array<float, 3> target  = {3.0F, 0.0F, 3.0F}; // 距離 3.0

    bool exceeded = monitor.is_deviation_exceeded(current, target);
    EXPECT_TRUE(exceeded);
}

TEST(PositionSafetyMonitorTest, DeviationExactlyAtThreshold)
{
    // 閾值 2.0m，偏差恰好 2.0m → 應觸發（≥ 閾值）
    PositionSafetyMonitor monitor(2.0F);

    std::array<float, 3> current = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target  = {2.0F, 0.0F, 0.0F}; // 距離 2.0

    bool exceeded = monitor.is_deviation_exceeded(current, target);
    EXPECT_TRUE(exceeded);
}

TEST(PositionSafetyMonitorTest, CustomThreshold)
{
    // 自訂閾值 5.0m
    PositionSafetyMonitor monitor(5.0F);

    std::array<float, 3> current = {0.0F, 0.0F, 0.0F};
    std::array<float, 3> target  = {4.0F, 0.0F, 0.0F}; // 距離 4.0

    bool exceeded = monitor.is_deviation_exceeded(current, target);
    EXPECT_FALSE(exceeded);

    EXPECT_NEAR(monitor.get_threshold(), 5.0F, 1e-6F);
}

TEST(PositionSafetyMonitorTest, NegativeCoordinates)
{
    PositionSafetyMonitor monitor(3.0F);

    // 含負值的 NED 座標
    std::array<float, 3> current = {-10.0F, 5.0F, -2.0F};
    std::array<float, 3> target  = {-6.0F, 2.0F, -2.0F}; // dx=4, dy=-3, dz=0 → dist=5

    float dist = monitor.compute_distance(current, target);
    EXPECT_NEAR(dist, 5.0F, 1e-6F);

    bool exceeded = monitor.is_deviation_exceeded(current, target);
    EXPECT_TRUE(exceeded);
}
