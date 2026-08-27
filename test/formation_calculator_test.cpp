/**
 * @file formation_calculator_test.cpp
 * @brief FormationCalculator 單元測試（gtest）
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 測試 FormationCalculator 的線上計數、質心計算與編隊目標位置計算。
 * 需在 colcon 環境中編譯執行：colcon test --packages-select taiphoon_agent
 */

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include "formation/FormationCalculator.hpp"

using taiphoon::formation::FormationCalculator;
using taiphoon::formation::FormationType;
using taiphoon::formation::FormationParams;
using taiphoon::formation::PeerPosition;

namespace
{

/// 測試用基準 GPS：位於台北市區
constexpr double TEST_BASE_LAT = 25.03996592258936;
constexpr double TEST_BASE_LON = 121.51195163336611;

/// 等距圓柱投影：該緯度下經度每 1 度對應的東向距離（公尺）
double lon_per_deg(double lat_deg)
{
    return 111320.0 * std::cos(lat_deg * 3.14159265358979323846 / 180.0);
}

}  // namespace

// ==================== count_online 測試 ====================

TEST(FormationCalculatorTest, CountOnlineAllOffline)
{
    std::vector<PeerPosition> peers = {
        {{0, 0, 0}, false},
        {{0, 0, 0}, false},
    };
    EXPECT_EQ(FormationCalculator::count_online(peers), 0);
}

TEST(FormationCalculatorTest, CountOnlineMixed)
{
    std::vector<PeerPosition> peers = {
        {{0, 0, 0}, true},
        {{0, 0, 0}, false},
        {{0, 0, 0}, true},
    };
    EXPECT_EQ(FormationCalculator::count_online(peers), 2);
}

// ==================== compute_centroid 測試 ====================

TEST(FormationCalculatorTest, ComputeCentroidSinglePeer)
{
    std::vector<PeerPosition> peers = {
        {{10.0F, 20.0F, -3.0F}, true},
    };
    auto c = FormationCalculator::compute_centroid(peers);
    EXPECT_NEAR(c[0], 10.0F, 1e-6F);
    EXPECT_NEAR(c[1], 20.0F, 1e-6F);
    EXPECT_NEAR(c[2], -3.0F, 1e-6F);
}

TEST(FormationCalculatorTest, ComputeCentroidMultiplePeers)
{
    std::vector<PeerPosition> peers = {
        {{0.0F, 0.0F, 0.0F}, true},
        {{10.0F, 10.0F, 10.0F}, true},
        {{20.0F, 20.0F, 20.0F}, true},
    };
    auto c = FormationCalculator::compute_centroid(peers);
    EXPECT_NEAR(c[0], 10.0F, 1e-6F);
    EXPECT_NEAR(c[1], 10.0F, 1e-6F);
    EXPECT_NEAR(c[2], 10.0F, 1e-6F);
}

TEST(FormationCalculatorTest, ComputeCentroidSkipsOffline)
{
    std::vector<PeerPosition> peers = {
        {{0.0F, 0.0F, 0.0F}, true},
        {{100.0F, 100.0F, 100.0F}, false},
        {{20.0F, 20.0F, 20.0F}, true},
    };
    auto c = FormationCalculator::compute_centroid(peers);
    EXPECT_NEAR(c[0], 10.0F, 1e-6F);
    EXPECT_NEAR(c[1], 10.0F, 1e-6F);
    EXPECT_NEAR(c[2], 10.0F, 1e-6F);
}

TEST(FormationCalculatorTest, ComputeCentroidEmpty)
{
    std::vector<PeerPosition> peers;
    auto c = FormationCalculator::compute_centroid(peers);
    EXPECT_NEAR(c[0], 0.0F, 1e-6F);
    EXPECT_NEAR(c[1], 0.0F, 1e-6F);
    EXPECT_NEAR(c[2], 0.0F, 1e-6F);
}

// ==================== compute_formation_target 測試 ====================

TEST(FormationCalculatorTest, FormationCircle)
{
    // 3 架無人機，環形半徑 5m，高度 4m；自身 index=0 → 角度 0°
    std::array<float, 3> centroid = {0.0F, 0.0F, 0.0F};
    FormationParams params;
    params.param1 = 5.0F;  // 半徑
    params.param2 = 4.0F;  // 高度

    auto t = FormationCalculator::compute_formation_target(
        centroid, FormationType::CIRCLE, params, 0, 3);

    EXPECT_NEAR(t[0], 5.0F, 1e-4F);
    EXPECT_NEAR(t[1], 0.0F, 1e-4F);
    EXPECT_NEAR(t[2], -4.0F, 1e-4F);
    EXPECT_TRUE(std::isnan(t[3]));
}

TEST(FormationCalculatorTest, FormationCircleIndex1)
{
    // 3 架無人機，半徑 5m，自身 index=1 → 角度 120°
    std::array<float, 3> centroid = {0.0F, 0.0F, 0.0F};
    FormationParams params;
    params.param1 = 5.0F;
    params.param2 = 4.0F;

    auto t = FormationCalculator::compute_formation_target(
        centroid, FormationType::CIRCLE, params, 1, 3);

    constexpr float PI = 3.14159265358979323846F;
    float angle = 2.0F * PI / 3.0F;
    EXPECT_NEAR(t[0], 5.0F * std::cos(angle), 1e-4F);
    EXPECT_NEAR(t[1], 5.0F * std::sin(angle), 1e-4F);
    EXPECT_NEAR(t[2], -4.0F, 1e-4F);
}

TEST(FormationCalculatorTest, FormationHoriLinear)
{
    // 3 架無人機，水平直列，間距 3m，高度 5m
    std::array<float, 3> centroid = {10.0F, 20.0F, 30.0F};
    FormationParams params;
    params.param1 = 3.0F;  // 間距
    params.param2 = 5.0F;  // 高度

    auto t0 = FormationCalculator::compute_formation_target(
        centroid, FormationType::HORI_LINEAR, params, 0, 3);
    EXPECT_NEAR(t0[0], 10.0F, 1e-4F);
    EXPECT_NEAR(t0[1], 17.0F, 1e-4F);
    EXPECT_NEAR(t0[2], -5.0F, 1e-4F);
    EXPECT_TRUE(std::isnan(t0[3]));

    auto t1 = FormationCalculator::compute_formation_target(
        centroid, FormationType::HORI_LINEAR, params, 1, 3);
    EXPECT_NEAR(t1[1], 20.0F, 1e-4F);

    auto t2 = FormationCalculator::compute_formation_target(
        centroid, FormationType::HORI_LINEAR, params, 2, 3);
    EXPECT_NEAR(t2[1], 23.0F, 1e-4F);
}

TEST(FormationCalculatorTest, FormationVertLinear)
{
    // 2 架無人機，垂直直列，間距 2m
    std::array<float, 3> centroid = {5.0F, 5.0F, -10.0F};
    FormationParams params;
    params.param1 = 2.0F;  // 間距
    params.param2 = 0.0F;  // (unused for VERT_LINEAR)

    auto t0 = FormationCalculator::compute_formation_target(
        centroid, FormationType::VERT_LINEAR, params, 0, 2);
    EXPECT_NEAR(t0[0], 5.0F, 1e-4F);
    EXPECT_NEAR(t0[1], 5.0F, 1e-4F);
    EXPECT_NEAR(t0[2], -11.0F, 1e-4F);
    EXPECT_TRUE(std::isnan(t0[3]));

    auto t1 = FormationCalculator::compute_formation_target(
        centroid, FormationType::VERT_LINEAR, params, 1, 2);
    EXPECT_NEAR(t1[2], -9.0F, 1e-4F);
}

TEST(FormationCalculatorTest, FormationNone)
{
    std::array<float, 3> centroid = {1.0F, 2.0F, 3.0F};
    FormationParams params;
    auto t = FormationCalculator::compute_formation_target(
        centroid, FormationType::NONE, params, 0, 3);
    EXPECT_NEAR(t[0], 1.0F, 1e-6F);
    EXPECT_NEAR(t[1], 2.0F, 1e-6F);
    EXPECT_NEAR(t[2], 3.0F, 1e-6F);
    EXPECT_TRUE(std::isnan(t[3]));
}

// ==================== 編隊幾何安全測試 ====================

TEST(FormationCalculatorTest, CircleRejectsSlotsCloserThanSafetyMargin)
{
    FormationParams params;
    params.param1 = 1.0F;

    EXPECT_FALSE(FormationCalculator::is_formation_geometry_safe(
        FormationType::CIRCLE, params, 3, 3.0F));
}

TEST(FormationCalculatorTest, CircleAcceptsSafetyMarginBoundary)
{
    FormationParams params;
    params.param1 = 1.5F;

    EXPECT_TRUE(FormationCalculator::is_formation_geometry_safe(
        FormationType::CIRCLE, params, 2, 3.0F));
}

TEST(FormationCalculatorTest, HoriLinearRejectsSlotsCloserThanSafetyMargin)
{
    FormationParams params;
    params.param1 = 2.99F;

    EXPECT_FALSE(FormationCalculator::is_formation_geometry_safe(
        FormationType::HORI_LINEAR, params, 4, 3.0F));
}

TEST(FormationCalculatorTest, HoriLinearAcceptsSafetyMarginBoundary)
{
    FormationParams params;
    params.param1 = 3.0F;

    EXPECT_TRUE(FormationCalculator::is_formation_geometry_safe(
        FormationType::HORI_LINEAR, params, 4, 3.0F));
}

TEST(FormationCalculatorTest, DoesNotAllowMinimumDistanceBelowThreeMeters)
{
    FormationParams params;
    params.param1 = 2.0F;

    EXPECT_FALSE(FormationCalculator::is_formation_geometry_safe(
        FormationType::HORI_LINEAR, params, 4U, 1.0F));
}

TEST(FormationCalculatorTest, VertLinearRejectsSlotsCloserThanSafetyMargin)
{
    FormationParams params;
    params.param1 = 2.99F;

    EXPECT_FALSE(FormationCalculator::is_formation_geometry_safe(
        FormationType::VERT_LINEAR, params, 4, 3.0F));
}

TEST(FormationCalculatorTest, VertLinearAcceptsSafetyMarginBoundary)
{
    FormationParams params;
    params.param1 = 3.0F;

    EXPECT_TRUE(FormationCalculator::is_formation_geometry_safe(
        FormationType::VERT_LINEAR, params, 4, 3.0F));
}

// ==================== WGS84 交換後轉入 receiver-local frame 測試 ====================

TEST(FormationCalculatorTest, GpsOffsetMetersEast)
{
    // 自身 GPS；同儕位於自身東方 3 公尺、高度高 20 公尺
    std::array<float, 3> own_global = {
        static_cast<float>(TEST_BASE_LAT),
        static_cast<float>(TEST_BASE_LON), 100.0F};
    std::array<float, 3> peer_global = {
        own_global[0],
        own_global[1] + 3.0F / static_cast<float>(lon_per_deg(TEST_BASE_LAT)),
        120.0F};

    auto offset = FormationCalculator::gps_offset_meters(own_global, peer_global);
    EXPECT_NEAR(offset[0], 0.0F, 1e-3F);
    // 容差 0.5m：lat/lon 以 float32 儲存，在 25°N 有約 0.3m 的量化誤差
    EXPECT_NEAR(offset[1], 3.0F, 0.5F);
    EXPECT_NEAR(offset[2], -20.0F, 1e-3F);
}

TEST(FormationCalculatorTest, GpsOffsetMetersNorth)
{
    // 同儕位於自身北方 5 公尺、同高度
    std::array<float, 3> own_global = {
        static_cast<float>(TEST_BASE_LAT),
        static_cast<float>(TEST_BASE_LON), 100.0F};
    std::array<float, 3> peer_global = {
        own_global[0] + 5.0F / 111320.0F,
        own_global[1], 100.0F};

    auto offset = FormationCalculator::gps_offset_meters(own_global, peer_global);
    // 容差 0.5m：float32 緯度量化誤差
    EXPECT_NEAR(offset[0], 5.0F, 0.5F);
    EXPECT_NEAR(offset[1], 0.0F, 1e-3F);
    EXPECT_NEAR(offset[2], 0.0F, 1e-3F);
}

TEST(FormationCalculatorTest, GpsOffsetMetersInvalidInput)
{
    // GPS 無效（NaN）時回傳零偏移，不產生 NaN 汙染
    std::array<float, 3> own_global = {
        std::numeric_limits<float>::quiet_NaN(),
        static_cast<float>(TEST_BASE_LON), 100.0F};
    std::array<float, 3> peer_global = {
        static_cast<float>(TEST_BASE_LAT),
        static_cast<float>(TEST_BASE_LON), 100.0F};

    auto offset = FormationCalculator::gps_offset_meters(own_global, peer_global);
    EXPECT_NEAR(offset[0], 0.0F, 1e-6F);
    EXPECT_NEAR(offset[1], 0.0F, 1e-6F);
    EXPECT_FALSE(std::isnan(offset[0]));
}

TEST(FormationCalculatorTest, ComputeCommonFramePosition)
{
    // 自身 local 位置 (1, 2, -3)；同儕位於自身東方 5 公尺、同高度
    std::array<float, 3> own_ned = {1.0F, 2.0F, -3.0F};
    std::array<float, 3> own_global = {
        static_cast<float>(TEST_BASE_LAT),
        static_cast<float>(TEST_BASE_LON), 100.0F};
    std::array<float, 3> peer_global = {
        own_global[0],
        own_global[1] + 5.0F / static_cast<float>(lon_per_deg(TEST_BASE_LAT)),
        100.0F};

    std::array<float, 3> common{};
    bool ok = FormationCalculator::compute_common_frame_position(
        own_ned, own_global, peer_global, common);

    EXPECT_TRUE(ok);
    EXPECT_NEAR(common[0], 1.0F, 1e-3F);
    // 容差 0.5m：float32 經度量化誤差
    EXPECT_NEAR(common[1], 7.0F, 0.5F);
    EXPECT_NEAR(common[2], -3.0F, 1e-3F);
}

TEST(FormationCalculatorTest, ComputeCommonFrameSameHome)
{
    // 同 HOME（GPS 相同）：共同位置 = 自身位置（退化為原行為）
    std::array<float, 3> own_ned = {1.0F, 2.0F, -3.0F};
    std::array<float, 3> same_global = {
        static_cast<float>(TEST_BASE_LAT),
        static_cast<float>(TEST_BASE_LON), 100.0F};

    std::array<float, 3> common{};
    bool ok = FormationCalculator::compute_common_frame_position(
        own_ned, same_global, same_global, common);

    EXPECT_TRUE(ok);
    EXPECT_NEAR(common[0], 1.0F, 1e-6F);
    EXPECT_NEAR(common[1], 2.0F, 1e-6F);
    EXPECT_NEAR(common[2], -3.0F, 1e-6F);
}

TEST(FormationCalculatorTest, ComputeCommonFrameInvalidGps)
{
    // 同儕 GPS 無效 → 換算失敗，呼叫端應退回原算法
    std::array<float, 3> own_ned = {1.0F, 2.0F, -3.0F};
    std::array<float, 3> own_global = {
        static_cast<float>(TEST_BASE_LAT),
        static_cast<float>(TEST_BASE_LON), 100.0F};
    std::array<float, 3> peer_global = {
        std::numeric_limits<float>::quiet_NaN(),
        static_cast<float>(TEST_BASE_LON), 100.0F};

    std::array<float, 3> common{};
    EXPECT_FALSE(FormationCalculator::compute_common_frame_position(
        own_ned, own_global, peer_global, common));
}

// ==================== 同儕 state 時效判定測試 ====================

TEST(FormationCalculatorTest, PeerStaleBoundary)
{
    using namespace std::chrono;

    auto now = steady_clock::now();

    // 從未收到過 state → 視為逾時（離線）
    EXPECT_TRUE(FormationCalculator::is_peer_stale(
        steady_clock::time_point{}, now, 2.0));

    // 1.9 秒前收到 → 未逾時（線上）
    auto recent = now - milliseconds(1900);
    EXPECT_FALSE(FormationCalculator::is_peer_stale(recent, now, 2.0));

    // 2.1 秒前收到 → 逾時（離線）
    auto old = now - milliseconds(2100);
    EXPECT_TRUE(FormationCalculator::is_peer_stale(old, now, 2.0));

    // 剛收到（0 毫秒前）→ 未逾時
    EXPECT_FALSE(FormationCalculator::is_peer_stale(now, now, 2.0));
}

// ==================== 編隊質心快照測試 ====================

TEST(FormationCalculatorTest, ComputeReferenceCentroidGpsCorrected)
{
    // 四機 HOME 東向錯位 0/3/6/9m；自身 = uav_1（local 原點 = 自身 HOME）
    std::array<float, 3> own_ned = {0.0F, 0.0F, -8.0F};
    std::array<float, 3> own_global = {
        static_cast<float>(TEST_BASE_LAT),
        static_cast<float>(TEST_BASE_LON), 100.0F};

    auto make_east_global = [&](float east_m) -> std::array<float, 3>
    {
        return {
            own_global[0],
            own_global[1] + east_m /
                static_cast<float>(lon_per_deg(TEST_BASE_LAT)),
            100.0F};
    };

    std::vector<PeerPosition> peers = {
        {{0.0F, 0.0F, -8.0F}, true, own_global},
        {{0.0F, 0.0F, -8.0F}, true, make_east_global(3.0F)},
        {{0.0F, 0.0F, -8.0F}, true, make_east_global(6.0F)},
        {{0.0F, 0.0F, -8.0F}, true, make_east_global(9.0F)},
    };

    auto ref = FormationCalculator::compute_reference_centroid(
        peers, own_ned, own_global);

    EXPECT_TRUE(ref.gps_based);
    EXPECT_NEAR(ref.centroid[0], 0.0F, 1e-3F);
    // receiver-local 質心 = 自身位置 + 東向偏移平均（0+3+6+9)/4 = 4.5m
    EXPECT_NEAR(ref.centroid[1], 4.5F, 0.5F);
    EXPECT_NEAR(ref.centroid[2], -8.0F, 1e-3F);
}

TEST(FormationCalculatorTest, ComputeReferenceCentroidSameHome)
{
    // 同 HOME（各機 HOME 相同）：GPS 換算退化為原混框行為（直接平均 local 位置）
    std::array<float, 3> own_ned = {0.0F, 0.0F, -8.0F};
    std::array<float, 3> own_global = {
        static_cast<float>(TEST_BASE_LAT),
        static_cast<float>(TEST_BASE_LON), 100.0F};

    // 同儕 local 位置與 GPS 一致（GPS = 自身 GPS + local 偏移）
    std::array<float, 3> peer2_global = {
        own_global[0] + 10.0F / 111320.0F,
        own_global[1], 100.0F};
    std::array<float, 3> peer3_global = {
        own_global[0],
        own_global[1] + 10.0F /
            static_cast<float>(lon_per_deg(TEST_BASE_LAT)),
        100.0F};

    std::vector<PeerPosition> peers = {
        {{0.0F, 0.0F, -8.0F}, true, own_global},
        {{10.0F, 0.0F, -8.0F}, true, peer2_global},
        {{0.0F, 10.0F, -8.0F}, true, peer3_global},
    };

    auto ref = FormationCalculator::compute_reference_centroid(
        peers, own_ned, own_global);

    EXPECT_TRUE(ref.gps_based);
    // 容差 0.5m：float32 緯度/經度量化誤差
    EXPECT_NEAR(ref.centroid[0], 10.0F / 3.0F, 0.5F);
    EXPECT_NEAR(ref.centroid[1], 10.0F / 3.0F, 0.5F);
    EXPECT_NEAR(ref.centroid[2], -8.0F, 1e-3F);
}

TEST(FormationCalculatorTest, ComputeReferenceCentroidGpsInvalidFallback)
{
    // 任一同儕 GPS 無效 → 退回原混框算法（gps_based=false）
    std::array<float, 3> own_ned = {0.0F, 0.0F, -8.0F};
    std::array<float, 3> own_global = {
        static_cast<float>(TEST_BASE_LAT),
        static_cast<float>(TEST_BASE_LON), 100.0F};
    std::array<float, 3> nan_global = {
        std::numeric_limits<float>::quiet_NaN(),
        static_cast<float>(TEST_BASE_LON), 100.0F};

    std::vector<PeerPosition> peers = {
        {{0.0F, 0.0F, -8.0F}, true, own_global},
        {{10.0F, 4.0F, -8.0F}, true, nan_global},
    };

    auto ref = FormationCalculator::compute_reference_centroid(
        peers, own_ned, own_global);

    EXPECT_FALSE(ref.gps_based);
    EXPECT_NEAR(ref.centroid[0], 5.0F, 1e-4F);
    EXPECT_NEAR(ref.centroid[1], 2.0F, 1e-4F);
    EXPECT_NEAR(ref.centroid[2], -8.0F, 1e-4F);
}

TEST(FormationCalculatorTest, ComputeReferenceCentroidSkipsOffline)
{
    // 離線同儕不參與快照；自身 GPS 有效即可 GPS-based
    std::array<float, 3> own_ned = {0.0F, 0.0F, -8.0F};
    std::array<float, 3> own_global = {
        static_cast<float>(TEST_BASE_LAT),
        static_cast<float>(TEST_BASE_LON), 100.0F};
    std::array<float, 3> nan_global = {
        std::numeric_limits<float>::quiet_NaN(),
        static_cast<float>(TEST_BASE_LON), 100.0F};

    std::vector<PeerPosition> peers = {
        {{0.0F, 0.0F, -8.0F}, true, own_global},
        {{10.0F, 10.0F, -8.0F}, false, nan_global},
    };

    auto ref = FormationCalculator::compute_reference_centroid(
        peers, own_ned, own_global);

    EXPECT_TRUE(ref.gps_based);
    EXPECT_NEAR(ref.centroid[0], 0.0F, 1e-6F);
    EXPECT_NEAR(ref.centroid[1], 0.0F, 1e-6F);
}
