/**
 * @file peer_clock_skew_monitor_test.cpp
 * @brief 同儕時鐘差估計的正常、邊界、NaN／Inf、過期、時間倒退與恢復測試
 * @author Taiphoon Agent Team
 * @date 2026-10-20
 */

#include <gtest/gtest.h>

#include <limits>

#include "ego_swarm/PeerClockSkewMonitor.hpp"

namespace
{

using taiphoon::ego_swarm::PeerClockSkewEstimate;
using taiphoon::ego_swarm::PeerClockSkewMonitor;
using taiphoon::ego_swarm::peer_clock_skew_exceeds;

constexpr double NAN_VALUE = std::numeric_limits<double>::quiet_NaN();
constexpr double INF_VALUE = std::numeric_limits<double>::infinity();

TEST(PeerClockSkewMonitorTest, MinimumDifferenceFiltersNetworkLatency)
{
    PeerClockSkewMonitor monitor;
    // 同儕時鐘慢 2 秒，網路延遲介於 5～40 ms。
    EXPECT_TRUE(monitor.record(2, 98.000, 100.040));
    EXPECT_TRUE(monitor.record(2, 98.100, 100.105));
    EXPECT_TRUE(monitor.record(2, 98.200, 100.230));

    const auto estimate = monitor.estimate(2, 100.300);
    ASSERT_TRUE(estimate.is_valid);
    EXPECT_NEAR(estimate.offset_s, 2.005, 1.0e-9);
    EXPECT_EQ(estimate.sample_count, 3U);
}

TEST(PeerClockSkewMonitorTest, PeerAheadProducesNegativeOffset)
{
    PeerClockSkewMonitor monitor;
    EXPECT_TRUE(monitor.record(3, 103.500, 100.010));
    const auto estimate = monitor.estimate(3, 100.010);
    ASSERT_TRUE(estimate.is_valid);
    EXPECT_NEAR(estimate.offset_s, -3.490, 1.0e-9);
}

TEST(PeerClockSkewMonitorTest, MissingPeerIsInvalid)
{
    const PeerClockSkewMonitor monitor;
    const auto estimate = monitor.estimate(4, 100.0);
    EXPECT_FALSE(estimate.is_valid);
    EXPECT_EQ(estimate.sample_count, 0U);
}

TEST(PeerClockSkewMonitorTest, RejectsInvalidAgentAndNonFiniteInput)
{
    PeerClockSkewMonitor monitor;
    EXPECT_FALSE(monitor.record(0, 1.0, 1.0));
    EXPECT_FALSE(monitor.record(-1, 1.0, 1.0));
    EXPECT_FALSE(monitor.record(2, NAN_VALUE, 1.0));
    EXPECT_FALSE(monitor.record(2, 1.0, NAN_VALUE));
    EXPECT_FALSE(monitor.record(2, INF_VALUE, 1.0));
    EXPECT_FALSE(monitor.record(2, 1.0, -INF_VALUE));
    EXPECT_FALSE(monitor.estimate(2, 1.0).is_valid);
    ASSERT_TRUE(monitor.record(2, 1.0, 1.0));
    EXPECT_FALSE(monitor.estimate(2, NAN_VALUE).is_valid);
    EXPECT_FALSE(monitor.estimate(2, INF_VALUE).is_valid);
}

TEST(PeerClockSkewMonitorTest, StaleSamplesExpireOutsideWindow)
{
    PeerClockSkewMonitor monitor(5.0);
    ASSERT_TRUE(monitor.record(2, 90.0, 100.0));
    EXPECT_TRUE(monitor.estimate(2, 105.0).is_valid);
    EXPECT_FALSE(monitor.estimate(2, 105.001).is_valid);
}

TEST(PeerClockSkewMonitorTest, RecoversAfterTimeSyncCorrection)
{
    PeerClockSkewMonitor monitor(5.0);
    ASSERT_TRUE(monitor.record(2, 90.0, 100.0));
    ASSERT_TRUE(monitor.estimate(2, 100.0).is_valid);
    EXPECT_NEAR(monitor.estimate(2, 100.0).offset_s, 10.0, 1.0e-9);

    // 對時完成後新樣本差值趨近 0，舊的 10 秒樣本離開時間窗後不再影響估計。
    ASSERT_TRUE(monitor.record(2, 105.995, 106.0));
    const auto estimate = monitor.estimate(2, 106.0);
    ASSERT_TRUE(estimate.is_valid);
    EXPECT_NEAR(estimate.offset_s, 0.005, 1.0e-9);
    EXPECT_EQ(estimate.sample_count, 1U);
}

TEST(PeerClockSkewMonitorTest, LocalClockStepBackwardClearsOldSamples)
{
    PeerClockSkewMonitor monitor(5.0);
    ASSERT_TRUE(monitor.record(2, 199.0, 200.0));
    // 本機時間倒退：新樣本早於既有樣本，舊樣本的時間基準失效。
    ASSERT_TRUE(monitor.record(2, 149.98, 150.0));
    const auto estimate = monitor.estimate(2, 150.0);
    ASSERT_TRUE(estimate.is_valid);
    EXPECT_EQ(estimate.sample_count, 1U);
    EXPECT_NEAR(estimate.offset_s, 0.02, 1.0e-9);
}

TEST(PeerClockSkewMonitorTest, FutureSamplesAreIgnoredWhenQueryTimeIsEarlier)
{
    PeerClockSkewMonitor monitor(5.0);
    ASSERT_TRUE(monitor.record(2, 199.0, 200.0));
    EXPECT_FALSE(monitor.estimate(2, 150.0).is_valid);
}

TEST(PeerClockSkewMonitorTest, SampleCountIsBounded)
{
    PeerClockSkewMonitor monitor(100.0, 3U);
    ASSERT_TRUE(monitor.record(2, 0.0, 10.0));
    ASSERT_TRUE(monitor.record(2, 10.9, 11.0));
    ASSERT_TRUE(monitor.record(2, 11.9, 12.0));
    ASSERT_TRUE(monitor.record(2, 12.9, 13.0));
    const auto estimate = monitor.estimate(2, 13.0);
    ASSERT_TRUE(estimate.is_valid);
    EXPECT_EQ(estimate.sample_count, 3U);
    // 最舊的 10 秒樣本已被容量上限移除。
    EXPECT_NEAR(estimate.offset_s, 0.1, 1.0e-9);
}

TEST(PeerClockSkewMonitorTest, PeersAreIndependentAndErasable)
{
    PeerClockSkewMonitor monitor;
    ASSERT_TRUE(monitor.record(2, 99.0, 100.0));
    ASSERT_TRUE(monitor.record(3, 100.0, 100.0));
    EXPECT_NEAR(monitor.estimate(2, 100.0).offset_s, 1.0, 1.0e-9);
    EXPECT_NEAR(monitor.estimate(3, 100.0).offset_s, 0.0, 1.0e-9);
    EXPECT_TRUE(monitor.erase(2));
    EXPECT_FALSE(monitor.erase(2));
    EXPECT_FALSE(monitor.estimate(2, 100.0).is_valid);
    EXPECT_TRUE(monitor.estimate(3, 100.0).is_valid);
}

TEST(PeerClockSkewMonitorTest, InvalidConstructorArgumentsUseDefaults)
{
    const PeerClockSkewMonitor nan_window(NAN_VALUE, 0U);
    EXPECT_DOUBLE_EQ(nan_window.window_s(),
                     taiphoon::ego_swarm::DEFAULT_PEER_CLOCK_SKEW_WINDOW_S);
    const PeerClockSkewMonitor negative_window(-1.0);
    EXPECT_DOUBLE_EQ(negative_window.window_s(),
                     taiphoon::ego_swarm::DEFAULT_PEER_CLOCK_SKEW_WINDOW_S);
}

TEST(PeerClockSkewMonitorTest, ExceedsUsesAbsoluteValueAndStrictBoundary)
{
    EXPECT_FALSE(peer_clock_skew_exceeds(PeerClockSkewEstimate{true, 1.0, 1U}, 1.0));
    EXPECT_TRUE(peer_clock_skew_exceeds(PeerClockSkewEstimate{true, 1.001, 1U}, 1.0));
    EXPECT_TRUE(peer_clock_skew_exceeds(PeerClockSkewEstimate{true, -1.001, 1U}, 1.0));
    EXPECT_FALSE(peer_clock_skew_exceeds(PeerClockSkewEstimate{false, 5.0, 0U}, 1.0));
    EXPECT_FALSE(peer_clock_skew_exceeds(PeerClockSkewEstimate{true, NAN_VALUE, 1U}, 1.0));
    EXPECT_FALSE(peer_clock_skew_exceeds(PeerClockSkewEstimate{true, 5.0, 1U}, NAN_VALUE));
    EXPECT_FALSE(peer_clock_skew_exceeds(PeerClockSkewEstimate{true, 5.0, 1U}, -1.0));
}

}  // namespace
