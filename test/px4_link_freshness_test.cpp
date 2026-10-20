/**
 * @file px4_link_freshness_test.cpp
 * @brief PX4 狀態新鮮度與 Offboard 輸出決策的邊界、遺失、離開模式與時間倒退測試
 * @author Taiphoon Agent Team
 * @date 2026-10-20
 */

#include <gtest/gtest.h>

#include <cstdint>

#include "controllers/Px4LinkFreshness.hpp"

namespace
{

using taiphoon::controllers::FlightStateResolver;
using taiphoon::controllers::OffboardOutputDecision;
using taiphoon::controllers::PX4_STATE_TIMEOUT_NS;
using taiphoon::controllers::PX4_STATUS_GAP_WARN_NS;
using taiphoon::controllers::evaluate_offboard_output;
using taiphoon::controllers::is_px4_message_fresh;
using taiphoon::controllers::is_px4_status_gap_notable;

constexpr int64_t SECOND_NS = 1000000000LL;
constexpr int64_t RECEIVED_NS = 100LL * SECOND_NS;
constexpr uint8_t NAV_OFFBOARD = FlightStateResolver::PX4_NAV_STATE_OFFBOARD;
constexpr uint8_t NAV_POSCTL = 2U;

TEST(Px4LinkFreshnessTest, TimeoutCoversObservedSitlStatusGap)
{
    // SITL 實測最大 VehicleStatus 間隔 3.084 s；門檻必須保留餘裕。
    EXPECT_GE(PX4_STATE_TIMEOUT_NS, 5LL * SECOND_NS);
    EXPECT_LT(PX4_STATUS_GAP_WARN_NS, PX4_STATE_TIMEOUT_NS);
}

TEST(Px4LinkFreshnessTest, FreshAtBoundaryAndStaleJustAfter)
{
    EXPECT_TRUE(is_px4_message_fresh(
        RECEIVED_NS, RECEIVED_NS, PX4_STATE_TIMEOUT_NS));
    EXPECT_TRUE(is_px4_message_fresh(
        RECEIVED_NS, RECEIVED_NS + PX4_STATE_TIMEOUT_NS,
        PX4_STATE_TIMEOUT_NS));
    EXPECT_FALSE(is_px4_message_fresh(
        RECEIVED_NS, RECEIVED_NS + PX4_STATE_TIMEOUT_NS + 1,
        PX4_STATE_TIMEOUT_NS));
}

TEST(Px4LinkFreshnessTest, NeverReceivedFutureSampleAndNegativeTimeoutAreStale)
{
    EXPECT_FALSE(is_px4_message_fresh(0, RECEIVED_NS, PX4_STATE_TIMEOUT_NS));
    EXPECT_FALSE(is_px4_message_fresh(-1, RECEIVED_NS, PX4_STATE_TIMEOUT_NS));
    EXPECT_FALSE(is_px4_message_fresh(
        RECEIVED_NS, RECEIVED_NS - 1, PX4_STATE_TIMEOUT_NS));
    EXPECT_FALSE(is_px4_message_fresh(RECEIVED_NS, RECEIVED_NS, -1));
}

TEST(Px4LinkFreshnessTest, ShortStatusGapKeepsOffboardOutput)
{
    // 回歸：3.1 s 未收到 VehicleStatus 不得再中止 Offboard。
    EXPECT_EQ(evaluate_offboard_output(
                  RECEIVED_NS, NAV_OFFBOARD,
                  RECEIVED_NS + 3100LL * 1000000LL, PX4_STATE_TIMEOUT_NS),
              OffboardOutputDecision::CONTINUE);
}

TEST(Px4LinkFreshnessTest, LongStatusLossStopsOffboardOutput)
{
    EXPECT_EQ(evaluate_offboard_output(
                  RECEIVED_NS, NAV_OFFBOARD,
                  RECEIVED_NS + PX4_STATE_TIMEOUT_NS + 1,
                  PX4_STATE_TIMEOUT_NS),
              OffboardOutputDecision::STOP_STATUS_STALE);
    EXPECT_EQ(evaluate_offboard_output(
                  0, NAV_OFFBOARD, RECEIVED_NS, PX4_STATE_TIMEOUT_NS),
              OffboardOutputDecision::STOP_STATUS_STALE);
}

TEST(Px4LinkFreshnessTest, LeavingOffboardStopsImmediately)
{
    // 遙控接管或 failsafe：只要收到非 Offboard 狀態就立即停止，不等逾時。
    EXPECT_EQ(evaluate_offboard_output(
                  RECEIVED_NS, NAV_POSCTL, RECEIVED_NS, PX4_STATE_TIMEOUT_NS),
              OffboardOutputDecision::STOP_LEFT_OFFBOARD);
    EXPECT_EQ(evaluate_offboard_output(
                  RECEIVED_NS, FlightStateResolver::PX4_NAV_STATE_AUTO_RTL,
                  RECEIVED_NS + SECOND_NS, PX4_STATE_TIMEOUT_NS),
              OffboardOutputDecision::STOP_LEFT_OFFBOARD);
}

TEST(Px4LinkFreshnessTest, StaleTakesPrecedenceOverLastKnownNavState)
{
    // 過期狀態的 nav_state 已不可信，回報為過期而非離開 Offboard。
    EXPECT_EQ(evaluate_offboard_output(
                  RECEIVED_NS, NAV_POSCTL,
                  RECEIVED_NS + PX4_STATE_TIMEOUT_NS + 1,
                  PX4_STATE_TIMEOUT_NS),
              OffboardOutputDecision::STOP_STATUS_STALE);
}

TEST(Px4LinkFreshnessTest, StatusGapWarningOnlyAboveThreshold)
{
    EXPECT_FALSE(is_px4_status_gap_notable(
        0, RECEIVED_NS, PX4_STATUS_GAP_WARN_NS));
    EXPECT_FALSE(is_px4_status_gap_notable(
        RECEIVED_NS, RECEIVED_NS + PX4_STATUS_GAP_WARN_NS,
        PX4_STATUS_GAP_WARN_NS));
    EXPECT_TRUE(is_px4_status_gap_notable(
        RECEIVED_NS, RECEIVED_NS + PX4_STATUS_GAP_WARN_NS + 1,
        PX4_STATUS_GAP_WARN_NS));
    EXPECT_FALSE(is_px4_status_gap_notable(
        RECEIVED_NS, RECEIVED_NS - SECOND_NS, PX4_STATUS_GAP_WARN_NS));
}

}  // namespace
