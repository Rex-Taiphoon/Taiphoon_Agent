/**
 * @file flight_controller_reboot_test.cpp
 * @brief REBOOT_FC 安全條件（C2 表格與冷卻）、PX4 重新啟動偵測與連線轉換測試
 * @author Taiphoon Agent Team
 * @date 2026-10-30
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "controllers/FlightControllerReboot.hpp"

namespace
{

using taiphoon::controllers::FlightControllerRebootConditions;
using taiphoon::controllers::FlightControllerRebootDecision;
using taiphoon::controllers::PX4_POSITION_TIMEOUT_NS;
using taiphoon::controllers::PX4_REBOOT_PENDING_TIMEOUT_NS;
using taiphoon::controllers::PX4_STARTUP_GRACE_NS;
using taiphoon::controllers::PX4_STATE_TIMEOUT_NS;
using taiphoon::controllers::PX4_STATUS_GAP_WARN_NS;
using taiphoon::controllers::Px4LinkMonitor;
using taiphoon::controllers::Px4LinkTransition;
using taiphoon::controllers::Px4RestartDetector;
using taiphoon::controllers::evaluate_flight_controller_reboot;
using taiphoon::controllers::flight_controller_reboot_decision_detail;
using taiphoon::controllers::px4_message_age_s;

constexpr int64_t SECOND_NS = 1000000000LL;
constexpr int64_t START_NS = 100LL * SECOND_NS;
constexpr int64_t TICK_NS = SECOND_NS / 10;

FlightControllerRebootConditions make_conditions(const bool is_status_fresh,
                                                 const bool is_armed,
                                                 const bool is_landed,
                                                 const bool is_forced)
{
    FlightControllerRebootConditions conditions;
    conditions.is_status_fresh = is_status_fresh;
    conditions.is_armed = is_armed;
    conditions.is_land_detected_fresh = true;
    conditions.is_landed = is_landed;
    conditions.is_forced = is_forced;
    conditions.is_cooldown_active = false;
    return conditions;
}

// --- C2 表格 ---

TEST(FlightControllerRebootPolicyTest, DisarmedAndLandedSendsWithOrWithoutForce)
{
    EXPECT_EQ(evaluate_flight_controller_reboot(make_conditions(true, false, true, false)),
              FlightControllerRebootDecision::SEND);
    EXPECT_EQ(evaluate_flight_controller_reboot(make_conditions(true, false, true, true)),
              FlightControllerRebootDecision::SEND);
}

TEST(FlightControllerRebootPolicyTest, ArmedIsAlwaysRejectedEvenWithForce)
{
    EXPECT_EQ(evaluate_flight_controller_reboot(make_conditions(true, true, true, false)),
              FlightControllerRebootDecision::REJECT_ARMED);
    EXPECT_EQ(evaluate_flight_controller_reboot(make_conditions(true, true, true, true)),
              FlightControllerRebootDecision::REJECT_ARMED);
    EXPECT_EQ(evaluate_flight_controller_reboot(make_conditions(true, true, false, true)),
              FlightControllerRebootDecision::REJECT_ARMED);
}

TEST(FlightControllerRebootPolicyTest, DisarmedButNotLandedIsRejected)
{
    EXPECT_EQ(evaluate_flight_controller_reboot(make_conditions(true, false, false, false)),
              FlightControllerRebootDecision::REJECT_NOT_LANDED);
    EXPECT_EQ(evaluate_flight_controller_reboot(make_conditions(true, false, false, true)),
              FlightControllerRebootDecision::REJECT_NOT_LANDED);

    // land detected 逾時視同未確認落地
    auto conditions = make_conditions(true, false, true, true);
    conditions.is_land_detected_fresh = false;
    EXPECT_EQ(evaluate_flight_controller_reboot(conditions),
              FlightControllerRebootDecision::REJECT_NOT_LANDED);
}

TEST(FlightControllerRebootPolicyTest, UnknownStateNeedsForce)
{
    // 狀態不明時，解鎖與落地欄位都不可信，不參與判斷。
    EXPECT_EQ(evaluate_flight_controller_reboot(make_conditions(false, true, false, false)),
              FlightControllerRebootDecision::REJECT_STATE_UNKNOWN);
    EXPECT_EQ(evaluate_flight_controller_reboot(make_conditions(false, true, false, true)),
              FlightControllerRebootDecision::SEND);
    EXPECT_NE(std::string(flight_controller_reboot_decision_detail(
                  FlightControllerRebootDecision::REJECT_STATE_UNKNOWN)).find("REBOOT_FC 1"),
              std::string::npos);
}

TEST(FlightControllerRebootPolicyTest, CooldownRejectsEverything)
{
    for (const bool is_forced : {false, true})
    {
        auto conditions = make_conditions(true, false, true, is_forced);
        conditions.is_cooldown_active = true;
        EXPECT_EQ(evaluate_flight_controller_reboot(conditions),
                  FlightControllerRebootDecision::REJECT_COOLDOWN);
        conditions = make_conditions(false, false, false, is_forced);
        conditions.is_cooldown_active = true;
        EXPECT_EQ(evaluate_flight_controller_reboot(conditions),
                  FlightControllerRebootDecision::REJECT_COOLDOWN);
    }
    EXPECT_STREQ(flight_controller_reboot_decision_detail(
                     FlightControllerRebootDecision::SEND), "");
}

// --- 連線轉換 ---

TEST(Px4LinkMonitorTest, NeverReceivedAfterStartupGrace)
{
    Px4LinkMonitor monitor(PX4_STATE_TIMEOUT_NS, PX4_STARTUP_GRACE_NS);
    EXPECT_EQ(monitor.update(0, START_NS), Px4LinkTransition::NONE);
    EXPECT_EQ(monitor.update(0, START_NS + PX4_STARTUP_GRACE_NS - 1),
              Px4LinkTransition::NONE);
    EXPECT_FALSE(monitor.is_stale());
    EXPECT_EQ(monitor.update(0, START_NS + PX4_STARTUP_GRACE_NS),
              Px4LinkTransition::NEVER_RECEIVED);
    EXPECT_TRUE(monitor.is_stale());
    // 只回報一次
    EXPECT_EQ(monitor.update(0, START_NS + PX4_STARTUP_GRACE_NS + TICK_NS),
              Px4LinkTransition::NONE);

    const int64_t received_ns = START_NS + 10LL * SECOND_NS;
    EXPECT_EQ(monitor.update(received_ns, received_ns), Px4LinkTransition::RECOVERED);
    EXPECT_FALSE(monitor.is_stale());
}

TEST(Px4LinkMonitorTest, FirstFreshSampleIsNotRecovery)
{
    Px4LinkMonitor monitor(PX4_STATE_TIMEOUT_NS, PX4_STARTUP_GRACE_NS);
    EXPECT_EQ(monitor.update(START_NS, START_NS), Px4LinkTransition::NONE);
    EXPECT_EQ(monitor.update(START_NS, START_NS + PX4_STATE_TIMEOUT_NS),
              Px4LinkTransition::NONE);
    EXPECT_EQ(monitor.update(START_NS, START_NS + PX4_STATE_TIMEOUT_NS + 1),
              Px4LinkTransition::BECAME_STALE);
    EXPECT_EQ(monitor.update(START_NS, START_NS + PX4_STATE_TIMEOUT_NS + 2),
              Px4LinkTransition::NONE);
    const int64_t recovered_ns = START_NS + 2LL * PX4_STATE_TIMEOUT_NS;
    EXPECT_EQ(monitor.update(recovered_ns, recovered_ns), Px4LinkTransition::RECOVERED);
}

TEST(Px4LinkMonitorTest, PositionTimeoutIsOneSecond)
{
    EXPECT_EQ(PX4_POSITION_TIMEOUT_NS, SECOND_NS);
    Px4LinkMonitor monitor(PX4_POSITION_TIMEOUT_NS, PX4_STARTUP_GRACE_NS);
    EXPECT_EQ(monitor.update(START_NS, START_NS), Px4LinkTransition::NONE);
    EXPECT_EQ(monitor.update(START_NS, START_NS + SECOND_NS + 1),
              Px4LinkTransition::BECAME_STALE);
}

TEST(Px4LinkMonitorTest, AgeIsNegativeWhenNeverReceived)
{
    EXPECT_DOUBLE_EQ(px4_message_age_s(0, START_NS), -1.0);
    EXPECT_DOUBLE_EQ(px4_message_age_s(START_NS, START_NS + SECOND_NS / 2), 0.5);
}

// --- 重新啟動偵測 ---

TEST(Px4RestartDetectorTest, RecoveryWhileDisarmedIsRestart)
{
    Px4RestartDetector detector;
    EXPECT_TRUE(detector.update(Px4LinkTransition::RECOVERED, START_NS, false, START_NS));
    // 解鎖中恢復只是連線中斷，不重建狀態
    EXPECT_FALSE(detector.update(Px4LinkTransition::RECOVERED, START_NS, true, START_NS));
    EXPECT_FALSE(detector.update(Px4LinkTransition::NONE, START_NS, false, START_NS));
}

TEST(Px4RestartDetectorTest, AcceptedRebootWithStatusGapIsRestart)
{
    Px4RestartDetector detector;
    const int64_t accepted_ns = START_NS;
    detector.on_reboot_accepted(accepted_ns);
    EXPECT_TRUE(detector.is_reboot_pending());

    // 還沒中斷
    EXPECT_FALSE(detector.update(Px4LinkTransition::NONE, accepted_ns, false,
                                 accepted_ns + TICK_NS));
    // 中斷超過 PX4_STATUS_GAP_WARN_NS（未達 5 秒逾時，不會有 RECOVERED）
    EXPECT_FALSE(detector.update(Px4LinkTransition::NONE, accepted_ns, false,
                                 accepted_ns + PX4_STATUS_GAP_WARN_NS + TICK_NS));
    // 新的 VehicleStatus 到達
    const int64_t resumed_ns = accepted_ns + 3LL * SECOND_NS;
    EXPECT_TRUE(detector.update(Px4LinkTransition::NONE, resumed_ns, false, resumed_ns));
    EXPECT_FALSE(detector.is_reboot_pending());
}

TEST(Px4RestartDetectorTest, AcceptedRebootWithoutGapIsNotRestart)
{
    Px4RestartDetector detector;
    detector.on_reboot_accepted(START_NS);
    for (int64_t now_ns = START_NS; now_ns < START_NS + 5LL * SECOND_NS; now_ns += TICK_NS)
    {
        EXPECT_FALSE(detector.update(Px4LinkTransition::NONE, now_ns, false, now_ns));
    }
    EXPECT_TRUE(detector.is_reboot_pending());
}

TEST(Px4RestartDetectorTest, PendingRebootExpires)
{
    Px4RestartDetector detector;
    detector.on_reboot_accepted(START_NS);
    const int64_t expired_ns = START_NS + PX4_REBOOT_PENDING_TIMEOUT_NS + 1;
    EXPECT_FALSE(detector.update(Px4LinkTransition::NONE, START_NS, false, expired_ns));
    EXPECT_FALSE(detector.is_reboot_pending());
    // 放棄等待後恢復的新資料不算重新啟動
    EXPECT_FALSE(detector.update(Px4LinkTransition::NONE, expired_ns, false, expired_ns));
}

}  // namespace
