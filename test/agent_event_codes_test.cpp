/**
 * @file agent_event_codes_test.cpp
 * @brief 事件代碼、嚴重度、節流閘門、布林轉換偵測與 failsafe 位元測試
 * @author Taiphoon Agent Team
 * @date 2026-10-30
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "controllers/Px4HealthFlags.hpp"
#include "safety/AgentEventCodes.hpp"

namespace
{

using taiphoon::safety::AGENT_EVENT_THROTTLE_NS;
using taiphoon::safety::AgentEventCode;
using taiphoon::safety::AgentEventGate;
using taiphoon::safety::AgentEventSeverity;
using taiphoon::safety::BooleanTransition;
using taiphoon::safety::BooleanTransitionDetector;
using taiphoon::safety::agent_event_code_name;
using taiphoon::safety::agent_event_severity;
using taiphoon::safety::is_state_transition_event;
using taiphoon::safety::px4_command_result_name;
using taiphoon::safety::to_px4_result_field;

constexpr int64_t START_NS = 1000000000LL;

/// 欄位與 px4_msgs/FailsafeFlags 對應的假結構，不依賴 px4_msgs。
struct FakeFailsafeFlags
{
    bool local_position_invalid{false};
    bool global_position_invalid{false};
    bool home_position_invalid{false};
    bool attitude_invalid{false};
    bool manual_control_signal_lost{false};
    bool offboard_control_signal_lost{false};
    uint8_t battery_warning{0U};
    bool battery_unhealthy{false};
    bool geofence_breached{false};
    bool fd_critical_failure{false};
    bool fd_esc_arming_failure{false};
    bool position_accuracy_low{false};
};

TEST(AgentEventCodesTest, CodeValuesMatchInterfaceContract)
{
    // 代碼是對外契約，只增不改。
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::COMMAND_UNKNOWN), 100U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::COMMAND_INVALID_PARAMS), 101U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::COMMAND_REJECTED), 102U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::COMMAND_FAILED), 103U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::PX4_COMMAND_REJECTED), 200U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::PX4_COMMAND_TIMEOUT), 201U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::PX4_COMMAND_ACCEPTED), 202U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::TAKEOFF_CANCELLED), 203U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::OFFBOARD_LOST), 204U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::PX4_STATUS_STALE), 300U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::PX4_STATUS_RECOVERED), 301U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::PX4_POSITION_STALE), 302U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::PX4_PREFLIGHT_FAILED), 303U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::PX4_FAILSAFE), 304U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::PX4_RESTART_DETECTED), 305U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::PLANNER_COMMAND_REJECTED), 400U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::PLANNER_EMERGENCY_BRAKE), 401U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::PLANNER_FAILURE_PERSISTENT), 402U);
    EXPECT_EQ(static_cast<uint16_t>(AgentEventCode::SAFETY_POSITION_DEVIATION), 500U);
}

TEST(AgentEventCodesTest, SeverityFollowsCodeTable)
{
    EXPECT_EQ(agent_event_severity(AgentEventCode::PX4_COMMAND_ACCEPTED),
              AgentEventSeverity::INFO);
    EXPECT_EQ(agent_event_severity(AgentEventCode::PX4_STATUS_RECOVERED),
              AgentEventSeverity::INFO);
    EXPECT_EQ(agent_event_severity(AgentEventCode::PX4_RESTART_DETECTED),
              AgentEventSeverity::INFO);
    EXPECT_EQ(agent_event_severity(AgentEventCode::COMMAND_FAILED),
              AgentEventSeverity::ERROR);
    EXPECT_EQ(agent_event_severity(AgentEventCode::PX4_COMMAND_REJECTED),
              AgentEventSeverity::ERROR);
    EXPECT_EQ(agent_event_severity(AgentEventCode::PX4_COMMAND_TIMEOUT),
              AgentEventSeverity::ERROR);
    EXPECT_EQ(agent_event_severity(AgentEventCode::PX4_STATUS_STALE),
              AgentEventSeverity::ERROR);
    EXPECT_EQ(agent_event_severity(AgentEventCode::PLANNER_FAILURE_PERSISTENT),
              AgentEventSeverity::ERROR);
    EXPECT_EQ(agent_event_severity(AgentEventCode::SAFETY_POSITION_DEVIATION),
              AgentEventSeverity::ERROR);
    EXPECT_EQ(agent_event_severity(AgentEventCode::COMMAND_UNKNOWN),
              AgentEventSeverity::WARNING);
    EXPECT_EQ(agent_event_severity(AgentEventCode::PX4_POSITION_STALE),
              AgentEventSeverity::WARNING);
    EXPECT_EQ(agent_event_severity(AgentEventCode::PLANNER_EMERGENCY_BRAKE),
              AgentEventSeverity::WARNING);
    // 嚴重度數值與 AgentEvent.SEVERITY_* 一致
    EXPECT_EQ(static_cast<uint8_t>(AgentEventSeverity::INFO), 0U);
    EXPECT_EQ(static_cast<uint8_t>(AgentEventSeverity::WARNING), 1U);
    EXPECT_EQ(static_cast<uint8_t>(AgentEventSeverity::ERROR), 2U);
}

TEST(AgentEventCodesTest, NamesAndPx4Results)
{
    EXPECT_STREQ(agent_event_code_name(AgentEventCode::PX4_RESTART_DETECTED),
                 "PX4_RESTART_DETECTED");
    EXPECT_STREQ(agent_event_code_name(static_cast<AgentEventCode>(999U)), "UNKNOWN");
    EXPECT_STREQ(px4_command_result_name(2U), "DENIED");
    EXPECT_STREQ(px4_command_result_name(200U), "UNKNOWN_RESULT");
    EXPECT_EQ(to_px4_result_field(4U), 4);
    EXPECT_EQ(to_px4_result_field(100000U), 32767);
}

TEST(AgentEventGateTest, ThrottlesSameCodeAndKeyForFiveSeconds)
{
    AgentEventGate gate;
    EXPECT_TRUE(gate.should_publish(AgentEventCode::PX4_COMMAND_REJECTED, "ARM", START_NS));
    EXPECT_FALSE(gate.should_publish(
        AgentEventCode::PX4_COMMAND_REJECTED, "ARM",
        START_NS + AGENT_EVENT_THROTTLE_NS - 1));
    EXPECT_TRUE(gate.should_publish(
        AgentEventCode::PX4_COMMAND_REJECTED, "ARM", START_NS + AGENT_EVENT_THROTTLE_NS));
}

TEST(AgentEventGateTest, DifferentKeyOrCodeIsIndependent)
{
    AgentEventGate gate;
    EXPECT_TRUE(gate.should_publish(AgentEventCode::PX4_COMMAND_REJECTED, "ARM", START_NS));
    EXPECT_TRUE(gate.should_publish(
        AgentEventCode::PX4_COMMAND_REJECTED, "MODE_OFFBOARD", START_NS));
    EXPECT_TRUE(gate.should_publish(AgentEventCode::PX4_COMMAND_TIMEOUT, "ARM", START_NS));
}

TEST(AgentEventGateTest, StateTransitionEventsAreNotThrottled)
{
    EXPECT_TRUE(is_state_transition_event(AgentEventCode::PX4_STATUS_STALE));
    EXPECT_TRUE(is_state_transition_event(AgentEventCode::PX4_STATUS_RECOVERED));
    EXPECT_TRUE(is_state_transition_event(AgentEventCode::PX4_PREFLIGHT_FAILED));
    EXPECT_TRUE(is_state_transition_event(AgentEventCode::PX4_FAILSAFE));
    EXPECT_FALSE(is_state_transition_event(AgentEventCode::PX4_POSITION_STALE));

    AgentEventGate gate;
    EXPECT_TRUE(gate.should_publish(AgentEventCode::PX4_STATUS_STALE, "", START_NS));
    EXPECT_TRUE(gate.should_publish(AgentEventCode::PX4_STATUS_STALE, "", START_NS + 1));
}

TEST(AgentEventGateTest, TimeGoingBackwardsDoesNotSuppressForever)
{
    AgentEventGate gate;
    EXPECT_TRUE(gate.should_publish(AgentEventCode::COMMAND_UNKNOWN, "", START_NS));
    EXPECT_TRUE(gate.should_publish(AgentEventCode::COMMAND_UNKNOWN, "", START_NS - 1));
}

TEST(AgentEventGateTest, SequenceStartsAtOneAndIncrements)
{
    AgentEventGate gate;
    EXPECT_EQ(gate.next_sequence(), 1U);
    EXPECT_EQ(gate.next_sequence(), 2U);
}

TEST(BooleanTransitionDetectorTest, FirstSampleOnlySetsBaseline)
{
    BooleanTransitionDetector detector;
    EXPECT_EQ(detector.update(true), BooleanTransition::NONE);
    EXPECT_EQ(detector.update(true), BooleanTransition::NONE);
    EXPECT_EQ(detector.update(false), BooleanTransition::BECAME_FALSE);
    EXPECT_EQ(detector.update(false), BooleanTransition::NONE);
    EXPECT_EQ(detector.update(true), BooleanTransition::BECAME_TRUE);

    detector.reset();
    EXPECT_EQ(detector.update(false), BooleanTransition::NONE);
}

TEST(Px4HealthFlagsTest, MapsEachFailsafeFieldToItsBit)
{
    FakeFailsafeFlags flags;
    EXPECT_EQ(taiphoon::controllers::make_failsafe_flag_bits(flags), 0U);

    flags.local_position_invalid = true;
    flags.offboard_control_signal_lost = true;
    flags.battery_warning = 2U;
    flags.position_accuracy_low = true;
    EXPECT_EQ(taiphoon::controllers::make_failsafe_flag_bits(flags),
              taiphoon::controllers::FAILSAFE_BIT_LOCAL_POSITION_INVALID |
              taiphoon::controllers::FAILSAFE_BIT_OFFBOARD_SIGNAL_LOST |
              taiphoon::controllers::FAILSAFE_BIT_BATTERY_WARNING |
              taiphoon::controllers::FAILSAFE_BIT_LOCAL_POSITION_ACCURACY_LOW);

    FakeFailsafeFlags all;
    all.local_position_invalid = true;
    all.global_position_invalid = true;
    all.home_position_invalid = true;
    all.attitude_invalid = true;
    all.manual_control_signal_lost = true;
    all.offboard_control_signal_lost = true;
    all.battery_warning = 1U;
    all.battery_unhealthy = true;
    all.geofence_breached = true;
    all.fd_critical_failure = true;
    all.fd_esc_arming_failure = true;
    all.position_accuracy_low = true;
    // 12 個位元與 AgentHealth.FAILSAFE_* 一致（1～2048）
    EXPECT_EQ(taiphoon::controllers::make_failsafe_flag_bits(all), 4095U);
}

}  // namespace
