/**
 * @file test_state_publisher.cpp
 * @brief StatePublisher 單元測試（含 GPS 與電池擴充測試）
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 測試 StatePublisher 的 PX4 odometry / GPS / Battery → AgentState/AgentAttitude 轉譯邏輯。
 * 需在 colcon 環境中編譯執行：colcon test --packages-select taiphoon_agent
 */

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "safety/StatePublisher.hpp"
#include "taiphoon_agent/msg/agent_state.hpp"
#include "taiphoon_agent/msg/agent_attitude.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "controllers/BasicController.hpp"
#include "controllers/FlightStateResolver.hpp"

using taiphoon::safety::StatePublisher;

// ==================== 狀態發布頻率測試 ====================

TEST(StatePublisherTest, NormalizePublishRate_ClampsBelowMinimum)
{
    EXPECT_DOUBLE_EQ(StatePublisher::normalize_publish_rate_hz(2.0), 10.0);
}

TEST(StatePublisherTest, NormalizePublishRate_UsesMinimumForInvalidValues)
{
    EXPECT_DOUBLE_EQ(
        StatePublisher::normalize_publish_rate_hz(
            std::numeric_limits<double>::quiet_NaN()),
        10.0);
    EXPECT_DOUBLE_EQ(
        StatePublisher::normalize_publish_rate_hz(
            std::numeric_limits<double>::infinity()),
        10.0);
    EXPECT_DOUBLE_EQ(StatePublisher::normalize_publish_rate_hz(0.0), 10.0);
    EXPECT_DOUBLE_EQ(StatePublisher::normalize_publish_rate_hz(-5.0), 10.0);
}

TEST(StatePublisherTest, NormalizePublishRate_PreservesValidHigherRate)
{
    EXPECT_DOUBLE_EQ(StatePublisher::normalize_publish_rate_hz(20.0), 20.0);
}

// ==================== fill_agent_state 既有測試 ====================

TEST(StatePublisherTest, FillAgentState_PositionNED)
{
    px4_msgs::msg::VehicleOdometry odom;
    odom.position[0] = 10.0F;
    odom.position[1] = -5.0F;
    odom.position[2] = 3.0F;

    taiphoon_agent::msg::AgentState state;
    StatePublisher::fill_agent_state(
        state, odom,
        taiphoon::controllers::FlightState::HOVER, true);

    EXPECT_FLOAT_EQ(state.position_ned[0], 10.0F);
    EXPECT_FLOAT_EQ(state.position_ned[1], -5.0F);
    EXPECT_FLOAT_EQ(state.position_ned[2], 3.0F);
}

TEST(StatePublisherTest, FillAgentState_FlightModeMapping)
{
    px4_msgs::msg::VehicleOdometry odom;

    {
        taiphoon_agent::msg::AgentState state;
        StatePublisher::fill_agent_state(
            state, odom,
            taiphoon::controllers::FlightState::IDLE, true);
        EXPECT_EQ(state.flight_mode,
                  static_cast<uint8_t>(taiphoon::controllers::FlightState::IDLE));
    }

    {
        taiphoon_agent::msg::AgentState state;
        StatePublisher::fill_agent_state(
            state, odom,
            taiphoon::controllers::FlightState::HOVER, true);
        EXPECT_EQ(state.flight_mode,
                  static_cast<uint8_t>(taiphoon::controllers::FlightState::HOVER));
    }

    {
        taiphoon_agent::msg::AgentState state;
        StatePublisher::fill_agent_state(
            state, odom,
            taiphoon::controllers::FlightState::MOVING, true);
        EXPECT_EQ(state.flight_mode,
                  static_cast<uint8_t>(taiphoon::controllers::FlightState::MOVING));
    }

    {
        taiphoon_agent::msg::AgentState state;
        StatePublisher::fill_agent_state(
            state, odom,
            taiphoon::controllers::FlightState::ERROR, true);
        EXPECT_EQ(state.flight_mode,
                  static_cast<uint8_t>(taiphoon::controllers::FlightState::ERROR));
    }
}

TEST(StatePublisherTest, FillAgentState_BatteryDefault)
{
    px4_msgs::msg::VehicleOdometry odom;
    taiphoon_agent::msg::AgentState state;

    StatePublisher::fill_agent_state(
        state, odom,
        taiphoon::controllers::FlightState::HOVER, true);

    // 無 battery 資料時，battery_remaining 預設為 1.0
    EXPECT_FLOAT_EQ(state.battery_remaining, 1.0F);
}

TEST(StatePublisherTest, FillAgentState_OnlineFlag)
{
    px4_msgs::msg::VehicleOdometry odom;
    taiphoon_agent::msg::AgentState state_online;
    taiphoon_agent::msg::AgentState state_offline;

    StatePublisher::fill_agent_state(
        state_online, odom,
        taiphoon::controllers::FlightState::HOVER, true);
    StatePublisher::fill_agent_state(
        state_offline, odom,
        taiphoon::controllers::FlightState::HOVER, false);

    EXPECT_TRUE(state_online.online);
    EXPECT_FALSE(state_offline.online);
}

TEST(StatePublisherTest, FillAgentState_Px4FlightObservation)
{
    px4_msgs::msg::VehicleOdometry odom;
    taiphoon::controllers::FlightStateObservation observation;
    observation.is_px4_state_valid = true;
    observation.is_armed = false;
    observation.is_landed = true;
    observation.px4_nav_state = 14U;

    taiphoon_agent::msg::AgentState state;
    StatePublisher::fill_agent_state(
        state, odom,
        taiphoon::controllers::FlightState::TAKEOFF,
        true, nullptr, nullptr, &observation);

    EXPECT_EQ(state.flight_mode,
              static_cast<uint8_t>(taiphoon::controllers::FlightState::IDLE));
    EXPECT_FALSE(state.armed);
    EXPECT_TRUE(state.landed);
    EXPECT_EQ(state.px4_nav_state, 14U);
    EXPECT_TRUE(state.px4_state_valid);
}

TEST(StatePublisherTest, FillAgentState_StampIsSet)
{
    px4_msgs::msg::VehicleOdometry odom;
    odom.timestamp = 1700000000ULL;

    taiphoon_agent::msg::AgentState state;
    StatePublisher::fill_agent_state(
        state, odom,
        taiphoon::controllers::FlightState::HOVER, true);

    EXPECT_EQ(state.stamp.sec, 1700);
    EXPECT_EQ(state.stamp.nanosec, 0U);
}

// ==================== GPS 擴充測試 ====================

TEST(StatePublisherTest, FillAgentState_GpsFromGlobalPosition)
{
    px4_msgs::msg::VehicleOdometry odom;

    px4_msgs::msg::VehicleGlobalPosition gps;
    gps.lat = 25.03996592258936;
    gps.lon = 121.51195163336611;
    gps.alt = 50.0F;

    taiphoon_agent::msg::AgentState state;
    StatePublisher::fill_agent_state(
        state, odom,
        taiphoon::controllers::FlightState::HOVER,
        true, &gps, nullptr);

    EXPECT_FLOAT_EQ(state.position_global[0], static_cast<float>(gps.lat));
    EXPECT_FLOAT_EQ(state.position_global[1], static_cast<float>(gps.lon));
    EXPECT_FLOAT_EQ(state.position_global[2], gps.alt);
}

TEST(StatePublisherTest, FillAgentState_GpsNullptr)
{
    px4_msgs::msg::VehicleOdometry odom;

    taiphoon_agent::msg::AgentState state;
    StatePublisher::fill_agent_state(
        state, odom,
        taiphoon::controllers::FlightState::HOVER,
        true, nullptr, nullptr);

    EXPECT_TRUE(std::isnan(state.position_global[0]));
    EXPECT_TRUE(std::isnan(state.position_global[1]));
    EXPECT_TRUE(std::isnan(state.position_global[2]));
}

// ==================== Battery 擴充測試 ====================

TEST(StatePublisherTest, FillAgentState_BatteryFromBatteryStatus)
{
    px4_msgs::msg::VehicleOdometry odom;

    px4_msgs::msg::BatteryStatus battery;
    battery.voltage_v = 11.6F;
    battery.current_a = 5.2F;
    battery.remaining = 0.85F;

    taiphoon_agent::msg::AgentState state;
    StatePublisher::fill_agent_state(
        state, odom,
        taiphoon::controllers::FlightState::HOVER,
        true, nullptr, &battery);

    EXPECT_FLOAT_EQ(state.battery_voltage_v, 11.6F);
    EXPECT_FLOAT_EQ(state.battery_current_a, 5.2F);
    EXPECT_FLOAT_EQ(state.battery_remaining, 0.85F);
}

TEST(StatePublisherTest, FillAgentState_BatteryNullptr)
{
    px4_msgs::msg::VehicleOdometry odom;

    taiphoon_agent::msg::AgentState state;
    StatePublisher::fill_agent_state(
        state, odom,
        taiphoon::controllers::FlightState::HOVER,
        true, nullptr, nullptr);

    // Battery nullptr → remaining=1.0, voltage/current=NaN
    EXPECT_FLOAT_EQ(state.battery_remaining, 1.0F);
    EXPECT_TRUE(std::isnan(state.battery_voltage_v));
    EXPECT_TRUE(std::isnan(state.battery_current_a));
}

// ==================== 完整整合測試 ====================

TEST(StatePublisherTest, FillAgentState_AllPresent)
{
    px4_msgs::msg::VehicleOdometry odom;
    odom.position[0] = 10.0F;
    odom.position[1] = -5.0F;
    odom.position[2] = 3.0F;

    px4_msgs::msg::VehicleGlobalPosition gps;
    gps.lat = 25.04;
    gps.lon = 121.51;
    gps.alt = 123.4F;

    px4_msgs::msg::BatteryStatus battery;
    battery.voltage_v = 12.3F;
    battery.current_a = 2.1F;
    battery.remaining = 0.72F;

    taiphoon_agent::msg::AgentState state;
    StatePublisher::fill_agent_state(
        state, odom,
        taiphoon::controllers::FlightState::HOVER,
        true, &gps, &battery);

    // NED 不受影響
    EXPECT_FLOAT_EQ(state.position_ned[0], 10.0F);
    EXPECT_FLOAT_EQ(state.position_ned[1], -5.0F);
    EXPECT_FLOAT_EQ(state.position_ned[2], 3.0F);

    // GPS
    EXPECT_FLOAT_EQ(state.position_global[0], static_cast<float>(gps.lat));
    EXPECT_FLOAT_EQ(state.position_global[1], static_cast<float>(gps.lon));
    EXPECT_FLOAT_EQ(state.position_global[2], gps.alt);

    // Battery
    EXPECT_FLOAT_EQ(state.battery_voltage_v, 12.3F);
    EXPECT_FLOAT_EQ(state.battery_current_a, 2.1F);
    EXPECT_FLOAT_EQ(state.battery_remaining, 0.72F);

    // Flight mode + online
    EXPECT_EQ(state.flight_mode,
              static_cast<uint8_t>(taiphoon::controllers::FlightState::HOVER));
    EXPECT_TRUE(state.online);
}

TEST(StatePublisherTest, FillAgentState_DefaultObservationValues)
{
    // 未提供 PX4 飛行觀測資料時，新增欄位應明確標示為無效。
    px4_msgs::msg::VehicleOdometry odom;
    odom.position[0] = 7.0F;
    odom.position[1] = 8.0F;
    odom.position[2] = 9.0F;

    taiphoon_agent::msg::AgentState state;
    StatePublisher::fill_agent_state(
        state, odom,
        taiphoon::controllers::FlightState::MOVING, true);

    EXPECT_FLOAT_EQ(state.position_ned[0], 7.0F);
    EXPECT_FLOAT_EQ(state.position_ned[1], 8.0F);
    EXPECT_FLOAT_EQ(state.position_ned[2], 9.0F);
    EXPECT_EQ(state.flight_mode,
              static_cast<uint8_t>(taiphoon::controllers::FlightState::MOVING));
    EXPECT_TRUE(state.online);

    // GPS、電池與 PX4 飛行狀態預設值
    EXPECT_TRUE(std::isnan(state.position_global[0]));
    EXPECT_TRUE(std::isnan(state.position_global[1]));
    EXPECT_TRUE(std::isnan(state.position_global[2]));
    EXPECT_FLOAT_EQ(state.battery_remaining, 1.0F);
    EXPECT_TRUE(std::isnan(state.battery_voltage_v));
    EXPECT_TRUE(std::isnan(state.battery_current_a));
    EXPECT_FALSE(state.armed);
    EXPECT_FALSE(state.landed);
    EXPECT_EQ(state.px4_nav_state, 0U);
    EXPECT_FALSE(state.px4_state_valid);
}

// ==================== fill_agent_attitude 既有測試 ====================

TEST(StatePublisherTest, FillAgentAttitude_Quaternion)
{
    px4_msgs::msg::VehicleOdometry odom;
    odom.q[0] = 0.707F;
    odom.q[1] = 0.0F;
    odom.q[2] = 0.707F;
    odom.q[3] = 0.0F;

    taiphoon_agent::msg::AgentAttitude att;
    StatePublisher::fill_agent_attitude(att, odom);

    EXPECT_FLOAT_EQ(att.q[0], 0.707F);
    EXPECT_FLOAT_EQ(att.q[1], 0.0F);
    EXPECT_FLOAT_EQ(att.q[2], 0.707F);
    EXPECT_FLOAT_EQ(att.q[3], 0.0F);
}

TEST(StatePublisherTest, FillAgentAttitude_StampIsSet)
{
    px4_msgs::msg::VehicleOdometry odom;
    odom.timestamp = 2500000500ULL;

    taiphoon_agent::msg::AgentAttitude att;
    StatePublisher::fill_agent_attitude(att, odom);

    EXPECT_EQ(att.stamp.sec, 2500);
    EXPECT_EQ(att.stamp.nanosec, 500000U);
}

TEST(StatePublisherTest, FillAgentAttitude_ZeroQuaternion)
{
    px4_msgs::msg::VehicleOdometry odom;
    odom.q[0] = 0.0F;
    odom.q[1] = 0.0F;
    odom.q[2] = 0.0F;
    odom.q[3] = 0.0F;

    taiphoon_agent::msg::AgentAttitude att;
    StatePublisher::fill_agent_attitude(att, odom);

    EXPECT_FLOAT_EQ(att.q[0], 0.0F);
    EXPECT_FLOAT_EQ(att.q[1], 0.0F);
    EXPECT_FLOAT_EQ(att.q[2], 0.0F);
    EXPECT_FLOAT_EQ(att.q[3], 0.0F);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
