/**
 * @file test_state_publisher.cpp
 * @brief StatePublisher 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 測試 StatePublisher 的 PX4 odometry → AgentState/AgentAttitude 轉譯邏輯。
 * 需在 colcon 環境中編譯執行：colcon test --packages-select taiphoon_agent
 */

#include <gtest/gtest.h>

#include "safety/StatePublisher.hpp"
#include "taiphoon_agent/msg/agent_state.hpp"
#include "taiphoon_agent/msg/agent_attitude.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "controllers/BasicController.hpp"

using taiphoon::safety::StatePublisher;

// ==================== fill_agent_state 測試 ====================

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

    // IDLE → 0
    {
        taiphoon_agent::msg::AgentState state;
        StatePublisher::fill_agent_state(
            state, odom,
            taiphoon::controllers::FlightState::IDLE, true);
        EXPECT_EQ(state.flight_mode,
                  static_cast<uint8_t>(taiphoon::controllers::FlightState::IDLE));
    }

    // HOVER → static_cast<uint8_t>(HOVER)
    {
        taiphoon_agent::msg::AgentState state;
        StatePublisher::fill_agent_state(
            state, odom,
            taiphoon::controllers::FlightState::HOVER, true);
        EXPECT_EQ(state.flight_mode,
                  static_cast<uint8_t>(taiphoon::controllers::FlightState::HOVER));
    }

    // MOVING → static_cast<uint8_t>(MOVING)
    {
        taiphoon_agent::msg::AgentState state;
        StatePublisher::fill_agent_state(
            state, odom,
            taiphoon::controllers::FlightState::MOVING, true);
        EXPECT_EQ(state.flight_mode,
                  static_cast<uint8_t>(taiphoon::controllers::FlightState::MOVING));
    }

    // ERROR → static_cast<uint8_t>(ERROR)
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

    // 預設電量應為 1.0（PX4 odometry 不含電量資訊）
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

TEST(StatePublisherTest, FillAgentState_StampIsSet)
{
    // 模擬 PX4 時間戳 (微秒): 1,700,000,000 us = 1700 秒
    px4_msgs::msg::VehicleOdometry odom;
    odom.timestamp = 1700000000ULL;

    taiphoon_agent::msg::AgentState state;
    StatePublisher::fill_agent_state(
        state, odom,
        taiphoon::controllers::FlightState::HOVER, true);

    // stamp.sec = 1700000000 / 1000000 = 1700
    EXPECT_EQ(state.stamp.sec, 1700);
    EXPECT_EQ(state.stamp.nanosec, 0U);
}

// ==================== fill_agent_attitude 測試 ====================

TEST(StatePublisherTest, FillAgentAttitude_Quaternion)
{
    px4_msgs::msg::VehicleOdometry odom;
    odom.q[0] = 0.707F;
    odom.q[1] = 0.0F;
    odom.q[2] = 0.707F;
    odom.q[3] = 0.0F;

    taiphoon_agent::msg::AgentAttitude att;
    StatePublisher::fill_agent_attitude(att, odom);

    EXPECT_FLOAT_EQ(att.q[0], 0.707F); // w
    EXPECT_FLOAT_EQ(att.q[1], 0.0F);   // x
    EXPECT_FLOAT_EQ(att.q[2], 0.707F); // y
    EXPECT_FLOAT_EQ(att.q[3], 0.0F);   // z
}

TEST(StatePublisherTest, FillAgentAttitude_StampIsSet)
{
    // 模擬 PX4 時間戳 (微秒): 2,500,000,500 us = 2500 秒 + 500000 奈秒
    px4_msgs::msg::VehicleOdometry odom;
    odom.timestamp = 2500000500ULL;

    taiphoon_agent::msg::AgentAttitude att;
    StatePublisher::fill_agent_attitude(att, odom);

    // stamp.sec = 2500000500 / 1000000 = 2500
    // stamp.nanosec = (2500000500 % 1000000) * 1000 = 500 * 1000 = 500000
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
