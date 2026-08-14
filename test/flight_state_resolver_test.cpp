/**
 * @file flight_state_resolver_test.cpp
 * @brief FlightStateResolver 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-06
 */

#include <gtest/gtest.h>

#include "controllers/FlightStateResolver.hpp"

using taiphoon::controllers::FlightState;
using taiphoon::controllers::FlightStateObservation;
using taiphoon::controllers::FlightStateResolver;

namespace
{

FlightStateObservation make_valid_observation()
{
    FlightStateObservation observation;
    observation.is_px4_state_valid = true;
    observation.is_armed = true;
    observation.is_landed = false;
    return observation;
}

}  // namespace

TEST(FlightStateResolverTest, InvalidPx4StateReportsError)
{
    FlightStateObservation observation;
    observation.is_px4_state_valid = false;

    auto resolution = FlightStateResolver::resolve(
        observation, FlightState::TAKEOFF);

    EXPECT_EQ(resolution.effective_state, FlightState::ERROR);
    EXPECT_FALSE(resolution.should_transition_to_hover);
}

TEST(FlightStateResolverTest, GroundFactsOverridePendingTakeoff)
{
    auto observation = make_valid_observation();
    observation.is_armed = false;
    observation.is_landed = true;

    auto disarmed_resolution = FlightStateResolver::resolve(
        observation, FlightState::TAKEOFF);

    EXPECT_EQ(disarmed_resolution.effective_state, FlightState::IDLE);

    observation.is_armed = true;
    auto armed_resolution = FlightStateResolver::resolve(
        observation, FlightState::MOVING);

    EXPECT_EQ(armed_resolution.effective_state, FlightState::ARMED);
}

TEST(FlightStateResolverTest, Px4AutomaticModesOverrideAgentIntent)
{
    auto observation = make_valid_observation();
    observation.px4_nav_state = FlightStateResolver::PX4_NAV_STATE_AUTO_RTL;

    auto rtl_resolution = FlightStateResolver::resolve(
        observation, FlightState::MOVING);

    EXPECT_EQ(rtl_resolution.effective_state, FlightState::RTL);

    observation.px4_nav_state = FlightStateResolver::PX4_NAV_STATE_AUTO_LAND;
    auto land_resolution = FlightStateResolver::resolve(
        observation, FlightState::HOVER);

    EXPECT_EQ(land_resolution.effective_state, FlightState::LAND);
}

TEST(FlightStateResolverTest, OffboardNavigationStateIsRecognized)
{
    auto observation = make_valid_observation();
    observation.px4_nav_state = FlightStateResolver::PX4_NAV_STATE_OFFBOARD;

    auto resolution = FlightStateResolver::resolve(
        observation, FlightState::IDLE);

    EXPECT_EQ(resolution.effective_state, FlightState::HOVER);
    EXPECT_FALSE(resolution.should_transition_to_hover);
}

TEST(FlightStateResolverTest, TakeoffTransitionsToHoverAtTargetAltitude)
{
    auto observation = make_valid_observation();
    observation.current_altitude_m = 2.85F;
    observation.target_altitude_m = 3.0F;

    auto resolution = FlightStateResolver::resolve(
        observation, FlightState::TAKEOFF);

    EXPECT_EQ(resolution.effective_state, FlightState::HOVER);
    EXPECT_TRUE(resolution.should_transition_to_hover);
    EXPECT_FALSE(resolution.should_hold_current_position);
}

TEST(FlightStateResolverTest, TakeoffTimeoutTransitionsToCurrentPositionHold)
{
    auto observation = make_valid_observation();
    observation.current_altitude_m = 1.0F;
    observation.target_altitude_m = 3.0F;
    observation.takeoff_elapsed_s = 30.0;

    auto resolution = FlightStateResolver::resolve(
        observation, FlightState::TAKEOFF);

    EXPECT_EQ(resolution.effective_state, FlightState::HOVER);
    EXPECT_TRUE(resolution.should_transition_to_hover);
    EXPECT_TRUE(resolution.should_hold_current_position);
}

TEST(FlightStateResolverTest, MovingTransitionsToHoverAtTargetPosition)
{
    auto observation = make_valid_observation();
    observation.target_distance_m = 0.3F;

    auto arrived_resolution = FlightStateResolver::resolve(
        observation, FlightState::MOVING);

    EXPECT_EQ(arrived_resolution.effective_state, FlightState::HOVER);
    EXPECT_TRUE(arrived_resolution.should_transition_to_hover);

    observation.target_distance_m = 0.31F;
    auto moving_resolution = FlightStateResolver::resolve(
        observation, FlightState::MOVING);

    EXPECT_EQ(moving_resolution.effective_state, FlightState::MOVING);
    EXPECT_FALSE(moving_resolution.should_transition_to_hover);
}

TEST(FlightStateResolverTest, AirborneWithoutActiveMissionReportsHover)
{
    auto observation = make_valid_observation();

    auto resolution = FlightStateResolver::resolve(
        observation, FlightState::IDLE);

    EXPECT_EQ(resolution.effective_state, FlightState::HOVER);
}
