/**
 * @file flight_mission_lifecycle_test.cpp
 * @brief 降落、無效觀測與編隊重新啟動回歸測試
 * @author Taiphoon Agent Team
 * @date 2026-10-06
 */
#include <gtest/gtest.h>
#include "ego_swarm/FlightMissionLifecycle.hpp"

using taiphoon::ego_swarm::FlightMissionLifecycle;

TEST(FlightMissionLifecycle, GroundCommandsCannotBeQueuedIntoTakeoff)
{
    FlightMissionLifecycle policy;
    EXPECT_FALSE(policy.can_plan());
    EXPECT_TRUE(policy.observe(true, false, true));
    policy.accept_formation_command();
    EXPECT_FALSE(policy.can_adopt_formation());
    EXPECT_FALSE(policy.observe(true, true, true));
    EXPECT_FALSE(policy.can_plan());
    EXPECT_FALSE(policy.observe(true, true, false));
    EXPECT_TRUE(policy.can_plan());
    EXPECT_FALSE(policy.can_adopt_formation());
}

TEST(FlightMissionLifecycle, LandingResetsOnceAndOldEpochCannotResume)
{
    FlightMissionLifecycle policy;
    policy.observe(true, true, false);
    policy.accept_formation_command();
    ASSERT_TRUE(policy.can_adopt_formation());
    EXPECT_TRUE(policy.observe(true, true, true));
    EXPECT_FALSE(policy.can_plan());
    EXPECT_FALSE(policy.observe(true, false, true));
    policy.observe(true, true, false);
    EXPECT_FALSE(policy.can_adopt_formation());
    policy.accept_formation_command();
    EXPECT_TRUE(policy.can_adopt_formation());
}

TEST(FlightMissionLifecycle, InvalidObservationSuspendsWithoutInventingLanding)
{
    FlightMissionLifecycle policy;
    policy.observe(true, true, false);
    policy.accept_formation_command();
    EXPECT_FALSE(policy.observe(false, false, true));
    EXPECT_FALSE(policy.can_plan());
    EXPECT_FALSE(policy.can_adopt_formation());
    policy.observe(true, true, false);
    EXPECT_TRUE(policy.can_adopt_formation());
    EXPECT_TRUE(policy.observe(true, false, false));
    EXPECT_FALSE(policy.can_plan());
    policy.observe(true, true, false);
    EXPECT_FALSE(policy.can_adopt_formation());
}
