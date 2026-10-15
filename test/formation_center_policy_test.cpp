/**
 * @file formation_center_policy_test.cpp
 * @brief 初次編隊中心高度回歸測試（先移動再編隊不得疊加高度）
 * @author Taiphoon Agent Team
 * @date 2026-10-15
 */
#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "ego_swarm/FormationCenterPolicy.hpp"
#include "ego_swarm/FormationSlotPlanner.hpp"

using taiphoon::ego_swarm::FORMATION_CENTER_HOME_DOWN_M;
using taiphoon::ego_swarm::FormationConfig;
using taiphoon::ego_swarm::FormationSlotPlanner;
using taiphoon::ego_swarm::FormationType;
using taiphoon::ego_swarm::Vector3;
using taiphoon::ego_swarm::make_formation_center_from_targets;

TEST(FormationCenterPolicy, UsesHorizontalMeanAndHomeAltitude)
{
    const std::vector<Vector3> targets{
        Vector3{10.0, 0.0, -2.0}, Vector3{0.0, 4.0, -2.0}, Vector3{2.0, 2.0, -5.0}};
    Vector3 center{};
    ASSERT_TRUE(make_formation_center_from_targets(targets, center));
    EXPECT_NEAR(center.at(0), 4.0, 1.0e-9);
    EXPECT_NEAR(center.at(1), 2.0, 1.0e-9);
    EXPECT_NEAR(center.at(2), FORMATION_CENTER_HOME_DOWN_M, 1.0e-9);
}

TEST(FormationCenterPolicy, MovedAgentsDoNotDoubleFormationHeight)
{
    // 起飛 2 m 後移動，再下 FORMATION_CIRCLE 2 2：槽位應位於 2 m 而非 4 m。
    const double takeoff_down_m = -2.0;
    const double formation_height_m = 2.0;
    const std::vector<Vector3> targets{
        Vector3{5.0, 0.0, takeoff_down_m}, Vector3{0.0, 3.0, takeoff_down_m},
        Vector3{-3.0, 0.0, takeoff_down_m}};
    Vector3 center{};
    ASSERT_TRUE(make_formation_center_from_targets(targets, center));

    FormationConfig config;
    config.type = FormationType::CIRCLE;
    config.radius_m = 2.0;
    config.vertical_offset_m = -formation_height_m;
    std::vector<Vector3> offsets;
    ASSERT_TRUE(FormationSlotPlanner::compute_slot_offsets(
        targets.size(), config, offsets));
    for (const Vector3& offset : offsets)
    {
        EXPECT_NEAR(center.at(2) + offset.at(2), -formation_height_m, 1.0e-9);
    }
}

TEST(FormationCenterPolicy, SingleTargetBoundary)
{
    Vector3 center{};
    ASSERT_TRUE(make_formation_center_from_targets(
        std::vector<Vector3>{Vector3{-7.5, 1.25, -30.0}}, center));
    EXPECT_NEAR(center.at(0), -7.5, 1.0e-9);
    EXPECT_NEAR(center.at(1), 1.25, 1.0e-9);
    EXPECT_NEAR(center.at(2), 0.0, 1.0e-9);
}

TEST(FormationCenterPolicy, RejectsEmptyAndNonFiniteTargets)
{
    Vector3 center{1.0, 1.0, 1.0};
    EXPECT_FALSE(make_formation_center_from_targets({}, center));
    EXPECT_NEAR(center.at(0), 0.0, 1.0e-9);

    const double nan_value = std::numeric_limits<double>::quiet_NaN();
    const double inf_value = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(make_formation_center_from_targets(
        {Vector3{0.0, 0.0, -2.0}, Vector3{nan_value, 0.0, -2.0}}, center));
    EXPECT_FALSE(make_formation_center_from_targets(
        {Vector3{0.0, inf_value, -2.0}}, center));
    EXPECT_FALSE(make_formation_center_from_targets(
        {Vector3{0.0, 0.0, nan_value}}, center));
}
