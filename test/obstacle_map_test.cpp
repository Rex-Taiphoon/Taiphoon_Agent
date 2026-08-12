/**
 * @file obstacle_map_test.cpp
 * @brief LocalObstacleMap freshness 與幾何查詢單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-12
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "perception/RawPointObstacleMap.hpp"

namespace
{

using taiphoon::ego_swarm::Vector3;
using taiphoon::perception::ObstacleMapStatus;
using taiphoon::perception::RawPointObstacleMap;

}  // namespace

TEST(RawPointObstacleMapTest, DistinguishesMissingFromFreshExplicitEmpty)
{
    RawPointObstacleMap map(1.0);

    EXPECT_FALSE(map.is_valid());
    EXPECT_EQ(map.status(0.0), ObstacleMapStatus::Missing);
    EXPECT_FALSE(map.is_fresh(0.0));

    ASSERT_TRUE(map.set_explicit_empty(10.0));
    EXPECT_TRUE(map.is_valid());
    EXPECT_TRUE(map.is_fresh(10.0));
    EXPECT_EQ(map.status(10.0), ObstacleMapStatus::Fresh);
    EXPECT_EQ(map.point_count(), 0U);
    EXPECT_TRUE(std::isinf(map.nearest_distance(Vector3{0.0, 0.0, 0.0})));
    EXPECT_TRUE(map.is_position_safe(Vector3{0.0, 0.0, 0.0}, 1.0));
    EXPECT_DOUBLE_EQ(map.obstacle_cost(Vector3{0.0, 0.0, 0.0}), 0.0);
}

TEST(RawPointObstacleMapTest, MarksValidMapStaleAfterTimeout)
{
    RawPointObstacleMap map(0.5);
    ASSERT_TRUE(map.update({Vector3{5.0, 0.0, 0.0}}, 2.0));

    EXPECT_EQ(map.status(2.5), ObstacleMapStatus::Fresh);
    EXPECT_EQ(map.status(2.6), ObstacleMapStatus::Stale);
    EXPECT_TRUE(map.is_valid());
    EXPECT_FALSE(map.is_fresh(2.6));
}

TEST(RawPointObstacleMapTest, QueriesNearestDistanceAndClearance)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.update({
        Vector3{5.0, 0.0, 0.0},
        Vector3{8.0, 0.0, 0.0}}, 0.0));

    EXPECT_DOUBLE_EQ(
        map.nearest_distance(Vector3{0.0, 0.0, 0.0}), 5.0);
    EXPECT_TRUE(map.is_position_safe(Vector3{0.0, 0.0, 0.0}, 4.0));
    EXPECT_FALSE(map.is_position_safe(Vector3{0.0, 0.0, 0.0}, 5.1));
    EXPECT_TRUE(std::isfinite(
        map.obstacle_cost(Vector3{0.0, 0.0, 0.0})));
}

TEST(RawPointObstacleMapTest, RejectsInvalidUpdateWithoutCreatingEmptyMap)
{
    RawPointObstacleMap map;
    const double nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(map.update({Vector3{nan, 0.0, 0.0}}, 0.0));
    EXPECT_FALSE(map.update({}, -1.0));
    EXPECT_EQ(map.status(0.0), ObstacleMapStatus::Missing);
    EXPECT_FALSE(map.is_valid());

    ASSERT_TRUE(map.update({Vector3{1.0, 0.0, 0.0}}, 0.0));
    EXPECT_EQ(map.status(-1.0), ObstacleMapStatus::Invalid);
    EXPECT_EQ(map.status(2.0), ObstacleMapStatus::Stale);
}

TEST(RawPointObstacleMapTest, FutureTimestampIsInvalid)
{
    RawPointObstacleMap map;
    ASSERT_TRUE(map.update({}, 10.0));

    EXPECT_EQ(map.status(9.0), ObstacleMapStatus::Invalid);
    EXPECT_FALSE(map.is_fresh(9.0));
}
