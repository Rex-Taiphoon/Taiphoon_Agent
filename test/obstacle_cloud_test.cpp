/**
 * @file obstacle_cloud_test.cpp
 * @brief 障礙物 PointCloud2 純邏輯資料鏈單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * 測試幾何採樣、三種 source、range/FOV 與有限數值。
 * 測試三種 simulation source 與其共同 PointCloud2 資料契約。
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "perception/ObstacleCloudGenerator.hpp"

namespace
{

using taiphoon::perception::CloudMode;
using taiphoon::perception::GenerationConfig;
using taiphoon::perception::ObstacleCloudGenerator;
using taiphoon::perception::ObstacleDefinition;
using taiphoon::perception::ObstacleType;
using taiphoon::perception::Point3f;
using taiphoon::perception::SensorPose;

/**
 * @brief 建立測試用盒體
 *
 * @param center_ned 盒體中心
 * @return 盒體障礙物
 */
ObstacleDefinition make_box(const Point3f& center_ned)
{
    ObstacleDefinition obstacle;
    obstacle.type = ObstacleType::Box;
    obstacle.center_ned = center_ned;
    obstacle.size_m = {2.0F, 2.0F, 2.0F};
    return obstacle;
}

/**
 * @brief 建立使用固定間距的 ground truth 設定
 *
 * @return 測試用生成設定
 */
GenerationConfig make_ground_truth_config()
{
    GenerationConfig config;
    config.frame_id = "swarm_ned";
    config.mode = CloudMode::GroundTruthMap;
    config.sample_spacing_m = 0.5F;
    return config;
}

/**
 * @brief 確認點位於盒體範圍內
 *
 * @param point 待檢查點
 * @param center_ned 盒體中心
 * @param size_m 盒體尺寸
 * @return true 表示點位於盒體範圍內
 */
bool is_inside_box(
    const Point3f& point,
    const Point3f& center_ned,
    const Point3f& size_m)
{
    constexpr float EPSILON_M = 1.0e-4F;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        if (std::abs(point.at(axis) - center_ned.at(axis)) >
            size_m.at(axis) * 0.5F + EPSILON_M)
        {
            return false;
        }
    }
    return true;
}

}  // namespace

TEST(ObstacleCloudTest, SamplesBoxWithSwarmNedFloat32Fields)
{
    const auto result = ObstacleCloudGenerator::generate(
        {make_box({5.0F, 0.0F, -2.0F})},
        make_ground_truth_config());
    const auto& cloud = result.first;

    ASSERT_FALSE(cloud.points.empty());
    EXPECT_EQ(cloud.frame_id, "swarm_ned");
    ASSERT_EQ(cloud.fields.size(), 3U);
    EXPECT_EQ(cloud.fields.at(0).name, "x");
    EXPECT_EQ(cloud.fields.at(1).name, "y");
    EXPECT_EQ(cloud.fields.at(2).name, "z");
    EXPECT_EQ(cloud.fields.at(0).datatype, 7U);
    EXPECT_EQ(cloud.fields.at(1).datatype, 7U);
    EXPECT_EQ(cloud.fields.at(2).datatype, 7U);
    EXPECT_EQ(cloud.point_step, 12U);
    EXPECT_EQ(cloud.data.size(), cloud.points.size() * 12U);
    EXPECT_TRUE(cloud.has_only_finite_points());

    for (const auto& point : cloud.points)
    {
        EXPECT_TRUE(is_inside_box(point, {5.0F, 0.0F, -2.0F}, {2.0F, 2.0F, 2.0F}));
    }
}

TEST(ObstacleCloudTest, SamplesCylinderSurface)
{
    ObstacleDefinition cylinder;
    cylinder.type = ObstacleType::Cylinder;
    cylinder.center_ned = {8.0F, -2.0F, -3.0F};
    cylinder.radius_m = 2.0F;
    cylinder.height_m = 4.0F;

    const auto result = ObstacleCloudGenerator::generate(
        {cylinder}, make_ground_truth_config());
    const auto& cloud = result.first;

    ASSERT_FALSE(cloud.points.empty());
    for (const auto& point : cloud.points)
    {
        const float dx = point.at(0) - cylinder.center_ned.at(0);
        const float dy = point.at(1) - cylinder.center_ned.at(1);
        const float radial_distance = std::sqrt(dx * dx + dy * dy);
        EXPECT_LE(radial_distance, cylinder.radius_m + 1.0e-4F);
        EXPECT_GE(point.at(2), cylinder.center_ned.at(2) - cylinder.height_m * 0.5F - 1.0e-4F);
        EXPECT_LE(point.at(2), cylinder.center_ned.at(2) + cylinder.height_m * 0.5F + 1.0e-4F);
    }
}

TEST(ObstacleCloudTest, SamplesWallAsThinBox)
{
    ObstacleDefinition wall;
    wall.type = ObstacleType::Wall;
    wall.center_ned = {10.0F, 0.0F, -3.0F};
    wall.size_m = {0.5F, 8.0F, 6.0F};

    const auto result = ObstacleCloudGenerator::generate(
        {wall}, make_ground_truth_config());
    const auto& cloud = result.first;

    ASSERT_FALSE(cloud.points.empty());
    for (const auto& point : cloud.points)
    {
        EXPECT_TRUE(is_inside_box(point, wall.center_ned, wall.size_m));
    }
}

TEST(ObstacleCloudTest, GroundTruthMapIgnoresSensorRangeAndFov)
{
    auto config = make_ground_truth_config();
    config.sensor.range_m = 1.0F;
    config.sensor.horizontal_fov_deg = 10.0F;
    config.sensor.vertical_fov_deg = 10.0F;

    const auto result = ObstacleCloudGenerator::generate(
        {make_box({10.0F, 0.0F, 0.0F})}, config);

    EXPECT_FALSE(result.first.points.empty());
    EXPECT_EQ(result.second.sampled_points, result.second.selected_points);
    EXPECT_EQ(result.second.dropped_points, 0U);
}

TEST(ObstacleCloudTest, ExplicitEmptyCreatesFreshZeroPointSource)
{
    auto config = make_ground_truth_config();
    config.mode = CloudMode::ExplicitEmpty;
    config.sensor.range_m = 0.0F;
    config.sensor.dropout_ratio = 1.0F;

    const auto result = ObstacleCloudGenerator::generate(
        {make_box({5.0F, 0.0F, 0.0F})}, config,
        SensorPose{{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F},
                   0.0F, 0.0F});

    EXPECT_TRUE(result.first.points.empty());
    EXPECT_TRUE(result.first.has_only_finite_points());
    EXPECT_EQ(result.second.sampled_points, 0U);
    EXPECT_EQ(result.second.selected_points, 0U);
}

TEST(ObstacleCloudTest, SensorEmulationFiltersRangeAndHorizontalFov)
{
    auto config = make_ground_truth_config();
    config.mode = CloudMode::SensorEmulation;
    config.sensor.range_m = 10.0F;
    config.sensor.horizontal_fov_deg = 60.0F;
    config.sensor.vertical_fov_deg = 180.0F;
    config.sensor.noise_std_m = 0.0F;
    config.sensor.dropout_ratio = 0.0F;

    const std::vector<ObstacleDefinition> obstacles = {
        make_box({5.0F, 0.0F, 0.0F}),
        make_box({0.0F, 5.0F, 0.0F}),
        make_box({15.0F, 0.0F, 0.0F})};
    const SensorPose pose;
    const auto result = ObstacleCloudGenerator::generate(obstacles, config, pose);

    ASSERT_FALSE(result.first.points.empty());
    for (const auto& point : result.first.points)
    {
        EXPECT_GT(point.at(0), 0.0F);
        EXPECT_LT(std::abs(point.at(1)), 2.0F);
        EXPECT_LT(point.at(0), 10.0F);
    }
    EXPECT_GT(result.second.dropped_points, 0U);
}

TEST(ObstacleCloudTest, SensorEmulationUsesMovingSensorPose)
{
    auto config = make_ground_truth_config();
    config.mode = CloudMode::SyntheticSensor;
    config.sensor.range_m = 3.0F;
    config.sensor.horizontal_fov_deg = 360.0F;
    config.sensor.vertical_fov_deg = 180.0F;
    config.sensor.noise_std_m = 0.0F;
    config.sensor.dropout_ratio = 0.0F;

    const auto obstacles = std::vector<ObstacleDefinition>{
        make_box({5.0F, 0.0F, 0.0F})};
    const auto away = ObstacleCloudGenerator::generate(
        obstacles, config, SensorPose{{0.0F, 0.0F, 0.0F}, 0.0F, 0.0F});
    const auto near = ObstacleCloudGenerator::generate(
        obstacles, config, SensorPose{{4.0F, 0.0F, 0.0F}, 0.0F, 0.0F});

    EXPECT_TRUE(away.first.points.empty());
    EXPECT_FALSE(near.first.points.empty());
}

TEST(ObstacleCloudTest, SensorEmulationAppliesNoiseWithoutNonFiniteValues)
{
    auto config = make_ground_truth_config();
    config.mode = CloudMode::SensorEmulation;
    config.sensor.range_m = 20.0F;
    config.sensor.horizontal_fov_deg = 360.0F;
    config.sensor.vertical_fov_deg = 180.0F;
    config.sensor.noise_std_m = 0.25F;
    config.sensor.dropout_ratio = 0.0F;
    config.random_seed = 1234U;

    const auto result = ObstacleCloudGenerator::generate(
        {make_box({5.0F, 0.0F, 0.0F})}, config);

    EXPECT_FALSE(result.first.points.empty());
    EXPECT_TRUE(result.first.has_only_finite_points());
    EXPECT_EQ(result.second.sampled_points, result.second.selected_points);
}

TEST(ObstacleCloudTest, FullDropoutProducesEmptyFiniteCloud)
{
    auto config = make_ground_truth_config();
    config.mode = CloudMode::SensorEmulation;
    config.sensor.range_m = 20.0F;
    config.sensor.horizontal_fov_deg = 360.0F;
    config.sensor.vertical_fov_deg = 180.0F;
    config.sensor.dropout_ratio = 1.0F;

    const auto result = ObstacleCloudGenerator::generate(
        {make_box({5.0F, 0.0F, 0.0F})}, config);

    EXPECT_TRUE(result.first.points.empty());
    EXPECT_TRUE(result.first.has_only_finite_points());
    EXPECT_EQ(result.second.selected_points, 0U);
    EXPECT_EQ(result.second.sampled_points, result.second.dropped_points);
}

TEST(ObstacleCloudTest, RejectsInvalidSensorConfiguration)
{
    auto config = make_ground_truth_config();
    config.sensor.dropout_ratio = 1.1F;
    std::string error;

    EXPECT_FALSE(ObstacleCloudGenerator::validate_config(config, error));
    EXPECT_FALSE(error.empty());
    EXPECT_THROW(
        ObstacleCloudGenerator::generate({}, config),
        std::invalid_argument);
}
