/**
 * @file ObstacleCloudGenerator.cpp
 * @brief 障礙物幾何採樣與 PointCloud2 相容資料生成實作
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * 所有幾何均在 swarm_ned 中採樣，並在輸出前再次驗證有限數值，避免非法點
 * 被未來的飛行規劃器或 ROS2 PointCloud2 Adapter 使用。
 */

#include "perception/ObstacleCloudGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

namespace taiphoon
{
namespace perception
{
namespace
{

constexpr float FLOAT_EPSILON = 1.0e-5F;
constexpr float DEG_TO_RAD = OBSTACLE_CLOUD_PI / 180.0F;
constexpr std::size_t MINIMUM_AXIS_SAMPLES = 2U;

/**
 * @brief 檢查三維點是否全部為有限值
 *
 * @param point 待檢查的三維點
 * @return true 表示點不含 NaN 或 Inf
 */
bool is_finite(const Point3f& point)
{
    return std::isfinite(point.at(0)) &&
           std::isfinite(point.at(1)) &&
           std::isfinite(point.at(2));
}

/**
 * @brief 檢查標量是否為有限正值
 *
 * @param value 待檢查數值
 * @return true 表示數值為有限且大於零
 */
bool is_finite_positive(const float value)
{
    return std::isfinite(value) && value > 0.0F;
}

/**
 * @brief 計算軸向採樣座標
 *
 * @param minimum 軸向下限
 * @param maximum 軸向上限
 * @param spacing_m 採樣間距
 * @return 包含兩端點的規則採樣座標
 */
std::vector<float> sample_axis(
    const float minimum,
    const float maximum,
    const float spacing_m)
{
    std::vector<float> samples;
    const float length = maximum - minimum;
    const std::size_t sample_count = std::max(
        MINIMUM_AXIS_SAMPLES,
        static_cast<std::size_t>(std::ceil(length / spacing_m)) + 1U);
    samples.reserve(sample_count);

    for (std::size_t index = 0U; index < sample_count; ++index)
    {
        const float ratio = sample_count > 1U
            ? static_cast<float>(index) / static_cast<float>(sample_count - 1U)
            : 0.0F;
        samples.push_back(minimum + ratio * length);
    }
    return samples;
}

/**
 * @brief 將幾何點加入採樣結果
 *
 * @param point 待加入的幾何點
 * @param samples 採樣結果
 */
void append_point(const Point3f& point, std::vector<Point3f>& samples)
{
    if (is_finite(point))
    {
        samples.push_back(point);
    }
}

/**
 * @brief 採樣盒體或薄牆的六個表面
 *
 * @param obstacle 盒體或牆面障礙物
 * @param spacing_m 表面採樣間距
 * @param samples 輸出採樣點
 */
void sample_box_surface(
    const ObstacleDefinition& obstacle,
    const float spacing_m,
    std::vector<Point3f>& samples)
{
    const float half_x = obstacle.size_m.at(0) * 0.5F;
    const float half_y = obstacle.size_m.at(1) * 0.5F;
    const float half_z = obstacle.size_m.at(2) * 0.5F;
    const auto x_values = sample_axis(
        obstacle.center_ned.at(0) - half_x,
        obstacle.center_ned.at(0) + half_x,
        spacing_m);
    const auto y_values = sample_axis(
        obstacle.center_ned.at(1) - half_y,
        obstacle.center_ned.at(1) + half_y,
        spacing_m);
    const auto z_values = sample_axis(
        obstacle.center_ned.at(2) - half_z,
        obstacle.center_ned.at(2) + half_z,
        spacing_m);

    for (const float x : x_values)
    {
        for (const float y : y_values)
        {
            append_point({x, y, obstacle.center_ned.at(2) - half_z}, samples);
            append_point({x, y, obstacle.center_ned.at(2) + half_z}, samples);
        }
    }

    for (const float x : x_values)
    {
        for (const float z : z_values)
        {
            append_point({x, obstacle.center_ned.at(1) - half_y, z}, samples);
            append_point({x, obstacle.center_ned.at(1) + half_y, z}, samples);
        }
    }

    for (const float y : y_values)
    {
        for (const float z : z_values)
        {
            append_point({obstacle.center_ned.at(0) - half_x, y, z}, samples);
            append_point({obstacle.center_ned.at(0) + half_x, y, z}, samples);
        }
    }
}

/**
 * @brief 採樣垂直圓柱的側面與上下蓋面
 *
 * @param obstacle 圓柱障礙物
 * @param spacing_m 表面採樣間距
 * @param samples 輸出採樣點
 */
void sample_cylinder_surface(
    const ObstacleDefinition& obstacle,
    const float spacing_m,
    std::vector<Point3f>& samples)
{
    const float half_height = obstacle.height_m * 0.5F;
    const auto z_values = sample_axis(
        obstacle.center_ned.at(2) - half_height,
        obstacle.center_ned.at(2) + half_height,
        spacing_m);
    const float circumference = 2.0F * OBSTACLE_CLOUD_PI * obstacle.radius_m;
    const std::size_t angle_count = std::max(
        MINIMUM_AXIS_SAMPLES,
        static_cast<std::size_t>(std::ceil(circumference / spacing_m)));
    const auto radial_values = sample_axis(0.0F, obstacle.radius_m, spacing_m);

    for (const float z : z_values)
    {
        for (std::size_t angle_index = 0U; angle_index < angle_count; ++angle_index)
        {
            const float ratio = static_cast<float>(angle_index) /
                static_cast<float>(angle_count);
            const float angle = ratio * 2.0F * OBSTACLE_CLOUD_PI;
            append_point({
                obstacle.center_ned.at(0) + obstacle.radius_m * std::cos(angle),
                obstacle.center_ned.at(1) + obstacle.radius_m * std::sin(angle),
                z}, samples);
        }
    }

    for (const float radius : radial_values)
    {
        const std::size_t ring_count = std::max(
            MINIMUM_AXIS_SAMPLES,
            static_cast<std::size_t>(std::ceil(
                2.0F * OBSTACLE_CLOUD_PI * radius / spacing_m)));
        for (std::size_t angle_index = 0U; angle_index < ring_count; ++angle_index)
        {
            const float ratio = static_cast<float>(angle_index) /
                static_cast<float>(ring_count);
            const float angle = ratio * 2.0F * OBSTACLE_CLOUD_PI;
            const float x = obstacle.center_ned.at(0) + radius * std::cos(angle);
            const float y = obstacle.center_ned.at(1) + radius * std::sin(angle);
            append_point({x, y, obstacle.center_ned.at(2) - half_height}, samples);
            append_point({x, y, obstacle.center_ned.at(2) + half_height}, samples);
        }
    }
}

/**
 * @brief 依障礙物類型採樣表面
 *
 * @param obstacle 障礙物定義
 * @param spacing_m 採樣間距
 * @param samples 輸出採樣點
 */
void sample_obstacle(
    const ObstacleDefinition& obstacle,
    const float spacing_m,
    std::vector<Point3f>& samples)
{
    switch (obstacle.type)
    {
        case ObstacleType::Box:
        case ObstacleType::Wall:
            sample_box_surface(obstacle, spacing_m, samples);
            break;
        case ObstacleType::Cylinder:
            sample_cylinder_surface(obstacle, spacing_m, samples);
            break;
    }
}

/**
 * @brief 將角度正規化至 [-pi, pi]
 *
 * @param angle_rad 待正規化角度
 * @return 正規化後角度
 */
float normalize_angle(const float angle_rad)
{
    float normalized = angle_rad;
    while (normalized > OBSTACLE_CLOUD_PI)
    {
        normalized -= 2.0F * OBSTACLE_CLOUD_PI;
    }
    while (normalized < -OBSTACLE_CLOUD_PI)
    {
        normalized += 2.0F * OBSTACLE_CLOUD_PI;
    }
    return normalized;
}

/**
 * @brief 判斷點是否落在感測器的 range 與視野內
 *
 * @param point 障礙物點
 * @param pose 感測器姿態
 * @param model 感測模型
 * @return true 表示點可被感測器選取
 */
bool is_visible(
    const Point3f& point,
    const SensorPose& pose,
    const SensorModel& model)
{
    const float dx = point.at(0) - pose.position_ned.at(0);
    const float dy = point.at(1) - pose.position_ned.at(1);
    const float dz = point.at(2) - pose.position_ned.at(2);
    const float horizontal_distance = std::sqrt(dx * dx + dy * dy);
    const float distance = std::sqrt(
        horizontal_distance * horizontal_distance + dz * dz);
    if (distance > model.range_m + FLOAT_EPSILON)
    {
        return false;
    }

    const float horizontal_fov_rad = model.horizontal_fov_deg * DEG_TO_RAD;
    if (horizontal_fov_rad < 2.0F * OBSTACLE_CLOUD_PI - FLOAT_EPSILON)
    {
        const float bearing = std::atan2(dy, dx);
        const float bearing_error = normalize_angle(bearing - pose.heading_rad);
        if (std::abs(bearing_error) > horizontal_fov_rad * 0.5F + FLOAT_EPSILON)
        {
            return false;
        }
    }

    const float vertical_fov_rad = model.vertical_fov_deg * DEG_TO_RAD;
    if (vertical_fov_rad < OBSTACLE_CLOUD_PI - FLOAT_EPSILON)
    {
        const float elevation_rad = std::atan2(-dz, horizontal_distance);
        if (std::abs(elevation_rad - pose.pitch_rad) >
            vertical_fov_rad * 0.5F + FLOAT_EPSILON)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 檢查障礙物幾何數值
 *
 * @param obstacle 待檢查障礙物
 * @param error 輸出失敗原因
 * @param obstacle_index 障礙物索引
 * @return true 表示幾何可採樣
 */
bool validate_obstacle(
    const ObstacleDefinition& obstacle,
    std::string& error,
    const std::size_t obstacle_index)
{
    if (!is_finite(obstacle.center_ned))
    {
        error = "障礙物 " + std::to_string(obstacle_index) + " 的中心含有非有限值";
        return false;
    }

    if (obstacle.type == ObstacleType::Cylinder)
    {
        if (!is_finite_positive(obstacle.radius_m) ||
            !is_finite_positive(obstacle.height_m))
        {
            error = "障礙物 " + std::to_string(obstacle_index) +
                " 的 cylinder radius_m/height_m 必須為有限正值";
            return false;
        }
        return true;
    }

    if (!is_finite(obstacle.size_m) ||
        !is_finite_positive(obstacle.size_m.at(0)) ||
        !is_finite_positive(obstacle.size_m.at(1)) ||
        !is_finite_positive(obstacle.size_m.at(2)))
    {
        error = "障礙物 " + std::to_string(obstacle_index) +
            " 的 size_m 必須是三個有限正值";
        return false;
    }
    return true;
}

/**
 * @brief 將點加入輸出並套用 synthetic_sensor 的雜訊
 *
 * @param point 原始幾何點
 * @param config 生成設定
 * @param random_engine 隨機引擎
 * @param output 輸出點列表
 * @param report 生成統計
 */
void select_point(
    const Point3f& point,
    const GenerationConfig& config,
    std::mt19937& random_engine,
    std::vector<Point3f>& output,
    GenerationReport& report)
{
    ++report.sampled_points;
    if (config.mode == CloudMode::SyntheticSensor)
    {
        std::uniform_real_distribution<float> dropout_distribution(0.0F, 1.0F);
        if (dropout_distribution(random_engine) < config.sensor.dropout_ratio)
        {
            ++report.dropped_points;
            return;
        }
    }

    Point3f selected = point;
    if (config.mode == CloudMode::SyntheticSensor &&
        config.sensor.noise_std_m > 0.0F)
    {
        std::normal_distribution<float> noise_distribution(
            0.0F, config.sensor.noise_std_m);
        selected.at(0) += noise_distribution(random_engine);
        selected.at(1) += noise_distribution(random_engine);
        selected.at(2) += noise_distribution(random_engine);
    }

    if (is_finite(selected))
    {
        output.push_back(selected);
        ++report.selected_points;
    }
    else
    {
        ++report.dropped_points;
    }
}

/**
 * @brief 建立 PointCloud2 的 x/y/z 欄位描述
 *
 * @return 欄位描述列表
 */
std::vector<PointCloud2Field> make_fields()
{
    return {
        {"x", 0U, POINT_CLOUD_FLOAT32_DATATYPE, 1U},
        {"y", 4U, POINT_CLOUD_FLOAT32_DATATYPE, 1U},
        {"z", 8U, POINT_CLOUD_FLOAT32_DATATYPE, 1U}};
}

/**
 * @brief 將 float32 寫入連續資料緩衝區
 *
 * @param value 待寫入數值
 * @param data 目標資料緩衝區
 * @param offset 緩衝區偏移量
 */
void write_float32(
    const float value,
    std::vector<std::uint8_t>& data,
    const std::size_t offset)
{
    if (offset + sizeof(float) > data.size())
    {
        return;
    }
    std::uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(float));
    data.at(offset) = static_cast<std::uint8_t>(bits & 0xFFU);
    data.at(offset + 1U) = static_cast<std::uint8_t>((bits >> 8U) & 0xFFU);
    data.at(offset + 2U) = static_cast<std::uint8_t>((bits >> 16U) & 0xFFU);
    data.at(offset + 3U) = static_cast<std::uint8_t>((bits >> 24U) & 0xFFU);
}

/**
 * @brief 從 little-endian 緩衝區讀取 float32
 *
 * @param data 來源資料緩衝區
 * @param offset 緩衝區偏移量
 * @return 解碼後的 float32；越界時回傳 NaN
 */
float read_float32(
    const std::vector<std::uint8_t>& data,
    const std::size_t offset)
{
    if (offset + sizeof(float) > data.size())
    {
        return std::numeric_limits<float>::quiet_NaN();
    }

    const std::uint32_t bits =
        static_cast<std::uint32_t>(data.at(offset)) |
        (static_cast<std::uint32_t>(data.at(offset + 1U)) << 8U) |
        (static_cast<std::uint32_t>(data.at(offset + 2U)) << 16U) |
        (static_cast<std::uint32_t>(data.at(offset + 3U)) << 24U);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(float));
    return value;
}

/**
 * @brief 將點列表封裝成 PointCloud2 相容資料
 *
 * @param frame_id 座標框架名稱
 * @param points 有限三維點列表
 * @return PointCloud2 相容資料
 */
PointCloud2Data make_cloud(
    const std::string& frame_id,
    const std::vector<Point3f>& points)
{
    PointCloud2Data cloud;
    cloud.frame_id = frame_id;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.fields = make_fields();
    cloud.points = points;
    cloud.data.resize(static_cast<std::size_t>(cloud.row_step), 0U);

    for (std::size_t index = 0U; index < points.size(); ++index)
    {
        const std::size_t offset = index * cloud.point_step;
        write_float32(points.at(index).at(0), cloud.data, offset);
        write_float32(points.at(index).at(1), cloud.data, offset + sizeof(float));
        write_float32(points.at(index).at(2), cloud.data, offset + 2U * sizeof(float));
    }
    return cloud;
}

}  // namespace

bool PointCloud2Data::has_only_finite_points() const
{
    if (is_bigendian || !is_dense || point_step != POINT_CLOUD_POINT_STEP_BYTES ||
        width != points.size() || row_step != width * point_step ||
        data.size() != row_step)
    {
        return false;
    }

    for (const auto& point : points)
    {
        if (!is_finite(point))
        {
            return false;
        }
    }

    for (std::size_t offset = 0U; offset + sizeof(float) <= data.size();
         offset += sizeof(float))
    {
        const float value = read_float32(data, offset);
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    return true;
}

bool ObstacleCloudGenerator::validate_config(
    const GenerationConfig& config,
    std::string& error)
{
    error.clear();
    if (config.frame_id.empty())
    {
        error = "frame_id 不可為空";
        return false;
    }
    if (!is_finite_positive(config.sample_spacing_m))
    {
        error = "sample_spacing_m 必須為有限正值";
        return false;
    }

    if (config.mode == CloudMode::ExplicitEmpty)
    {
        return true;
    }

    const auto& sensor = config.sensor;
    if (!is_finite_positive(sensor.range_m))
    {
        error = "sensor.range_m 必須為有限正值";
        return false;
    }
    if (!std::isfinite(sensor.horizontal_fov_deg) ||
        sensor.horizontal_fov_deg <= 0.0F ||
        sensor.horizontal_fov_deg > 360.0F)
    {
        error = "sensor.horizontal_fov_deg 必須在 (0, 360] 度";
        return false;
    }
    if (!std::isfinite(sensor.vertical_fov_deg) ||
        sensor.vertical_fov_deg <= 0.0F ||
        sensor.vertical_fov_deg > 180.0F)
    {
        error = "sensor.vertical_fov_deg 必須在 (0, 180] 度";
        return false;
    }
    if (!std::isfinite(sensor.noise_std_m) || sensor.noise_std_m < 0.0F)
    {
        error = "sensor.noise_std_m 必須為有限非負值";
        return false;
    }
    if (!std::isfinite(sensor.dropout_ratio) ||
        sensor.dropout_ratio < 0.0F || sensor.dropout_ratio > 1.0F)
    {
        error = "sensor.dropout_ratio 必須在 [0, 1]";
        return false;
    }
    return true;
}

std::pair<PointCloud2Data, GenerationReport> ObstacleCloudGenerator::generate(
    const std::vector<ObstacleDefinition>& obstacles,
    const GenerationConfig& config,
    const SensorPose& sensor_pose)
{
    std::string error;
    if (!validate_config(config, error))
    {
        throw std::invalid_argument(error);
    }
    if (config.mode != CloudMode::ExplicitEmpty &&
        (!is_finite(sensor_pose.position_ned) ||
         !std::isfinite(sensor_pose.heading_rad) ||
         !std::isfinite(sensor_pose.pitch_rad)))
    {
        throw std::invalid_argument("sensor_pose 含有非有限值");
    }

    if (config.mode == CloudMode::ExplicitEmpty)
    {
        return {make_cloud(config.frame_id, {}), GenerationReport{}};
    }

    std::vector<Point3f> samples;
    std::mt19937 random_engine(config.random_seed);
    GenerationReport report;

    for (std::size_t obstacle_index = 0U;
         obstacle_index < obstacles.size();
         ++obstacle_index)
    {
        const auto& obstacle = obstacles.at(obstacle_index);
        if (!validate_obstacle(obstacle, error, obstacle_index))
        {
            throw std::invalid_argument(error);
        }

        std::vector<Point3f> obstacle_points;
        sample_obstacle(obstacle, config.sample_spacing_m, obstacle_points);
        for (const auto& point : obstacle_points)
        {
            if (config.mode == CloudMode::SyntheticSensor &&
                !is_visible(point, sensor_pose, config.sensor))
            {
                ++report.sampled_points;
                ++report.dropped_points;
                continue;
            }
            select_point(point, config, random_engine, samples, report);
        }
    }

    return {make_cloud(config.frame_id, samples), report};
}

}  // namespace perception
}  // namespace taiphoon
