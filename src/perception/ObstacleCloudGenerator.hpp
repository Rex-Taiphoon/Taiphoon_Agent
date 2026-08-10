/**
 * @file ObstacleCloudGenerator.hpp
 * @brief 障礙物幾何採樣與 PointCloud2 相容資料生成介面
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * 本檔案只包含純 C++ 介面，不依賴 ROS2、sensor_msgs 或 PCL。
 * 產生的 PointCloud2Data 保留 ROS2 PointCloud2 所需的欄位與位元組排列，
 * 未來可由獨立 Adapter 轉換為 sensor_msgs::msg::PointCloud2。
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace taiphoon
{
namespace perception
{

constexpr float OBSTACLE_CLOUD_DEFAULT_SAMPLE_SPACING_M = 0.5F;
constexpr float OBSTACLE_CLOUD_DEFAULT_RANGE_M = 20.0F;
constexpr float OBSTACLE_CLOUD_DEFAULT_HORIZONTAL_FOV_DEG = 270.0F;
constexpr float OBSTACLE_CLOUD_DEFAULT_VERTICAL_FOV_DEG = 90.0F;
constexpr float OBSTACLE_CLOUD_DEFAULT_NOISE_STD_M = 0.0F;
constexpr float OBSTACLE_CLOUD_DEFAULT_DROPOUT_RATIO = 0.0F;
constexpr std::uint32_t OBSTACLE_CLOUD_DEFAULT_RANDOM_SEED = 20260810U;
constexpr std::uint8_t POINT_CLOUD_FLOAT32_DATATYPE = 7U;
constexpr std::uint32_t POINT_CLOUD_POINT_STEP_BYTES = 12U;
constexpr float OBSTACLE_CLOUD_PI = 3.14159265358979323846F;

using Point3f = std::array<float, 3>;

/**
 * @brief 障礙物幾何類型
 */
enum class ObstacleType
{
    Box,
    Cylinder,
    Wall
};

/**
 * @brief PointCloud2 資料選擇模式
 */
enum class CloudMode
{
    GroundTruthMap,
    SensorEmulation
};

/**
 * @brief 一個障礙物的幾何描述
 *
 * `size_m` 對 Box 與 Wall 使用，排列為 x、y、z 尺寸。
 * `radius_m` 與 `height_m` 對 Cylinder 使用，Cylinder 的軸線固定為 NED 的 z 軸。
 */
struct ObstacleDefinition
{
    ObstacleType type{ObstacleType::Box};
    Point3f center_ned{0.0F, 0.0F, 0.0F};
    Point3f size_m{1.0F, 1.0F, 1.0F};
    float radius_m{1.0F};
    float height_m{1.0F};
};

/**
 * @brief 感測器在 swarm_ned 中的姿態
 *
 * heading_rad 以 +x（North）為 0，朝 +y（East）為正方向。
 * pitch_rad 為向上為正；這樣可在 NED 座標中明確表達垂直視野。
 */
struct SensorPose
{
    Point3f position_ned{0.0F, 0.0F, 0.0F};
    float heading_rad{0.0F};
    float pitch_rad{0.0F};
};

/**
 * @brief sensor_emulation 的感測模型參數
 */
struct SensorModel
{
    float range_m{OBSTACLE_CLOUD_DEFAULT_RANGE_M};
    float horizontal_fov_deg{OBSTACLE_CLOUD_DEFAULT_HORIZONTAL_FOV_DEG};
    float vertical_fov_deg{OBSTACLE_CLOUD_DEFAULT_VERTICAL_FOV_DEG};
    float noise_std_m{OBSTACLE_CLOUD_DEFAULT_NOISE_STD_M};
    float dropout_ratio{OBSTACLE_CLOUD_DEFAULT_DROPOUT_RATIO};
};

/**
 * @brief 障礙物雲生成設定
 *
 * 此結構可直接由 ROS2 parameters 或 YAML 設定檔映射，純邏輯模組不負責解析
 * YAML，以避免將 yaml-cpp 或 ROS2 依賴帶入幾何演算法。
 */
struct GenerationConfig
{
    std::string frame_id{"swarm_ned"};
    CloudMode mode{CloudMode::GroundTruthMap};
    float sample_spacing_m{OBSTACLE_CLOUD_DEFAULT_SAMPLE_SPACING_M};
    SensorModel sensor{};
    std::uint32_t random_seed{OBSTACLE_CLOUD_DEFAULT_RANDOM_SEED};
};

/**
 * @brief PointCloud2 欄位描述
 */
struct PointCloud2Field
{
    std::string name{};
    std::uint32_t offset{0U};
    std::uint8_t datatype{POINT_CLOUD_FLOAT32_DATATYPE};
    std::uint32_t count{1U};
};

/**
 * @brief 不依賴 ROS2 的 PointCloud2 中介資料
 *
 * points 供純邏輯演算法使用；data 使用 little-endian 的 x/y/z float32 連續排列，
 * 可由未來 ROS2 Adapter 填入 sensor_msgs::msg::PointCloud2::data。
 */
struct PointCloud2Data
{
    std::string frame_id{"swarm_ned"};
    std::uint32_t height{1U};
    std::uint32_t width{0U};
    bool is_bigendian{false};
    std::uint32_t point_step{POINT_CLOUD_POINT_STEP_BYTES};
    std::uint32_t row_step{0U};
    bool is_dense{true};
    std::vector<PointCloud2Field> fields{};
    std::vector<Point3f> points{};
    std::vector<std::uint8_t> data{};

    /**
     * @brief 檢查所有點與序列化資料是否為有限值
     *
     * @return true 表示所有點均為有限值，且資料長度符合 PointCloud2 排列
     */
    bool has_only_finite_points() const;
};

/**
 * @brief 生成結果統計
 */
struct GenerationReport
{
    std::size_t sampled_points{0U};
    std::size_t selected_points{0U};
    std::size_t dropped_points{0U};
};

/**
 * @brief 幾何採樣與障礙物雲生成器
 *
 * 這是感測模組的外部 seam：呼叫者只需提供障礙物、模式、感測器參數與姿態，
 * 不需要知道 box、wall、cylinder 的採樣細節。實作完全無 ROS2 副作用，
 * 因而可由 ROS2 Node 或測試共用。
 */
class ObstacleCloudGenerator
{
public:
    /**
     * @brief 驗證生成設定
     *
     * @param config 待驗證的生成設定
     * @param error 輸出第一個驗證失敗原因
     * @return true 表示設定可安全用於生成
     */
    static bool validate_config(const GenerationConfig& config, std::string& error);

    /**
     * @brief 依幾何與感測模型生成 PointCloud2 相容資料
     *
     * GroundTruthMap 會保留所有幾何採樣點；SensorEmulation 會依感測器位置、
     * heading、pitch、range、FOV、雜訊與 dropout 選擇資料。
     *
     * @param obstacles 障礙物定義列表
     * @param config 生成模式與採樣設定
     * @param sensor_pose 感測器在 swarm_ned 中的姿態
     * @return PointCloud2 相容資料與生成統計
     * @throws std::invalid_argument 當設定或障礙物幾何無效時拋出
     */
    static std::pair<PointCloud2Data, GenerationReport> generate(
        const std::vector<ObstacleDefinition>& obstacles,
        const GenerationConfig& config,
        const SensorPose& sensor_pose = SensorPose{});
};

}  // namespace perception
}  // namespace taiphoon
