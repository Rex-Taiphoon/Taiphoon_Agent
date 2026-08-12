/**
 * @file EgoSwarmTypes.hpp
 * @brief EGO-Swarm 純 C++ 核心共用資料型別與數值工具
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * 本檔案只提供不依賴 ROS2 或飛控訊息的資料型別，讓演算法模組與
 * ROS2 Adapter 之間維持清楚的 seam。
 */

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace taiphoon
{
namespace ego_swarm
{

/// 三維 NED 向量，單位依使用情境而定，位置通常為公尺。
using Vector3 = std::array<double, 3>;

/// GPS 全域座標，緯度與經度為度，高度為公尺 AMSL。
struct GeoCoordinate
{
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
};

/// 編隊槽位類型。
enum class FormationType
{
    CIRCLE = 0,
    HORI_LINEAR,
    VERT_LINEAR
};

/// 編隊槽位參數；圓形使用 radius_m，直線使用 spacing_m。
struct FormationConfig
{
    FormationType type{FormationType::CIRCLE};
    double radius_m{5.0};
    double spacing_m{3.0};
    double vertical_offset_m{0.0};
};

/// 浮點計算容差，避免在安全距離判斷中受微小數值誤差影響。
constexpr double EGO_SWARM_EPSILON_M = 1.0e-6;
/// 圓周率，供編隊與候選軌跡的週期計算使用。
constexpr double EGO_SWARM_PI = 3.14159265358979323846;
/// 候選軌跡取樣數上限，避免異常設定造成無界計算。
constexpr std::size_t EGO_SWARM_MAX_SAMPLE_COUNT = 2000U;
/// AgentTrajectory 對外全球 WGS84 座標 contract 的版本。
constexpr uint32_t GLOBAL_WGS84_FRAME_REVISION = 1U;

/**
 * @brief 檢查純量是否為有限值
 *
 * @param value 待檢查的浮點數
 * @return true 表示不是 NaN 或 Inf
 */
inline bool is_finite(const double value)
{
    return std::isfinite(value);
}

/**
 * @brief 檢查三維向量是否全部為有限值
 *
 * @param value 待檢查的三維向量
 * @return true 表示三個分量皆為有限值
 */
inline bool is_finite_vector(const Vector3& value)
{
    return is_finite(value.at(0)) &&
           is_finite(value.at(1)) &&
           is_finite(value.at(2));
}

/**
 * @brief 單一 planner cycle 使用的自身飛行狀態
 *
 * 位置、速度與加速度必須屬於同一個共同時間，讓 dynamic safety 與
 * emergency brake 不會把不同 callback 時刻的資料混在同一個決策中。
 */
struct VehiclePlanningState
{
    Vector3 position{0.0, 0.0, 0.0};
    Vector3 velocity{0.0, 0.0, 0.0};
    Vector3 acceleration{0.0, 0.0, 0.0};
    double timestamp_s{0.0};

    /**
     * @brief 檢查狀態是否可供 safety module 使用
     * @return true 表示位置、速度、加速度與 timestamp 均為有限值
     */
    bool is_finite() const
    {
        return is_finite_vector(position) &&
               is_finite_vector(velocity) &&
               is_finite_vector(acceleration) &&
               std::isfinite(timestamp_s);
    }
};

/**
 * @brief 計算三維向量的平方長度
 *
 * @param value 三維向量
 * @return 向量平方長度；輸入非有限時回傳零
 */
inline double squared_norm(const Vector3& value)
{
    if (!is_finite_vector(value))
    {
        return 0.0;
    }

    return value.at(0) * value.at(0) +
           value.at(1) * value.at(1) +
           value.at(2) * value.at(2);
}

/**
 * @brief 計算兩個機體中心的三維距離
 *
 * @param first 第一個機體中心位置
 * @param second 第二個機體中心位置
 * @return 三維中心距離；任一輸入非有限時回傳零
 */
inline double distance_between_centers(
    const Vector3& first,
    const Vector3& second)
{
    if (!is_finite_vector(first) || !is_finite_vector(second))
    {
        return 0.0;
    }

    const Vector3 difference{
        first.at(0) - second.at(0),
        first.at(1) - second.at(1),
        first.at(2) - second.at(2)};
    return std::sqrt(squared_norm(difference));
}

/**
 * @brief 計算兩個三維向量的加法
 *
 * @param first 第一個向量
 * @param second 第二個向量
 * @return 加法結果
 */
inline Vector3 add_vectors(const Vector3& first, const Vector3& second)
{
    return Vector3{
        first.at(0) + second.at(0),
        first.at(1) + second.at(1),
        first.at(2) + second.at(2)};
}

/**
 * @brief 計算兩個三維向量的減法
 *
 * @param first 被減向量
 * @param second 減數向量
 * @return 減法結果
 */
inline Vector3 subtract_vectors(
    const Vector3& first,
    const Vector3& second)
{
    return Vector3{
        first.at(0) - second.at(0),
        first.at(1) - second.at(1),
        first.at(2) - second.at(2)};
}

/**
 * @brief 將三維向量乘以純量
 *
 * @param value 三維向量
 * @param scalar 乘數
 * @return 縮放後向量
 */
inline Vector3 scale_vector(const Vector3& value, const double scalar)
{
    return Vector3{
        value.at(0) * scalar,
        value.at(1) * scalar,
        value.at(2) * scalar};
}

/**
 * @brief 計算兩個向量的線性插值
 *
 * @param first 起點向量
 * @param second 終點向量
 * @param ratio 插值比例
 * @return 插值結果
 */
inline Vector3 interpolate_vectors(
    const Vector3& first,
    const Vector3& second,
    const double ratio)
{
    return add_vectors(
        first,
        scale_vector(subtract_vectors(first, second), -ratio));
}

}  // namespace ego_swarm
}  // namespace taiphoon
