/**
 * @file SwarmFrame.hpp
 * @brief GPS 座標轉換至共同 swarm_ned 的純 C++ 模組
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * 本模組提供區域 WGS84 NED 轉換，讓每台 Agent 的 GPS 位置能在
 * 同一個共同座標系中進行編隊與同儕避碰計算。
 */

#pragma once

#include <cmath>
#include <cstdint>

#include "ego_swarm/EgoSwarmTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// WGS84 橢球體長半軸，單位為公尺。
constexpr double WGS84_SEMI_MAJOR_AXIS_M = 6378137.0;
/// WGS84 第一偏心率平方。
constexpr double WGS84_FIRST_ECCENTRICITY_SQUARED = 6.6943799901413165e-3;
/// 角度轉弧度常數。
constexpr double DEGREES_TO_RADIANS = 0.017453292519943295;
/// 弧度轉角度常數。
constexpr double RADIANS_TO_DEGREES = 57.2957795130823208768;
/// 緯度／經度原點相容性容差，單位為度。
constexpr double SWARM_FRAME_ORIGIN_LAT_LON_EPSILON_DEG = 1.0e-9;
/// 高度原點相容性容差，單位為公尺。
constexpr double SWARM_FRAME_ORIGIN_ALTITUDE_EPSILON_M = 1.0e-3;

/**
 * @brief WGS84 與局部 metric NED 座標轉換器
 *
 * 設定原點後，所有 GPS 座標都會轉成 x=North、y=East、z=Down。
 * 轉換採用原點附近的 WGS84 曲率半徑，適合機群尺度的局部座標計算。
 */
class SwarmFrame
{
public:
    /**
     * @brief 建立尚未設定原點的座標轉換器
     */
    SwarmFrame() = default;

    /**
     * @brief 檢查 GPS 座標的有限值與地理範圍
     *
     * @param coordinate 待檢查的 GPS 座標
     * @return true 表示座標可用
     */
    static bool is_valid_geodetic(const GeoCoordinate& coordinate)
    {
        constexpr double MIN_LATITUDE_DEG = -90.0;
        constexpr double MAX_LATITUDE_DEG = 90.0;
        constexpr double MIN_LONGITUDE_DEG = -180.0;
        constexpr double MAX_LONGITUDE_DEG = 180.0;

        return is_finite(coordinate.latitude_deg) &&
               is_finite(coordinate.longitude_deg) &&
               is_finite(coordinate.altitude_m) &&
               coordinate.latitude_deg >= MIN_LATITUDE_DEG &&
               coordinate.latitude_deg <= MAX_LATITUDE_DEG &&
               coordinate.longitude_deg >= MIN_LONGITUDE_DEG &&
               coordinate.longitude_deg <= MAX_LONGITUDE_DEG;
    }

    /**
     * @brief 檢查兩個 GPS 原點是否代表同一個共同座標系
     *
     * 訊息序列化可能造成極小浮點誤差；集中使用相同容差，避免不同
     * ROS2 組裝層對 frame identity 得出不一致結果。
     *
     * @param left 第一個 GPS 原點
     * @param right 第二個 GPS 原點
     * @return true 表示兩個原點有效且在相容容差內
     */
    static bool are_origins_compatible(
        const GeoCoordinate& left,
        const GeoCoordinate& right)
    {
        if (!is_valid_geodetic(left) || !is_valid_geodetic(right))
        {
            return false;
        }

        return std::abs(left.latitude_deg - right.latitude_deg) <=
                   SWARM_FRAME_ORIGIN_LAT_LON_EPSILON_DEG &&
            std::abs(left.longitude_deg - right.longitude_deg) <=
                SWARM_FRAME_ORIGIN_LAT_LON_EPSILON_DEG &&
            std::abs(left.altitude_m - right.altitude_m) <=
                SWARM_FRAME_ORIGIN_ALTITUDE_EPSILON_M;
    }

    /**
     * @brief 設定共同座標系原點
     *
     * @param origin GPS 原點
     * @return true 表示原點有效並已設定
     */
    bool set_origin(const GeoCoordinate& origin)
    {
        if (!is_valid_geodetic(origin))
        {
            return false;
        }

        if (has_origin_)
        {
            return are_origins_compatible(origin_, origin);
        }

        origin_ = origin;
        has_origin_ = true;
        frame_revision_ = 1U;
        return true;
    }

    /**
     * @brief 檢查是否已設定有效原點
     *
     * @return true 表示可進行 GPS 轉換
     */
    bool has_origin() const
    {
        return has_origin_;
    }

    /**
     * @brief 取得目前共同座標系原點
     *
     * @return GPS 原點；未設定時為全零結構
     */
    GeoCoordinate origin() const
    {
        return origin_;
    }

    /**
     * @brief 取得共同座標系的 revision／epoch
     *
     * revision 在第一次成功設定 origin 時建立，之後 origin freeze，
     * 因此同一個 frame lifetime 內不會改變。
     *
     * @return frame revision；尚未設定 origin 時為零
     */
    uint32_t frame_revision() const
    {
        return frame_revision_;
    }

    /**
     * @brief 將 GPS 座標轉換為共同 swarm_ned
     *
     * @param coordinate 待轉換的 GPS 座標
     * @param[out] out_ned 轉換後的 North、East、Down 位置
     * @return true 表示轉換成功；失敗時 out_ned 會被清為零
     */
    bool to_swarm_ned(
        const GeoCoordinate& coordinate,
        Vector3& out_ned) const
    {
        out_ned = Vector3{0.0, 0.0, 0.0};
        if (!has_origin_ || !is_valid_geodetic(coordinate))
        {
            return false;
        }

        const double latitude_rad = origin_.latitude_deg * DEGREES_TO_RADIANS;
        const double sine_latitude = std::sin(latitude_rad);
        const double denominator = std::sqrt(
            1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED *
                sine_latitude * sine_latitude);
        if (!is_finite(denominator) ||
            denominator < EGO_SWARM_EPSILON_M)
        {
            return false;
        }

        const double prime_vertical_radius =
            WGS84_SEMI_MAJOR_AXIS_M / denominator;
        const double meridian_radius =
            WGS84_SEMI_MAJOR_AXIS_M *
            (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED) /
            (denominator * denominator * denominator);
        if (!is_finite(prime_vertical_radius) ||
            !is_finite(meridian_radius))
        {
            return false;
        }

        const double delta_latitude_rad =
            (coordinate.latitude_deg - origin_.latitude_deg) *
            DEGREES_TO_RADIANS;
        const double delta_longitude_deg = shortest_longitude_delta_deg(
            coordinate.longitude_deg - origin_.longitude_deg);
        const double delta_longitude_rad =
            delta_longitude_deg * DEGREES_TO_RADIANS;

        out_ned.at(0) = meridian_radius * delta_latitude_rad;
        out_ned.at(1) = prime_vertical_radius *
                       std::cos(latitude_rad) * delta_longitude_rad;
        out_ned.at(2) = origin_.altitude_m - coordinate.altitude_m;

        if (!is_finite_vector(out_ned))
        {
            out_ned = Vector3{0.0, 0.0, 0.0};
            return false;
        }

        return true;
    }

    /**
     * @brief 將局部 NED 位置轉回 WGS84 geodetic 座標
     *
     * 發送 AgentTrajectory 前將內部公尺座標轉成全球座標；接收端會
     * 使用自己的 frame 再轉回 NED，因此不會傳遞發送端的 local Home。
     * @param ned 局部 North、East、Down 位置，單位為公尺
     * @param[out] out_coordinate WGS84 緯度、經度與高度
     * @return true 表示轉換成功
     */
    bool from_swarm_ned(
        const Vector3& ned,
        GeoCoordinate& out_coordinate) const
    {
        out_coordinate = GeoCoordinate{};
        if (!has_origin_ || !is_finite_vector(ned))
        {
            return false;
        }

        const double latitude_rad = origin_.latitude_deg *
            DEGREES_TO_RADIANS;
        const double sine_latitude = std::sin(latitude_rad);
        const double denominator = std::sqrt(
            1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED *
                sine_latitude * sine_latitude);
        if (!is_finite(denominator) ||
            denominator < EGO_SWARM_EPSILON_M)
        {
            return false;
        }

        const double prime_vertical_radius =
            WGS84_SEMI_MAJOR_AXIS_M / denominator;
        const double meridian_radius =
            WGS84_SEMI_MAJOR_AXIS_M *
            (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED) /
            (denominator * denominator * denominator);
        const double cosine_latitude = std::cos(latitude_rad);
        if (!is_finite(prime_vertical_radius) ||
            !is_finite(meridian_radius) ||
            std::abs(cosine_latitude) < EGO_SWARM_EPSILON_M)
        {
            return false;
        }

        out_coordinate.latitude_deg = origin_.latitude_deg +
            ned.at(0) / meridian_radius * RADIANS_TO_DEGREES;
        out_coordinate.longitude_deg = origin_.longitude_deg +
            ned.at(1) / (prime_vertical_radius * cosine_latitude) *
            RADIANS_TO_DEGREES;
        out_coordinate.altitude_m = origin_.altitude_m - ned.at(2);
        return is_valid_geodetic(out_coordinate);
    }

private:
    /**
     * @brief 將經度差限制到最短角度
     *
     * @param delta_deg 原始經度差
     * @return -180 到 180 度之間的經度差
     */
    static double shortest_longitude_delta_deg(const double delta_deg)
    {
        double normalized = std::fmod(delta_deg + 180.0, 360.0);
        if (normalized < 0.0)
        {
            normalized += 360.0;
        }
        return normalized - 180.0;
    }

    GeoCoordinate origin_{};
    bool has_origin_{false};
    uint32_t frame_revision_{0U};
};

}  // namespace ego_swarm
}  // namespace taiphoon
