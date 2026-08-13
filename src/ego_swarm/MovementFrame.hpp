/**
 * @file MovementFrame.hpp
 * @brief 機體水平命令轉換為固定 local NED 位移的純 C++ 模組
 */

#pragma once

#include <array>
#include <cmath>

#include "ego_swarm/EgoSwarmTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief 將 quaternion（w,x,y,z）轉成 NED yaw。
 *
 * @param quaternion 機體姿態四元數
 * @param[out] yaw_rad NED yaw，North 為 0、East 為正
 * @return true 表示四元數與結果皆為有限值
 */
inline bool quaternion_to_ned_yaw(
    const std::array<float, 4>& quaternion,
    double& yaw_rad)
{
    yaw_rad = 0.0;
    for (const float value : quaternion)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }

    const double w = quaternion.at(0);
    const double x = quaternion.at(1);
    const double y = quaternion.at(2);
    const double z = quaternion.at(3);
    yaw_rad = std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z));
    return std::isfinite(yaw_rad);
}

/**
 * @brief 以命令收到當下的機頭方向將 body 位移轉成 NED 位移。
 *
 * @param yaw_rad 命令收到當下的固定 yaw
 * @param forward_m body-forward 位移（公尺）
 * @param right_m body-right 位移（公尺）
 * @param[out] offset_ned 固定 local NED 位移
 * @return true 表示輸入有效
 */
inline bool body_horizontal_offset_to_ned(
    const double yaw_rad,
    const double forward_m,
    const double right_m,
    Vector3& offset_ned)
{
    offset_ned = Vector3{0.0, 0.0, 0.0};
    if (!is_finite(yaw_rad) || !is_finite(forward_m) ||
        !is_finite(right_m))
    {
        return false;
    }

    const double cosine = std::cos(yaw_rad);
    const double sine = std::sin(yaw_rad);
    offset_ned = Vector3{
        forward_m * cosine - right_m * sine,
        forward_m * sine + right_m * cosine,
        0.0};
    return is_finite_vector(offset_ned);
}

}  // namespace ego_swarm
}  // namespace taiphoon
