/**
 * @file MovementFrame.hpp
 * @brief 機體水平命令轉換為固定 local NED 位移的純 C++ 模組
 */

#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "ego_swarm/EgoSwarmTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/** @brief 全體編隊命令使用的共同 swarm_ned 移動方向。 */
enum class SwarmMoveDirection : uint8_t
{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

/**
 * @brief 將全體命令距離轉為共同 swarm_ned 位移。
 *
 * 全體編隊命令不使用各機 target yaw；所有 Agent 使用相同的 North/East/Down
 * 基準，才能讓 Formation center 以同一方向平移。
 */
inline bool common_swarm_ned_offset(
    const SwarmMoveDirection direction,
    const double distance_m,
    Vector3& offset_ned)
{
    offset_ned = Vector3{0.0, 0.0, 0.0};
    if (!is_finite(distance_m))
    {
        return false;
    }

    switch (direction)
    {
        case SwarmMoveDirection::FORWARD:
            offset_ned.at(0) = distance_m;
            break;
        case SwarmMoveDirection::BACKWARD:
            offset_ned.at(0) = -distance_m;
            break;
        case SwarmMoveDirection::LEFT:
            offset_ned.at(1) = -distance_m;
            break;
        case SwarmMoveDirection::RIGHT:
            offset_ned.at(1) = distance_m;
            break;
        case SwarmMoveDirection::UP:
            offset_ned.at(2) = -distance_m;
            break;
        case SwarmMoveDirection::DOWN:
            offset_ned.at(2) = distance_m;
            break;
    }
    return is_finite_vector(offset_ned);
}

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
 * @brief 以保存的目標機頭方向將 body 位移轉成 NED 位移。
 *
 * @param yaw_rad 目前保存的 target yaw
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
