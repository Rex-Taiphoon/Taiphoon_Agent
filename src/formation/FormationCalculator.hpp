/**
 * @file FormationCalculator.hpp
 * @brief 編隊計算器 — 純邏輯類別，計算質心與編隊目標位置
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 不依賴 ROS2，僅執行編隊數學計算。
 * 由 MissionPlanner 節點在編隊控制迴圈中呼叫。
 */

#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <cstdint>

namespace taiphoon
{
namespace formation
{

/// 編隊類型
enum class FormationType : uint8_t
{
    NONE = 0,
    CIRCLE,
    HORI_LINEAR,
    VERT_LINEAR
};

/// 同儕無人機位置結構
struct PeerPosition
{
    std::array<float, 3> position_ned{0.0F, 0.0F, 0.0F};
    bool online{false};
};

/// 編隊參數
struct FormationParams
{
    float param1{0.0F};  ///< 半徑 (CIRCLE) 或間距 (LINEAR)
    float param2{0.0F};  ///< 高度 (CIRCLE 與 HORI_LINEAR)
};

constexpr float PI = 3.14159265358979323846F;

/**
 * @brief 編隊計算器
 *
 * 提供靜態方法計算：
 * 1. 多機質心 (centroid)
 * 2. 各種編隊類型下的目標位置
 */
class FormationCalculator
{
public:
    /**
     * @brief 計算線上無人機數量
     *
     * @param peers 同儕位置列表
     * @return size_t 線上數量
     */
    static size_t count_online(const std::vector<PeerPosition>& peers)
    {
        size_t count = 0;
        for (const auto& p : peers)
        {
            if (p.online) ++count;
        }
        return count;
    }

    /**
     * @brief 計算所有線上無人機的質心
     *
     * 僅計算 online=true 的無人機。若無線上無人機，回傳 (0,0,0)。
     *
     * @param peers 同儕位置列表
     * @return std::array<float, 3> 質心 NED 座標 (x, y, z)
     */
    static std::array<float, 3> compute_centroid(
        const std::vector<PeerPosition>& peers)
    {
        std::array<float, 3> centroid{0.0F, 0.0F, 0.0F};
        size_t count = 0;

        for (const auto& p : peers)
        {
            if (!p.online) continue;
            centroid[0] += p.position_ned[0];
            centroid[1] += p.position_ned[1];
            centroid[2] += p.position_ned[2];
            ++count;
        }

        if (count > 0)
        {
            centroid[0] /= static_cast<float>(count);
            centroid[1] /= static_cast<float>(count);
            centroid[2] /= static_cast<float>(count);
        }

        return centroid;
    }

    /**
     * @brief 計算編隊中此無人機的目標位置
     *
     * @param centroid    所有無人機的質心 NED
     * @param type        編隊類型
     * @param params      編隊參數 (半徑/間距/高度)
     * @param self_index  自身在編隊中的索引 (0-based)
     * @param online_count 線上無人機總數
     * @return std::array<float, 4> 目標 NED 位置 (x, y, z, yaw_deg)
     *         yaw_deg = NaN 表示不控制偏航
     */
    static std::array<float, 4> compute_formation_target(
        const std::array<float, 3>& centroid,
        FormationType type,
        const FormationParams& params,
        size_t self_index,
        size_t online_count)
    {
        std::array<float, 4> target{
            centroid[0], centroid[1], centroid[2],
            std::numeric_limits<float>::quiet_NaN()};

        if (online_count == 0) return target;

        switch (type)
        {
            case FormationType::CIRCLE:
            {
                // 環形編隊：等角分布
                float angle = 2.0F * PI *
                    static_cast<float>(self_index) /
                    static_cast<float>(online_count);
                float radius = params.param1;
                target[0] = centroid[0] + radius * std::cos(angle);
                target[1] = centroid[1] + radius * std::sin(angle);
                target[2] = -params.param2;  // NED: Z 向下為正，高度取負
                break;
            }

            case FormationType::HORI_LINEAR:
            {
                // 水平直列：沿 Y 軸排列
                float spacing = params.param1;
                float mid = (static_cast<float>(online_count) - 1.0F) / 2.0F;
                float offset = spacing *
                    (static_cast<float>(self_index) - mid);
                target[0] = centroid[0];
                target[1] = centroid[1] + offset;
                target[2] = -params.param2;
                break;
            }

            case FormationType::VERT_LINEAR:
            {
                // 垂直直列：沿 Z 軸排列
                float spacing = params.param1;
                float mid = (static_cast<float>(online_count) - 1.0F) / 2.0F;
                float offset = spacing *
                    (static_cast<float>(self_index) - mid);
                target[0] = centroid[0];
                target[1] = centroid[1];
                target[2] = centroid[2] + offset;
                break;
            }

            default:
                // NONE → 回傳質心
                break;
        }

        return target;
    }
};

}  // namespace formation
}  // namespace taiphoon
