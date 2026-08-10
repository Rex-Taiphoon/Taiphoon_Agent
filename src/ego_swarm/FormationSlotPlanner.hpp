/**
 * @file FormationSlotPlanner.hpp
 * @brief 非連續 Agent ID 編隊槽位計算模組
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * 本模組將 Agent ID 排序後轉換為穩定 rank，避免把 Agent ID 當成
 * 連續陣列索引，讓多機離線或重新加入時仍能產生可預期的槽位。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>

#include "ego_swarm/EgoSwarmTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// 編隊槽位結果。
struct FormationSlot
{
    int agent_id{0};
    std::size_t rank{0};
    Vector3 offset_ned{0.0, 0.0, 0.0};
    bool is_valid{false};
};

/**
 * @brief 計算穩定編隊槽位的深層模組
 *
 * 對外介面只暴露 Agent ID、目前成員與編隊參數；排序、重複檢查與
 * 幾何計算都集中在 implementation，讓 ROS2 Node 不需知道槽位細節。
 */
class FormationSlotPlanner
{
public:
    /**
     * @brief 計算指定 Agent 的編隊槽位
     *
     * @param agent_id 目標 Agent ID
     * @param agent_ids 目前參與編隊的 Agent ID 列表
     * @param config 編隊類型與幾何參數
     * @param[out] out_slot 計算後的槽位偏移
     * @return true 表示輸入與槽位皆有效
     */
    static bool compute_slot(
        const int agent_id,
        const std::vector<int>& agent_ids,
        const FormationConfig& config,
        FormationSlot& out_slot)
    {
        out_slot = FormationSlot{};
        std::vector<int> sorted_ids;
        if (!prepare_ids(agent_ids, sorted_ids))
        {
            return false;
        }

        const auto id_iterator = std::find(
            sorted_ids.begin(), sorted_ids.end(), agent_id);
        if (id_iterator == sorted_ids.end())
        {
            return false;
        }

        const std::size_t rank = static_cast<std::size_t>(
            std::distance(sorted_ids.begin(), id_iterator));
        Vector3 offset{0.0, 0.0, 0.0};
        if (!compute_offset(rank, sorted_ids.size(), config, offset))
        {
            return false;
        }

        out_slot.agent_id = agent_id;
        out_slot.rank = rank;
        out_slot.offset_ned = offset;
        out_slot.is_valid = true;
        return true;
    }

    /**
     * @brief 計算所有 Agent 的編隊槽位
     *
     * @param agent_ids 目前參與編隊的 Agent ID 列表
     * @param config 編隊類型與幾何參數
     * @param[out] out_slots 依 Agent ID 遞增排序的槽位結果
     * @return true 表示所有輸入皆有效
     */
    static bool compute_all_slots(
        const std::vector<int>& agent_ids,
        const FormationConfig& config,
        std::vector<FormationSlot>& out_slots)
    {
        out_slots.clear();
        std::vector<int> sorted_ids;
        if (!prepare_ids(agent_ids, sorted_ids))
        {
            return false;
        }

        out_slots.reserve(sorted_ids.size());
        for (const int agent_id : sorted_ids)
        {
            FormationSlot slot;
            if (!compute_slot(agent_id, sorted_ids, config, slot))
            {
                out_slots.clear();
                return false;
            }
            out_slots.push_back(slot);
        }
        return true;
    }

    /**
     * @brief 檢查槽位間的機體中心距離
     *
     * @param slots 槽位列表
     * @param minimum_distance_m 最小允許中心距離
     * @return true 表示所有槽位都符合距離要求
     */
    static bool are_slots_safe(
        const std::vector<FormationSlot>& slots,
        const double minimum_distance_m)
    {
        if (!is_finite(minimum_distance_m) || minimum_distance_m < 0.0)
        {
            return false;
        }

        for (const FormationSlot& slot : slots)
        {
            if (!slot.is_valid || !is_finite_vector(slot.offset_ned))
            {
                return false;
            }
        }

        for (std::size_t first = 0; first < slots.size(); ++first)
        {
            for (std::size_t second = first + 1;
                 second < slots.size();
                 ++second)
            {
                const double center_distance = distance_between_centers(
                    slots.at(first).offset_ned,
                    slots.at(second).offset_ned);
                if (center_distance + EGO_SWARM_EPSILON_M <
                    minimum_distance_m)
                {
                    return false;
                }
            }
        }
        return true;
    }

private:
    /**
     * @brief 排序並檢查 Agent ID 列表
     *
     * @param agent_ids 原始 Agent ID 列表
     * @param[out] sorted_ids 排序後且無重複的 ID 列表
     * @return true 表示列表可用
     */
    static bool prepare_ids(
        const std::vector<int>& agent_ids,
        std::vector<int>& sorted_ids)
    {
        sorted_ids = agent_ids;
        if (sorted_ids.empty())
        {
            return false;
        }

        std::sort(sorted_ids.begin(), sorted_ids.end());
        const auto duplicate_iterator = std::adjacent_find(
            sorted_ids.begin(), sorted_ids.end());
        return duplicate_iterator == sorted_ids.end();
    }

    /**
     * @brief 依 rank 產生局部 NED 槽位偏移
     *
     * @param rank Agent 在排序成員中的 rank
     * @param count 編隊成員數量
     * @param config 編隊幾何參數
     * @param[out] out_offset 槽位偏移
     * @return true 表示幾何參數有效
     */
    static bool compute_offset(
        const std::size_t rank,
        const std::size_t count,
        const FormationConfig& config,
        Vector3& out_offset)
    {
        out_offset = Vector3{0.0, 0.0, 0.0};
        if (count == 0U || !is_finite(config.radius_m) ||
            !is_finite(config.spacing_m) ||
            !is_finite(config.vertical_offset_m) ||
            config.radius_m < 0.0 || config.spacing_m < 0.0)
        {
            return false;
        }

        switch (config.type)
        {
            case FormationType::CIRCLE:
            {
                const double angle = 2.0 * EGO_SWARM_PI *
                    static_cast<double>(rank) / static_cast<double>(count);
                out_offset = Vector3{
                    config.radius_m * std::cos(angle),
                    config.radius_m * std::sin(angle),
                    config.vertical_offset_m};
                break;
            }
            case FormationType::HORI_LINEAR:
            {
                const double midpoint =
                    (static_cast<double>(count) - 1.0) / 2.0;
                out_offset = Vector3{
                    0.0,
                    config.spacing_m *
                        (static_cast<double>(rank) - midpoint),
                    config.vertical_offset_m};
                break;
            }
            case FormationType::VERT_LINEAR:
            {
                const double midpoint =
                    (static_cast<double>(count) - 1.0) / 2.0;
                out_offset = Vector3{
                    0.0,
                    0.0,
                    config.vertical_offset_m + config.spacing_m *
                        (static_cast<double>(rank) - midpoint)};
                break;
            }
            default:
            {
                return false;
            }
        }

        return is_finite_vector(out_offset);
    }
};

}  // namespace ego_swarm
}  // namespace taiphoon
