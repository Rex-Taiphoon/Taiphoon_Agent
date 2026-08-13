/**
 * @file FormationTransition.hpp
 * @brief 編隊轉換前序 Agent 到位條件的純 C++ 判斷模組
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <vector>

#include "ego_swarm/EgoSwarmTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief 編隊轉換判斷所需的單一前序 Agent snapshot。
 *
 * `has_fresh_trajectory` 由 ROS2 adapter 依 PeerTrajectoryTable 驗證後
 * 填入；本模組只判斷位置與速度，不依賴 ROS2 或訊息型別。
 */
struct FormationTransitionObservation
{
    int agent_id{0};
    Vector3 position{0.0, 0.0, 0.0};
    Vector3 target{0.0, 0.0, 0.0};
    Vector3 velocity{0.0, 0.0, 0.0};
    bool has_fresh_trajectory{false};
};

inline double distance_to_segment(
    const Vector3& point,
    const Vector3& segment_start,
    const Vector3& segment_end)
{
    if (!is_finite_vector(point) || !is_finite_vector(segment_start) ||
        !is_finite_vector(segment_end))
    {
        return 0.0;
    }
    const Vector3 direction = subtract_vectors(segment_end, segment_start);
    const double length_squared = squared_norm(direction);
    if (!is_finite(length_squared) || length_squared <= EGO_SWARM_EPSILON_M)
    {
        return distance_between_centers(point, segment_start);
    }
    const Vector3 from_start = subtract_vectors(point, segment_start);
    const double projection =
        (from_start.at(0) * direction.at(0) +
         from_start.at(1) * direction.at(1) +
         from_start.at(2) * direction.at(2)) / length_squared;
    const double clamped_projection = std::max(0.0, std::min(1.0, projection));
    const Vector3 closest = add_vectors(
        segment_start, scale_vector(direction, clamped_projection));
    return distance_between_centers(point, closest);
}

/**
 * @brief 依排序後 Agent ID 判斷前序機體是否已可釋放後續機體。
 *
 * 每台 Agent 都以相同 FormationEpoch、相同 Agent ID 順序獨立執行此
 * 判斷。前序機體必須同時具備 fresh trajectory、已在新槽位容許範圍內、
 * 且速度低於門檻；不要求訊息一定標記為 terminal_hold，避免實際飛控
 * 已到位但 heartbeat 標記尚未切換時永久等待。
 */
class FormationTransition
{
public:
    /**
     * @brief 依固定 snapshot 找出不穿越靜止成員的 deterministic 移動順序。
     *
     * 每次只釋放一台機體；其餘機體以目前位置或已完成槽位作為靜態
     * obstacle。候選優先使用直線，受阻時只檢查固定高度的 North/East
     * 側向折線，不用上方或下方繞行；若排序優先的候選導致後續無解，
     * 則依 Agent ID 順序回溯嘗試下一個候選。所有 Agent 對同一個
     * FormationEpoch 會得到同一順序。
     */
    static bool compute_safe_order(
        const std::vector<int>& ordered_member_ids,
        const std::vector<FormationTransitionObservation>& observations,
        const double minimum_distance_m,
        std::vector<int>& out_order)
    {
        return compute_safe_order(
            ordered_member_ids, observations, minimum_distance_m, 0.0,
            out_order);
    }

    /**
     * @brief 以直線與固定高度的左右水平繞行候選計算安全釋放順序。
     *
     * `horizontal_offset_m` 只作用於 North/East 中段控制點，Z 仍沿起點
     * 到目標的原有直線插值，因而不會把上升或下降當成避碰繞路。當直線
     * 被目前成員阻擋時，依固定順序嘗試正向側移、反向側移；所有 Agent
     * 對相同 FormationEpoch 會得到相同的 deterministic 順序。
     *
     * @param ordered_member_ids 編隊成員 ID
     * @param observations 成員目前位置與目標槽位
     * @param minimum_distance_m 最小允許中心距離
     * @param horizontal_offset_m 水平候選中段偏移量
     * @param[out] out_order 安全釋放順序
     * @return true 表示找到完整安全順序
     */
    static bool compute_safe_order(
        const std::vector<int>& ordered_member_ids,
        const std::vector<FormationTransitionObservation>& observations,
        const double minimum_distance_m,
        const double horizontal_offset_m,
        std::vector<int>& out_order)
    {
        out_order.clear();
        if (ordered_member_ids.empty() || !is_finite(minimum_distance_m) ||
            minimum_distance_m < 0.0 || !is_finite(horizontal_offset_m) ||
            horizontal_offset_m < 0.0)
        {
            return false;
        }
        std::map<int, FormationTransitionObservation> by_id;
        for (const auto& observation : observations)
        {
            if (observation.agent_id <= 0 ||
                !is_finite_vector(observation.position) ||
                !is_finite_vector(observation.target) ||
                by_id.find(observation.agent_id) != by_id.end())
            {
                return false;
            }
            by_id.emplace(observation.agent_id, observation);
        }
        std::vector<int> remaining = ordered_member_ids;
        std::sort(remaining.begin(), remaining.end());
        if (std::adjacent_find(remaining.begin(), remaining.end()) !=
                remaining.end() || remaining.size() != by_id.size())
        {
            return false;
        }
        for (const int agent_id : remaining)
        {
            if (by_id.find(agent_id) == by_id.end())
            {
                return false;
            }
        }

        const auto candidate_is_safe =
            [minimum_distance_m, horizontal_offset_m](
                const FormationTransitionObservation& candidate,
                const std::map<int, FormationTransitionObservation>& state,
                const double lateral_sign)
        {
            std::vector<Vector3> path_points;
            path_points.push_back(candidate.position);
            const double side_offset_m = horizontal_offset_m * lateral_sign;
            if (std::abs(side_offset_m) > EGO_SWARM_EPSILON_M)
            {
                const Vector3 direction = subtract_vectors(
                    candidate.target, candidate.position);
                const double horizontal_norm = std::sqrt(
                    direction.at(0) * direction.at(0) +
                    direction.at(1) * direction.at(1));
                Vector3 lateral{1.0, 0.0, 0.0};
                if (horizontal_norm > EGO_SWARM_EPSILON_M)
                {
                    lateral = Vector3{
                        -direction.at(1) / horizontal_norm,
                        direction.at(0) / horizontal_norm,
                        0.0};
                }
                const Vector3 midpoint = add_vectors(
                    interpolate_vectors(
                        candidate.position, candidate.target, 0.5),
                    scale_vector(lateral, side_offset_m));
                path_points.push_back(midpoint);
            }
            path_points.push_back(candidate.target);

            const auto path_is_safe =
                [&path_points, minimum_distance_m](
                    const Vector3& point)
            {
                double minimum_path_distance =
                    std::numeric_limits<double>::infinity();
                for (std::size_t index = 0U;
                     index + 1U < path_points.size(); ++index)
                {
                    minimum_path_distance = std::min(
                        minimum_path_distance,
                        distance_to_segment(
                            point, path_points.at(index),
                            path_points.at(index + 1U)));
                }
                return is_finite(minimum_path_distance) &&
                    minimum_path_distance + EGO_SWARM_EPSILON_M >=
                    minimum_distance_m;
            };

            for (const auto& entry : state)
            {
                const auto& other = entry.second;
                if (other.agent_id == candidate.agent_id)
                {
                    continue;
                }
                const double endpoint_distance = distance_between_centers(
                    candidate.target, other.position);
                if (!is_finite(endpoint_distance) ||
                    endpoint_distance + EGO_SWARM_EPSILON_M <
                        minimum_distance_m ||
                    !path_is_safe(other.position))
                {
                    return false;
                }
            }
            return true;
        };

        std::function<bool(
            const std::vector<int>&,
            const std::map<int, FormationTransitionObservation>&)> search;
        search = [&](
            const std::vector<int>& pending,
            const std::map<int, FormationTransitionObservation>& state)
        {
            if (pending.empty())
            {
                return true;
            }
            for (std::size_t index = 0U; index < pending.size(); ++index)
            {
                const int candidate_id = pending.at(index);
                const auto& candidate = state.at(candidate_id);
                bool is_safe = candidate_is_safe(candidate, state, 0.0);
                if (!is_safe && horizontal_offset_m > EGO_SWARM_EPSILON_M)
                {
                    is_safe = candidate_is_safe(candidate, state, 1.0);
                }
                if (!is_safe && horizontal_offset_m > EGO_SWARM_EPSILON_M)
                {
                    is_safe = candidate_is_safe(candidate, state, -1.0);
                }
                if (!is_safe)
                {
                    continue;
                }

                std::vector<int> next_pending = pending;
                next_pending.erase(next_pending.begin() +
                                   static_cast<std::ptrdiff_t>(index));
                auto next_state = state;
                next_state.at(candidate_id).position = candidate.target;
                out_order.push_back(candidate_id);
                if (search(next_pending, next_state))
                {
                    return true;
                }
                out_order.pop_back();
            }
            return false;
        };

        if (!search(remaining, by_id))
        {
            out_order.clear();
            return false;
        }
        return out_order.size() == ordered_member_ids.size();
    }

    static bool predecessors_ready(
        const std::vector<int>& ordered_member_ids,
        const int self_agent_id,
        const std::vector<FormationTransitionObservation>& observations,
        const double position_tolerance_m,
        const double velocity_tolerance_m)
    {
        if (ordered_member_ids.empty() || self_agent_id <= 0 ||
            !is_finite(position_tolerance_m) || position_tolerance_m < 0.0 ||
            !is_finite(velocity_tolerance_m) || velocity_tolerance_m < 0.0)
        {
            return false;
        }

        for (std::size_t index = 0U; index < ordered_member_ids.size(); ++index)
        {
            const int agent_id = ordered_member_ids.at(index);
            if (agent_id <= 0 ||
                (index > 0U && ordered_member_ids.at(index - 1U) >= agent_id))
            {
                return false;
            }
        }

        const auto self_iterator = std::find(
            ordered_member_ids.begin(), ordered_member_ids.end(), self_agent_id);
        if (self_iterator == ordered_member_ids.end())
        {
            return false;
        }

        for (const FormationTransitionObservation& observation : observations)
        {
            if (observation.agent_id <= 0 ||
                !is_finite_vector(observation.position) ||
                !is_finite_vector(observation.target) ||
                !is_finite_vector(observation.velocity))
            {
                return false;
            }
        }

        for (auto iterator = ordered_member_ids.begin();
             iterator != self_iterator; ++iterator)
        {
            const auto observation_iterator = std::find_if(
                observations.begin(), observations.end(),
                [iterator](const FormationTransitionObservation& observation)
                {
                    return observation.agent_id == *iterator;
                });
            if (observation_iterator == observations.end() ||
                !observation_iterator->has_fresh_trajectory ||
                distance_between_centers(
                    observation_iterator->position,
                    observation_iterator->target) > position_tolerance_m ||
                std::sqrt(squared_norm(observation_iterator->velocity)) >
                    velocity_tolerance_m)
            {
                return false;
            }
        }
        return true;
    }
};

}  // namespace ego_swarm
}  // namespace taiphoon
