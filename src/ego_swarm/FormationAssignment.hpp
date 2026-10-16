/**
 * @file FormationAssignment.hpp
 * @brief 編隊槽位 minimum-cost assignment 純 C++ 模組
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "ego_swarm/FormationSlotPlanner.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// 圓形相位搜尋解析度（rad，0.5 度）；n 台時只需搜尋一個 2π/n 週期。
constexpr double FORMATION_CIRCLE_ROTATION_STEP_RAD = EGO_SWARM_PI / 360.0;

struct FormationAssignmentResult
{
    bool is_valid{false};
    std::vector<FormationSlot> slots{};
    std::map<int, std::size_t> agent_to_slot{};
    double total_cost{0.0};
    /// 採用的圓形槽位相位；非圓形固定為 0，呼叫端需寫回 FormationConfig。
    double rotation_rad{0.0};
};

/**
 * @brief 依 canonical NED snapshot 對 Agent 與幾何槽位做一對一分配。
 *
 * 輸入 Agent 會先依 ID 排序，Hungarian row/column 迭代也固定採升冪，
 * 因此同成本時結果 deterministic。任何無效輸入都整體失敗，不會部分
 * 修改 assignment。
 */
class FormationAssignment
{
public:
    static bool assign(
        const std::vector<int>& agent_ids,
        const std::vector<Vector3>& agent_positions_ned,
        const FormationConfig& config,
        const std::map<int, std::size_t>& previous_assignment,
        const double slot_switch_penalty_m,
        FormationAssignmentResult& out_result)
    {
        return assign(
            agent_ids, agent_positions_ned, config, previous_assignment,
            slot_switch_penalty_m, 10.0, out_result);
    }

    /**
     * @brief 以 slot-switch 與水平 crossing penalty 建立 assignment。
     *
     * 基礎條件是總移動距離最短：Hungarian 先求 distance/switch 的基準解，
     * 再以 deterministic pairwise swap refinement 降低交叉成本，避免把
     * pairwise crossing state 塞進 Hungarian 的淺層輸入矩陣。圓形的相位不影響
     * 隊形，因此另外以固定解析度搜尋總成本最低的相位；同成本取較小相位。
     * 換槽成本只在 parent 與目前槽位數相同時才有意義，數量不同時不加計。
     *
     * @param agent_ids Agent ID
     * @param agent_positions_ned canonical NED 位置
     * @param config formation 幾何
     * @param previous_assignment parent epoch assignment
     * @param slot_switch_penalty_m 換槽位成本
     * @param crossing_penalty_m 水平路徑 crossing 成本
     * @param[out] out_result assignment 結果
     * @return true 表示建立完整有效 assignment
     */
    static bool assign(
        const std::vector<int>& agent_ids,
        const std::vector<Vector3>& agent_positions_ned,
        const FormationConfig& config,
        const std::map<int, std::size_t>& previous_assignment,
        const double slot_switch_penalty_m,
        const double crossing_penalty_m,
        FormationAssignmentResult& out_result)
    {
        out_result = FormationAssignmentResult{};
        if (agent_ids.empty() || agent_ids.size() != agent_positions_ned.size() ||
            !is_finite(slot_switch_penalty_m) || slot_switch_penalty_m < 0.0 ||
            !is_finite(crossing_penalty_m) || crossing_penalty_m < 0.0)
        {
            return false;
        }

        std::vector<std::pair<int, Vector3>> sorted_agents;
        sorted_agents.reserve(agent_ids.size());
        for (std::size_t index = 0U; index < agent_ids.size(); ++index)
        {
            if (agent_ids.at(index) <= 0 ||
                !is_finite_vector(agent_positions_ned.at(index)))
            {
                return false;
            }
            sorted_agents.emplace_back(
                agent_ids.at(index), agent_positions_ned.at(index));
        }
        std::sort(
            sorted_agents.begin(), sorted_agents.end(),
            [](const auto& first, const auto& second)
            {
                return first.first < second.first;
            });
        for (std::size_t index = 1U; index < sorted_agents.size(); ++index)
        {
            if (sorted_agents.at(index - 1U).first == sorted_agents.at(index).first)
            {
                return false;
            }
        }

        const std::size_t count = sorted_agents.size();
        const double effective_switch_penalty_m =
            previous_assignment.size() == count ? slot_switch_penalty_m : 0.0;
        std::size_t rotation_steps = 1U;
        double rotation_step_rad = 0.0;
        if (config.type == FormationType::CIRCLE)
        {
            const double period_rad = 2.0 * EGO_SWARM_PI / static_cast<double>(count);
            rotation_steps = static_cast<std::size_t>(
                std::lround(period_rad / FORMATION_CIRCLE_ROTATION_STEP_RAD));
            rotation_steps = std::max<std::size_t>(rotation_steps, 1U);
            rotation_step_rad = period_rad / static_cast<double>(rotation_steps);
        }

        bool has_candidate = false;
        double best_objective = std::numeric_limits<double>::infinity();
        FormationConfig best_config = config;
        std::vector<Vector3> slot_offsets;
        std::vector<std::vector<double>> costs;
        std::vector<std::size_t> row_to_column;
        for (std::size_t step = 0U; step < rotation_steps; ++step)
        {
            FormationConfig candidate_config = config;
            candidate_config.rotation_rad = config.type == FormationType::CIRCLE
                ? rotation_step_rad * static_cast<double>(step) : 0.0;
            std::vector<Vector3> candidate_offsets;
            std::vector<std::vector<double>> candidate_costs;
            std::vector<std::size_t> candidate_columns;
            if (!solve_for_config(
                    sorted_agents, candidate_config, previous_assignment,
                    effective_switch_penalty_m, crossing_penalty_m,
                    candidate_offsets, candidate_costs, candidate_columns))
            {
                return false;
            }
            const double objective = assignment_objective(
                candidate_costs, candidate_columns, crossing_penalty_m,
                sorted_agents, candidate_offsets);
            if (!is_finite(objective))
            {
                return false;
            }
            // 輸入來自同一份 epoch snapshot，各 Agent 計算結果逐位元相同；
            // 嚴格小於才更新，使同成本時固定採用較小相位。
            if (!has_candidate || objective < best_objective)
            {
                has_candidate = true;
                best_objective = objective;
                best_config = candidate_config;
                slot_offsets = candidate_offsets;
                costs = candidate_costs;
                row_to_column = candidate_columns;
            }
        }
        if (!has_candidate)
        {
            return false;
        }

        FormationAssignmentResult result;
        result.rotation_rad = best_config.rotation_rad;
        result.slots.reserve(count);
        for (std::size_t row = 0U; row < count; ++row)
        {
            const std::size_t column = row_to_column.at(row);
            FormationSlot slot;
            slot.agent_id = sorted_agents.at(row).first;
            slot.rank = column;
            slot.offset_ned = slot_offsets.at(column);
            slot.is_valid = true;
            result.slots.push_back(slot);
            result.agent_to_slot.emplace(slot.agent_id, column);
            result.total_cost += costs.at(row).at(column);
        }
        result.is_valid = result.agent_to_slot.size() == count &&
            is_finite(result.total_cost);
        if (!result.is_valid)
        {
            out_result = FormationAssignmentResult{};
            return false;
        }
        out_result = result;
        return true;
    }

private:
    /**
     * @brief 在指定幾何下求 Hungarian 解並降低交叉
     *
     * @param sorted_agents 依 ID 排序的 Agent 與 canonical 位置
     * @param config 本次候選幾何
     * @param previous_assignment parent epoch assignment
     * @param slot_switch_penalty_m 換槽位成本
     * @param crossing_penalty_m 水平路徑 crossing 成本
     * @param[out] out_offsets 槽位偏移
     * @param[out] out_costs 成本矩陣
     * @param[out] out_row_to_column 每個 Agent 的槽位
     * @return true 表示取得完整解
     */
    static bool solve_for_config(
        const std::vector<std::pair<int, Vector3>>& sorted_agents,
        const FormationConfig& config,
        const std::map<int, std::size_t>& previous_assignment,
        const double slot_switch_penalty_m,
        const double crossing_penalty_m,
        std::vector<Vector3>& out_offsets,
        std::vector<std::vector<double>>& out_costs,
        std::vector<std::size_t>& out_row_to_column)
    {
        const std::size_t count = sorted_agents.size();
        if (!FormationSlotPlanner::compute_slot_offsets(count, config, out_offsets) ||
            out_offsets.size() != count)
        {
            return false;
        }
        out_costs.assign(count, std::vector<double>(count, 0.0));
        for (std::size_t row = 0U; row < count; ++row)
        {
            for (std::size_t column = 0U; column < count; ++column)
            {
                const double distance = distance_between_centers(
                    sorted_agents.at(row).second, out_offsets.at(column));
                if (!is_finite(distance))
                {
                    return false;
                }
                double cost = distance;
                const auto previous = previous_assignment.find(
                    sorted_agents.at(row).first);
                if (previous != previous_assignment.end() &&
                    previous->second != column)
                {
                    cost += slot_switch_penalty_m;
                }
                if (!is_finite(cost))
                {
                    return false;
                }
                out_costs.at(row).at(column) = cost;
            }
        }
        if (!hungarian(out_costs, out_row_to_column))
        {
            return false;
        }
        refine_crossings(
            sorted_agents, out_offsets, out_costs, crossing_penalty_m,
            out_row_to_column);
        return true;
    }

    static double orientation_2d(
        const Vector3& first,
        const Vector3& second,
        const Vector3& third)
    {
        return (second.at(0) - first.at(0)) *
                (third.at(1) - first.at(1)) -
            (second.at(1) - first.at(1)) *
                (third.at(0) - first.at(0));
    }

    static bool segments_cross_2d(
        const Vector3& first_start,
        const Vector3& first_end,
        const Vector3& second_start,
        const Vector3& second_end)
    {
        const double first_side = orientation_2d(
            first_start, first_end, second_start);
        const double second_side = orientation_2d(
            first_start, first_end, second_end);
        const double third_side = orientation_2d(
            second_start, second_end, first_start);
        const double fourth_side = orientation_2d(
            second_start, second_end, first_end);
        return is_finite(first_side) && is_finite(second_side) &&
            is_finite(third_side) && is_finite(fourth_side) &&
            ((first_side > EGO_SWARM_EPSILON_M &&
              second_side < -EGO_SWARM_EPSILON_M) ||
             (first_side < -EGO_SWARM_EPSILON_M &&
              second_side > EGO_SWARM_EPSILON_M)) &&
            ((third_side > EGO_SWARM_EPSILON_M &&
              fourth_side < -EGO_SWARM_EPSILON_M) ||
             (third_side < -EGO_SWARM_EPSILON_M &&
              fourth_side > EGO_SWARM_EPSILON_M));
    }

    static std::size_t crossing_count(
        const std::vector<std::pair<int, Vector3>>& sorted_agents,
        const std::vector<Vector3>& slot_offsets,
        const std::vector<std::size_t>& row_to_column)
    {
        std::size_t count = 0U;
        for (std::size_t first = 0U;
             first < sorted_agents.size(); ++first)
        {
            for (std::size_t second = first + 1U;
                 second < sorted_agents.size(); ++second)
            {
                if (segments_cross_2d(
                        sorted_agents.at(first).second,
                        slot_offsets.at(row_to_column.at(first)),
                        sorted_agents.at(second).second,
                        slot_offsets.at(row_to_column.at(second))))
                {
                    ++count;
                }
            }
        }
        return count;
    }

    static double assignment_objective(
        const std::vector<std::vector<double>>& costs,
        const std::vector<std::size_t>& row_to_column,
        const double crossing_penalty_m,
        const std::vector<std::pair<int, Vector3>>& sorted_agents,
        const std::vector<Vector3>& slot_offsets)
    {
        double total = 0.0;
        for (std::size_t row = 0U; row < row_to_column.size(); ++row)
        {
            total += costs.at(row).at(row_to_column.at(row));
        }
        total += static_cast<double>(crossing_count(
            sorted_agents, slot_offsets, row_to_column)) * crossing_penalty_m;
        return total;
    }

    static void refine_crossings(
        const std::vector<std::pair<int, Vector3>>& sorted_agents,
        const std::vector<Vector3>& slot_offsets,
        const std::vector<std::vector<double>>& costs,
        const double crossing_penalty_m,
        std::vector<std::size_t>& row_to_column)
    {
        if (row_to_column.size() < 2U)
        {
            return;
        }
        double current_objective = assignment_objective(
            costs, row_to_column, crossing_penalty_m,
            sorted_agents, slot_offsets);
        const std::size_t max_rounds = row_to_column.size() *
            row_to_column.size();
        for (std::size_t round = 0U; round < max_rounds; ++round)
        {
            bool changed = false;
            std::vector<std::size_t> best = row_to_column;
            double best_objective = current_objective;
            for (std::size_t first = 0U;
                 first < row_to_column.size(); ++first)
            {
                for (std::size_t second = first + 1U;
                     second < row_to_column.size(); ++second)
                {
                    std::vector<std::size_t> candidate = row_to_column;
                    std::swap(
                        candidate.at(first), candidate.at(second));
                    const double objective = assignment_objective(
                        costs, candidate, crossing_penalty_m,
                        sorted_agents, slot_offsets);
                    if (objective < best_objective - EGO_SWARM_EPSILON_M)
                    {
                        best = candidate;
                        best_objective = objective;
                        changed = true;
                    }
                }
            }
            if (!changed)
            {
                return;
            }
            row_to_column = best;
            current_objective = best_objective;
        }
    }

    static bool hungarian(
        const std::vector<std::vector<double>>& costs,
        std::vector<std::size_t>& row_to_column)
    {
        row_to_column.clear();
        const std::size_t count = costs.size();
        if (count == 0U)
        {
            return false;
        }
        for (const auto& row : costs)
        {
            if (row.size() != count)
            {
                return false;
            }
            for (const double cost : row)
            {
                if (!is_finite(cost))
                {
                    return false;
                }
            }
        }

        const double infinity = std::numeric_limits<double>::infinity();
        std::vector<double> u(count + 1U, 0.0);
        std::vector<double> v(count + 1U, 0.0);
        std::vector<std::size_t> p(count + 1U, 0U);
        std::vector<std::size_t> way(count + 1U, 0U);
        constexpr double TIE_EPSILON = 1.0e-12;

        for (std::size_t row = 1U; row <= count; ++row)
        {
            p.at(0) = row;
            std::size_t column_zero = 0U;
            std::vector<double> minimum(count + 1U, infinity);
            std::vector<bool> used(count + 1U, false);
            do
            {
                used.at(column_zero) = true;
                const std::size_t row_zero = p.at(column_zero);
                double delta = infinity;
                std::size_t column_one = 0U;
                for (std::size_t column = 1U; column <= count; ++column)
                {
                    if (used.at(column))
                    {
                        continue;
                    }
                    const double current = costs.at(row_zero - 1U).at(column - 1U) -
                        u.at(row_zero) - v.at(column);
                    if (current < minimum.at(column) - TIE_EPSILON ||
                        (std::abs(current - minimum.at(column)) <= TIE_EPSILON &&
                         column < column_one))
                    {
                        minimum.at(column) = current;
                        way.at(column) = column_zero;
                    }
                    if (minimum.at(column) < delta - TIE_EPSILON ||
                        (std::abs(minimum.at(column) - delta) <= TIE_EPSILON &&
                         (column_one == 0U || column < column_one)))
                    {
                        delta = minimum.at(column);
                        column_one = column;
                    }
                }
                if (column_one == 0U || !is_finite(delta))
                {
                    return false;
                }
                for (std::size_t column = 0U; column <= count; ++column)
                {
                    if (used.at(column))
                    {
                        u.at(p.at(column)) += delta;
                        v.at(column) -= delta;
                    }
                    else
                    {
                        minimum.at(column) -= delta;
                    }
                }
                column_zero = column_one;
            } while (p.at(column_zero) != 0U);

            do
            {
                const std::size_t previous_column = way.at(column_zero);
                p.at(column_zero) = p.at(previous_column);
                column_zero = previous_column;
            } while (column_zero != 0U);
        }

        row_to_column.assign(count, count);
        for (std::size_t column = 1U; column <= count; ++column)
        {
            if (p.at(column) == 0U || p.at(column) > count)
            {
                return false;
            }
            row_to_column.at(p.at(column) - 1U) = column - 1U;
        }
        return std::all_of(
            row_to_column.begin(), row_to_column.end(),
            [count](const std::size_t value) { return value < count; });
    }
};

}  // namespace ego_swarm
}  // namespace taiphoon
