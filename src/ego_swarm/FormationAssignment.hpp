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

struct FormationAssignmentResult
{
    bool is_valid{false};
    std::vector<FormationSlot> slots{};
    std::map<int, std::size_t> agent_to_slot{};
    double total_cost{0.0};
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
        out_result = FormationAssignmentResult{};
        if (agent_ids.empty() || agent_ids.size() != agent_positions_ned.size() ||
            !is_finite(slot_switch_penalty_m) || slot_switch_penalty_m < 0.0)
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

        std::vector<Vector3> slot_offsets;
        if (!FormationSlotPlanner::compute_slot_offsets(
                sorted_agents.size(), config, slot_offsets) ||
            slot_offsets.size() != sorted_agents.size())
        {
            return false;
        }

        const std::size_t count = sorted_agents.size();
        std::vector<std::vector<double>> costs(
            count, std::vector<double>(count, 0.0));
        for (std::size_t row = 0U; row < count; ++row)
        {
            for (std::size_t column = 0U; column < count; ++column)
            {
                const double distance = distance_between_centers(
                    sorted_agents.at(row).second, slot_offsets.at(column));
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
                costs.at(row).at(column) = cost;
            }
        }

        std::vector<std::size_t> row_to_column;
        if (!hungarian(costs, row_to_column))
        {
            return false;
        }

        FormationAssignmentResult result;
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
