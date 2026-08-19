/**
 * @file LocalGoalSelector.hpp
 * @brief EGO-Swarm Local Goal 純 C++ 計算 helper
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 *
 * 本檔案只負責依自身位置、Final Target、路徑高度約束與距離 horizon
 * 計算單一 planning segment 的局部終點，不管理 ROS2、Formation 或
 * trajectory lifecycle。
 */

#pragma once

#include <cmath>

#include "ego_swarm/PlannerTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief Local Goal 計算設定
 */
struct LocalGoalConfig
{
    /// 單一 planning segment 的最大空間長度，單位為公尺。
    double horizon_m{10.0};
};

/**
 * @brief Local Goal 計算結果
 */
struct LocalGoalResult
{
    bool is_valid{false};
    bool is_final_goal{false};
    Vector3 goal{0.0, 0.0, 0.0};
};

/**
 * @brief 依 Final Target 計算單一 Local Goal
 *
 * FIXED_Z 會先將 planning current 的 Z 投影到 Final Target Z，再計算
 * horizon；如此水平移動不會因原始三維高度差而縮短可規劃的水平距離。
 * LINEAR_Z 與 FREE_3D 則沿 current 到 Final Target 的完整向量做比例插值。
 *
 * @param current 目前位置，使用共同 planning frame，單位為公尺
 * @param final_target 任務真正終點，使用共同 planning frame，單位為公尺
 * @param path_constraint 路徑高度約束
 * @param config Local Goal 計算設定
 * @return Local Goal 計算結果；輸入非法時回傳無效結果
 */
inline LocalGoalResult compute_local_goal(
    const Vector3& current,
    const Vector3& final_target,
    const PathConstraint path_constraint,
    const LocalGoalConfig& config)
{
    LocalGoalResult result;
    if (!is_finite_vector(current) || !is_finite_vector(final_target) ||
        !std::isfinite(config.horizon_m) || config.horizon_m <= 0.0)
    {
        return result;
    }

    Vector3 planning_current = current;
    switch (path_constraint)
    {
        case PathConstraint::FREE_3D:
        case PathConstraint::LINEAR_Z:
            break;
        case PathConstraint::FIXED_Z:
            planning_current.at(2) = final_target.at(2);
            break;
        default:
            return result;
    }

    const Vector3 difference = subtract_vectors(final_target, planning_current);
    if (!is_finite_vector(difference))
    {
        return result;
    }
    const double distance = std::sqrt(squared_norm(difference));
    if (!std::isfinite(distance))
    {
        return result;
    }

    if (distance <= config.horizon_m)
    {
        result.goal = final_target;
        result.is_valid = true;
        result.is_final_goal = true;
        return result;
    }

    const double ratio = config.horizon_m / distance;
    if (!std::isfinite(ratio) || ratio <= 0.0 || ratio >= 1.0 ||
        !is_finite_vector(planning_current))
    {
        return result;
    }

    result.goal = add_vectors(
        planning_current, scale_vector(difference, ratio));
    result.is_valid = is_finite_vector(result.goal);
    return result;
}

}  // namespace ego_swarm
}  // namespace taiphoon
