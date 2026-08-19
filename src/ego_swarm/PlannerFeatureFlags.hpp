/**
 * @file PlannerFeatureFlags.hpp
 * @brief EGO-Swarm planner 功能旗標與 segmented planning 參數
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 *
 * 本檔案只保存 planner 功能開關與其數值設定，不依賴 ROS2，也不直接
 * 改變 planner、reservation 或 trajectory lifecycle 的執行行為。
 */

#pragma once

#include <cmath>

namespace taiphoon
{
namespace ego_swarm
{

constexpr double DEFAULT_LOCAL_GOAL_HORIZON_M = 10.0;
constexpr double DEFAULT_LOCAL_GOAL_REPLAN_RATIO = 0.70;
constexpr double DEFAULT_MINIMUM_REPLAN_INTERVAL_S = 0.5;

/**
 * @brief planner 新功能的設定邊界
 *
 * 兩個功能旗標預設關閉，確保尚未接入新 planning pipeline 前仍維持
 * 舊架構行為。數值參數先在此 seam 完成有限值與範圍 normalization。
 */
struct PlannerFeatureFlags
{
    /// 是否啟用 Local Goal segmented planning。
    bool enable_local_goal_planning{false};
    /// 是否啟用所有 moving trajectory 共用的 reservation。
    bool enable_generic_trajectory_reservation{false};
    /// 單一 Local Goal segment 的距離 horizon，單位為公尺。
    double local_goal_horizon_m{DEFAULT_LOCAL_GOAL_HORIZON_M};
    /// 允許依 segment progress 觸發下一段規劃的比例。
    double local_goal_replan_ratio{DEFAULT_LOCAL_GOAL_REPLAN_RATIO};
    /// 正常 progress-based replan 之間的最短間隔，單位為秒。
    double minimum_replan_interval_s{DEFAULT_MINIMUM_REPLAN_INTERVAL_S};
};

/**
 * @brief 將 ROS2 parameter 讀值 normalization 為可安全使用的設定
 *
 * bool 旗標不需要 normalization；任一非法數值只回復該欄位的規格預設值，
 * 不影響其他合法覆寫或功能旗標。
 *
 * @param requested ROS2 parameter 讀取後的設定
 * @return 每個數值欄位皆有限且符合範圍的設定
 */
inline PlannerFeatureFlags normalize_planner_feature_flags(
    const PlannerFeatureFlags& requested)
{
    PlannerFeatureFlags normalized = requested;
    if (!std::isfinite(normalized.local_goal_horizon_m) ||
        normalized.local_goal_horizon_m <= 0.0)
    {
        normalized.local_goal_horizon_m = DEFAULT_LOCAL_GOAL_HORIZON_M;
    }

    if (!std::isfinite(normalized.local_goal_replan_ratio) ||
        normalized.local_goal_replan_ratio <= 0.0 ||
        normalized.local_goal_replan_ratio > 1.0)
    {
        normalized.local_goal_replan_ratio =
            DEFAULT_LOCAL_GOAL_REPLAN_RATIO;
    }

    if (!std::isfinite(normalized.minimum_replan_interval_s) ||
        normalized.minimum_replan_interval_s < 0.0)
    {
        normalized.minimum_replan_interval_s =
            DEFAULT_MINIMUM_REPLAN_INTERVAL_S;
    }

    return normalized;
}

}  // namespace ego_swarm
}  // namespace taiphoon
