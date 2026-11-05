/**
 * @file ProgressStallMonitor.hpp
 * @brief 依進度重新規劃時的停滯偵測（純 C++）
 * @author Taiphoon Agent Team
 * @date 2026-11-05
 *
 * Node 依軌跡進度（預設 30%）重新規劃時，若新軌跡前段幾乎不動，每次又從
 * 靜止開始，飛機會停在原地。本檔提供兩層判斷：
 *
 * 1. 候選軌跡的路徑進度：重新規劃得到的候選，在重新規劃比例的時間點沿自身
 *    路徑走過的比例不足時，Node 保留目前仍安全的軌跡，不替換。只看軌跡
 *    本身，不受量測雜訊影響，繞行也不會被誤判。
 * 2. 最佳距離監測：記錄離最終目標的最佳距離；時間窗內沒有再改善指定距離
 *    時判定停滯，Node 暫停依進度重新規劃，讓目前安全軌跡飛完。
 *
 * 本模組不管理 ROS2、active plan 生命週期或安全驗證；何時餵資料、何時清除
 * 由 Node 決定。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "ego_swarm/BsplineTrajectory.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// 最佳距離監測的時間窗（秒）。
constexpr double PROGRESS_STALL_WINDOW_S = 3.0;
/// 時間窗內最佳距離至少要改善的距離（公尺）。
constexpr double PROGRESS_STALL_MIN_IMPROVEMENT_M = 0.2;
/// 候選軌跡在重新規劃時間點至少要走過的路徑比例（無單位）。8×8 m 場域校正：
/// 停滯軌跡 0.10～0.14，直線、繞行與窄通道軌跡 0.27 以上（30% 時間點）。
constexpr double MIN_CANDIDATE_PATH_PROGRESS_RATIO = 0.20;
/// 計算路徑長度的取樣段數。
constexpr std::size_t PATH_PROGRESS_SAMPLE_COUNT = 100U;
/// 路徑總長低於此值（公尺）時視為原地軌跡，不判斷進度。
constexpr double PATH_PROGRESS_MIN_LENGTH_M = 0.05;

/**
 * @brief 計算軌跡在指定時間比例時，沿自身路徑走過的長度比例
 *
 * 以等時間取樣累加弧長：走過長度 ÷ 全長。路徑繞行時比例仍反映實際前進，
 * 前段停在原地或來回打轉的軌跡比例會偏低。
 *
 * @param trajectory 候選軌跡
 * @param time_ratio 時間比例，範圍 [0, 1]
 * @param out_ratio 輸出的路徑進度比例
 * @return 軌跡有效、時間比例合法且全長足夠時回傳 true
 */
inline bool trajectory_path_progress_ratio(
    const BsplineTrajectory& trajectory,
    const double time_ratio,
    double& out_ratio)
{
    out_ratio = 0.0;
    if (!trajectory.is_valid() || !std::isfinite(time_ratio) ||
        time_ratio < 0.0 || time_ratio > 1.0)
    {
        return false;
    }
    const double duration_s = trajectory.duration_s();
    if (!std::isfinite(duration_s) || duration_s <= 0.0)
    {
        return false;
    }
    const double split_time_s = duration_s * time_ratio;
    double total_length_m = 0.0;
    double split_length_m = 0.0;
    TrajectoryState previous = trajectory.evaluate_state(0.0);
    if (!previous.is_valid)
    {
        return false;
    }
    for (std::size_t index = 1U; index <= PATH_PROGRESS_SAMPLE_COUNT; ++index)
    {
        const double time_s = duration_s * static_cast<double>(index) /
            static_cast<double>(PATH_PROGRESS_SAMPLE_COUNT);
        const TrajectoryState state = trajectory.evaluate_state(time_s);
        if (!state.is_valid)
        {
            return false;
        }
        const double segment_m = std::sqrt(
            squared_norm(subtract_vectors(state.position, previous.position)));
        if (!std::isfinite(segment_m))
        {
            return false;
        }
        total_length_m += segment_m;
        if (time_s <= split_time_s)
        {
            split_length_m += segment_m;
        }
        previous = state;
    }
    if (total_length_m < PATH_PROGRESS_MIN_LENGTH_M)
    {
        return false;
    }
    out_ratio = split_length_m / total_length_m;
    return std::isfinite(out_ratio);
}

/**
 * @brief 判斷候選軌跡在重新規劃時間點的路徑進度是否不足
 *
 * 無法計算（例如原地軌跡）時回傳 false，不阻擋替換。
 *
 * @param trajectory 候選軌跡
 * @param replan_ratio Node 依進度重新規劃的時間比例
 * @return 進度不足時回傳 true
 */
inline bool candidate_lacks_path_progress(
    const BsplineTrajectory& trajectory,
    const double replan_ratio)
{
    double ratio = 0.0;
    if (!trajectory_path_progress_ratio(trajectory, replan_ratio, ratio))
    {
        return false;
    }
    return ratio < MIN_CANDIDATE_PATH_PROGRESS_RATIO;
}

/**
 * @brief 判斷停滯偵測是否適用時需要的 active plan 狀態
 */
struct ProgressStallContext
{
    /// 是否有 active plan。
    bool has_active_plan{false};
    /// active plan 是否仍在 duration 內、且 Final Target 未改變。
    bool active_plan_running{false};
    /// active plan 是否為 hold 類軌跡（terminal／waiting hold）。
    bool active_plan_is_hold{false};
    /// 是否處於緊急煞停。
    bool emergency_brake_active{false};
    /// 是否在規劃失敗後等待重試。
    bool waiting_for_replan{false};
    /// active plan 的終點是否就是最終目標。
    bool active_target_is_final{false};
};

/**
 * @brief 停滯偵測是否適用目前的 active plan
 *
 * 只適用「正在飛往最終目標的一般軌跡」。hold、等待重試、緊急煞停時
 * 飛機本來就刻意不動；segment 已結束、Final Target 改變或終點只是中途
 * Local Goal 時也不適用。
 *
 * @param context active plan 狀態
 * @return 適用時回傳 true
 */
inline bool progress_stall_applies(const ProgressStallContext& context)
{
    return context.has_active_plan && context.active_plan_running &&
        !context.active_plan_is_hold && !context.emergency_brake_active &&
        !context.waiting_for_replan && context.active_target_is_final;
}

/**
 * @brief 以「離最終目標的最佳距離」偵測停滯
 *
 * 最佳距離只會變小，對位置抖動較穩定：只有比上次記錄的最佳距離再縮短
 * `min_improvement_m` 才算進展並重新計時；時間窗內沒有進展即判定停滯。
 */
class ProgressStallMonitor
{
public:
    /**
     * @brief 建立監測器
     *
     * @param window_s 時間窗（秒）；非正或非有限值時使用預設值
     * @param min_improvement_m 最小改善距離（公尺）；非正或非有限值時使用預設值
     */
    explicit ProgressStallMonitor(
        const double window_s = PROGRESS_STALL_WINDOW_S,
        const double min_improvement_m = PROGRESS_STALL_MIN_IMPROVEMENT_M)
        : window_s_(std::isfinite(window_s) && window_s > 0.0
              ? window_s : PROGRESS_STALL_WINDOW_S),
          min_improvement_m_(
              std::isfinite(min_improvement_m) && min_improvement_m > 0.0
              ? min_improvement_m : PROGRESS_STALL_MIN_IMPROVEMENT_M)
    {
    }

    /**
     * @brief 清除所有狀態；下一次 update 重新開始計時
     */
    void reset()
    {
        has_reference_ = false;
        best_distance_m_ = 0.0;
        best_time_s_ = 0.0;
        last_time_s_ = 0.0;
        is_stalled_ = false;
    }

    /**
     * @brief 餵入目前離最終目標的距離
     *
     * 時間倒退或輸入非有限值時重新開始計時。
     *
     * @param now_s 目前時間（秒）
     * @param distance_to_target_m 目前離最終目標的距離（公尺）
     * @return 目前是否判定停滯
     */
    bool update(const double now_s, const double distance_to_target_m)
    {
        if (!std::isfinite(now_s) || !std::isfinite(distance_to_target_m) ||
            distance_to_target_m < 0.0)
        {
            reset();
            return false;
        }
        if (!has_reference_ || now_s < last_time_s_)
        {
            has_reference_ = true;
            best_distance_m_ = distance_to_target_m;
            best_time_s_ = now_s;
            last_time_s_ = now_s;
            is_stalled_ = false;
            return false;
        }
        last_time_s_ = now_s;
        if (distance_to_target_m <= best_distance_m_ - min_improvement_m_)
        {
            best_distance_m_ = distance_to_target_m;
            best_time_s_ = now_s;
            is_stalled_ = false;
            return false;
        }
        if (now_s - best_time_s_ >= window_s_)
        {
            is_stalled_ = true;
        }
        return is_stalled_;
    }

    /**
     * @brief 依 active plan 狀態更新或清除停滯偵測
     *
     * 不適用（見 progress_stall_applies）或 planner frame revision 改變時清除，
     * 下一次適用時重新計時。依進度重新規劃替換軌跡不會清除，才能跨越多次
     * 重新規劃累計停滯時間。
     *
     * @param context active plan 狀態
     * @param frame_revision 目前 planner frame revision
     * @param now_s 目前時間（秒）
     * @param distance_to_target_m 目前離最終目標的距離（公尺）
     * @return 目前是否判定停滯
     */
    bool track(
        const ProgressStallContext& context,
        const std::uint32_t frame_revision,
        const double now_s,
        const double distance_to_target_m)
    {
        if (!has_frame_revision_ || frame_revision != frame_revision_)
        {
            reset();
            has_frame_revision_ = true;
            frame_revision_ = frame_revision;
        }
        if (!progress_stall_applies(context))
        {
            reset();
            return false;
        }
        return update(now_s, distance_to_target_m);
    }

    /// 目前是否判定停滯。
    bool is_stalled() const
    {
        return is_stalled_;
    }

private:
    double window_s_{PROGRESS_STALL_WINDOW_S};
    double min_improvement_m_{PROGRESS_STALL_MIN_IMPROVEMENT_M};
    bool has_reference_{false};
    double best_distance_m_{0.0};
    double best_time_s_{0.0};
    double last_time_s_{0.0};
    bool is_stalled_{false};
    bool has_frame_revision_{false};
    std::uint32_t frame_revision_{0U};
};

}  // namespace ego_swarm
}  // namespace taiphoon
