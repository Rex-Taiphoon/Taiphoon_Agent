/**
 * @file PeerTrajectoryTable.hpp
 * @brief 同儕未來 B-spline 軌跡生命週期與 freshness 保存模組
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * 這個模組刻意不把資料年齡轉成額外安全半徑。軌跡只有在 receive time
 * 尚未逾時且目前時間仍位於軌跡有效區間時，才會交給 planner 使用。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "ego_swarm/BsplineTrajectory.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// 同儕未來軌跡的生命週期狀態。
enum class PeerTrajectoryStatus
{
    ACTIVE = 0,
    STALE,
    MISSING,
    INVALID
};

/// 同儕軌跡 freshness 與共同時間驗證設定。
struct PeerTrajectoryTableConfig
{
    double timeout_s{1.0};
    double max_peer_time_skew_s{0.5};
};

constexpr double PEER_TRAJECTORY_TIME_EPSILON_S = 1.0e-6;

/// 同儕軌跡及其共同時間與座標 frame metadata。
struct PeerTrajectory
{
    int agent_id{0};
    uint32_t trajectory_id{0U};
    uint32_t frame_revision{0U};
    double message_time_s{0.0};
    double start_time_s{0.0};
    double receive_time_s{0.0};
    double end_time_s{0.0};
    BsplineTrajectory trajectory{};
};

/**
 * @brief 以 Agent ID 索引的同儕軌跡表
 *
 * update 只接受完整有效、時間合理且 trajectory ID 單調不倒退的軌跡。
 * planner 必須使用 get_active()，不得使用未經 freshness 檢查的原始表內容。
 */
class PeerTrajectoryTable
{
public:
    /**
     * @brief 使用 freshness 設定建立同儕軌跡表
     *
     * @param config freshness 與共同時間偏差設定
     */
    explicit PeerTrajectoryTable(
        const PeerTrajectoryTableConfig& config = PeerTrajectoryTableConfig{})
    {
        if (!set_config(config))
        {
            config_ = PeerTrajectoryTableConfig{};
        }
    }

    /**
     * @brief 更新 freshness 與共同時間偏差設定
     *
     * @param config 新設定
     * @return true 表示設定有效並已套用
     */
    bool set_config(const PeerTrajectoryTableConfig& config)
    {
        if (!is_finite(config.timeout_s) || config.timeout_s <= 0.0 ||
            !is_finite(config.max_peer_time_skew_s) ||
            config.max_peer_time_skew_s < 0.0)
        {
            return false;
        }
        config_ = config;
        return true;
    }

    /**
     * @brief 取得目前 freshness 與時間設定
     *
     * @return 唯讀設定參考
     */
    const PeerTrajectoryTableConfig& config() const
    {
        return config_;
    }

    /**
     * @brief 儲存一份有效同儕軌跡
     *
     * @param peer_trajectory 待儲存的同儕軌跡
     * @return true 表示已接受並取代同 Agent 的舊軌跡
     */
    bool update(const PeerTrajectory& peer_trajectory)
    {
        if (!is_valid_record(peer_trajectory))
        {
            if (peer_trajectory.agent_id > 0)
            {
                last_update_status_[peer_trajectory.agent_id] =
                    PeerTrajectoryStatus::INVALID;
            }
            return false;
        }

        const auto latest_id = latest_trajectory_ids_.find(
            peer_trajectory.agent_id);
        if (latest_id != latest_trajectory_ids_.end() &&
            peer_trajectory.trajectory_id < latest_id->second)
        {
            last_update_status_[peer_trajectory.agent_id] =
                PeerTrajectoryStatus::INVALID;
            return false;
        }

        trajectories_[peer_trajectory.agent_id] = peer_trajectory;
        latest_trajectory_ids_[peer_trajectory.agent_id] =
            peer_trajectory.trajectory_id;
        last_update_status_.erase(peer_trajectory.agent_id);
        return true;
    }

    /**
     * @brief 取得指定 Agent 的最後有效軌跡
     *
     * @param agent_id 同儕 Agent ID
     * @param[out] out_trajectory 最後有效軌跡
     * @return true 表示表中存在有效資料
     */
    bool get_last_valid(
        const int agent_id,
        PeerTrajectory& out_trajectory) const
    {
        out_trajectory = PeerTrajectory{};
        const auto iterator = trajectories_.find(agent_id);
        if (iterator == trajectories_.end())
        {
            return false;
        }

        out_trajectory = iterator->second;
        return true;
    }

    /**
     * @brief 取得目前仍可供 planner 使用的同儕軌跡
     *
     * @param agent_id 同儕 Agent ID
     * @param now_s planner 目前共同時間
     * @param[out] out_trajectory 同儕軌跡
     * @return true 表示軌跡為 ACTIVE
     */
    bool get_fresh(
        const int agent_id,
        const double now_s,
        PeerTrajectory& out_trajectory) const
    {
        out_trajectory = PeerTrajectory{};
        if (get_status(agent_id, now_s) != PeerTrajectoryStatus::ACTIVE)
        {
            return false;
        }
        const auto iterator = trajectories_.find(agent_id);
        if (iterator == trajectories_.end())
        {
            return false;
        }
        out_trajectory = iterator->second;
        return true;
    }

    /**
     * @brief 取得全部最後有效同儕軌跡
     *
     * @param[out] out_trajectories 依 Agent ID 遞增的軌跡列表
     */
    void get_all(std::vector<PeerTrajectory>& out_trajectories) const
    {
        out_trajectories.clear();
        out_trajectories.reserve(trajectories_.size());
        for (const auto& entry : trajectories_)
        {
            out_trajectories.push_back(entry.second);
        }
    }

    /**
     * @brief 取得所有 ACTIVE 同儕軌跡
     *
     * @param now_s planner 目前共同時間
     * @param[out] out_trajectories 依 Agent ID 遞增的有效軌跡列表
     */
    void get_active(
        const double now_s,
        std::vector<PeerTrajectory>& out_trajectories) const
    {
        out_trajectories.clear();
        if (!is_finite(now_s))
        {
            return;
        }
        out_trajectories.reserve(trajectories_.size());
        for (const auto& entry : trajectories_)
        {
            if (get_status(entry.first, now_s) ==
                PeerTrajectoryStatus::ACTIVE)
            {
                out_trajectories.push_back(entry.second);
            }
        }
    }

    /**
     * @brief 查詢同儕軌跡生命週期狀態
     *
     * @param agent_id 同儕 Agent ID
     * @param now_s planner 目前共同時間
     * @return ACTIVE、STALE、MISSING 或 INVALID
     */
    PeerTrajectoryStatus get_status(
        const int agent_id,
        const double now_s) const
    {
        const auto iterator = trajectories_.find(agent_id);
        if (iterator == trajectories_.end())
        {
            const auto invalid_iterator = last_update_status_.find(agent_id);
            if (invalid_iterator != last_update_status_.end())
            {
                return invalid_iterator->second;
            }
            return PeerTrajectoryStatus::MISSING;
        }

        if (!is_finite(now_s) || !is_valid_record(iterator->second))
        {
            return PeerTrajectoryStatus::INVALID;
        }

        const PeerTrajectory& peer = iterator->second;
        if (now_s + config_.max_peer_time_skew_s +
                PEER_TRAJECTORY_TIME_EPSILON_S < peer.receive_time_s)
        {
            return PeerTrajectoryStatus::INVALID;
        }
        if (now_s - peer.receive_time_s > config_.timeout_s +
                PEER_TRAJECTORY_TIME_EPSILON_S ||
            now_s > peer.end_time_s + PEER_TRAJECTORY_TIME_EPSILON_S)
        {
            return PeerTrajectoryStatus::STALE;
        }
        return PeerTrajectoryStatus::ACTIVE;
    }

    /**
     * @brief 檢查指定同儕是否仍具 freshness
     *
     * @param agent_id 同儕 Agent ID
     * @param now_s planner 目前共同時間
     * @return true 表示目前狀態為 ACTIVE
     */
    bool is_fresh(const int agent_id, const double now_s) const
    {
        return get_status(agent_id, now_s) == PeerTrajectoryStatus::ACTIVE;
    }

    /**
     * @brief 檢查是否至少有一筆 ACTIVE 同儕軌跡
     *
     * @param now_s planner 目前共同時間
     * @return true 表示至少有一筆可供 planner 使用的軌跡
     */
    bool has_fresh(const double now_s) const
    {
        for (const auto& entry : trajectories_)
        {
            if (is_fresh(entry.first, now_s))
            {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief 清除已過期或無效的同儕軌跡
     *
     * @param now_s planner 目前共同時間
     * @return 清除的 record 數量
     */
    std::size_t prune(const double now_s)
    {
        std::size_t removed_count = 0U;
        for (auto iterator = trajectories_.begin();
             iterator != trajectories_.end();)
        {
            if (get_status(iterator->first, now_s) !=
                PeerTrajectoryStatus::ACTIVE)
            {
                last_update_status_.erase(iterator->first);
                iterator = trajectories_.erase(iterator);
                ++removed_count;
            }
            else
            {
                ++iterator;
            }
        }
        return removed_count;
    }

    /**
     * @brief 移除指定同儕的軌跡
     *
     * @param agent_id 同儕 Agent ID
     * @return true 表示原本存在並已移除
     */
    bool erase(const int agent_id)
    {
        const bool existed = trajectories_.erase(agent_id) > 0U;
        // erase 代表 peer lifecycle 結束；peer 重新出現時允許從新的
        // trajectory ID 序列開始。單純 timeout/prune 則保留最高 ID，
        // 以拒絕同一 lifecycle 的 replay。
        latest_trajectory_ids_.erase(agent_id);
        last_update_status_.erase(agent_id);
        return existed;
    }

    /**
     * @brief 檢查指定 Agent 是否存在最後有效軌跡
     *
     * @param agent_id 同儕 Agent ID
     * @return true 表示存在可用軌跡
     */
    bool contains(const int agent_id) const
    {
        return trajectories_.find(agent_id) != trajectories_.end();
    }

    /**
     * @brief 取得表中軌跡數量
     *
     * @return 最後有效軌跡數量
     */
    std::size_t size() const
    {
        return trajectories_.size();
    }

    /**
     * @brief 清除所有保存的最後有效軌跡
     */
    void clear()
    {
        trajectories_.clear();
        latest_trajectory_ids_.clear();
        last_update_status_.clear();
    }

private:
    bool is_valid_record(const PeerTrajectory& peer_trajectory) const
    {
        if (peer_trajectory.agent_id <= 0 ||
            peer_trajectory.frame_revision == 0U ||
            !peer_trajectory.trajectory.is_valid() ||
            !is_finite(peer_trajectory.message_time_s) ||
            !is_finite(peer_trajectory.start_time_s) ||
            !is_finite(peer_trajectory.receive_time_s) ||
            !is_finite(peer_trajectory.end_time_s))
        {
            return false;
        }

        const double trajectory_duration_s =
            peer_trajectory.trajectory.duration_s();
        if (!is_finite(trajectory_duration_s) || trajectory_duration_s <= 0.0 ||
            peer_trajectory.end_time_s <= peer_trajectory.start_time_s)
        {
            return false;
        }

        const double duration_error = peer_trajectory.end_time_s -
            peer_trajectory.start_time_s - trajectory_duration_s;
        if (!is_finite(duration_error) ||
            std::abs(duration_error) > PEER_TRAJECTORY_TIME_EPSILON_S)
        {
            return false;
        }

        const double max_skew_s = config_.max_peer_time_skew_s;
        if (!is_finite(max_skew_s) || max_skew_s < 0.0 ||
            std::abs(peer_trajectory.message_time_s -
                     peer_trajectory.receive_time_s) >
                max_skew_s + PEER_TRAJECTORY_TIME_EPSILON_S ||
            peer_trajectory.start_time_s > peer_trajectory.receive_time_s +
                max_skew_s + PEER_TRAJECTORY_TIME_EPSILON_S ||
            peer_trajectory.end_time_s < peer_trajectory.receive_time_s -
                max_skew_s - PEER_TRAJECTORY_TIME_EPSILON_S)
        {
            return false;
        }

        return true;
    }

    PeerTrajectoryTableConfig config_{};
    std::map<int, PeerTrajectory> trajectories_{};
    std::map<int, uint32_t> latest_trajectory_ids_{};
    std::map<int, PeerTrajectoryStatus> last_update_status_{};
};

}  // namespace ego_swarm
}  // namespace taiphoon
