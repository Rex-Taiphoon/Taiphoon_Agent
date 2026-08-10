/**
 * @file PeerTrajectoryTable.hpp
 * @brief 同儕最後有效 B-spline 軌跡保存模組
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * 這個模組刻意不把資料年齡轉成額外安全半徑。短暫缺少新資料時，
 * 呼叫端取得的仍是最後一份完整有效軌跡，距離門檻保持原設定值。
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <map>
#include <vector>

#include "ego_swarm/BsplineTrajectory.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// 同儕軌跡及其相對開始時間。
struct PeerTrajectory
{
    int agent_id{0};
    double start_time_s{0.0};
    BsplineTrajectory trajectory{};
};

/**
 * @brief 以 Agent ID 索引的最後有效同儕軌跡表
 *
 * update 只接受完整有效的軌跡。沒有新資料或收到無效資料時，既有
 * record 不會被刪除或修改，因此短暫缺資料可原樣沿用最後有效軌跡。
 */
class PeerTrajectoryTable
{
public:
    /**
     * @brief 儲存一份有效同儕軌跡
     *
     * @param peer_trajectory 待儲存的同儕軌跡
     * @return true 表示已接受並取代同 Agent 的舊軌跡
     */
    bool update(const PeerTrajectory& peer_trajectory)
    {
        if (!is_finite(peer_trajectory.start_time_s) ||
            !peer_trajectory.trajectory.is_valid())
        {
            return false;
        }

        trajectories_[peer_trajectory.agent_id] = peer_trajectory;
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
    }

private:
    std::map<int, PeerTrajectory> trajectories_{};
};

}  // namespace ego_swarm
}  // namespace taiphoon
