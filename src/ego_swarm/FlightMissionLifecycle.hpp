/**
 * @file FlightMissionLifecycle.hpp
 * @brief 依飛控觀測限制任務生命週期，避免降落後恢復舊編隊
 * @author Taiphoon Agent Team
 * @date 2026-10-06
 */
#pragma once

namespace taiphoon
{
namespace ego_swarm
{

class FlightMissionLifecycle
{
public:
    /**
     * @brief 更新觀測；有效地面狀態的首次進入要求清除舊任務。
     * @param is_valid PX4 狀態是否有效
     * @param is_armed 是否已解鎖
     * @param is_landed 是否已落地
     * @return 是否應清除上一趟飛行的任務
     */
    bool observe(bool is_valid, bool is_armed, bool is_landed)
    {
        can_plan_ = is_valid && is_armed && !is_landed;
        if (!is_valid)
        {
            return false;
        }
        const bool is_grounded = !is_armed || is_landed;
        const bool should_reset = is_grounded && !is_grounded_;
        is_grounded_ = is_grounded;
        if (is_grounded)
        {
            has_new_formation_command_ = false;
        }
        return should_reset;
    }

    /** @brief 判斷是否允許任務規劃。 @return 有效且已離地、解鎖時為 true。 */
    bool can_plan() const
    {
        return can_plan_;
    }

    /** @brief 禁止舊 heartbeat 恢復編隊。 @return 是否收到本次飛行的新編隊命令。 */
    bool can_adopt_formation() const
    {
        return can_plan_ && has_new_formation_command_;
    }

    /** @brief 記錄已完成參數驗證的新編隊命令。 */
    void accept_formation_command()
    {
        has_new_formation_command_ = can_plan_;
    }

private:
    bool can_plan_{false};
    bool is_grounded_{false};
    bool has_new_formation_command_{false};
};

}
}
