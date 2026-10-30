/**
 * @file FormationEpochPolicy.hpp
 * @brief FormationEpoch 成員重組、提案逾時與重新加入的純 C++ 策略
 * @author Taiphoon Agent Team
 * @date 2026-10-15
 *
 * 這些規則用來避免 Agent 關閉或重新啟動時，存活成員因無法完成的
 * proposal 永久停在 hold。啟用 epoch 仍需全體成員 ADOPTED，本檔只決定
 * 何時接受、丟棄或略過候選 epoch。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace taiphoon
{
namespace ego_swarm
{

/// 未全員 ADOPTED 的候選 epoch 最長等待時間（秒）；需涵蓋兩次 peer scan。
constexpr double FORMATION_EPOCH_PROPOSAL_TIMEOUT_S = 15.0;

/// 非既有成員送出 FormationEpoch 後，仍視為「已準備加入」的時間（秒）。
constexpr double FORMATION_JOIN_READY_TIMEOUT_S = 3.0;

/// 記住已逾時候選 epoch 的數量上限，避免 heartbeat 把它重新帶回。
constexpr std::size_t FORMATION_EXPIRED_EPOCH_HISTORY_SIZE = 16U;

/// 編隊最少成員數；只剩一台時解散編隊並原地懸停，不飛往單機虛擬槽位。
constexpr std::size_t FORMATION_MIN_MEMBER_COUNT = 2U;

/**
 * @brief 判斷候選 epoch 的 lineage 是否可被考慮。
 *
 * Agent 若在 epoch 切換途中關閉，存活成員可能各自停在不同的 accepted
 * epoch。FormationEpoch 已攜帶完整 snapshot，且啟用仍需全員 ADOPTED，
 * 因此 sequence 較新的 epoch 即使 parent 不同也可接受，避免彼此永久拒絕。
 *
 * @param has_accepted 本機是否已有 accepted epoch
 * @param accepted_sequence 本機 accepted epoch sequence
 * @param parent_matches 候選 epoch 的 parent 是否等於本機 accepted epoch
 * @param epoch_sequence 候選 epoch sequence
 * @return true 表示可進入 priority 比較
 */
inline bool is_epoch_lineage_acceptable(
    const bool has_accepted,
    const uint32_t accepted_sequence,
    const bool parent_matches,
    const uint32_t epoch_sequence)
{
    return !has_accepted || parent_matches || epoch_sequence > accepted_sequence;
}

/**
 * @brief 判斷 epoch 是否包含已穩定確認離線的成員。
 *
 * 包含離線成員的 proposal 永遠收不齊 ADOPTED；若仍被採用，存活成員會
 * 透過 heartbeat 互相把它傳回來。成員檢視尚未穩定時不判定離線。
 *
 * @param epoch_members epoch 成員 ID（已排序）
 * @param alive_members 本機穩定檢視中的存活成員 ID（已排序）
 * @param self_id 本機 Agent ID
 * @param is_view_stable 存活成員檢視是否已連續穩定
 * @return true 表示 epoch 含有已確認離線的成員
 */
inline bool has_departed_member(
    const std::vector<int>& epoch_members,
    const std::vector<int>& alive_members,
    const int self_id,
    const bool is_view_stable)
{
    if (!is_view_stable)
    {
        return false;
    }
    for (const int member_id : epoch_members)
    {
        if (member_id != self_id &&
            !std::binary_search(
                alive_members.begin(), alive_members.end(), member_id))
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief 判斷成員變更是否包含既有成員離開。
 *
 * 只有離開需要 hold 等待重組；新成員加入時既有槽位仍然有效，不能
 * 因新成員尚未 ADOPTED 而凍結整個編隊。
 *
 * @param accepted_members accepted epoch 成員 ID（已排序）
 * @param candidate_members 目前存活成員 ID（已排序）
 * @return true 表示至少一個 accepted 成員不在存活集合
 */
inline bool is_membership_departure(
    const std::vector<int>& accepted_members,
    const std::vector<int>& candidate_members)
{
    for (const int member_id : accepted_members)
    {
        if (!std::binary_search(
                candidate_members.begin(), candidate_members.end(), member_id))
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief 判斷 peer 是否可列入編隊重組成員。
 *
 * 既有成員依狀態新鮮度判斷即可；非既有成員必須近期送出 FormationEpoch，
 * 代表它已收到本趟飛行的編隊命令並能 ADOPTED。重新啟動但尚未收到
 * 編隊命令的 Agent 不會被納入，避免存活成員等待一個不會回覆的成員。
 *
 * @param is_accepted_member 是否為目前 accepted epoch 成員
 * @param has_ready_signal 是否曾收到該 peer 的 FormationEpoch
 * @param ready_signal_age_s 最近一次 FormationEpoch 的經過時間（秒）
 * @return true 表示可列入候選成員
 */
inline bool can_include_formation_peer(
    const bool is_accepted_member,
    const bool has_ready_signal,
    const double ready_signal_age_s)
{
    if (is_accepted_member)
    {
        return true;
    }
    return has_ready_signal && std::isfinite(ready_signal_age_s) &&
        ready_signal_age_s >= 0.0 &&
        ready_signal_age_s <= FORMATION_JOIN_READY_TIMEOUT_S;
}

/**
 * @brief 判斷新 proposal 是否帶入 parent epoch 的槽位歸屬。
 *
 * 換槽成本假設相同槽位編號代表相同幾何角色。隊形類型改變時編號語意
 * 不同（一字是「由西到東第 n 個」，燕行是「機頭／左翼近位…」），沿用舊
 * 歸屬只會讓指派偏向保留無意義的編號，因此只在隊形類型相同時帶入。
 * 由提案端決定，所有 Agent 收到的 epoch 內容一致，指派仍逐位元相同。
 *
 * @param has_accepted_epoch 本機是否已有 accepted epoch
 * @param parent_formation_type parent epoch 的隊形類型
 * @param proposal_formation_type 新 proposal 的隊形類型
 * @return true 表示應帶入 parent assignment
 */
inline bool should_carry_parent_assignment(
    const bool has_accepted_epoch,
    const uint8_t parent_formation_type,
    const uint8_t proposal_formation_type)
{
    return has_accepted_epoch && parent_formation_type == proposal_formation_type;
}

/**
 * @brief 判斷未完成的候選 epoch 是否已逾時。
 * @param candidate_age_s 候選 epoch 建立或採用後的經過時間（秒）
 * @return true 表示應丟棄並等待重新提案
 */
inline bool is_epoch_proposal_expired(const double candidate_age_s)
{
    return std::isfinite(candidate_age_s) &&
        candidate_age_s > FORMATION_EPOCH_PROPOSAL_TIMEOUT_S;
}

/// 候選 epoch 的識別鍵（sequence、snapshot hash、proposer）。
struct FormationEpochKey
{
    uint32_t sequence{0U};
    uint32_t snapshot_hash{0U};
    int32_t proposer_agent_id{0};

    /**
     * @brief 比較兩個識別鍵是否相同。
     * @param other 另一個識別鍵
     * @return true 表示指向同一個 epoch snapshot
     */
    bool operator==(const FormationEpochKey& other) const
    {
        return sequence == other.sequence &&
            snapshot_hash == other.snapshot_hash &&
            proposer_agent_id == other.proposer_agent_id;
    }
};

/**
 * @brief 記錄已逾時的候選 epoch，防止 heartbeat 重新帶回。
 */
class ExpiredFormationEpochs
{
public:
    /**
     * @brief 記錄一個已逾時的候選 epoch。
     * @param key epoch 識別鍵
     */
    void remember(const FormationEpochKey& key)
    {
        if (contains(key))
        {
            return;
        }
        keys_.push_back(key);
        while (keys_.size() > FORMATION_EXPIRED_EPOCH_HISTORY_SIZE)
        {
            keys_.pop_front();
        }
    }

    /**
     * @brief 判斷 epoch 是否曾逾時。
     * @param key epoch 識別鍵
     * @return true 表示不可再次採用
     */
    bool contains(const FormationEpochKey& key) const
    {
        return std::find(keys_.begin(), keys_.end(), key) != keys_.end();
    }

    /** @brief 降落或任務清除時重置紀錄。 */
    void clear()
    {
        keys_.clear();
    }

private:
    std::deque<FormationEpochKey> keys_;
};

}  // namespace ego_swarm
}  // namespace taiphoon
