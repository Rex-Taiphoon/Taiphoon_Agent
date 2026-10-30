/**
 * @file formation_epoch_policy_test.cpp
 * @brief Agent 關閉／重新啟動時 FormationEpoch 重組不死鎖的回歸測試
 * @author Taiphoon Agent Team
 * @date 2026-10-15
 */
#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <vector>

#include "ego_swarm/EgoSwarmTypes.hpp"
#include "ego_swarm/FormationEpochPolicy.hpp"

using taiphoon::ego_swarm::ExpiredFormationEpochs;
using taiphoon::ego_swarm::FORMATION_EPOCH_PROPOSAL_TIMEOUT_S;
using taiphoon::ego_swarm::FORMATION_EXPIRED_EPOCH_HISTORY_SIZE;
using taiphoon::ego_swarm::FORMATION_JOIN_READY_TIMEOUT_S;
using taiphoon::ego_swarm::FormationEpochKey;
using taiphoon::ego_swarm::FormationType;
using taiphoon::ego_swarm::can_include_formation_peer;
using taiphoon::ego_swarm::has_departed_member;
using taiphoon::ego_swarm::is_epoch_lineage_acceptable;
using taiphoon::ego_swarm::is_epoch_proposal_expired;
using taiphoon::ego_swarm::is_membership_departure;
using taiphoon::ego_swarm::should_carry_parent_assignment;

namespace
{

/**
 * @brief 轉成 FormationEpoch 訊息使用的隊形類型編碼
 * @param type 隊形類型
 * @return 訊息欄位值
 */
uint8_t to_message_type(const FormationType type)
{
    return static_cast<uint8_t>(type);
}

}  // namespace

TEST(FormationEpochPolicy, NewerEpochWithDifferentParentIsAcceptable)
{
    // 存活成員停在 seq 3，另一成員已啟用 seq 4 並提出 seq 5（parent 4）。
    EXPECT_TRUE(is_epoch_lineage_acceptable(true, 3U, false, 5U));
    EXPECT_TRUE(is_epoch_lineage_acceptable(true, 3U, true, 4U));
    EXPECT_TRUE(is_epoch_lineage_acceptable(false, 0U, false, 7U));
}

TEST(FormationEpochPolicy, OlderOrSameSequenceForkIsRejected)
{
    EXPECT_FALSE(is_epoch_lineage_acceptable(true, 4U, false, 4U));
    EXPECT_FALSE(is_epoch_lineage_acceptable(true, 4U, false, 3U));
}

TEST(FormationEpochPolicy, DetectsDepartedMemberOnlyWithStableView)
{
    const std::vector<int> zombie_epoch{1, 2, 3};
    const std::vector<int> alive{2, 3};
    EXPECT_TRUE(has_departed_member(zombie_epoch, alive, 2, true));
    EXPECT_FALSE(has_departed_member(zombie_epoch, alive, 2, false));
    EXPECT_FALSE(has_departed_member(alive, alive, 2, true));
    // 本機永遠視為存活，即使本機狀態暫時不在檢視中。
    EXPECT_FALSE(has_departed_member({2, 3}, {3}, 2, true));
    EXPECT_FALSE(has_departed_member({}, alive, 2, true));
}

TEST(FormationEpochPolicy, OnlyDepartureRequiresHold)
{
    EXPECT_TRUE(is_membership_departure({1, 2, 3}, {2, 3}));
    EXPECT_TRUE(is_membership_departure({1, 2, 3}, {1, 2, 4}));
    EXPECT_FALSE(is_membership_departure({1, 2}, {1, 2, 3}));
    EXPECT_FALSE(is_membership_departure({1, 2, 3}, {1, 2, 3}));
    EXPECT_FALSE(is_membership_departure({}, {1}));
}

TEST(FormationEpochPolicy, RestartedAgentJoinsOnlyAfterReadySignal)
{
    // 既有成員不需要 ready 訊號。
    EXPECT_TRUE(can_include_formation_peer(true, false, 0.0));
    // 重新啟動、尚未收到編隊命令的 Agent 不列入，存活成員不會被卡住。
    EXPECT_FALSE(can_include_formation_peer(false, false, 0.0));
    EXPECT_TRUE(can_include_formation_peer(false, true, 0.5));
    EXPECT_TRUE(can_include_formation_peer(
        false, true, FORMATION_JOIN_READY_TIMEOUT_S));
}

TEST(FormationEpochPolicy, StaleOrInvalidReadySignalIsIgnored)
{
    EXPECT_FALSE(can_include_formation_peer(
        false, true, FORMATION_JOIN_READY_TIMEOUT_S + 0.01));
    EXPECT_FALSE(can_include_formation_peer(false, true, -0.1));
    EXPECT_FALSE(can_include_formation_peer(
        false, true, std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(can_include_formation_peer(
        false, true, std::numeric_limits<double>::infinity()));
}

TEST(FormationEpochPolicy, ProposalTimeoutBoundary)
{
    EXPECT_FALSE(is_epoch_proposal_expired(0.0));
    EXPECT_FALSE(is_epoch_proposal_expired(FORMATION_EPOCH_PROPOSAL_TIMEOUT_S));
    EXPECT_TRUE(is_epoch_proposal_expired(FORMATION_EPOCH_PROPOSAL_TIMEOUT_S + 0.01));
    EXPECT_FALSE(is_epoch_proposal_expired(
        std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(is_epoch_proposal_expired(
        std::numeric_limits<double>::infinity()));
}

TEST(FormationEpochPolicy, ExpiredEpochCannotBeReinfectedByHeartbeat)
{
    ExpiredFormationEpochs expired;
    const FormationEpochKey zombie{5U, 1234U, 1};
    const FormationEpochKey recovery{5U, 5678U, 2};
    EXPECT_FALSE(expired.contains(zombie));
    expired.remember(zombie);
    expired.remember(zombie);
    EXPECT_TRUE(expired.contains(zombie));
    EXPECT_FALSE(expired.contains(recovery));
    expired.clear();
    EXPECT_FALSE(expired.contains(zombie));
}

TEST(FormationEpochPolicy, ExpiredHistoryIsBounded)
{
    ExpiredFormationEpochs expired;
    for (uint32_t sequence = 1U;
         sequence <= FORMATION_EXPIRED_EPOCH_HISTORY_SIZE + 1U; ++sequence)
    {
        expired.remember(FormationEpochKey{sequence, sequence, 1});
    }
    EXPECT_FALSE(expired.contains(FormationEpochKey{1U, 1U, 1}));
    EXPECT_TRUE(expired.contains(FormationEpochKey{
        FORMATION_EXPIRED_EPOCH_HISTORY_SIZE + 1U,
        FORMATION_EXPIRED_EPOCH_HISTORY_SIZE + 1U, 1}));
}

TEST(FormationEpochPolicy, ParentAssignmentCarriedOnlyForSameFormationType)
{
    // 同隊形的成員變更或群組 MOVE 保留換槽成本，避免無謂交換槽位。
    EXPECT_TRUE(should_carry_parent_assignment(
        true, to_message_type(FormationType::HORI_LINEAR),
        to_message_type(FormationType::HORI_LINEAR)));
    EXPECT_TRUE(should_carry_parent_assignment(
        true, to_message_type(FormationType::V_SHAPE),
        to_message_type(FormationType::V_SHAPE)));
    // 一字 → 燕行：槽位編號語意不同，不帶入舊歸屬。
    EXPECT_FALSE(should_carry_parent_assignment(
        true, to_message_type(FormationType::HORI_LINEAR),
        to_message_type(FormationType::V_SHAPE)));
    EXPECT_FALSE(should_carry_parent_assignment(
        true, to_message_type(FormationType::V_SHAPE),
        to_message_type(FormationType::HORI_LINEAR)));
    EXPECT_FALSE(should_carry_parent_assignment(
        true, to_message_type(FormationType::CIRCLE),
        to_message_type(FormationType::VERT_LINEAR)));
}

TEST(FormationEpochPolicy, NoAcceptedEpochHasNoParentAssignment)
{
    EXPECT_FALSE(should_carry_parent_assignment(
        false, to_message_type(FormationType::HORI_LINEAR),
        to_message_type(FormationType::HORI_LINEAR)));
}
