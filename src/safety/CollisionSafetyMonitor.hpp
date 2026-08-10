/**
 * @file CollisionSafetyMonitor.hpp
 * @brief 多機路徑安全距離監控器介面
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 */

#pragma once

#include <array>
#include <limits>
#include <vector>

namespace taiphoon
{
namespace safety
{

/// 已轉換至自身 NED frame 的同儕碰撞觀測
struct PeerCollisionObservation
{
    int agent_index{0};
    std::array<float, 3> position_ned{0.0F, 0.0F, 0.0F};
    bool is_valid{false};
};

/// 路徑安全判斷結果
struct CollisionDecision
{
    bool is_safe{false};
    bool has_conflict{false};
    int conflicting_agent_index{0};
    float minimum_distance_m{std::numeric_limits<float>::infinity()};
};

/// 防撞目標關卡要求節點執行的動作
enum class CollisionTargetAction
{
    NONE = 0,
    APPLY_TARGET,
    HOLD_POSITION
};

/// 防撞目標關卡評估結果
struct CollisionTargetEvaluation
{
    CollisionTargetAction action{CollisionTargetAction::NONE};
    std::array<float, 3> target_ned{0.0F, 0.0F, 0.0F};
    float yaw_deg{std::numeric_limits<float>::quiet_NaN()};
    CollisionDecision decision;
};

/**
 * @brief 以三維線段距離判斷目標路徑是否符合多機安全邊界
 */
class CollisionSafetyMonitor
{
public:
    /**
     * @brief 建立防撞安全監控器
     * @param safety_margin_m 兩機最小允許三維距離（公尺）
     */
    explicit CollisionSafetyMonitor(float safety_margin_m = 3.0F);

    /**
     * @brief 評估目前位置到目標位置的整段路徑
     * @param current_ned 自身目前 NED 位置
     * @param target_ned 自身目標 NED 位置
     * @param peers 已轉換至自身 NED frame 的同儕觀測
     * @return 路徑安全判斷結果
     */
    CollisionDecision evaluate_path(
        const std::array<float, 3>& current_ned,
        const std::array<float, 3>& target_ned,
        const std::vector<PeerCollisionObservation>& peers) const;

private:
    float safety_margin_m_{3.0F};
};

/**
 * @brief 保存最新移動目標並管理碰撞保持與自動恢復
 */
class CollisionTargetGate
{
public:
    /**
     * @brief 建立防撞目標關卡
     * @param safety_margin_m 兩機最小允許三維距離（公尺）
     */
    explicit CollisionTargetGate(float safety_margin_m = 3.0F);

    /**
     * @brief 登記最新移動目標，取代先前尚未完成的目標
     * @param target_ned_yaw NED 目標 [x, y, z, yaw_deg]
     */
    void set_requested_target(const std::array<float, 4>& target_ned_yaw);

    /** @brief 清除移動目標與碰撞保持狀態 */
    void clear_target();

    /**
     * @brief 依最新觀測決定套用目標、原地保持或維持現狀
     * @param current_ned 自身目前 NED 位置
     * @param peers 已轉換至自身 NED frame 的同儕觀測
     * @return 節點應執行的動作與安全判斷
     */
    CollisionTargetEvaluation evaluate(
        const std::array<float, 3>& current_ned,
        const std::vector<PeerCollisionObservation>& peers);

private:
    CollisionSafetyMonitor monitor_;
    std::array<float, 4> requested_target_{
        0.0F, 0.0F, 0.0F, std::numeric_limits<float>::quiet_NaN()};
    bool has_requested_target_{false};
    bool is_target_applied_{false};
    bool is_collision_hold_active_{false};
};

}  // namespace safety
}  // namespace taiphoon
