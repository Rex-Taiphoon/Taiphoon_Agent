/**
 * @file CollisionSafetyMonitor.cpp
 * @brief 多機路徑安全距離監控器實作
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 */

#include "safety/CollisionSafetyMonitor.hpp"

#include <algorithm>
#include <cmath>

namespace taiphoon
{
namespace safety
{

namespace
{

constexpr float DEFAULT_SAFETY_MARGIN_M = 3.0F;
constexpr float DISTANCE_EPSILON_M = 1e-6F;

bool is_finite_position(const std::array<float, 3>& position)
{
    return std::isfinite(position.at(0)) &&
        std::isfinite(position.at(1)) &&
        std::isfinite(position.at(2));
}

float point_to_segment_distance(
    const std::array<float, 3>& point,
    const std::array<float, 3>& segment_start,
    const std::array<float, 3>& segment_end)
{
    const float segment_x = segment_end.at(0) - segment_start.at(0);
    const float segment_y = segment_end.at(1) - segment_start.at(1);
    const float segment_z = segment_end.at(2) - segment_start.at(2);
    const float segment_length_squared =
        segment_x * segment_x + segment_y * segment_y + segment_z * segment_z;

    float projection = 0.0F;
    if (segment_length_squared > DISTANCE_EPSILON_M)
    {
        const float point_x = point.at(0) - segment_start.at(0);
        const float point_y = point.at(1) - segment_start.at(1);
        const float point_z = point.at(2) - segment_start.at(2);
        projection = (point_x * segment_x + point_y * segment_y + point_z * segment_z) /
            segment_length_squared;
        projection = std::clamp(projection, 0.0F, 1.0F);
    }

    const float closest_x = segment_start.at(0) + projection * segment_x;
    const float closest_y = segment_start.at(1) + projection * segment_y;
    const float closest_z = segment_start.at(2) + projection * segment_z;
    const float delta_x = point.at(0) - closest_x;
    const float delta_y = point.at(1) - closest_y;
    const float delta_z = point.at(2) - closest_z;
    return std::sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
}

}  // namespace

CollisionSafetyMonitor::CollisionSafetyMonitor(float safety_margin_m)
    : safety_margin_m_(std::isfinite(safety_margin_m) ?
        std::max(safety_margin_m, DEFAULT_SAFETY_MARGIN_M) :
        DEFAULT_SAFETY_MARGIN_M)
{
}

CollisionDecision CollisionSafetyMonitor::evaluate_path(
    const std::array<float, 3>& current_ned,
    const std::array<float, 3>& target_ned,
    const std::vector<PeerCollisionObservation>& peers) const
{
    CollisionDecision decision;
    decision.is_safe = is_finite_position(current_ned) &&
        is_finite_position(target_ned);
    if (!decision.is_safe)
    {
        return decision;
    }

    for (const auto& peer : peers)
    {
        if (!peer.is_valid || !is_finite_position(peer.position_ned))
        {
            decision.is_safe = false;
            decision.has_conflict = true;
            decision.conflicting_agent_index = peer.agent_index;
            return decision;
        }

        const float distance = point_to_segment_distance(
            peer.position_ned, current_ned, target_ned);
        if (distance < decision.minimum_distance_m)
        {
            decision.minimum_distance_m = distance;
            decision.conflicting_agent_index = peer.agent_index;
        }

        if (distance + DISTANCE_EPSILON_M < safety_margin_m_)
        {
            decision.is_safe = false;
            decision.has_conflict = true;
        }
    }

    return decision;
}

CollisionTargetGate::CollisionTargetGate(float safety_margin_m)
    : monitor_(safety_margin_m)
{
}

void CollisionTargetGate::set_requested_target(
    const std::array<float, 4>& target_ned_yaw)
{
    requested_target_ = target_ned_yaw;
    has_requested_target_ = true;
    is_target_applied_ = false;
}

void CollisionTargetGate::clear_target()
{
    has_requested_target_ = false;
    is_target_applied_ = false;
    is_collision_hold_active_ = false;
}

CollisionTargetEvaluation CollisionTargetGate::evaluate(
    const std::array<float, 3>& current_ned,
    const std::vector<PeerCollisionObservation>& peers)
{
    CollisionTargetEvaluation evaluation;
    if (!has_requested_target_)
    {
        return evaluation;
    }

    evaluation.target_ned = {
        requested_target_.at(0), requested_target_.at(1), requested_target_.at(2)};
    evaluation.yaw_deg = requested_target_.at(3);
    evaluation.decision = monitor_.evaluate_path(
        current_ned, evaluation.target_ned, peers);

    if (!evaluation.decision.is_safe)
    {
        if (!is_collision_hold_active_)
        {
            evaluation.action = CollisionTargetAction::HOLD_POSITION;
        }
        is_collision_hold_active_ = true;
        is_target_applied_ = false;
        return evaluation;
    }

    if (!is_target_applied_ || is_collision_hold_active_)
    {
        evaluation.action = CollisionTargetAction::APPLY_TARGET;
    }
    is_target_applied_ = true;
    is_collision_hold_active_ = false;
    return evaluation;
}

}  // namespace safety
}  // namespace taiphoon
