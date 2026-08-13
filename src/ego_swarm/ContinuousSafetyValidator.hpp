/**
 * @file ContinuousSafetyValidator.hpp
 * @brief EGO-Swarm trajectory 的連續取樣安全驗證
 * @author Taiphoon Agent Team
 * @date 2026-08-12
 *
 * 此模組維持 planner backend 與 safety contract 的獨立 seam。它只依賴
 * 純 C++ trajectory、peer table 與 obstacle map，不直接接觸 ROS2 或 PX4。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "ego_swarm/EgoSwarmPlanner.hpp"
#include "ego_swarm/TrajectoryPriority.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// ContinuousSafetyValidator 失敗原因，供 Node 診斷與測試使用。
enum class SafetyFailureReason
{
    NONE = 0,
    INVALID_CONFIG,
    INVALID_CONTEXT,
    INVALID_FRAME,
    INVALID_TIME,
    STALE_PEER,
    STALE_MAP,
    NONFINITE_TRAJECTORY,
    DYNAMIC_INFEASIBLE,
    PEER_COLLISION,
    OBSTACLE_COLLISION
};

/**
 * @brief 驗證器需要的 immutable planner cycle context
 */
struct SafetyValidationContext
{
    VehiclePlanningState self_state{};
    double now_s{0.0};
    double trajectory_start_time_s{0.0};
    double max_time_skew_s{0.5};
    uint32_t frame_revision{0U};
    uint32_t trajectory_frame_revision{0U};
    int self_agent_id{0};
    const PeerTrajectoryTable* peer_table{nullptr};
    const perception::IObstacleMap* obstacle_map{nullptr};
};

/**
 * @brief 驗證結果與可觀測安全指標
 */
struct SafetyValidationResult
{
    bool is_safe{false};
    SafetyFailureReason reason{SafetyFailureReason::INVALID_CONTEXT};
    double minimum_peer_distance_m{0.0};
    double minimum_obstacle_distance_m{0.0};
    double maximum_velocity_mps{0.0};
    double maximum_acceleration_mps2{0.0};
    double maximum_jerk_mps3{0.0};

    /**
     * @brief 檢查驗證結果指標是否可供發布
     * @return true 表示所有指標為有限值
     */
    bool has_finite_metrics() const
    {
        return std::isfinite(minimum_peer_distance_m) &&
               std::isfinite(minimum_obstacle_distance_m) &&
               std::isfinite(maximum_velocity_mps) &&
               std::isfinite(maximum_acceleration_mps2) &&
               std::isfinite(maximum_jerk_mps3);
    }
};

/**
 * @brief ContinuousSafetyValidator
 *
 * 以固定時間間隔取樣整條 B-spline，檢查時間／frame、動力學限制、
 * 同儕中心距離與局部障礙物 clearance。這是 safety gate，不是 planner
 * cost；任何 backend 都必須通過同一個介面。
 */
class ContinuousSafetyValidator
{
public:
    /**
     * @brief 使用 planner 設定建立驗證器
     * @param config planner 與 safety 共用設定
     */
    explicit ContinuousSafetyValidator(
        const PlannerConfig& config = PlannerConfig{})
    {
        if (!set_config(config))
        {
            config_ = PlannerConfig{};
        }
    }

    /**
     * @brief 更新驗證器設定
     * @param config planner 與 safety 共用設定
     * @return true 表示設定有效
     */
    bool set_config(const PlannerConfig& config)
    {
        if (!is_valid_config(config))
        {
            return false;
        }
        config_ = config;
        return true;
    }

    /**
     * @brief 驗證一條具時間與 frame metadata 的 trajectory
     * @param trajectory 待驗證 B-spline
     * @param context immutable planner cycle context
     * @return safety 結果與最小距離／最大導數指標
     */
    SafetyValidationResult validate(
        const BsplineTrajectory& trajectory,
        const SafetyValidationContext& context) const
    {
        SafetyValidationResult result;
        if (!is_valid_config(config_))
        {
            result.reason = SafetyFailureReason::INVALID_CONFIG;
            return result;
        }
        if (context.peer_table == nullptr ||
            context.obstacle_map == nullptr ||
            context.self_agent_id <= 0 ||
            !context.self_state.is_finite())
        {
            result.reason = SafetyFailureReason::INVALID_CONTEXT;
            return result;
        }
        if (!is_valid_frame(context) || !is_valid_time(trajectory, context))
        {
            result.reason = is_valid_frame(context)
                ? SafetyFailureReason::INVALID_TIME
                : SafetyFailureReason::INVALID_FRAME;
            return result;
        }
        if (!context.obstacle_map->is_valid() ||
            !context.obstacle_map->is_fresh(context.now_s))
        {
            result.reason = SafetyFailureReason::STALE_MAP;
            return result;
        }
        if (!context.peer_table->all_fresh(context.now_s))
        {
            result.reason = SafetyFailureReason::STALE_PEER;
            return result;
        }

        std::vector<PeerTrajectory> peers;
        context.peer_table->get_active(context.now_s, peers);
        peers.erase(
            std::remove_if(
                peers.begin(), peers.end(),
                [&context](const PeerTrajectory& peer)
                {
                    return peer.agent_id == context.self_agent_id;
                }),
            peers.end());
        return validate_samples(trajectory, context, peers, result);
    }

    /**
     * @brief 取得最後設定
     * @return 唯讀 planner 設定
     */
    const PlannerConfig& config() const
    {
        return config_;
    }

    /**
     * @brief 將失敗原因轉為穩定診斷文字
     * @param reason safety 失敗原因
     * @return 簡短英文 reason code
     */
    static std::string reason_string(const SafetyFailureReason reason)
    {
        switch (reason)
        {
            case SafetyFailureReason::NONE:
                return "none";
            case SafetyFailureReason::INVALID_CONFIG:
                return "invalid_config";
            case SafetyFailureReason::INVALID_CONTEXT:
                return "invalid_context";
            case SafetyFailureReason::INVALID_FRAME:
                return "invalid_frame";
            case SafetyFailureReason::INVALID_TIME:
                return "invalid_time";
            case SafetyFailureReason::STALE_PEER:
                return "stale_peer";
            case SafetyFailureReason::STALE_MAP:
                return "stale_map";
            case SafetyFailureReason::NONFINITE_TRAJECTORY:
                return "nonfinite_trajectory";
            case SafetyFailureReason::DYNAMIC_INFEASIBLE:
                return "dynamic_infeasible";
            case SafetyFailureReason::PEER_COLLISION:
                return "peer_collision";
            case SafetyFailureReason::OBSTACLE_COLLISION:
                return "obstacle_collision";
        }
        return "unknown";
    }

private:
    static bool is_valid_config(const PlannerConfig& config)
    {
        return std::isfinite(config.sample_interval_s) &&
               config.sample_interval_s > EGO_SWARM_EPSILON_M &&
               std::isfinite(config.max_velocity_mps) &&
               config.max_velocity_mps > EGO_SWARM_EPSILON_M &&
               std::isfinite(config.max_acceleration_mps2) &&
               config.max_acceleration_mps2 > EGO_SWARM_EPSILON_M &&
               std::isfinite(config.max_jerk_mps3) &&
               config.max_jerk_mps3 > EGO_SWARM_EPSILON_M &&
               std::isfinite(config.obstacle_clearance_m) &&
               config.obstacle_clearance_m >= 0.0 &&
               std::isfinite(config.peer_collision_min_distance_m) &&
               config.peer_collision_min_distance_m >= 0.0;
    }

    static bool is_valid_frame(const SafetyValidationContext& context)
    {
        return context.frame_revision != 0U &&
               context.trajectory_frame_revision != 0U &&
               context.frame_revision == context.trajectory_frame_revision;
    }

    bool is_valid_time(
        const BsplineTrajectory& trajectory,
        const SafetyValidationContext& context) const
    {
        constexpr double TIME_EPSILON_S = 1.0e-6;
        constexpr double VEHICLE_STATE_TIMEOUT_S = 1.0;
        if (!trajectory.is_valid() || !std::isfinite(context.now_s) ||
            !std::isfinite(context.trajectory_start_time_s) ||
            !std::isfinite(context.self_state.timestamp_s) ||
            !std::isfinite(context.max_time_skew_s) ||
            context.max_time_skew_s < 0.0)
        {
            return false;
        }
        if (context.self_state.timestamp_s > context.now_s +
                context.max_time_skew_s + TIME_EPSILON_S ||
            context.now_s - context.self_state.timestamp_s >
                VEHICLE_STATE_TIMEOUT_S + TIME_EPSILON_S)
        {
            return false;
        }
        if (context.trajectory_start_time_s > context.now_s +
                context.max_time_skew_s + TIME_EPSILON_S)
        {
            return false;
        }
        const double trajectory_end_time =
            context.trajectory_start_time_s + trajectory.duration_s();
        return std::isfinite(trajectory_end_time) &&
               context.now_s <= trajectory_end_time + TIME_EPSILON_S;
    }

    std::size_t sample_count(const double duration_s) const
    {
        const double raw_count = std::ceil(
            duration_s / config_.sample_interval_s);
        if (!std::isfinite(raw_count) || raw_count <= 0.0)
        {
            return 1U;
        }
        return std::min(
            static_cast<std::size_t>(raw_count), EGO_SWARM_MAX_SAMPLE_COUNT);
    }

    static double vector_norm(const Vector3& value)
    {
        return std::sqrt(squared_norm(value));
    }

    SafetyValidationResult validate_samples(
        const BsplineTrajectory& trajectory,
        const SafetyValidationContext& context,
        const std::vector<PeerTrajectory>& peers,
        SafetyValidationResult result) const
    {
        result.reason = SafetyFailureReason::NONE;
        result.minimum_peer_distance_m = peers.empty()
            ? 0.0 : std::numeric_limits<double>::max();
        result.minimum_obstacle_distance_m =
            context.obstacle_map->point_count() == 0U
            ? 0.0 : std::numeric_limits<double>::max();
        const std::size_t count = sample_count(trajectory.duration_s());
        for (std::size_t index = 0U; index <= count; ++index)
        {
            const double relative_time = std::min(
                trajectory.duration_s(),
                static_cast<double>(index) * config_.sample_interval_s);
            const TrajectoryState sample = trajectory.evaluate_state(
                relative_time);
            if (!sample.is_valid)
            {
                result.reason = SafetyFailureReason::NONFINITE_TRAJECTORY;
                return result;
            }
            if (!validate_dynamic_state(sample, result))
            {
                result.reason = SafetyFailureReason::DYNAMIC_INFEASIBLE;
                return result;
            }
            if (!validate_obstacle(sample.position, context, result))
            {
                result.reason = SafetyFailureReason::OBSTACLE_COLLISION;
                return result;
            }
            if (!validate_peers(
                    sample.position, relative_time, context, peers, result))
            {
                result.reason = SafetyFailureReason::PEER_COLLISION;
                return result;
            }
        }
        result.is_safe = result.has_finite_metrics();
        if (!result.is_safe)
        {
            result.reason = SafetyFailureReason::NONFINITE_TRAJECTORY;
        }
        return result;
    }

    bool validate_dynamic_state(
        const TrajectoryState& sample,
        SafetyValidationResult& result) const
    {
        const double velocity = vector_norm(sample.velocity);
        const double acceleration = vector_norm(sample.acceleration);
        const double jerk = vector_norm(sample.jerk);
        if (!std::isfinite(velocity) || !std::isfinite(acceleration) ||
            !std::isfinite(jerk))
        {
            return false;
        }
        result.maximum_velocity_mps =
            std::max(result.maximum_velocity_mps, velocity);
        result.maximum_acceleration_mps2 =
            std::max(result.maximum_acceleration_mps2, acceleration);
        result.maximum_jerk_mps3 =
            std::max(result.maximum_jerk_mps3, jerk);
        return velocity <= config_.max_velocity_mps + EGO_SWARM_EPSILON_M &&
               acceleration <= config_.max_acceleration_mps2 +
                   EGO_SWARM_EPSILON_M &&
               jerk <= config_.max_jerk_mps3 + EGO_SWARM_EPSILON_M;
    }

    bool validate_obstacle(
        const Vector3& position,
        const SafetyValidationContext& context,
        SafetyValidationResult& result) const
    {
        if (context.obstacle_map->point_count() == 0U)
        {
            return true;
        }
        const double distance = context.obstacle_map->nearest_distance(
            position);
        if (!std::isfinite(distance))
        {
            return false;
        }
        result.minimum_obstacle_distance_m = std::min(
            result.minimum_obstacle_distance_m, distance);
        return context.obstacle_map->is_position_safe(
            position, config_.obstacle_clearance_m);
    }

    bool validate_peers(
        const Vector3& position,
        const double relative_time_s,
        const SafetyValidationContext& context,
        const std::vector<PeerTrajectory>& peers,
        SafetyValidationResult& result) const
    {
        const double absolute_time_s =
            context.trajectory_start_time_s + relative_time_s;
        if (!std::isfinite(absolute_time_s))
        {
            return false;
        }
        for (const PeerTrajectory& peer : peers)
        {
            if (absolute_time_s > peer.end_time_s + EGO_SWARM_EPSILON_M)
            {
                continue;
            }
            // 路線開始前的同儕仍佔據自己的目前位置；只有同儕已進入
            // 路線時間窗後，較早的路線才可取得優先權。此規則必須與
            // planner 的候選評估一致，避免 planner 放行後又被 safety
            // 以不同的時間仲裁結果否決。
            const bool peer_route_started = absolute_time_s +
                EGO_SWARM_EPSILON_M >= peer.start_time_s;
            if (peer_route_started && has_trajectory_priority(
                    context.trajectory_start_time_s,
                    context.self_agent_id,
                    peer.start_time_s,
                    peer.agent_id))
            {
                continue;
            }
            const double peer_time_s = std::max(
                0.0, absolute_time_s - peer.start_time_s);
            const TrajectoryState peer_sample =
                peer.trajectory.evaluate_state(peer_time_s);
            if (!peer_sample.is_valid)
            {
                return false;
            }
            const double distance = distance_between_centers(
                position, peer_sample.position);
            if (!std::isfinite(distance))
            {
                return false;
            }
            result.minimum_peer_distance_m = std::min(
                result.minimum_peer_distance_m, distance);
            if (distance + EGO_SWARM_EPSILON_M <
                config_.peer_collision_min_distance_m)
            {
                return false;
            }
        }
        return true;
    }

    PlannerConfig config_{};
};

}  // namespace ego_swarm
}  // namespace taiphoon
