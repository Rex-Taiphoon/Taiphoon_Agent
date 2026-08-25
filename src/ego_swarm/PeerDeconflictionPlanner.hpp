/**
 * @file PeerDeconflictionPlanner.hpp
 * @brief 近距離同儕衝突的 deterministic recovery planner
 *
 * 這個模組只處理「目前已經低於 peer hard clearance」的恢復階段。
 * 正常任務仍由 OptimizedEgoPlanner 與 ContinuousSafetyValidator 處理。
 * recovery 的特殊安全契約是：不讓既有違規距離惡化，並把衝突 peer
 * 拉到 release distance；對原本未違規的 peer 則全程維持 hard clearance。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "ego_swarm/PlannerTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

enum class PeerDeconflictionAction
{
    NONE = 0,
    HOLD,
    ESCAPE,
    BLOCKED
};

struct PeerDeconflictionPeer
{
    int agent_id{0};
    Vector3 position{0.0, 0.0, 0.0};
    Vector3 velocity{0.0, 0.0, 0.0};
    bool velocity_valid{false};
    // terminal-hold peer 已明確宣告自己只佔位、不參與本次 mover 仲裁。
    bool can_move{true};
};

struct PeerDeconflictionConfig
{
    double hard_distance_m{3.0};
    double release_margin_m{1.0};
    double max_escape_distance_m{12.0};
    double sample_interval_s{0.1};
    double stationary_peer_speed_mps{0.15};
    double progress_tolerance_m{0.10};
};

struct PeerDeconflictionInput
{
    int self_agent_id{0};
    Vector3 self_position{0.0, 0.0, 0.0};
    Vector3 final_target{0.0, 0.0, 0.0};
    double now_s{0.0};
    bool peers_are_fresh{false};
    const perception::IObstacleMap* obstacle_map{nullptr};
    PlannerConfig planner_config{};
    std::vector<PeerDeconflictionPeer> peers{};
};

struct PeerDeconflictionResult
{
    PeerDeconflictionAction action{PeerDeconflictionAction::NONE};
    bool is_valid{false};
    int owner_agent_id{0};
    Vector3 escape_target{0.0, 0.0, 0.0};
    PlannerResult plan{};
    double minimum_start_distance_m{0.0};
    std::vector<int> conflict_peer_ids{};
    std::string reason{};
};

struct PeerDeconflictionValidationResult
{
    bool is_safe{false};
    double minimum_peer_distance_m{0.0};
    double minimum_obstacle_distance_m{0.0};
    std::string reason{};
};

/**
 * @brief 近距離 peer recovery 的 deterministic planner
 *
 * Interface 只有決策與剩餘軌跡驗證兩個入口。呼叫端不需要知道 component
 * 選擇、owner 仲裁、方向候選、平滑軌跡建構或 recovery safety invariant。
 */
class PeerDeconflictionPlanner final
{
public:
    explicit PeerDeconflictionPlanner(
        const PeerDeconflictionConfig& config = PeerDeconflictionConfig{})
    {
        if (!set_config(config))
        {
            config_ = PeerDeconflictionConfig{};
        }
    }

    bool set_config(const PeerDeconflictionConfig& config)
    {
        if (!std::isfinite(config.hard_distance_m) ||
            config.hard_distance_m <= EGO_SWARM_EPSILON_M ||
            !std::isfinite(config.release_margin_m) ||
            config.release_margin_m <= 0.0 ||
            !std::isfinite(config.max_escape_distance_m) ||
            config.max_escape_distance_m <= config.hard_distance_m ||
            !std::isfinite(config.sample_interval_s) ||
            config.sample_interval_s <= EGO_SWARM_EPSILON_M ||
            !std::isfinite(config.stationary_peer_speed_mps) ||
            config.stationary_peer_speed_mps < 0.0 ||
            !std::isfinite(config.progress_tolerance_m) ||
            config.progress_tolerance_m < 0.0)
        {
            return false;
        }
        config_ = config;
        return true;
    }

    const PeerDeconflictionConfig& config() const
    {
        return config_;
    }

    /**
     * @brief 依目前 live peer 狀態決定 hold、escape 或無需 recovery。
     *
     * 只有 owner 會得到 ESCAPE。其他 Agent 得到 HOLD；資料、地圖、動力學
     * 或候選路徑無法證明安全時得到 BLOCKED，交由 Node 的 fail-closed 路徑處理。
     */
    PeerDeconflictionResult decide(
        const PeerDeconflictionInput& input) const
    {
        PeerDeconflictionResult result;
        if (!is_valid_input(input))
        {
            result.action = PeerDeconflictionAction::BLOCKED;
            result.reason = "invalid recovery input";
            return result;
        }

        std::vector<int> component_peer_indices;
        if (!build_self_component(input, component_peer_indices))
        {
            result.is_valid = true;
            result.action = PeerDeconflictionAction::NONE;
            result.reason = "no peer inside hard clearance";
            return result;
        }

        result.conflict_peer_ids.reserve(component_peer_indices.size());
        result.owner_agent_id = input.self_agent_id;
        result.minimum_start_distance_m =
            std::numeric_limits<double>::infinity();
        for (const int peer_index : component_peer_indices)
        {
            const auto& peer = input.peers.at(static_cast<std::size_t>(peer_index));
            result.conflict_peer_ids.push_back(peer.agent_id);
            if (peer.can_move)
            {
                result.owner_agent_id = std::min(
                    result.owner_agent_id, peer.agent_id);
            }
            result.minimum_start_distance_m = std::min(
                result.minimum_start_distance_m,
                distance_between_centers(input.self_position, peer.position));
        }

        if (result.owner_agent_id != input.self_agent_id)
        {
            result.is_valid = true;
            result.action = PeerDeconflictionAction::HOLD;
            result.reason = "waiting for deterministic recovery owner";
            return result;
        }

        for (const int peer_index : component_peer_indices)
        {
            const auto& peer = input.peers.at(static_cast<std::size_t>(peer_index));
            if (peer.velocity_valid &&
                (!is_finite_vector(peer.velocity) ||
                 vector_norm(peer.velocity) > config_.stationary_peer_speed_mps))
            {
                result.is_valid = true;
                result.action = PeerDeconflictionAction::HOLD;
                result.reason = "waiting for conflicting peer to brake";
                return result;
            }
        }

        const std::vector<Vector3> directions = build_directions(input);
        for (const Vector3& direction : directions)
        {
            const Vector3 target = target_on_ray(input, direction);
            PlannerResult candidate = build_escape_plan(input, target);
            if (!candidate.is_valid || !candidate.trajectory.is_valid())
            {
                continue;
            }
            const auto validation = validate_remaining(
                candidate.trajectory, 0.0, input, true);
            if (!validation.is_safe)
            {
                continue;
            }

            result.is_valid = true;
            result.action = PeerDeconflictionAction::ESCAPE;
            result.escape_target = target;
            result.plan = candidate;
            result.reason = "deterministic lateral peer escape";
            return result;
        }

        result.action = PeerDeconflictionAction::BLOCKED;
        result.reason = "no recovery path satisfies peer and obstacle clearance";
        return result;
    }

    /**
     * @brief 驗證 recovery trajectory 的剩餘時間段。
     *
     * `start_relative_time_s` 之前的樣本不再重播；這讓 Node 可以在實機
     * tracking 有小幅誤差時驗證目前即時位置到 trajectory 結束的部分。
     * 對已違規 peer 要求距離不惡化並在結束前達到 release distance；對
     * 原本安全的 peer 則全程不得低於 hard distance。
     */
    PeerDeconflictionValidationResult validate_remaining(
        const BsplineTrajectory& trajectory,
        const double start_relative_time_s,
        const PeerDeconflictionInput& input,
        const bool require_release) const
    {
        PeerDeconflictionValidationResult result;
        if (!is_valid_input(input) || !trajectory.is_valid() ||
            !std::isfinite(start_relative_time_s) ||
            start_relative_time_s < 0.0 ||
            start_relative_time_s > trajectory.duration_s() + EGO_SWARM_EPSILON_M)
        {
            result.reason = "invalid recovery validation input";
            return result;
        }

        const double release_distance = release_distance_m();
        result.minimum_peer_distance_m = input.peers.empty()
            ? 0.0 : std::numeric_limits<double>::infinity();
        result.minimum_obstacle_distance_m =
            input.obstacle_map->point_count() == 0U
            ? 0.0 : std::numeric_limits<double>::infinity();

        const double start_time = std::min(
            start_relative_time_s, trajectory.duration_s());
        const std::size_t count = sample_count(
            trajectory.duration_s() - start_time);
        std::vector<double> initial_distances(input.peers.size(), 0.0);
        for (std::size_t peer_index = 0U;
             peer_index < input.peers.size(); ++peer_index)
        {
            initial_distances.at(peer_index) = distance_between_centers(
                input.self_position, input.peers.at(peer_index).position);
        }

        for (std::size_t sample_index = 0U;
             sample_index <= count; ++sample_index)
        {
            const double relative_time = std::min(
                trajectory.duration_s(), start_time +
                static_cast<double>(sample_index) * config_.sample_interval_s);
            const TrajectoryState sample = trajectory.evaluate_state(relative_time);
            if (!sample.is_valid || !is_dynamically_feasible(
                    sample, input.planner_config))
            {
                result.reason = "recovery trajectory violates dynamics";
                return result;
            }

            const bool empty_obstacle_map =
                input.obstacle_map->point_count() == 0U;
            const double obstacle_distance = empty_obstacle_map
                ? 0.0 : input.obstacle_map->nearest_distance(sample.position);
            if ((!empty_obstacle_map && !std::isfinite(obstacle_distance)) ||
                (!empty_obstacle_map && !input.obstacle_map->is_position_safe(
                    sample.position, input.planner_config.obstacle_clearance_m)))
            {
                result.reason = "recovery trajectory violates obstacle clearance";
                return result;
            }
            result.minimum_obstacle_distance_m = std::min(
                result.minimum_obstacle_distance_m, obstacle_distance);

            for (std::size_t peer_index = 0U;
                 peer_index < input.peers.size(); ++peer_index)
            {
                const auto& peer = input.peers.at(peer_index);
                const double distance = distance_between_centers(
                    sample.position, peer.position);
                if (!std::isfinite(distance))
                {
                    result.reason = "non-finite peer distance";
                    return result;
                }
                result.minimum_peer_distance_m = std::min(
                    result.minimum_peer_distance_m, distance);
                const double initial_distance = initial_distances.at(peer_index);
                if (initial_distance < config_.hard_distance_m)
                {
                    if (distance + config_.progress_tolerance_m <
                            initial_distance)
                    {
                        result.reason = "recovery distance is worsening";
                        return result;
                    }
                }
                else if (distance + EGO_SWARM_EPSILON_M <
                         config_.hard_distance_m)
                {
                    result.reason = "recovery creates a new peer conflict";
                    return result;
                }

                const bool endpoint = sample_index == count ||
                    relative_time >= trajectory.duration_s() -
                        EGO_SWARM_EPSILON_M;
                if (endpoint && require_release &&
                    initial_distance < config_.hard_distance_m &&
                    distance + EGO_SWARM_EPSILON_M < release_distance)
                {
                    result.reason = "recovery does not reach release distance";
                    return result;
                }
            }
        }

        result.is_safe = true;
        result.reason = "recovery trajectory accepted";
        return result;
    }

    double release_distance_m() const
    {
        return config_.hard_distance_m + config_.release_margin_m;
    }

private:
    static double vector_norm(const Vector3& value)
    {
        return std::sqrt(squared_norm(value));
    }

    static double dot_product(const Vector3& first, const Vector3& second)
    {
        return first.at(0) * second.at(0) +
            first.at(1) * second.at(1) + first.at(2) * second.at(2);
    }

    static Vector3 horizontal(const Vector3& value)
    {
        return Vector3{value.at(0), value.at(1), 0.0};
    }

    static bool normalize_horizontal(const Vector3& value, Vector3& output)
    {
        const Vector3 flattened = horizontal(value);
        const double norm = vector_norm(flattened);
        if (!std::isfinite(norm) || norm <= EGO_SWARM_EPSILON_M)
        {
            return false;
        }
        output = scale_vector(flattened, 1.0 / norm);
        return is_finite_vector(output);
    }

    bool is_valid_input(const PeerDeconflictionInput& input) const
    {
        if (input.self_agent_id <= 0 || !is_finite_vector(input.self_position) ||
            !is_finite_vector(input.final_target) || !std::isfinite(input.now_s) ||
            !input.peers_are_fresh || input.obstacle_map == nullptr ||
            !input.obstacle_map->is_valid() ||
            !input.obstacle_map->is_fresh(input.now_s) ||
            !is_valid_planner_config(input.planner_config))
        {
            return false;
        }
        for (const auto& peer : input.peers)
        {
            if (peer.agent_id <= 0 || peer.agent_id == input.self_agent_id ||
                !is_finite_vector(peer.position) ||
                (peer.velocity_valid && !is_finite_vector(peer.velocity)))
            {
                return false;
            }
        }
        return true;
    }

    static bool is_valid_planner_config(const PlannerConfig& config)
    {
        return std::isfinite(config.obstacle_clearance_m) &&
            config.obstacle_clearance_m >= 0.0 &&
            std::isfinite(config.max_velocity_mps) &&
            config.max_velocity_mps > EGO_SWARM_EPSILON_M &&
            std::isfinite(config.max_acceleration_mps2) &&
            config.max_acceleration_mps2 > EGO_SWARM_EPSILON_M &&
            std::isfinite(config.max_jerk_mps3) &&
            config.max_jerk_mps3 > EGO_SWARM_EPSILON_M &&
            std::isfinite(config.knot_interval_s) &&
            config.knot_interval_s > EGO_SWARM_EPSILON_M;
    }

    bool build_self_component(
        const PeerDeconflictionInput& input,
        std::vector<int>& component_peer_indices) const
    {
        component_peer_indices.clear();
        std::vector<bool> visited(input.peers.size(), false);
        std::vector<int> frontier;
        for (std::size_t index = 0U; index < input.peers.size(); ++index)
        {
            if (distance_between_centers(
                    input.self_position, input.peers.at(index).position) <
                config_.hard_distance_m)
            {
                frontier.push_back(static_cast<int>(index));
                visited.at(index) = true;
            }
        }
        if (frontier.empty())
        {
            return false;
        }

        while (!frontier.empty())
        {
            const int current = frontier.back();
            frontier.pop_back();
            component_peer_indices.push_back(current);
            for (std::size_t index = 0U; index < input.peers.size(); ++index)
            {
                if (visited.at(index))
                {
                    continue;
                }
                if (distance_between_centers(
                        input.peers.at(static_cast<std::size_t>(current)).position,
                        input.peers.at(index).position) <
                    config_.hard_distance_m)
                {
                    visited.at(index) = true;
                    frontier.push_back(static_cast<int>(index));
                }
            }
        }
        std::sort(
            component_peer_indices.begin(), component_peer_indices.end(),
            [&input](const int first, const int second)
            {
                return input.peers.at(static_cast<std::size_t>(first)).agent_id <
                    input.peers.at(static_cast<std::size_t>(second)).agent_id;
            });
        return true;
    }

    std::vector<Vector3> build_directions(
        const PeerDeconflictionInput& input) const
    {
        std::vector<Vector3> directions;
        const auto append = [&directions](const Vector3& candidate)
            {
                Vector3 normalized{};
                if (!normalize_horizontal(candidate, normalized))
                {
                    return;
                }
                for (const Vector3& existing : directions)
                {
                    if (distance_between_centers(existing, normalized) < 1.0e-6)
                    {
                        return;
                    }
                }
                directions.push_back(normalized);
            };

        Vector3 away{0.0, 0.0, 0.0};
        for (const auto& peer : input.peers)
        {
            const Vector3 delta = subtract_vectors(input.self_position, peer.position);
            const double distance = vector_norm(delta);
            if (std::isfinite(distance) && distance > EGO_SWARM_EPSILON_M)
            {
                away = add_vectors(
                    away, scale_vector(horizontal(delta), 1.0 / distance));
            }
        }
        append(away);

        Vector3 task_direction{};
        if (normalize_horizontal(
                subtract_vectors(input.final_target, input.self_position),
                task_direction))
        {
            append(Vector3{-task_direction.at(1), task_direction.at(0), 0.0});
            append(Vector3{task_direction.at(1), -task_direction.at(0), 0.0});
        }

        const int rotation = std::abs(input.self_agent_id) % 4;
        const std::vector<Vector3> cardinal{
            Vector3{1.0, 0.0, 0.0}, Vector3{0.0, 1.0, 0.0},
            Vector3{-1.0, 0.0, 0.0}, Vector3{0.0, -1.0, 0.0}};
        for (int index = 0; index < 4; ++index)
        {
            append(cardinal.at(static_cast<std::size_t>((rotation + index) % 4)));
        }
        return directions;
    }

    Vector3 target_on_ray(
        const PeerDeconflictionInput& input,
        const Vector3& direction) const
    {
        double distance = std::max(
            release_distance_m(), config_.hard_distance_m + 0.5);
        for (const auto& peer : input.peers)
        {
            const Vector3 delta = subtract_vectors(
                input.self_position, peer.position);
            const double b = dot_product(delta, direction);
            const double c = dot_product(delta, delta) -
                release_distance_m() * release_distance_m();
            if (c < 0.0)
            {
                const double discriminant = b * b - c;
                if (std::isfinite(discriminant) && discriminant >= 0.0)
                {
                    distance = std::max(
                        distance, -b + std::sqrt(discriminant) + 0.2);
                }
            }
        }
        distance = std::min(distance, config_.max_escape_distance_m);
        return add_vectors(
            input.self_position,
            Vector3{direction.at(0) * distance,
                    direction.at(1) * distance, 0.0});
    }

    PlannerResult build_escape_plan(
        const PeerDeconflictionInput& input,
        const Vector3& target) const
    {
        PlannerResult result;
        const double distance = distance_between_centers(
            input.self_position, target);
        if (!std::isfinite(distance) || distance <= EGO_SWARM_EPSILON_M)
        {
            return result;
        }

        const double velocity_time = 1.5 * distance /
            input.planner_config.max_velocity_mps;
        const double acceleration_time = std::sqrt(
            6.0 * distance / input.planner_config.max_acceleration_mps2);
        const double jerk_time = std::cbrt(
            12.0 * distance / input.planner_config.max_jerk_mps3);
        const double duration = 1.2 * std::max(
            input.planner_config.knot_interval_s,
            std::max(velocity_time, std::max(acceleration_time, jerk_time)));
        if (!std::isfinite(duration) || duration <= EGO_SWARM_EPSILON_M)
        {
            return result;
        }

        const std::vector<Vector3> control_points{
            input.self_position, input.self_position, target, target};
        result.trajectory = BsplineTrajectory(control_points, duration);
        result.is_valid = result.trajectory.is_valid();
        result.is_hold = false;
        result.trajectory_start_time_s = input.now_s;
        result.termination = result.is_valid
            ? PlannerTerminationReason::SUCCESS
            : PlannerTerminationReason::INFEASIBLE;
        return result;
    }

    static bool is_dynamically_feasible(
        const TrajectoryState& state,
        const PlannerConfig& planner_config)
    {
        const double velocity = vector_norm(state.velocity);
        const double acceleration = vector_norm(state.acceleration);
        const double jerk = vector_norm(state.jerk);
        return std::isfinite(velocity) && std::isfinite(acceleration) &&
            std::isfinite(jerk) &&
            velocity <= planner_config.max_velocity_mps + EGO_SWARM_EPSILON_M &&
            acceleration <= planner_config.max_acceleration_mps2 +
                EGO_SWARM_EPSILON_M &&
            jerk <= planner_config.max_jerk_mps3 + EGO_SWARM_EPSILON_M;
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

    PeerDeconflictionConfig config_{};
};

}  // namespace ego_swarm
}  // namespace taiphoon
