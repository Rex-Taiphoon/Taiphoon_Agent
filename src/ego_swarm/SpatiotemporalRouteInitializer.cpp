/**
 * @file SpatiotemporalRouteInitializer.cpp
 * @brief FIXED_Z 時空路徑 seed 的 bounded A* 實作
 */

#include "ego_swarm/SpatiotemporalRouteInitializer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <queue>
#include <unordered_map>

#include "ego_swarm/TrajectoryPriority.hpp"

namespace taiphoon
{
namespace ego_swarm
{

namespace
{

constexpr double ROUTE_GRID_RESOLUTION_M = 0.5;
constexpr double ROUTE_BOUND_EPSILON_M = 1.0e-6;
constexpr double ROUTE_SPEED_MARGIN = 0.80;
constexpr double ROUTE_MAX_TIME_MULTIPLIER = 2.0;
constexpr double ROUTE_EXTRA_TIME_S = 2.0;
constexpr double ROUTE_OBSTACLE_CACHE_CELL_M = 2.0;
constexpr std::size_t ROUTE_MAX_EXPANDED_NODES = 100000U;
// Route seed 的 nominal speed 與 10 點 planner 共用原本 6 點 cubic
// B-spline 的 3 spans timing reference。
constexpr double LEGACY_TIMING_SPAN_COUNT = 3.0;

struct GridState
{
    int x{0};
    int y{0};
    std::size_t time_index{0U};

    bool operator==(const GridState& other) const
    {
        return x == other.x && y == other.y &&
               time_index == other.time_index;
    }
};

struct GridStateHash
{
    std::size_t operator()(const GridState& state) const
    {
        std::size_t hash = static_cast<std::size_t>(state.x) *
            static_cast<std::size_t>(0x9e3779b1U);
        hash ^= static_cast<std::size_t>(state.y) +
            static_cast<std::size_t>(0x9e3779b9U) +
            (hash << 6U) + (hash >> 2U);
        hash ^= state.time_index + static_cast<std::size_t>(0x85ebca6bU) +
            (hash << 6U) + (hash >> 2U);
        return hash;
    }
};

struct ObstacleCacheKey
{
    std::uint64_t x_bits{0U};
    std::uint64_t y_bits{0U};
    std::uint64_t z_bits{0U};

    bool operator==(const ObstacleCacheKey& other) const
    {
        return x_bits == other.x_bits && y_bits == other.y_bits &&
               z_bits == other.z_bits;
    }
};

struct ObstacleCacheKeyHash
{
    std::size_t operator()(const ObstacleCacheKey& key) const
    {
        std::size_t hash = static_cast<std::size_t>(key.x_bits);
        hash ^= static_cast<std::size_t>(key.y_bits) +
            static_cast<std::size_t>(0x9e3779b9U) +
            (hash << 6U) + (hash >> 2U);
        hash ^= static_cast<std::size_t>(key.z_bits) +
            static_cast<std::size_t>(0x85ebca6bU) +
            (hash << 6U) + (hash >> 2U);
        return hash;
    }
};

struct ObstacleCellKey
{
    int x{0};
    int y{0};

    bool operator==(const ObstacleCellKey& other) const
    {
        return x == other.x && y == other.y;
    }
};

struct ObstacleCellKeyHash
{
    std::size_t operator()(const ObstacleCellKey& key) const
    {
        std::size_t hash = static_cast<std::size_t>(key.x);
        hash ^= static_cast<std::size_t>(key.y) +
            static_cast<std::size_t>(0x9e3779b9U) +
            (hash << 6U) + (hash >> 2U);
        return hash;
    }
};

ObstacleCacheKey make_obstacle_cache_key(const Vector3& position)
{
    ObstacleCacheKey key;
    std::memcpy(&key.x_bits, &position.at(0), sizeof(key.x_bits));
    std::memcpy(&key.y_bits, &position.at(1), sizeof(key.y_bits));
    std::memcpy(&key.z_bits, &position.at(2), sizeof(key.z_bits));
    return key;
}

struct SearchRecord
{
    GridState state{};
    Vector3 position{0.0, 0.0, 0.0};
    double cost_s{0.0};
    double path_length_m{0.0};
    double start_delay_s{0.0};
    std::size_t parent_index{std::numeric_limits<std::size_t>::max()};
};

struct OpenEntry
{
    std::size_t record_index{0U};
    double estimated_cost_s{0.0};
    double cost_s{0.0};
    double path_length_m{0.0};
};

struct OpenEntryGreater
{
    bool operator()(const OpenEntry& left, const OpenEntry& right) const
    {
        if (std::abs(left.estimated_cost_s - right.estimated_cost_s) >
            EGO_SWARM_EPSILON_M)
        {
            return left.estimated_cost_s > right.estimated_cost_s;
        }
        if (std::abs(left.cost_s - right.cost_s) > EGO_SWARM_EPSILON_M)
        {
            return left.cost_s > right.cost_s;
        }
        if (std::abs(left.path_length_m - right.path_length_m) >
            EGO_SWARM_EPSILON_M)
        {
            return left.path_length_m > right.path_length_m;
        }
        return left.record_index > right.record_index;
    }
};

struct SearchBounds
{
    int min_x{0};
    int max_x{0};
    int min_y{0};
    int max_y{0};
};

struct Neighbor
{
    int dx{0};
    int dy{0};
};

constexpr Neighbor NEIGHBORS[] = {
    {1, 0}, {0, 1}, {-1, 0}, {0, -1},
    {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

double horizontal_distance(const Vector3& first, const Vector3& second)
{
    return std::hypot(
        first.at(0) - second.at(0), first.at(1) - second.at(1));
}

Vector3 interpolate_position(
    const Vector3& start,
    const Vector3& goal,
    const double ratio,
    const double fixed_z)
{
    return Vector3{
        start.at(0) + (goal.at(0) - start.at(0)) * ratio,
        start.at(1) + (goal.at(1) - start.at(1)) * ratio,
        fixed_z};
}

double estimate_nominal_speed(
    const PlannerConfig& config,
    const double distance_m,
    const std::size_t control_point_count)
{
    if (!is_finite(distance_m) || distance_m <= EGO_SWARM_EPSILON_M ||
        !is_finite(config.max_velocity_mps) ||
        !is_finite(config.max_acceleration_mps2) ||
        config.max_velocity_mps <= EGO_SWARM_EPSILON_M ||
        config.max_acceleration_mps2 <= EGO_SWARM_EPSILON_M ||
        control_point_count <= BSPLINE_DEGREE)
    {
        return 0.0;
    }

    const double span_count = LEGACY_TIMING_SPAN_COUNT;
    const double velocity_duration = distance_m / config.max_velocity_mps;
    const double acceleration_duration = 2.0 * std::sqrt(
        distance_m / config.max_acceleration_mps2);
    const double base_duration = std::max(
        span_count * config.knot_interval_s,
        std::max(velocity_duration, acceleration_duration));
    const double duration = std::max(
        1.5 * base_duration,
        1.0 * distance_m / config.max_velocity_mps);
    if (!is_finite(duration) || duration <= EGO_SWARM_EPSILON_M)
    {
        return 0.0;
    }
    return std::max(
        EGO_SWARM_EPSILON_M,
        distance_m / duration * ROUTE_SPEED_MARGIN);
}

SearchBounds make_bounds(
    const Vector3& start,
    const Vector3& goal,
    const double radius_m)
{
    const double min_x = std::min(start.at(0), goal.at(0)) - radius_m;
    const double max_x = std::max(start.at(0), goal.at(0)) + radius_m;
    const double min_y = std::min(start.at(1), goal.at(1)) - radius_m;
    const double max_y = std::max(start.at(1), goal.at(1)) + radius_m;
    return SearchBounds{
        static_cast<int>(std::floor((min_x - start.at(0)) /
            ROUTE_GRID_RESOLUTION_M)),
        static_cast<int>(std::ceil((max_x - start.at(0)) /
            ROUTE_GRID_RESOLUTION_M)),
        static_cast<int>(std::floor((min_y - start.at(1)) /
            ROUTE_GRID_RESOLUTION_M)),
        static_cast<int>(std::ceil((max_y - start.at(1)) /
            ROUTE_GRID_RESOLUTION_M))};
}

bool in_bounds(const GridState& state, const SearchBounds& bounds)
{
    return state.x >= bounds.min_x && state.x <= bounds.max_x &&
           state.y >= bounds.min_y && state.y <= bounds.max_y;
}

GridState search_key(const GridState& state, const bool time_sensitive)
{
    return time_sensitive
        ? state
        : GridState{state.x, state.y, 0U};
}

Vector3 grid_position(
    const Vector3& start,
    const Vector3& goal,
    const GridState& state,
    const double fixed_z)
{
    const Vector3 position{
        start.at(0) + static_cast<double>(state.x) *
            ROUTE_GRID_RESOLUTION_M,
        start.at(1) + static_cast<double>(state.y) *
            ROUTE_GRID_RESOLUTION_M,
        fixed_z};
    const int goal_x = static_cast<int>(std::llround(
        (goal.at(0) - start.at(0)) / ROUTE_GRID_RESOLUTION_M));
    const int goal_y = static_cast<int>(std::llround(
        (goal.at(1) - start.at(1)) / ROUTE_GRID_RESOLUTION_M));
    if (state.x == goal_x && state.y == goal_y)
    {
        return Vector3{goal.at(0), goal.at(1), fixed_z};
    }
    return position;
}

bool is_peer_position_safe(
    const Vector3& position,
    const double absolute_time_s,
    const double candidate_start_time_s,
    const PlanningSnapshot& snapshot,
    const PlannerConfig& config)
{
    for (const PeerTrajectory& peer : snapshot.peers)
    {
        if (peer.agent_id == snapshot.self_agent_id)
        {
            continue;
        }
        if (!peer.terminal_hold &&
            absolute_time_s > peer.end_time_s + EGO_SWARM_EPSILON_M)
        {
            continue;
        }
        if (!should_check_peer_occupancy(
                peer, absolute_time_s, candidate_start_time_s,
                snapshot.self_agent_id))
        {
            continue;
        }
        const double peer_time_s = std::max(
            0.0, absolute_time_s - peer.start_time_s);
        const TrajectoryState peer_state = peer.trajectory.evaluate_state(
            peer_time_s);
        if (!peer_state.is_valid)
        {
            return false;
        }
        const double distance = distance_between_centers(
            position, peer_state.position);
        if (!is_finite(distance) ||
            distance + EGO_SWARM_EPSILON_M <
                config.peer_collision_min_distance_m)
        {
            return false;
        }
    }
    return true;
}

bool is_position_safe_at_time(
    const Vector3& position,
    const double relative_time_s,
    const double candidate_start_time_s,
    const PlanningSnapshot& snapshot,
    const PlannerConfig& config)
{
    if (snapshot.obstacle_map == nullptr ||
        !snapshot.obstacle_map->is_position_safe(
            position, config.obstacle_clearance_m))
    {
        return false;
    }
    return is_peer_position_safe(
        position, candidate_start_time_s + relative_time_s,
        candidate_start_time_s, snapshot, config);
}

bool direct_seed_is_safe(
    const Vector3& start,
    const Vector3& goal,
    const double fixed_z,
    const PlanningSnapshot& snapshot,
    const PlannerConfig& config,
    const double nominal_speed)
{
    const double distance = horizontal_distance(start, goal);
    const double duration = distance / nominal_speed;
    if (!is_finite(distance) || !is_finite(duration))
    {
        return false;
    }
    const std::size_t count = std::max<std::size_t>(
        2U, std::min<std::size_t>(
            EGO_SWARM_MAX_SAMPLE_COUNT,
            static_cast<std::size_t>(std::ceil(
                distance / (ROUTE_GRID_RESOLUTION_M * 0.5)))));
    for (std::size_t index = 0U; index <= count; ++index)
    {
        const double ratio = static_cast<double>(index) /
            static_cast<double>(count);
        const Vector3 position = interpolate_position(
            start, goal, ratio, fixed_z);
        if (!is_position_safe_at_time(
                position, ratio * duration,
                snapshot.trajectory_start_time_s, snapshot, config))
        {
            return false;
        }
    }
    return true;
}

bool better_record(
    const SearchRecord& candidate,
    const SearchRecord& existing)
{
    if (candidate.cost_s < existing.cost_s - EGO_SWARM_EPSILON_M)
    {
        return true;
    }
    if (std::abs(candidate.cost_s - existing.cost_s) <=
            EGO_SWARM_EPSILON_M &&
        candidate.path_length_m < existing.path_length_m -
            EGO_SWARM_EPSILON_M)
    {
        return true;
    }
    return false;
}

}  // namespace

SpatiotemporalRouteResult SpatiotemporalRouteInitializer::initialize(
    const PlanningSnapshot& snapshot,
    const PlannerConfig& config,
    const std::vector<Vector3>& straight_seed) const
{
    SpatiotemporalRouteResult result;
    const auto started_at = std::chrono::steady_clock::now();
    const auto finish = [&result, &started_at](
        SpatiotemporalRouteResult output)
    {
        output.search_time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started_at).count();
        return output;
    };

    if (!snapshot.is_valid() || snapshot.path_constraint !=
            PathConstraint::FIXED_Z || straight_seed.size() <= BSPLINE_DEGREE ||
        !is_finite(config.max_velocity_mps) ||
        config.max_velocity_mps <= EGO_SWARM_EPSILON_M ||
        !is_finite(config.max_acceleration_mps2) ||
        config.max_acceleration_mps2 <= EGO_SWARM_EPSILON_M ||
        !is_finite(config.optimizer_search_radius_m) ||
        config.optimizer_search_radius_m <= EGO_SWARM_EPSILON_M ||
        !is_finite(config.sample_interval_s) ||
        config.sample_interval_s <= EGO_SWARM_EPSILON_M ||
        !is_finite(config.reservation_step_s) ||
        config.reservation_step_s <= EGO_SWARM_EPSILON_M ||
        !is_finite(config.reservation_horizon_s) ||
        config.reservation_horizon_s < 0.0 ||
        !is_finite(config.obstacle_clearance_m) ||
        config.obstacle_clearance_m < 0.0 ||
        !is_finite(config.peer_collision_min_distance_m) ||
        config.peer_collision_min_distance_m < 0.0 ||
        snapshot.obstacle_map == nullptr ||
        !snapshot.obstacle_map->is_valid() ||
        !snapshot.obstacle_map->is_fresh(snapshot.planning_time_s))
    {
        return finish(result);
    }

    for (const Vector3& point : straight_seed)
    {
        if (!is_finite_vector(point))
        {
            return finish(result);
        }
    }

    Vector3 start = snapshot.self_state.position;
    Vector3 goal = snapshot.local_goal;
    start.at(2) = snapshot.fixed_z_m;
    goal.at(2) = snapshot.fixed_z_m;
    const double direct_distance = horizontal_distance(start, goal);
    if (!is_finite(direct_distance))
    {
        return finish(result);
    }
    if (direct_distance <= EGO_SWARM_EPSILON_M)
    {
        result.is_valid = true;
        result.termination = RouteInitializerTerminationReason::SUCCESS;
        result.trajectory_start_time_s = snapshot.trajectory_start_time_s;
        return finish(result);
    }

    const double nominal_speed = estimate_nominal_speed(
        config, direct_distance, straight_seed.size());
    if (!is_finite(nominal_speed) || nominal_speed <= EGO_SWARM_EPSILON_M)
    {
        return finish(result);
    }

    if (direct_seed_is_safe(
            start, goal, snapshot.fixed_z_m, snapshot, config, nominal_speed))
    {
        result.is_valid = true;
        result.used_search = false;
        result.trajectory_start_time_s = snapshot.trajectory_start_time_s;
        result.duration_s = direct_distance / nominal_speed;
        result.path_length_m = direct_distance;
        result.termination = RouteInitializerTerminationReason::SUCCESS;
        return finish(result);
    }

    const double time_step_s = config.sample_interval_s;
    const double direct_duration_s = direct_distance / nominal_speed;
    const double maximum_relative_time_s = std::max(
        direct_duration_s * ROUTE_MAX_TIME_MULTIPLIER,
        direct_duration_s + config.reservation_horizon_s) +
        ROUTE_EXTRA_TIME_S;
    const std::size_t maximum_time_index = static_cast<std::size_t>(
        std::ceil(maximum_relative_time_s / time_step_s));
    const SearchBounds bounds = make_bounds(
        start, goal, config.optimizer_search_radius_m);
    const int goal_x = static_cast<int>(std::llround(
        (goal.at(0) - start.at(0)) / ROUTE_GRID_RESOLUTION_M));
    const int goal_y = static_cast<int>(std::llround(
        (goal.at(1) - start.at(1)) / ROUTE_GRID_RESOLUTION_M));
    const std::size_t delay_steps = snapshot.peers.empty()
        ? 0U
        : static_cast<std::size_t>(std::ceil(
            config.reservation_horizon_s / config.reservation_step_s));
    const bool time_sensitive = !snapshot.peers.empty();

    std::vector<SearchRecord> records;
    records.reserve(4096U);
    std::unordered_map<GridState, std::size_t, GridStateHash> best_records;
    std::unordered_map<ObstacleCacheKey, bool, ObstacleCacheKeyHash>
        obstacle_safety_cache;
    std::unordered_map<ObstacleCellKey, int, ObstacleCellKeyHash>
        obstacle_cell_cache;
    std::unordered_map<std::uint64_t, std::vector<TrajectoryState>>
        peer_state_cache;
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryGreater> open;

    const auto is_obstacle_position_safe = [&]()
    {
        return [&snapshot, &config, &obstacle_safety_cache,
                &obstacle_cell_cache](
            const Vector3& position)
        {
            const double cell_size = ROUTE_OBSTACLE_CACHE_CELL_M;
            const ObstacleCellKey cell{
                static_cast<int>(std::floor(position.at(0) / cell_size)),
                static_cast<int>(std::floor(position.at(1) / cell_size))};
            const auto cell_cached = obstacle_cell_cache.find(cell);
            if (cell_cached != obstacle_cell_cache.end())
            {
                if (cell_cached->second == 1)
                {
                    return true;
                }
                if (cell_cached->second == 2)
                {
                    return false;
                }
            }
            else
            {
                const Vector3 cell_center{
                    (static_cast<double>(cell.x) + 0.5) * cell_size,
                    (static_cast<double>(cell.y) + 0.5) * cell_size,
                    position.at(2)};
                const double center_distance =
                    snapshot.obstacle_map->nearest_distance(cell_center);
                const double cell_radius = std::sqrt(2.0) * cell_size * 0.5;
                if (std::isfinite(center_distance) &&
                    center_distance > config.obstacle_clearance_m +
                        cell_radius + ROUTE_BOUND_EPSILON_M)
                {
                    obstacle_cell_cache.emplace(cell, 1);
                    return true;
                }
                if (std::isfinite(center_distance) &&
                    center_distance + cell_radius <
                        config.obstacle_clearance_m -
                        ROUTE_BOUND_EPSILON_M)
                {
                    obstacle_cell_cache.emplace(cell, 2);
                    return false;
                }
                obstacle_cell_cache.emplace(cell, 3);
            }
            const ObstacleCacheKey key = make_obstacle_cache_key(position);
            const auto cached = obstacle_safety_cache.find(key);
            if (cached != obstacle_safety_cache.end())
            {
                return cached->second;
            }
            const bool safe = snapshot.obstacle_map->is_position_safe(
                position, config.obstacle_clearance_m);
            obstacle_safety_cache.emplace(key, safe);
            return safe;
        };
    }();

    const auto is_peer_position_safe_cached = [&]()
    {
        return [&snapshot, &config, &peer_state_cache](
            const Vector3& position,
            const double absolute_time_s,
            const double candidate_start_time_s)
        {
            std::uint64_t time_key{0U};
            std::memcpy(&time_key, &absolute_time_s, sizeof(time_key));
            auto cached = peer_state_cache.find(time_key);
            if (cached == peer_state_cache.end())
            {
                std::vector<TrajectoryState> states(
                    snapshot.peers.size());
                for (std::size_t index = 0U;
                     index < snapshot.peers.size(); ++index)
                {
                    const PeerTrajectory& peer = snapshot.peers.at(index);
                    if (peer.agent_id == snapshot.self_agent_id ||
                        (!peer.terminal_hold &&
                         absolute_time_s > peer.end_time_s +
                             EGO_SWARM_EPSILON_M))
                    {
                        continue;
                    }
                    const double peer_time_s = std::max(
                        0.0, absolute_time_s - peer.start_time_s);
                    states.at(index) = peer.trajectory.evaluate_state(
                        peer_time_s);
                }
                cached = peer_state_cache.emplace(
                    time_key, std::move(states)).first;
            }
            for (std::size_t index = 0U;
                 index < snapshot.peers.size(); ++index)
            {
                const PeerTrajectory& peer = snapshot.peers.at(index);
                if (peer.agent_id == snapshot.self_agent_id ||
                    (!peer.terminal_hold &&
                     absolute_time_s > peer.end_time_s +
                         EGO_SWARM_EPSILON_M) ||
                    !should_check_peer_occupancy(
                        peer, absolute_time_s, candidate_start_time_s,
                        snapshot.self_agent_id))
                {
                    continue;
                }
                const TrajectoryState& peer_state = cached->second.at(index);
                if (!peer_state.is_valid)
                {
                    return false;
                }
                const double distance = distance_between_centers(
                    position, peer_state.position);
                if (!is_finite(distance) ||
                    distance + EGO_SWARM_EPSILON_M <
                        config.peer_collision_min_distance_m)
                {
                    return false;
                }
            }
            return true;
        };
    }();

    const GridState start_state{0, 0, 0U};
    for (std::size_t delay_index = 0U; delay_index <= delay_steps;
         ++delay_index)
    {
        const double delay_s = std::min(
            config.reservation_horizon_s,
            static_cast<double>(delay_index) * config.reservation_step_s);
        const std::size_t time_index = static_cast<std::size_t>(std::ceil(
            delay_s / time_step_s));
        if (time_index > maximum_time_index)
        {
            continue;
        }
        const double actual_delay_s = static_cast<double>(time_index) *
            time_step_s;
        const double candidate_start_time_s =
            snapshot.trajectory_start_time_s + actual_delay_s;
        if (!is_obstacle_position_safe(start) ||
            !is_peer_position_safe_cached(
                start, candidate_start_time_s, candidate_start_time_s))
        {
            continue;
        }
        const SearchRecord record{
            GridState{start_state.x, start_state.y, time_index},
            start, actual_delay_s, 0.0, actual_delay_s,
            std::numeric_limits<std::size_t>::max()};
        const std::size_t record_index = records.size();
        records.push_back(record);
        const auto existing = best_records.find(
            search_key(record.state, time_sensitive));
        if (existing == best_records.end() ||
            better_record(record, records.at(existing->second)))
        {
            best_records[search_key(record.state, time_sensitive)] =
                record_index;
            const double heuristic = direct_distance / nominal_speed;
            open.push(OpenEntry{
                record_index, record.cost_s + heuristic,
                record.cost_s, record.path_length_m});
        }
    }

    if (open.empty())
    {
        result.termination = RouteInitializerTerminationReason::NO_FEASIBLE_PATH;
        return finish(result);
    }

    std::size_t goal_record_index = std::numeric_limits<std::size_t>::max();
    bool timed_out = false;
    while (!open.empty())
    {
        const auto elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started_at).count();
        if (elapsed_ms >= config.optimizer_time_budget_ms ||
            records.size() >= ROUTE_MAX_EXPANDED_NODES)
        {
            timed_out = true;
            break;
        }

        const OpenEntry entry = open.top();
        open.pop();
        const auto current_best = best_records.find(
            search_key(records.at(entry.record_index).state, time_sensitive));
        if (current_best == best_records.end() ||
            current_best->second != entry.record_index)
        {
            continue;
        }
        // 新增鄰居可能造成 vector 重新配置；使用值副本，確保展開期間
        // current record 的欄位仍然有效。
        const SearchRecord current = records.at(entry.record_index);
        ++result.expanded_nodes;
        if (current.state.x == goal_x && current.state.y == goal_y)
        {
            goal_record_index = entry.record_index;
            break;
        }

        for (const Neighbor& neighbor : NEIGHBORS)
        {
            GridState next_state{
                current.state.x + neighbor.dx,
                current.state.y + neighbor.dy,
                current.state.time_index};
            if (!in_bounds(next_state, bounds))
            {
                continue;
            }
            const double edge_distance = ROUTE_GRID_RESOLUTION_M *
                std::hypot(
                    static_cast<double>(neighbor.dx),
                    static_cast<double>(neighbor.dy));
            const std::size_t edge_steps = std::max<std::size_t>(
                1U, static_cast<std::size_t>(std::ceil(
                    edge_distance / nominal_speed / time_step_s)));
            next_state.time_index = current.state.time_index + edge_steps;
            if (next_state.time_index > maximum_time_index)
            {
                continue;
            }
            const Vector3 next_position = grid_position(
                start, goal, next_state, snapshot.fixed_z_m);
            bool edge_safe = true;
            for (std::size_t step = 1U; step <= edge_steps; ++step)
            {
                const double ratio = static_cast<double>(step) /
                    static_cast<double>(edge_steps);
                const Vector3 sample_position = interpolate_position(
                    current.position, next_position, ratio,
                    snapshot.fixed_z_m);
                const double relative_time_s = (
                    static_cast<double>(current.state.time_index + step) *
                    time_step_s) - current.start_delay_s;
                const double candidate_start_time_s =
                    snapshot.trajectory_start_time_s + current.start_delay_s;
                if (!is_obstacle_position_safe(sample_position) ||
                    !is_peer_position_safe_cached(
                        sample_position,
                        candidate_start_time_s + relative_time_s,
                        candidate_start_time_s))
                {
                    edge_safe = false;
                    break;
                }
            }
            if (!edge_safe)
            {
                continue;
            }

            const SearchRecord candidate{
                next_state, next_position,
                static_cast<double>(next_state.time_index) * time_step_s,
                current.path_length_m + edge_distance,
                current.start_delay_s, entry.record_index};
            const auto existing = best_records.find(
                search_key(candidate.state, time_sensitive));
            if (existing != best_records.end() &&
                !better_record(candidate, records.at(existing->second)))
            {
                continue;
            }
            const std::size_t record_index = records.size();
            records.push_back(candidate);
            best_records[search_key(candidate.state, time_sensitive)] =
                record_index;
            const double remaining_distance = horizontal_distance(
                candidate.position, goal);
            open.push(OpenEntry{
                record_index,
                candidate.cost_s + remaining_distance / nominal_speed,
                candidate.cost_s, candidate.path_length_m});
        }
    }

    if (goal_record_index == std::numeric_limits<std::size_t>::max())
    {
        result.termination = timed_out
            ? RouteInitializerTerminationReason::TIMEOUT
            : RouteInitializerTerminationReason::NO_FEASIBLE_PATH;
        return finish(result);
    }

    std::vector<TimedRoutePoint> reverse_route;
    std::size_t current_index = goal_record_index;
    const double selected_delay_s = records.at(current_index).start_delay_s;
    while (current_index != std::numeric_limits<std::size_t>::max())
    {
        const SearchRecord& record = records.at(current_index);
        reverse_route.push_back(TimedRoutePoint{
            record.position,
            static_cast<double>(record.state.time_index) * time_step_s -
                selected_delay_s});
        current_index = record.parent_index;
    }
    std::reverse(reverse_route.begin(), reverse_route.end());
    if (reverse_route.size() < 2U ||
        reverse_route.front().time_s < -ROUTE_BOUND_EPSILON_M)
    {
        result.termination = RouteInitializerTerminationReason::NO_FEASIBLE_PATH;
        return finish(result);
    }
    reverse_route.front().position = start;
    reverse_route.front().time_s = 0.0;
    reverse_route.back().position = goal;

    result.is_valid = true;
    result.used_search = true;
    result.trajectory_start_time_s = snapshot.trajectory_start_time_s +
        selected_delay_s;
    result.duration_s = reverse_route.back().time_s;
    result.path_length_m = records.at(goal_record_index).path_length_m;
    result.route = std::move(reverse_route);
    result.termination = RouteInitializerTerminationReason::SUCCESS;
    return finish(result);
}

}  // namespace ego_swarm
}  // namespace taiphoon
