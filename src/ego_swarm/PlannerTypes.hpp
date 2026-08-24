/**
 * @file PlannerTypes.hpp
 * @brief EGO-Swarm planner backend 共用資料型別與 planning snapshot
 * @author Taiphoon Agent Team
 * @date 2026-08-14
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ego_swarm/PeerTrajectoryTable.hpp"
#include "perception/IObstacleMap.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// 可替換的 planner backend 類型。
enum class PlannerBackendKind
{
    DETERMINISTIC = 0,
    OPTIMIZED
};

/// planner backend 的結束原因，供 fallback 與測試使用。
enum class PlannerTerminationReason
{
    SUCCESS = 0,
    INVALID_INPUT,
    NO_FEASIBLE_PATH,
    NONFINITE_GRADIENT,
    ITERATION_LIMIT,
    TIMEOUT,
    DIVERGED,
    INFEASIBLE
};

inline std::string planner_termination_reason_string(
    const PlannerTerminationReason reason)
{
    switch (reason)
    {
        case PlannerTerminationReason::SUCCESS:
            return "success";
        case PlannerTerminationReason::INVALID_INPUT:
            return "invalid input";
        case PlannerTerminationReason::NO_FEASIBLE_PATH:
            return "no feasible path";
        case PlannerTerminationReason::NONFINITE_GRADIENT:
            return "nonfinite gradient";
        case PlannerTerminationReason::ITERATION_LIMIT:
            return "iteration limit";
        case PlannerTerminationReason::TIMEOUT:
            return "timeout";
        case PlannerTerminationReason::DIVERGED:
            return "diverged";
        case PlannerTerminationReason::INFEASIBLE:
            return "infeasible";
    }
    return "unknown";
}

/// 由 planner snapshot 強制的路徑高度約束。
enum class PathConstraint
{
    FREE_3D = 0,
    FIXED_Z,
    LINEAR_Z
};

/// EGO-Swarm planner 與 safety validator 共用設定。
struct PlannerConfig
{
    // Local Goal feature gate 也控制 FIXED_Z route initializer；由 Node
    // 在新路徑流程驗收後啟用。
    bool enable_spatiotemporal_route_planning{false};
    double knot_interval_s{1.0};
    std::size_t control_point_count{10U};
    double sample_interval_s{0.1};
    // 長距離水平繞行需要保留 tracking 與 lateral detour 的動力學餘裕。
    // 這些仍是 planner／validator 的 hard upper bounds，不代表命令會
    // 強制以最大速度飛行。
    double max_velocity_mps{2.5};
    double max_acceleration_mps2{1.5};
    double max_jerk_mps3{3.0};
    // 舊參數名稱保留，讓既有 launch / 純邏輯呼叫端可以平滑遷移。
    double max_velocity_m_s{2.5};
    double max_acceleration_m_s2{1.5};
    double candidate_offset_m{3.0};
    // optimized backend 的內部水平搜尋範圍；不作為 ROS2 對外參數。
    // 以較大的 envelope 保留大膽側向繞行空間，但仍受 hard validator
    // 與 dynamic feasibility 共同限制。
    double optimizer_search_radius_m{10.0};
    double optimizer_peer_soft_margin_m{1.0};
    double optimizer_obstacle_soft_margin_m{0.5};
    double obstacle_clearance_m{1.0};
    double peer_collision_min_distance_m{3.0};
    double goal_weight{1.0};
    double smoothness_weight{0.1};
    double dynamic_weight{10.0};
    double obstacle_weight{10.0};
    double peer_weight{10.0};
    int optimizer_max_iterations{40};
    double optimizer_time_budget_ms{50.0};
    double optimizer_gradient_epsilon_m{0.05};
    double optimizer_initial_step_m{0.20};
    double optimizer_minimum_step_m{0.01};
    double optimizer_convergence_tolerance{1.0e-3};
    double formation_crossing_penalty_m{10.0};
    double reservation_lead_time_s{0.5};
    double reservation_step_s{0.5};
    double reservation_horizon_s{5.0};
    // Planner 內部為 PX4 setpoint tracking、狀態同步與取樣誤差保留的
    // 機間距離裕度；ContinuousSafetyValidator 仍只使用
    // peer_collision_min_distance_m 作為對外安全契約。
    double peer_planning_margin_m{1.0};
};

/**
 * @brief 取得 optimizer 的機間 soft tracking barrier 距離
 *
 * 這個距離只用於成本函式，不會取代 planner hard check 或最後的
 * ContinuousSafetyValidator 門檻。
 */
inline double peer_planning_soft_clearance_m(const PlannerConfig& config)
{
    return config.peer_collision_min_distance_m +
        config.peer_planning_margin_m;
}

/// 單一 planner cycle 使用的 immutable 輸入資料。
struct PlanningSnapshot
{
    VehiclePlanningState self_state{};
    Vector3 local_goal{0.0, 0.0, 0.0};
    double planning_time_s{0.0};
    double trajectory_start_time_s{0.0};
    uint32_t frame_revision{0U};
    int self_agent_id{0};
    PathConstraint path_constraint{PathConstraint::FREE_3D};
    double fixed_z_m{0.0};
    std::vector<PeerTrajectory> peers{};
    const perception::IObstacleMap* obstacle_map{nullptr};
    bool peers_are_fresh{false};

    /**
     * @brief 檢查 snapshot 是否具備 planner 所需的基本資料。
     * @return true 表示可交給 backend。
     */
    bool is_valid() const
    {
        return self_state.is_finite() &&
               is_finite_vector(local_goal) &&
               is_finite(planning_time_s) &&
               is_finite(trajectory_start_time_s) &&
               frame_revision != 0U &&
               self_agent_id > 0 &&
               is_finite(fixed_z_m) &&
               obstacle_map != nullptr &&
               obstacle_map->is_valid() &&
               obstacle_map->is_fresh(planning_time_s) &&
               peers_are_fresh;
    }
};

/// planner backend 的輸出與診斷資訊。
struct PlannerResult
{
    bool is_valid{false};
    bool is_hold{true};
    BsplineTrajectory trajectory{};
    double cost{0.0};
    double minimum_obstacle_distance_m{0.0};
    double minimum_peer_distance_m{0.0};
    double goal_cost{0.0};
    double smoothness_cost{0.0};
    double dynamic_cost{0.0};
    double obstacle_cost{0.0};
    double peer_cost{0.0};
    double trajectory_start_time_s{0.0};
    double planning_time_ms{0.0};
    std::size_t iteration_count{0U};
    bool route_search_used{false};
    std::size_t route_expanded_nodes{0U};
    double route_search_time_ms{0.0};
    PlannerBackendKind backend{PlannerBackendKind::DETERMINISTIC};
    PlannerTerminationReason termination{PlannerTerminationReason::INVALID_INPUT};

    /**
     * @brief 檢查規劃結果中所有可輸出的數值。
     * @return true 表示成本、距離、時間與控制點皆為有限值。
     */
    bool has_finite_output() const
    {
        if (!is_finite(cost) ||
            !is_finite(minimum_obstacle_distance_m) ||
            !is_finite(minimum_peer_distance_m) ||
            !is_finite(goal_cost) ||
            !is_finite(smoothness_cost) ||
            !is_finite(dynamic_cost) ||
            !is_finite(obstacle_cost) ||
            !is_finite(peer_cost) ||
            !is_finite(trajectory_start_time_s) ||
            !is_finite(planning_time_ms) ||
            !is_finite(route_search_time_ms))
        {
            return false;
        }

        for (const Vector3& control_point : trajectory.control_points())
        {
            if (!is_finite_vector(control_point))
            {
                return false;
            }
        }
        return true;
    }
};

}  // namespace ego_swarm
}  // namespace taiphoon
