/**
 * @file EgoSwarmPlanner.hpp
 * @brief EGO-Swarm 風格 deterministic 局部候選軌跡規劃器
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * 本模組以目標、平滑性、PointCloud 障礙物 clearance 與同儕未來
 * 軌跡成本選擇候選 B-spline。它是可測試的 deterministic baseline，
 * 不宣稱已實作原始 EGO-Swarm 的完整非線性最佳化器。
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "ego_swarm/ITrajectoryPlanner.hpp"
#include "ego_swarm/TrajectoryFactory.hpp"
#include "ego_swarm/TrajectoryPriority.hpp"
#include "perception/IObstacleMap.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief EGO-Swarm 風格局部規劃深層模組
 *
 * 對外只需要目前位置、編隊槽位目標、局部障礙物地圖、同儕軌跡表與
 * 時間，即可取得一條 deterministic B-spline。完整梯度最佳化可在
 * build_candidates 與 evaluate_candidate 的內部 seam 後續替換。
 */
class EgoSwarmPlanner : public ITrajectoryPlanner
{
public:
    /**
     * @brief 使用設定建立規劃器
     *
     * @param config 規劃器設定；無效設定會退回預設值
     */
    explicit EgoSwarmPlanner(const PlannerConfig& config = PlannerConfig{})
    {
        if (!set_config(config))
        {
            config_ = PlannerConfig{};
        }
    }

    /**
     * @brief 更新規劃器設定
     *
     * @param config 新設定
     * @return true 表示設定有效並已套用
     */
    bool set_config(const PlannerConfig& config)
    {
        const PlannerConfig normalized_config = normalize_config(config);
        if (!is_valid_config(normalized_config))
        {
            return false;
        }
        config_ = normalized_config;
        return true;
    }

    /**
     * @brief 取得目前規劃器設定
     *
     * @return 唯讀設定參考
     */
    const PlannerConfig& config() const
    {
        return config_;
    }

    /**
     * @brief 依 immutable planning snapshot 執行 deterministic backend。
     *
     * Peer trajectories 在 Node 建立 snapshot 時已完成 freshness 驗證；
     * 這裡複製到 local table，避免 planner 執行期間重新讀取 callback state。
     *
     * @param snapshot 單一 planner cycle 的唯讀資料
     * @return deterministic planner 結果
     */
    PlannerResult plan(const PlanningSnapshot& snapshot) const override
    {
        const auto start = std::chrono::steady_clock::now();
        const auto finish = [&start](PlannerResult output)
        {
            output.planning_time_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            return output;
        };
        PlannerResult result;
        result.backend = PlannerBackendKind::DETERMINISTIC;
        result.trajectory_start_time_s = snapshot.trajectory_start_time_s;
        if (!snapshot.is_valid())
        {
            result.termination = PlannerTerminationReason::INVALID_INPUT;
            return finish(result);
        }

        PeerTrajectoryTableConfig table_config;
        table_config.timeout_s = 1.0e9;
        table_config.max_peer_time_skew_s = 1.0e9;
        PeerTrajectoryTable peer_table(table_config);
        for (const PeerTrajectory& peer : snapshot.peers)
        {
            if (!peer_table.update(peer))
            {
                result.termination = PlannerTerminationReason::INVALID_INPUT;
                return finish(result);
            }
        }

        PlannerResult output = plan(
            snapshot.self_state.position,
            snapshot.local_goal,
            *snapshot.obstacle_map,
            peer_table,
            snapshot.trajectory_start_time_s,
            snapshot.self_agent_id,
            snapshot.path_constraint != PathConstraint::FREE_3D);
        output.planning_time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        return output;
    }

    /**
     * @brief 產生一條避開障礙物與同儕未來軌跡的局部軌跡
     *
     * 只有 table 判定為 ACTIVE 或 TERMINAL_HOLD 的同儕軌跡會參與評估；
     * 移動軌跡逾時後不會猜測 endpoint，完成任務的 peer 則以 terminal hold
     * heartbeat 明確保留固定終點。
     * 若所有候選都不安全，回傳有限值 hold 軌跡並將 is_valid 設為 false。
     *
     * @param current_position 目前機體中心的 swarm_ned 位置
     * @param target_position 編隊槽位或任務目標的 swarm_ned 位置
     * @param obstacle_map 已驗證且具 freshness 狀態的局部障礙物地圖
     * @param peer_table 同儕軌跡生命週期表
     * @param now_s 目前共同時間，單位為秒
     * @param self_agent_id 自身 Agent ID，用於排除自身軌跡
     * @param preserve_altitude true 時只產生固定 Z 的水平候選
     * @return 規劃結果；所有輸出數值都會維持有限
     */
    PlannerResult plan(
        const Vector3& current_position,
        const Vector3& target_position,
        const perception::IObstacleMap& obstacle_map,
        const PeerTrajectoryTable& peer_table,
        const double now_s,
        const int self_agent_id,
        const bool preserve_altitude = false) const
    {
        const auto start = std::chrono::steady_clock::now();
        const auto finish = [&start](PlannerResult output)
        {
            output.planning_time_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            return output;
        };
        PlannerResult result;
        result.backend = PlannerBackendKind::DETERMINISTIC;
        result.trajectory_start_time_s = now_s;
        result.termination = PlannerTerminationReason::INVALID_INPUT;
        if (!is_valid_config(config_) ||
            !is_finite_vector(current_position) ||
            !is_finite_vector(target_position) ||
            !is_finite(now_s) ||
            !obstacle_map.is_valid() ||
            !obstacle_map.is_fresh(now_s))
        {
            return finish(result);
        }

        const std::vector<Vector3> offsets = build_candidate_offsets(
            current_position, target_position, preserve_altitude);
        bool has_best = false;
        CandidateEvaluation best_evaluation;
        BsplineTrajectory best_trajectory;
        for (const Vector3& offset : offsets)
        {
            const double path_length_m = estimate_candidate_path_length_m(
                current_position, target_position, offset);
            double duration_s = compute_trajectory_duration_s(
                path_length_m);
            const double span_count = static_cast<double>(
                config_.control_point_count - BSPLINE_DEGREE);
            BsplineTrajectory candidate;
            bool dynamically_feasible = false;
            constexpr std::size_t MAX_DYNAMIC_DURATION_ATTEMPTS = 12U;
            for (std::size_t attempt = 0U;
                 attempt < MAX_DYNAMIC_DURATION_ATTEMPTS; ++attempt)
            {
                const double candidate_knot_interval_s =
                    duration_s / span_count;
                candidate = build_candidate(
                    current_position, target_position, offset,
                    candidate_knot_interval_s);
                if (candidate.is_valid() &&
                    is_dynamically_feasible(candidate))
                {
                    dynamically_feasible = true;
                    break;
                }
                duration_s *= 1.5;
            }
            if (!dynamically_feasible)
            {
                continue;
            }
            const CandidateEvaluation evaluation = evaluate_candidate(
                candidate,
                target_position,
                obstacle_map,
                peer_table,
                now_s,
                self_agent_id);
            if (!evaluation.is_safe)
            {
                continue;
            }
            if (!has_best || evaluation.cost <
                best_evaluation.cost - EGO_SWARM_EPSILON_M)
            {
                has_best = true;
                best_evaluation = evaluation;
                best_trajectory = candidate;
            }
        }

        if (!has_best)
        {
            result = make_hold_result(current_position);
            result.termination = PlannerTerminationReason::NO_FEASIBLE_PATH;
            result.trajectory_start_time_s = now_s;
            return finish(result);
        }

        result.is_valid = true;
        result.is_hold = false;
        result.trajectory = best_trajectory;
        result.cost = best_evaluation.cost;
        result.minimum_obstacle_distance_m =
            best_evaluation.minimum_obstacle_distance_m;
        result.minimum_peer_distance_m =
            best_evaluation.minimum_peer_distance_m;
        result.goal_cost = best_evaluation.goal_cost;
        result.smoothness_cost = best_evaluation.smoothness_cost;
        result.obstacle_cost = best_evaluation.obstacle_cost;
        result.peer_cost = best_evaluation.peer_cost;
        result.termination = PlannerTerminationReason::SUCCESS;
        return finish(result);
    }

    /**
     * @brief 建立精確鎖定指定座標的 hold 軌跡
     *
     * 到達任務目標後不能以即時位置重新建立 hold，否則 odometry 的微小
     * 漂移會讓下一個 setpoint 跟著移動。此方法以指定座標填滿所有
     * B-spline 控制點，使軌跡在整個有效時間窗與逾時後的端點都固定。
     *
     * @param hold_position 要鎖定的 swarm NED 座標
     * @return 有效的固定 hold 軌跡；輸入或設定無效時回傳空結果
     */
    PlannerResult make_exact_hold_result(const Vector3& hold_position) const
    {
        return ego_swarm::make_exact_hold_result(hold_position, config_);
    }

private:
    /// 候選軌跡的安全與成本評估結果。
    struct CandidateEvaluation
    {
        bool is_safe{false};
        double cost{0.0};
        double goal_cost{0.0};
        double smoothness_cost{0.0};
        double obstacle_cost{0.0};
        double peer_cost{0.0};
        double minimum_obstacle_distance_m{0.0};
        double minimum_peer_distance_m{0.0};
    };

    /**
     * @brief 驗證規劃器設定
     *
     * @param config 待驗證設定
     * @return true 表示所有設定可用
     */
    static bool is_valid_config(const PlannerConfig& config)
    {
        return is_finite(config.knot_interval_s) &&
               config.knot_interval_s > EGO_SWARM_EPSILON_M &&
               config.control_point_count >= BSPLINE_DEGREE + 1U &&
               config.control_point_count <= EGO_SWARM_MAX_SAMPLE_COUNT &&
               is_finite(config.sample_interval_s) &&
               config.sample_interval_s > EGO_SWARM_EPSILON_M &&
               is_finite(config.max_velocity_mps) &&
               config.max_velocity_mps > EGO_SWARM_EPSILON_M &&
               is_finite(config.max_acceleration_mps2) &&
               config.max_acceleration_mps2 > EGO_SWARM_EPSILON_M &&
               is_finite(config.max_jerk_mps3) &&
               config.max_jerk_mps3 > EGO_SWARM_EPSILON_M &&
               is_finite(config.candidate_offset_m) &&
               config.candidate_offset_m >= 0.0 &&
               is_finite(config.obstacle_clearance_m) &&
               config.obstacle_clearance_m >= 0.0 &&
               is_finite(config.peer_collision_min_distance_m) &&
               config.peer_collision_min_distance_m >= 0.0 &&
               is_non_negative_finite(config.goal_weight) &&
               is_non_negative_finite(config.smoothness_weight) &&
               is_non_negative_finite(config.obstacle_weight) &&
               is_non_negative_finite(config.peer_weight);
    }

    /**
     * @brief 將 Phase 3 舊動力學參數名稱正規化為 Phase 4 契約
     * @param config 待正規化設定
     * @return 同時保留新舊欄位一致性的設定
     */
    static PlannerConfig normalize_config(PlannerConfig config)
    {
        constexpr double DEFAULT_MAX_VELOCITY_MPS = 2.0;
        constexpr double DEFAULT_MAX_ACCELERATION_MPS2 = 1.0;
        if (std::abs(config.max_velocity_mps - DEFAULT_MAX_VELOCITY_MPS) <=
                EGO_SWARM_EPSILON_M &&
            std::abs(config.max_velocity_m_s - DEFAULT_MAX_VELOCITY_MPS) >
                EGO_SWARM_EPSILON_M)
        {
            config.max_velocity_mps = config.max_velocity_m_s;
        }
        if (std::abs(config.max_acceleration_mps2 -
                     DEFAULT_MAX_ACCELERATION_MPS2) <= EGO_SWARM_EPSILON_M &&
            std::abs(config.max_acceleration_m_s2 -
                     DEFAULT_MAX_ACCELERATION_MPS2) > EGO_SWARM_EPSILON_M)
        {
            config.max_acceleration_mps2 = config.max_acceleration_m_s2;
        }
        config.max_velocity_m_s = config.max_velocity_mps;
        config.max_acceleration_m_s2 = config.max_acceleration_mps2;
        return config;
    }

    /**
     * @brief 檢查非負有限權重
     *
     * @param value 待驗證數值
     * @return true 表示可作為成本權重
     */
    static bool is_non_negative_finite(const double value)
    {
        return is_finite(value) && value >= 0.0;
    }

    /**
     * @brief 建立直行與側向／垂直繞行候選的偏移基底
     *
     * @param current_position 起點位置
     * @param target_position 終點位置
     * @param preserve_altitude 是否排除垂直繞行候選
     * @return 固定順序的候選彎曲偏移列表
     */
    std::vector<Vector3> build_candidate_offsets(
        const Vector3& current_position,
        const Vector3& target_position,
        const bool preserve_altitude) const
    {
        const Vector3 direction = subtract_vectors(
            target_position, current_position);
        const double horizontal_norm = std::sqrt(
            direction.at(0) * direction.at(0) +
            direction.at(1) * direction.at(1));
        Vector3 lateral{1.0, 0.0, 0.0};
        if (horizontal_norm > EGO_SWARM_EPSILON_M)
        {
            lateral = Vector3{
                -direction.at(1) / horizontal_norm,
                direction.at(0) / horizontal_norm,
                0.0};
        }

        const Vector3 vertical{0.0, 0.0, 1.0};
        const Vector3 zero{0.0, 0.0, 0.0};
        const Vector3 lateral_offset = scale_vector(
            lateral, config_.candidate_offset_m);
        const Vector3 vertical_offset = scale_vector(
            vertical, config_.candidate_offset_m);
        if (preserve_altitude)
        {
            return std::vector<Vector3>{
                zero,
                lateral_offset,
                scale_vector(lateral_offset, -1.0)};
        }
        return std::vector<Vector3>{
            zero,
            lateral_offset,
            scale_vector(lateral_offset, -1.0),
            vertical_offset,
            scale_vector(vertical_offset, -1.0)};
    }

    /**
     * @brief 將候選偏移套用到 B-spline 控制點
     *
     * @param current_position 起點位置
     * @param target_position 終點位置
     * @param bend_offset 中段彎曲偏移
     * @return 有限且可取樣的候選軌跡
     */
    BsplineTrajectory build_candidate(
        const Vector3& current_position,
        const Vector3& target_position,
        const Vector3& bend_offset,
        const double knot_interval_s) const
    {
        std::vector<Vector3> control_points;
        control_points.reserve(config_.control_point_count);
        const double denominator = static_cast<double>(
            config_.control_point_count - 1U);
        for (std::size_t index = 0;
             index < config_.control_point_count;
             ++index)
        {
            const double progress = static_cast<double>(index) / denominator;
            const double profile = std::sin(EGO_SWARM_PI * progress);
            const Vector3 line_position = interpolate_vectors(
                current_position, target_position, progress);
            control_points.push_back(add_vectors(
                line_position, scale_vector(bend_offset, profile)));
        }
        return BsplineTrajectory(control_points, knot_interval_s);
    }

    /**
     * @brief 依候選路徑長度與動力學上限決定軌跡時間
     *
     * knot_interval_s 是最小基準 knot 間隔；實際時間由候選路徑長度、
     * 最大速度與最大加速度共同決定。這使 EGO-Swarm 不再對所有距離
     * 使用固定 3 秒軌跡。
     *
     * @param path_length_m 候選路徑估計長度（公尺）
     * @return 有限且大於零的軌跡時間（秒）
     */
    double compute_trajectory_duration_s(const double path_length_m) const
    {
        if (!is_finite(path_length_m) || path_length_m < 0.0)
        {
            return 0.0;
        }

        const double span_count = static_cast<double>(
            config_.control_point_count - BSPLINE_DEGREE);
        const double minimum_duration =
            span_count * config_.knot_interval_s;
        const double velocity_duration =
            path_length_m / config_.max_velocity_mps;
        const double acceleration_duration = 2.0 * std::sqrt(
            path_length_m / config_.max_acceleration_mps2);
        return std::max(
            minimum_duration,
            std::max(velocity_duration, acceleration_duration));
    }

    /**
     * @brief 估計含中段偏移候選的路徑長度
     *
     * 直線距離加上兩倍中段偏移量，作為保守且 deterministic 的局部
     * 路徑長度估計；後續 Phase 可替換為完整弧長積分。
     */
    double estimate_candidate_path_length_m(
        const Vector3& current_position,
        const Vector3& target_position,
        const Vector3& bend_offset) const
    {
        const double direct_distance = distance_between_centers(
            current_position, target_position);
        const double bend_distance = std::sqrt(squared_norm(bend_offset));
        return direct_distance + 2.0 * bend_distance;
    }

    /**
     * @brief 評估候選的障礙物、同儕與平滑成本
     *
     * @param candidate 候選 B-spline
     * @param target_position 目標位置
     * @param obstacle_map 局部障礙物地圖
     * @param peer_table 最後有效同儕軌跡表
     * @param now_s 共同現在時間
     * @param self_agent_id 自身 Agent ID
     * @return 候選評估結果
     */
    CandidateEvaluation evaluate_candidate(
        const BsplineTrajectory& candidate,
        const Vector3& target_position,
        const perception::IObstacleMap& obstacle_map,
        const PeerTrajectoryTable& peer_table,
        const double now_s,
        const int self_agent_id) const
    {
        CandidateEvaluation evaluation;
        if (!candidate.is_valid())
        {
            return evaluation;
        }

        std::vector<PeerTrajectory> peers;
        peer_table.get_active(now_s, peers);
        peers.erase(
            std::remove_if(
                peers.begin(),
                peers.end(),
                [self_agent_id](const PeerTrajectory& peer)
                {
                    return peer.agent_id == self_agent_id;
                }),
            peers.end());
        const std::size_t sample_count = compute_sample_count(
            candidate.duration_s());
        double goal_cost = 0.0;
        double obstacle_cost = 0.0;
        double peer_cost = 0.0;
        double minimum_obstacle_distance =
            std::numeric_limits<double>::max();
        double minimum_peer_distance =
            std::numeric_limits<double>::max();
        for (std::size_t sample_index = 0;
             sample_index <= sample_count;
             ++sample_index)
        {
            const double sample_time = std::min(
                candidate.duration_s(),
                static_cast<double>(sample_index) *
                    config_.sample_interval_s);
            const TrajectorySample sample = candidate.evaluate(sample_time);
            if (!sample.is_valid)
            {
                return evaluation;
            }
            goal_cost += distance_between_centers(
                sample.position, target_position);
            if (!evaluate_obstacles(
                    sample.position,
                    obstacle_map,
                    minimum_obstacle_distance,
                    obstacle_cost))
            {
                return evaluation;
            }
            if (!evaluate_peers(
                    sample.position,
                    sample_time,
                    now_s,
                    self_agent_id,
                    peers,
                    minimum_peer_distance,
                    peer_cost))
            {
                return evaluation;
            }
        }

        if (obstacle_map.point_count() == 0U)
        {
            minimum_obstacle_distance = 0.0;
        }
        if (peers.empty())
        {
            minimum_peer_distance = 0.0;
        }
        const double sample_divisor = static_cast<double>(sample_count + 1U);
        const double smoothness_cost = compute_smoothness_cost(candidate);
        evaluation.cost = config_.goal_weight * goal_cost / sample_divisor +
            config_.smoothness_weight * smoothness_cost +
            config_.obstacle_weight * obstacle_cost / sample_divisor +
            config_.peer_weight * peer_cost / sample_divisor;
        evaluation.goal_cost = goal_cost / sample_divisor;
        evaluation.smoothness_cost = smoothness_cost;
        evaluation.obstacle_cost = obstacle_cost / sample_divisor;
        evaluation.peer_cost = peer_cost / sample_divisor;
        evaluation.minimum_obstacle_distance_m = minimum_obstacle_distance;
        evaluation.minimum_peer_distance_m = minimum_peer_distance;
        evaluation.is_safe = is_finite(evaluation.cost) &&
            is_finite(minimum_obstacle_distance) &&
            is_finite(minimum_peer_distance);
        return evaluation;
    }

    /**
     * @brief 依持續時間計算取樣數量
     *
     * @param duration_s 軌跡持續時間
     * @return 不超過上限的取樣區間數
     */
    std::size_t compute_sample_count(const double duration_s) const
    {
        const double raw_count = std::ceil(
            duration_s / config_.sample_interval_s);
        if (!is_finite(raw_count) || raw_count <= 0.0)
        {
            return 1U;
        }
        if (raw_count >= static_cast<double>(EGO_SWARM_MAX_SAMPLE_COUNT))
        {
            return EGO_SWARM_MAX_SAMPLE_COUNT;
        }
        return std::min(
            static_cast<std::size_t>(raw_count), EGO_SWARM_MAX_SAMPLE_COUNT);
    }

    bool is_dynamically_feasible(
        const BsplineTrajectory& trajectory) const
    {
        const std::size_t count = compute_sample_count(
            trajectory.duration_s());
        for (std::size_t index = 0U; index <= count; ++index)
        {
            const double sample_time = std::min(
                trajectory.duration_s(),
                static_cast<double>(index) * config_.sample_interval_s);
            const TrajectoryState sample = trajectory.evaluate_state(
                sample_time);
            if (!sample.is_valid ||
                std::sqrt(squared_norm(sample.velocity)) >
                    config_.max_velocity_mps + EGO_SWARM_EPSILON_M ||
                std::sqrt(squared_norm(sample.acceleration)) >
                    config_.max_acceleration_mps2 + EGO_SWARM_EPSILON_M ||
                std::sqrt(squared_norm(sample.jerk)) >
                    config_.max_jerk_mps3 + EGO_SWARM_EPSILON_M)
            {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief 計算候選控制點的二階差分平滑成本
     *
     * @param candidate 候選 B-spline
     * @return 平滑成本
     */
    static double compute_smoothness_cost(
        const BsplineTrajectory& candidate)
    {
        const std::vector<Vector3>& points = candidate.control_points();
        double cost = 0.0;
        if (points.size() < 3U)
        {
            return cost;
        }
        for (std::size_t index = 1U; index + 1U < points.size(); ++index)
        {
            const Vector3 second_difference{
                points.at(index + 1U).at(0) -
                    2.0 * points.at(index).at(0) +
                    points.at(index - 1U).at(0),
                points.at(index + 1U).at(1) -
                    2.0 * points.at(index).at(1) +
                    points.at(index - 1U).at(1),
                points.at(index + 1U).at(2) -
                    2.0 * points.at(index).at(2) +
                    points.at(index - 1U).at(2)};
            cost += std::sqrt(squared_norm(second_difference));
        }
        return is_finite(cost) ? cost : 0.0;
    }

    /**
     * @brief 評估單一取樣點與靜態障礙物的關係
     *
     * @param sample_position 本機候選位置
     * @param obstacle_map 局部障礙物地圖
     * @param[in,out] minimum_distance 目前最小障礙物距離
     * @param[in,out] cost 障礙物成本累計值
     * @return false 表示已違反 clearance 或輸入異常
     */
    bool evaluate_obstacles(
        const Vector3& sample_position,
        const perception::IObstacleMap& obstacle_map,
        double& minimum_distance,
        double& cost) const
    {
        const double nearest_distance = obstacle_map.nearest_distance(
            sample_position);
        if (obstacle_map.point_count() > 0U &&
            !is_finite(nearest_distance))
        {
            return false;
        }
        if (obstacle_map.point_count() == 0U)
        {
            return true;
        }
        minimum_distance = std::min(minimum_distance, nearest_distance);
        if (!obstacle_map.is_position_safe(
                sample_position, config_.obstacle_clearance_m))
        {
            return false;
        }
        const double obstacle_cost = obstacle_map.obstacle_cost(
            sample_position);
        if (!is_finite(obstacle_cost))
        {
            return false;
        }
        cost += obstacle_cost;
        return is_finite(cost);
    }

    /**
     * @brief 評估單一取樣點與同儕未來軌跡的關係
     *
     * @param sample_position 本機候選位置
     * @param sample_time_s 本機候選相對時間
     * @param now_s 規劃當下共同時間
     * @param self_agent_id 自身 Agent ID
     * @param peers 最後有效同儕軌跡
     * @param[in,out] minimum_distance 目前最小同儕中心距離
     * @param[in,out] cost 同儕成本累計值
     * @return false 表示已違反中心距離或輸入異常
     */
    bool evaluate_peers(
        const Vector3& sample_position,
        const double sample_time_s,
        const double now_s,
        const int self_agent_id,
        const std::vector<PeerTrajectory>& peers,
        double& minimum_distance,
        double& cost) const
    {
        for (const PeerTrajectory& peer : peers)
        {
            if (peer.agent_id == self_agent_id)
            {
                continue;
            }
            const double absolute_sample_time_s = now_s + sample_time_s;
            if (!is_finite(absolute_sample_time_s))
            {
                return false;
            }
            if (!peer.terminal_hold &&
                absolute_sample_time_s > peer.end_time_s +
                EGO_SWARM_EPSILON_M)
            {
                continue;
            }
            // 路線開始前的同儕仍停在自己的目前位置，不能把它當成
            // 已經讓路；只有在同儕路線開始後，較早的路線才可取得
            // 通行優先權，讓較晚的 Agent 重新規劃或等待。
            if (!should_check_peer_occupancy(
                    peer, absolute_sample_time_s, now_s, self_agent_id))
            {
                continue;
            }
            const double peer_relative_time = std::max(
                0.0, absolute_sample_time_s - peer.start_time_s);
            const TrajectorySample peer_sample =
                peer.trajectory.evaluate(peer_relative_time);
            if (!peer_sample.is_valid)
            {
                return false;
            }
            const double distance = distance_between_centers(
                sample_position, peer_sample.position);
            if (!is_finite(distance))
            {
                return false;
            }
            minimum_distance = std::min(minimum_distance, distance);
            if (distance + EGO_SWARM_EPSILON_M <
                config_.peer_collision_min_distance_m)
            {
                return false;
            }
            cost += 1.0 / std::max(
                distance, EGO_SWARM_EPSILON_M);
        }
        return is_finite(cost);
    }

    /**
     * @brief 在無安全候選時建立目前位置 hold 軌跡
     *
     * @param current_position 目前位置
     * @return 失敗狀態與有限值 hold 軌跡
     */
    PlannerResult make_hold_result(const Vector3& current_position) const
    {
        PlannerResult result = make_exact_hold_result(current_position);
        // 無安全候選時仍保留既有語意：這是 planner failure 的 hold，
        // 而非已確認任務完成的有效結果。
        result.is_valid = false;
        return result;
    }

    PlannerConfig config_{};
};

}  // namespace ego_swarm
}  // namespace taiphoon
