/**
 * @file EmergencyBrakePlanner.hpp
 * @brief 依自身動態狀態產生有限 jerk braking trajectory
 * @author Taiphoon Agent Team
 * @date 2026-08-12
 *
 * 此模組是 planner failure、peer conflict 與 obstacle conflict 的共同
 * fail-safe path。速度非零時會產生向前減速的 B-spline，不會把目前位置
 * 複製成一串靜態控制點。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "ego_swarm/EgoSwarmPlanner.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief 產生 emergency braking B-spline 的純 C++ 模組
 */
class EmergencyBrakePlanner
{
public:
    /**
     * @brief 使用 planner 動力學限制建立煞車規劃器
     * @param config planner 與 safety 共用設定
     */
    explicit EmergencyBrakePlanner(
        const PlannerConfig& config = PlannerConfig{})
    {
        if (!set_config(config))
        {
            config_ = PlannerConfig{};
        }
    }

    /**
     * @brief 更新煞車規劃設定
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
     * @brief 從目前位置、速度、加速度產生 braking trajectory
     * @param state 單一 planner cycle 的自身狀態
     * @return 有限且可取樣的 emergency trajectory；輸入無效時回傳無效結果
     */
    PlannerResult plan(const VehiclePlanningState& state) const
    {
        PlannerResult result;
        if (!is_valid_config(config_) || !state.is_finite())
        {
            return result;
        }

        const double speed_mps = vector_norm(state.velocity);
        const Vector3 effective_acceleration = braking_acceleration(state);
        const double acceleration_mps2 = vector_norm(effective_acceleration);
        constexpr double STATIONARY_SPEED_EPSILON_MPS = 0.05;
        if (speed_mps <= STATIONARY_SPEED_EPSILON_MPS)
        {
            const std::vector<Vector3> hold_points(
                config_.control_point_count, state.position);
            result.trajectory = BsplineTrajectory(
                hold_points, config_.knot_interval_s);
            result.is_valid = result.trajectory.is_valid();
            result.is_hold = true;
            return result;
        }

        const double initial_duration = minimum_brake_duration(
            speed_mps, acceleration_mps2);
        double duration_s = std::max(
            config_.knot_interval_s, initial_duration);
        constexpr std::size_t MAX_DURATION_ATTEMPTS = 24U;
        constexpr double DURATION_GROWTH = 1.5;
        for (std::size_t attempt = 0U;
             attempt < MAX_DURATION_ATTEMPTS; ++attempt)
        {
            VehiclePlanningState effective_state = state;
            effective_state.acceleration = effective_acceleration;
            const BsplineTrajectory candidate = build_candidate(
                effective_state, duration_s);
            if (candidate.is_valid() && is_dynamically_feasible(candidate))
            {
                result.trajectory = candidate;
                result.is_valid = true;
                result.is_hold = true;
                return result;
            }
            duration_s *= DURATION_GROWTH;
        }
        return result;
    }

    /**
     * @brief 取得最後設定
     * @return 唯讀 planner 設定
     */
    const PlannerConfig& config() const
    {
        return config_;
    }

private:
    static bool is_valid_config(const PlannerConfig& config)
    {
        return std::isfinite(config.knot_interval_s) &&
               config.knot_interval_s > EGO_SWARM_EPSILON_M &&
               config.control_point_count >= BSPLINE_DEGREE + 1U &&
               std::isfinite(config.sample_interval_s) &&
               config.sample_interval_s > EGO_SWARM_EPSILON_M &&
               std::isfinite(config.max_velocity_mps) &&
               config.max_velocity_mps > EGO_SWARM_EPSILON_M &&
               std::isfinite(config.max_acceleration_mps2) &&
               config.max_acceleration_mps2 > EGO_SWARM_EPSILON_M &&
               std::isfinite(config.max_jerk_mps3) &&
               config.max_jerk_mps3 > EGO_SWARM_EPSILON_M;
    }

    static double vector_norm(const Vector3& value)
    {
        return std::sqrt(squared_norm(value));
    }

    double minimum_brake_duration(
        const double speed_mps,
        const double acceleration_mps2) const
    {
        const double acceleration_duration = speed_mps /
            config_.max_acceleration_mps2;
        const double jerk_duration = std::sqrt(
            6.0 * speed_mps / config_.max_jerk_mps3);
        const double acceleration_change_duration = acceleration_mps2 /
            config_.max_jerk_mps3;
        return std::max(
            acceleration_duration,
            std::max(jerk_duration, acceleration_change_duration));
    }

    Vector3 limit_acceleration(const Vector3& acceleration) const
    {
        const double norm = vector_norm(acceleration);
        if (!std::isfinite(norm) || norm <= config_.max_acceleration_mps2)
        {
            return acceleration;
        }
        return scale_vector(
            acceleration, config_.max_acceleration_mps2 / norm);
    }

    Vector3 braking_acceleration(const VehiclePlanningState& state) const
    {
        const double speed_squared = squared_norm(state.velocity);
        if (!std::isfinite(speed_squared) ||
            speed_squared <= EGO_SWARM_EPSILON_M)
        {
            return Vector3{0.0, 0.0, 0.0};
        }

        const double projection =
            (state.acceleration.at(0) * state.velocity.at(0) +
             state.acceleration.at(1) * state.velocity.at(1) +
             state.acceleration.at(2) * state.velocity.at(2)) /
            speed_squared;
        // 只有沿目前速度方向的減速分量才納入 emergency trajectory。
        // 受雜訊影響的加速或側向 acceleration 不應讓煞車控制點變成
        // 無法滿足 endpoint acceleration limit 的軌跡。
        if (!std::isfinite(projection) || projection >= 0.0)
        {
            return Vector3{0.0, 0.0, 0.0};
        }
        return limit_acceleration(scale_vector(state.velocity, projection));
    }

    BsplineTrajectory build_candidate(
        const VehiclePlanningState& state,
        const double duration_s) const
    {
        // 這組控制點是 cubic polynomial 的 Hermite-like braking 形式：
        // P0 對齊目前位置，P1 對齊目前速度，P2=P3 讓末端速度為零。
        const Vector3 first = add_vectors(
            state.position, scale_vector(state.velocity, duration_s / 3.0));
        const Vector3 second = add_vectors(
            state.position,
            add_vectors(
                scale_vector(state.velocity, 2.0 * duration_s / 3.0),
                scale_vector(
                    state.acceleration, duration_s * duration_s / 6.0)));
        const std::vector<Vector3> control_points{
            state.position, first, second, second};
        return BsplineTrajectory(control_points, duration_s);
    }

    bool is_dynamically_feasible(
        const BsplineTrajectory& trajectory) const
    {
        const TrajectoryState initial_state = trajectory.evaluate_state(0.0);
        if (!initial_state.is_valid)
        {
            return false;
        }
        // 煞車的觸發時刻可能已經超過 max_velocity；不能因為起點本身
        // 超速就拒絕唯一能讓速度下降的 fail-safe trajectory。
        const double initial_speed = vector_norm(initial_state.velocity);
        const double permitted_speed = std::max(
            config_.max_velocity_mps, initial_speed);
        const double raw_count = std::ceil(
            trajectory.duration_s() / config_.sample_interval_s);
        const std::size_t count = raw_count <= 0.0 || !std::isfinite(raw_count)
            ? 1U
            : std::min(
                static_cast<std::size_t>(raw_count),
                EGO_SWARM_MAX_SAMPLE_COUNT);
        for (std::size_t index = 0U; index <= count; ++index)
        {
            const double sample_time = std::min(
                trajectory.duration_s(),
                static_cast<double>(index) * config_.sample_interval_s);
            const TrajectoryState sample = trajectory.evaluate_state(
                sample_time);
            if (!sample.is_valid ||
                vector_norm(sample.velocity) > permitted_speed +
                    EGO_SWARM_EPSILON_M ||
                vector_norm(sample.acceleration) >
                    config_.max_acceleration_mps2 + EGO_SWARM_EPSILON_M ||
                vector_norm(sample.jerk) > config_.max_jerk_mps3 +
                    EGO_SWARM_EPSILON_M)
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
