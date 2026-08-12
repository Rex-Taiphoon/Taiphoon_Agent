/**
 * @file BsplineTrajectory.hpp
 * @brief 可驗證有限值的均勻 clamped cubic B-spline 軌跡
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * 本模組提供 ROS2 Node 取樣軌跡所需的最小深層介面。它不負責飛控
 * 控制，也不依賴任何 ROS2 型別。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "ego_swarm/EgoSwarmTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// cubic B-spline 的預設階數。
constexpr std::size_t BSPLINE_DEGREE = 3U;

/**
 * @brief B-spline 在單一時間點的完整狀態。
 *
 * 所有導數都直接由同一組控制點的 analytic derivative 計算，避免控制
 * 迴圈以 finite difference 估算速度、加速度或 jerk。
 */
struct TrajectoryState
{
    Vector3 position{0.0, 0.0, 0.0};
    Vector3 velocity{0.0, 0.0, 0.0};
    Vector3 acceleration{0.0, 0.0, 0.0};
    Vector3 jerk{0.0, 0.0, 0.0};
    bool is_valid{false};
};

/// 保留既有呼叫端名稱，讓 trajectory sampling API 向後相容。
using TrajectorySample = TrajectoryState;

/**
 * @brief 均勻 clamped cubic B-spline 軌跡
 *
 * 控制點數量至少為四個。時間超出軌跡範圍時會夾到端點，避免控制端
 * 因取樣時鐘略微超前而收到非有限或未定義位置。
 */
class BsplineTrajectory
{
public:
    /**
     * @brief 使用控制點與 knot interval 建立軌跡
     *
     * @param control_points cubic B-spline 控制點
     * @param knot_interval_s 相鄰 knot 的時間間隔，單位為秒
     */
    BsplineTrajectory(
        const std::vector<Vector3>& control_points,
        const double knot_interval_s)
    {
        set_control_points(control_points, knot_interval_s);
    }

    /**
     * @brief 使用控制點、knot interval 與階數建立軌跡
     *
     * @param control_points B-spline 控制點
     * @param knot_interval_s 相鄰 knot 的時間間隔，單位為秒
     * @param degree B-spline 多項式階數
     */
    BsplineTrajectory(
        const std::vector<Vector3>& control_points,
        const double knot_interval_s,
        const std::size_t degree)
    {
        set_control_points(control_points, knot_interval_s, degree);
    }

    /**
     * @brief 建立空白無效軌跡
     */
    BsplineTrajectory() = default;

    /**
     * @brief 設定控制點並驗證輸入
     *
     * @param control_points cubic B-spline 控制點
     * @param knot_interval_s 相鄰 knot 的時間間隔，單位為秒
     * @return true 表示軌跡有效並已儲存
     */
    bool set_control_points(
        const std::vector<Vector3>& control_points,
        const double knot_interval_s,
        const std::size_t degree = BSPLINE_DEGREE)
    {
        if (degree == 0U ||
            control_points.size() <= degree ||
            !is_finite(knot_interval_s) ||
            knot_interval_s <= EGO_SWARM_EPSILON_M)
        {
            control_points_.clear();
            knot_interval_s_ = 0.0;
            degree_ = BSPLINE_DEGREE;
            return false;
        }

        for (const Vector3& control_point : control_points)
        {
            if (!is_finite_vector(control_point))
            {
                control_points_.clear();
                knot_interval_s_ = 0.0;
                return false;
            }
        }

        control_points_ = control_points;
        knot_interval_s_ = knot_interval_s;
        degree_ = degree;
        return true;
    }

    /**
     * @brief 檢查軌跡是否包含足夠且有限的資料
     *
     * @return true 表示可供取樣
     */
    bool is_valid() const
    {
        if (degree_ == 0U ||
            control_points_.size() <= degree_ ||
            !is_finite(knot_interval_s_) ||
            knot_interval_s_ <= EGO_SWARM_EPSILON_M)
        {
            return false;
        }

        for (const Vector3& control_point : control_points_)
        {
            if (!is_finite_vector(control_point))
            {
                return false;
            }
        }
        return is_finite(duration_s());
    }

    /**
     * @brief 取得軌跡持續時間
     *
     * @return 軌跡持續時間，無效軌跡回傳零
     */
    double duration_s() const
    {
        if (degree_ == 0U ||
            control_points_.size() <= degree_ ||
            !is_finite(knot_interval_s_))
        {
            return 0.0;
        }
        return static_cast<double>(control_points_.size() - degree_) *
            knot_interval_s_;
    }

    /**
     * @brief 以相對時間評估軌跡位置
     *
     * @param time_s 相對軌跡起點時間，單位為秒
     * @return 取樣結果；輸入或軌跡無效時 is_valid 為 false
     */
    TrajectorySample evaluate(const double time_s) const
    {
        return evaluate_state(time_s);
    }

    /**
     * @brief 以相對時間評估位置、速度、加速度與 jerk
     *
     * @param time_s 相對軌跡起點時間，單位為秒
     * @return 完整取樣狀態；輸入或軌跡無效時 is_valid 為 false
     */
    TrajectoryState evaluate_state(const double time_s) const
    {
        TrajectoryState result;
        if (!is_valid() || !is_finite(time_s))
        {
            return result;
        }

        const double clamped_time = std::max(
            0.0, std::min(time_s, duration_s()));
        if (!evaluate_curve(
                control_points_, degree_, knot_interval_s_, clamped_time,
                result.position))
        {
            return TrajectoryState{};
        }

        std::vector<Vector3> derivative_control_points = control_points_;
        std::size_t derivative_degree = degree_;
        Vector3* derivative_outputs[] = {
            &result.velocity, &result.acceleration, &result.jerk};

        for (Vector3* derivative_output : derivative_outputs)
        {
            if (derivative_degree == 0U)
            {
                *derivative_output = Vector3{0.0, 0.0, 0.0};
                continue;
            }

            derivative_control_points = compute_derivative_control_points(
                derivative_control_points, derivative_degree,
                knot_interval_s_);
            if (derivative_control_points.empty())
            {
                return TrajectoryState{};
            }
            --derivative_degree;
            if (!evaluate_curve(
                    derivative_control_points, derivative_degree,
                    knot_interval_s_, clamped_time, *derivative_output))
            {
                return TrajectoryState{};
            }
        }

        result.is_valid = is_finite_vector(result.position) &&
            is_finite_vector(result.velocity) &&
            is_finite_vector(result.acceleration) &&
            is_finite_vector(result.jerk);
        if (!result.is_valid)
        {
            return TrajectoryState{};
        }
        return result;
    }

    /**
     * @brief 取得控制點
     *
     * @return 唯讀控制點列表
     */
    const std::vector<Vector3>& control_points() const
    {
        return control_points_;
    }

    /**
     * @brief 取得 knot interval
     *
     * @return knot interval，單位為秒
     */
    double knot_interval_s() const
    {
        return knot_interval_s_;
    }

    /**
     * @brief 取得 B-spline 多項式階數
     * @return B-spline 階數
     */
    std::size_t degree() const
    {
        return degree_;
    }

private:
    /**
     * @brief 建立均勻 clamped knot 向量
     *
     * @param control_point_count 控制點數量
     * @param degree B-spline 階數
     * @param knot_interval_s knot 間隔，單位為秒
     * @return knot 向量
     */
    static std::vector<double> build_knots(
        const std::size_t control_point_count,
        const std::size_t degree,
        const double knot_interval_s)
    {
        const std::size_t knot_count =
            control_point_count + degree + 1U;
        const double duration = static_cast<double>(
            control_point_count - degree) * knot_interval_s;
        std::vector<double> knots(knot_count, 0.0);
        for (std::size_t index = 0; index < knot_count; ++index)
        {
            if (index <= degree)
            {
                knots.at(index) = 0.0;
            }
            else if (index >= control_point_count)
            {
                knots.at(index) = duration;
            }
            else
            {
                knots.at(index) = static_cast<double>(
                    index - degree) * knot_interval_s;
            }
        }
        return knots;
    }

    /**
     * @brief 評估任意階數的均勻 clamped B-spline curve
     *
     * @param control_points 曲線控制點
     * @param degree 曲線階數
     * @param knot_interval_s knot 間隔，單位為秒
     * @param time_s 取樣時間，單位為秒
     * @param[out] value 曲線取樣結果
     * @return true 表示取樣成功
     */
    static bool evaluate_curve(
        const std::vector<Vector3>& control_points,
        const std::size_t degree,
        const double knot_interval_s,
        const double time_s,
        Vector3& value)
    {
        if (degree >= control_points.size() ||
            !is_finite(knot_interval_s) ||
            knot_interval_s <= EGO_SWARM_EPSILON_M ||
            !is_finite(time_s))
        {
            return false;
        }

        const double duration = static_cast<double>(
            control_points.size() - degree) * knot_interval_s;
        const double clamped_time = std::max(
            0.0, std::min(time_s, duration));
        if (clamped_time <= EGO_SWARM_EPSILON_M)
        {
            value = control_points.front();
            return is_finite_vector(value);
        }
        if (duration - clamped_time <= EGO_SWARM_EPSILON_M)
        {
            value = control_points.back();
            return is_finite_vector(value);
        }

        const std::vector<double> knots = build_knots(
            control_points.size(), degree, knot_interval_s);
        value = Vector3{0.0, 0.0, 0.0};
        for (std::size_t index = 0U; index < control_points.size(); ++index)
        {
            const double basis = evaluate_basis(
                index, degree, clamped_time, knots);
            if (!is_finite(basis))
            {
                return false;
            }
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                value.at(axis) += basis * control_points.at(index).at(axis);
            }
        }
        return is_finite_vector(value);
    }

    /**
     * @brief 建立一階 analytic derivative 的控制點
     *
     * @param control_points 原始曲線控制點
     * @param degree 原始曲線階數
     * @param knot_interval_s knot 間隔，單位為秒
     * @return 一階 derivative 控制點；輸入無效時回傳空列表
     */
    static std::vector<Vector3> compute_derivative_control_points(
        const std::vector<Vector3>& control_points,
        const std::size_t degree,
        const double knot_interval_s)
    {
        std::vector<Vector3> derivative_control_points;
        if (degree == 0U || degree >= control_points.size() ||
            !is_finite(knot_interval_s) ||
            knot_interval_s <= EGO_SWARM_EPSILON_M)
        {
            return derivative_control_points;
        }

        const std::vector<double> knots = build_knots(
            control_points.size(), degree, knot_interval_s);
        derivative_control_points.reserve(control_points.size() - 1U);
        for (std::size_t index = 0U;
             index + 1U < control_points.size();
             ++index)
        {
            const double denominator = knots.at(index + degree + 1U) -
                knots.at(index + 1U);
            if (!is_finite(denominator) ||
                denominator <= EGO_SWARM_EPSILON_M)
            {
                return std::vector<Vector3>{};
            }

            const double scale = static_cast<double>(degree) / denominator;
            const Vector3 derivative = scale_vector(
                subtract_vectors(
                    control_points.at(index + 1U),
                    control_points.at(index)),
                scale);
            if (!is_finite_vector(derivative))
            {
                return std::vector<Vector3>{};
            }
            derivative_control_points.push_back(derivative);
        }
        return derivative_control_points;
    }

    /**
     * @brief 以 Cox-de Boor 遞迴公式計算基底函數
     *
     * @param index 控制點索引
     * @param degree 基底函數階數
     * @param time_s 評估時間
     * @param knots knot 向量
     * @return 基底函數權重
     */
    static double evaluate_basis(
        const std::size_t index,
        const std::size_t degree,
        const double time_s,
        const std::vector<double>& knots)
    {
        if (index + degree + 1U >= knots.size())
        {
            return 0.0;
        }
        if (degree == 0U)
        {
            const bool is_in_interval =
                knots.at(index) <= time_s && time_s < knots.at(index + 1U);
            return is_in_interval ? 1.0 : 0.0;
        }

        const double left_denominator =
            knots.at(index + degree) - knots.at(index);
        const double right_denominator =
            knots.at(index + degree + 1U) - knots.at(index + 1U);
        double left_term = 0.0;
        double right_term = 0.0;
        if (left_denominator > EGO_SWARM_EPSILON_M)
        {
            left_term = (time_s - knots.at(index)) / left_denominator *
                evaluate_basis(index, degree - 1U, time_s, knots);
        }
        if (right_denominator > EGO_SWARM_EPSILON_M)
        {
            right_term = (knots.at(index + degree + 1U) - time_s) /
                right_denominator *
                evaluate_basis(index + 1U, degree - 1U, time_s, knots);
        }
        return left_term + right_term;
    }

    std::vector<Vector3> control_points_{};
    double knot_interval_s_{0.0};
    std::size_t degree_{BSPLINE_DEGREE};
};

}  // namespace ego_swarm
}  // namespace taiphoon
