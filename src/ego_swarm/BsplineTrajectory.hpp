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

/// cubic B-spline 的固定階數。
constexpr std::size_t BSPLINE_DEGREE = 3U;

/// B-spline 評估結果。
struct TrajectorySample
{
    Vector3 position{0.0, 0.0, 0.0};
    bool is_valid{false};
};

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
        const double knot_interval_s)
    {
        if (control_points.size() < BSPLINE_DEGREE + 1U ||
            !is_finite(knot_interval_s) ||
            knot_interval_s <= EGO_SWARM_EPSILON_M)
        {
            control_points_.clear();
            knot_interval_s_ = 0.0;
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
        return true;
    }

    /**
     * @brief 檢查軌跡是否包含足夠且有限的資料
     *
     * @return true 表示可供取樣
     */
    bool is_valid() const
    {
        if (control_points_.size() < BSPLINE_DEGREE + 1U ||
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
        if (control_points_.size() < BSPLINE_DEGREE + 1U ||
            !is_finite(knot_interval_s_))
        {
            return 0.0;
        }
        return static_cast<double>(
            control_points_.size() - BSPLINE_DEGREE) * knot_interval_s_;
    }

    /**
     * @brief 以相對時間評估軌跡位置
     *
     * @param time_s 相對軌跡起點時間，單位為秒
     * @return 取樣結果；輸入或軌跡無效時 is_valid 為 false
     */
    TrajectorySample evaluate(const double time_s) const
    {
        TrajectorySample result;
        if (!is_valid() || !is_finite(time_s))
        {
            return result;
        }

        const double duration = duration_s();
        const double clamped_time = std::max(0.0, std::min(time_s, duration));
        if (clamped_time <= EGO_SWARM_EPSILON_M)
        {
            result.position = control_points_.front();
            result.is_valid = true;
            return result;
        }
        if (duration - clamped_time <= EGO_SWARM_EPSILON_M)
        {
            result.position = control_points_.back();
            result.is_valid = true;
            return result;
        }

        const std::vector<double> knots = build_knots();
        Vector3 position{0.0, 0.0, 0.0};
        for (std::size_t index = 0; index < control_points_.size(); ++index)
        {
            const double basis = evaluate_basis(
                index, BSPLINE_DEGREE, clamped_time, knots);
            if (!is_finite(basis))
            {
                return TrajectorySample{};
            }
            position.at(0) += basis * control_points_.at(index).at(0);
            position.at(1) += basis * control_points_.at(index).at(1);
            position.at(2) += basis * control_points_.at(index).at(2);
        }

        if (!is_finite_vector(position))
        {
            return TrajectorySample{};
        }
        result.position = position;
        result.is_valid = true;
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

private:
    /**
     * @brief 建立均勻 clamped knot 向量
     *
     * @return knot 向量
     */
    std::vector<double> build_knots() const
    {
        const std::size_t knot_count =
            control_points_.size() + BSPLINE_DEGREE + 1U;
        const double duration = duration_s();
        std::vector<double> knots(knot_count, 0.0);
        for (std::size_t index = 0; index < knot_count; ++index)
        {
            if (index <= BSPLINE_DEGREE)
            {
                knots.at(index) = 0.0;
            }
            else if (index >= control_points_.size())
            {
                knots.at(index) = duration;
            }
            else
            {
                knots.at(index) = static_cast<double>(
                    index - BSPLINE_DEGREE) * knot_interval_s_;
            }
        }
        return knots;
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
};

}  // namespace ego_swarm
}  // namespace taiphoon
