/**
 * @file RawPointObstacleMap.cpp
 * @brief raw-point 局部障礙物地圖查詢實作
 * @author Taiphoon Agent Team
 * @date 2026-08-12
 */

#include "perception/RawPointObstacleMap.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace taiphoon
{
namespace perception
{
namespace
{

constexpr double OBSTACLE_MAP_EPSILON_S = 1.0e-6;
constexpr double OBSTACLE_MAP_EPSILON_M = 1.0e-6;

bool is_finite_points(const std::vector<ego_swarm::Vector3>& points)
{
    for (const auto& point : points)
    {
        if (!ego_swarm::is_finite_vector(point))
        {
            return false;
        }
    }
    return true;
}

}  // namespace

RawPointObstacleMap::RawPointObstacleMap(const double freshness_timeout_s)
{
    if (!set_freshness_timeout(freshness_timeout_s))
    {
        freshness_timeout_s_ = DEFAULT_OBSTACLE_MAP_TIMEOUT_S;
    }
}

bool RawPointObstacleMap::set_freshness_timeout(
    const double freshness_timeout_s)
{
    if (!std::isfinite(freshness_timeout_s) || freshness_timeout_s <= 0.0)
    {
        return false;
    }
    freshness_timeout_s_ = freshness_timeout_s;
    return true;
}

bool RawPointObstacleMap::update(
    const std::vector<ego_swarm::Vector3>& points,
    const double stamp_s)
{
    if (!std::isfinite(stamp_s) || stamp_s < 0.0 ||
        !is_finite_points(points))
    {
        return false;
    }
    points_ = points;
    stamp_s_ = stamp_s;
    has_received_ = true;
    is_valid_ = true;
    return true;
}

bool RawPointObstacleMap::set_explicit_empty(const double stamp_s)
{
    return update({}, stamp_s);
}

bool RawPointObstacleMap::invalidate()
{
    has_received_ = true;
    is_valid_ = false;
    points_.clear();
    return true;
}

bool RawPointObstacleMap::is_valid() const
{
    return has_received_ && is_valid_;
}

bool RawPointObstacleMap::is_fresh(const double now_s) const
{
    return status(now_s) == ObstacleMapStatus::Fresh;
}

ObstacleMapStatus RawPointObstacleMap::status(const double now_s) const
{
    if (!has_received_)
    {
        return ObstacleMapStatus::Missing;
    }
    if (!is_valid_ || !std::isfinite(stamp_s_) ||
        !std::isfinite(freshness_timeout_s_))
    {
        return ObstacleMapStatus::Invalid;
    }
    if (!std::isfinite(now_s) || now_s + OBSTACLE_MAP_EPSILON_S < stamp_s_)
    {
        return ObstacleMapStatus::Invalid;
    }
    if (now_s - stamp_s_ > freshness_timeout_s_ + OBSTACLE_MAP_EPSILON_S)
    {
        return ObstacleMapStatus::Stale;
    }
    return ObstacleMapStatus::Fresh;
}

std::size_t RawPointObstacleMap::point_count() const
{
    return is_valid_ ? points_.size() : 0U;
}

double RawPointObstacleMap::nearest_distance(
    const ego_swarm::Vector3& position) const
{
    if (!is_valid_ || !ego_swarm::is_finite_vector(position))
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (points_.empty())
    {
        return std::numeric_limits<double>::infinity();
    }

    double nearest = std::numeric_limits<double>::infinity();
    for (const auto& point : points_)
    {
        const double distance = ego_swarm::distance_between_centers(
            position, point);
        if (!std::isfinite(distance))
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
        nearest = std::min(nearest, distance);
    }
    return nearest;
}

bool RawPointObstacleMap::is_position_safe(
    const ego_swarm::Vector3& position,
    const double clearance_m) const
{
    if (!is_valid_ || !ego_swarm::is_finite_vector(position) ||
        !std::isfinite(clearance_m) || clearance_m < 0.0)
    {
        return false;
    }

    for (const auto& point : points_)
    {
        const double distance = ego_swarm::distance_between_centers(
            position, point);
        if (!std::isfinite(distance) ||
            distance + OBSTACLE_MAP_EPSILON_M < clearance_m)
        {
            return false;
        }
    }
    return true;
}

double RawPointObstacleMap::obstacle_cost(
    const ego_swarm::Vector3& position) const
{
    if (!is_valid_ || !ego_swarm::is_finite_vector(position))
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double cost = 0.0;
    for (const auto& point : points_)
    {
        const double distance = ego_swarm::distance_between_centers(
            position, point);
        if (!std::isfinite(distance))
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
        cost += 1.0 / std::max(distance, OBSTACLE_MAP_EPSILON_M);
    }
    return std::isfinite(cost)
        ? cost : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace perception
}  // namespace taiphoon
