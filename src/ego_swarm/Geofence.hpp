/**
 * @file Geofence.hpp
 * @brief planner 水平地理圍籬（簡單多邊形）的解析與幾何查詢
 * @author Taiphoon Agent Team
 * @date 2026-11-05
 *
 * 圍籬是 Agent planner 的硬性規劃約束，只限制 North／East 水平位置。
 * 本檔為 pure C++，WGS84 頂點由 Node 在 planner Home 凍結後一次轉成
 * planner NED，之後所有規劃階段共用同一份唯讀幾何。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

#include "ego_swarm/EgoSwarmTypes.hpp"
#include "ego_swarm/SwarmFrame.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// 圍籬頂點上限；避免誤填大量頂點拖慢每次取樣的查詢。
constexpr std::size_t GEOFENCE_MAX_VERTEX_COUNT = 64U;
/// 頂點距 planner Home 的上限（m）；超過視為座標誤填。
constexpr double GEOFENCE_MAX_VERTEX_DISTANCE_M = 5000.0;
/// 圍籬最小面積（m²）；退化多邊形無法提供可飛空間。
constexpr double GEOFENCE_MIN_AREA_M2 = 1.0;
/// 最佳化器在硬性 margin 外再保留的軟性緩衝（m），把軌跡推離邊界。
constexpr double GEOFENCE_OPTIMIZER_SOFT_MARGIN_M = 0.5;
/// 起點已在 margin 帶內時，允許軌跡比起點更外側的容差（m）。
constexpr double GEOFENCE_RECOVERY_TOLERANCE_M = 0.05;

/**
 * @brief 解析 `lat,lon;lat,lon;...` 格式的 WGS84 圍籬頂點
 *
 * 空字串（或只含空白）代表停用圍籬，回傳 true 且輸出為空。
 * 數值以 classic locale 解析，避免容器 locale 改變小數點符號。
 *
 * @param text 參數字串
 * @param[out] out_vertices 解析出的頂點；失敗時清空
 * @return true 表示格式正確（含停用）；false 表示格式錯誤
 */
inline bool parse_geofence_polygon(
    const std::string& text,
    std::vector<GeoCoordinate>& out_vertices)
{
    out_vertices.clear();
    if (text.find_first_not_of(" \t\r\n") == std::string::npos)
    {
        return true;
    }
    std::istringstream vertex_stream(text);
    std::string vertex_text;
    while (std::getline(vertex_stream, vertex_text, ';'))
    {
        if (vertex_text.find_first_not_of(" \t\r\n") == std::string::npos)
        {
            continue;
        }
        std::istringstream value_stream(vertex_text);
        value_stream.imbue(std::locale::classic());
        GeoCoordinate vertex{};
        char separator = '\0';
        const bool is_parsed = static_cast<bool>(
            value_stream >> vertex.latitude_deg >> separator >>
            vertex.longitude_deg);
        std::string trailing;
        value_stream >> trailing;
        if (!is_parsed || separator != ',' || !trailing.empty() ||
            !LocalNedFrame::is_valid_geodetic(vertex) ||
            out_vertices.size() >= GEOFENCE_MAX_VERTEX_COUNT)
        {
            out_vertices.clear();
            return false;
        }
        out_vertices.push_back(vertex);
    }
    if (out_vertices.size() < 3U)
    {
        out_vertices.clear();
        return false;
    }
    return true;
}

/**
 * @brief planner NED 中的水平簡單多邊形圍籬
 *
 * clearance 定義為「點到最近邊的水平距離」，多邊形內為正、外為負。
 * 所有查詢遇到無效圍籬或非有限輸入時 fail closed（視為在圍籬外）。
 */
class Geofence
{
public:
    /**
     * @brief 設定 planner NED 頂點與硬性 margin
     * @param vertices_ned 依序排列的頂點（只使用 North／East）
     * @param margin_m 機體中心與邊界的最小水平距離
     * @return true 表示多邊形有效（不自交、面積足夠）
     */
    bool set_polygon(
        const std::vector<Vector3>& vertices_ned,
        const double margin_m)
    {
        vertices_.clear();
        is_valid_ = false;
        if (vertices_ned.size() < 3U ||
            vertices_ned.size() > GEOFENCE_MAX_VERTEX_COUNT ||
            !is_finite(margin_m) || margin_m < 0.0)
        {
            return false;
        }
        for (const Vector3& vertex : vertices_ned)
        {
            if (!is_finite(vertex.at(0)) || !is_finite(vertex.at(1)))
            {
                return false;
            }
            vertices_.push_back(Vector3{vertex.at(0), vertex.at(1), 0.0});
        }
        if (std::abs(signed_area_m2()) < GEOFENCE_MIN_AREA_M2 ||
            is_self_intersecting())
        {
            vertices_.clear();
            return false;
        }
        margin_m_ = margin_m;
        is_valid_ = true;
        return true;
    }

    /**
     * @brief 圍籬是否可用
     * @return true 表示已設定有效多邊形
     */
    bool is_valid() const
    {
        return is_valid_;
    }

    /**
     * @brief 取得硬性 margin
     * @return 機體中心與邊界的最小水平距離（m）
     */
    double margin_m() const
    {
        return margin_m_;
    }

    /**
     * @brief 取得 planner NED 頂點
     * @return 唯讀頂點列表
     */
    const std::vector<Vector3>& vertices() const
    {
        return vertices_;
    }

    /**
     * @brief 計算水平 clearance
     * @param position planner NED 位置（忽略 Down）
     * @return 多邊形內為正、外為負；無效時回傳 -infinity
     */
    double clearance_m(const Vector3& position) const
    {
        if (!is_valid_ || !is_finite(position.at(0)) ||
            !is_finite(position.at(1)))
        {
            return -std::numeric_limits<double>::infinity();
        }
        double minimum_distance = std::numeric_limits<double>::max();
        bool is_inside = false;
        const std::size_t count = vertices_.size();
        for (std::size_t index = 0U, previous = count - 1U; index < count;
             previous = index++)
        {
            const Vector3& start = vertices_.at(previous);
            const Vector3& end = vertices_.at(index);
            minimum_distance = std::min(
                minimum_distance, point_segment_distance(position, start, end));
            const bool crosses = (end.at(1) > position.at(1)) !=
                (start.at(1) > position.at(1));
            if (crosses)
            {
                const double north_at_crossing = end.at(0) +
                    (position.at(1) - end.at(1)) *
                    (start.at(0) - end.at(0)) / (start.at(1) - end.at(1));
                if (position.at(0) < north_at_crossing)
                {
                    is_inside = !is_inside;
                }
            }
        }
        return is_inside ? minimum_distance : -minimum_distance;
    }

    /**
     * @brief 檢查位置是否在圍籬內且 clearance 足夠
     * @param position planner NED 位置
     * @param required_clearance_m 需要的最小 clearance
     * @return true 表示通過
     */
    bool contains(
        const Vector3& position,
        const double required_clearance_m) const
    {
        const double clearance = clearance_m(position);
        return is_finite(clearance) && is_finite(required_clearance_m) &&
            clearance + EGO_SWARM_EPSILON_M >= required_clearance_m;
    }

    /**
     * @brief 檢查位置是否滿足硬性 margin
     * @param position planner NED 位置
     * @return true 表示在圍籬內且 clearance ≥ margin
     */
    bool contains_with_margin(const Vector3& position) const
    {
        return contains(position, margin_m_);
    }

    /**
     * @brief 依起點決定本次軌跡需要的 clearance
     *
     * 起點已在 margin 帶內或圍籬外時，仍要允許往內修正；因此門檻放寬為
     * 起點 clearance 減去小容差，但絕不高於 margin。
     *
     * @param start_clearance_m 起點 clearance
     * @return 本次軌跡每個取樣點需要的最小 clearance
     */
    double required_clearance_from_start(const double start_clearance_m) const
    {
        if (!is_finite(start_clearance_m))
        {
            return margin_m_;
        }
        return std::min(
            margin_m_, start_clearance_m - GEOFENCE_RECOVERY_TOLERANCE_M);
    }

    /**
     * @brief 檢查水平直線段是否全程滿足 clearance
     *
     * 兩端點都在圍籬內且線段與每條邊的距離都不小於 clearance 時，
     * 線段不會穿越邊界（凹多邊形亦然）。
     *
     * @param start 線段起點
     * @param end 線段終點
     * @param required_clearance_m 需要的最小 clearance（≥ 0）
     * @return true 表示整段通過
     */
    bool segment_inside(
        const Vector3& start,
        const Vector3& end,
        const double required_clearance_m) const
    {
        if (!contains(start, required_clearance_m) ||
            !contains(end, required_clearance_m))
        {
            return false;
        }
        const std::size_t count = vertices_.size();
        for (std::size_t index = 0U, previous = count - 1U; index < count;
             previous = index++)
        {
            const double distance = segment_segment_distance(
                start, end, vertices_.at(previous), vertices_.at(index));
            if (!is_finite(distance) ||
                distance + EGO_SWARM_EPSILON_M < required_clearance_m ||
                distance <= EGO_SWARM_EPSILON_M)
            {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief 以 planner frame 把 WGS84 頂點轉成圍籬
     * @param frame 已凍結原點的 planner frame
     * @param vertices WGS84 頂點
     * @param margin_m 硬性 margin
     * @param[out] out_fence 建立結果；失敗時為無效圍籬
     * @return true 表示建立成功
     */
    static bool from_geodetic(
        const LocalNedFrame& frame,
        const std::vector<GeoCoordinate>& vertices,
        const double margin_m,
        Geofence& out_fence)
    {
        out_fence = Geofence{};
        std::vector<Vector3> vertices_ned;
        vertices_ned.reserve(vertices.size());
        for (const GeoCoordinate& vertex : vertices)
        {
            GeoCoordinate horizontal = vertex;
            horizontal.altitude_m = 0.0;
            Vector3 ned{};
            if (!frame.to_local_ned(horizontal, ned) ||
                std::hypot(ned.at(0), ned.at(1)) >
                    GEOFENCE_MAX_VERTEX_DISTANCE_M)
            {
                return false;
            }
            vertices_ned.push_back(ned);
        }
        return out_fence.set_polygon(vertices_ned, margin_m);
    }

private:
    static double cross_2d(
        const Vector3& origin, const Vector3& first, const Vector3& second)
    {
        return (first.at(0) - origin.at(0)) * (second.at(1) - origin.at(1)) -
            (first.at(1) - origin.at(1)) * (second.at(0) - origin.at(0));
    }

    static double point_segment_distance(
        const Vector3& point, const Vector3& start, const Vector3& end)
    {
        const double delta_north = end.at(0) - start.at(0);
        const double delta_east = end.at(1) - start.at(1);
        const double length_squared =
            delta_north * delta_north + delta_east * delta_east;
        double ratio = 0.0;
        if (length_squared > EGO_SWARM_EPSILON_M)
        {
            ratio = ((point.at(0) - start.at(0)) * delta_north +
                     (point.at(1) - start.at(1)) * delta_east) /
                length_squared;
            ratio = std::max(0.0, std::min(1.0, ratio));
        }
        return std::hypot(
            point.at(0) - (start.at(0) + ratio * delta_north),
            point.at(1) - (start.at(1) + ratio * delta_east));
    }

    static bool segments_intersect(
        const Vector3& first_start, const Vector3& first_end,
        const Vector3& second_start, const Vector3& second_end)
    {
        const double d1 = cross_2d(second_start, second_end, first_start);
        const double d2 = cross_2d(second_start, second_end, first_end);
        const double d3 = cross_2d(first_start, first_end, second_start);
        const double d4 = cross_2d(first_start, first_end, second_end);
        return ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0));
    }

    static double segment_segment_distance(
        const Vector3& first_start, const Vector3& first_end,
        const Vector3& second_start, const Vector3& second_end)
    {
        if (segments_intersect(
                first_start, first_end, second_start, second_end))
        {
            return 0.0;
        }
        return std::min(
            std::min(point_segment_distance(
                         first_start, second_start, second_end),
                     point_segment_distance(
                         first_end, second_start, second_end)),
            std::min(point_segment_distance(
                         second_start, first_start, first_end),
                     point_segment_distance(
                         second_end, first_start, first_end)));
    }

    double signed_area_m2() const
    {
        double twice_area = 0.0;
        const std::size_t count = vertices_.size();
        for (std::size_t index = 0U, previous = count - 1U; index < count;
             previous = index++)
        {
            twice_area += vertices_.at(previous).at(0) *
                    vertices_.at(index).at(1) -
                vertices_.at(index).at(0) * vertices_.at(previous).at(1);
        }
        return 0.5 * twice_area;
    }

    bool is_self_intersecting() const
    {
        const std::size_t count = vertices_.size();
        for (std::size_t first = 0U; first < count; ++first)
        {
            const std::size_t first_next = (first + 1U) % count;
            for (std::size_t second = first + 1U; second < count; ++second)
            {
                const std::size_t second_next = (second + 1U) % count;
                if (second == first_next || second_next == first)
                {
                    continue;
                }
                if (segment_segment_distance(
                        vertices_.at(first), vertices_.at(first_next),
                        vertices_.at(second), vertices_.at(second_next)) <=
                    EGO_SWARM_EPSILON_M)
                {
                    return true;
                }
            }
        }
        return false;
    }

    std::vector<Vector3> vertices_{};
    double margin_m_{0.0};
    bool is_valid_{false};
};

}  // namespace ego_swarm
}  // namespace taiphoon
