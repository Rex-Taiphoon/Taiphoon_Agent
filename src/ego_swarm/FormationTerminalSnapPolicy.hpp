/**
 * @file FormationTerminalSnapPolicy.hpp
 * @brief 編隊以容許誤差直接到位前的直線段同儕淨空判定
 * @author Taiphoon Agent Team
 * @date 2026-10-30
 *
 * 編隊到位使用 `global_arrival_tolerance_m`（預設 3 m），進入容許範圍後
 * 直接以槽位座標建立 terminal hold；PX4 會以位置控制飛完「目前位置 →
 * 槽位」這段直線，不經 optimizer 與 ContinuousSafetyValidator。本檔只提供
 * pure C++ 判定：這段直線與同儕即時位置及未來軌跡都保持碰撞距離時，
 * 才允許直接到位，否則由 node 繼續一般規劃。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "ego_swarm/EgoSwarmTypes.hpp"
#include "ego_swarm/PeerTrajectoryTable.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// 取樣同儕未來軌跡的時間長度（秒）；需涵蓋 PX4 以位置控制飛完 3 m 直線，並保留餘裕。
constexpr double FORMATION_TERMINAL_SNAP_PEER_LOOKAHEAD_S = 4.0;

/// 同儕軌跡取樣間隔（秒）；相鄰取樣點以線段相連，不會漏掉兩點之間的路徑。
constexpr double FORMATION_TERMINAL_SNAP_PEER_SAMPLE_STEP_S = 0.1;

/// 單一同儕最多取樣點數，避免異常參數造成過量計算。
constexpr std::size_t FORMATION_TERMINAL_SNAP_MAX_PEER_SAMPLES = 256U;

/**
 * @brief 同儕在直接到位期間可能佔用的一段路徑。
 *
 * 起點與終點相同時代表靜止點。
 */
struct FormationSnapPeerSegment
{
    Vector3 start{0.0, 0.0, 0.0};
    Vector3 end{0.0, 0.0, 0.0};
};

/**
 * @brief 計算兩個向量的內積
 * @param first 第一個向量
 * @param second 第二個向量
 * @return 內積
 */
inline double dot_vectors(const Vector3& first, const Vector3& second)
{
    return first.at(0) * second.at(0) +
        first.at(1) * second.at(1) +
        first.at(2) * second.at(2);
}

/**
 * @brief 計算兩條 3D 線段之間的最短距離
 *
 * 依線段最近點參數化求解並夾在 [0, 1]；平行或退化成點的線段也成立。
 * 任一輸入非有限值時回傳 0.0，讓呼叫端以 fail-closed 視為碰撞。
 *
 * @param first_start 第一條線段起點
 * @param first_end 第一條線段終點
 * @param second_start 第二條線段起點
 * @param second_end 第二條線段終點
 * @return 最短距離（公尺）
 */
inline double distance_between_segments(
    const Vector3& first_start,
    const Vector3& first_end,
    const Vector3& second_start,
    const Vector3& second_end)
{
    if (!is_finite_vector(first_start) || !is_finite_vector(first_end) ||
        !is_finite_vector(second_start) || !is_finite_vector(second_end))
    {
        return 0.0;
    }
    const Vector3 first_direction = subtract_vectors(first_end, first_start);
    const Vector3 second_direction = subtract_vectors(second_end, second_start);
    const Vector3 start_offset = subtract_vectors(first_start, second_start);
    const double first_length_squared = dot_vectors(first_direction, first_direction);
    const double second_length_squared = dot_vectors(second_direction, second_direction);
    const double second_offset = dot_vectors(second_direction, start_offset);

    double first_parameter = 0.0;
    double second_parameter = 0.0;
    if (first_length_squared <= EGO_SWARM_EPSILON_M &&
        second_length_squared <= EGO_SWARM_EPSILON_M)
    {
        return distance_between_centers(first_start, second_start);
    }
    if (first_length_squared <= EGO_SWARM_EPSILON_M)
    {
        second_parameter = std::clamp(second_offset / second_length_squared, 0.0, 1.0);
    }
    else
    {
        const double first_offset = dot_vectors(first_direction, start_offset);
        if (second_length_squared <= EGO_SWARM_EPSILON_M)
        {
            first_parameter = std::clamp(-first_offset / first_length_squared, 0.0, 1.0);
        }
        else
        {
            const double cross_term = dot_vectors(first_direction, second_direction);
            const double denominator =
                first_length_squared * second_length_squared - cross_term * cross_term;
            // 平行時任一最近點都成立，固定從第一條線段起點開始求解。
            if (denominator > EGO_SWARM_EPSILON_M)
            {
                first_parameter = std::clamp(
                    (cross_term * second_offset - first_offset * second_length_squared) /
                        denominator,
                    0.0, 1.0);
            }
            second_parameter =
                (cross_term * first_parameter + second_offset) / second_length_squared;
            if (second_parameter < 0.0)
            {
                second_parameter = 0.0;
                first_parameter = std::clamp(-first_offset / first_length_squared, 0.0, 1.0);
            }
            else if (second_parameter > 1.0)
            {
                second_parameter = 1.0;
                first_parameter = std::clamp(
                    (cross_term - first_offset) / first_length_squared, 0.0, 1.0);
            }
        }
    }
    const Vector3 first_closest = add_vectors(
        first_start, scale_vector(first_direction, first_parameter));
    const Vector3 second_closest = add_vectors(
        second_start, scale_vector(second_direction, second_parameter));
    const double distance = distance_between_centers(first_closest, second_closest);
    return is_finite(distance) ? distance : 0.0;
}

/**
 * @brief 把單一同儕在直接到位期間可能佔用的路徑加入清單
 *
 * 路徑由即時位置開始，接著依序連接 fresh trajectory 在
 * [now, now + lookahead] 的取樣點。同儕在 terminal hold 時，第一段即為
 * 「即時位置 → hold 槽位」，能涵蓋同儕本身也正在直接到位的情況。
 * 沒有 fresh trajectory 時只加入即時位置。
 *
 * @param live_position 同儕即時位置（planner-local NED）
 * @param has_trajectory 是否有 fresh trajectory
 * @param trajectory 同儕 fresh trajectory；has_trajectory 為 false 時不使用
 * @param now_s 共同時間（秒）
 * @param lookahead_s 取樣時間長度（秒）
 * @param step_s 取樣間隔（秒）
 * @param segments 輸出清單；成功時附加路徑線段
 * @return 任一輸入或取樣無效時回傳 false，呼叫端須視為不可直接到位
 */
inline bool append_formation_snap_peer_path(
    const Vector3& live_position,
    const bool has_trajectory,
    const PeerTrajectory& trajectory,
    const double now_s,
    const double lookahead_s,
    const double step_s,
    std::vector<FormationSnapPeerSegment>& segments)
{
    if (!is_finite_vector(live_position))
    {
        return false;
    }
    if (!has_trajectory)
    {
        segments.push_back(FormationSnapPeerSegment{live_position, live_position});
        return true;
    }
    if (!is_finite(now_s) || !is_finite(lookahead_s) || lookahead_s < 0.0 ||
        !is_finite(step_s) || step_s <= 0.0 ||
        !is_finite(trajectory.start_time_s) || !trajectory.trajectory.is_valid())
    {
        return false;
    }
    const double step_count = std::ceil(lookahead_s / step_s);
    if (!is_finite(step_count) ||
        step_count + 1.0 > static_cast<double>(FORMATION_TERMINAL_SNAP_MAX_PEER_SAMPLES))
    {
        return false;
    }
    const std::size_t sample_count = static_cast<std::size_t>(step_count) + 1U;
    std::vector<FormationSnapPeerSegment> peer_segments;
    peer_segments.reserve(sample_count);
    Vector3 previous = live_position;
    for (std::size_t index = 0U; index < sample_count; ++index)
    {
        const double absolute_time_s = now_s + std::min(
            lookahead_s, static_cast<double>(index) * step_s);
        // 尚未開始的軌跡取起點，已結束的 hold 取終點，與 PeerTrajectoryTable 語意一致。
        const TrajectorySample sample = trajectory.trajectory.evaluate(
            std::max(0.0, absolute_time_s - trajectory.start_time_s));
        if (!sample.is_valid || !is_finite_vector(sample.position))
        {
            return false;
        }
        peer_segments.push_back(FormationSnapPeerSegment{previous, sample.position});
        previous = sample.position;
    }
    segments.insert(segments.end(), peer_segments.begin(), peer_segments.end());
    return true;
}

/**
 * @brief 判斷「目前位置 → 槽位」直線是否可直接交給 PX4 位置控制飛完
 *
 * 直線段與每段同儕路徑的最短距離都必須不小於淨空距離。沒有同儕時
 * 視為淨空；任一輸入無效時 fail-closed 回傳 false。
 *
 * @param current_position 本機目前位置（planner-local NED）
 * @param slot_position 槽位座標（planner-local NED）
 * @param peer_segments 同儕可能佔用的路徑
 * @param clearance_m 淨空距離（公尺），必須為正
 * @return true 表示可直接建立槽位 terminal hold
 */
inline bool is_formation_terminal_snap_clear(
    const Vector3& current_position,
    const Vector3& slot_position,
    const std::vector<FormationSnapPeerSegment>& peer_segments,
    const double clearance_m)
{
    if (!is_finite_vector(current_position) || !is_finite_vector(slot_position) ||
        !is_finite(clearance_m) || clearance_m <= 0.0)
    {
        return false;
    }
    return std::all_of(
        peer_segments.begin(), peer_segments.end(),
        [&current_position, &slot_position, clearance_m](
            const FormationSnapPeerSegment& segment)
        {
            return is_finite_vector(segment.start) && is_finite_vector(segment.end) &&
                distance_between_segments(
                    current_position, slot_position,
                    segment.start, segment.end) >= clearance_m;
        });
}

}  // namespace ego_swarm
}  // namespace taiphoon
