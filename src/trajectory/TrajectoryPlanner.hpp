/**
 * @file TrajectoryPlanner.hpp
 * @brief 軌跡規劃器 — 純邏輯類別，規劃無碰撞直線路徑
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 不依賴 ROS2，僅執行路徑生成、碰撞檢查與時間計算。
 * 由 TrajectoryController 節點在控制迴圈中呼叫。
 */

#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <limits>

namespace taiphoon
{
namespace trajectory
{

/**
 * @brief Waypoint 結構
 *
 * 包含 NED 位置與預定到達時間（單位：秒）。
 * 時間為相對起始時間的偏移量，由 TrajectoryController 轉換為 ROS2 Time。
 */
struct Waypoint
{
    std::array<float, 3> position_ned{0.0F, 0.0F, 0.0F};
    double arrival_time_s{0.0};
};

/**
 * @brief 軌跡規劃器
 *
 * 提供靜態方法：
 * 1. 沿直線生成等距 waypoint 序列
 * 2. 計算 waypoint 到達時間（固定速度）
 * 3. 檢查 waypoint 與同儕位置的安全距離
 * 4. 找出不安全的 waypoint 索引
 */
class TrajectoryPlanner
{
public:
    /**
     * @brief 從目前位置到目標位置，沿直線生成等距 waypoint
     *
     * @param current_ned   目前 NED 位置
     * @param target_ned    目標 NED 位置
     * @param spacing_m     waypoint 間距（公尺），預設 1.0
     * @return std::vector<Waypoint> waypoint 序列（含起點與終點）
     */
    static std::vector<Waypoint> generate_path(
        const std::array<float, 3>& current_ned,
        const std::array<float, 3>& target_ned,
        float spacing_m = 1.0F)
    {
        std::vector<Waypoint> result;

        if (spacing_m <= 0.0F)
        {
            spacing_m = 1.0F;
        }

        float dx = target_ned[0] - current_ned[0];
        float dy = target_ned[1] - current_ned[1];
        float dz = target_ned[2] - current_ned[2];
        float total_dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        // 若距離為 0，僅回傳一個 waypoint（目前位置）
        if (total_dist < 1e-6F)
        {
            Waypoint wp;
            wp.position_ned = current_ned;
            result.push_back(wp);
            return result;
        }

        // 從起點到終點，以 spacing 為間隔
        float dist = 0.0F;
        while (dist < total_dist)
        {
            Waypoint wp;
            float t = dist / total_dist;
            wp.position_ned[0] = current_ned[0] + t * dx;
            wp.position_ned[1] = current_ned[1] + t * dy;
            wp.position_ned[2] = current_ned[2] + t * dz;
            result.push_back(wp);
            dist += spacing_m;
        }

        // 加入終點（即使在剛好整除的情況下也確保精確性）
        Waypoint last_wp;
        last_wp.position_ned = target_ned;
        result.push_back(last_wp);

        return result;
    }

    /**
     * @brief 為 waypoint 序列計算到達時間
     *
     * 假設固定速度，從起始時間開始計算每個 waypoint 的抵達時間。
     *
     * @param[in,out] waypoints  waypoint 序列（arrival_time_s 會被寫入）
     * @param start_time_s       起始時間（秒），預設 0.0
     * @param speed_mps          飛行速度（公尺/秒），預設 5.0
     */
    static void compute_arrival_times(
        std::vector<Waypoint>& waypoints,
        double start_time_s = 0.0,
        double speed_mps = 5.0)
    {
        if (waypoints.empty()) return;
        if (speed_mps <= 0.0) speed_mps = 5.0;

        waypoints[0].arrival_time_s = start_time_s;
        double cumulative_time = start_time_s;

        for (size_t i = 1; i < waypoints.size(); ++i)
        {
            auto& prev = waypoints[i - 1];
            auto& curr = waypoints[i];

            float dx = curr.position_ned[0] - prev.position_ned[0];
            float dy = curr.position_ned[1] - prev.position_ned[1];
            float dz = curr.position_ned[2] - prev.position_ned[2];
            float seg_dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            cumulative_time += static_cast<double>(seg_dist) / speed_mps;
            curr.arrival_time_s = cumulative_time;
        }
    }

    /**
     * @brief 檢查單一 waypoint 與所有同儕位置的安全距離
     *
     * @param wp_position  waypoint 的 NED 位置
     * @param peer_positions 所有同儕的 NED 位置列表
     * @param safety_margin_m 安全邊界（公尺）
     * @return true 所有同儕距離 ≥ 安全邊界
     */
    static bool is_waypoint_safe(
        const std::array<float, 3>& wp_position,
        const std::vector<std::array<float, 3>>& peer_positions,
        float safety_margin_m)
    {
        for (const auto& peer : peer_positions)
        {
            float dx = wp_position[0] - peer[0];
            float dy = wp_position[1] - peer[1];
            float dz = wp_position[2] - peer[2];
            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist < safety_margin_m)
            {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief 找出所有不安全的 waypoint 索引
     *
     * 遍歷 waypoint 序列，檢查每個 waypoint 與所有同儕位置的距離。
     *
     * @param waypoints    waypoint 序列
     * @param peer_positions 所有同儕的 NED 位置列表
     * @param safety_margin_m 安全邊界（公尺）
     * @return std::vector<size_t> 不安全 waypoint 的索引列表
     */
    static std::vector<size_t> find_unsafe_waypoints(
        const std::vector<Waypoint>& waypoints,
        const std::vector<std::array<float, 3>>& peer_positions,
        float safety_margin_m)
    {
        std::vector<size_t> unsafe_indices;

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            if (!is_waypoint_safe(
                    waypoints[i].position_ned, peer_positions, safety_margin_m))
            {
                unsafe_indices.push_back(i);
            }
        }

        return unsafe_indices;
    }
};

}  // namespace trajectory
}  // namespace taiphoon
