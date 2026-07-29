/**
 * @file PositionSafetyMonitor.hpp
 * @brief 位置安全監控器 — 純邏輯類別，監控 NED 位置偏差
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 不依賴 ROS2，僅執行歐氏距離計算與閾值比較。
 * 由 BasicControllerNode 在控制迴圈中呼叫。
 */

#pragma once

#include <array>
#include <cmath>

namespace taiphoon
{
namespace safety
{

/**
 * @brief 位置安全監控器
 *
 * 比較目前位置與目標位置的 NED 歐氏距離，超過閾值時回報偏差超標。
 * 設計為純邏輯類別，便於單元測試。
 */
class PositionSafetyMonitor
{
public:
    /**
     * @brief 建構子
     *
     * @param threshold_m 偏差閾值（公尺），預設 2.0
     */
    explicit PositionSafetyMonitor(float threshold_m = 2.0F)
        : threshold_m_(threshold_m)
    {
    }

    /**
     * @brief 檢查目前位置與目標位置的偏差是否超過閾值
     *
     * 偏差 ≥ 閾值時回傳 true。
     *
     * @param current_ned 目前 NED 位置 [x, y, z]（公尺）
     * @param target_ned  目標 NED 位置 [x, y, z]（公尺）
     * @return true  偏差超過或等於閾值
     * @return false 偏差未達閾值
     */
    bool is_deviation_exceeded(
        const std::array<float, 3>& current_ned,
        const std::array<float, 3>& target_ned) const
    {
        return compute_distance(current_ned, target_ned) >= threshold_m_;
    }

    /**
     * @brief 計算兩點之間的歐氏距離
     *
     * @param a 點 A 的 NED 座標 [x, y, z]（公尺）
     * @param b 點 B 的 NED 座標 [x, y, z]（公尺）
     * @return float 歐氏距離（公尺）
     */
    static float compute_distance(
        const std::array<float, 3>& a,
        const std::array<float, 3>& b)
    {
        float dx = a[0] - b[0];
        float dy = a[1] - b[1];
        float dz = a[2] - b[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    /**
     * @brief 取得目前設定的偏差閾值
     *
     * @return float 閾值（公尺）
     */
    float get_threshold() const { return threshold_m_; }

private:
    float threshold_m_; ///< 偏差閾值（公尺）
};

}  // namespace safety
}  // namespace taiphoon
