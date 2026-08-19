/**
 * @file RawPointObstacleMap.hpp
 * @brief 以有限三維點表示的局部障礙物地圖
 * @author Taiphoon Agent Team
 * @date 2026-08-12
 *
 * 此類別是 raw-point backend。planner 只依賴 IObstacleMap，
 * 未來可以在不修改 planner API 的情況下替換為 voxel 或 spatial map。
 */

#pragma once

#include <vector>

#include "perception/IObstacleMap.hpp"

namespace taiphoon
{
namespace perception
{

constexpr double DEFAULT_OBSTACLE_MAP_TIMEOUT_S = 1.0;

/**
 * @brief 由有限 swarm NED 點集合構成的局部障礙物地圖
 */
class RawPointObstacleMap final : public IObstacleMap
{
public:
    /**
     * @brief 建立尚未收到資料的地圖
     *
     * @param freshness_timeout_s 地圖 freshness timeout，單位為秒
     */
    explicit RawPointObstacleMap(
        double freshness_timeout_s = DEFAULT_OBSTACLE_MAP_TIMEOUT_S);

    /**
     * @brief 設定地圖 freshness timeout
     *
     * @param freshness_timeout_s timeout，必須為有限正值，單位為秒
     * @return true 表示設定已套用
     */
    bool set_freshness_timeout(double freshness_timeout_s);

    /**
     * @brief 以一筆已驗證 timestamp 更新地圖
     *
     * 空的 points 仍會建立 valid map，這是 explicit empty 的核心語意。
     * 
     * @param points swarm NED 障礙物點
     * @param stamp_s 地圖時間戳，單位為秒
     * @return true 表示資料已接受
     */
    bool update(
        const std::vector<ego_swarm::Vector3>& points,
        double stamp_s);

    /**
     * @brief 明確建立沒有障礙物的有效地圖
     *
     * @param stamp_s empty map 的時間戳，單位為秒
     * @return true 表示 empty map 已建立
     */
    bool set_explicit_empty(double stamp_s);

    /**
     * @brief 將地圖標記為無效，避免沿用未確認的內容
     *
     * @return true 表示地圖已進入 Invalid 狀態
     */
    bool invalidate();

    /**
     * @brief 檢查地圖內容是否曾被成功更新
     * @return true 表示地圖內容有效
     */
    bool is_valid() const override;

    /**
     * @brief 檢查地圖是否仍然 fresh
     * @param now_s planner 的共同時間，單位為秒
     * @return true 表示地圖有效且未逾時
     */
    bool is_fresh(double now_s) const override;

    /**
     * @brief 取得地圖生命週期狀態
     * @param now_s planner 的共同時間，單位為秒
     * @return Missing、Fresh、Stale 或 Invalid
     */
    ObstacleMapStatus status(double now_s) const override;

    /**
     * @brief 取得有效地圖中的點數
     * @return 障礙物點數
     */
    std::size_t point_count() const override;

    /**
     * @brief 查詢最近障礙物點距離
     * @param position 待查詢的 swarm NED 位置
     * @return 最近距離；empty map 為正無限大
     */
    double nearest_distance(
        const ego_swarm::Vector3& position) const override;

    /**
     * @brief 檢查位置是否滿足 clearance
     * @param position 待查詢的 swarm NED 位置
     * @param clearance_m 最小 clearance，單位為公尺
     * @return true 表示位置安全
     */
    bool is_position_safe(
        const ego_swarm::Vector3& position,
        double clearance_m) const override;

    /**
     * @brief 計算位置的 inverse-distance 障礙物成本
     * @param position 待查詢的 swarm NED 位置
     * @return 障礙物成本
     */
    double obstacle_cost(
        const ego_swarm::Vector3& position) const override;

private:
    bool has_received_{false};
    bool is_valid_{false};
    double freshness_timeout_s_{DEFAULT_OBSTACLE_MAP_TIMEOUT_S};
    double stamp_s_{0.0};
    std::vector<ego_swarm::Vector3> points_{};
};

}  // namespace perception
}  // namespace taiphoon
