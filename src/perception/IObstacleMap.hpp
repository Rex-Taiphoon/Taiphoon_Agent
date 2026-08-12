/**
 * @file IObstacleMap.hpp
 * @brief 感測器無關的局部障礙物地圖查詢介面
 * @author Taiphoon Agent Team
 * @date 2026-08-12
 *
 * 此介面只描述 planner 需要的障礙物查詢，不暴露 PointCloud2 或任何
 * 感測器來源，讓模擬資料與未來真實感測器可以共用相同的規劃資料流。
 */

#pragma once

#include <cstddef>

#include "ego_swarm/EgoSwarmTypes.hpp"

namespace taiphoon
{
namespace perception
{

/**
 * @brief 局部障礙物地圖的生命週期狀態
 */
enum class ObstacleMapStatus
{
    Missing,
    Fresh,
    Stale,
    Invalid
};

/**
 * @brief planner 使用的純 C++ 局部障礙物地圖抽象
 *
 * 地圖來源必須先完成 frame、數值與 timestamp 驗證，再透過此介面提供
 * 查詢。`Missing` 或 `Stale` 不得被呼叫端當成 explicit empty map。
 */
class IObstacleMap
{
public:
    virtual ~IObstacleMap() = default;

    /**
     * @brief 檢查地圖內容是否曾被成功更新
     *
     * @return true 表示地圖內容有效；不代表目前仍然 fresh
     */
    virtual bool is_valid() const = 0;

    /**
     * @brief 檢查地圖在指定共同時間是否仍然新鮮
     *
     * @param now_s planner 的共同時間，單位為秒
     * @return true 表示有效且未超過 freshness timeout
     */
    virtual bool is_fresh(double now_s) const = 0;

    /**
     * @brief 取得地圖目前的生命週期狀態
     *
     * @param now_s planner 的共同時間，單位為秒
     * @return Missing、Fresh、Stale 或 Invalid
     */
    virtual ObstacleMapStatus status(double now_s) const = 0;

    /**
     * @brief 取得有效地圖中的障礙物點數
     *
     * @return 障礙物點數；empty map 為零
     */
    virtual std::size_t point_count() const = 0;

    /**
     * @brief 查詢位置到最近障礙物點的距離
     *
     * @param position 待查詢的 swarm NED 位置
     * @return 最近距離；有效空地圖回傳正無限大，無效地圖回傳 NaN
     */
    virtual double nearest_distance(
        const ego_swarm::Vector3& position) const = 0;

    /**
     * @brief 檢查位置是否滿足障礙物 clearance
     *
     * @param position 待查詢的 swarm NED 位置
     * @param clearance_m 所需最小距離，單位為公尺
     * @return true 表示位置安全；無效地圖一律回傳 false
     */
    virtual bool is_position_safe(
        const ego_swarm::Vector3& position,
        double clearance_m) const = 0;

    /**
     * @brief 計算位置的障礙物距離成本
     *
     * @param position 待查詢的 swarm NED 位置
     * @return 所有障礙物點的 inverse-distance 成本；empty map 為零
     */
    virtual double obstacle_cost(
        const ego_swarm::Vector3& position) const = 0;
};

}  // namespace perception
}  // namespace taiphoon
