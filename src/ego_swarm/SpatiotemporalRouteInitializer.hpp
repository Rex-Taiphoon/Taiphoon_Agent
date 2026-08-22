/**
 * @file SpatiotemporalRouteInitializer.hpp
 * @brief FIXED_Z 時空路徑 seed 的 pure C++ interface
 *
 * 本 module 只負責在直線 seed 發生碰撞時，產生可供 B-spline optimizer
 * 使用的 timed route。它不負責 safety acceptance、reservation 或 PX4。
 */

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "ego_swarm/PlannerTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// 時空 route initializer 的結束原因。
enum class RouteInitializerTerminationReason
{
    SUCCESS = 0,
    INVALID_INPUT,
    NO_FEASIBLE_PATH,
    TIMEOUT
};

inline std::string route_initializer_reason_string(
    const RouteInitializerTerminationReason reason)
{
    switch (reason)
    {
        case RouteInitializerTerminationReason::SUCCESS:
            return "success";
        case RouteInitializerTerminationReason::INVALID_INPUT:
            return "invalid input";
        case RouteInitializerTerminationReason::NO_FEASIBLE_PATH:
            return "no feasible path";
        case RouteInitializerTerminationReason::TIMEOUT:
            return "timeout";
    }
    return "unknown";
}

/// 帶有相對於 candidate trajectory start time 的 route sample。
struct TimedRoutePoint
{
    Vector3 position{0.0, 0.0, 0.0};
    double time_s{0.0};
};

/// 時空 route 搜尋結果；尚未經 B-spline optimizer 或 safety validator 驗證。
struct SpatiotemporalRouteResult
{
    bool is_valid{false};
    bool used_search{false};
    double trajectory_start_time_s{0.0};
    double duration_s{0.0};
    double path_length_m{0.0};
    std::size_t expanded_nodes{0U};
    double search_time_ms{0.0};
    RouteInitializerTerminationReason termination{
        RouteInitializerTerminationReason::INVALID_INPUT};
    std::vector<TimedRoutePoint> route{};
};

/**
 * @brief 依 FIXED_Z snapshot 產生 bounded 時空 route seed。
 *
 * 直線無衝突時回傳 valid 且 used_search=false，呼叫端應保留原始 seed。
 * 發生衝突時，搜尋只允許起點延遲，不產生途中停等點；每個成功 route
 * 仍必須交給既有 B-spline optimizer 與 ContinuousSafetyValidator。
 */
class SpatiotemporalRouteInitializer
{
public:
    /**
     * @brief 產生單一 Local Goal window 的 timed route。
     * @param snapshot immutable planner cycle 資料
     * @param config planner 與 safety 共用設定
     * @param straight_seed 目前 planner 建立的直線 control-point seed
     * @return route result；失敗不得視為可執行軌跡
     */
    SpatiotemporalRouteResult initialize(
        const PlanningSnapshot& snapshot,
        const PlannerConfig& config,
        const std::vector<Vector3>& straight_seed) const;
};

}  // namespace ego_swarm
}  // namespace taiphoon
