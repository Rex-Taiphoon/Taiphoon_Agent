/**
 * @file FormationCalculator.hpp
 * @brief 編隊計算器 — 純邏輯類別，計算質心與編隊目標位置
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 不依賴 ROS2，僅執行編隊數學計算。
 * 由 MissionPlanner 節點在編隊控制迴圈中呼叫。
 */

#pragma once

#include <algorithm>
#include <vector>
#include <array>
#include <cmath>
#include <chrono>
#include <limits>
#include <cstdint>

namespace taiphoon
{
namespace formation
{

/// 編隊類型
enum class FormationType : uint8_t
{
    NONE = 0,
    CIRCLE,
    HORI_LINEAR,
    VERT_LINEAR
};

/// 同儕無人機位置結構
struct PeerPosition
{
    std::array<float, 3> position_ned{0.0F, 0.0F, 0.0F};
    bool online{false};
    /// 全域 GPS 座標 [lat(deg), lon(deg), alt(m AMSL)]，無效時為 NaN。
    /// 置於 online 之後以維持既有聚合初始化（{position_ned, online}）相容。
    std::array<float, 3> position_global{
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN()};
};

/// 編隊參數
struct FormationParams
{
    float param1{0.0F};  ///< 半徑 (CIRCLE) 或間距 (LINEAR)
    float param2{0.0F};  ///< 高度 (CIRCLE 與 HORI_LINEAR)
};

/// 編隊質心快照結果
struct ReferenceCentroid
{
    std::array<float, 3> centroid{0.0F, 0.0F, 0.0F};
    bool gps_based{false};  ///< 是否以 WGS84 交換後轉入自身 local frame 計算
};

constexpr float PI = 3.14159265358979323846F;
/// 等距圓柱投影：緯度每 1 度對應的北向距離（公尺）
constexpr float METERS_PER_DEG_LAT = 111320.0F;
/// 編隊任兩個目標 slot 的最小三維安全距離（公尺）
constexpr float MIN_FORMATION_SLOT_DISTANCE_M = 3.0F;
/// 浮點距離邊界比較容差（公尺）
constexpr float FORMATION_DISTANCE_EPSILON_M = 1.0e-5F;

/**
 * @brief 編隊計算器
 *
 * 提供靜態方法計算：
 * 1. 多機質心 (centroid)
 * 2. 各種編隊類型下的目標位置
 */
class FormationCalculator
{
public:
    /**
     * @brief 計算線上無人機數量
     *
     * @param peers 同儕位置列表
     * @return size_t 線上數量
     */
    static size_t count_online(const std::vector<PeerPosition>& peers)
    {
        size_t count = 0;
        for (const auto& p : peers)
        {
            if (p.online) ++count;
        }
        return count;
    }

    /**
     * @brief 計算所有線上無人機的質心
     *
     * 僅計算 online=true 的無人機。若無線上無人機，回傳 (0,0,0)。
     *
     * @param peers 同儕位置列表
     * @return std::array<float, 3> 質心 NED 座標 (x, y, z)
     */
    static std::array<float, 3> compute_centroid(
        const std::vector<PeerPosition>& peers)
    {
        std::array<float, 3> centroid{0.0F, 0.0F, 0.0F};
        size_t count = 0;

        for (const auto& p : peers)
        {
            if (!p.online) continue;
            centroid[0] += p.position_ned[0];
            centroid[1] += p.position_ned[1];
            centroid[2] += p.position_ned[2];
            ++count;
        }

        if (count > 0)
        {
            centroid[0] /= static_cast<float>(count);
            centroid[1] /= static_cast<float>(count);
            centroid[2] /= static_cast<float>(count);
        }

        return centroid;
    }

    /**
     * @brief 計算自身全域座標到同儕全域座標的 NED 偏移（公尺）
     *
     * 使用等距圓柱投影（equirectangular approximation），適用於
     * 機群間距僅數公尺的編隊場景。Z 軸換算：NED 向下為正，
     * 同儕高度高於自身時偏移為負。
     *
     * @param own_global  自身全域座標 [lat, lon, alt]
     * @param peer_global 同儕全域座標 [lat, lon, alt]
     * @return std::array<float, 3> NED 偏移 (x=北, y=東, z=下)
     */
    static std::array<float, 3> gps_offset_meters(
        const std::array<float, 3>& own_global,
        const std::array<float, 3>& peer_global)
    {
        std::array<float, 3> offset{0.0F, 0.0F, 0.0F};

        // 緯度/經度任一無效則僅回傳零偏移（由呼叫端決定是否退回原算法）
        if (!std::isnan(own_global[0]) && !std::isnan(own_global[1]) &&
            !std::isnan(peer_global[0]) && !std::isnan(peer_global[1]))
        {
            float cos_lat = std::cos(own_global[0] * PI / 180.0F);
            offset[0] = (peer_global[0] - own_global[0]) * METERS_PER_DEG_LAT;
            offset[1] = (peer_global[1] - own_global[1]) *
                        METERS_PER_DEG_LAT * cos_lat;
        }

        // 高度有效時才換算 Z（AMSL 差為負的 NED Z 偏移）
        if (!std::isnan(own_global[2]) && !std::isnan(peer_global[2]))
        {
            offset[2] = own_global[2] - peer_global[2];
        }

        return offset;
    }

    /**
     * @brief 將同儕 WGS84 位置換算到自身 local frame
     *
     * 各機 position_ned 以自身 HOME 為原點，直接混算質心會造成
     * 編隊圓心在全球座標下偏移；以 GPS 差補正後，所有位置都在
     * 自身 local frame 中可比：common_j = own_ned + gps_offset。
     *
     * @param own_position_ned 自身目前 NED 位置（公尺）
     * @param own_global       自身全域座標 [lat, lon, alt]
     * @param peer_global      同儕全域座標 [lat, lon, alt]
     * @param[out] out_common  換算後的同儕位置（自身 local frame）
     * @return true 換算成功（緯度/經度皆有效）；false 時 out_common 未定義
     */
    static bool compute_common_frame_position(
        const std::array<float, 3>& own_position_ned,
        const std::array<float, 3>& own_global,
        const std::array<float, 3>& peer_global,
        std::array<float, 3>& out_common)
    {
        if (std::isnan(own_global[0]) || std::isnan(own_global[1]) ||
            std::isnan(peer_global[0]) || std::isnan(peer_global[1]))
        {
            return false;
        }

        auto offset = gps_offset_meters(own_global, peer_global);
        out_common[0] = own_position_ned[0] + offset[0];
        out_common[1] = own_position_ned[1] + offset[1];
        out_common[2] = own_position_ned[2] + offset[2];
        return true;
    }

    /**
     * @brief 判斷同儕 state 是否已逾時（離線加速器）
     *
     * 超過 timeout_s 未收到 AgentState 即視為逾時。從未收到過
     * （time_point 預設值）亦視為逾時。
     *
     * @param last_arrival 最後一次收到 state 的時間（steady_clock）
     * @param now          目前時間（steady_clock）
     * @param timeout_s    逾時秒數
     * @return true 表示已逾時（離線）
     */
    static bool is_peer_stale(
        std::chrono::steady_clock::time_point last_arrival,
        std::chrono::steady_clock::time_point now,
        double timeout_s)
    {
        if (last_arrival == std::chrono::steady_clock::time_point{})
        {
            return true;
        }
        double elapsed_s =
            std::chrono::duration<double>(now - last_arrival).count();
        return elapsed_s > timeout_s;
    }

    /**
     * @brief 計算編隊質心快照（指令當下或成員變化時呼叫）
     *
     * 將所有線上同儕的 WGS84 位置換算到自身 local frame 後取平均，
     * 作為編隊的固定參考點；任一同儕 GPS 無效時退回原混框算法（直接平均
     * 各機 local 位置）並回傳 gps_based=false。
     *
     * @param peers            同儕位置列表（含自身，含 online 旗標）
     * @param own_position_ned 自身目前 NED 位置（公尺）
     * @param own_global       自身全域座標 [lat, lon, alt]
     * @return ReferenceCentroid 快照質心與計算方式
     */
    static ReferenceCentroid compute_reference_centroid(
        const std::vector<PeerPosition>& peers,
        const std::array<float, 3>& own_position_ned,
        const std::array<float, 3>& own_global)
    {
        ReferenceCentroid result;

        // 檢查所有線上同儕的 GPS 是否有效
        bool is_gps_valid = true;
        for (const auto& peer : peers)
        {
            if (!peer.online) continue;
            if (std::isnan(peer.position_global[0]) ||
                std::isnan(peer.position_global[1]))
            {
                is_gps_valid = false;
                break;
            }
        }

        if (is_gps_valid)
        {
            std::vector<PeerPosition> ref_peers = peers;
            for (auto& peer : ref_peers)
            {
                if (!peer.online) continue;
                std::array<float, 3> common_position{};
                if (compute_common_frame_position(
                        own_position_ned, own_global,
                        peer.position_global, common_position))
                {
                    peer.position_ned = common_position;
                }
            }
            result.centroid = compute_centroid(ref_peers);
            result.gps_based = true;
        }
        else
        {
            result.centroid = compute_centroid(peers);
            result.gps_based = false;
        }

        return result;
    }

    /**
     * @brief 計算編隊中此無人機的目標位置
     *
     * @param centroid    所有無人機的質心 NED
     * @param type        編隊類型
     * @param params      編隊參數 (半徑/間距/高度)
     * @param self_index  自身在編隊中的索引 (0-based)
     * @param online_count 線上無人機總數
     * @return std::array<float, 4> 目標 NED 位置 (x, y, z, yaw_deg)
     *         yaw_deg = NaN 表示不控制偏航
     */
    static std::array<float, 4> compute_formation_target(
        const std::array<float, 3>& centroid,
        FormationType type,
        const FormationParams& params,
        size_t self_index,
        size_t online_count)
    {
        std::array<float, 4> target{
            centroid[0], centroid[1], centroid[2],
            std::numeric_limits<float>::quiet_NaN()};

        if (online_count == 0) return target;

        switch (type)
        {
            case FormationType::CIRCLE:
            {
                // 環形編隊：等角分布
                float angle = 2.0F * PI *
                    static_cast<float>(self_index) /
                    static_cast<float>(online_count);
                float radius = params.param1;
                target[0] = centroid[0] + radius * std::cos(angle);
                target[1] = centroid[1] + radius * std::sin(angle);
                target[2] = -params.param2;  // NED: Z 向下為正，高度取負
                break;
            }

            case FormationType::HORI_LINEAR:
            {
                // 水平直列：沿 Y 軸排列
                float spacing = params.param1;
                float mid = (static_cast<float>(online_count) - 1.0F) / 2.0F;
                float offset = spacing *
                    (static_cast<float>(self_index) - mid);
                target[0] = centroid[0];
                target[1] = centroid[1] + offset;
                target[2] = -params.param2;
                break;
            }

            case FormationType::VERT_LINEAR:
            {
                // 垂直直列：沿 Z 軸排列
                float spacing = params.param1;
                float mid = (static_cast<float>(online_count) - 1.0F) / 2.0F;
                float offset = spacing *
                    (static_cast<float>(self_index) - mid);
                target[0] = centroid[0];
                target[1] = centroid[1];
                target[2] = centroid[2] + offset;
                break;
            }

            default:
                // NONE → 回傳質心
                break;
        }

        return target;
    }

    /**
     * @brief 判斷編隊中任兩個目標 slot 是否符合三維安全距離
     *
     * @param type 編隊類型
     * @param params 編隊參數（半徑／間距／高度）
     * @param slot_count 目標 slot 數量
     * @param minimum_distance_m 最小允許三維距離（公尺）
     * @return true 表示所有 slot 間距皆符合安全距離；false 表示不安全或輸入無效
     */
    static bool is_formation_geometry_safe(
        FormationType type,
        const FormationParams& params,
        size_t slot_count,
        float minimum_distance_m = MIN_FORMATION_SLOT_DISTANCE_M)
    {
        if (type == FormationType::NONE || slot_count < 2)
        {
            return true;
        }
        if (!std::isfinite(minimum_distance_m))
        {
            return false;
        }
        const float effective_minimum_distance_m = std::max(
            minimum_distance_m, MIN_FORMATION_SLOT_DISTANCE_M);

        const std::array<float, 3> centroid{0.0F, 0.0F, 0.0F};
        std::vector<std::array<float, 4>> targets;
        targets.reserve(slot_count);
        for (size_t index = 0; index < slot_count; ++index)
        {
            auto target = compute_formation_target(
                centroid, type, params, index, slot_count);
            if (!std::isfinite(target.at(0)) ||
                !std::isfinite(target.at(1)) ||
                !std::isfinite(target.at(2)))
            {
                return false;
            }
            targets.push_back(target);
        }

        for (size_t first = 0; first < targets.size(); ++first)
        {
            for (size_t second = first + 1; second < targets.size(); ++second)
            {
                float dx = targets.at(first).at(0) - targets.at(second).at(0);
                float dy = targets.at(first).at(1) - targets.at(second).at(1);
                float dz = targets.at(first).at(2) - targets.at(second).at(2);
                float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (distance + FORMATION_DISTANCE_EPSILON_M <
                    effective_minimum_distance_m)
                {
                    return false;
                }
            }
        }
        return true;
    }
};

}  // namespace formation
}  // namespace taiphoon
