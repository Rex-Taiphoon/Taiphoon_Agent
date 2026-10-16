/**
 * @file DdsPeerRelayPlan.hpp
 * @brief Docker Desktop 模擬的 DDS 同儕單播轉送埠段計算
 * @author Taiphoon Agent Team
 * @date 2026-10-16
 *
 * Docker Desktop 模擬為了讓主機上的 Taiphoon_QGCS 連線，每台 Agent 都以
 * 相同的 ExternalNetworkAddress（主機區網 IPv4）公告自己。CycloneDDS 0.10
 * 收到「等於自己外部位址」的同儕 locator 時，會判定為同一台機器並改寫成
 * 自己的容器內部 IP，結果 Agent 之間的單播全部送回自己，彼此無法發現。
 *
 * 本模組只計算「其他 Agent 的埠段」；轉送程式在本機內部 IP 綁定這些埠，
 * 把被改寫的封包原封不動送到該埠段所屬 Agent 容器在 Docker 網路上的位址
 * （以主機名稱解析），無法解析時才退回主機區網 IP 的埠映射。埠段公式必須與
 * agent_launcher 及 docker-compose.local-qgcs.yml 相同。
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace taiphoon
{
namespace utils
{

/// DDS 埠號基準（RTPS PB）
constexpr unsigned int DDS_RELAY_PORT_BASE = 7400U;
/// 每個 ROS_DOMAIN_ID 的埠號間距（RTPS DG）
constexpr unsigned int DDS_RELAY_DOMAIN_GAIN = 250U;
/// 單播埠相對 domain 基準的偏移（RTPS d1 = 10）
constexpr unsigned int DDS_RELAY_UNICAST_OFFSET = 10U;
/// 每個 participant index 佔用的埠數（meta + data）
constexpr unsigned int DDS_RELAY_PARTICIPANT_PORT_STEP = 2U;
/// 每台 Agent 可使用的最大 participant index（與啟動器 HOST_GCS_MAX_PARTICIPANT_INDEX 相同）
constexpr unsigned int DDS_RELAY_MAX_PARTICIPANT_INDEX = 11U;
/// 每台 Agent 專屬埠段長度
constexpr unsigned int DDS_RELAY_AGENT_PORT_SPAN =
    (DDS_RELAY_MAX_PARTICIPANT_INDEX + 1U) * DDS_RELAY_PARTICIPANT_PORT_STEP;
/// 單一主機模擬支援的最大 Agent 數
constexpr unsigned int DDS_RELAY_MAX_INSTANCES = 4U;
/// 使埠號仍落在 uint16 範圍內的最大 domain（Cyclone 亦限制 domain <= 232）
constexpr unsigned int DDS_RELAY_MAX_DOMAIN_ID = 232U;

/// 轉送埠段設定
struct DdsPeerRelayConfig
{
    unsigned int domain_id{0U};       ///< ROS_DOMAIN_ID
    unsigned int instance{1U};        ///< 本機 Agent 的模擬編號（1-based）
    unsigned int instance_count{DDS_RELAY_MAX_INSTANCES};  ///< 模擬 Agent 總數
};

/**
 * @brief 計算指定 Agent 專屬埠段的第一個埠
 *
 * @param domain_id ROS_DOMAIN_ID
 * @param instance 模擬編號（1-based）
 * @return 埠段第一個埠
 */
inline unsigned int dds_relay_first_port(const unsigned int domain_id, const unsigned int instance)
{
    return DDS_RELAY_PORT_BASE + DDS_RELAY_DOMAIN_GAIN * domain_id + DDS_RELAY_UNICAST_OFFSET
        + DDS_RELAY_AGENT_PORT_SPAN * (instance - 1U);
}

/**
 * @brief 檢查轉送設定是否在支援範圍內
 *
 * @param config 轉送設定
 * @return true 表示 domain、編號與總數皆有效
 */
inline bool is_valid_dds_peer_relay_config(const DdsPeerRelayConfig& config)
{
    return config.domain_id <= DDS_RELAY_MAX_DOMAIN_ID && config.instance_count >= 1U
        && config.instance_count <= DDS_RELAY_MAX_INSTANCES && config.instance >= 1U
        && config.instance <= config.instance_count;
}

/**
 * @brief 列出需要轉送的其他 Agent 埠
 *
 * 本機埠段由 CycloneDDS 自己綁定，不得列入；其他 Agent 的埠段在本機容器內
 * 不會被 CycloneDDS 使用，可安全綁定在內部 IP 上。
 *
 * @param config 轉送設定
 * @param[out] out_ports 依遞增排序的埠號；設定無效時清空
 * @return true 表示設定有效
 */
inline bool make_dds_peer_relay_ports(const DdsPeerRelayConfig& config, std::vector<std::uint16_t>& out_ports)
{
    out_ports.clear();
    if (!is_valid_dds_peer_relay_config(config))
    {
        return false;
    }
    out_ports.reserve(static_cast<std::size_t>(DDS_RELAY_AGENT_PORT_SPAN) * (config.instance_count - 1U));
    for (unsigned int peer = 1U; peer <= config.instance_count; ++peer)
    {
        if (peer == config.instance)
        {
            continue;
        }
        const unsigned int first = dds_relay_first_port(config.domain_id, peer);
        for (unsigned int offset = 0U; offset < DDS_RELAY_AGENT_PORT_SPAN; ++offset)
        {
            out_ports.push_back(static_cast<std::uint16_t>(first + offset));
        }
    }
    return true;
}

/**
 * @brief 依埠號找出所屬 Agent 的模擬編號
 *
 * @param config 轉送設定
 * @param port 埠號
 * @return 1-based 模擬編號；不屬於任何 Agent 埠段時為 0
 */
inline unsigned int dds_relay_instance_for_port(const DdsPeerRelayConfig& config, const unsigned int port)
{
    if (!is_valid_dds_peer_relay_config(config))
    {
        return 0U;
    }
    const unsigned int first = dds_relay_first_port(config.domain_id, 1U);
    if (port < first)
    {
        return 0U;
    }
    const unsigned int instance = (port - first) / DDS_RELAY_AGENT_PORT_SPAN + 1U;
    return instance <= config.instance_count ? instance : 0U;
}

/**
 * @brief 以模板產生同儕 Agent 在 Docker 網路上的主機名稱
 *
 * 模板中的每個 `{N}` 換成模擬編號，例如 `uav_{N}_agent` → `uav_2_agent`、
 * `agent_{N}` → `agent_2`；模板為空時回傳空字串，呼叫端改用主機區網 IP。
 *
 * @param host_format 主機名稱模板
 * @param instance 模擬編號（1-based）
 * @return 主機名稱
 */
inline std::string make_dds_peer_host(const std::string& host_format, const unsigned int instance)
{
    static const std::string PLACEHOLDER = "{N}";
    std::string host = host_format;
    const std::string number = std::to_string(instance);
    std::size_t position = host.find(PLACEHOLDER);
    while (position != std::string::npos)
    {
        host.replace(position, PLACEHOLDER.size(), number);
        position = host.find(PLACEHOLDER, position + number.size());
    }
    return host;
}

}  // namespace utils
}  // namespace taiphoon
