/**
 * @file PeerMatcher.hpp
 * @brief 同儕發現輔助類別 — 純字串比對邏輯，無 ROS2 依賴
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 提供靜態方法：
 * 1. 判斷 node 名稱是否為 agent 節點（/uav_<N>_agent/... 主模式，
 *    /uav_<N>/ego_swarm_planner 與 /uav_<N>/basic_controller_node 相容模式）
 *    — agent 節點存在性即為離線判定依據
 * 2. 從 node/topic 名稱擷取 agent 索引數字
 * 3. 過濾 node 列表（含自身），供同儕發現使用
 * 4. 判斷 topic 名稱是否為 /uav_<N>_agent/state（向後相容）
 * 5. 比對已知與目前探索結果，找出已消失的 Agent
 *
 * 由 MissionPlanner 節點在 scan_peers_cb 中呼叫。
 */

#pragma once

#include <string>
#include <vector>
#include <utility>
#include <cstdlib>
#include <algorithm>
#include <set>

#include "utils/AgentNaming.hpp"

namespace taiphoon
{
namespace formation
{

/// 同儕發現結果：pair<agent_index, topic_name>
using PeerEntry = std::pair<int, std::string>;

/**
 * @brief 同儕發現輔助類別
 *
 * 所有方法皆為靜態，便於單元測試。
 */
class PeerMatcher
{
public:
    // =====================================================================
    // Node-based 同儕發現（取代 topic-based，解決 discovery 殘留問題）
    // =====================================================================

    /**
     * @brief 檢查 node 名稱是否為 agent 節點
     *
     * 主模式：/uav_<數字>_agent/<節點名>（一機一 agent 架構，
     *         如 "/uav_2_agent/mission_planner"、"trajectory_controller"）
     * 相容模式：/uav_<數字>/ego_swarm_planner 或
     *         /uav_<數字>/basic_controller_node（單容器 launch 部署）
     *
     * 機間離線判定以 agent 節點存在性為依據，因此只辨識 agent 命名空間
     * 節點與 basic_controller_node，不將其他 /uav_<數字>/ 節點視為 agent。
     *
     * @param node_name 完整 node 名稱 (例如 "/uav_2_agent/mission_planner")
     * @return true 符合 agent 節點格式
     */
    static bool is_agent_node(const std::string& node_name)
    {
        if (node_name.size() < 2 || node_name.front() != '/')
        {
            return false;
        }

        const std::size_t namespace_end = node_name.find('/', 1);
        if (namespace_end == std::string::npos ||
            namespace_end + 1 >= node_name.size())
        {
            return false;
        }

        const std::string namespace_name = node_name.substr(1, namespace_end - 1);
        const std::string node_suffix = node_name.substr(namespace_end);
        const int agent_index = utils::agent_naming::parse_agent_namespace_index(
            namespace_name);
        if (agent_index >= 0)
        {
            return true;
        }

        // 相容模式只辨識 PX4 namespace 下由本專案啟動的兩個 Agent 節點。
        if (utils::agent_naming::parse_uav_namespace_index(namespace_name) < 0)
        {
            return false;
        }

        const std::string legacy_nodes[] = {
            "/ego_swarm_planner",
            "/basic_controller_node",
        };
        for (const auto& legacy_node : legacy_nodes)
        {
            if (node_suffix == legacy_node)
            {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief 從 node 名稱擷取 agent 索引數字
     *
     * 例如 "/uav_3/basic_controller_node" → 3
     *
     * @param node_name 完整 node 名稱
     * @return int agent 索引 (-1 表示無法解析)
     */
    static int extract_agent_index_from_node(const std::string& node_name)
    {
        if (!is_agent_node(node_name)) return -1;

        const std::size_t namespace_end = node_name.find('/', 1);
        if (namespace_end == std::string::npos)
        {
            return -1;
        }
        const std::string namespace_name = node_name.substr(1, namespace_end - 1);
        const int agent_index = utils::agent_naming::parse_agent_namespace_index(
            namespace_name);
        if (agent_index >= 0)
        {
            return agent_index;
        }
        return utils::agent_naming::parse_uav_namespace_index(namespace_name);
    }

    /**
     * @brief 從 node name 列表過濾 agent 節點，回傳不重複索引
     *
     * 排除符合自身 namespace 的節點。
     *
     * @param node_names    get_node_names() 回傳的 node 名稱列表
     * @param own_namespace 自身 agent namespace (例如 "uav_1_agent")
     * @return std::vector<int> 不重複的 agent 索引
     */
    static std::vector<int> filter_agent_nodes(
        const std::vector<std::string>& node_names,
        const std::string& own_namespace)
    {
        const int own_num = utils::agent_naming::parse_agent_namespace_index(
            own_namespace);

        std::set<int> seen;
        for (const auto& node : node_names)
        {
            if (!is_agent_node(node)) continue;

            int idx = extract_agent_index_from_node(node);
            if (idx < 0) continue;
            if (idx == own_num) continue;  // 排除自身

            seen.insert(idx);
        }

        return std::vector<int>(seen.begin(), seen.end());
    }

    // =====================================================================
    // Topic-based 方法（向後相容，供 TrajectoryController 使用）
    // =====================================================================

    /**
     * @brief 檢查 topic 名稱是否為 AgentState 主題
     *
     * 符合格式：/uav_<數字>_agent/state
     *
     * @param topic 完整 topic 路徑
     * @return true 符合格式
     */
    static bool is_agent_state_topic(const std::string& topic)
    {
        if (topic.empty()) return false;

        const std::string suffix = "/state";
        if (topic.size() <= suffix.size()) return false;
        if (topic.compare(
                topic.size() - suffix.size(), suffix.size(), suffix) != 0)
            return false;

        const std::size_t namespace_end = topic.find('/', 1);
        if (namespace_end == std::string::npos)
        {
            return false;
        }
        const std::string namespace_name = topic.substr(1, namespace_end - 1);
        return utils::agent_naming::parse_agent_namespace_index(namespace_name) >= 0 &&
            topic.substr(namespace_end) == "/state";
    }

    /**
     * @brief 從 topic 名稱擷取 agent 索引數字
     *
     * 例如 "/uav_1_agent/state" → 1
     *
     * @param topic 完整 topic 路徑
     * @return int agent 索引 (-1 表示無法解析)
     */
    static int extract_agent_index(const std::string& topic)
    {
        if (!is_agent_state_topic(topic)) return -1;

        const std::size_t namespace_end = topic.find('/', 1);
        if (namespace_end == std::string::npos)
        {
            return -1;
        }
        return utils::agent_naming::parse_agent_namespace_index(
            topic.substr(1, namespace_end - 1));
    }

    /**
     * @brief 過濾 topic 列表，提取所有同儕的 AgentState topic
     *
     * 包含自身 namespace 的 topic（自身位置亦參與編隊計算）。
     *
     * @param topic_names ROS2 get_topic_names_and_types() 回傳的 topic 列表
     * @param own_namespace 自身 agent namespace（保留參數以相容既有呼叫者）
     * @return std::vector<PeerEntry> 同儕列表 (index, topic_name)
     */
    static std::vector<PeerEntry> filter_peers(
        const std::vector<std::string>& topic_names,
        const std::string& own_namespace)
    {
        (void)own_namespace;

        std::vector<PeerEntry> result;

        for (const auto& topic : topic_names)
        {
            if (!is_agent_state_topic(topic)) continue;

            int index = extract_agent_index(topic);
            if (index >= 0)
            {
                result.emplace_back(index, topic);
            }
        }

        return result;
    }

    /**
     * @brief 找出已知但不再出現在探索結果中的 Agent 索引
     *
     * Agent 消失後必須退出編隊與防撞集合，後續飛行行為交由其 PX4
     * failsafe 處理。回傳順序依索引遞增，讓呼叫端可穩定記錄與移除。
     *
     * @param known_indices 已建立狀態或訂閱的 Agent 索引
     * @param discovered_indices 目前 ROS2 graph 中探索到的 Agent 索引
     * @return 已消失的 Agent 索引
     */
    static std::vector<int> find_removed_indices(
        const std::set<int>& known_indices,
        const std::set<int>& discovered_indices)
    {
        std::vector<int> removed_indices;
        for (const int agent_index : known_indices)
        {
            if (discovered_indices.find(agent_index) ==
                discovered_indices.end())
            {
                removed_indices.push_back(agent_index);
            }
        }
        return removed_indices;
    }
};

}  // namespace formation
}  // namespace taiphoon
