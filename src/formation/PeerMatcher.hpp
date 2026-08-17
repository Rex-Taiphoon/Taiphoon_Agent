/**
 * @file PeerMatcher.hpp
 * @brief 同儕發現輔助類別 — 純字串比對邏輯，無 ROS2 依賴
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 提供靜態方法：
 * 1. 判斷 node 名稱是否為 agent 節點（/fmu<N>_agent/... 主模式，
 *    /fmu<N>/ego_swarm_planner 與 /fmu<N>/basic_controller_node 相容模式）
 *    — agent 節點存在性即為離線判定依據
 * 2. 從 node/topic 名稱擷取 agent 索引數字
 * 3. 過濾 node 列表（含自身），供同儕發現使用
 * 4. 判斷 topic 名稱是否為 /fmu*_agent/state（向後相容）
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
     * 主模式：/fmu<數字>_agent/<節點名>（一機一 agent 架構，
     *         如 "/fmu2_agent/mission_planner"、"trajectory_controller"）
     * 相容模式：/fmu<數字>/ego_swarm_planner 或
     *         /fmu<數字>/basic_controller_node（單容器 launch 部署）
     *
     * 機間離線判定以 agent 節點存在性為依據，因此只辨識 agent 命名空間
     * 節點與 basic_controller_node，不將其他 /fmu<數字>/ 節點視為 agent。
     *
     * @param node_name 完整 node 名稱 (例如 "/fmu2_agent/mission_planner")
     * @return true 符合 agent 節點格式
     */
    static bool is_agent_node(const std::string& node_name)
    {
        // 最短 "/fmu1/" 為 7 字元
        if (node_name.size() < 7) return false;
        if (node_name[0] != '/') return false;
        if (node_name.substr(1, 3) != "fmu") return false;

        // fmu 後必須是數字
        size_t digit_start = 4;
        if (!std::isdigit(static_cast<unsigned char>(node_name[digit_start])))
            return false;

        // 掃描連續數字
        size_t pos = digit_start;
        while (pos < node_name.size() &&
               std::isdigit(static_cast<unsigned char>(node_name[pos])))
        {
            ++pos;
        }
        if (pos == digit_start || pos >= node_name.size()) return false;

        // 主模式：/fmu<數字>_agent/<節點名>
        if (node_name[pos] == '_')
        {
            const std::string agent_suffix = "_agent/";
            if (node_name.compare(pos, agent_suffix.size(), agent_suffix) != 0)
            {
                return false;
            }
            // "_agent/" 之後必須有節點名
            return pos + agent_suffix.size() < node_name.size();
        }

        // 相容模式：/fmu<數字>/ego_swarm_planner 或
        // /fmu<數字>/basic_controller_node（完整節點名，無後綴）。
        // Docker SITL 目前將 planner 放在 /fmu<N>/ namespace；若只辨識
        // basic_controller_node，planner 自身的 peer discovery 會永遠為空。
        if (node_name[pos] == '/')
        {
            const std::string legacy_nodes[] = {
                "/ego_swarm_planner",
                "/basic_controller_node",
            };
            for (const auto& legacy_node : legacy_nodes)
            {
                if (node_name.compare(pos, legacy_node.size(), legacy_node) == 0 &&
                    node_name.size() == pos + legacy_node.size())
                {
                    return true;
                }
            }
            return false;
        }

        return false;
    }

    /**
     * @brief 從 node 名稱擷取 agent 索引數字
     *
     * 例如 "/fmu3/basic_controller_node" → 3
     *
     * @param node_name 完整 node 名稱
     * @return int agent 索引 (-1 表示無法解析)
     */
    static int extract_agent_index_from_node(const std::string& node_name)
    {
        if (!is_agent_node(node_name)) return -1;

        // "/fmu" 位於 node_name[0..3]，從 [4] 開始擷取數字
        size_t digit_start = 4;
        size_t num_end = digit_start;
        for (; num_end < node_name.size(); ++num_end)
        {
            if (!std::isdigit(static_cast<unsigned char>(node_name[num_end])))
                break;
        }

        if (num_end == digit_start) return -1;

        std::string num_str = node_name.substr(digit_start, num_end - digit_start);
        return std::stoi(num_str);
    }

    /**
     * @brief 從 node name 列表過濾 agent 節點，回傳不重複索引
     *
     * 排除符合自身 namespace 的節點。
     *
     * @param node_names    get_node_names() 回傳的 node 名稱列表
     * @param own_namespace 自身 agent namespace (例如 "fmu1_agent")
     * @return std::vector<int> 不重複的 agent 索引
     */
    static std::vector<int> filter_agent_nodes(
        const std::vector<std::string>& node_names,
        const std::string& own_namespace)
    {
        // 從 own_namespace 擷取自身 fmu 編號以排除自身
        int own_num = -1;
        if (own_namespace.size() > 3 && own_namespace.substr(0, 3) == "fmu")
        {
            size_t us = own_namespace.find('_');
            if (us != std::string::npos)
            {
                try
                {
                    own_num = std::stoi(own_namespace.substr(3, us - 3));
                }
                catch (...) { own_num = -1; }
            }
        }

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
     * 符合格式：/fmu<數字>_agent/state
     *
     * @param topic 完整 topic 路徑
     * @return true 符合格式
     */
    static bool is_agent_state_topic(const std::string& topic)
    {
        if (topic.empty()) return false;

        const std::string suffix = "/state";
        if (topic.size() <= suffix.size()) return false;
        if (topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) != 0)
            return false;

        size_t fmu_pos = topic.rfind("/fmu");
        if (fmu_pos == std::string::npos) return false;

        std::string remaining = topic.substr(fmu_pos + 4);

        if (remaining.substr(0, 3) == "all") return true;

        size_t num_end = 0;
        for (; num_end < remaining.size(); ++num_end)
        {
            if (!std::isdigit(static_cast<unsigned char>(remaining[num_end])))
                break;
        }
        if (num_end == 0) return false;

        std::string expected_mid = "_agent/state";
        if (remaining.compare(num_end, expected_mid.size(), expected_mid) != 0)
            return false;

        return true;
    }

    /**
     * @brief 從 topic 名稱擷取 agent 索引數字
     *
     * 例如 "/fmu1_agent/state" → 1
     *
     * @param topic 完整 topic 路徑
     * @return int agent 索引 (-1 表示無法解析)
     */
    static int extract_agent_index(const std::string& topic)
    {
        if (!is_agent_state_topic(topic)) return -1;

        size_t fmu_pos = topic.rfind("/fmu");
        if (fmu_pos == std::string::npos) return -1;

        std::string remaining = topic.substr(fmu_pos + 4);

        if (remaining.substr(0, 3) == "all") return -1;

        size_t num_end = 0;
        for (; num_end < remaining.size(); ++num_end)
        {
            if (!std::isdigit(static_cast<unsigned char>(remaining[num_end])))
                break;
        }
        if (num_end == 0) return -1;

        std::string num_str = remaining.substr(0, num_end);
        return std::stoi(num_str);
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
