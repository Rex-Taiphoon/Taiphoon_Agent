/**
 * @file PeerMatcher.hpp
 * @brief 同儕發現輔助類別 — 純字串比對邏輯，無 ROS2 依賴
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 提供靜態方法：
 * 1. 判斷 topic 名稱是否為 /fmu*_agent/state
 * 2. 從 topic 名稱擷取 agent 索引數字
 * 3. 過濾 topic 列表，排除自身 namespace
 *
 * 由 MissionPlanner 節點在 scan_peers_cb 中呼叫。
 */

#pragma once

#include <string>
#include <vector>
#include <utility>
#include <cstdlib>
#include <algorithm>

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
        // 格式：/fmu<數字>_agent/state
        if (topic.empty()) return false;

        // 檢查結尾
        const std::string suffix = "/state";
        if (topic.size() <= suffix.size()) return false;
        if (topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) != 0)
            return false;

        // 尋找 "/fmu" 前綴
        size_t fmu_pos = topic.rfind("/fmu");
        if (fmu_pos == std::string::npos) return false;

        // fmu 後面的部分： "<數字>_agent/state" 或 "all_agent/state"
        std::string remaining = topic.substr(fmu_pos + 4);

        // 如果是 "all_agent/state"，也符合（用於未來擴充）
        if (remaining.substr(0, 3) == "all") return true;

        // 找到數字部分：從 fmu 後到第一個非數字字元
        size_t num_end = 0;
        for (; num_end < remaining.size(); ++num_end)
        {
            if (!std::isdigit(static_cast<unsigned char>(remaining[num_end])))
                break;
        }
        if (num_end == 0) return false;  // 沒有數字

        // 數字後必須是 "_agent/state"
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

        // 如果是 "all..."，回傳特殊值 -1
        if (remaining.substr(0, 3) == "all") return -1;

        // 收集數字字元
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
     * 排除符合自身 namespace 的 topic。
     *
     * @param topic_names ROS2 get_topic_names_and_types() 回傳的 topic 列表
     * @param own_namespace 自身 agent namespace (例如 "fmu1_agent")
     * @return std::vector<PeerEntry> 同儕列表 (index, topic_name)
     */
    static std::vector<PeerEntry> filter_peers(
        const std::vector<std::string>& topic_names,
        const std::string& own_namespace)
    {
        std::vector<PeerEntry> result;

        for (const auto& topic : topic_names)
        {
            if (!is_agent_state_topic(topic)) continue;

            // 排除自身
            std::string own_topic = "/" + own_namespace + "/state";
            if (topic == own_topic) continue;

            int index = extract_agent_index(topic);
            if (index >= 0)
            {
                result.emplace_back(index, topic);
            }
        }

        return result;
    }
};

}  // namespace formation
}  // namespace taiphoon
