/**
 * @file AgentNaming.hpp
 * @brief Taiphoon Agent 的 UAV namespace 與 topic 命名工具
 * @author Taiphoon Agent Team
 * @date 2026-08-27
 *
 * 集中維護 PX4 的 /uav_N namespace、Agent 的 /uav_N_agent namespace
 * 與 /uav_all_agent 廣播 namespace，讓節點、同儕發現及測試使用相同規則。
 */

#pragma once

#include <cctype>
#include <climits>
#include <string>

namespace taiphoon
{
namespace utils
{
namespace agent_naming
{

namespace detail
{

/**
 * @brief 解析指定位置後的十進位 UAV 索引。
 * @param value 待解析的完整字串
 * @param digit_start 第一個數字的位置
 * @return 非負索引；格式錯誤或超出 int 範圍時回傳 -1
 */
inline int parse_index(const std::string& value, const std::size_t digit_start)
{
    if (digit_start >= value.size())
    {
        return -1;
    }

    int index = 0;
    for (std::size_t position = digit_start; position < value.size(); ++position)
    {
        const unsigned char character =
            static_cast<unsigned char>(value.at(position));
        if (!std::isdigit(character))
        {
            return -1;
        }

        const int digit = static_cast<int>(
            character - static_cast<unsigned char>('0'));
        if (index > (INT_MAX - digit) / 10)
        {
            return -1;
        }
        index = index * 10 + digit;
    }
    return index;
}

}  // namespace detail

/**
 * @brief 從 PX4 UAV namespace 解析索引。
 * @param namespace_name 例如 "uav_1"
 * @return 非負 UAV 索引；格式不符時回傳 -1
 */
inline int parse_uav_namespace_index(const std::string& namespace_name)
{
    static const std::string prefix = "uav_";
    if (namespace_name.size() <= prefix.size() ||
        namespace_name.compare(0, prefix.size(), prefix) != 0)
    {
        return -1;
    }
    return detail::parse_index(namespace_name, prefix.size());
}

/**
 * @brief 從 Agent namespace 解析索引。
 * @param namespace_name 例如 "uav_1_agent"
 * @return 非負 UAV 索引；格式不符時回傳 -1
 */
inline int parse_agent_namespace_index(const std::string& namespace_name)
{
    static const std::string suffix = "_agent";
    if (namespace_name.size() <= suffix.size() ||
        namespace_name.compare(
            namespace_name.size() - suffix.size(), suffix.size(), suffix) != 0)
    {
        return -1;
    }

    return parse_uav_namespace_index(
        namespace_name.substr(0, namespace_name.size() - suffix.size()));
}

/**
 * @brief 建立 PX4 UAV namespace。
 * @param index UAV 索引
 * @return 例如 index=1 時回傳 "uav_1"；索引為負值時回傳空字串
 */
inline std::string make_uav_namespace(const int index)
{
    if (index < 0)
    {
        return {};
    }
    return "uav_" + std::to_string(index);
}

/**
 * @brief 建立 Agent 對外 namespace。
 * @param index UAV 索引
 * @return 例如 index=1 時回傳 "uav_1_agent"；索引為負值時回傳空字串
 */
inline std::string make_agent_namespace(const int index)
{
    const std::string uav_namespace = make_uav_namespace(index);
    if (uav_namespace.empty())
    {
        return {};
    }
    return uav_namespace + "_agent";
}

/**
 * @brief 建立單一 Agent 的 AgentState topic。
 * @param index UAV 索引
 * @return 例如 index=1 時回傳 "/uav_1_agent/state"；索引為負值時回傳空字串
 */
inline std::string make_agent_state_topic(const int index)
{
    const std::string agent_namespace = make_agent_namespace(index);
    if (agent_namespace.empty())
    {
        return {};
    }
    return "/" + agent_namespace + "/state";
}

/**
 * @brief 建立全體 Agent 廣播 topic。
 * @param suffix topic 後綴，可帶或不帶開頭斜線
 * @return 例如 suffix="command" 時回傳 "/uav_all_agent/command"
 */
inline std::string make_all_agent_topic(const std::string& suffix)
{
    if (suffix.empty())
    {
        return "/uav_all_agent";
    }
    if (suffix.front() == '/')
    {
        return "/uav_all_agent" + suffix;
    }
    return "/uav_all_agent/" + suffix;
}

}  // namespace agent_naming
}  // namespace utils
}  // namespace taiphoon
