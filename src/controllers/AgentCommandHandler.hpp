/**
 * @file AgentCommandHandler.hpp
 * @brief 高階命令處理器，負責接收來自地面站或其他 Agent 的 ROS2 命令
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 *
 * 訂閱主題：
 *   - /uav_1_agent/command  (特定 Agent 命令)
 *   - /uav_all_agent/command (全體 Agent 命令)
 *
 * 支援命令請參閱 CommandTypes.hpp
 */

#pragma once

#include <string>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "utils/CommandTypes.hpp"
#include "core/IBasicFlightController.hpp"

namespace taiphoon
{
namespace controllers
{

/**
 * @brief 高階命令處理器
 *
 * 負責監聽 ROS2 命令主題，解析高階命令字串後，
 * 透過 IBasicFlightController 介面執行對應的飛行控制動作。
 */
class AgentCommandHandler
{
public:
    /**
     * @brief 命令執行回呼類型
     *
     * 當命令解析完成並需要執行時，呼叫此回呼。
     * 參數為解析後的命令結構。
     */
    using CommandCallback = std::function<void(const utils::ParsedCommand&)>;

    /**
     * @brief 含命令來源的執行回呼類型
     *
     * 部分命令（MISSION_*、REBOOT_FC）只接受單機 topic，節點需依來源判斷。
     */
    using SourcedCommandCallback = std::function<void(
        const utils::ParsedCommand&, utils::CommandSource)>;

    /**
     * @brief 無法解析命令的回呼類型
     *
     * 參數為原始命令字串與來源，供節點回報 COMMAND_UNKNOWN 事件。
     */
    using UnknownCommandCallback = std::function<void(
        const std::string&, utils::CommandSource)>;

    /**
     * @brief 建構子
     *
     * @param node ROS2 節點指標 (不得為 nullptr)
     * @param agent_namespace Agent 的命名空間 (例如 "uav_1_agent")
     */
    explicit AgentCommandHandler(
        rclcpp::Node* node,
        const std::string& agent_namespace);

    ~AgentCommandHandler() = default;

    /**
     * @brief 初始化命令處理器 (建立 Publisher 與 Subscriber)
     *
     * @return true 初始化成功
     */
    bool initialize();

    /**
     * @brief 設定命令執行回呼
     *
     * @param callback 命令執行回呼函式
     */
    void set_command_callback(CommandCallback callback);

    /**
     * @brief 設定含命令來源的執行回呼（取代 set_command_callback 的設定）
     *
     * @param callback 命令執行回呼函式
     */
    void set_sourced_command_callback(SourcedCommandCallback callback);

    /**
     * @brief 設定無法解析命令的回呼
     *
     * @param callback 回呼函式；空值表示只記錄日誌
     */
    void set_unknown_command_callback(UnknownCommandCallback callback);

    /**
     * @brief 取得此 Agent 的命名空間
     *
     * @return const std::string& 命名空間字串
     */
    const std::string& get_agent_namespace() const;

private:
    /**
     * @brief 訂閱特定 Agent 命令主題的回呼
     *
     * 監聽 /<agent_namespace>/command 主題
     *
     * @param msg 接收到的字串訊息
     */
    void agent_command_cb(const std_msgs::msg::String::SharedPtr msg);

    /**
     * @brief 訂閱全體 Agent 命令主題的回呼
     *
     * 監聽 /uav_all_agent/command 主題
     *
     * @param msg 接收到的字串訊息
     */
    void all_agent_command_cb(const std_msgs::msg::String::SharedPtr msg);

    /**
     * @brief 解析命令字串並交給對應回呼
     *
     * @param cmd_str 原始命令字串
     * @param source 命令來源 topic
     */
    void dispatch_command(const std::string& cmd_str,
                          utils::CommandSource source);

    /**
     * @brief 建立特定 Agent 命令的訂閱者
     *
     * @return true 建立成功
     */
    bool create_agent_command_subscriber();

    /**
     * @brief 建立全體 Agent 命令的訂閱者
     *
     * @return true 建立成功
     */
    bool create_all_agent_command_subscriber();

    /// ROS2 節點指標
    rclcpp::Node* node_;

    /// 此 Agent 的命名空間 (例如 "uav_1_agent")
    std::string agent_namespace_;

    /// 訂閱特定 Agent 命令的主題 (/<agent_namespace>/command)
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr agent_command_sub_;

    /// 訂閱全體 Agent 命令的主題 (/uav_all_agent/command)
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr all_agent_command_sub_;

    /// 命令執行回呼（set_command_callback 也轉為此型別保存）
    SourcedCommandCallback command_callback_;

    /// 無法解析命令的回呼
    UnknownCommandCallback unknown_command_callback_;

    /// 初始化完成旗標
    bool initialized_{false};
};

}  // namespace controllers
}  // namespace taiphoon
