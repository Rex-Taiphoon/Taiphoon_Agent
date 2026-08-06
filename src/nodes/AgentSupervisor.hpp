/**
 * @file AgentSupervisor.hpp
 * @brief Taiphoon_Agent 容器生命週期管理節點
 * @author Taiphoon Agent Team
 * @date 2026-08-06
 */

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace taiphoon
{
namespace nodes
{

/**
 * @brief 提供單一 Agent 的遠端容器關閉服務
 *
 * 此節點刻意不檢查飛行狀態，也不執行 RTL、LAND 或 HOLD。
 * 服務接受後會在回覆送出後對容器 PID 1 發送 SIGTERM；entrypoint 會立即退出，
 * 以模擬飛行電腦完全失效；
 * 後續行為由 PX4 的 failsafe 設定決定。
 */
class AgentSupervisor : public rclcpp::Node
{
public:
    using TerminateCallback = std::function<bool()>;

    /**
     * @brief 建立 Agent 生命週期節點
     *
     * @param node_options ROS2 節點選項，測試可用來覆寫 agent_namespace
     * @param terminate_callback 容器終止回呼，未提供時會終止 PID 1
     */
    explicit AgentSupervisor(
        const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions(),
        TerminateCallback terminate_callback = {});

    ~AgentSupervisor() override = default;

    /**
     * @brief 建立遠端關閉 service
     *
     * @return true service 建立成功
     */
    bool initialize();

    /**
     * @brief 查詢是否已接受關閉請求
     *
     * @return true 已接受至少一個有效關閉請求
     */
    bool is_shutdown_requested() const;

private:
    using ShutdownService = std_srvs::srv::SetBool;

    /**
     * @brief 處理地面站的關閉請求
     *
     * @param request service 請求
     * @param response service 回應
     */
    void shutdown_service_cb(
        const std::shared_ptr<ShutdownService::Request> request,
        std::shared_ptr<ShutdownService::Response> response);

    /**
     * @brief 延遲執行容器終止回呼
     */
    void terminate_timer_cb();

    /**
     * @brief 建立 shutdown service
     *
     * @return true service 建立成功
     */
    bool create_shutdown_service();

    /**
     * @brief 發送 SIGTERM 讓容器的 PID 1 立即退出
     *
     * @return true SIGTERM 成功送達
     */
    static bool terminate_container();

    std::string agent_namespace_;
    TerminateCallback terminate_callback_;

    rclcpp::Service<ShutdownService>::SharedPtr shutdown_service_;
    rclcpp::TimerBase::SharedPtr termination_timer_;
    bool is_shutdown_requested_{false};
};

}  // namespace nodes
}  // namespace taiphoon
