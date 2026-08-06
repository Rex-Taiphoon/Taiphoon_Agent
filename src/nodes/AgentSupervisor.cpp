/**
 * @file AgentSupervisor.cpp
 * @brief Taiphoon_Agent 容器生命週期管理節點實作
 * @author Taiphoon Agent Team
 * @date 2026-08-06
 */

#include "nodes/AgentSupervisor.hpp"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <utility>
#include <unistd.h>

namespace taiphoon
{
namespace nodes
{

namespace
{

constexpr int SHUTDOWN_DELAY_MS = 100;

}  // namespace

AgentSupervisor::AgentSupervisor(
    const rclcpp::NodeOptions& node_options,
    TerminateCallback terminate_callback)
    : rclcpp::Node("agent_supervisor", node_options),
      terminate_callback_(std::move(terminate_callback))
{
    this->declare_parameter<std::string>("agent_namespace", "fmu1_agent");
    agent_namespace_ = this->get_parameter("agent_namespace").as_string();

    if (!terminate_callback_)
    {
        terminate_callback_ = &AgentSupervisor::terminate_container;
    }

    RCLCPP_INFO(this->get_logger(),
                "AgentSupervisor: agent_namespace=%s",
                agent_namespace_.c_str());
}

bool AgentSupervisor::initialize()
{
    if (agent_namespace_.empty())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Cannot initialize: agent_namespace is empty");
        return false;
    }

    if (!create_shutdown_service())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to initialize shutdown service");
        return false;
    }

    return true;
}

bool AgentSupervisor::is_shutdown_requested() const
{
    return is_shutdown_requested_;
}

bool AgentSupervisor::create_shutdown_service()
{
    const std::string service_name = "/" + agent_namespace_ + "/shutdown";
    shutdown_service_ = this->create_service<ShutdownService>(
        service_name,
        std::bind(&AgentSupervisor::shutdown_service_cb,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    if (shutdown_service_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create service '%s'",
                     service_name.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Remote shutdown service ready: %s",
                service_name.c_str());
    return true;
}

void AgentSupervisor::shutdown_service_cb(
    const std::shared_ptr<ShutdownService::Request> request,
    std::shared_ptr<ShutdownService::Response> response)
{
    if (request == nullptr || response == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Shutdown service received a null request or response");
        return;
    }

    if (!request->data)
    {
        response->success = false;
        response->message =
            "拒絕關閉：request.data 必須為 true";
        return;
    }

    response->success = true;

    if (is_shutdown_requested_)
    {
        response->message =
            "關閉請求已接受；容器終止等待中";
        return;
    }

    is_shutdown_requested_ = true;
    response->message =
        "關閉請求已接受；已排程終止容器";

    termination_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(SHUTDOWN_DELAY_MS),
        std::bind(&AgentSupervisor::terminate_timer_cb, this));
}

void AgentSupervisor::terminate_timer_cb()
{
    if (termination_timer_ != nullptr)
    {
        termination_timer_->cancel();
    }

    if (terminate_callback_ && terminate_callback_())
    {
        return;
    }

    RCLCPP_ERROR(this->get_logger(),
                 "Failed to terminate Agent container PID 1: %s",
                 std::strerror(errno));
    rclcpp::shutdown();
}

bool AgentSupervisor::terminate_container()
{
    if (::getpid() == 1)
    {
        return ::raise(SIGTERM) == 0;
    }

    return ::kill(1, SIGTERM) == 0;
}

}  // namespace nodes
}  // namespace taiphoon
