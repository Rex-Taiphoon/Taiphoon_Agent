/**
 * @file agent_supervisor_test.cpp
 * @brief AgentSupervisor shutdown service 的 ROS2 整合測試
 * @author Taiphoon Agent Team
 * @date 2026-08-06
 *
 * 測試透過注入終止回呼驗證 service 行為，避免測試程序真的終止 PID 1。
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nodes/AgentSupervisor.hpp"

namespace
{

constexpr std::chrono::seconds SERVICE_TIMEOUT{2};
constexpr std::chrono::milliseconds SPIN_INTERVAL{1};

class AgentSupervisorTest : public ::testing::Test
{
protected:
    /**
     * @brief 建立獨立 ROS2 context、supervisor 與 service client
     */
    void SetUp() override
    {
        context_ = std::make_shared<rclcpp::Context>();
        context_->init(0, nullptr);

        rclcpp::NodeOptions supervisor_options;
        supervisor_options.context(context_);
        supervisor_options.append_parameter_override(
            "agent_namespace", "test_agent");

        supervisor_ = std::make_shared<taiphoon::nodes::AgentSupervisor>(
            supervisor_options,
            [this]()
            {
                terminate_callback_count_.fetch_add(
                    1, std::memory_order_relaxed);
                return true;
            });

        ASSERT_TRUE(supervisor_->initialize());

        rclcpp::NodeOptions client_options;
        client_options.context(context_);
        client_node_ = std::make_shared<rclcpp::Node>(
            "agent_supervisor_test_client", client_options);
        client_ = client_node_->create_client<std_srvs::srv::SetBool>(
            "/test_agent/shutdown");

        rclcpp::ExecutorOptions executor_options;
        executor_options.context = context_;
        executor_ = std::make_unique<
            rclcpp::executors::SingleThreadedExecutor>(executor_options);
        executor_->add_node(supervisor_);
        executor_->add_node(client_node_);
    }

    /**
     * @brief 釋放 executor、節點與 ROS2 context
     */
    void TearDown() override
    {
        if (executor_ != nullptr)
        {
            executor_->remove_node(supervisor_);
            executor_->remove_node(client_node_);
            executor_.reset();
        }

        client_.reset();
        client_node_.reset();
        supervisor_.reset();

        if (context_ != nullptr)
        {
            context_->shutdown("AgentSupervisor test teardown");
            context_.reset();
        }
    }

    /**
     * @brief 發送 shutdown service 請求並等待回應
     *
     * @param should_terminate SetBool request 的 data 欄位
     * @return service 回應；逾時時回傳 nullptr
     */
    std::shared_ptr<std_srvs::srv::SetBool::Response>
    send_shutdown_request(bool should_terminate)
    {
        if (!client_->wait_for_service(SERVICE_TIMEOUT))
        {
            ADD_FAILURE() << "等待 /test_agent/shutdown service 逾時";
            return nullptr;
        }

        auto request = std::make_shared<
            std_srvs::srv::SetBool::Request>();
        request->data = should_terminate;
        auto response_future = client_->async_send_request(request);

        const auto deadline =
            std::chrono::steady_clock::now() + SERVICE_TIMEOUT;
        while (response_future.wait_for(std::chrono::milliseconds(0)) !=
               std::future_status::ready)
        {
            executor_->spin_some(SPIN_INTERVAL);
            if (std::chrono::steady_clock::now() >= deadline)
            {
                ADD_FAILURE() << "等待 /test_agent/shutdown response 逾時";
                return nullptr;
            }
        }

        auto response = response_future.get();

        if (should_terminate)
        {
            const auto callback_deadline =
                std::chrono::steady_clock::now() +
                std::chrono::milliseconds(500);
            while (terminate_callback_count_.load(
                       std::memory_order_relaxed) == 0 &&
                   std::chrono::steady_clock::now() < callback_deadline)
            {
                executor_->spin_some(SPIN_INTERVAL);
            }
        }

        return response;
    }

    std::shared_ptr<rclcpp::Context> context_;
    std::shared_ptr<taiphoon::nodes::AgentSupervisor> supervisor_;
    std::shared_ptr<rclcpp::Node> client_node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::atomic<int> terminate_callback_count_{0};
};

TEST_F(AgentSupervisorTest, RejectsFalseShutdownRequest)
{
    const auto response = send_shutdown_request(false);

    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(terminate_callback_count_.load(std::memory_order_relaxed), 0);
}

TEST_F(AgentSupervisorTest, AcceptsTrueShutdownRequest)
{
    const auto response = send_shutdown_request(true);

    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);
    EXPECT_EQ(terminate_callback_count_.load(std::memory_order_relaxed), 1);
    EXPECT_TRUE(supervisor_->is_shutdown_requested());
}

TEST_F(AgentSupervisorTest, RepeatedTrueRequestsAreIdempotent)
{
    const auto first_response = send_shutdown_request(true);
    ASSERT_NE(first_response, nullptr);
    ASSERT_TRUE(first_response->success);

    const auto second_response = send_shutdown_request(true);

    ASSERT_NE(second_response, nullptr);
    EXPECT_TRUE(second_response->success);
    EXPECT_EQ(terminate_callback_count_.load(std::memory_order_relaxed), 1);
}

}  // namespace
