/**
 * @file basic_controller_setpoint_test.cpp
 * @brief BasicController Offboard 位置設定點鎖定行為測試
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * 透過公開控制 API 下達命令，並從 ROS2 TrajectorySetpoint topic
 * 驗證命令當下的位置目標不會跟隨後續 odometry 漂移。
 */

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include "controllers/BasicController.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr float POSITION_EPSILON_M = 1.0e-5F;
constexpr int MAX_PUBLISH_ATTEMPTS = 50;
constexpr auto DISCOVERY_WAIT = std::chrono::milliseconds(20);

class BasicControllerSetpointTest : public ::testing::Test
{
protected:
    /** @brief 初始化單元測試共用的 ROS2 context */
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok())
        {
            int argc = 0;
            rclcpp::init(argc, nullptr);
        }
    }

    /** @brief 關閉單元測試共用的 ROS2 context */
    static void TearDownTestSuite()
    {
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
    }

    /** @brief 建立控制器與可觀測 TrajectorySetpoint 的測試節點 */
    void SetUp() override
    {
        static int node_index = 0;
        node_ = std::make_shared<rclcpp::Node>(
            "basic_controller_setpoint_test_" + std::to_string(node_index++));
        controller_ = std::make_unique<taiphoon::controllers::BasicController>(node_.get());
        ASSERT_TRUE(controller_->initialize());

        trajectory_setpoint_sub_ =
            node_->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
                "fmu/in/trajectory_setpoint",
                rclcpp::QoS(10).best_effort(),
                [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
                {
                    if (msg == nullptr)
                    {
                        RCLCPP_WARN(node_->get_logger(),
                                    "Received nullptr TrajectorySetpoint");
                        return;
                    }

                    latest_setpoint_ = *msg;
                    has_setpoint_ = true;
                });
        executor_.add_node(node_);
    }

    /** @brief 釋放每個測試建立的 ROS2 實體 */
    void TearDown() override
    {
        controller_->shutdown();
        executor_.remove_node(node_);
        trajectory_setpoint_sub_.reset();
        controller_.reset();
        node_.reset();
    }

    /**
     * @brief 建立指定 NED 位置的測試里程計
     * @param x NED X（公尺）
     * @param y NED Y（公尺）
     * @param z NED Z（公尺）
     * @return 含單位四元數的 VehicleOdometry
     */
    px4_msgs::msg::VehicleOdometry make_odometry(
        float x, float y, float z) const
    {
        px4_msgs::msg::VehicleOdometry odometry;
        odometry.position[0] = x;
        odometry.position[1] = y;
        odometry.position[2] = z;
        odometry.q[0] = 1.0F;
        odometry.q[1] = 0.0F;
        odometry.q[2] = 0.0F;
        odometry.q[3] = 0.0F;
        return odometry;
    }

    /** @brief 發布控制命令直到測試訂閱者收到第一筆設定點 */
    void publish_until_setpoint_received()
    {
        for (int attempt = 0; attempt < MAX_PUBLISH_ATTEMPTS && !has_setpoint_; ++attempt)
        {
            controller_->publish_control_commands();
            executor_.spin_some(DISCOVERY_WAIT);
        }
        ASSERT_TRUE(has_setpoint_);
    }

    /** @brief 驗證位置與偏航欄位均為有限值。 */
    void expect_finite_position_and_yaw() const
    {
        EXPECT_TRUE(std::isfinite(latest_setpoint_.position[0]));
        EXPECT_TRUE(std::isfinite(latest_setpoint_.position[1]));
        EXPECT_TRUE(std::isfinite(latest_setpoint_.position[2]));
        EXPECT_TRUE(std::isfinite(latest_setpoint_.yaw));
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::unique_ptr<taiphoon::controllers::BasicController> controller_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
        trajectory_setpoint_sub_;
    px4_msgs::msg::TrajectorySetpoint latest_setpoint_;
    bool has_setpoint_{false};
};

TEST_F(BasicControllerSetpointTest, TakeoffLocksCommandPositionAgainstOdometryDrift)
{
    controller_->update_odometry(make_odometry(1.25F, -2.5F, -0.2F));
    ASSERT_TRUE(controller_->takeoff(6.0F));

    controller_->update_odometry(make_odometry(8.0F, 9.0F, -1.0F));
    publish_until_setpoint_received();

    EXPECT_NEAR(latest_setpoint_.position[0], 1.25F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[1], -2.5F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[2], -6.0F, POSITION_EPSILON_M);
}

TEST_F(BasicControllerSetpointTest, HoldPositionLocksAllAxesAgainstOdometryDrift)
{
    controller_->update_odometry(make_odometry(3.0F, 4.0F, -5.0F));
    ASSERT_TRUE(controller_->hold_position(2.0F));

    controller_->update_odometry(make_odometry(-7.0F, 11.0F, -4.0F));
    publish_until_setpoint_received();

    EXPECT_NEAR(latest_setpoint_.position[0], 3.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[1], 4.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[2], -5.0F, POSITION_EPSILON_M);
}

TEST_F(BasicControllerSetpointTest,
       IdleRtlAndLandPublishFinitePositionAndYaw)
{
    controller_->update_odometry(make_odometry(4.0F, -2.0F, -3.0F));

    publish_until_setpoint_received();
    expect_finite_position_and_yaw();

    has_setpoint_ = false;
    ASSERT_TRUE(controller_->switch_to_rtl_mode());
    publish_until_setpoint_received();
    expect_finite_position_and_yaw();

    has_setpoint_ = false;
    ASSERT_TRUE(controller_->switch_to_land_mode());
    publish_until_setpoint_received();
    expect_finite_position_and_yaw();
}

}  // namespace
