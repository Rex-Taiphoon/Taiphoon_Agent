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
#include <cmath>
#include <memory>
#include <string>

#include "controllers/BasicController.hpp"
#include "controllers/FlightStateResolver.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr float POSITION_EPSILON_M = 1.0e-5F;
constexpr int MAX_PUBLISH_ATTEMPTS = 50;
constexpr auto DISCOVERY_WAIT = std::chrono::milliseconds(20);
constexpr uint8_t PX4_NAV_STATE_POSCTL = 4U;

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
        offboard_control_mode_sub_ =
            node_->create_subscription<px4_msgs::msg::OffboardControlMode>(
                "fmu/in/offboard_control_mode",
                rclcpp::QoS(10).best_effort(),
                [this](const px4_msgs::msg::OffboardControlMode::SharedPtr msg)
                {
                    if (msg != nullptr) ++offboard_control_mode_count_;
                });
        vehicle_command_sub_ =
            node_->create_subscription<px4_msgs::msg::VehicleCommand>(
                "fmu/in/vehicle_command",
                rclcpp::QoS(10).best_effort(),
                [this](const px4_msgs::msg::VehicleCommand::SharedPtr msg)
                {
                    if (msg != nullptr) ++vehicle_command_count_;
                });
        vehicle_status_pub_ =
            node_->create_publisher<px4_msgs::msg::VehicleStatus>(
                "fmu/out/vehicle_status_v4", rclcpp::QoS(10).best_effort());
        vehicle_command_ack_pub_ =
            node_->create_publisher<px4_msgs::msg::VehicleCommandAck>(
                "fmu/out/vehicle_command_ack_v1", rclcpp::QoS(10).best_effort());
        vehicle_land_detected_pub_ =
            node_->create_publisher<px4_msgs::msg::VehicleLandDetected>(
                "fmu/out/vehicle_land_detected", rclcpp::QoS(10).best_effort());
        executor_.add_node(node_);

        // 既有位置鎖定測試需要在有效 Offboard session 中執行；先完成
        // 真實控制器要求的 10 次 OffboardControlMode 心跳，再由
        // VehicleStatus 確認模式；此流程不應產生 TrajectorySetpoint。
        enter_offboard_session_for_test();
        offboard_control_mode_count_ = 0;
        vehicle_command_count_ = 0;
        has_setpoint_ = false;
    }

    /** @brief 釋放每個測試建立的 ROS2 實體 */
    void TearDown() override
    {
        controller_->shutdown();
        executor_.remove_node(node_);
        trajectory_setpoint_sub_.reset();
        offboard_control_mode_sub_.reset();
        vehicle_command_sub_.reset();
        vehicle_status_pub_.reset();
        vehicle_command_ack_pub_.reset();
        vehicle_land_detected_pub_.reset();
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

    /**
     * @brief 建立含指定偏航角的 NED 里程計
     * @param x NED X（公尺）
     * @param y NED Y（公尺）
     * @param z NED Z（公尺）
     * @param yaw_rad 偏航角（弧度）
     * @return 含指定偏航角的 VehicleOdometry
     */
    px4_msgs::msg::VehicleOdometry make_odometry_with_yaw(
        float x, float y, float z, float yaw_rad) const
    {
        auto odometry = make_odometry(x, y, z);
        odometry.q[0] = std::cos(yaw_rad * 0.5F);
        odometry.q[3] = std::sin(yaw_rad * 0.5F);
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

    /** @brief 發布最新 VehicleStatus，讓控制器能判斷輸出閘門。 */
    void publish_status(uint8_t nav_state)
    {
        px4_msgs::msg::VehicleStatus status;
        status.nav_state = nav_state;
        status.arming_state =
            px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        for (int attempt = 0; attempt < 5; ++attempt)
        {
            vehicle_status_pub_->publish(status);
            executor_.spin_some(DISCOVERY_WAIT);
        }
    }

    /** @brief 發布最新落地偵測狀態。 */
    void publish_land_detected(bool landed)
    {
        px4_msgs::msg::VehicleLandDetected message;
        message.landed = landed;
        for (int attempt = 0; attempt < 5; ++attempt)
        {
            vehicle_land_detected_pub_->publish(message);
            executor_.spin_some(DISCOVERY_WAIT);
        }
    }

    /** @brief 以測試用 VehicleStatus 完成一次 Offboard 進入流程。 */
    void enter_offboard_session_for_test()
    {
        publish_status(PX4_NAV_STATE_POSCTL);
        ASSERT_TRUE(controller_->switch_to_offboard_mode());
        for (uint32_t heartbeat = 0U;
             heartbeat < taiphoon::controllers::OFFBOARD_PRESTREAM_COUNT;
             ++heartbeat)
        {
            controller_->publish_control_commands();
            executor_.spin_some(DISCOVERY_WAIT);
        }
        publish_status(
            taiphoon::controllers::FlightStateResolver::PX4_NAV_STATE_OFFBOARD);
        controller_->publish_control_commands();
        executor_.spin_some(DISCOVERY_WAIT);
        ASSERT_EQ(controller_->get_flight_state(),
                  taiphoon::controllers::FlightState::OFFBOARD_ACTIVE);
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
    rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr
        offboard_control_mode_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr
        vehicle_command_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr
        vehicle_status_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommandAck>::SharedPtr
        vehicle_command_ack_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleLandDetected>::SharedPtr
        vehicle_land_detected_pub_;
    px4_msgs::msg::TrajectorySetpoint latest_setpoint_;
    bool has_setpoint_{false};
    std::size_t offboard_control_mode_count_{0U};
    std::size_t vehicle_command_count_{0U};
};

TEST_F(BasicControllerSetpointTest,
       OffboardAndArmBeforeTakeoffPublishHeartbeatWithoutTrajectory)
{
    publish_status(
        taiphoon::controllers::FlightStateResolver::PX4_NAV_STATE_OFFBOARD);
    offboard_control_mode_count_ = 0U;
    has_setpoint_ = false;

    ASSERT_TRUE(controller_->arm());
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);

    EXPECT_GT(offboard_control_mode_count_, 0U);
    EXPECT_FALSE(has_setpoint_);
    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::ARMED);
}

TEST_F(BasicControllerSetpointTest,
       TakeoffPublishesOneSetpointAndStopsUntilCompletion)
{
    controller_->update_odometry(make_odometry(1.25F, -2.5F, -0.2F));
    ASSERT_TRUE(controller_->takeoff(6.0F));

    publish_until_setpoint_received();
    EXPECT_NEAR(latest_setpoint_.position[2], -6.0F, POSITION_EPSILON_M);

    has_setpoint_ = false;
    const auto heartbeat_count_before_idle = offboard_control_mode_count_;
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_FALSE(has_setpoint_);
    EXPECT_GT(offboard_control_mode_count_, heartbeat_count_before_idle);

    controller_->update_odometry(make_odometry(1.25F, -2.5F, -6.0F));
    publish_status(
        taiphoon::controllers::FlightStateResolver::PX4_NAV_STATE_OFFBOARD);
    publish_land_detected(false);
    has_setpoint_ = false;
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);

    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::HOVER);
    EXPECT_FALSE(has_setpoint_);

    const auto heartbeat_count_after_completion = offboard_control_mode_count_;
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_FALSE(has_setpoint_);
    EXPECT_GT(offboard_control_mode_count_, heartbeat_count_after_completion);
}

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
       HoldPublishesOneSetpointThenMaintainsHeartbeatOnly)
{
    controller_->update_odometry(make_odometry(3.0F, 4.0F, -5.0F));
    ASSERT_TRUE(controller_->hold_position(2.0F));

    publish_until_setpoint_received();
    has_setpoint_ = false;
    const auto heartbeat_count_before_repeat = offboard_control_mode_count_;
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);

    EXPECT_FALSE(has_setpoint_);
    EXPECT_GT(offboard_control_mode_count_, heartbeat_count_before_repeat);
}

TEST_F(BasicControllerSetpointTest,
       RelativeYawStartsFromCurrentHeadingAndUsesNedClockwisePositive)
{
    constexpr float PI = 3.14159265358979323846F;
    constexpr float INITIAL_YAW_RAD = 30.0F * PI / 180.0F;
    constexpr float EXPECTED_YAW_RAD = 120.0F * PI / 180.0F;

    controller_->update_odometry(make_odometry_with_yaw(
        3.0F, 4.0F, -5.0F, INITIAL_YAW_RAD));
    ASSERT_TRUE(controller_->hold_position(2.0F));
    ASSERT_TRUE(controller_->rotate_yaw(90.0F));

    publish_until_setpoint_received();

    EXPECT_NEAR(latest_setpoint_.yaw, EXPECTED_YAW_RAD, 1.0e-5F);
}

TEST_F(BasicControllerSetpointTest, AbsoluteYawSetsMovePointHeading)
{
    constexpr float PI = 3.14159265358979323846F;
    constexpr float EXPECTED_YAW_RAD = -160.0F * PI / 180.0F;

    controller_->update_odometry(make_odometry(3.0F, 4.0F, -5.0F));
    ASSERT_TRUE(controller_->hold_position(2.0F));
    ASSERT_TRUE(controller_->set_absolute_yaw(200.0F));

    publish_until_setpoint_received();

    EXPECT_NEAR(latest_setpoint_.yaw, EXPECTED_YAW_RAD, 1.0e-5F);
}

TEST_F(BasicControllerSetpointTest,
       NonOffboardStopsSetpointAndHeartbeatButStillSendsRtlAndLandCommands)
{
    controller_->update_odometry(make_odometry(4.0F, -2.0F, -3.0F));

    publish_status(PX4_NAV_STATE_POSCTL);
    has_setpoint_ = false;
    offboard_control_mode_count_ = 0U;
    vehicle_command_count_ = 0U;
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_FALSE(has_setpoint_);
    EXPECT_EQ(offboard_control_mode_count_, 0U);

    ASSERT_TRUE(controller_->switch_to_rtl_mode());
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_FALSE(has_setpoint_);
    EXPECT_EQ(offboard_control_mode_count_, 0U);
    EXPECT_GE(vehicle_command_count_, 1U);

    const auto command_count_after_rtl = vehicle_command_count_;
    ASSERT_TRUE(controller_->switch_to_land_mode());
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_FALSE(has_setpoint_);
    EXPECT_EQ(offboard_control_mode_count_, 0U);
    EXPECT_GT(vehicle_command_count_, command_count_after_rtl);
}

TEST_F(BasicControllerSetpointTest,
       OffboardRequiresTenHeartbeatsBeforeModeRequestAndStatusConfirmation)
{
    publish_status(PX4_NAV_STATE_POSCTL);
    offboard_control_mode_count_ = 0U;
    vehicle_command_count_ = 0U;
    ASSERT_TRUE(controller_->switch_to_offboard_mode());

    for (uint32_t heartbeat = 0U;
         heartbeat < taiphoon::controllers::OFFBOARD_PRESTREAM_COUNT - 1U;
         ++heartbeat)
    {
        controller_->publish_control_commands();
        executor_.spin_some(DISCOVERY_WAIT);
    }
    EXPECT_EQ(offboard_control_mode_count_, 9U);
    EXPECT_FALSE(has_setpoint_);
    EXPECT_EQ(vehicle_command_count_, 0U);

    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_EQ(offboard_control_mode_count_, 10U);
    EXPECT_FALSE(has_setpoint_);
    EXPECT_EQ(vehicle_command_count_, 1U);
    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::OFFBOARD_INIT);

    px4_msgs::msg::VehicleCommandAck accepted_ack;
    accepted_ack.command = 176U;
    accepted_ack.result =
        px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
    vehicle_command_ack_pub_->publish(accepted_ack);
    executor_.spin_some(DISCOVERY_WAIT);
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::OFFBOARD_INIT);

    publish_status(
        taiphoon::controllers::FlightStateResolver::PX4_NAV_STATE_OFFBOARD);
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::OFFBOARD_ACTIVE);
}

TEST_F(BasicControllerSetpointTest,
       RejectedOffboardCommandStopsOutputAndRequiresNewRequest)
{
    publish_status(PX4_NAV_STATE_POSCTL);
    offboard_control_mode_count_ = 0U;
    vehicle_command_count_ = 0U;
    ASSERT_TRUE(controller_->switch_to_offboard_mode());
    for (uint32_t heartbeat = 0U;
         heartbeat < taiphoon::controllers::OFFBOARD_PRESTREAM_COUNT;
         ++heartbeat)
    {
        controller_->publish_control_commands();
        executor_.spin_some(DISCOVERY_WAIT);
    }

    px4_msgs::msg::VehicleCommandAck ack;
    ack.command = 176U;
    ack.result =
        px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED;
    vehicle_command_ack_pub_->publish(ack);
    executor_.spin_some(DISCOVERY_WAIT);

    const auto heartbeats_before_failure = offboard_control_mode_count_;
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::ERROR);
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_EQ(offboard_control_mode_count_, heartbeats_before_failure);
    EXPECT_EQ(vehicle_command_count_, 1U);
}

}  // namespace
