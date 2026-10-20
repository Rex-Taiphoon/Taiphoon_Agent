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
#include <thread>
#include <vector>

#include "controllers/BasicController.hpp"
#include "controllers/FlightStateResolver.hpp"
#include "controllers/YawSlewLimiter.hpp"
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
constexpr float YAW_EPSILON_RAD = 1.0e-4F;
constexpr int MAX_YAW_SLEW_CYCLES = 200;
/// 預設 30 deg/s、10 Hz 下每個控制週期最多 3 度
constexpr float DEFAULT_YAW_STEP_RAD =
    static_cast<float>(taiphoon::controllers::DEFAULT_YAW_RATE_LIMIT_DEG_S) *
    taiphoon::controllers::YAW_DEG_TO_RAD *
    taiphoon::controllers::DEFAULT_CONTROL_PERIOD_S;
constexpr float DEFAULT_YAW_RATE_RAD_S =
    static_cast<float>(taiphoon::controllers::DEFAULT_YAW_RATE_LIMIT_DEG_S) *
    taiphoon::controllers::YAW_DEG_TO_RAD;

/** @brief 角度轉弧度。 */
float deg_to_rad(float degrees)
{
    return degrees * taiphoon::controllers::YAW_DEG_TO_RAD;
}

/** @brief 以最短角度比較兩個偏航角。 */
float yaw_error_rad(float actual, float expected)
{
    return std::abs(taiphoon::controllers::shortest_yaw_delta_rad(actual, expected));
}

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
                    if (msg != nullptr)
                    {
                        latest_vehicle_command_ = *msg;
                        ++vehicle_command_count_;
                    }
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

    /**
     * @brief 逐週期驅動控制迴圈，直到某一週期不再送出設定點
     * @param max_cycles 最多控制週期數
     * @return 期間依序收到的設定點
     */
    std::vector<px4_msgs::msg::TrajectorySetpoint> publish_until_setpoints_stop(
        int max_cycles = MAX_YAW_SLEW_CYCLES)
    {
        std::vector<px4_msgs::msg::TrajectorySetpoint> received;
        for (int cycle = 0; cycle < max_cycles; ++cycle)
        {
            has_setpoint_ = false;
            controller_->publish_control_commands();
            executor_.spin_some(DISCOVERY_WAIT);
            if (!has_setpoint_)
            {
                break;
            }
            received.push_back(latest_setpoint_);
        }
        return received;
    }

    /**
     * @brief 驗證連續設定點的偏航步進不超過單週期上限，且前饋方向與步進一致
     * @param previous_yaw 第一筆之前已送出的偏航（弧度）
     * @param setpoints 依序收到的設定點
     */
    void expect_rate_limited_yaw(
        float previous_yaw,
        const std::vector<px4_msgs::msg::TrajectorySetpoint>& setpoints) const
    {
        for (const auto& setpoint : setpoints)
        {
            const float step = taiphoon::controllers::shortest_yaw_delta_rad(
                previous_yaw, setpoint.yaw);
            EXPECT_LE(std::abs(step), DEFAULT_YAW_STEP_RAD + YAW_EPSILON_RAD);
            EXPECT_LE(std::abs(setpoint.yawspeed),
                      DEFAULT_YAW_RATE_RAD_S + YAW_EPSILON_RAD);
            if (std::abs(setpoint.yawspeed) > YAW_EPSILON_RAD)
            {
                EXPECT_GT(step * setpoint.yawspeed, 0.0F);
            }
            previous_yaw = setpoint.yaw;
        }
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
    px4_msgs::msg::VehicleCommand latest_vehicle_command_;
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
       DisarmStopsOffboardOutputAndPublishesDisarmCommand)
{
    ASSERT_TRUE(controller_->disarm());
    executor_.spin_some(DISCOVERY_WAIT);

    EXPECT_GE(vehicle_command_count_, 1U);
    EXPECT_EQ(latest_vehicle_command_.command, 400U);
    EXPECT_FLOAT_EQ(latest_vehicle_command_.param1, 0.0F);
    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::IDLE);

    has_setpoint_ = false;
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_FALSE(has_setpoint_);
    EXPECT_EQ(offboard_control_mode_count_, 0U);
}

TEST_F(BasicControllerSetpointTest,
       DirectLandPublishesLandCommandAndLeavesOffboard)
{
    ASSERT_TRUE(controller_->land());
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);

    EXPECT_EQ(latest_vehicle_command_.command, 21U);
    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::LAND);
    EXPECT_EQ(offboard_control_mode_count_, 0U);
}

TEST_F(BasicControllerSetpointTest,
       SwitchToHoldPublishesPositionHoldCommandAndLeavesOffboard)
{
    ASSERT_TRUE(controller_->switch_to_hold_mode());
    executor_.spin_some(DISCOVERY_WAIT);

    EXPECT_EQ(latest_vehicle_command_.command, 176U);
    EXPECT_FLOAT_EQ(latest_vehicle_command_.param2, 8.0F);
    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::IDLE);
    EXPECT_EQ(offboard_control_mode_count_, 0U);
}

TEST_F(BasicControllerSetpointTest,
       TakeoffLandSequenceStartsNewOffboardSession)
{
    ASSERT_TRUE(controller_->execute_takeoff_land_sequence(6.0F, 2.0F));

    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::OFFBOARD_INIT);
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
       HoldCurrentPositionLocksAllAxesAgainstOdometryDrift)
{
    controller_->update_odometry(make_odometry(3.0F, 4.0F, -5.0F));
    ASSERT_TRUE(controller_->hold_current_position());

    controller_->update_odometry(make_odometry(-7.0F, 11.0F, -4.0F));
    publish_until_setpoint_received();

    EXPECT_NEAR(latest_setpoint_.position[0], 3.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[1], 4.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[2], -5.0F, POSITION_EPSILON_M);
}

TEST_F(BasicControllerSetpointTest,
       DirectMovementCommandsPublishExpectedNedTargets)
{
    controller_->update_odometry(make_odometry_with_yaw(
        1.0F, 2.0F, -4.0F, 0.0F));

    ASSERT_TRUE(controller_->move_forward(3.0F));
    publish_until_setpoint_received();
    EXPECT_NEAR(latest_setpoint_.position[0], 4.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[1], 2.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[2], -4.0F, POSITION_EPSILON_M);

    has_setpoint_ = false;
    ASSERT_TRUE(controller_->move_right(2.0F));
    publish_until_setpoint_received();
    EXPECT_NEAR(latest_setpoint_.position[0], 1.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[1], 4.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[2], -4.0F, POSITION_EPSILON_M);

    has_setpoint_ = false;
    ASSERT_TRUE(controller_->move_vertical(1.0F));
    publish_until_setpoint_received();
    EXPECT_NEAR(latest_setpoint_.position[0], 1.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[1], 4.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[2], -5.0F, POSITION_EPSILON_M);

    has_setpoint_ = false;
    ASSERT_TRUE(controller_->set_ned_target(8.0F, 9.0F, -6.0F, 45.0F));
    publish_until_setpoint_received();
    EXPECT_NEAR(latest_setpoint_.position[0], 8.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[1], 9.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[2], -6.0F, POSITION_EPSILON_M);
    // 絕對 yaw 不再一次跳到 45 度，而是自目前送出航向（0 度）限速逼近。
    EXPECT_NEAR(latest_setpoint_.yaw, DEFAULT_YAW_STEP_RAD, YAW_EPSILON_RAD);

    const auto slew = publish_until_setpoints_stop();
    ASSERT_FALSE(slew.empty());
    expect_rate_limited_yaw(DEFAULT_YAW_STEP_RAD, slew);
    EXPECT_LT(yaw_error_rad(slew.back().yaw, deg_to_rad(45.0F)), YAW_EPSILON_RAD);
    EXPECT_NEAR(slew.back().position[0], 8.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(slew.back().position[1], 9.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(slew.back().position[2], -6.0F, POSITION_EPSILON_M);
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
    const float initial_yaw_rad = deg_to_rad(30.0F);
    const float expected_yaw_rad = deg_to_rad(120.0F);

    controller_->update_odometry(make_odometry_with_yaw(
        3.0F, 4.0F, -5.0F, initial_yaw_rad));
    ASSERT_TRUE(controller_->hold_position(2.0F));
    ASSERT_TRUE(controller_->rotate_yaw(90.0F));

    publish_until_setpoint_received();

    // 第一筆只前進一個限速步進，並以正（順時針）yawspeed 前饋。
    EXPECT_LT(yaw_error_rad(latest_setpoint_.yaw,
                            initial_yaw_rad + DEFAULT_YAW_STEP_RAD),
              YAW_EPSILON_RAD);
    EXPECT_NEAR(latest_setpoint_.yawspeed, DEFAULT_YAW_RATE_RAD_S, YAW_EPSILON_RAD);

    const float first_yaw = latest_setpoint_.yaw;
    const auto slew = publish_until_setpoints_stop();
    ASSERT_FALSE(slew.empty());
    expect_rate_limited_yaw(first_yaw, slew);
    // 90 度 ÷ 3 度/週期：約 30 週期，第一筆已在上面收到。
    EXPECT_GE(slew.size() + 1U, 30U);
    EXPECT_LE(slew.size() + 1U, 31U);
    EXPECT_LT(yaw_error_rad(slew.back().yaw, expected_yaw_rad), YAW_EPSILON_RAD);
    EXPECT_FLOAT_EQ(slew.back().yawspeed, 0.0F);
    // 純旋轉期間位置維持 hold，不得跟隨 odometry。
    for (const auto& setpoint : slew)
    {
        EXPECT_NEAR(setpoint.position[0], 3.0F, POSITION_EPSILON_M);
        EXPECT_NEAR(setpoint.position[1], 4.0F, POSITION_EPSILON_M);
        EXPECT_NEAR(setpoint.position[2], -5.0F, POSITION_EPSILON_M);
    }
}

TEST_F(BasicControllerSetpointTest, AbsoluteYawSetsMovePointHeading)
{
    const float expected_yaw_rad = deg_to_rad(-160.0F);

    controller_->update_odometry(make_odometry(3.0F, 4.0F, -5.0F));
    ASSERT_TRUE(controller_->hold_position(2.0F));
    ASSERT_TRUE(controller_->set_absolute_yaw(200.0F));

    publish_until_setpoint_received();

    // 0 度到 200 度（= -160 度）的最短路徑是逆時針。
    EXPECT_LT(yaw_error_rad(latest_setpoint_.yaw, -DEFAULT_YAW_STEP_RAD),
              YAW_EPSILON_RAD);
    EXPECT_NEAR(latest_setpoint_.yawspeed, -DEFAULT_YAW_RATE_RAD_S, YAW_EPSILON_RAD);

    const float first_yaw = latest_setpoint_.yaw;
    const auto slew = publish_until_setpoints_stop();
    ASSERT_FALSE(slew.empty());
    expect_rate_limited_yaw(first_yaw, slew);
    EXPECT_LT(yaw_error_rad(slew.back().yaw, expected_yaw_rad), YAW_EPSILON_RAD);
}

TEST_F(BasicControllerSetpointTest, TakeoffLocksCurrentHeadingInsteadOfNorth)
{
    // 回歸：實機起飛後機頭轉向正北，因為目標航向未設定時送出 yaw=0。
    const float heading_rad = deg_to_rad(120.0F);
    controller_->update_odometry(make_odometry_with_yaw(
        1.0F, 2.0F, -0.1F, heading_rad));
    ASSERT_TRUE(controller_->takeoff(6.0F));

    publish_until_setpoint_received();
    EXPECT_LT(yaw_error_rad(latest_setpoint_.yaw, heading_rad), YAW_EPSILON_RAD);
    EXPECT_FLOAT_EQ(latest_setpoint_.yawspeed, 0.0F);

    // 航向已鎖定，不應再為偏航逼近重送。
    has_setpoint_ = false;
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_FALSE(has_setpoint_);
}

TEST_F(BasicControllerSetpointTest, HoldWithoutYawTargetKeepsCurrentHeading)
{
    // 目標航向未設定時的防呆值是目前航向，不是 0（北）。
    const float heading_rad = deg_to_rad(-75.0F);
    controller_->update_odometry(make_odometry_with_yaw(
        3.0F, 4.0F, -5.0F, heading_rad));
    ASSERT_TRUE(controller_->hold_current_position());

    publish_until_setpoint_received();
    EXPECT_LT(yaw_error_rad(latest_setpoint_.yaw, heading_rad), YAW_EPSILON_RAD);
    EXPECT_FLOAT_EQ(latest_setpoint_.yawspeed, 0.0F);
}

TEST_F(BasicControllerSetpointTest, GroundTakeoffRelocksHeadingAfterLanding)
{
    controller_->update_odometry(make_odometry_with_yaw(0.0F, 0.0F, -0.1F, 0.0F));
    ASSERT_TRUE(controller_->takeoff(5.0F));
    publish_until_setpoint_received();
    ASSERT_TRUE(controller_->rotate_yaw(90.0F));
    const auto first_flight = publish_until_setpoints_stop();
    ASSERT_FALSE(first_flight.empty());
    EXPECT_LT(yaw_error_rad(first_flight.back().yaw, deg_to_rad(90.0F)),
              YAW_EPSILON_RAD);

    // 落地後機體被搬動到另一個航向；下一次起飛必須鎖定新航向，不得轉回 90 度。
    publish_land_detected(true);
    const float new_heading_rad = deg_to_rad(-45.0F);
    controller_->update_odometry(make_odometry_with_yaw(
        5.0F, 5.0F, -0.1F, new_heading_rad));
    ASSERT_TRUE(controller_->takeoff(5.0F));
    has_setpoint_ = false;
    publish_until_setpoint_received();
    EXPECT_LT(yaw_error_rad(latest_setpoint_.yaw, new_heading_rad), YAW_EPSILON_RAD);
    EXPECT_FLOAT_EQ(latest_setpoint_.yawspeed, 0.0F);
}

TEST_F(BasicControllerSetpointTest, AirborneTakeoffKeepsTargetHeading)
{
    controller_->update_odometry(make_odometry_with_yaw(0.0F, 0.0F, -0.1F, 0.0F));
    ASSERT_TRUE(controller_->takeoff(5.0F));
    publish_until_setpoint_received();
    ASSERT_TRUE(controller_->rotate_yaw(90.0F));
    ASSERT_FALSE(publish_until_setpoints_stop().empty());

    // 空中重送 TAKEOFF（未落地）保留目標航向，與 planner 的 target yaw 一致。
    publish_land_detected(false);
    controller_->update_odometry(make_odometry_with_yaw(
        0.0F, 0.0F, -5.0F, deg_to_rad(80.0F)));
    ASSERT_TRUE(controller_->takeoff(7.0F));
    has_setpoint_ = false;
    publish_until_setpoint_received();
    EXPECT_LT(yaw_error_rad(latest_setpoint_.yaw, deg_to_rad(90.0F)), YAW_EPSILON_RAD);
    EXPECT_NEAR(latest_setpoint_.position[2], -7.0F, POSITION_EPSILON_M);
}

TEST_F(BasicControllerSetpointTest, ConsecutiveYawCommandsAccumulateOnFinalTarget)
{
    controller_->update_odometry(make_odometry_with_yaw(0.0F, 0.0F, -0.1F, 0.0F));
    ASSERT_TRUE(controller_->takeoff(5.0F));
    publish_until_setpoint_received();

    // 第一個 YAW 尚在逼近中就收到第二個 YAW：累加基準是最終目標（90 度），
    // 不是逼近中的送出值，與 planner 的 target yaw 累加方式相同。
    ASSERT_TRUE(controller_->rotate_yaw(90.0F));
    for (int cycle = 0; cycle < 5; ++cycle)
    {
        controller_->publish_control_commands();
        executor_.spin_some(DISCOVERY_WAIT);
    }
    ASSERT_TRUE(controller_->rotate_yaw(45.0F));
    const auto slew = publish_until_setpoints_stop();
    ASSERT_FALSE(slew.empty());
    EXPECT_LT(yaw_error_rad(slew.back().yaw, deg_to_rad(135.0F)), YAW_EPSILON_RAD);
}

TEST_F(BasicControllerSetpointTest, OffboardReentryStartsYawFromActualHeading)
{
    controller_->update_odometry(make_odometry_with_yaw(0.0F, 0.0F, -0.1F, 0.0F));
    ASSERT_TRUE(controller_->takeoff(5.0F));
    publish_until_setpoint_received();
    ASSERT_TRUE(controller_->rotate_yaw(90.0F));
    ASSERT_FALSE(publish_until_setpoints_stop().empty());

    // PX4 離開 Offboard 期間機頭被轉到 180 度。
    publish_status(PX4_NAV_STATE_POSCTL);
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    ASSERT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::IDLE);
    controller_->update_odometry(make_odometry_with_yaw(
        0.0F, 0.0F, -5.0F, deg_to_rad(180.0F)));

    enter_offboard_session_for_test();
    ASSERT_TRUE(controller_->hold_current_position());
    has_setpoint_ = false;
    publish_until_setpoint_received();

    // 重新進入後從實際航向（180 度）限速轉回目標 90 度，不得瞬間跳轉。
    EXPECT_LT(yaw_error_rad(latest_setpoint_.yaw,
                            deg_to_rad(180.0F) - DEFAULT_YAW_STEP_RAD),
              YAW_EPSILON_RAD);
    EXPECT_NEAR(latest_setpoint_.yawspeed, -DEFAULT_YAW_RATE_RAD_S, YAW_EPSILON_RAD);
}

TEST_F(BasicControllerSetpointTest, YawRateLimitConfigurationChangesStepAndRejectsInvalid)
{
    EXPECT_NEAR(controller_->get_yaw_rate_limit_deg_s(),
                static_cast<float>(taiphoon::controllers::DEFAULT_YAW_RATE_LIMIT_DEG_S),
                1.0e-4F);
    EXPECT_FALSE(controller_->set_yaw_rate_limit(0.0F, 0.1F));
    EXPECT_FALSE(controller_->set_yaw_rate_limit(std::nanf(""), 0.1F));
    EXPECT_FALSE(controller_->set_yaw_rate_limit(500.0F, 0.1F));
    EXPECT_FALSE(controller_->set_yaw_rate_limit(45.0F, 0.0F));
    EXPECT_NEAR(controller_->get_yaw_rate_limit_deg_s(),
                static_cast<float>(taiphoon::controllers::DEFAULT_YAW_RATE_LIMIT_DEG_S),
                1.0e-4F);

    ASSERT_TRUE(controller_->set_yaw_rate_limit(90.0F, 0.1F));
    controller_->update_odometry(make_odometry_with_yaw(3.0F, 4.0F, -5.0F, 0.0F));
    ASSERT_TRUE(controller_->hold_position(2.0F));
    ASSERT_TRUE(controller_->rotate_yaw(90.0F));
    publish_until_setpoint_received();
    EXPECT_LT(yaw_error_rad(latest_setpoint_.yaw, deg_to_rad(9.0F)), YAW_EPSILON_RAD);
    EXPECT_NEAR(latest_setpoint_.yawspeed, deg_to_rad(90.0F), YAW_EPSILON_RAD);
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
    EXPECT_EQ(latest_vehicle_command_.command, 20U);

    const auto command_count_after_rtl = vehicle_command_count_;
    ASSERT_TRUE(controller_->switch_to_land_mode());
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_FALSE(has_setpoint_);
    EXPECT_EQ(offboard_control_mode_count_, 0U);
    EXPECT_GT(vehicle_command_count_, command_count_after_rtl);
    EXPECT_EQ(latest_vehicle_command_.command, 21U);
}

TEST_F(BasicControllerSetpointTest,
       VehicleStatusGapKeepsOffboardUntilTimeoutThenStops)
{
    // 回歸：SITL 實測 VehicleStatus 最長間隔約 3.1 s，不得因此中止 Offboard。
    offboard_control_mode_count_ = 0U;
    std::this_thread::sleep_for(std::chrono::milliseconds(3200));
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::OFFBOARD_ACTIVE);
    EXPECT_GT(offboard_control_mode_count_, 0U);

    // 超過 PX4_STATE_TIMEOUT_MS 仍未收到狀態才停止輸出。
    std::this_thread::sleep_for(std::chrono::milliseconds(
        taiphoon::controllers::PX4_STATE_TIMEOUT_MS - 3200 + 300));
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::IDLE);

    const auto heartbeats_after_stop = offboard_control_mode_count_;
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_EQ(offboard_control_mode_count_, heartbeats_after_stop);
}

TEST_F(BasicControllerSetpointTest,
       LeavingOffboardStopsOutputWithoutWaitingForTimeout)
{
    publish_status(PX4_NAV_STATE_POSCTL);
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::IDLE);
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
