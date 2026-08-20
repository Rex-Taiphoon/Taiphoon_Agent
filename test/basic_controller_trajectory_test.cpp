/**
 * @file basic_controller_trajectory_test.cpp
 * @brief BasicController 定時 B-spline 軌跡取樣與安全 hold 測試
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 */

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controllers/BasicController.hpp"
#include "controllers/FlightStateResolver.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr float POSITION_EPSILON_M = 1.0e-4F;
constexpr int MAX_PUBLISH_ATTEMPTS = 50;
constexpr auto DISCOVERY_WAIT = std::chrono::milliseconds(20);
constexpr uint8_t PX4_NAV_STATE_POSCTL = 4U;

using taiphoon::controllers::BasicController;
using taiphoon::controllers::TimedTrajectory;
using taiphoon::controllers::TimedTrajectorySampleResult;

class BasicControllerTrajectoryTest : public ::testing::Test
{
protected:
    /** @brief 初始化 ROS2 測試 context。 */
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok())
        {
            int argc = 0;
            rclcpp::init(argc, nullptr);
        }
    }

    /** @brief 關閉 ROS2 測試 context。 */
    static void TearDownTestSuite()
    {
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
    }

    /** @brief 建立控制器與 TrajectorySetpoint 觀測訂閱。 */
    void SetUp() override
    {
        static int node_index = 0;
        node_ = std::make_shared<rclcpp::Node>(
            "basic_controller_trajectory_test_" +
            std::to_string(node_index++));
        controller_ = std::make_unique<BasicController>(node_.get());
        ASSERT_TRUE(controller_->initialize());

        trajectory_setpoint_sub_ =
            node_->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
                "fmu/in/trajectory_setpoint",
                rclcpp::QoS(10).best_effort(),
                [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
                {
                    if (msg == nullptr)
                    {
                        return;
                    }
                    latest_setpoint_ = *msg;
                    has_setpoint_ = true;
                });
        vehicle_status_pub_ =
            node_->create_publisher<px4_msgs::msg::VehicleStatus>(
                "fmu/out/vehicle_status_v4", rclcpp::QoS(10).best_effort());
        executor_.add_node(node_);
        enter_offboard_session_for_test();
        has_setpoint_ = false;
    }

    /** @brief 釋放測試建立的 ROS2 資源。 */
    void TearDown() override
    {
        controller_->shutdown();
        executor_.remove_node(node_);
        trajectory_setpoint_sub_.reset();
        vehicle_status_pub_.reset();
        controller_.reset();
        node_.reset();
    }

    /**
     * @brief 建立指定目前位置的有限里程計。
     * @param x NED X
     * @param y NED Y
     * @param z NED Z
     * @return VehicleOdometry
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
     * @brief 建立線性測試軌跡。
     * @param start_time 軌跡開始時間
     * @param frame_revision 座標版本
     * @return 有效線性軌跡
     */
    TimedTrajectory make_linear_trajectory(
        const rclcpp::Time& start_time,
        uint32_t frame_revision = 1U) const
    {
        TimedTrajectory trajectory;
        trajectory.stamp = start_time;
        trajectory.start_time = start_time;
        trajectory.trajectory_id = 1U;
        trajectory.agent_index = 1U;
        trajectory.frame_revision = frame_revision;
        trajectory.order = 1U;
        trajectory.knot_interval_s = 1.0F;
        trajectory.control_points = {
            {0.0F, 0.0F, -1.0F},
            {10.0F, 20.0F, -3.0F}};
        trajectory.duration_s = 2.0F;
        trajectory.valid = true;
        return trajectory;
    }

    /** @brief 驅動發布直到測試訂閱者收到設定點。 */
    void publish_until_setpoint_received()
    {
        for (int attempt = 0;
             attempt < MAX_PUBLISH_ATTEMPTS && !has_setpoint_;
             ++attempt)
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

    /** @brief 驗證目前 PX4 設定點各數值欄位均為有限值。 */
    void expect_finite_setpoint_fields() const
    {
        for (const float value : latest_setpoint_.position)
        {
            EXPECT_TRUE(std::isfinite(value));
        }
        for (const float value : latest_setpoint_.velocity)
        {
            EXPECT_TRUE(std::isfinite(value));
        }
        for (const float value : latest_setpoint_.acceleration)
        {
            EXPECT_TRUE(std::isfinite(value));
        }
        for (const float value : latest_setpoint_.jerk)
        {
            EXPECT_TRUE(std::isfinite(value));
        }
        EXPECT_TRUE(std::isfinite(latest_setpoint_.yaw));
        EXPECT_TRUE(std::isfinite(latest_setpoint_.yawspeed));
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::unique_ptr<BasicController> controller_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
        trajectory_setpoint_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr
        vehicle_status_pub_;
    px4_msgs::msg::TrajectorySetpoint latest_setpoint_;
    bool has_setpoint_{false};
};

TEST_F(BasicControllerTrajectoryTest, SamplesLinearTrajectoryAtRosTime)
{
    const auto start_time = node_->now();
    const auto trajectory = make_linear_trajectory(start_time);
    ASSERT_TRUE(controller_->update_timed_trajectory(trajectory));

    std::array<float, 3> position{};
    const auto result = controller_->sample_timed_trajectory(
        start_time + rclcpp::Duration::from_seconds(1.0), position);

    EXPECT_EQ(result, TimedTrajectorySampleResult::VALID);
    EXPECT_NEAR(position[0], 5.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(position[1], 10.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(position[2], -2.0F, POSITION_EPSILON_M);
}

TEST_F(BasicControllerTrajectoryTest, SamplesCubicTrajectoryWithAnalyticDerivatives)
{
    TimedTrajectory trajectory;
    trajectory.stamp = node_->now();
    trajectory.start_time = trajectory.stamp;
    trajectory.trajectory_id = 2U;
    trajectory.agent_index = 1U;
    trajectory.frame_revision = 1U;
    trajectory.order = 3U;
    trajectory.knot_interval_s = 1.0F;
    trajectory.control_points = {
        {0.0F, 0.0F, 0.0F},
        {1.0F, 2.0F, -1.0F},
        {2.0F, 4.0F, -2.0F},
        {3.0F, 6.0F, -3.0F}};
    trajectory.duration_s = 1.0F;
    trajectory.valid = true;
    ASSERT_TRUE(controller_->update_timed_trajectory(trajectory));

    std::array<float, 3> position{};
    std::array<float, 3> velocity{};
    std::array<float, 3> acceleration{};
    const auto result = controller_->sample_timed_trajectory(
        trajectory.start_time + rclcpp::Duration::from_seconds(0.5), position,
        velocity, acceleration);

    EXPECT_EQ(result, TimedTrajectorySampleResult::VALID);
    EXPECT_NEAR(position[0], 1.5F, POSITION_EPSILON_M);
    EXPECT_NEAR(position[1], 3.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(position[2], -1.5F, POSITION_EPSILON_M);
    EXPECT_NEAR(velocity[0], 3.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(velocity[1], 6.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(velocity[2], -3.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(acceleration[0], 0.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(acceleration[1], 0.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(acceleration[2], 0.0F, POSITION_EPSILON_M);
}

TEST_F(BasicControllerTrajectoryTest,
       PublishesFiniteAnalyticFeedForwardToPx4)
{
    const auto start_time = node_->now() -
        rclcpp::Duration::from_seconds(0.25);
    TimedTrajectory trajectory;
    trajectory.stamp = start_time;
    trajectory.start_time = start_time;
    trajectory.trajectory_id = 7U;
    trajectory.agent_index = 1U;
    trajectory.frame_revision = 1U;
    trajectory.order = 3U;
    trajectory.knot_interval_s = 1.0F;
    trajectory.control_points = {
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F},
        {1.0F, 0.0F, 0.0F}};
    trajectory.duration_s = 1.0F;
    trajectory.valid = true;
    ASSERT_TRUE(controller_->update_timed_trajectory(trajectory));
    ASSERT_TRUE(controller_->set_ned_target(1.0F, 0.0F, 0.0F, 0.0F));

    publish_until_setpoint_received();

    expect_finite_setpoint_fields();
    EXPECT_GT(latest_setpoint_.velocity[0], 0.1F);
    EXPECT_GT(latest_setpoint_.acceleration[0], 0.1F);
}

TEST_F(BasicControllerTrajectoryTest,
       RejectsNonFiniteTrajectoryAndKeepsLastValidTrajectory)
{
    const auto start_time = node_->now();
    const auto valid_trajectory = make_linear_trajectory(start_time);
    ASSERT_TRUE(controller_->update_timed_trajectory(valid_trajectory));

    auto invalid_trajectory = valid_trajectory;
    invalid_trajectory.trajectory_id = 3U;
    invalid_trajectory.control_points[1][0] =
        std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(controller_->update_timed_trajectory(invalid_trajectory));

    std::array<float, 3> position{};
    const auto result = controller_->sample_timed_trajectory(
        start_time + rclcpp::Duration::from_seconds(1.0), position);

    EXPECT_EQ(result, TimedTrajectorySampleResult::VALID);
    EXPECT_NEAR(position[0], 5.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(position[1], 10.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(position[2], -2.0F, POSITION_EPSILON_M);
}

TEST_F(BasicControllerTrajectoryTest,
       ShortTrajectoryGapUsesLastValidTrajectoryWithoutAgeExpansion)
{
    const auto start_time = node_->now();
    const auto trajectory = make_linear_trajectory(start_time);
    ASSERT_TRUE(controller_->update_timed_trajectory(trajectory));

    std::array<float, 3> first_position{};
    std::array<float, 3> second_position{};
    EXPECT_EQ(
        controller_->sample_timed_trajectory(
            start_time + rclcpp::Duration::from_seconds(0.25),
            first_position),
        TimedTrajectorySampleResult::VALID);
    EXPECT_EQ(
        controller_->sample_timed_trajectory(
            start_time + rclcpp::Duration::from_seconds(0.5),
            second_position),
        TimedTrajectorySampleResult::VALID);

    // 沒有收到新軌跡時仍取樣同一份有效 B-spline；控制器不引入
    // 依資料年齡變化的額外安全半徑。
    EXPECT_NEAR(first_position[0], 1.25F, POSITION_EPSILON_M);
    EXPECT_NEAR(second_position[0], 2.5F, POSITION_EPSILON_M);
    EXPECT_NEAR(first_position[1], 2.5F, POSITION_EPSILON_M);
    EXPECT_NEAR(second_position[1], 5.0F, POSITION_EPSILON_M);
}

TEST_F(BasicControllerTrajectoryTest,
       FrameRevisionChangeClearsTrajectoryUntilNewRevisionIsSynchronized)
{
    const auto start_time = node_->now();
    const auto first_trajectory = make_linear_trajectory(start_time, 1U);
    ASSERT_TRUE(controller_->update_timed_trajectory(first_trajectory));

    auto changed_frame_trajectory = make_linear_trajectory(start_time, 2U);
    changed_frame_trajectory.trajectory_id = 4U;
    EXPECT_FALSE(controller_->update_timed_trajectory(changed_frame_trajectory));
    EXPECT_FALSE(controller_->has_timed_trajectory());

    std::array<float, 3> position{};
    EXPECT_EQ(
        controller_->sample_timed_trajectory(
            start_time + rclcpp::Duration::from_seconds(0.5), position),
        TimedTrajectorySampleResult::NO_TRAJECTORY);

    changed_frame_trajectory.trajectory_id = 5U;
    ASSERT_TRUE(controller_->update_timed_trajectory(changed_frame_trajectory));
    EXPECT_EQ(
        controller_->sample_timed_trajectory(
            start_time + rclcpp::Duration::from_seconds(0.5), position),
        TimedTrajectorySampleResult::VALID);
}

TEST_F(BasicControllerTrajectoryTest,
       CompletedTrajectoryPublishesEndpointOnce)
{
    controller_->update_odometry(make_odometry(4.0F, -2.0F, -3.0F));
    ASSERT_TRUE(controller_->set_ned_target(100.0F, 100.0F, -3.0F, 0.0F));

    const auto expired_start =
        node_->now() - rclcpp::Duration::from_seconds(2.0);
    auto trajectory = make_linear_trajectory(expired_start);
    trajectory.trajectory_id = 6U;
    trajectory.duration_s = 1.0F;
    ASSERT_TRUE(controller_->update_timed_trajectory(trajectory));

    std::array<float, 3> sampled_position{};
    EXPECT_EQ(
        controller_->sample_timed_trajectory(
            node_->now(), sampled_position),
        TimedTrajectorySampleResult::COMPLETED);

    publish_until_setpoint_received();

    expect_finite_setpoint_fields();
    EXPECT_TRUE(std::isfinite(latest_setpoint_.position[0]));
    EXPECT_TRUE(std::isfinite(latest_setpoint_.position[1]));
    EXPECT_TRUE(std::isfinite(latest_setpoint_.position[2]));
    // 軌跡抵達終點後，hold setpoint 必須鎖在軌跡終點，不能因即時
    // odometry 漂移而改變。
    EXPECT_NEAR(latest_setpoint_.position[0], 10.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[1], 20.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[2], -3.0F, POSITION_EPSILON_M);
    for (const float value : latest_setpoint_.velocity)
    {
        EXPECT_FLOAT_EQ(value, 0.0F);
    }
    for (const float value : latest_setpoint_.acceleration)
    {
        EXPECT_FLOAT_EQ(value, 0.0F);
    }

    has_setpoint_ = false;
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_FALSE(has_setpoint_);
    EXPECT_FALSE(controller_->has_timed_trajectory());
}

TEST_F(BasicControllerTrajectoryTest,
       CompletedTrajectoryReturnsEndpointWithZeroFeedForward)
{
    const auto start_time = node_->now() -
        rclcpp::Duration::from_seconds(2.0);
    auto trajectory = make_linear_trajectory(start_time);
    trajectory.trajectory_id = 8U;
    trajectory.duration_s = 1.0F;
    ASSERT_TRUE(controller_->update_timed_trajectory(trajectory));

    std::array<float, 3> position{};
    std::array<float, 3> velocity{1.0F, 1.0F, 1.0F};
    std::array<float, 3> acceleration{1.0F, 1.0F, 1.0F};
    EXPECT_EQ(
        controller_->sample_timed_trajectory(
            node_->now(), position, velocity, acceleration),
        TimedTrajectorySampleResult::COMPLETED);
    EXPECT_NEAR(position[0], 10.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(position[1], 20.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(position[2], -3.0F, POSITION_EPSILON_M);
    for (const float value : velocity)
    {
        EXPECT_FLOAT_EQ(value, 0.0F);
    }
    for (const float value : acceleration)
    {
        EXPECT_FLOAT_EQ(value, 0.0F);
    }
}

TEST_F(BasicControllerTrajectoryTest,
       StreamsActiveTrajectoryAndStopsAfterCompletion)
{
    ASSERT_TRUE(controller_->set_ned_target(2.0F, 0.0F, -2.0F, 0.0F));

    const auto active_start = node_->now() -
        rclcpp::Duration::from_seconds(0.25);
    auto active_trajectory = make_linear_trajectory(active_start);
    active_trajectory.trajectory_id = 10U;
    active_trajectory.duration_s = 1.0F;
    ASSERT_TRUE(controller_->update_timed_trajectory(active_trajectory));

    publish_until_setpoint_received();
    has_setpoint_ = false;
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_TRUE(has_setpoint_);

    has_setpoint_ = false;
    auto terminal_trajectory = active_trajectory;
    terminal_trajectory.trajectory_id = 11U;
    terminal_trajectory.start_time = node_->now() -
        rclcpp::Duration::from_seconds(2.0);
    terminal_trajectory.stamp = terminal_trajectory.start_time;
    ASSERT_TRUE(controller_->update_timed_trajectory(terminal_trajectory));

    publish_until_setpoint_received();
    EXPECT_NEAR(latest_setpoint_.position[0], 10.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[1], 20.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(latest_setpoint_.position[2], -3.0F, POSITION_EPSILON_M);
    EXPECT_EQ(controller_->get_flight_state(),
              taiphoon::controllers::FlightState::HOVER);

    has_setpoint_ = false;
    controller_->publish_control_commands();
    executor_.spin_some(DISCOVERY_WAIT);
    EXPECT_FALSE(has_setpoint_);
    EXPECT_FALSE(controller_->has_timed_trajectory());
}

TEST_F(BasicControllerTrajectoryTest,
       CompletedTrajectoryRejectsStaleIdAndAcceptsNewId)
{
    ASSERT_TRUE(controller_->set_ned_target(2.0F, 0.0F, -2.0F, 0.0F));

    auto completed_trajectory = make_linear_trajectory(
        node_->now() - rclcpp::Duration::from_seconds(2.0));
    completed_trajectory.trajectory_id = 20U;
    completed_trajectory.duration_s = 1.0F;
    ASSERT_TRUE(controller_->update_timed_trajectory(completed_trajectory));

    publish_until_setpoint_received();
    EXPECT_FALSE(controller_->has_timed_trajectory());

    EXPECT_FALSE(controller_->update_timed_trajectory(completed_trajectory));
    EXPECT_FALSE(controller_->has_timed_trajectory());

    auto new_trajectory = make_linear_trajectory(node_->now());
    new_trajectory.trajectory_id = 21U;
    ASSERT_TRUE(controller_->update_timed_trajectory(new_trajectory));
    EXPECT_TRUE(controller_->has_timed_trajectory());
}

TEST_F(BasicControllerTrajectoryTest,
       PreTakeoffTrajectoryCannotOverrideTakeoffSetpoint)
{
    controller_->update_odometry(make_odometry(0.0F, 0.0F, 0.0F));

    auto pre_takeoff_hold = make_linear_trajectory(
        node_->now() - rclcpp::Duration::from_seconds(2.0));
    pre_takeoff_hold.trajectory_id = 9U;
    pre_takeoff_hold.control_points = {
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F}};
    pre_takeoff_hold.duration_s = 1.0F;
    ASSERT_TRUE(controller_->update_timed_trajectory(pre_takeoff_hold));

    ASSERT_TRUE(controller_->takeoff(8.0F));
    EXPECT_FALSE(controller_->has_timed_trajectory());
    EXPECT_FALSE(controller_->update_timed_trajectory(pre_takeoff_hold));

    // TAKEOFF 狀態必須維持 BasicController 的本機起飛目標。
    publish_until_setpoint_received();
    EXPECT_NEAR(latest_setpoint_.position[2], -8.0F, POSITION_EPSILON_M);
}

}  // namespace
