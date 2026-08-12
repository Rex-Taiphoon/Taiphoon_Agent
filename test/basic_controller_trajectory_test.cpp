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
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr float POSITION_EPSILON_M = 1.0e-4F;
constexpr int MAX_PUBLISH_ATTEMPTS = 50;
constexpr auto DISCOVERY_WAIT = std::chrono::milliseconds(20);

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
        executor_.add_node(node_);
    }

    /** @brief 釋放測試建立的 ROS2 資源。 */
    void TearDown() override
    {
        controller_->shutdown();
        executor_.remove_node(node_);
        trajectory_setpoint_sub_.reset();
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

TEST_F(BasicControllerTrajectoryTest, SamplesCubicTrajectoryWithDeBoor)
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
    const auto result = controller_->sample_timed_trajectory(
        trajectory.start_time + rclcpp::Duration::from_seconds(0.5), position);

    EXPECT_EQ(result, TimedTrajectorySampleResult::VALID);
    EXPECT_NEAR(position[0], 1.5F, POSITION_EPSILON_M);
    EXPECT_NEAR(position[1], 3.0F, POSITION_EPSILON_M);
    EXPECT_NEAR(position[2], -1.5F, POSITION_EPSILON_M);
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
       ExpiredTrajectoryPublishesTrajectoryEndpointHold)
{
    controller_->update_odometry(make_odometry(4.0F, -2.0F, -3.0F));
    ASSERT_TRUE(controller_->set_ned_target(100.0F, 100.0F, -3.0F, 0.0F));

    const auto expired_start =
        node_->now() - rclcpp::Duration::from_seconds(2.0);
    auto trajectory = make_linear_trajectory(expired_start);
    trajectory.trajectory_id = 6U;
    trajectory.duration_s = 1.0F;
    ASSERT_TRUE(controller_->update_timed_trajectory(trajectory));

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
}

}  // namespace
