/**
 * @file planning_target_selector_test.cpp
 * @brief Planning Target 選擇的純 C++ 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 */

#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "ego_swarm/BsplineTrajectory.hpp"
#include "ego_swarm/PlanningTargetSelector.hpp"

namespace
{

using taiphoon::ego_swarm::LocalGoalConfig;
using taiphoon::ego_swarm::LocalGoalEnvironment;
using taiphoon::ego_swarm::PathConstraint;
using taiphoon::ego_swarm::PeerTrajectory;
using taiphoon::ego_swarm::select_planning_target;
using taiphoon::ego_swarm::Vector3;

class EmptyObstacleMap final : public taiphoon::perception::IObstacleMap
{
public:
    bool is_valid() const override
    {
        return true;
    }

    bool is_fresh(const double) const override
    {
        return true;
    }

    taiphoon::perception::ObstacleMapStatus status(const double) const override
    {
        return taiphoon::perception::ObstacleMapStatus::Fresh;
    }

    std::size_t point_count() const override
    {
        return 0U;
    }

    double nearest_distance(const Vector3&) const override
    {
        return std::numeric_limits<double>::infinity();
    }

    bool is_position_safe(const Vector3&, const double) const override
    {
        return true;
    }

    double obstacle_cost(const Vector3&) const override
    {
        return 0.0;
    }
};

class PointObstacleMap final : public taiphoon::perception::IObstacleMap
{
public:
    explicit PointObstacleMap(const Vector3& point)
    : point_(point)
    {
    }

    bool is_valid() const override
    {
        return true;
    }

    bool is_fresh(const double) const override
    {
        return true;
    }

    taiphoon::perception::ObstacleMapStatus status(const double) const override
    {
        return taiphoon::perception::ObstacleMapStatus::Fresh;
    }

    std::size_t point_count() const override
    {
        return 1U;
    }

    double nearest_distance(const Vector3& position) const override
    {
        return taiphoon::ego_swarm::distance_between_centers(position, point_);
    }

    bool is_position_safe(
        const Vector3& position,
        const double clearance_m) const override
    {
        return nearest_distance(position) >= clearance_m;
    }

    double obstacle_cost(const Vector3& position) const override
    {
        const double distance = nearest_distance(position);
        return distance > 0.0 ? 1.0 / distance :
            std::numeric_limits<double>::infinity();
    }

private:
    Vector3 point_;
};

TEST(PlanningTargetSelectorTest, DisabledFeaturePreservesFinalTarget)
{
    const auto result = select_planning_target(
        false,
        Vector3{0.0, 0.0, 0.0},
        Vector3{30.0, 0.0, 0.0},
        PathConstraint::FREE_3D,
        LocalGoalConfig{10.0});

    ASSERT_TRUE(result.is_valid);
    EXPECT_TRUE(result.is_final_goal);
    EXPECT_NEAR(result.goal.at(0), 30.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(1), 0.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(2), 0.0, 1.0e-9);
}

TEST(PlanningTargetSelectorTest, EnabledFeatureSelectsLocalGoal)
{
    const auto result = select_planning_target(
        true,
        Vector3{0.0, 0.0, 0.0},
        Vector3{30.0, 0.0, 0.0},
        PathConstraint::FREE_3D,
        LocalGoalConfig{10.0});

    ASSERT_TRUE(result.is_valid);
    EXPECT_FALSE(result.is_final_goal);
    EXPECT_NEAR(result.goal.at(0), 10.0, 1.0e-9);
}

TEST(PlanningTargetSelectorTest, EnabledFeatureKeepsPathConstraint)
{
    const auto result = select_planning_target(
        true,
        Vector3{0.0, 0.0, 5.0},
        Vector3{30.0, 0.0, 0.0},
        PathConstraint::FIXED_Z,
        LocalGoalConfig{10.0});

    ASSERT_TRUE(result.is_valid);
    EXPECT_NEAR(result.goal.at(0), 10.0, 1.0e-9);
    EXPECT_NEAR(result.goal.at(2), 0.0, 1.0e-9);
}

TEST(PlanningTargetSelectorTest, InvalidFinalTargetFailsClosed)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto result = select_planning_target(
        true,
        Vector3{0.0, 0.0, 0.0},
        Vector3{nan, 0.0, 0.0},
        PathConstraint::FREE_3D,
        LocalGoalConfig{10.0});

    EXPECT_FALSE(result.is_valid);
}

TEST(PlanningTargetSelectorTest, RejectsLocalGoalInsideTerminalPeerClearance)
{
    EmptyObstacleMap obstacle_map;

    PeerTrajectory peer;
    peer.agent_id = 2;
    peer.trajectory_id = 1U;
    peer.frame_revision = 1U;
    peer.message_time_s = 0.0;
    peer.start_time_s = 0.0;
    peer.receive_time_s = 0.0;
    peer.terminal_hold = true;
    peer.trajectory = taiphoon::ego_swarm::BsplineTrajectory(
        std::vector<Vector3>(4U, Vector3{8.0, 0.0, 0.0}), 1.0);
    peer.end_time_s = peer.start_time_s + peer.trajectory.duration_s();
    ASSERT_TRUE(peer.trajectory.is_valid());

    LocalGoalEnvironment environment;
    environment.now_s = 0.0;
    environment.candidate_start_time_s = 0.0;
    environment.self_agent_id = 1;
    environment.peers_are_fresh = true;
    environment.peers = {peer};
    environment.obstacle_map = &obstacle_map;
    environment.peer_collision_min_distance_m = 3.0;

    const auto result = select_planning_target(
        true,
        Vector3{0.0, 0.0, 0.0},
        Vector3{30.0, 0.0, 0.0},
        PathConstraint::FIXED_Z,
        LocalGoalConfig{10.0},
        environment);

    ASSERT_TRUE(result.is_valid);
    EXPECT_FALSE(result.is_final_goal);
    EXPECT_NEAR(result.goal.at(0), 10.0, 1.0e-9);
    EXPECT_GT(std::abs(result.goal.at(1)), 3.0);
}

TEST(PlanningTargetSelectorTest, RejectsLocalGoalInsideObstacleClearance)
{
    PointObstacleMap obstacle_map(Vector3{10.0, 0.0, 0.0});
    LocalGoalEnvironment environment;
    environment.now_s = 0.0;
    environment.candidate_start_time_s = 0.0;
    environment.self_agent_id = 1;
    environment.peers_are_fresh = true;
    environment.obstacle_map = &obstacle_map;
    environment.obstacle_clearance_m = 1.0;

    const auto result = select_planning_target(
        true,
        Vector3{0.0, 0.0, 0.0},
        Vector3{30.0, 0.0, 0.0},
        PathConstraint::FIXED_Z,
        LocalGoalConfig{10.0},
        environment);

    ASSERT_TRUE(result.is_valid);
    EXPECT_FALSE(result.is_final_goal);
    EXPECT_NEAR(result.goal.at(0), 10.0, 1.0e-9);
    EXPECT_GT(std::abs(result.goal.at(1)), 3.0);
}

}  // namespace
