/**
 * @file collision_safety_monitor_test.cpp
 * @brief 多機防撞安全監控器單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 */

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <vector>

#include "safety/CollisionSafetyMonitor.hpp"

using taiphoon::safety::CollisionSafetyMonitor;
using taiphoon::safety::CollisionTargetAction;
using taiphoon::safety::CollisionTargetGate;
using taiphoon::safety::PeerCollisionObservation;

TEST(CollisionSafetyMonitorTest, AllowsPathBeyondThreeMeters)
{
    CollisionSafetyMonitor monitor(3.0F);
    const std::array<float, 3> current{0.0F, 0.0F, 0.0F};
    const std::array<float, 3> target{10.0F, 0.0F, 0.0F};
    const std::vector<PeerCollisionObservation> peers{
        {2, {5.0F, 4.0F, 0.0F}, true}};

    const auto decision = monitor.evaluate_path(current, target, peers);

    EXPECT_TRUE(decision.is_safe);
    EXPECT_FALSE(decision.has_conflict);
    EXPECT_NEAR(decision.minimum_distance_m, 4.0F, 1e-6F);
}

TEST(CollisionSafetyMonitorTest, BlocksWhenKnownPeerObservationIsInvalid)
{
    CollisionSafetyMonitor monitor(3.0F);
    const std::array<float, 3> current{0.0F, 0.0F, 0.0F};
    const std::array<float, 3> target{10.0F, 0.0F, 0.0F};
    const std::vector<PeerCollisionObservation> peers{
        {2, {100.0F, 100.0F, 0.0F}, false}};

    const auto decision = monitor.evaluate_path(current, target, peers);

    EXPECT_FALSE(decision.is_safe);
    EXPECT_TRUE(decision.has_conflict);
    EXPECT_EQ(decision.conflicting_agent_index, 2);
}

TEST(CollisionSafetyMonitorTest, BlocksWhenSegmentCrossesPeerSafetyMargin)
{
    CollisionSafetyMonitor monitor(3.0F);
    const std::array<float, 3> current{0.0F, 0.0F, 0.0F};
    const std::array<float, 3> target{10.0F, 0.0F, 0.0F};
    const std::vector<PeerCollisionObservation> peers{
        {4, {5.0F, 1.0F, 0.0F}, true}};

    const auto decision = monitor.evaluate_path(current, target, peers);

    EXPECT_FALSE(decision.is_safe);
    EXPECT_TRUE(decision.has_conflict);
    EXPECT_EQ(decision.conflicting_agent_index, 4);
    EXPECT_NEAR(decision.minimum_distance_m, 1.0F, 1e-6F);
}

TEST(CollisionSafetyMonitorTest, AllowsPathAtExactThreeMeterBoundary)
{
    CollisionSafetyMonitor monitor(3.0F);
    const std::array<float, 3> current{0.0F, 0.0F, 0.0F};
    const std::array<float, 3> target{10.0F, 0.0F, 0.0F};
    const std::vector<PeerCollisionObservation> peers{
        {3, {5.0F, 0.0F, 3.0F}, true}};

    const auto decision = monitor.evaluate_path(current, target, peers);

    EXPECT_TRUE(decision.is_safe);
    EXPECT_NEAR(decision.minimum_distance_m, 3.0F, 1e-6F);
}

TEST(CollisionSafetyMonitorTest, DoesNotAllowConfiguredMarginBelowThreeMeters)
{
    CollisionSafetyMonitor monitor(1.0F);
    const std::array<float, 3> current{0.0F, 0.0F, 0.0F};
    const std::array<float, 3> target{10.0F, 0.0F, 0.0F};
    const std::vector<PeerCollisionObservation> peers{
        {3, {5.0F, 2.0F, 0.0F}, true}};

    const auto decision = monitor.evaluate_path(current, target, peers);

    EXPECT_FALSE(decision.is_safe);
    EXPECT_TRUE(decision.has_conflict);
}

TEST(CollisionSafetyMonitorTest, AllowsFinitePathWhenNoPeersAreKnown)
{
    CollisionSafetyMonitor monitor(3.0F);
    const std::array<float, 3> current{0.0F, 0.0F, 0.0F};
    const std::array<float, 3> target{1.0F, 2.0F, -3.0F};

    const auto decision = monitor.evaluate_path(current, target, {});

    EXPECT_TRUE(decision.is_safe);
    EXPECT_FALSE(decision.has_conflict);
}

TEST(CollisionSafetyMonitorTest, BlocksInvalidTarget)
{
    CollisionSafetyMonitor monitor(3.0F);
    const std::array<float, 3> current{0.0F, 0.0F, 0.0F};
    const std::array<float, 3> target{
        std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F};

    const auto decision = monitor.evaluate_path(current, target, {});

    EXPECT_FALSE(decision.is_safe);
    EXPECT_FALSE(decision.has_conflict);
}

TEST(CollisionTargetGateTest, HoldsUnsafeTargetAndAutomaticallyAppliesAfterRecovery)
{
    CollisionTargetGate gate(3.0F);
    gate.set_requested_target({10.0F, 0.0F, 0.0F, 90.0F});
    const std::array<float, 3> current{0.0F, 0.0F, 0.0F};
    const std::vector<PeerCollisionObservation> blocking_peers{
        {2, {5.0F, 1.0F, 0.0F}, true}};

    const auto blocked = gate.evaluate(current, blocking_peers);
    EXPECT_EQ(blocked.action, CollisionTargetAction::HOLD_POSITION);

    const std::vector<PeerCollisionObservation> safe_peers{
        {2, {5.0F, 4.0F, 0.0F}, true}};
    const auto recovered = gate.evaluate(current, safe_peers);
    EXPECT_EQ(recovered.action, CollisionTargetAction::APPLY_TARGET);
    EXPECT_FLOAT_EQ(recovered.target_ned.at(0), 10.0F);
    EXPECT_FLOAT_EQ(recovered.yaw_deg, 90.0F);

    const auto unchanged = gate.evaluate(current, safe_peers);
    EXPECT_EQ(unchanged.action, CollisionTargetAction::NONE);
}

TEST(CollisionTargetGateTest, HoldsWhenAppliedTargetBecomesUnsafe)
{
    CollisionTargetGate gate(3.0F);
    gate.set_requested_target({10.0F, 0.0F, 0.0F, 0.0F});
    const std::array<float, 3> current{0.0F, 0.0F, 0.0F};
    const std::vector<PeerCollisionObservation> safe_peers{
        {2, {5.0F, 4.0F, 0.0F}, true}};

    EXPECT_EQ(gate.evaluate(current, safe_peers).action,
              CollisionTargetAction::APPLY_TARGET);

    const std::vector<PeerCollisionObservation> approaching_peers{
        {2, {5.0F, 2.0F, 0.0F}, true}};
    EXPECT_EQ(gate.evaluate(current, approaching_peers).action,
              CollisionTargetAction::HOLD_POSITION);
}

TEST(CollisionTargetGateTest, ClearTargetPreventsAutomaticResume)
{
    CollisionTargetGate gate(3.0F);
    gate.set_requested_target({10.0F, 0.0F, 0.0F, 0.0F});
    const std::array<float, 3> current{0.0F, 0.0F, 0.0F};
    const std::vector<PeerCollisionObservation> blocking_peers{
        {2, {5.0F, 1.0F, 0.0F}, true}};

    EXPECT_EQ(gate.evaluate(current, blocking_peers).action,
              CollisionTargetAction::HOLD_POSITION);

    gate.clear_target();
    const std::vector<PeerCollisionObservation> safe_peers{
        {2, {5.0F, 4.0F, 0.0F}, true}};
    EXPECT_EQ(gate.evaluate(current, safe_peers).action,
              CollisionTargetAction::NONE);
}
