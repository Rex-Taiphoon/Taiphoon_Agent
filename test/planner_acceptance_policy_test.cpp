/**
 * @file planner_acceptance_policy_test.cpp
 * @brief planner candidate acceptance chain 的純 C++ 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 */

#include <gtest/gtest.h>

#include "ego_swarm/PlannerAcceptancePolicy.hpp"

namespace
{

using taiphoon::ego_swarm::CandidateAcceptanceInput;
using taiphoon::ego_swarm::CandidateAcceptanceReason;
using taiphoon::ego_swarm::evaluate_candidate_acceptance;

TEST(PlannerAcceptancePolicyTest, AcceptsOnlyFullyValidatedCandidate)
{
    const CandidateAcceptanceInput input{
        true,   // planner succeeded and trajectory is valid
        true,   // planner output is finite
        true,   // reservation required
        true,   // reservation accepted
        true};  // continuous safety validation passed

    const auto decision = evaluate_candidate_acceptance(input);

    EXPECT_TRUE(decision.should_accept);
    EXPECT_EQ(decision.reason, CandidateAcceptanceReason::ACCEPTED);
}

TEST(PlannerAcceptancePolicyTest, RejectsPlannerFailureBeforeOtherStages)
{
    const CandidateAcceptanceInput input{
        false, true, true, true, true};

    const auto decision = evaluate_candidate_acceptance(input);

    EXPECT_FALSE(decision.should_accept);
    EXPECT_EQ(decision.reason, CandidateAcceptanceReason::PLANNER_FAILED);
}

TEST(PlannerAcceptancePolicyTest, RejectsNonFinitePlannerOutput)
{
    const CandidateAcceptanceInput input{
        true, false, true, true, true};

    const auto decision = evaluate_candidate_acceptance(input);

    EXPECT_FALSE(decision.should_accept);
    EXPECT_EQ(
        decision.reason, CandidateAcceptanceReason::PLANNER_OUTPUT_NONFINITE);
}

TEST(PlannerAcceptancePolicyTest, RejectsReservationFailure)
{
    const CandidateAcceptanceInput input{
        true, true, true, false, true};

    const auto decision = evaluate_candidate_acceptance(input);

    EXPECT_FALSE(decision.should_accept);
    EXPECT_EQ(
        decision.reason, CandidateAcceptanceReason::RESERVATION_REJECTED);
}

TEST(PlannerAcceptancePolicyTest, RejectsSafetyFailure)
{
    const CandidateAcceptanceInput input{
        true, true, true, true, false};

    const auto decision = evaluate_candidate_acceptance(input);

    EXPECT_FALSE(decision.should_accept);
    EXPECT_EQ(decision.reason, CandidateAcceptanceReason::SAFETY_REJECTED);
}

}  // namespace
