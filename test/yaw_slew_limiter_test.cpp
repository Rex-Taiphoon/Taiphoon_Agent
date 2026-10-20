/**
 * @file yaw_slew_limiter_test.cpp
 * @brief 偏航限速逼近純邏輯測試：角度正規化、最短路徑、跨 ±180 度、收斂與參數防呆
 * @author Taiphoon Agent Team
 * @date 2026-10-20
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "controllers/YawSlewLimiter.hpp"

namespace
{

using taiphoon::controllers::DEFAULT_YAW_RATE_LIMIT_DEG_S;
using taiphoon::controllers::MAX_YAW_RATE_LIMIT_DEG_S;
using taiphoon::controllers::MIN_YAW_RATE_LIMIT_DEG_S;
using taiphoon::controllers::YAW_DEG_TO_RAD;
using taiphoon::controllers::YAW_PI;
using taiphoon::controllers::YawRateLimitStatus;
using taiphoon::controllers::YawSlewStep;
using taiphoon::controllers::is_yaw_short_of_target;
using taiphoon::controllers::normalize_yaw_rate_limit_deg_s;
using taiphoon::controllers::shortest_yaw_delta_rad;
using taiphoon::controllers::step_yaw_toward;
using taiphoon::controllers::wrap_yaw_rad;

constexpr float ANGLE_EPSILON_RAD = 1.0e-5F;
constexpr float CONTROL_PERIOD_S = 0.1F;
constexpr float RATE_30_DEG_S_RAD = 30.0F * YAW_DEG_TO_RAD;

/** @brief 角度轉弧度，簡化測試資料。 */
float deg(const float degrees)
{
    return degrees * YAW_DEG_TO_RAD;
}

TEST(YawSlewLimiterTest, WrapKeepsAnglesInHalfOpenRange)
{
    EXPECT_NEAR(wrap_yaw_rad(0.0F), 0.0F, ANGLE_EPSILON_RAD);
    EXPECT_NEAR(wrap_yaw_rad(YAW_PI), -YAW_PI, ANGLE_EPSILON_RAD);
    EXPECT_NEAR(wrap_yaw_rad(deg(270.0F)), deg(-90.0F), ANGLE_EPSILON_RAD);
    EXPECT_NEAR(wrap_yaw_rad(deg(-270.0F)), deg(90.0F), ANGLE_EPSILON_RAD);
    EXPECT_NEAR(wrap_yaw_rad(deg(725.0F)), deg(5.0F), 1.0e-4F);
    for (const float angle : {-1.0e-9F, -YAW_PI, YAW_PI, 7.0F * YAW_PI})
    {
        const float wrapped = wrap_yaw_rad(angle);
        EXPECT_GE(wrapped, -YAW_PI);
        EXPECT_LT(wrapped, YAW_PI);
    }
    EXPECT_TRUE(std::isnan(wrap_yaw_rad(std::nanf(""))));
}

TEST(YawSlewLimiterTest, ShortestDeltaCrossesPlusMinus180)
{
    EXPECT_NEAR(shortest_yaw_delta_rad(deg(170.0F), deg(-170.0F)),
                deg(20.0F), ANGLE_EPSILON_RAD);
    EXPECT_NEAR(shortest_yaw_delta_rad(deg(-170.0F), deg(170.0F)),
                deg(-20.0F), ANGLE_EPSILON_RAD);
    EXPECT_NEAR(shortest_yaw_delta_rad(deg(10.0F), deg(280.0F)),
                deg(-90.0F), 1.0e-4F);
    // 正好 180 度取順時針，與 YAW 命令正方向一致。
    EXPECT_NEAR(shortest_yaw_delta_rad(0.0F, YAW_PI), YAW_PI, ANGLE_EPSILON_RAD);
    EXPECT_NEAR(shortest_yaw_delta_rad(0.0F, -YAW_PI), YAW_PI, ANGLE_EPSILON_RAD);
}

TEST(YawSlewLimiterTest, LargeTurnAdvancesOneLimitedStepWithFeedForward)
{
    const YawSlewStep step = step_yaw_toward(
        deg(30.0F), deg(120.0F), RATE_30_DEG_S_RAD, CONTROL_PERIOD_S);

    ASSERT_TRUE(step.is_valid);
    EXPECT_FALSE(step.is_converged);
    EXPECT_NEAR(step.yaw_rad, deg(33.0F), ANGLE_EPSILON_RAD);
    EXPECT_NEAR(step.yawspeed_rad_s, RATE_30_DEG_S_RAD, ANGLE_EPSILON_RAD);
}

TEST(YawSlewLimiterTest, CounterClockwiseTurnUsesNegativeFeedForward)
{
    const YawSlewStep step = step_yaw_toward(
        0.0F, deg(-160.0F), RATE_30_DEG_S_RAD, CONTROL_PERIOD_S);

    ASSERT_TRUE(step.is_valid);
    EXPECT_NEAR(step.yaw_rad, deg(-3.0F), ANGLE_EPSILON_RAD);
    EXPECT_NEAR(step.yawspeed_rad_s, -RATE_30_DEG_S_RAD, ANGLE_EPSILON_RAD);
}

TEST(YawSlewLimiterTest, RemainingAngleWithinOneStepLandsExactlyWithoutFeedForward)
{
    const YawSlewStep step = step_yaw_toward(
        deg(118.5F), deg(120.0F), RATE_30_DEG_S_RAD, CONTROL_PERIOD_S);

    ASSERT_TRUE(step.is_valid);
    EXPECT_TRUE(step.is_converged);
    EXPECT_NEAR(step.yaw_rad, deg(120.0F), ANGLE_EPSILON_RAD);
    EXPECT_FLOAT_EQ(step.yawspeed_rad_s, 0.0F);
}

TEST(YawSlewLimiterTest, StepAcrossBoundaryWrapsToNegativeSide)
{
    const YawSlewStep step = step_yaw_toward(
        deg(179.0F), deg(-170.0F), RATE_30_DEG_S_RAD, CONTROL_PERIOD_S);

    ASSERT_TRUE(step.is_valid);
    EXPECT_NEAR(step.yaw_rad, deg(-178.0F), 1.0e-4F);
    EXPECT_GT(step.yawspeed_rad_s, 0.0F);
}

TEST(YawSlewLimiterTest, NinetyDegreesAtThirtyDegPerSecondTakesThirtyCyclesWithoutOvershoot)
{
    float commanded = deg(-60.0F);
    const float target = deg(30.0F);
    int cycles = 0;
    float previous_remaining = std::abs(shortest_yaw_delta_rad(commanded, target));
    bool is_converged = false;
    while (!is_converged && cycles < 100)
    {
        const YawSlewStep step = step_yaw_toward(
            commanded, target, RATE_30_DEG_S_RAD, CONTROL_PERIOD_S);
        ASSERT_TRUE(step.is_valid);
        EXPECT_LE(std::abs(shortest_yaw_delta_rad(commanded, step.yaw_rad)),
                  deg(3.0F) + ANGLE_EPSILON_RAD);
        const float remaining = std::abs(shortest_yaw_delta_rad(step.yaw_rad, target));
        EXPECT_LE(remaining, previous_remaining);
        previous_remaining = remaining;
        commanded = step.yaw_rad;
        is_converged = step.is_converged;
        ++cycles;
    }

    EXPECT_TRUE(is_converged);
    EXPECT_EQ(cycles, 30);
    EXPECT_NEAR(commanded, target, ANGLE_EPSILON_RAD);
    EXPECT_FALSE(is_yaw_short_of_target(commanded, target));
}

TEST(YawSlewLimiterTest, EveryNonConvergedStepLeavesMoreThanArrivalTolerance)
{
    // 回歸：float 累加讓剩餘角度略大於一步時，最後一步必須標記收斂並把
    // 前饋歸零；否則控制器停止重送後 PX4 會保留非零 yawspeed。
    for (const float start_deg : {30.0F, -60.0F, 170.0F, -179.0F})
    {
        for (const float turn_deg : {3.0F, 45.0F, 90.0F, 135.0F, -90.0F})
        {
            float commanded = deg(start_deg);
            const float target = wrap_yaw_rad(deg(start_deg + turn_deg));
            bool is_converged = false;
            for (int cycle = 0; cycle < 100 && !is_converged; ++cycle)
            {
                const YawSlewStep step = step_yaw_toward(
                    commanded, target, RATE_30_DEG_S_RAD, CONTROL_PERIOD_S);
                ASSERT_TRUE(step.is_valid);
                commanded = step.yaw_rad;
                is_converged = step.is_converged;
                if (!is_converged)
                {
                    EXPECT_TRUE(is_yaw_short_of_target(commanded, target))
                        << "start=" << start_deg << " turn=" << turn_deg;
                }
                else
                {
                    EXPECT_FLOAT_EQ(step.yawspeed_rad_s, 0.0F);
                }
            }
            EXPECT_TRUE(is_converged);
        }
    }
}

TEST(YawSlewLimiterTest, InvalidInputsAreRejected)
{
    const float nan_value = std::nanf("");
    const float inf_value = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(step_yaw_toward(nan_value, 0.0F, RATE_30_DEG_S_RAD,
                                 CONTROL_PERIOD_S).is_valid);
    EXPECT_FALSE(step_yaw_toward(0.0F, inf_value, RATE_30_DEG_S_RAD,
                                 CONTROL_PERIOD_S).is_valid);
    EXPECT_FALSE(step_yaw_toward(0.0F, 1.0F, 0.0F, CONTROL_PERIOD_S).is_valid);
    EXPECT_FALSE(step_yaw_toward(0.0F, 1.0F, -1.0F, CONTROL_PERIOD_S).is_valid);
    EXPECT_FALSE(step_yaw_toward(0.0F, 1.0F, RATE_30_DEG_S_RAD, 0.0F).is_valid);
    EXPECT_FALSE(is_yaw_short_of_target(nan_value, 1.0F));
    EXPECT_FALSE(is_yaw_short_of_target(1.0F, nan_value));
    EXPECT_TRUE(is_yaw_short_of_target(0.0F, deg(1.0F)));
}

TEST(YawSlewLimiterTest, RateLimitParameterIsNormalized)
{
    double normalized = 0.0;
    EXPECT_EQ(normalize_yaw_rate_limit_deg_s(45.0, normalized),
              YawRateLimitStatus::ACCEPTED);
    EXPECT_DOUBLE_EQ(normalized, 45.0);

    EXPECT_EQ(normalize_yaw_rate_limit_deg_s(std::nan(""), normalized),
              YawRateLimitStatus::DEFAULTED);
    EXPECT_DOUBLE_EQ(normalized, DEFAULT_YAW_RATE_LIMIT_DEG_S);

    EXPECT_EQ(normalize_yaw_rate_limit_deg_s(0.0, normalized),
              YawRateLimitStatus::DEFAULTED);
    EXPECT_DOUBLE_EQ(normalized, DEFAULT_YAW_RATE_LIMIT_DEG_S);

    EXPECT_EQ(normalize_yaw_rate_limit_deg_s(-10.0, normalized),
              YawRateLimitStatus::DEFAULTED);
    EXPECT_DOUBLE_EQ(normalized, DEFAULT_YAW_RATE_LIMIT_DEG_S);

    EXPECT_EQ(normalize_yaw_rate_limit_deg_s(0.5, normalized),
              YawRateLimitStatus::CLAMPED);
    EXPECT_DOUBLE_EQ(normalized, MIN_YAW_RATE_LIMIT_DEG_S);

    EXPECT_EQ(normalize_yaw_rate_limit_deg_s(500.0, normalized),
              YawRateLimitStatus::CLAMPED);
    EXPECT_DOUBLE_EQ(normalized, MAX_YAW_RATE_LIMIT_DEG_S);
}

TEST(YawSlewLimiterTest, DefaultRateStaysWithinRecommendedBand)
{
    // 實機回報 YAW 過急造成拉高；預設值需落在 30～45 deg/s。
    EXPECT_GE(DEFAULT_YAW_RATE_LIMIT_DEG_S, 30.0);
    EXPECT_LE(DEFAULT_YAW_RATE_LIMIT_DEG_S, 45.0);
}

}  // namespace
