/**
 * @file YawSlewLimiter.hpp
 * @brief 偏航設定點逐週期限速逼近的純邏輯
 * @author Taiphoon Agent Team
 * @date 2026-10-20
 *
 * PX4 Offboard 位置模式不套用 MPC_YAWRAUTO_MAX：yaw setpoint 一次跳到最終
 * 角度時，PX4 會以 MC_YAWRATE_MAX 全速追趕，偏航飽和時 Airmode 會提高總推力
 * 造成機體瞬間拉高。因此 BasicController 每個控制週期只把送出的 yaw 沿最短
 * 角度推進「角速率上限 × 控制週期」，並以 yawspeed 提供前饋。
 * 此檔不依賴 ROS2，供 gtest 直接驗證。
 */

#pragma once

#include <cmath>
#include <cstdint>

namespace taiphoon
{
namespace controllers
{

/// 圓周率（弧度）
constexpr float YAW_PI = 3.14159265358979323846F;

/// 一整圈（弧度）
constexpr float YAW_TWO_PI = 2.0F * YAW_PI;

/// 度轉弧度係數
constexpr float YAW_DEG_TO_RAD = YAW_PI / 180.0F;

/// 預設偏航角速率上限（度/秒）；低於 PX4 MPC_YAWRAUTO_MAX 預設值以保留推力餘裕
constexpr double DEFAULT_YAW_RATE_LIMIT_DEG_S = 30.0;

/// 偏航角速率上限的最小允許值（度/秒）；不提供關閉限速的設定
constexpr double MIN_YAW_RATE_LIMIT_DEG_S = 1.0;

/// 偏航角速率上限的最大允許值（度/秒）；仍應小於等於 PX4 MC_YAWRATE_MAX
constexpr double MAX_YAW_RATE_LIMIT_DEG_S = 180.0;

/// 視為已到達目標偏航的容差（弧度，約 0.006 度）
constexpr float YAW_ARRIVAL_EPSILON_RAD = 1.0e-4F;

/**
 * @brief 偏航角速率參數正規化結果
 */
enum class YawRateLimitStatus : uint8_t
{
    ACCEPTED = 0U,   ///< 原值合法
    DEFAULTED = 1U,  ///< 非有限值或非正值，改用預設值
    CLAMPED = 2U     ///< 超出範圍，夾到允許邊界
};

/**
 * @brief 單一控制週期的偏航輸出
 */
struct YawSlewStep
{
    float yaw_rad{0.0F};         ///< 本週期送出的 yaw（弧度，[-π, π)）
    float yawspeed_rad_s{0.0F};  ///< 前饋角速度（弧度/秒）；收斂週期為 0
    bool is_converged{true};     ///< 本週期送出後是否已到達目標
    bool is_valid{false};        ///< 輸入是否有效
};

/**
 * @brief 將偏航角正規化到 [-π, π)
 * @param yaw_rad 任意偏航角（弧度）
 * @return 正規化後的偏航角；輸入非有限值時回傳 NaN
 */
inline float wrap_yaw_rad(const float yaw_rad)
{
    float wrapped = std::fmod(yaw_rad + YAW_PI, YAW_TWO_PI);
    if (wrapped < 0.0F)
    {
        wrapped += YAW_TWO_PI;
    }
    wrapped -= YAW_PI;
    // 負小數加 2π 後可能因 float 捨入剛好等於 π，需折回區間下界。
    if (wrapped >= YAW_PI)
    {
        wrapped -= YAW_TWO_PI;
    }
    return wrapped;
}

/**
 * @brief 計算由 from 轉到 to 的最短角度差
 *
 * 範圍為 (-π, π]；正好相差 180 度時取 +π（順時針），與 YAW 命令的正方向一致。
 *
 * @param from_rad 起始偏航角（弧度）
 * @param to_rad 目標偏航角（弧度）
 * @return 最短角度差（弧度，順時針為正）
 */
inline float shortest_yaw_delta_rad(const float from_rad, const float to_rad)
{
    float delta = std::fmod(to_rad - from_rad, YAW_TWO_PI);
    if (delta > YAW_PI)
    {
        delta -= YAW_TWO_PI;
    }
    else if (delta <= -YAW_PI)
    {
        delta += YAW_TWO_PI;
    }
    return delta;
}

/**
 * @brief 將 ROS 參數提供的偏航角速率上限正規化到允許範圍
 * @param requested_deg_s 參數值（度/秒）
 * @param normalized_deg_s 輸出可使用的角速率上限（度/秒）
 * @return 正規化結果，非 ACCEPTED 時呼叫端應記錄警告
 */
inline YawRateLimitStatus normalize_yaw_rate_limit_deg_s(
    const double requested_deg_s, double& normalized_deg_s)
{
    if (!std::isfinite(requested_deg_s) || requested_deg_s <= 0.0)
    {
        normalized_deg_s = DEFAULT_YAW_RATE_LIMIT_DEG_S;
        return YawRateLimitStatus::DEFAULTED;
    }
    if (requested_deg_s < MIN_YAW_RATE_LIMIT_DEG_S)
    {
        normalized_deg_s = MIN_YAW_RATE_LIMIT_DEG_S;
        return YawRateLimitStatus::CLAMPED;
    }
    if (requested_deg_s > MAX_YAW_RATE_LIMIT_DEG_S)
    {
        normalized_deg_s = MAX_YAW_RATE_LIMIT_DEG_S;
        return YawRateLimitStatus::CLAMPED;
    }
    normalized_deg_s = requested_deg_s;
    return YawRateLimitStatus::ACCEPTED;
}

/**
 * @brief 判斷送出偏航是否仍未到達目標
 * @param commanded_rad 目前送出的偏航角（弧度）
 * @param target_rad 目標偏航角（弧度）
 * @return true 兩者皆為有限值且最短角度差大於到達容差
 */
inline bool is_yaw_short_of_target(const float commanded_rad, const float target_rad)
{
    if (!std::isfinite(commanded_rad) || !std::isfinite(target_rad))
    {
        return false;
    }
    return std::abs(shortest_yaw_delta_rad(commanded_rad, target_rad)) >
        YAW_ARRIVAL_EPSILON_RAD;
}

/**
 * @brief 將送出偏航沿最短角度朝目標推進一個控制週期
 *
 * 剩餘角度不超過一步時直接送出目標並停止前饋，避免越過目標後反向修正。
 *
 * @param commanded_rad 上一週期送出的偏航角（弧度）
 * @param target_rad 目標偏航角（弧度）
 * @param max_rate_rad_s 角速率上限（弧度/秒），必須大於 0
 * @param period_s 控制週期（秒），必須大於 0
 * @return 本週期的偏航輸出；任一輸入非法時 is_valid 為 false
 */
inline YawSlewStep step_yaw_toward(
    const float commanded_rad, const float target_rad,
    const float max_rate_rad_s, const float period_s)
{
    YawSlewStep step;
    if (!std::isfinite(commanded_rad) || !std::isfinite(target_rad) ||
        !std::isfinite(max_rate_rad_s) || max_rate_rad_s <= 0.0F ||
        !std::isfinite(period_s) || period_s <= 0.0F)
    {
        return step;
    }

    const float delta_rad = shortest_yaw_delta_rad(commanded_rad, target_rad);
    const float max_step_rad = max_rate_rad_s * period_s;
    step.is_valid = true;
    // 容差需涵蓋 float 累加誤差：若剩餘角度只比一步多出容差內的量，
    // 推進後會落在目標容差內卻仍帶前饋，且之後不再重送而留下非零 yawspeed。
    if (std::abs(delta_rad) <= max_step_rad + YAW_ARRIVAL_EPSILON_RAD)
    {
        step.yaw_rad = wrap_yaw_rad(target_rad);
        step.yawspeed_rad_s = 0.0F;
        step.is_converged = true;
        return step;
    }

    const float direction = delta_rad > 0.0F ? 1.0F : -1.0F;
    step.yaw_rad = wrap_yaw_rad(commanded_rad + direction * max_step_rad);
    step.yawspeed_rad_s = direction * max_rate_rad_s;
    step.is_converged = false;
    return step;
}

}  // namespace controllers
}  // namespace taiphoon
