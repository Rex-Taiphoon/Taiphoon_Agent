/**
 * @file PeerClockSkewMonitor.hpp
 * @brief 以同儕軌跡時間戳估計 Agent 主機之間的時鐘差（僅供診斷）
 * @author Taiphoon Agent Team
 * @date 2026-10-20
 *
 * 每筆樣本為「本機收到時間 − 同儕發送時間戳」，等於兩台主機時鐘差加上
 * 網路延遲。延遲只會讓差值變大，因此在時間窗內取最小值可濾除大部分延遲。
 * 本模組只提供估計值，不參與任何規劃或控制決策；設計見 docs/time-sync.md。
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <map>

namespace taiphoon
{
namespace ego_swarm
{

constexpr double DEFAULT_PEER_CLOCK_SKEW_WINDOW_S = 5.0;
constexpr std::size_t DEFAULT_PEER_CLOCK_SKEW_MAX_SAMPLES = 200U;

/**
 * @brief 單一同儕的時鐘差估計
 *
 * `offset_s` 為正代表同儕時鐘比本機慢；包含最小網路延遲。
 * `is_valid == false` 代表時間窗內沒有可用樣本，呼叫端不得使用 offset_s。
 */
struct PeerClockSkewEstimate
{
    bool is_valid{false};
    double offset_s{0.0};
    std::size_t sample_count{0U};
};

/**
 * @brief 判定時鐘差估計是否超過門檻
 *
 * @param estimate 時鐘差估計
 * @param limit_s 允許的時鐘差（秒），必須為非負有限值
 * @return true 表示估計有效且絕對值超過門檻
 */
inline bool peer_clock_skew_exceeds(
    const PeerClockSkewEstimate& estimate,
    const double limit_s)
{
    return estimate.is_valid && std::isfinite(limit_s) && limit_s >= 0.0 &&
        std::isfinite(estimate.offset_s) &&
        std::fabs(estimate.offset_s) > limit_s;
}

/**
 * @brief 以 Agent ID 保存近期時間戳樣本並估計時鐘差
 */
class PeerClockSkewMonitor
{
public:
    /**
     * @brief 建立監測器
     *
     * @param window_s 估計使用的時間窗（秒）；非正或非有限值時使用預設值
     * @param max_samples 每台同儕保留的最多樣本數；0 時使用預設值
     */
    explicit PeerClockSkewMonitor(
        const double window_s = DEFAULT_PEER_CLOCK_SKEW_WINDOW_S,
        const std::size_t max_samples = DEFAULT_PEER_CLOCK_SKEW_MAX_SAMPLES)
        : window_s_(std::isfinite(window_s) && window_s > 0.0
              ? window_s : DEFAULT_PEER_CLOCK_SKEW_WINDOW_S),
          max_samples_(max_samples > 0U
              ? max_samples : DEFAULT_PEER_CLOCK_SKEW_MAX_SAMPLES)
    {
    }

    /**
     * @brief 記錄一筆同儕時間戳樣本
     *
     * 本機時間倒退（新樣本早於既有樣本）時，舊樣本的時間基準已失效，
     * 先清除該同儕的樣本再記錄，避免混用兩個時間基準。
     *
     * @param agent_id 同儕 Agent ID，必須為正
     * @param message_time_s 同儕發送時間戳（秒）
     * @param receive_time_s 本機收到時間（秒）
     * @return 樣本是否被接受
     */
    bool record(
        const int agent_id,
        const double message_time_s,
        const double receive_time_s)
    {
        if (agent_id <= 0 || !std::isfinite(message_time_s) ||
            !std::isfinite(receive_time_s))
        {
            return false;
        }
        const double difference_s = receive_time_s - message_time_s;
        if (!std::isfinite(difference_s))
        {
            return false;
        }

        auto& samples = samples_[agent_id];
        if (!samples.empty() && receive_time_s < samples.back().receive_time_s)
        {
            samples.clear();
        }
        samples.push_back(Sample{receive_time_s, difference_s});
        while (samples.size() > max_samples_ ||
               (!samples.empty() &&
                samples.front().receive_time_s < receive_time_s - window_s_))
        {
            samples.pop_front();
        }
        return true;
    }

    /**
     * @brief 取得指定同儕在時間窗內的時鐘差估計
     *
     * 只使用收到時間落在 [now_s - window_s, now_s] 的樣本；晚於 now_s 的樣本
     * 代表本機時間已倒退，不納入估計。
     *
     * @param agent_id 同儕 Agent ID
     * @param now_s 本機目前時間（秒）
     * @return 估計結果
     */
    PeerClockSkewEstimate estimate(const int agent_id, const double now_s) const
    {
        PeerClockSkewEstimate result;
        const auto iterator = samples_.find(agent_id);
        if (!std::isfinite(now_s) || iterator == samples_.end())
        {
            return result;
        }

        double minimum_difference_s = std::numeric_limits<double>::infinity();
        for (const auto& sample : iterator->second)
        {
            if (sample.receive_time_s < now_s - window_s_ ||
                sample.receive_time_s > now_s)
            {
                continue;
            }
            if (sample.difference_s < minimum_difference_s)
            {
                minimum_difference_s = sample.difference_s;
            }
            ++result.sample_count;
        }
        if (result.sample_count == 0U || !std::isfinite(minimum_difference_s))
        {
            result.sample_count = 0U;
            return result;
        }
        result.is_valid = true;
        result.offset_s = minimum_difference_s;
        return result;
    }

    /**
     * @brief 移除指定同儕的樣本
     *
     * @param agent_id 同儕 Agent ID
     * @return 是否有樣本被移除
     */
    bool erase(const int agent_id)
    {
        return samples_.erase(agent_id) > 0U;
    }

    /**
     * @brief 取得估計使用的時間窗
     *
     * @return 時間窗（秒）
     */
    double window_s() const
    {
        return window_s_;
    }

private:
    struct Sample
    {
        double receive_time_s{0.0};
        double difference_s{0.0};
    };

    double window_s_{DEFAULT_PEER_CLOCK_SKEW_WINDOW_S};
    std::size_t max_samples_{DEFAULT_PEER_CLOCK_SKEW_MAX_SAMPLES};
    std::map<int, std::deque<Sample>> samples_{};
};

}  // namespace ego_swarm
}  // namespace taiphoon
