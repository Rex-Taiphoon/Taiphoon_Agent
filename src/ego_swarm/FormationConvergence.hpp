/**
 * @file FormationConvergence.hpp
 * @brief 編隊收斂條件的純 C++ evaluator
 * @author Taiphoon Agent Team
 * @date 2026-08-14
 */

#pragma once

#include <vector>

#include "ego_swarm/EgoSwarmTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

struct FormationConvergenceObservation
{
    int agent_id{0};
    Vector3 position{0.0, 0.0, 0.0};
    Vector3 target{0.0, 0.0, 0.0};
    // 保留作診斷資料，但不作為 formation convergence 的判斷條件。
    Vector3 velocity{0.0, 0.0, 0.0};
    bool has_fresh_state{false};
    bool has_fresh_occupancy{false};
    bool has_same_epoch{false};
};

struct FormationConvergenceResult
{
    bool is_converged{false};
    std::size_t ready_count{0U};
    std::size_t member_count{0U};
};

/**
 * @brief 由 fresh state、槽位誤差、occupancy 與 epoch identity
 *        推導 formation convergence；不使用速度門檻。
 */
class FormationConvergenceEvaluator
{
public:
    /**
     * @brief 評估所有成員是否已在固定槽位安全收斂。
     * @param observations 每個 member 的 immutable observation
     * @param position_tolerance_m 槽位位置容許誤差
     * @return convergence result
     */
    static FormationConvergenceResult evaluate(
        const std::vector<FormationConvergenceObservation>& observations,
        double position_tolerance_m);
};

}  // namespace ego_swarm
}  // namespace taiphoon
