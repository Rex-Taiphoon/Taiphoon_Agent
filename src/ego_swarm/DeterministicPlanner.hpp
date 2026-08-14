/**
 * @file DeterministicPlanner.hpp
 * @brief deterministic candidate planner 的 Phase 6 backend 名稱
 * @author Taiphoon Agent Team
 * @date 2026-08-14
 */

#pragma once

#include "ego_swarm/EgoSwarmPlanner.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief 既有 deterministic candidate planner 的可替換 backend adapter。
 *
 * 漸進遷移期間保留 EgoSwarmPlanner 的舊類別名稱與舊 plan signature；新
 * runtime seam 透過這個具名 adapter 組裝 deterministic backend。
 */
class DeterministicPlanner final : public EgoSwarmPlanner
{
public:
    using EgoSwarmPlanner::EgoSwarmPlanner;
};

}  // namespace ego_swarm
}  // namespace taiphoon
