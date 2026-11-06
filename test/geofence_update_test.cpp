/**
 * @file geofence_update_test.cpp
 * @brief 地面站執行期設定地理圍籬的更新規則測試
 * @author Taiphoon Agent Team
 * @date 2026-11-06
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "ego_swarm/GeofenceUpdate.hpp"

namespace
{

using taiphoon::ego_swarm::GeoCoordinate;
using taiphoon::ego_swarm::Geofence;
using taiphoon::ego_swarm::GeofenceAction;
using taiphoon::ego_swarm::GeofenceRequest;
using taiphoon::ego_swarm::GeofenceState;
using taiphoon::ego_swarm::GeofenceUpdatePolicy;
using taiphoon::ego_swarm::GeofenceUpdateResult;
using taiphoon::ego_swarm::LocalNedFrame;
using taiphoon::ego_swarm::Vector3;
using taiphoon::ego_swarm::first_target_outside_geofence;
using taiphoon::ego_swarm::is_geofence_request_valid;
using taiphoon::ego_swarm::is_target_outside_geofence;

/// 實際場域中心（WGS84）。
constexpr double FIELD_CENTER_LAT_DEG = 24.881051576424703;
constexpr double FIELD_CENTER_LON_DEG = 121.0696306771756;
/// 測試用 margin（m）。
constexpr double FIELD_MARGIN_M = 0.5;

LocalNedFrame make_home_frame()
{
    LocalNedFrame frame;
    EXPECT_TRUE(frame.set_origin(
        GeoCoordinate{FIELD_CENTER_LAT_DEG, FIELD_CENTER_LON_DEG, 50.0}));
    return frame;
}

/// 以 Home 為中心、半寬 half_m 的正方形 SET 命令。
GeofenceRequest make_square_request(
    const LocalNedFrame& frame,
    const uint32_t fence_id,
    const double half_m,
    const double margin_m = FIELD_MARGIN_M)
{
    GeofenceRequest request;
    request.fence_id = fence_id;
    request.action = static_cast<uint8_t>(GeofenceAction::SET);
    request.margin_m = margin_m;
    for (const Vector3& corner :
         {Vector3{-half_m, -half_m, 0.0}, Vector3{half_m, -half_m, 0.0},
          Vector3{half_m, half_m, 0.0}, Vector3{-half_m, half_m, 0.0}})
    {
        GeoCoordinate vertex{};
        EXPECT_TRUE(frame.from_local_ned(corner, vertex));
        request.vertices.push_back(vertex);
    }
    return request;
}

GeofenceRequest make_clear_request(const uint32_t fence_id)
{
    GeofenceRequest request;
    request.fence_id = fence_id;
    request.action = static_cast<uint8_t>(GeofenceAction::CLEAR);
    return request;
}

// --- 命令檢查 ---

TEST(GeofenceRequestTest, AcceptsValidSetAndClear)
{
    const LocalNedFrame frame = make_home_frame();
    std::string detail;
    EXPECT_TRUE(is_geofence_request_valid(make_square_request(frame, 1U, 4.0), detail));
    EXPECT_TRUE(is_geofence_request_valid(make_clear_request(2U), detail));
}

TEST(GeofenceRequestTest, RejectsInvalidFields)
{
    const LocalNedFrame frame = make_home_frame();
    std::string detail;

    GeofenceRequest request = make_square_request(frame, 0U, 4.0);
    EXPECT_FALSE(is_geofence_request_valid(request, detail));
    EXPECT_FALSE(is_geofence_request_valid(make_clear_request(0U), detail));

    request = make_square_request(frame, 1U, 4.0);
    request.action = 3U;
    EXPECT_FALSE(is_geofence_request_valid(request, detail));

    for (const double margin_m :
         {-0.1, 10.1, std::numeric_limits<double>::quiet_NaN()})
    {
        request = make_square_request(frame, 1U, 4.0, margin_m);
        EXPECT_FALSE(is_geofence_request_valid(request, detail)) << margin_m;
    }
    request = make_square_request(frame, 1U, 4.0, 10.0);
    EXPECT_TRUE(is_geofence_request_valid(request, detail));

    request = make_square_request(frame, 1U, 4.0);
    request.vertices.resize(2U);
    EXPECT_FALSE(is_geofence_request_valid(request, detail));
    request = make_square_request(frame, 1U, 4.0);
    request.vertices.resize(65U, request.vertices.front());
    EXPECT_FALSE(is_geofence_request_valid(request, detail));

    request = make_square_request(frame, 1U, 4.0);
    request.vertices.at(1).latitude_deg = 91.0;
    EXPECT_FALSE(is_geofence_request_valid(request, detail));
    request = make_square_request(frame, 1U, 4.0);
    request.vertices.at(2).longitude_deg = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(is_geofence_request_valid(request, detail));
}

// --- 套用與保留 ---

TEST(GeofenceUpdatePolicyTest, StartsDisabledWithoutFence)
{
    const GeofenceUpdatePolicy policy;
    EXPECT_EQ(policy.state(), GeofenceState::DISABLED);
    EXPECT_EQ(policy.active_fence(), nullptr);
    EXPECT_EQ(policy.last_request_id(), 0U);
    EXPECT_EQ(policy.last_result(), GeofenceUpdateResult::NONE);
}

TEST(GeofenceUpdatePolicyTest, SetActivatesWhenHomeReady)
{
    const LocalNedFrame frame = make_home_frame();
    GeofenceUpdatePolicy policy;
    const auto outcome = policy.handle_request(
        make_square_request(frame, 7U, 4.0), &frame);
    EXPECT_TRUE(outcome.is_fence_changed);
    EXPECT_TRUE(outcome.is_status_changed);
    EXPECT_EQ(outcome.result, GeofenceUpdateResult::ACCEPTED);
    EXPECT_EQ(policy.state(), GeofenceState::ACTIVE);
    ASSERT_NE(policy.active_fence(), nullptr);
    EXPECT_NEAR(policy.active_fence()->clearance_m(Vector3{0.0, 0.0, 0.0}), 4.0, 1.0e-3);
    EXPECT_DOUBLE_EQ(policy.active_fence()->margin_m(), FIELD_MARGIN_M);
    EXPECT_EQ(policy.definition().fence_id, 7U);
    EXPECT_EQ(policy.definition().vertices.size(), 4U);
    EXPECT_EQ(policy.last_request_id(), 7U);
}

TEST(GeofenceUpdatePolicyTest, RepeatedSameContentDoesNotReapply)
{
    const LocalNedFrame frame = make_home_frame();
    GeofenceUpdatePolicy policy;
    const GeofenceRequest request = make_square_request(frame, 7U, 4.0);
    ASSERT_TRUE(policy.handle_request(request, &frame).is_fence_changed);

    const auto repeated = policy.handle_request(request, &frame);
    EXPECT_FALSE(repeated.is_fence_changed);
    EXPECT_FALSE(repeated.is_status_changed);
    EXPECT_EQ(repeated.result, GeofenceUpdateResult::ACCEPTED);
    EXPECT_EQ(policy.state(), GeofenceState::ACTIVE);
}

TEST(GeofenceUpdatePolicyTest, NewVersionReplacesFence)
{
    const LocalNedFrame frame = make_home_frame();
    GeofenceUpdatePolicy policy;
    ASSERT_TRUE(policy.handle_request(
        make_square_request(frame, 1U, 4.0), &frame).is_fence_changed);
    const auto outcome = policy.handle_request(
        make_square_request(frame, 2U, 2.0), &frame);
    EXPECT_TRUE(outcome.is_fence_changed);
    ASSERT_NE(policy.active_fence(), nullptr);
    EXPECT_NEAR(policy.active_fence()->clearance_m(Vector3{0.0, 0.0, 0.0}), 2.0, 1.0e-3);
    EXPECT_EQ(policy.definition().fence_id, 2U);

    // 同一版本號但內容不同時仍以新內容替換。
    EXPECT_TRUE(policy.handle_request(
        make_square_request(frame, 2U, 3.0), &frame).is_fence_changed);
    EXPECT_NEAR(policy.active_fence()->clearance_m(Vector3{0.0, 0.0, 0.0}), 3.0, 1.0e-3);
}

TEST(GeofenceUpdatePolicyTest, RejectedRequestKeepsPreviousFence)
{
    const LocalNedFrame frame = make_home_frame();
    GeofenceUpdatePolicy policy;
    ASSERT_TRUE(policy.handle_request(
        make_square_request(frame, 1U, 4.0), &frame).is_fence_changed);

    GeofenceRequest bow_tie = make_square_request(frame, 2U, 4.0);
    std::swap(bow_tie.vertices.at(1), bow_tie.vertices.at(2));
    const auto outcome = policy.handle_request(bow_tie, &frame);
    EXPECT_FALSE(outcome.is_fence_changed);
    EXPECT_TRUE(outcome.is_status_changed);
    EXPECT_EQ(outcome.result, GeofenceUpdateResult::REJECTED_INVALID);
    EXPECT_EQ(policy.state(), GeofenceState::ACTIVE);
    EXPECT_EQ(policy.definition().fence_id, 1U);
    EXPECT_EQ(policy.last_request_id(), 2U);
    EXPECT_EQ(policy.last_result(), GeofenceUpdateResult::REJECTED_INVALID);
    ASSERT_NE(policy.active_fence(), nullptr);
    EXPECT_NEAR(policy.active_fence()->clearance_m(Vector3{0.0, 0.0, 0.0}), 4.0, 1.0e-3);

    GeofenceRequest bad_margin = make_square_request(frame, 3U, 4.0, -1.0);
    EXPECT_EQ(policy.handle_request(bad_margin, &frame).result,
              GeofenceUpdateResult::REJECTED_INVALID);
    EXPECT_EQ(policy.definition().fence_id, 1U);
}

TEST(GeofenceUpdatePolicyTest, FarVertexReportsTooFar)
{
    const LocalNedFrame frame = make_home_frame();
    GeofenceUpdatePolicy policy;
    GeofenceRequest request = make_square_request(frame, 4U, 4.0);
    request.vertices.at(2).latitude_deg += 0.1;  // 約 11 km
    const auto outcome = policy.handle_request(request, &frame);
    EXPECT_EQ(outcome.result, GeofenceUpdateResult::REJECTED_TOO_FAR);
    EXPECT_FALSE(outcome.is_fence_changed);
    EXPECT_EQ(policy.state(), GeofenceState::DISABLED);
}

TEST(GeofenceUpdatePolicyTest, ClearDisablesFence)
{
    const LocalNedFrame frame = make_home_frame();
    GeofenceUpdatePolicy policy;
    ASSERT_TRUE(policy.handle_request(
        make_square_request(frame, 1U, 4.0), &frame).is_fence_changed);
    const auto outcome = policy.handle_request(make_clear_request(2U), &frame);
    EXPECT_TRUE(outcome.is_fence_changed);
    EXPECT_TRUE(outcome.is_status_changed);
    EXPECT_EQ(policy.state(), GeofenceState::DISABLED);
    EXPECT_EQ(policy.active_fence(), nullptr);
    EXPECT_EQ(policy.definition().fence_id, 0U);
    EXPECT_TRUE(policy.definition().vertices.empty());
    EXPECT_EQ(policy.last_request_id(), 2U);

    // 已停用時重送 CLEAR 不改變圍籬。
    const auto repeated = policy.handle_request(make_clear_request(2U), &frame);
    EXPECT_FALSE(repeated.is_fence_changed);
    EXPECT_FALSE(repeated.is_status_changed);
}

// --- 等待 Home ---

TEST(GeofenceUpdatePolicyTest, PendingUntilHomeReady)
{
    const LocalNedFrame frame = make_home_frame();
    GeofenceUpdatePolicy policy;
    const auto outcome = policy.handle_request(
        make_square_request(frame, 5U, 4.0), nullptr);
    EXPECT_FALSE(outcome.is_fence_changed);
    EXPECT_TRUE(outcome.is_status_changed);
    EXPECT_EQ(policy.state(), GeofenceState::PENDING);
    EXPECT_EQ(policy.active_fence(), nullptr);
    EXPECT_EQ(policy.definition().fence_id, 5U);

    const auto ready = policy.on_home_ready(frame);
    EXPECT_TRUE(ready.is_fence_changed);
    EXPECT_EQ(policy.state(), GeofenceState::ACTIVE);
    ASSERT_NE(policy.active_fence(), nullptr);

    // 之後再次凍結 Home 不影響已生效的圍籬。
    EXPECT_FALSE(policy.on_home_ready(frame).is_fence_changed);
}

TEST(GeofenceUpdatePolicyTest, PendingRejectedAtHomeFallsBackToDisabled)
{
    const LocalNedFrame frame = make_home_frame();
    GeofenceUpdatePolicy policy;
    GeofenceRequest request = make_square_request(frame, 6U, 4.0);
    request.vertices.at(0).latitude_deg -= 0.1;
    ASSERT_EQ(policy.handle_request(request, nullptr).result,
              GeofenceUpdateResult::ACCEPTED);
    EXPECT_EQ(policy.state(), GeofenceState::PENDING);

    const auto ready = policy.on_home_ready(frame);
    EXPECT_FALSE(ready.is_fence_changed);
    EXPECT_TRUE(ready.is_status_changed);
    EXPECT_EQ(ready.result, GeofenceUpdateResult::REJECTED_TOO_FAR);
    EXPECT_EQ(policy.state(), GeofenceState::DISABLED);
    EXPECT_EQ(policy.active_fence(), nullptr);
}

TEST(GeofenceUpdatePolicyTest, ClearWhilePendingDoesNotRequestRecheck)
{
    const LocalNedFrame frame = make_home_frame();
    GeofenceUpdatePolicy policy;
    ASSERT_EQ(policy.handle_request(make_square_request(frame, 1U, 4.0), nullptr).result,
              GeofenceUpdateResult::ACCEPTED);
    const auto outcome = policy.handle_request(make_clear_request(2U), nullptr);
    EXPECT_FALSE(outcome.is_fence_changed);
    EXPECT_TRUE(outcome.is_status_changed);
    EXPECT_EQ(policy.state(), GeofenceState::DISABLED);
    EXPECT_FALSE(policy.on_home_ready(frame).is_fence_changed);
}

// --- 更新後的目標判斷 ---

TEST(GeofenceTargetCheckTest, DetectsTargetsOutsideNewFence)
{
    Geofence fence;
    ASSERT_TRUE(fence.set_polygon(
        {Vector3{-2.0, -2.0, 0.0}, Vector3{2.0, -2.0, 0.0},
         Vector3{2.0, 2.0, 0.0}, Vector3{-2.0, 2.0, 0.0}},
        FIELD_MARGIN_M));

    EXPECT_FALSE(is_target_outside_geofence(nullptr, Vector3{100.0, 0.0, -2.0}));
    EXPECT_FALSE(is_target_outside_geofence(&fence, Vector3{1.0, 1.0, -2.0}));
    // margin 帶內也視為越界。
    EXPECT_TRUE(is_target_outside_geofence(&fence, Vector3{1.8, 0.0, -2.0}));
    EXPECT_TRUE(is_target_outside_geofence(&fence, Vector3{3.0, 0.0, -2.0}));

    const std::vector<std::pair<std::size_t, Vector3>> targets{
        {2U, Vector3{0.0, 0.0, -2.0}},
        {3U, Vector3{3.0, 0.0, -2.0}},
        {4U, Vector3{5.0, 0.0, -2.0}}};
    const auto outside = first_target_outside_geofence(&fence, targets);
    ASSERT_TRUE(outside.has_value());
    EXPECT_EQ(*outside, 3U);
    EXPECT_FALSE(first_target_outside_geofence(nullptr, targets).has_value());
    EXPECT_FALSE(first_target_outside_geofence(
        &fence, {{0U, Vector3{0.0, 0.0, -2.0}}}).has_value());
}

}  // namespace
