/**
 * @file peer_matcher_test.cpp
 * @brief PeerMatcher 單元測試（gtest）
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 測試 PeerMatcher 的 node/topic 判斷、agent 索引擷取與同儕過濾邏輯。
 * 需在 colcon 環境中編譯執行：colcon test --packages-select taiphoon_agent
 */

#include <gtest/gtest.h>

#include <set>
#include <string>
#include <vector>

#include "formation/PeerMatcher.hpp"

using taiphoon::formation::PeerMatcher;

// ==================== is_agent_state_topic 測試 ====================

TEST(PeerMatcherTest, IsAgentStateTopicMatch)
{
    EXPECT_TRUE(PeerMatcher::is_agent_state_topic("/fmu1_agent/state"));
    EXPECT_TRUE(PeerMatcher::is_agent_state_topic("/fmu2_agent/state"));
    EXPECT_TRUE(PeerMatcher::is_agent_state_topic("/fmu10_agent/state"));
    EXPECT_FALSE(PeerMatcher::is_agent_state_topic("/fmu_all_agent/state"));
}

TEST(PeerMatcherTest, IsAgentStateTopicNoMatch)
{
    EXPECT_FALSE(PeerMatcher::is_agent_state_topic("/fmu1/fmu/out/vehicle_odometry"));
    EXPECT_FALSE(PeerMatcher::is_agent_state_topic("/fmu1_agent/ned_target"));
    EXPECT_FALSE(PeerMatcher::is_agent_state_topic("/fmu1_agent/attitude"));
    EXPECT_FALSE(PeerMatcher::is_agent_state_topic(""));
    EXPECT_FALSE(PeerMatcher::is_agent_state_topic("/fmu1/state"));
}

// ==================== extract_agent_index 測試 ====================

TEST(PeerMatcherTest, ExtractAgentIndex)
{
    EXPECT_EQ(PeerMatcher::extract_agent_index("/fmu1_agent/state"), 1);
    EXPECT_EQ(PeerMatcher::extract_agent_index("/fmu2_agent/state"), 2);
    EXPECT_EQ(PeerMatcher::extract_agent_index("/fmu10_agent/state"), 10);
    EXPECT_EQ(PeerMatcher::extract_agent_index("/fmu0_agent/state"), 0);
}

TEST(PeerMatcherTest, ExtractAgentIndexNegative)
{
    EXPECT_EQ(PeerMatcher::extract_agent_index(""), -1);
    EXPECT_EQ(PeerMatcher::extract_agent_index("/invalid/topic"), -1);
    EXPECT_EQ(PeerMatcher::extract_agent_index("/fmu_agent/state"), -1);
}

// ==================== filter_peers 測試 ====================

TEST(PeerMatcherTest, FilterPeersBasic)
{
    std::vector<std::string> topics = {
        "/fmu1_agent/state",
        "/fmu2_agent/state",
        "/fmu3_agent/state",
        "/fmu1/fmu/out/vehicle_odometry",
        "/fmu2/fmu/out/vehicle_odometry",
    };

    // 自身 namespace = "fmu2_agent"，filter_peers 現在包含自身
    auto peers = PeerMatcher::filter_peers(topics, "fmu2_agent");
    ASSERT_EQ(peers.size(), 3u);
    EXPECT_EQ(peers[0].first, 1);
    EXPECT_EQ(peers[0].second, "/fmu1_agent/state");
    EXPECT_EQ(peers[1].first, 2);
    EXPECT_EQ(peers[1].second, "/fmu2_agent/state");
    EXPECT_EQ(peers[2].first, 3);
    EXPECT_EQ(peers[2].second, "/fmu3_agent/state");
}

TEST(PeerMatcherTest, FilterPeersEmptyInput)
{
    auto peers = PeerMatcher::filter_peers({}, "fmu1_agent");
    EXPECT_TRUE(peers.empty());
}

TEST(PeerMatcherTest, FilterPeersNoMatches)
{
    std::vector<std::string> topics = {
        "/some/other/topic",
        "/not/agent/state",
    };
    auto peers = PeerMatcher::filter_peers(topics, "fmu1_agent");
    EXPECT_TRUE(peers.empty());
}

TEST(PeerMatcherTest, FindsPeerRemovedFromDiscovery)
{
    const std::set<int> known_indices{2, 3, 4};
    const std::set<int> discovered_indices{2, 3};

    const auto removed_indices = PeerMatcher::find_removed_indices(
        known_indices, discovered_indices);

    ASSERT_EQ(removed_indices.size(), 1U);
    EXPECT_EQ(removed_indices.at(0), 4);
}

TEST(PeerMatcherTest, DoesNotRemoveRediscoveredPeer)
{
    const std::set<int> known_indices{2, 3, 4};
    const std::set<int> discovered_indices{2, 3, 4};

    const auto removed_indices = PeerMatcher::find_removed_indices(
        known_indices, discovered_indices);

    EXPECT_TRUE(removed_indices.empty());
}

// ==================== is_agent_node 測試 ====================

TEST(PeerMatcherTest, IsAgentNodeMatch)
{
    // 主模式：/fmu<N>_agent/<節點名>（一機一 agent 架構）
    EXPECT_TRUE(PeerMatcher::is_agent_node("/fmu1_agent/mission_planner"));
    EXPECT_TRUE(PeerMatcher::is_agent_node("/fmu2_agent/trajectory_controller"));
    EXPECT_TRUE(PeerMatcher::is_agent_node("/fmu10_agent/mission_planner"));
    EXPECT_TRUE(PeerMatcher::is_agent_node("/fmu3_agent/basic_controller_node"));
    // 相容模式：/fmu<N>/basic_controller_node（單容器 launch 部署）
    EXPECT_TRUE(PeerMatcher::is_agent_node("/fmu1/basic_controller_node"));
    EXPECT_TRUE(PeerMatcher::is_agent_node("/fmu3/basic_controller_node"));
}

TEST(PeerMatcherTest, IsAgentNodeNoMatch)
{
    // 相容模式只認 basic_controller_node，不將其他 /fmu<N>/ 節點視為 agent
    EXPECT_FALSE(PeerMatcher::is_agent_node("/fmu3/mission_planner"));
    EXPECT_FALSE(PeerMatcher::is_agent_node("/fmu10/some_node"));
    // 必須是 "_agent/" 接續（不可為其他後綴）
    EXPECT_FALSE(PeerMatcher::is_agent_node("/fmu1_agentx/mission_planner"));
    EXPECT_FALSE(PeerMatcher::is_agent_node("/fmu1_agent/"));
    EXPECT_FALSE(PeerMatcher::is_agent_node("/fmu1_agent"));
    // PX4 原生節點/topic 不誤判
    EXPECT_FALSE(PeerMatcher::is_agent_node("/fmu2/fmu/out/vehicle_odometry"));
    EXPECT_FALSE(PeerMatcher::is_agent_node("/fmu/fmu/out/odometry"));
    EXPECT_FALSE(PeerMatcher::is_agent_node(""));
    EXPECT_FALSE(PeerMatcher::is_agent_node("fmu1/node"));
    EXPECT_FALSE(PeerMatcher::is_agent_node("/fmu1"));
}

TEST(PeerMatcherTest, ExtractAgentIndexFromNode)
{
    EXPECT_EQ(PeerMatcher::extract_agent_index_from_node("/fmu1/basic_controller_node"), 1);
    EXPECT_EQ(PeerMatcher::extract_agent_index_from_node("/fmu2_agent/mission_planner"), 2);
    EXPECT_EQ(PeerMatcher::extract_agent_index_from_node("/fmu10_agent/trajectory_controller"), 10);
    EXPECT_EQ(PeerMatcher::extract_agent_index_from_node("/fmu3_agent/basic_controller_node"), 3);
    EXPECT_EQ(PeerMatcher::extract_agent_index_from_node("/invalid/node"), -1);
    EXPECT_EQ(PeerMatcher::extract_agent_index_from_node(""), -1);
}

// ==================== filter_agent_nodes 測試 ====================

TEST(PeerMatcherTest, FilterAgentNodesBasic)
{
    std::vector<std::string> nodes = {
        "/fmu1/basic_controller_node",
        "/fmu1_agent/mission_planner",
        "/fmu2/basic_controller_node",
        "/fmu2_agent/mission_planner",
        "/fmu3_agent/trajectory_controller",
        "/some/other/node",
    };

    // 自身 = "fmu2_agent"，應排除 fmu2 相關節點
    auto indices = PeerMatcher::filter_agent_nodes(nodes, "fmu2_agent");
    ASSERT_EQ(indices.size(), 2u);
    EXPECT_EQ(indices[0], 1);
    EXPECT_EQ(indices[1], 3);
}

TEST(PeerMatcherTest, FilterAgentNodesEmpty)
{
    auto indices = PeerMatcher::filter_agent_nodes({}, "fmu1_agent");
    EXPECT_TRUE(indices.empty());
}

TEST(PeerMatcherTest, FilterAgentNodesSelfOnly)
{
    std::vector<std::string> nodes = {
        "/fmu1/basic_controller_node",
        "/fmu1_agent/mission_planner",
        "/fmu1_agent/trajectory_controller",
    };
    // 只有自身 → 空結果
    auto indices = PeerMatcher::filter_agent_nodes(nodes, "fmu1_agent");
    EXPECT_TRUE(indices.empty());
}
