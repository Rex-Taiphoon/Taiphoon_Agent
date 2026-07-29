/**
 * @file peer_matcher_test.cpp
 * @brief PeerMatcher 單元測試
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 */

#include "formation/PeerMatcher.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

static int g_test_count = 0;
static int g_pass_count = 0;
static int g_fail_count = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        g_test_count++; \
        if (!(cond)) { \
            std::printf("  FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
            g_fail_count++; \
        } else { \
            g_pass_count++; \
        } \
    } while(0)

using taiphoon::formation::PeerMatcher;

void test_is_agent_state_topic_match()
{
    std::printf("\n--- test_is_agent_state_topic_match ---\n");
    TEST_ASSERT(PeerMatcher::is_agent_state_topic("/fmu1_agent/state"),
                "/fmu1_agent/state 應符合");
    TEST_ASSERT(PeerMatcher::is_agent_state_topic("/fmu2_agent/state"),
                "/fmu2_agent/state 應符合");
    TEST_ASSERT(PeerMatcher::is_agent_state_topic("/fmu10_agent/state"),
                "/fmu10_agent/state 應符合");
    TEST_ASSERT(!PeerMatcher::is_agent_state_topic("/fmu_all_agent/state"),
                "/fmu_all_agent/state 不應符合 (這是命令 topic 前綴)");
}

void test_is_agent_state_topic_no_match()
{
    std::printf("\n--- test_is_agent_state_topic_no_match ---\n");
    TEST_ASSERT(!PeerMatcher::is_agent_state_topic("/fmu1/fmu/out/vehicle_odometry"),
                "PX4 內部 topic 不應符合");
    TEST_ASSERT(!PeerMatcher::is_agent_state_topic("/fmu1_agent/ned_target"),
                "ned_target topic 不應符合");
    TEST_ASSERT(!PeerMatcher::is_agent_state_topic("/fmu1_agent/attitude"),
                "attitude topic 不應符合");
    TEST_ASSERT(!PeerMatcher::is_agent_state_topic(""),
                "空字串不應符合");
    TEST_ASSERT(!PeerMatcher::is_agent_state_topic("/fmu1/state"),
                "缺少 _agent 不應符合");
}

void test_extract_agent_index()
{
    std::printf("\n--- test_extract_agent_index ---\n");
    TEST_ASSERT(PeerMatcher::extract_agent_index("/fmu1_agent/state") == 1,
                "fmu1 → 1");
    TEST_ASSERT(PeerMatcher::extract_agent_index("/fmu2_agent/state") == 2,
                "fmu2 → 2");
    TEST_ASSERT(PeerMatcher::extract_agent_index("/fmu10_agent/state") == 10,
                "fmu10 → 10");
    TEST_ASSERT(PeerMatcher::extract_agent_index("/fmu0_agent/state") == 0,
                "fmu0 → 0 (edge case)");
}

void test_extract_agent_index_negative()
{
    std::printf("\n--- test_extract_agent_index_negative ---\n");
    // 無法提取時回傳 -1
    TEST_ASSERT(PeerMatcher::extract_agent_index("") == -1,
                "空字串 → -1");
    TEST_ASSERT(PeerMatcher::extract_agent_index("/invalid/topic") == -1,
                "不符合格式 → -1");
    TEST_ASSERT(PeerMatcher::extract_agent_index("/fmu_agent/state") == -1,
                "缺少數字 → -1");
}

void test_filter_peers_basic()
{
    std::printf("\n--- test_filter_peers_basic ---\n");
    std::vector<std::string> topics = {
        "/fmu1_agent/state",
        "/fmu2_agent/state",
        "/fmu3_agent/state",
        "/fmu1/fmu/out/vehicle_odometry",
        "/fmu2/fmu/out/vehicle_odometry",
    };

    // 自身 namespace = "fmu2_agent"，應排除 fmu2_agent/state
    auto peers = PeerMatcher::filter_peers(topics, "fmu2_agent");
    TEST_ASSERT(peers.size() == 2, "應找到 2 個同儕 (排除自身)");
    if (peers.size() >= 2)
    {
        TEST_ASSERT(peers[0].first == 1, "peer[0] index = 1");
        TEST_ASSERT(peers[0].second == "/fmu1_agent/state",
                    "peer[0] topic = /fmu1_agent/state");
        TEST_ASSERT(peers[1].first == 3, "peer[1] index = 3");
    }
}

void test_filter_peers_empty_input()
{
    std::printf("\n--- test_filter_peers_empty_input ---\n");
    auto peers = PeerMatcher::filter_peers({}, "fmu1_agent");
    TEST_ASSERT(peers.empty(), "空輸入 → 空結果");
}

void test_filter_peers_no_matches()
{
    std::printf("\n--- test_filter_peers_no_matches ---\n");
    std::vector<std::string> topics = {
        "/some/other/topic",
        "/not/agent/state",
    };
    auto peers = PeerMatcher::filter_peers(topics, "fmu1_agent");
    TEST_ASSERT(peers.empty(), "無符合 → 空結果");
}

// ==================== 主程式 ====================

int main()
{
    std::printf("=== PeerMatcher 單元測試 ===\n");

    test_is_agent_state_topic_match();
    test_is_agent_state_topic_no_match();
    test_extract_agent_index();
    test_extract_agent_index_negative();
    test_filter_peers_basic();
    test_filter_peers_empty_input();
    test_filter_peers_no_matches();

    std::printf("\n=== 測試結果: %d/%d 通過, %d 失敗 ===\n",
                g_pass_count, g_test_count, g_fail_count);

    return (g_fail_count == 0) ? 0 : 1;
}
