/**
 * @file agent_naming_test.cpp
 * @brief UAV namespace 與 Agent topic 命名工具單元測試
 * @author Taiphoon Agent Team
 * @date 2026-08-27
 */

#include <gtest/gtest.h>

#include "utils/AgentNaming.hpp"

namespace
{

using taiphoon::utils::agent_naming::make_agent_namespace;
using taiphoon::utils::agent_naming::make_agent_state_topic;
using taiphoon::utils::agent_naming::make_all_agent_topic;
using taiphoon::utils::agent_naming::make_uav_namespace;
using taiphoon::utils::agent_naming::parse_agent_namespace_index;
using taiphoon::utils::agent_naming::parse_uav_namespace_index;

TEST(AgentNamingTest, BuildsUavAndAgentNamespaces)
{
    EXPECT_EQ(make_uav_namespace(1), "uav_1");
    EXPECT_EQ(make_uav_namespace(2), "uav_2");
    EXPECT_EQ(make_agent_namespace(1), "uav_1_agent");
    EXPECT_EQ(make_agent_namespace(2), "uav_2_agent");
    EXPECT_EQ(make_agent_state_topic(1), "/uav_1_agent/state");
}

TEST(AgentNamingTest, ParsesOnlyTheNewNamespaceFormat)
{
    EXPECT_EQ(parse_uav_namespace_index("uav_1"), 1);
    EXPECT_EQ(parse_uav_namespace_index("uav_10"), 10);
    EXPECT_EQ(parse_agent_namespace_index("uav_2_agent"), 2);
    EXPECT_EQ(parse_uav_namespace_index("fmu1"), -1);
    EXPECT_EQ(parse_agent_namespace_index("fmu1_agent"), -1);
    EXPECT_EQ(parse_agent_namespace_index("uav__agent"), -1);
    EXPECT_EQ(parse_agent_namespace_index("uav_1_agent_extra"), -1);
}

TEST(AgentNamingTest, BuildsBroadcastTopics)
{
    EXPECT_EQ(make_all_agent_topic("command"), "/uav_all_agent/command");
    EXPECT_EQ(make_all_agent_topic("/swarm/formation_epochs"),
              "/uav_all_agent/swarm/formation_epochs");
    EXPECT_EQ(make_all_agent_topic(""), "/uav_all_agent");
}

}  // namespace
