/**
 * @file dds_peer_relay_plan_test.cpp
 * @brief Docker Desktop 模擬 DDS 同儕轉送埠段單元測試
 * @author Taiphoon Agent Team
 * @date 2026-10-16
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "utils/DdsPeerRelayPlan.hpp"

namespace
{

using taiphoon::utils::DdsPeerRelayConfig;
using taiphoon::utils::dds_relay_first_port;
using taiphoon::utils::make_dds_peer_relay_ports;

TEST(DdsPeerRelayPlanTest, FirstPortsMatchLauncherRanges)
{
    // 與 agent_launcher／docker-compose.local-qgcs.yml 的埠段一致：
    // domain 0：agent_1 7410-7433、agent_2 7434-7457、agent_3 7458-7481、agent_4 7482-7505。
    EXPECT_EQ(dds_relay_first_port(0U, 1U), 7410U);
    EXPECT_EQ(dds_relay_first_port(0U, 2U), 7434U);
    EXPECT_EQ(dds_relay_first_port(0U, 3U), 7458U);
    EXPECT_EQ(dds_relay_first_port(0U, 4U), 7482U);
    EXPECT_EQ(dds_relay_first_port(1U, 1U), 7660U);
}

TEST(DdsPeerRelayPlanTest, ExcludesOwnRangeAndCoversEveryPeer)
{
    std::vector<std::uint16_t> ports;
    ASSERT_TRUE(make_dds_peer_relay_ports(DdsPeerRelayConfig{0U, 2U, 4U}, ports));
    ASSERT_EQ(ports.size(), 72U);
    EXPECT_EQ(ports.front(), 7410U);
    EXPECT_EQ(ports.at(23), 7433U);
    EXPECT_EQ(ports.at(24), 7458U);
    EXPECT_EQ(ports.back(), 7505U);
    for (const std::uint16_t port : ports)
    {
        EXPECT_FALSE(port >= 7434U && port <= 7457U) << port;
    }
}

TEST(DdsPeerRelayPlanTest, SingleAgentNeedsNoRelayPorts)
{
    std::vector<std::uint16_t> ports{1U};
    ASSERT_TRUE(make_dds_peer_relay_ports(DdsPeerRelayConfig{0U, 1U, 1U}, ports));
    EXPECT_TRUE(ports.empty());
}

TEST(DdsPeerRelayPlanTest, RejectsOutOfRangeConfig)
{
    std::vector<std::uint16_t> ports{1U};
    EXPECT_FALSE(make_dds_peer_relay_ports(DdsPeerRelayConfig{0U, 0U, 4U}, ports));
    EXPECT_TRUE(ports.empty());
    EXPECT_FALSE(make_dds_peer_relay_ports(DdsPeerRelayConfig{0U, 5U, 4U}, ports));
    EXPECT_FALSE(make_dds_peer_relay_ports(DdsPeerRelayConfig{0U, 1U, 5U}, ports));
    EXPECT_FALSE(make_dds_peer_relay_ports(DdsPeerRelayConfig{0U, 1U, 0U}, ports));
    EXPECT_FALSE(make_dds_peer_relay_ports(DdsPeerRelayConfig{233U, 1U, 4U}, ports));
}

TEST(DdsPeerRelayPlanTest, MapsPortToOwningInstance)
{
    using taiphoon::utils::dds_relay_instance_for_port;
    const DdsPeerRelayConfig config{0U, 1U, 4U};
    EXPECT_EQ(dds_relay_instance_for_port(config, 7409U), 0U);
    EXPECT_EQ(dds_relay_instance_for_port(config, 7410U), 1U);
    EXPECT_EQ(dds_relay_instance_for_port(config, 7433U), 1U);
    EXPECT_EQ(dds_relay_instance_for_port(config, 7434U), 2U);
    EXPECT_EQ(dds_relay_instance_for_port(config, 7481U), 3U);
    EXPECT_EQ(dds_relay_instance_for_port(config, 7505U), 4U);
    // 超出 4 台埠段（例如 QGCS 的 7510）不屬於任何 Agent
    EXPECT_EQ(dds_relay_instance_for_port(config, 7510U), 0U);
    EXPECT_EQ(dds_relay_instance_for_port(DdsPeerRelayConfig{0U, 1U, 2U}, 7458U), 0U);
    EXPECT_EQ(dds_relay_instance_for_port(DdsPeerRelayConfig{1U, 2U, 4U}, 7684U), 2U);
}

TEST(DdsPeerRelayPlanTest, BuildsPeerHostFromTemplate)
{
    using taiphoon::utils::make_dds_peer_host;
    EXPECT_EQ(make_dds_peer_host("uav_{N}_agent", 3U), "uav_3_agent");
    EXPECT_EQ(make_dds_peer_host("agent_{N}", 12U), "agent_12");
    EXPECT_EQ(make_dds_peer_host("{N}-{N}", 2U), "2-2");
    EXPECT_EQ(make_dds_peer_host("", 2U), "");
}

TEST(DdsPeerRelayPlanTest, HighestDomainStaysInsideUdpPortRange)
{
    std::vector<std::uint16_t> ports;
    ASSERT_TRUE(make_dds_peer_relay_ports(DdsPeerRelayConfig{232U, 1U, 4U}, ports));
    ASSERT_FALSE(ports.empty());
    EXPECT_EQ(ports.back(), 7400U + 250U * 232U + 10U + 24U * 4U - 1U);
}

}  // namespace
