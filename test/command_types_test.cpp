/**
 * @file command_types_test.cpp
 * @brief MISSION_START／MISSION_STOP／REBOOT_FC 解析與有效命令判斷測試
 * @author Taiphoon Agent Team
 * @date 2026-10-30
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "utils/CommandTypes.hpp"

namespace
{

using taiphoon::utils::AgentCommand;
using taiphoon::utils::MAX_MISSION_ID;
using taiphoon::utils::ParsedCommand;
using taiphoon::utils::agent_command_to_string;
using taiphoon::utils::has_valid_command_params;
using taiphoon::utils::is_agent_only_command;
using taiphoon::utils::is_mission_command;
using taiphoon::utils::parse_command_string;

TEST(CommandTypesTest, MissionStartAcceptsDecimalMissionId)
{
    const ParsedCommand command = parse_command_string("MISSION_START 123");
    EXPECT_EQ(command.command, AgentCommand::MISSION_START);
    EXPECT_TRUE(command.has_param);
    EXPECT_FALSE(command.has_param_error);
    EXPECT_EQ(command.mission_id, 123U);
    EXPECT_TRUE(has_valid_command_params(command));

    // 大小寫不分
    EXPECT_EQ(parse_command_string("mission_start 7").mission_id, 7U);

    // QGCS 以十進位整數輸出，超過 int32 的識別碼仍須完整保留
    const ParsedCommand large = parse_command_string("MISSION_START 4000000000");
    EXPECT_EQ(large.mission_id, 4000000000U);
    EXPECT_TRUE(has_valid_command_params(large));
}

TEST(CommandTypesTest, MissionStartAcceptsMaximumUint32)
{
    const ParsedCommand command = parse_command_string("MISSION_START 4294967295");
    EXPECT_EQ(command.mission_id, static_cast<uint32_t>(MAX_MISSION_ID));
    EXPECT_TRUE(has_valid_command_params(command));
}

TEST(CommandTypesTest, MissionStartRejectsInvalidMissionIds)
{
    const char* invalid_commands[] = {
        "MISSION_START 1.5",
        "MISSION_START 0",
        "MISSION_START -1",
        "MISSION_START +1",
        "MISSION_START 4294967296",
        "MISSION_START 12345678901",
        "MISSION_START abc",
        "MISSION_START 1e3"};
    for (const char* text : invalid_commands)
    {
        const ParsedCommand command = parse_command_string(text);
        EXPECT_EQ(command.command, AgentCommand::MISSION_START) << text;
        EXPECT_TRUE(command.has_param_error) << text;
        EXPECT_FALSE(has_valid_command_params(command)) << text;
    }

    // 缺少識別碼：不是格式錯誤，但仍不是可執行的命令
    const ParsedCommand missing = parse_command_string("MISSION_START");
    EXPECT_EQ(missing.command, AgentCommand::MISSION_START);
    EXPECT_FALSE(missing.has_param);
    EXPECT_FALSE(has_valid_command_params(missing));
}

TEST(CommandTypesTest, MissionStopHasNoParameters)
{
    const ParsedCommand command = parse_command_string("MISSION_STOP");
    EXPECT_EQ(command.command, AgentCommand::MISSION_STOP);
    EXPECT_TRUE(has_valid_command_params(command));
    EXPECT_TRUE(is_mission_command(command.command));
    EXPECT_TRUE(is_agent_only_command(command.command));
}

TEST(CommandTypesTest, RebootAcceptsOnlyZeroOrOne)
{
    const ParsedCommand plain = parse_command_string("REBOOT_FC");
    EXPECT_EQ(plain.command, AgentCommand::REBOOT_FC);
    EXPECT_FALSE(plain.has_param);
    EXPECT_TRUE(has_valid_command_params(plain));

    const ParsedCommand forced = parse_command_string("REBOOT_FC 1");
    EXPECT_TRUE(forced.has_param);
    EXPECT_FLOAT_EQ(forced.param_value, 1.0F);
    EXPECT_TRUE(has_valid_command_params(forced));

    const ParsedCommand not_forced = parse_command_string("REBOOT_FC 0");
    EXPECT_TRUE(not_forced.has_param);
    EXPECT_FLOAT_EQ(not_forced.param_value, 0.0F);

    for (const char* text : {"REBOOT_FC 2", "REBOOT_FC yes", "REBOOT_FC 1.0"})
    {
        const ParsedCommand command = parse_command_string(text);
        EXPECT_EQ(command.command, AgentCommand::REBOOT_FC) << text;
        EXPECT_TRUE(command.has_param_error) << text;
        EXPECT_FALSE(has_valid_command_params(command)) << text;
    }
    EXPECT_TRUE(is_agent_only_command(AgentCommand::REBOOT_FC));
    EXPECT_FALSE(is_mission_command(AgentCommand::REBOOT_FC));
}

TEST(CommandTypesTest, TakeoffWithNonNumericAltitudeIsInvalid)
{
    // 舊版會以預設高度起飛；現在不執行並回報 COMMAND_INVALID_PARAMS。
    const ParsedCommand invalid = parse_command_string("TAKEOFF abc");
    EXPECT_EQ(invalid.command, AgentCommand::TAKEOFF);
    EXPECT_TRUE(invalid.has_param_error);
    EXPECT_FALSE(has_valid_command_params(invalid));

    EXPECT_TRUE(has_valid_command_params(parse_command_string("TAKEOFF")));
    EXPECT_TRUE(has_valid_command_params(parse_command_string("TAKEOFF 5")));
}

TEST(CommandTypesTest, ParameterRequirementsForExistingCommands)
{
    EXPECT_FALSE(has_valid_command_params(parse_command_string("YAW")));
    EXPECT_TRUE(has_valid_command_params(parse_command_string("YAW 90")));
    EXPECT_FALSE(has_valid_command_params(parse_command_string("MOVE_FORWARD")));
    EXPECT_FALSE(has_valid_command_params(parse_command_string("MOVE_FORWARD x")));
    EXPECT_TRUE(has_valid_command_params(parse_command_string("MOVE_FORWARD 3")));
    EXPECT_TRUE(has_valid_command_params(parse_command_string("MOVE_POINT 1 2 -3 nan")));
    EXPECT_FALSE(has_valid_command_params(parse_command_string("MOVE_POINT 1 2")));
    EXPECT_FALSE(has_valid_command_params(parse_command_string("FORMATION_CIRCLE 5")));
    EXPECT_TRUE(has_valid_command_params(parse_command_string("FORMATION_CIRCLE 5 10")));
    EXPECT_TRUE(has_valid_command_params(parse_command_string("HOLD")));
    EXPECT_TRUE(has_valid_command_params(parse_command_string("MODE_RTL")));
    EXPECT_FALSE(has_valid_command_params(parse_command_string("NOT_A_COMMAND")));
    EXPECT_FALSE(is_agent_only_command(AgentCommand::HOLD));
}

TEST(CommandTypesTest, NewCommandNames)
{
    EXPECT_EQ(std::string(agent_command_to_string(AgentCommand::MISSION_START)),
              "MISSION_START");
    EXPECT_EQ(std::string(agent_command_to_string(AgentCommand::MISSION_STOP)),
              "MISSION_STOP");
    EXPECT_EQ(std::string(agent_command_to_string(AgentCommand::REBOOT_FC)), "REBOOT_FC");
}

}  // namespace
