/**
 * @file CommandTypes.hpp
 * @brief Taiphoon Agent 高階命令類型定義與解析工具
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 */

#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <cstdint>

namespace taiphoon
{
namespace utils
{

/**
 * @brief 支援的高階命令列舉
 *
 * 這些命令可透過 ROS2 topic 傳送給 Taiphoon Agent，
 * 用於控制無人機的飛行模式、移動與任務。
 */
enum class AgentCommand : uint8_t
{
    UNKNOWN = 0,        ///< 未知或無法解析的命令
    MODE_RTL,           ///< 切換至 RTL (Return to Launch) 模式
    MODE_LAND,          ///< 切換至 LAND 模式
    MODE_OFFBOARD,      ///< 切換至 OFFBOARD 模式
    MOVE_FORWARD,       ///< 向前移動 x 公尺 (參數: 距離)
    MOVE_BACKWARD,      ///< 向後移動 x 公尺 (參數: 距離)
    MOVE_LEFT,          ///< 向左移動 x 公尺 (參數: 距離)
    MOVE_RIGHT,         ///< 向右移動 x 公尺 (參數: 距離)
    ARM,                ///< 解鎖 (Arm)
    DISARM,             ///< 上鎖 (Disarm)
    TAKEOFF,            ///< 起飛至指定高度 (參數: 高度公尺，預設 3.0m)
    HOLD,               ///< 切換至 Hold 模式 (保持目前位置)
    MOVE_UP,            ///< 向上移動 x 公尺 (參數: 距離)
    MOVE_DOWN,          ///< 向下移動 x 公尺 (參數: 距離)
    YAW,                ///< 順時針旋轉 x 度 (參數: 角度)
    MOVE_POINT,         ///< 移動至指定 NED 絕對位置 (參數: x y z yaw)
    FORMATION_CIRCLE,   ///< 環形編隊 (參數: 半徑 高度)
    FORMATION_HORI_LINEAR, ///< 水平直列編隊 (參數: 間距 高度)
    FORMATION_VERT_LINEAR, ///< 垂直直列編隊 (參數: 間距)
    FORMATION_CANCEL      ///< 取消當前編隊任務
};

/**
 * @brief 解析後的命令結構
 */
struct ParsedCommand
{
    AgentCommand command{AgentCommand::UNKNOWN};  ///< 命令類型
    float param_value{0.0F};                      ///< 命令參數值 (例如移動距離)
    bool has_param{false};                        ///< 是否包含數值參數

    /// 多參數命令支援 (用於 MOVE_POINT, FORMATION_* 等)
    float param2{0.0F};                           ///< 第二參數 (y / spacing)
    float param3{0.0F};                           ///< 第三參數 (z / altitude)
    float param4{0.0F};                           ///< 第四參數 (yaw)
    bool has_multi_params{false};                 ///< 是否為多參數命令
};

/**
 * @brief 將字串解析為高階命令
 *
 * 支援的命令格式：
 *   MODE_RTL, MODE_LAND, MODE_OFFBOARD,
 *   MOVE_FORWARD x, MOVE_BACKWARD x, MOVE_LEFT x, MOVE_RIGHT x,
 *   ARM, DISARM, TAKEOFF, HOLD,
 *   MOVE_UP x, MOVE_DOWN x,
 *   YAW x
 *
 * 為相容性亦支援簡寫：FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN
 *
 * @param cmd_str 輸入命令字串 (不區分大小寫)
 * @return ParsedCommand 解析結果
 */
inline ParsedCommand parse_command_string(const std::string& cmd_str)
{
    ParsedCommand result;

    if (cmd_str.empty())
    {
        return result;
    }

    // 將輸入字串轉為大寫以利比對
    std::string upper_str = cmd_str;
    for (auto& ch : upper_str)
    {
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }

    // 分割字串為 token
    std::istringstream iss(upper_str);
    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token)
    {
        tokens.push_back(token);
    }

    if (tokens.empty())
    {
        return result;
    }

    const std::string& cmd_name = tokens[0];

    if (cmd_name == "MODE_RTL")
    {
        result.command = AgentCommand::MODE_RTL;
    }
    else if (cmd_name == "MODE_LAND")
    {
        result.command = AgentCommand::MODE_LAND;
    }
    else if (cmd_name == "MODE_OFFBOARD")
    {
        result.command = AgentCommand::MODE_OFFBOARD;
    }
    else if (cmd_name == "MOVE_FORWARD" || cmd_name == "FORWARD")
    {
        result.command = AgentCommand::MOVE_FORWARD;
        if (tokens.size() >= 2)
        {
            try
            {
                result.param_value = std::stof(tokens[1]);
                result.has_param = true;
            }
            catch (...)
            {
                result.param_value = 0.0F;
                result.has_param = false;
            }
        }
    }
    else if (cmd_name == "MOVE_BACKWARD" || cmd_name == "BACKWARD")
    {
        result.command = AgentCommand::MOVE_BACKWARD;
        if (tokens.size() >= 2)
        {
            try
            {
                result.param_value = std::stof(tokens[1]);
                result.has_param = true;
            }
            catch (...)
            {
                result.param_value = 0.0F;
                result.has_param = false;
            }
        }
    }
    else if (cmd_name == "MOVE_LEFT" || cmd_name == "LEFT")
    {
        result.command = AgentCommand::MOVE_LEFT;
        if (tokens.size() >= 2)
        {
            try
            {
                result.param_value = std::stof(tokens[1]);
                result.has_param = true;
            }
            catch (...)
            {
                result.param_value = 0.0F;
                result.has_param = false;
            }
        }
    }
    else if (cmd_name == "MOVE_RIGHT" || cmd_name == "RIGHT")
    {
        result.command = AgentCommand::MOVE_RIGHT;
        if (tokens.size() >= 2)
        {
            try
            {
                result.param_value = std::stof(tokens[1]);
                result.has_param = true;
            }
            catch (...)
            {
                result.param_value = 0.0F;
                result.has_param = false;
            }
        }
    }
    else if (cmd_name == "ARM")
    {
        result.command = AgentCommand::ARM;
    }
    else if (cmd_name == "DISARM")
    {
        result.command = AgentCommand::DISARM;
    }
    else if (cmd_name == "TAKEOFF")
    {
        result.command = AgentCommand::TAKEOFF;
        if (tokens.size() >= 2)
        {
            try
            {
                result.param_value = std::stof(tokens[1]);
                result.has_param = true;
            }
            catch (...)
            {
                result.param_value = 0.0F;
                result.has_param = false;
            }
        }
    }
    else if (cmd_name == "HOLD")
    {
        result.command = AgentCommand::HOLD;
    }
    else if (cmd_name == "MOVE_UP" || cmd_name == "UP")
    {
        result.command = AgentCommand::MOVE_UP;
        if (tokens.size() >= 2)
        {
            try
            {
                result.param_value = std::stof(tokens[1]);
                result.has_param = true;
            }
            catch (...)
            {
                result.param_value = 0.0F;
                result.has_param = false;
            }
        }
    }
    else if (cmd_name == "MOVE_DOWN" || cmd_name == "DOWN")
    {
        result.command = AgentCommand::MOVE_DOWN;
        if (tokens.size() >= 2)
        {
            try
            {
                result.param_value = std::stof(tokens[1]);
                result.has_param = true;
            }
            catch (...)
            {
                result.param_value = 0.0F;
                result.has_param = false;
            }
        }
    }
    else if (cmd_name == "YAW")
    {
        result.command = AgentCommand::YAW;
        if (tokens.size() >= 2)
        {
            try
            {
                result.param_value = std::stof(tokens[1]);
                result.has_param = true;
            }
            catch (...)
            {
                result.param_value = 0.0F;
                result.has_param = false;
            }
        }
    }
    else if (cmd_name == "MOVE_POINT")
    {
        result.command = AgentCommand::MOVE_POINT;
        if (tokens.size() >= 5)
        {
            try
            {
                result.param_value = std::stof(tokens[1]);
                result.param2 = std::stof(tokens[2]);
                result.param3 = std::stof(tokens[3]);
                result.param4 = std::stof(tokens[4]);
                result.has_multi_params = true;
            }
            catch (...)
            {
                result.param_value = 0.0F;
            }
        }
    }
    else if (cmd_name == "FORMATION_CIRCLE")
    {
        result.command = AgentCommand::FORMATION_CIRCLE;
        if (tokens.size() >= 3)
        {
            try
            {
                result.param_value = std::stof(tokens[1]);  // 半徑
                result.param2 = std::stof(tokens[2]);        // 高度
                result.has_param = true;
                result.has_multi_params = true;
            }
            catch (...)
            {
                result.param_value = 0.0F;
            }
        }
    }
    else if (cmd_name == "FORMATION_HORI_LINEAR")
    {
        result.command = AgentCommand::FORMATION_HORI_LINEAR;
        if (tokens.size() >= 3)
        {
            try
            {
                result.param_value = std::stof(tokens[1]);  // 間距
                result.param2 = std::stof(tokens[2]);        // 高度
                result.has_param = true;
                result.has_multi_params = true;
            }
            catch (...)
            {
                result.param_value = 0.0F;
            }
        }
    }
    else if (cmd_name == "FORMATION_VERT_LINEAR")
    {
        result.command = AgentCommand::FORMATION_VERT_LINEAR;
        if (tokens.size() >= 2)
        {
            try
            {
                result.param_value = std::stof(tokens[1]);  // 間距
                // 垂直直列只有一個數值參數，不能標記成多參數命令；
                // planner 會依 has_param 驗證其間距是否有效。
                result.has_param = true;
            }
            catch (...)
            {
                result.param_value = 0.0F;
                result.has_param = false;
            }
        }
    }
    else if (cmd_name == "FORMATION_CANCEL")
    {
        result.command = AgentCommand::FORMATION_CANCEL;
    }
    else
    {
        result.command = AgentCommand::UNKNOWN;
    }

    return result;
}

/**
 * @brief 將命令轉為可讀字串 (用於日誌輸出)
 *
 * @param cmd 命令列舉值
 * @return const char* 命令名稱字串
 */
inline const char* agent_command_to_string(AgentCommand cmd)
{
    switch (cmd)
    {
        case AgentCommand::MODE_RTL:       return "MODE_RTL";
        case AgentCommand::MODE_LAND:      return "MODE_LAND";
        case AgentCommand::MODE_OFFBOARD:  return "MODE_OFFBOARD";
        case AgentCommand::MOVE_FORWARD:   return "MOVE_FORWARD";
        case AgentCommand::MOVE_BACKWARD:  return "MOVE_BACKWARD";
        case AgentCommand::MOVE_LEFT:      return "MOVE_LEFT";
        case AgentCommand::MOVE_RIGHT:     return "MOVE_RIGHT";
        case AgentCommand::ARM:            return "ARM";
        case AgentCommand::DISARM:         return "DISARM";
        case AgentCommand::TAKEOFF:        return "TAKEOFF";
        case AgentCommand::HOLD:           return "HOLD";
        case AgentCommand::MOVE_UP:        return "MOVE_UP";
        case AgentCommand::MOVE_DOWN:      return "MOVE_DOWN";
        case AgentCommand::YAW:                 return "YAW";
        case AgentCommand::MOVE_POINT:          return "MOVE_POINT";
        case AgentCommand::FORMATION_CIRCLE:     return "FORMATION_CIRCLE";
        case AgentCommand::FORMATION_HORI_LINEAR: return "FORMATION_HORI_LINEAR";
        case AgentCommand::FORMATION_VERT_LINEAR: return "FORMATION_VERT_LINEAR";
        case AgentCommand::FORMATION_CANCEL:      return "FORMATION_CANCEL";
        default:                                   return "UNKNOWN";
    }
}

}  // namespace utils
}  // namespace taiphoon
