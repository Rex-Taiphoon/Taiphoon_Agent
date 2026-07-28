/**
 * @file IBasicFlightController.hpp
 * @brief 基礎飛行控制器介面定義
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 *
 * 定義所有飛行控制器應實作的基本功能介面，
 * 包含起飛、降落、返航、模式切換、水平移動與旋轉等。
 */

#pragma once

#include <cstdint>
#include "IController.hpp"

namespace taiphoon
{
namespace core
{

/**
 * @brief 基礎飛行控制器抽象介面
 *
 * 所有飛行控制器實作必須繼承此介面。
 * 遵循 SOLID 原則中的介面隔離原則 (ISP)。
 */
class IBasicFlightController : public IController
{
public:
    IBasicFlightController() = default;
    ~IBasicFlightController() override = default;

    // 禁止複製語義
    IBasicFlightController(const IBasicFlightController&) = delete;
    IBasicFlightController& operator=(const IBasicFlightController&) = delete;

    // 允許移動語義
    IBasicFlightController(IBasicFlightController&&) = default;
    IBasicFlightController& operator=(IBasicFlightController&&) = default;

    // ==================== 基本飛行命令 ====================

    /**
     * @brief 解鎖 (Arm)
     *
     * @return true 解鎖命令發送成功
     */
    virtual bool arm() = 0;

    /**
     * @brief 上鎖 (Disarm)
     *
     * @return true 上鎖命令發送成功
     */
    virtual bool disarm() = 0;

    /**
     * @brief 起飛至指定高度
     *
     * @param altitude_m 目標高度 (公尺)，必須為正數
     * @return true 起飛命令設定成功
     */
    virtual bool takeoff(float altitude_m) = 0;

    /**
     * @brief 在空中停留指定時間
     *
     * @param duration_s 停留時間 (秒)，必須為正數
     * @return true 停留命令設定成功
     */
    virtual bool hold_position(float duration_s) = 0;

    /**
     * @brief 降落
     *
     * @return true 降落命令設定成功
     */
    virtual bool land() = 0;

    // ==================== 任務序列 ====================

    /**
     * @brief 執行起飛-停留-降落完整任務序列
     *
     * @param altitude_m 目標高度 (公尺)
     * @param hover_duration_s 停留時間 (秒)
     * @return true 序列啟動成功
     */
    virtual bool execute_takeoff_land_sequence(
        float altitude_m, float hover_duration_s) = 0;

    // ==================== 高階命令 ====================

    /**
     * @brief 切換至 RTL (Return to Launch) 模式
     *
     * @return true 命令設定成功
     */
    virtual bool switch_to_rtl_mode() = 0;

    /**
     * @brief 切換至 LAND 模式
     *
     * @return true 命令設定成功
     */
    virtual bool switch_to_land_mode() = 0;

    /**
     * @brief 切換至 OFFBOARD 模式
     *
     * @return true 命令設定成功
     */
    virtual bool switch_to_offboard_mode() = 0;

    /**
     * @brief 向前或向後移動指定距離
     *
     * @param distance_m 移動距離 (公尺)，正數為向前，負數為向後
     * @return true 移動命令設定成功
     */
    virtual bool move_forward(float distance_m) = 0;

    /**
     * @brief 向右或向左移動指定距離
     *
     * @param distance_m 移動距離 (公尺)，正數為向右，負數為向左
     * @return true 移動命令設定成功
     */
    virtual bool move_right(float distance_m) = 0;

    /**
     * @brief 垂直移動指定高度 (相對位移)
     *
     * @param delta_m 高度變化量 (公尺)，正數為上升，負數為下降
     * @return true 移動命令設定成功
     */
    virtual bool move_vertical(float delta_m) = 0;

    /**
     * @brief 旋轉至指定偏航角 (順時針)
     *
     * @param degrees 偏航角度 (度)，正數為順時針
     * @return true 旋轉命令設定成功
     */
    virtual bool rotate_yaw(float degrees) = 0;

    /**
     * @brief 設定 NED 絕對座標目標位置與航向
     *
     * 直接寫入目標 NED 位置，由控制器持續發布 TrajectorySetpoint
     * 直到收到新的命令為止。不進行內部達成判斷。
     *
     * @param x NED X 軸目標位置 (前向為正，公尺)
     * @param y NED Y 軸目標位置 (右向為正，公尺)
     * @param z NED Z 軸目標位置 (向下為正，公尺)
     * @param yaw_deg 目標航向角 (度，順時針為正)，NaN 表示不控制
     * @return true 目標設定成功
     */
    virtual bool set_ned_target(float x, float y, float z, float yaw_deg) = 0;

    // ==================== 狀態查詢 ====================

    /**
     * @brief 取得目前飛行狀態字串
     *
     * @return const char* 狀態字串指標
     */
    virtual const char* get_state() const = 0;
};

}  // namespace core
}  // namespace taiphoon
