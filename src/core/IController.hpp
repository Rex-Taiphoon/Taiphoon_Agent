/**
 * @file IController.hpp
 * @brief 控制器基礎介面（純虛擬基底類別）
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 */

#pragma once

namespace taiphoon
{
namespace core
{

/**
 * @brief 控制器基礎介面
 *
 * 所有飛行控制器均應實作此介面，定義控制器生命週期之基本操作。
 */
class IController
{
public:
    IController() = default;

    virtual ~IController() = default;

    /// 刪除複製建構子與賦值運算子，避免切片問題
    IController(const IController&) = delete;
    IController& operator=(const IController&) = delete;

    /// 移動建構子與賦值運算子
    IController(IController&&) = default;
    IController& operator=(IController&&) = default;

    /**
     * @brief 初始化控制器
     * @return true 初始化成功，false 初始化失敗
     */
    virtual bool initialize() = 0;

    /**
     * @brief 釋放控制器資源
     */
    virtual void shutdown() = 0;

    /**
     * @brief 檢查控制器是否已初始化
     * @return true 已初始化，false 尚未初始化
     */
    virtual bool is_initialized() const = 0;
};

}  // namespace core
}  // namespace taiphoon
