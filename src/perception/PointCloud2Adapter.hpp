/**
 * @file PointCloud2Adapter.hpp
 * @brief 純邏輯 PointCloud2Data 到 ROS2 PointCloud2 的 Adapter 介面
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 *
 * ROS2 依賴只存在於此 Adapter seam；幾何採樣器與測試不需要 sensor_msgs。
 * 目前依使用者限制不修改 CMakeLists.txt/package.xml，因此此檔案尚未接入建置目標。
 */

#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "perception/ObstacleCloudGenerator.hpp"

namespace taiphoon
{
namespace perception
{

/**
 * @brief 將純邏輯 PointCloud2Data 轉成 ROS2 PointCloud2
 */
class PointCloud2Adapter
{
public:
    /**
     * @brief 建立可發布的 ROS2 PointCloud2 訊息
     *
     * @param cloud 純邏輯 PointCloud2Data
     * @return ROS2 PointCloud2 訊息
     * @throws std::invalid_argument 當輸入資料不是有限且一致的 PointCloud2Data 時拋出
     */
    static sensor_msgs::msg::PointCloud2 to_ros_message(const PointCloud2Data& cloud);
};

}  // namespace perception
}  // namespace taiphoon
