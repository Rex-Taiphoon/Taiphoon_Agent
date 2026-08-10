/**
 * @file PointCloud2Adapter.cpp
 * @brief 純邏輯 PointCloud2Data 到 ROS2 PointCloud2 的 Adapter 實作
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 */

#include "perception/PointCloud2Adapter.hpp"

#include <stdexcept>

namespace taiphoon
{
namespace perception
{

sensor_msgs::msg::PointCloud2 PointCloud2Adapter::to_ros_message(
    const PointCloud2Data& cloud)
{
    if (!cloud.has_only_finite_points())
    {
        throw std::invalid_argument(
            "PointCloud2Data 含有非有限值或欄位排列不一致");
    }

    sensor_msgs::msg::PointCloud2 message;
    message.header.frame_id = cloud.frame_id;
    message.height = cloud.height;
    message.width = cloud.width;
    message.is_bigendian = cloud.is_bigendian;
    message.point_step = cloud.point_step;
    message.row_step = cloud.row_step;
    message.is_dense = cloud.is_dense;
    message.data = cloud.data;
    message.fields.reserve(cloud.fields.size());

    for (const auto& field : cloud.fields)
    {
        sensor_msgs::msg::PointField ros_field;
        ros_field.name = field.name;
        ros_field.offset = field.offset;
        ros_field.datatype = field.datatype;
        ros_field.count = field.count;
        message.fields.push_back(ros_field);
    }
    return message;
}

}  // namespace perception
}  // namespace taiphoon
