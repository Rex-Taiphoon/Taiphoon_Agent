/**
 * @file obstacle_cloud_simulator_node.cpp
 * @brief PointCloud2 障礙物模擬 ROS2 節點實作
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 */

#include "nodes/obstacle_cloud_simulator_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "perception/PointCloud2Adapter.hpp"

namespace taiphoon
{
namespace nodes
{

namespace
{
std::string trim(const std::string& value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1U);
}

bool parse_scalar(const std::string& line, float& value)
{
    const auto colon = line.find(':');
    if (colon == std::string::npos)
    {
        return false;
    }
    try
    {
        value = std::stof(trim(line.substr(colon + 1U)));
        return std::isfinite(value);
    }
    catch (const std::exception& error)
    {
        (void)error;
        return false;
    }
}

bool parse_vector(const std::string& line, perception::Point3f& value)
{
    const auto open = line.find('[');
    const auto close = line.find(']');
    if (open == std::string::npos || close <= open)
    {
        return false;
    }
    std::string list = line.substr(open + 1U, close - open - 1U);
    std::replace(list.begin(), list.end(), ',', ' ');
    std::istringstream stream(list);
    if (!(stream >> value.at(0) >> value.at(1) >> value.at(2)))
    {
        return false;
    }
    return std::isfinite(value.at(0)) && std::isfinite(value.at(1)) &&
        std::isfinite(value.at(2));
}

std::string scalar_text(const std::string& line)
{
    const auto colon = line.find(':');
    return colon == std::string::npos ? std::string{} :
        trim(line.substr(colon + 1U));
}
}

ObstacleCloudSimulatorNode::ObstacleCloudSimulatorNode()
    : rclcpp::Node("obstacle_cloud_simulator")
{
    declare_parameter<std::string>("agent_namespace", "fmu1_agent");
    declare_parameter<std::string>("config_file", "");
    declare_parameter<double>("publish_rate_hz", 15.0);
    declare_parameter<std::string>("mode", "sensor_emulation");
    declare_parameter<double>("range_m", 20.0);
    declare_parameter<double>("horizontal_fov_deg", 270.0);
    declare_parameter<double>("vertical_fov_deg", 90.0);
    declare_parameter<double>("noise_std_m", 0.02);
    declare_parameter<double>("dropout_ratio", 0.05);
    declare_parameter<double>("sample_spacing_m", 0.5);

    agent_namespace_ = get_parameter("agent_namespace").as_string();
    config_file_ = get_parameter("config_file").as_string();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    generation_config_.mode = get_parameter("mode").as_string() ==
        "ground_truth_map" ? perception::CloudMode::GroundTruthMap :
        perception::CloudMode::SensorEmulation;
    generation_config_.sensor.range_m = static_cast<float>(
        get_parameter("range_m").as_double());
    generation_config_.sensor.horizontal_fov_deg = static_cast<float>(
        get_parameter("horizontal_fov_deg").as_double());
    generation_config_.sensor.vertical_fov_deg = static_cast<float>(
        get_parameter("vertical_fov_deg").as_double());
    generation_config_.sensor.noise_std_m = static_cast<float>(
        get_parameter("noise_std_m").as_double());
    generation_config_.sensor.dropout_ratio = static_cast<float>(
        get_parameter("dropout_ratio").as_double());
    generation_config_.sample_spacing_m = static_cast<float>(
        get_parameter("sample_spacing_m").as_double());

    if (!config_file_.empty() && !load_scene_file(config_file_))
    {
        RCLCPP_WARN(get_logger(),
                    "Could not load obstacle config '%s'; using parameters",
                    config_file_.c_str());
    }
    if (obstacles_.empty())
    {
        perception::ObstacleDefinition obstacle;
        obstacle.type = perception::ObstacleType::Box;
        obstacle.center_ned = {12.0F, 0.0F, -3.0F};
        obstacle.size_m = {4.0F, 8.0F, 6.0F};
        obstacles_.push_back(obstacle);
    }

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/" + agent_namespace_ + "/perception/obstacles",
        rclcpp::QoS(10).best_effort());
    if (cloud_pub_ == nullptr)
    {
        RCLCPP_ERROR(get_logger(), "Failed to create obstacle cloud publisher");
        return;
    }

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0)
    {
        publish_rate_hz_ = 15.0;
    }
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_));
    publish_timer_ = create_wall_timer(
        std::max(period, std::chrono::milliseconds(1)),
        std::bind(&ObstacleCloudSimulatorNode::publish_cloud_cb, this));
    publish_cloud_cb();
}

bool ObstacleCloudSimulatorNode::load_scene_file(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        return false;
    }

    perception::ObstacleDefinition* current = nullptr;
    std::string line;
    while (std::getline(file, line))
    {
        const std::string clean = trim(line.substr(0, line.find('#')));
        if (clean.empty())
        {
            continue;
        }
        if (clean.find("frame_id:") == 0U)
        {
            generation_config_.frame_id = scalar_text(clean);
        }
        else if (clean.find("mode:") == 0U)
        {
            generation_config_.mode = scalar_text(clean) ==
                "ground_truth_map" ? perception::CloudMode::GroundTruthMap :
                perception::CloudMode::SensorEmulation;
        }
        else if (clean.find("publish_rate_hz:") == 0U)
        {
            float value = 0.0F;
            if (parse_scalar(clean, value))
            {
                publish_rate_hz_ = value;
            }
        }
        else if (clean.find("range_m:") == 0U)
        {
            parse_scalar(clean, generation_config_.sensor.range_m);
        }
        else if (clean.find("horizontal_fov_deg:") == 0U)
        {
            parse_scalar(clean, generation_config_.sensor.horizontal_fov_deg);
        }
        else if (clean.find("vertical_fov_deg:") == 0U)
        {
            parse_scalar(clean, generation_config_.sensor.vertical_fov_deg);
        }
        else if (clean.find("noise_std_m:") == 0U)
        {
            parse_scalar(clean, generation_config_.sensor.noise_std_m);
        }
        else if (clean.find("dropout_ratio:") == 0U)
        {
            parse_scalar(clean, generation_config_.sensor.dropout_ratio);
        }
        else if (clean.find("sample_spacing_m:") == 0U)
        {
            parse_scalar(clean, generation_config_.sample_spacing_m);
        }
        else if (clean.find("- type:") == 0U)
        {
            perception::ObstacleDefinition obstacle;
            const std::string type = scalar_text(clean);
            if (type == "box")
            {
                obstacle.type = perception::ObstacleType::Box;
            }
            else if (type == "cylinder")
            {
                obstacle.type = perception::ObstacleType::Cylinder;
            }
            else if (type == "wall")
            {
                obstacle.type = perception::ObstacleType::Wall;
            }
            else
            {
                current = nullptr;
                continue;
            }
            obstacles_.push_back(obstacle);
            current = &obstacles_.back();
        }
        else if (current != nullptr && clean.find("center_ned:") == 0U)
        {
            parse_vector(clean, current->center_ned);
        }
        else if (current != nullptr && clean.find("size_m:") == 0U)
        {
            parse_vector(clean, current->size_m);
        }
        else if (current != nullptr && clean.find("radius_m:") == 0U)
        {
            parse_scalar(clean, current->radius_m);
        }
        else if (current != nullptr && clean.find("height_m:") == 0U)
        {
            parse_scalar(clean, current->height_m);
        }
    }
    return !file.bad() && !obstacles_.empty();
}

void ObstacleCloudSimulatorNode::publish_cloud_cb()
{
    if (cloud_pub_ == nullptr)
    {
        return;
    }
    try
    {
        const auto generated = perception::ObstacleCloudGenerator::generate(
            obstacles_, generation_config_, sensor_pose_);
        auto message = perception::PointCloud2Adapter::to_ros_message(
            generated.first);
        message.header.stamp = now();
        cloud_pub_->publish(message);
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                              "Obstacle cloud generation failed: %s",
                              error.what());
    }
}

}  // namespace nodes
}  // namespace taiphoon
