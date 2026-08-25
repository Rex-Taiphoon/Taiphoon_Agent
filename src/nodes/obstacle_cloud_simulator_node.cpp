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
#include <functional>
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

perception::CloudMode parse_cloud_mode(const std::string& value)
{
    if (value == "explicit_empty")
    {
        return perception::CloudMode::ExplicitEmpty;
    }
    if (value == "ground_truth" || value == "ground_truth_map")
    {
        return perception::CloudMode::GroundTruth;
    }
    return perception::CloudMode::SyntheticSensor;
}

const char* cloud_mode_name(const perception::CloudMode mode)
{
    switch (mode)
    {
        case perception::CloudMode::ExplicitEmpty:
            return "explicit_empty";
        case perception::CloudMode::GroundTruth:
            return "ground_truth";
        case perception::CloudMode::SyntheticSensor:
        default:
            return "synthetic_sensor";
    }
}

bool is_valid_global_state(const taiphoon_agent::msg::AgentState& state)
{
    return state.online && state.px4_state_valid &&
        std::isfinite(state.position_global.at(0)) &&
        std::isfinite(state.position_global.at(1)) &&
        std::isfinite(state.position_global.at(2));
}

ego_swarm::GeoCoordinate to_geo(
    const taiphoon_agent::msg::AgentState& state)
{
    return ego_swarm::GeoCoordinate{
        static_cast<double>(state.position_global.at(0)),
        static_cast<double>(state.position_global.at(1)),
        static_cast<double>(state.position_global.at(2))};
}
}

ObstacleCloudSimulatorNode::ObstacleCloudSimulatorNode()
    : rclcpp::Node("obstacle_cloud_simulator")
{
    declare_parameter<std::string>("agent_namespace", "fmu1_agent");
    declare_parameter<std::string>("config_file", "");
    declare_parameter<int>("agent_index", 1);
    declare_parameter<int>("swarm_origin_agent_index", 1);
    declare_parameter<double>("publish_rate_hz", 15.0);
    // 模擬預設為無障礙，但仍保留 simulator 發布 fresh empty cloud，讓
    // planner 可以區分「沒有障礙物」與「障礙物資料遺失」兩種狀態。
    declare_parameter<std::string>("mode", "explicit_empty");
    declare_parameter<double>("range_m", 20.0);
    declare_parameter<double>("horizontal_fov_deg", 270.0);
    declare_parameter<double>("vertical_fov_deg", 90.0);
    declare_parameter<double>("noise_std_m", 0.02);
    declare_parameter<double>("dropout_ratio", 0.05);
    declare_parameter<double>("sample_spacing_m", 0.5);

    agent_namespace_ = get_parameter("agent_namespace").as_string();
    config_file_ = get_parameter("config_file").as_string();
    agent_index_ = get_parameter("agent_index").as_int();
    if (agent_index_ <= 0)
    {
        agent_index_ = compute_agent_index();
    }
    swarm_origin_agent_index_ = get_parameter(
        "swarm_origin_agent_index").as_int();
    if (swarm_origin_agent_index_ <= 0)
    {
        swarm_origin_agent_index_ = 1;
    }
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    const std::string requested_mode = get_parameter("mode").as_string();
    const perception::CloudMode parameter_mode = parse_cloud_mode(requested_mode);
    generation_config_.mode = parameter_mode;
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
    // YAML 僅提供場景幾何與 sensor 參數；有效 source mode 必須由 ROS2
    // parameter 決定，避免 default.yaml 的 synthetic_sensor 意外讓無障礙
    // 基線重新產生障礙物點雲。
    generation_config_.mode = parameter_mode;
    if (obstacles_.empty() &&
        generation_config_.mode != perception::CloudMode::ExplicitEmpty)
    {
        perception::ObstacleDefinition obstacle;
        obstacle.type = perception::ObstacleType::Box;
        obstacle.center_ned = {12.0F, 0.0F, -3.0F};
        obstacle.size_m = {4.0F, 8.0F, 6.0F};
        obstacles_.push_back(obstacle);
    }

    RCLCPP_INFO(
        get_logger(),
        "Obstacle cloud simulator effective mode=%s (ROS2 mode='%s'); "
        "scene='%s' obstacles=%zu",
        cloud_mode_name(generation_config_.mode), requested_mode.c_str(),
        config_file_.c_str(), obstacles_.size());

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/" + agent_namespace_ + "/perception/obstacles",
        rclcpp::QoS(10).best_effort());
    if (cloud_pub_ == nullptr)
    {
        RCLCPP_ERROR(get_logger(), "Failed to create obstacle cloud publisher");
        return;
    }

    if (generation_config_.mode == perception::CloudMode::SyntheticSensor &&
        !initialize_pose_subscribers())
    {
        RCLCPP_ERROR(get_logger(),
                     "Failed to initialize synthetic sensor pose subscribers");
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
            // mode 的優先權屬於 ROS2 parameter；保留此分支只為明確忽略
            // YAML 欄位，幾何與 sensor 設定仍由同一份檔案載入。
            continue;
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

bool ObstacleCloudSimulatorNode::initialize_pose_subscribers()
{
    own_state_sub_ = create_subscription<taiphoon_agent::msg::AgentState>(
        "/" + agent_namespace_ + "/state", rclcpp::QoS(10).best_effort(),
        std::bind(&ObstacleCloudSimulatorNode::own_state_cb, this,
                  std::placeholders::_1));
    attitude_sub_ = create_subscription<taiphoon_agent::msg::AgentAttitude>(
        "/" + agent_namespace_ + "/attitude", rclcpp::QoS(10).best_effort(),
        std::bind(&ObstacleCloudSimulatorNode::attitude_cb, this,
                  std::placeholders::_1));

    if (agent_index_ != swarm_origin_agent_index_)
    {
        const std::string origin_topic = "/fmu" +
            std::to_string(swarm_origin_agent_index_) + "_agent/state";
        origin_state_sub_ = create_subscription<
            taiphoon_agent::msg::AgentState>(
                origin_topic, rclcpp::QoS(10).best_effort(),
                std::bind(&ObstacleCloudSimulatorNode::origin_state_cb, this,
                          std::placeholders::_1));
    }

    return own_state_sub_ != nullptr && attitude_sub_ != nullptr &&
        (agent_index_ == swarm_origin_agent_index_ ||
         origin_state_sub_ != nullptr);
}

void ObstacleCloudSimulatorNode::own_state_cb(
    const taiphoon_agent::msg::AgentState::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "own_state_cb received nullptr");
        return;
    }
    own_state_ = *msg;
    has_own_state_ = true;
    if (agent_index_ == swarm_origin_agent_index_)
    {
        origin_state_ = *msg;
        has_origin_state_ = true;
    }
    update_sensor_pose();
}

void ObstacleCloudSimulatorNode::origin_state_cb(
    const taiphoon_agent::msg::AgentState::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "origin_state_cb received nullptr");
        return;
    }
    origin_state_ = *msg;
    has_origin_state_ = true;
    update_sensor_pose();
}

void ObstacleCloudSimulatorNode::attitude_cb(
    const taiphoon_agent::msg::AgentAttitude::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "attitude_cb received nullptr");
        return;
    }
    for (const float component : msg->q)
    {
        if (!std::isfinite(component))
        {
            RCLCPP_WARN(get_logger(), "Rejecting non-finite sensor attitude");
            return;
        }
    }

    const double w = static_cast<double>(msg->q.at(0));
    const double x = static_cast<double>(msg->q.at(1));
    const double y = static_cast<double>(msg->q.at(2));
    const double z = static_cast<double>(msg->q.at(3));
    const double norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (!std::isfinite(norm) || norm <= 1.0e-6)
    {
        RCLCPP_WARN(get_logger(), "Rejecting zero-length sensor attitude");
        return;
    }

    const double normalized_w = w / norm;
    const double normalized_x = x / norm;
    const double normalized_y = y / norm;
    const double normalized_z = z / norm;
    const double pitch_argument = std::max(
        -1.0, std::min(1.0, 2.0 * (
            normalized_w * normalized_y - normalized_z * normalized_x)));
    const double heading = std::atan2(
        2.0 * (normalized_w * normalized_z +
               normalized_x * normalized_y),
        1.0 - 2.0 * (normalized_y * normalized_y +
                      normalized_z * normalized_z));
    const double pitch = std::asin(pitch_argument);
    if (!std::isfinite(heading) || !std::isfinite(pitch))
    {
        RCLCPP_WARN(get_logger(), "Rejecting invalid sensor attitude angles");
        return;
    }
    sensor_pose_.heading_rad = static_cast<float>(heading);
    sensor_pose_.pitch_rad = static_cast<float>(pitch);
}

void ObstacleCloudSimulatorNode::update_sensor_pose()
{
    if (!has_own_state_ || !is_valid_global_state(own_state_))
    {
        return;
    }

    if (!shared_obstacle_frame_.has_origin())
    {
        if (!has_origin_state_ || !is_valid_global_state(origin_state_) ||
            !shared_obstacle_frame_.set_origin(to_geo(origin_state_)))
        {
            return;
        }
    }

    ego_swarm::Vector3 position{};
    if (!shared_obstacle_frame_.to_local_ned(to_geo(own_state_), position))
    {
        return;
    }
    sensor_pose_.position_ned = {
        static_cast<float>(position.at(0)),
        static_cast<float>(position.at(1)),
        static_cast<float>(position.at(2))};
    has_sensor_pose_ = true;
}

void ObstacleCloudSimulatorNode::publish_cloud_cb()
{
    if (cloud_pub_ == nullptr)
    {
        return;
    }
    if (generation_config_.mode == perception::CloudMode::SyntheticSensor &&
        !has_sensor_pose_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Synthetic sensor is waiting for the current UAV pose");
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

int ObstacleCloudSimulatorNode::compute_agent_index() const
{
    const std::size_t underscore = agent_namespace_.find('_');
    if (underscore == std::string::npos ||
        agent_namespace_.substr(0, 3) != "fmu")
    {
        return 1;
    }
    try
    {
        return std::max(1, std::stoi(agent_namespace_.substr(
            3, underscore - 3)));
    }
    catch (const std::exception& error)
    {
        RCLCPP_WARN(get_logger(), "Cannot parse agent index: %s",
                    error.what());
        return 1;
    }
}

}  // namespace nodes
}  // namespace taiphoon
