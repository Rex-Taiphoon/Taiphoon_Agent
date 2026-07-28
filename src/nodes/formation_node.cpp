/**
 * @file formation_node.cpp
 * @brief 編隊節點實作
 * @author Taiphoon Agent Team
 * @date 2026-07-28
 */

#include "nodes/formation_node.hpp"
#include <cmath>
#include <sstream>
#include <algorithm>
#include <limits>

namespace taiphoon
{
namespace nodes
{

// =============================================================================
// 建構子
// =============================================================================

FormationNode::FormationNode()
    : rclcpp::Node("formation_node")
{
    // 宣告 ROS2 參數
    this->declare_parameter<double>("drone_count", DEFAULT_DRONE_COUNT);
    this->declare_parameter<std::string>("agent_namespace", "fmu1_agent");
    this->declare_parameter<double>("formation_update_rate_hz",
                                    DEFAULT_FORMATION_RATE_HZ);

    drone_count_ = static_cast<int>(this->get_parameter("drone_count").as_double());
    agent_namespace_ = this->get_parameter("agent_namespace").as_string();
    formation_update_rate_hz_ =
        this->get_parameter("formation_update_rate_hz").as_double();

    // 從 agent_namespace 推算自身索引: "fmu1_agent" → 0, "fmu2_agent" → 1, ...
    if (agent_namespace_.size() >= 4 &&
        agent_namespace_.substr(0, 3) == "fmu")
    {
        try
        {
            size_t end_pos = agent_namespace_.find('_');
            std::string num_str = (end_pos != std::string::npos)
                ? agent_namespace_.substr(3, end_pos - 3)
                : agent_namespace_.substr(3);
            self_index_ = std::stoi(num_str) - 1;
            if (self_index_ < 0) self_index_ = 0;
        }
        catch (...)
        {
            self_index_ = 0;
        }
    }

    RCLCPP_INFO(this->get_logger(),
                "FormationNode: drone_count=%d, agent_namespace=%s, "
                "self_index=%d, update_rate=%.1f Hz",
                drone_count_, agent_namespace_.c_str(),
                self_index_, formation_update_rate_hz_);

    // 預先分配 drone_states_
    drone_states_.resize(static_cast<size_t>(drone_count_));
    auto now = this->now();
    for (auto& ds : drone_states_)
    {
        ds.last_update = now;
        ds.online = false;
    }

    // --- 初始化各組件 ---
    if (!initialize_odometry_subscribers())
    {
        RCLCPP_ERROR(this->get_logger(), "Odometry subscribers init failed");
        return;
    }

    if (!initialize_command_subscriber())
    {
        RCLCPP_ERROR(this->get_logger(), "Command subscriber init failed");
        return;
    }

    if (!initialize_ned_target_publisher())
    {
        RCLCPP_ERROR(this->get_logger(), "NED target publisher init failed");
        return;
    }

    // --- 建立編隊控制迴圈計時器 ---
    auto period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 / formation_update_rate_hz_));
    formation_loop_timer_ = this->create_wall_timer(
        period_ms,
        std::bind(&FormationNode::formation_loop_cb, this));

    RCLCPP_INFO(this->get_logger(),
                "FormationNode initialized successfully");
}


// =============================================================================
// 初始化
// =============================================================================

bool FormationNode::initialize_odometry_subscribers()
{
    odometry_subs_.resize(static_cast<size_t>(drone_count_));

    for (int i = 1; i <= drone_count_; ++i)
    {
        std::string topic =
            "/fmu" + std::to_string(i) + "/fmu/out/vehicle_odometry";
        size_t idx = static_cast<size_t>(i - 1);

        odometry_subs_[idx] =
            this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                topic,
                rclcpp::QoS(10).best_effort(),
                [this, idx](const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
                {
                    this->odometry_cb(idx, msg);
                });

        if (odometry_subs_[idx] == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to subscribe to %s", topic.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Subscribed to %s", topic.c_str());
    }
    return true;
}

bool FormationNode::initialize_command_subscriber()
{
    all_agent_command_sub_ =
        this->create_subscription<std_msgs::msg::String>(
            "/fmu_all_agent/command",
            rclcpp::QoS(10).best_effort(),
            std::bind(&FormationNode::all_agent_command_cb,
                      this, std::placeholders::_1));

    if (all_agent_command_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to subscribe to /fmu_all_agent/command");
        return false;
    }
    return true;
}

bool FormationNode::initialize_ned_target_publisher()
{
    std::string topic = "/" + agent_namespace_ + "/ned_target";

    ned_target_pub_ =
        this->create_publisher<taiphoon_agent::msg::NedTarget>(
            topic, rclcpp::QoS(5).best_effort());

    if (ned_target_pub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create publisher on %s", topic.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Publishing NED targets to %s", topic.c_str());
    return true;
}


// =============================================================================
// ROS2 回呼
// =============================================================================

void FormationNode::odometry_cb(
    size_t drone_index,
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    if (msg == nullptr) return;

    if (drone_index >= drone_states_.size()) return;

    auto& ds = drone_states_[drone_index];
    ds.position_ned[0] = msg->position[0];
    ds.position_ned[1] = msg->position[1];
    ds.position_ned[2] = msg->position[2];
    ds.last_update = this->now();
    ds.online = true;
}

void FormationNode::all_agent_command_cb(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (msg == nullptr) return;

    const std::string& cmd_str = msg->data;
    utils::ParsedCommand parsed = utils::parse_command_string(cmd_str);

    switch (parsed.command)
    {
        case utils::AgentCommand::FORMATION_CIRCLE:
            if (parsed.has_multi_params)
            {
                formation_type_ = FormationType::CIRCLE;
                formation_param1_ = parsed.param_value;  // 半徑
                formation_param2_ = parsed.param2;        // 高度
                // 重置上次發布目標，確保新編隊立即生效
                last_published_target_[3] = std::numeric_limits<float>::quiet_NaN();
                RCLCPP_INFO(this->get_logger(),
                            "Formation: CIRCLE radius=%.2f m altitude=%.2f m",
                            static_cast<double>(formation_param1_),
                            static_cast<double>(formation_param2_));
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "FORMATION_CIRCLE requires radius and altitude");
            }
            break;

        case utils::AgentCommand::FORMATION_HORI_LINEAR:
            if (parsed.has_multi_params)
            {
                formation_type_ = FormationType::HORI_LINEAR;
                formation_param1_ = parsed.param_value;  // 間距
                formation_param2_ = parsed.param2;        // 高度
                // 重置上次發布目標，確保新編隊立即生效
                last_published_target_[3] = std::numeric_limits<float>::quiet_NaN();
                RCLCPP_INFO(this->get_logger(),
                            "Formation: HORI_LINEAR spacing=%.2f m altitude=%.2f m",
                            static_cast<double>(formation_param1_),
                            static_cast<double>(formation_param2_));
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "FORMATION_HORI_LINEAR requires spacing and altitude");
            }
            break;

        case utils::AgentCommand::FORMATION_VERT_LINEAR:
            if (parsed.has_multi_params)
            {
                formation_type_ = FormationType::VERT_LINEAR;
                formation_param1_ = parsed.param_value;  // 間距
                formation_param2_ = 0.0F;
                // 重置上次發布目標，確保新編隊立即生效
                last_published_target_[3] = std::numeric_limits<float>::quiet_NaN();
                RCLCPP_INFO(this->get_logger(),
                            "Formation: VERT_LINEAR spacing=%.2f m",
                            static_cast<double>(formation_param1_));
            }
            else
            {
                formation_param1_ = parsed.param_value;
                formation_param2_ = 0.0F;
                RCLCPP_INFO(this->get_logger(),
                            "Formation: VERT_LINEAR spacing=%.2f m",
                            static_cast<double>(formation_param1_));
                formation_type_ = FormationType::VERT_LINEAR;
            }
            break;

        case utils::AgentCommand::FORMATION_CANCEL:
            formation_type_ = FormationType::NONE;
            formation_param1_ = 0.0F;
            formation_param2_ = 0.0F;
            RCLCPP_INFO(this->get_logger(), "Formation cancelled");
            break;

        default:
            // 非編隊命令，忽略
            break;
    }
}


// =============================================================================
// 編隊控制迴圈
// =============================================================================

void FormationNode::formation_loop_cb()
{
    // 沒有啟用編隊則不發布
    if (formation_type_ == FormationType::NONE) return;

    // 檢查是否有任何線上無人機
    size_t online_count = get_online_count();
    if (online_count == 0)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No drones online, skipping formation update");
        return;
    }

    // 計算質心
    auto centroid = compute_centroid();

    // 計算自身目標位置
    auto target = compute_formation_target(centroid);

    // 變化檢測：避免重複發布相同目標
    float pos_dx = target[0] - last_published_target_[0];
    float pos_dy = target[1] - last_published_target_[1];
    float pos_dz = target[2] - last_published_target_[2];
    float pos_dist = std::sqrt(pos_dx * pos_dx + pos_dy * pos_dy + pos_dz * pos_dz);

    float yaw_diff = 0.0F;
    if (!std::isnan(target[3]) && !std::isnan(last_published_target_[3]))
    {
        yaw_diff = std::abs(target[3] - last_published_target_[3]);
    }
    else if (!std::isnan(target[3]) || !std::isnan(last_published_target_[3]))
    {
        // 一方有 yaw 一方沒有，需要發布
        yaw_diff = YAW_CHANGE_THRESHOLD_DEG + 1.0F;
    }

    if (pos_dist < POSITION_CHANGE_THRESHOLD_M &&
        yaw_diff < YAW_CHANGE_THRESHOLD_DEG)
    {
        return;  // 變化太小，不發布
    }

    // 發布 NED 目標
    auto ned_msg = taiphoon_agent::msg::NedTarget();
    ned_msg.x = target[0];
    ned_msg.y = target[1];
    ned_msg.z = target[2];
    ned_msg.yaw = target[3];

    ned_target_pub_->publish(ned_msg);
    last_published_target_ = target;
}


// =============================================================================
// 編隊計算
// =============================================================================

std::array<float, 3> FormationNode::compute_centroid() const
{
    std::array<float, 3> centroid{0.0F, 0.0F, 0.0F};
    size_t count = 0;

    for (const auto& ds : drone_states_)
    {
        if (!ds.online) continue;
        centroid[0] += ds.position_ned[0];
        centroid[1] += ds.position_ned[1];
        centroid[2] += ds.position_ned[2];
        ++count;
    }

    if (count > 0)
    {
        centroid[0] /= static_cast<float>(count);
        centroid[1] /= static_cast<float>(count);
        centroid[2] /= static_cast<float>(count);
    }

    return centroid;
}

std::array<float, 4> FormationNode::compute_formation_target(
    const std::array<float, 3>& centroid) const
{
    std::array<float, 4> target{centroid[0], centroid[1], centroid[2],
        std::numeric_limits<float>::quiet_NaN()};

    size_t online_count = get_online_count();
    if (online_count == 0) return target;

    constexpr float PI = 3.14159265358979323846F;

    switch (formation_type_)
    {
        case FormationType::CIRCLE:
        {
            // 環形編隊：等角分布
            float angle = 2.0F * PI *
                static_cast<float>(self_index_) / static_cast<float>(online_count);
            float radius = formation_param1_;
            target[0] = centroid[0] + radius * std::cos(angle);
            target[1] = centroid[1] + radius * std::sin(angle);
            target[2] = -formation_param2_;  // NED: Z 向下為正，高度取負
            // 不強制設定偏航，由外部節點或操作員決定
            // target[3] 保持 NaN = 不控制偏航
            break;
        }

        case FormationType::HORI_LINEAR:
        {
            // 水平直列：沿 Y 軸排列
            float spacing = formation_param1_;
            float mid = (static_cast<float>(online_count) - 1.0F) / 2.0F;
            float offset = spacing *
                (static_cast<float>(self_index_) - mid);
            target[0] = centroid[0];
            target[1] = centroid[1] + offset;
            target[2] = -formation_param2_;
            // target[3] 保持 NaN = 不控制偏航
            break;
        }

        case FormationType::VERT_LINEAR:
        {
            // 垂直直列：沿 Z 軸排列
            float spacing = formation_param1_;
            float mid = (static_cast<float>(online_count) - 1.0F) / 2.0F;
            float offset = spacing *
                (static_cast<float>(self_index_) - mid);
            target[0] = centroid[0];
            target[1] = centroid[1];
            target[2] = centroid[2] + offset;
            // target[3] 保持 NaN = 不控制偏航
            break;
        }

        default:
            break;
    }

    return target;
}

size_t FormationNode::get_online_count() const
{
    auto now = this->now();
    size_t count = 0;
    for (const auto& ds : drone_states_)
    {
        if (!ds.online) continue;
        double elapsed = (now - ds.last_update).seconds();
        if (elapsed > DRONE_TIMEOUT_S) continue;
        ++count;
    }
    return count;
}

}  // namespace nodes
}  // namespace taiphoon
