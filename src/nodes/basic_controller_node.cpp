/**
 * @file basic_controller_node.cpp
 * @brief BasicController ROS2 節點實作
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 */

#include "nodes/basic_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace taiphoon
{
namespace nodes
{

namespace
{
constexpr double RATE_COMPARISON_EPSILON = 1.0E-6;
constexpr char VEHICLE_GLOBAL_POSITION_TOPIC[] =
    "fmu/out/vehicle_global_position";

/// COMMAND_UNKNOWN 事件 detail 保留的原始命令長度上限
constexpr std::size_t MAX_UNKNOWN_COMMAND_DETAIL_LENGTH = 64U;

/// 健康狀態與位置逾時使用單調時鐘，與 BasicController 的 PX4 新鮮度一致。
int64_t steady_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

/** @brief 驗證三維向量的所有分量均為有限值。 */
bool is_finite_vector3(const std::array<float, 3>& value)
{
    return std::isfinite(value[0]) &&
        std::isfinite(value[1]) &&
        std::isfinite(value[2]);
}
}  // namespace

BasicControllerNode::BasicControllerNode()
    : rclcpp::Node("basic_controller_node")
{
    // L2-19：宣告 ROS2 參數並指定預設值
    this->declare_parameter<double>("target_altitude_m", DEFAULT_TARGET_ALTITUDE_M);
    this->declare_parameter<double>("hover_duration_s", DEFAULT_HOVER_DURATION_S);
    this->declare_parameter<int>("target_system", 1);
    this->declare_parameter<std::string>("agent_namespace", "uav_1_agent");
    this->declare_parameter<double>("state_publish_rate_hz", DEFAULT_STATE_PUBLISH_RATE_HZ);
    this->declare_parameter<double>("position_deviation_threshold_m",
                                    DEFAULT_POSITION_DEVIATION_THRESHOLD_M);
    this->declare_parameter<double>("yaw_rate_limit_deg_s",
                                    controllers::DEFAULT_YAW_RATE_LIMIT_DEG_S);
    this->declare_parameter<double>("health_publish_rate_hz",
                                    DEFAULT_HEALTH_PUBLISH_RATE_HZ);
    this->declare_parameter<double>("px4_command_ack_timeout_s",
                                    controllers::DEFAULT_PX4_COMMAND_ACK_TIMEOUT_S);
    this->declare_parameter<double>("reboot_cooldown_s",
                                    controllers::DEFAULT_REBOOT_COOLDOWN_S);

    target_altitude_m_ = this->get_parameter("target_altitude_m").as_double();
    hover_duration_s_ = this->get_parameter("hover_duration_s").as_double();
    target_system_ = this->get_parameter("target_system").as_int();
    agent_namespace_ = this->get_parameter("agent_namespace").as_string();
    const double requested_state_publish_rate_hz =
        this->get_parameter("state_publish_rate_hz").as_double();
    state_publish_rate_hz_ = safety::StatePublisher::normalize_publish_rate_hz(
        requested_state_publish_rate_hz);
    if (!std::isfinite(requested_state_publish_rate_hz) ||
        std::abs(requested_state_publish_rate_hz - state_publish_rate_hz_) >
            RATE_COMPARISON_EPSILON)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Requested state publish rate %.3f Hz is invalid or below "
                    "10 Hz; using %.1f Hz",
                    requested_state_publish_rate_hz, state_publish_rate_hz_);
    }
    position_deviation_threshold_m_ =
        this->get_parameter("position_deviation_threshold_m").as_double();
    RCLCPP_INFO(this->get_logger(),
                "Parameters: target_altitude=%.2f m, hover_duration=%.2f s, "
                "target_system=%d, agent_namespace=%s, "
                "state_publish_rate=%.1f Hz, deviation_threshold=%.2f m",
                target_altitude_m_, hover_duration_s_,
                target_system_,
                agent_namespace_.c_str(),
                state_publish_rate_hz_, position_deviation_threshold_m_);

    // --- 建立飛行控制器 ---
    controller_ = std::make_unique<controllers::BasicController>(
        this, static_cast<uint8_t>(target_system_));

    if (controller_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Controller creation failed");
        return;
    }

    if (!controller_->initialize())
    {
        RCLCPP_ERROR(this->get_logger(), "Controller init failed");
        return;
    }

    // --- 偏航角速率上限：PX4 Offboard 不套用 MPC_YAWRAUTO_MAX，由 Agent 限速 ---
    const double requested_yaw_rate_deg_s =
        this->get_parameter("yaw_rate_limit_deg_s").as_double();
    double yaw_rate_limit_deg_s = controllers::DEFAULT_YAW_RATE_LIMIT_DEG_S;
    if (controllers::normalize_yaw_rate_limit_deg_s(
            requested_yaw_rate_deg_s, yaw_rate_limit_deg_s) !=
        controllers::YawRateLimitStatus::ACCEPTED)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Requested yaw rate limit %.3f deg/s is invalid or outside "
                    "[%.1f, %.1f]; using %.1f deg/s",
                    requested_yaw_rate_deg_s,
                    controllers::MIN_YAW_RATE_LIMIT_DEG_S,
                    controllers::MAX_YAW_RATE_LIMIT_DEG_S,
                    yaw_rate_limit_deg_s);
    }
    if (!controller_->set_yaw_rate_limit(
            static_cast<float>(yaw_rate_limit_deg_s),
            static_cast<float>(1.0 / CONTROL_LOOP_RATE_HZ)))
    {
        RCLCPP_ERROR(this->get_logger(), "Yaw rate limit configuration failed");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Yaw rate limit %.1f deg/s",
                yaw_rate_limit_deg_s);

    // --- 事件與健康狀態：控制器事件經由回呼交回節點發布 ---
    const double requested_health_rate_hz =
        this->get_parameter("health_publish_rate_hz").as_double();
    health_publish_rate_hz_ = requested_health_rate_hz;
    if (!std::isfinite(requested_health_rate_hz) ||
        requested_health_rate_hz <= 0.0 ||
        requested_health_rate_hz > CONTROL_LOOP_RATE_HZ)
    {
        health_publish_rate_hz_ = DEFAULT_HEALTH_PUBLISH_RATE_HZ;
        RCLCPP_WARN(this->get_logger(),
                    "Requested health publish rate %.3f Hz is invalid; "
                    "using %.1f Hz",
                    requested_health_rate_hz, health_publish_rate_hz_);
    }
    const double requested_ack_timeout_s =
        this->get_parameter("px4_command_ack_timeout_s").as_double();
    if (!controller_->set_command_ack_timeout_s(requested_ack_timeout_s))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Requested PX4 command ack timeout %.3f s is invalid; "
                    "using %.1f s",
                    requested_ack_timeout_s,
                    controllers::DEFAULT_PX4_COMMAND_ACK_TIMEOUT_S);
    }
    const double requested_reboot_cooldown_s =
        this->get_parameter("reboot_cooldown_s").as_double();
    if (!controller_->set_reboot_cooldown_s(requested_reboot_cooldown_s))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Requested reboot cooldown %.3f s is invalid; using %.1f s",
                    requested_reboot_cooldown_s,
                    controllers::DEFAULT_REBOOT_COOLDOWN_S);
    }
    if (!initialize_event_and_health_publishers())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Event and health publishers init failed");
        return;
    }
    controller_->set_event_callback(
        std::bind(&BasicControllerNode::on_controller_event, this,
                  std::placeholders::_1));

    // --- 初始化位置安全監控器 ---
    position_safety_monitor_ = safety::PositionSafetyMonitor(
        static_cast<float>(position_deviation_threshold_m_));
    // --- 初始化里程計訂閱者 ---
    if (!initialize_odometry_subscriber())
    {
        RCLCPP_ERROR(this->get_logger(), "Odometry subscriber init failed");
        return;
    }

    // --- 初始化 GPS 訂閱者 ---
    if (!initialize_gps_subscriber())
    {
        RCLCPP_ERROR(this->get_logger(), "GPS subscriber init failed");
        return;
    }

    // --- 初始化電池狀態訂閱者 ---
    if (!initialize_battery_subscriber())
    {
        RCLCPP_ERROR(this->get_logger(), "Battery subscriber init failed");
        return;
    }

    // --- 初始化 NED 目標訂閱者 ---
    if (!initialize_ned_target_subscriber())
    {
        RCLCPP_ERROR(this->get_logger(), "NED target subscriber init failed");
        return;
    }

    if (!initialize_agent_trajectory_subscriber())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "AgentTrajectory subscriber init failed");
        return;
    }

    if (!initialize_mission_intent_publisher())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Mission intent publisher init failed");
        return;
    }

    // --- 初始化命令處理器 ---
    if (!initialize_agent_command_handler())
    {
        RCLCPP_ERROR(this->get_logger(), "Agent command handler init failed");
        return;
    }

    // --- 初始化 Agent 狀態發布者 ---
    if (!initialize_state_publishers())
    {
        RCLCPP_ERROR(this->get_logger(), "State publishers init failed");
        return;
    }

    // --- 建立控制迴圈計時器 ---
    auto ctrl_period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 / CONTROL_LOOP_RATE_HZ));

    control_loop_timer_ = this->create_wall_timer(
        ctrl_period_ms,
        std::bind(&BasicControllerNode::control_loop_cb, this));

    // --- 建立狀態發布計時器 ---
    auto state_period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 / state_publish_rate_hz_));

    state_publish_timer_ = this->create_wall_timer(
        state_period_ms,
        std::bind(&BasicControllerNode::state_publish_cb, this));

    // --- 建立健康狀態發布計時器：PX4 完全沒有資料時也照常發布 ---
    auto health_period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 / health_publish_rate_hz_));

    health_publish_timer_ = this->create_wall_timer(
        health_period_ms,
        std::bind(&BasicControllerNode::health_publish_cb, this));

    RCLCPP_INFO(this->get_logger(),
                "BasicControllerNode initialized, control rate %.0f Hz, "
                "state publish rate %.1f Hz, health publish rate %.1f Hz",
                CONTROL_LOOP_RATE_HZ, state_publish_rate_hz_,
                health_publish_rate_hz_);
}

bool BasicControllerNode::initialize_event_and_health_publishers()
{
    const std::string events_topic = "/" + agent_namespace_ + "/events";
    const std::string health_topic = "/" + agent_namespace_ + "/health";

    // 事件不重播：GCS 連上前的舊事件可能被誤認為新錯誤，持續性問題由 health 呈現。
    events_pub_ = this->create_publisher<taiphoon_agent::msg::AgentEvent>(
        events_topic, rclcpp::QoS(safety::AGENT_EVENT_QOS_DEPTH).reliable().durability_volatile());
    if (events_pub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create AgentEvent publisher on '%s'",
                     events_topic.c_str());
        return false;
    }

    health_pub_ = this->create_publisher<taiphoon_agent::msg::AgentHealth>(
        health_topic, rclcpp::QoS(AGENT_HEALTH_QOS_DEPTH).reliable().durability_volatile());
    if (health_pub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create AgentHealth publisher on '%s'",
                     health_topic.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Event and health publishers created: %s, %s",
                events_topic.c_str(), health_topic.c_str());
    return true;
}

bool BasicControllerNode::initialize_odometry_subscriber()
{
    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "fmu/out/vehicle_odometry",
        rclcpp::QoS(10).best_effort(),
        std::bind(&BasicControllerNode::odometry_cb, this,
                  std::placeholders::_1));

    if (odometry_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create odometry subscriber");
        return false;
    }

    return true;
}

bool BasicControllerNode::initialize_gps_subscriber()
{
    // 節點 namespace 為 /uav_<N>，此相對名稱解析成
    // /uav_<N>/fmu/out/vehicle_global_position。
    gps_sub_ =
        this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            VEHICLE_GLOBAL_POSITION_TOPIC,
            rclcpp::QoS(10).best_effort(),
            std::bind(&BasicControllerNode::gps_cb, this,
                      std::placeholders::_1));

    if (gps_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create GPS subscriber");
        return false;
    }

    return true;
}

bool BasicControllerNode::initialize_battery_subscriber()
{
    battery_sub_ =
        this->create_subscription<px4_msgs::msg::BatteryStatus>(
            "fmu/out/battery_status_v1",
            rclcpp::QoS(10).best_effort(),
            std::bind(&BasicControllerNode::battery_cb, this,
                      std::placeholders::_1));

    if (battery_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create battery subscriber");
        return false;
    }

    return true;
}

bool BasicControllerNode::initialize_ned_target_subscriber()
{
    std::string ned_topic = "/" + agent_namespace_ + "/ned_target";
    ned_target_sub_ = this->create_subscription<taiphoon_agent::msg::NedTarget>(
        ned_topic,
        rclcpp::QoS(5).best_effort(),
        std::bind(&BasicControllerNode::ned_target_cb, this,
                  std::placeholders::_1));

    if (ned_target_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create NED target subscriber on '%s'",
                     ned_topic.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Subscribed to NED target topic: %s", ned_topic.c_str());
    return true;
}

bool BasicControllerNode::initialize_agent_trajectory_subscriber()
{
    const std::string agent_trajectory_topic =
        "/" + agent_namespace_ + "/planning/trajectory";

    agent_trajectory_sub_ =
        this->create_subscription<taiphoon_agent::msg::AgentTrajectory>(
            agent_trajectory_topic,
            rclcpp::QoS(10).best_effort(),
            std::bind(&BasicControllerNode::agent_trajectory_cb, this,
                      std::placeholders::_1));

    if (agent_trajectory_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create AgentTrajectory subscriber on '%s'",
                     agent_trajectory_topic.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Subscribed to EGO-Swarm trajectory topic: %s",
                agent_trajectory_topic.c_str());
    return true;
}

bool BasicControllerNode::initialize_mission_intent_publisher()
{
    const std::string topic = "/" + agent_namespace_ +
        "/planning/mission_intent";
    mission_intent_pub_ = this->create_publisher<std_msgs::msg::String>(
        topic, rclcpp::QoS(10));
    if (mission_intent_pub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create mission intent publisher on '%s'",
                     topic.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Mission intent publisher: %s", topic.c_str());
    return true;
}

bool BasicControllerNode::initialize_agent_command_handler()
{
    agent_cmd_handler_ =
        std::make_unique<controllers::AgentCommandHandler>(
            this, agent_namespace_);

    if (agent_cmd_handler_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create AgentCommandHandler");
        return false;
    }

    agent_cmd_handler_->set_sourced_command_callback(
        std::bind(&BasicControllerNode::on_agent_command, this,
                  std::placeholders::_1, std::placeholders::_2));
    agent_cmd_handler_->set_unknown_command_callback(
        std::bind(&BasicControllerNode::on_unknown_command, this,
                  std::placeholders::_1, std::placeholders::_2));

    // 初始化命令處理器 (建立 Subscriber)
    if (!agent_cmd_handler_->initialize())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "AgentCommandHandler initialization failed");
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "AgentCommandHandler initialized for namespace '%s'",
                agent_namespace_.c_str());
    return true;
}

bool BasicControllerNode::initialize_state_publishers()
{
    std::string state_topic = "/" + agent_namespace_ + "/state";
    std::string attitude_topic = "/" + agent_namespace_ + "/attitude";

    agent_state_pub_ =
        this->create_publisher<taiphoon_agent::msg::AgentState>(
            state_topic, rclcpp::QoS(10));

    if (agent_state_pub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create AgentState publisher on '%s'",
                     state_topic.c_str());
        return false;
    }

    agent_attitude_pub_ =
        this->create_publisher<taiphoon_agent::msg::AgentAttitude>(
            attitude_topic, rclcpp::QoS(10));

    if (agent_attitude_pub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create AgentAttitude publisher on '%s'",
                     attitude_topic.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "State publishers created: %s, %s",
                state_topic.c_str(), attitude_topic.c_str());
    return true;
}

// =============================================================================
// 回呼實作
// =============================================================================

void BasicControllerNode::odometry_cb(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "odometry_cb received nullptr message, ignoring");
        return;
    }

    latest_odometry_ = *msg;
    has_odometry_data_ = true;
    last_odometry_ns_ = steady_now_ns();

    if (controller_ != nullptr)
    {
        controller_->update_odometry(*msg);
    }
}

void BasicControllerNode::gps_cb(
    const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "gps_cb received nullptr message, ignoring");
        return;
    }

    latest_gps_ = *msg;
    has_gps_data_ = true;
    last_global_position_ns_ = steady_now_ns();
}

void BasicControllerNode::battery_cb(
    const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "battery_cb received nullptr message, ignoring");
        return;
    }

    latest_battery_ = *msg;
    has_battery_data_ = true;
    last_battery_ns_ = steady_now_ns();
}

void BasicControllerNode::control_loop_cb()
{
    if (controller_ == nullptr)
    {
        return;
    }

    // BasicController 是唯一的 PX4 setpoint publisher；舊 CollisionTargetGate
    // 已移除，不得在控制週期覆寫 EGO-Swarm 的取樣結果。

//     // --- 位置安全監控：檢查實際位置與目標位置偏差 ---
//     {
//         auto current_pos = controller_->get_current_position_ned();
//         auto target_pos = controller_->get_target_position_ned();
//
//         // TAKEOFF 期間不檢查位置偏差（爬升為有意行為）
//         if (controller_->get_flight_state() != FlightState::TAKEOFF &&
//             position_safety_monitor_.is_deviation_exceeded(
//                 current_pos, target_pos))
//         {
//             float dist = safety::PositionSafetyMonitor::compute_distance(
//                 current_pos, target_pos);
//
//             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
//                                  3000,
//                                  "Position deviation %.2f m exceeds threshold "
//                                  "%.2f m — triggering hover intervention",
//                                  static_cast<double>(dist),
//                                  position_deviation_threshold_m_);
//
//             // 觸發 hover 介入：重設目標為目前位置
//             controller_->set_ned_target(
//                 current_pos[0], current_pos[1], current_pos[2],
//                 std::numeric_limits<float>::quiet_NaN());
//         }
//     }
//
    // 驅動控制命令發布
    controller_->publish_control_commands();

    update_position_link_events();
}

void BasicControllerNode::update_position_link_events()
{
    const int64_t now_ns = steady_now_ns();
    const auto report_transition =
        [this](const controllers::Px4LinkTransition transition,
               const char* source_name)
        {
            if (transition != controllers::Px4LinkTransition::BECAME_STALE &&
                transition != controllers::Px4LinkTransition::NEVER_RECEIVED)
            {
                return;
            }
            std::string detail = std::string(source_name) +
                (transition == controllers::Px4LinkTransition::NEVER_RECEIVED ?
                     " never received" :
                     " not received for more than " +
                         std::to_string(controllers::PX4_POSITION_TIMEOUT_MS) +
                         " ms");
            RCLCPP_WARN(this->get_logger(), "%s", detail.c_str());
            safety::AgentEventRecord event;
            event.code = safety::AgentEventCode::PX4_POSITION_STALE;
            event.detail = detail;
            // 兩個資料來源各自節流，避免其中一個逾時遮住另一個。
            publish_event(event, source_name);
        };

    report_transition(
        odometry_link_monitor_.update(last_odometry_ns_, now_ns), "VehicleOdometry");
    report_transition(
        global_position_link_monitor_.update(last_global_position_ns_, now_ns),
        "VehicleGlobalPosition");
}

void BasicControllerNode::health_publish_cb()
{
    if (controller_ == nullptr || health_pub_ == nullptr)
    {
        return;
    }

    const int64_t now_ns = steady_now_ns();
    const auto snapshot = controller_->get_px4_health_snapshot();

    taiphoon_agent::msg::AgentHealth msg;
    msg.stamp = this->now();
    msg.px4_status_age_s = static_cast<float>(
        controllers::px4_message_age_s(snapshot.status_received_ns, now_ns));
    msg.px4_odometry_age_s = static_cast<float>(
        controllers::px4_message_age_s(last_odometry_ns_, now_ns));
    msg.px4_global_position_age_s = static_cast<float>(
        controllers::px4_message_age_s(last_global_position_ns_, now_ns));
    msg.px4_land_detected_age_s = static_cast<float>(
        controllers::px4_message_age_s(snapshot.land_detected_received_ns, now_ns));
    msg.px4_battery_age_s = static_cast<float>(
        controllers::px4_message_age_s(last_battery_ns_, now_ns));
    msg.px4_command_ack_age_s = static_cast<float>(
        controllers::px4_message_age_s(snapshot.command_ack_received_ns, now_ns));
    msg.px4_pre_flight_checks_pass = snapshot.pre_flight_checks_pass;
    msg.px4_failsafe = snapshot.failsafe;
    msg.px4_arming_state = snapshot.arming_state;
    msg.px4_latest_arming_reason = snapshot.latest_arming_reason;
    msg.failsafe_flags_valid = controllers::is_px4_message_fresh(
        snapshot.failsafe_flags_received_ns, now_ns,
        controllers::PX4_STATE_TIMEOUT_NS);
    msg.failsafe_flags = msg.failsafe_flags_valid ? snapshot.failsafe_flag_bits : 0U;
    msg.offboard_session_active = snapshot.offboard_session_active;
    health_pub_->publish(msg);
}

void BasicControllerNode::on_controller_event(
    const safety::AgentEventRecord& event)
{
    if (event.code == safety::AgentEventCode::PX4_RESTART_DETECTED)
    {
        // PX4 重新啟動後 EKF 原點與 local NED 可能改變；下一筆軌跡重新建立
        // WGS84 → PX4 local 平移，避免沿用舊原點。
        has_trajectory_frame_ = false;
        std::lock_guard<std::mutex> lock(agent_trajectory_mutex_);
        has_latest_agent_trajectory_ = false;
    }
    publish_event(event);
}

void BasicControllerNode::publish_event(const safety::AgentEventRecord& event)
{
    // 節流鍵用命令名稱；沒有命令時同代碼共用一個鍵。
    publish_event(event, event.command);
}

void BasicControllerNode::publish_event(const safety::AgentEventRecord& event,
                                        const std::string& throttle_key)
{
    if (events_pub_ == nullptr)
    {
        return;
    }
    if (!event_gate_.should_publish(event.code, throttle_key, steady_now_ns()))
    {
        return;
    }

    taiphoon_agent::msg::AgentEvent msg;
    msg.stamp = this->now();
    msg.sequence = event_gate_.next_sequence();
    msg.severity = static_cast<uint8_t>(safety::agent_event_severity(event.code));
    msg.source = taiphoon_agent::msg::AgentEvent::SOURCE_BASIC_CONTROLLER;
    msg.code = static_cast<uint16_t>(event.code);
    msg.command = event.command;
    msg.px4_result = event.px4_result;
    msg.detail = event.detail;
    events_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Event %u %s [%s]: %s",
                static_cast<unsigned int>(msg.code),
                safety::agent_event_code_name(event.code),
                event.command.c_str(), event.detail.c_str());
}

void BasicControllerNode::publish_event(const safety::AgentEventCode code,
                                        const std::string& command,
                                        const std::string& detail)
{
    safety::AgentEventRecord event;
    event.code = code;
    event.command = command;
    event.detail = detail;
    publish_event(event);
}

void BasicControllerNode::on_unknown_command(
    const std::string& raw_command, const utils::CommandSource source)
{
    std::string shown = raw_command.substr(
        0U, std::min(raw_command.size(), MAX_UNKNOWN_COMMAND_DETAIL_LENGTH));
    const std::string detail = std::string("Unknown ") +
        (source == utils::CommandSource::BROADCAST ? "broadcast " : "") +
        "command: '" + shown + "'";
    publish_event(safety::AgentEventCode::COMMAND_UNKNOWN, "", detail);
}

bool BasicControllerNode::handle_reboot_command(const utils::ParsedCommand& cmd)
{
    const bool is_forced = cmd.has_param &&
        cmd.param_value == utils::REBOOT_FC_FORCE_ENABLED;
    const auto reboot_result = controller_->reboot_flight_controller(is_forced);
    if (!reboot_result.is_sent)
    {
        publish_event(safety::AgentEventCode::COMMAND_REJECTED, "REBOOT_FC",
                      reboot_result.detail);
    }
    return reboot_result.is_sent;
}

void BasicControllerNode::state_publish_cb()
{
    if (!has_odometry_data_)
    {
        return;
    }

    if (controller_ == nullptr)
    {
        return;
    }

    const px4_msgs::msg::VehicleGlobalPosition* gps_ptr =
        has_gps_data_ ? &latest_gps_ : nullptr;

    const px4_msgs::msg::BatteryStatus* battery_ptr =
        has_battery_data_ ? &latest_battery_ : nullptr;

    // --- 發布 AgentState ---
    {
        auto observation = controller_->get_flight_state_observation();
        taiphoon_agent::msg::AgentState state_msg;
        safety::StatePublisher::fill_agent_state(
            state_msg, latest_odometry_,
            controller_->get_flight_state(), true,
            gps_ptr, battery_ptr, &observation);
        agent_state_pub_->publish(state_msg);
    }

    // --- 發布 AgentAttitude ---
    {
        taiphoon_agent::msg::AgentAttitude attitude_msg;
        safety::StatePublisher::fill_agent_attitude(
            attitude_msg, latest_odometry_);
        agent_attitude_pub_->publish(attitude_msg);
    }
}

void BasicControllerNode::ned_target_cb(
    const taiphoon_agent::msg::NedTarget::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "ned_target_cb received nullptr, ignoring");
        return;
    }

    if (controller_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Controller not available, NED target ignored");
        return;
    }

    if (!std::isfinite(msg->x) || !std::isfinite(msg->y) ||
        !std::isfinite(msg->z) || std::isinf(msg->yaw))
    {
        RCLCPP_ERROR(this->get_logger(),
                     "NED target contains NaN or Inf, rejecting");
        return;
    }

    if (mission_intent_pub_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Mission intent publisher unavailable; NED target ignored");
        return;
    }

    std_msgs::msg::String intent;
    std::ostringstream stream;
    stream << "MOVE_POINT " << msg->x << " " << msg->y << " " << msg->z
           << " " << msg->yaw;
    intent.data = stream.str();
    mission_intent_pub_->publish(intent);
}

void BasicControllerNode::agent_trajectory_cb(
    const taiphoon_agent::msg::AgentTrajectory::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "agent_trajectory_cb received nullptr, ignoring");
        return;
    }

    if (msg->agent_index != static_cast<uint32_t>(target_system_))
    {
        return;
    }

    if (!msg->valid)
    {
        std::lock_guard<std::mutex> lock(agent_trajectory_mutex_);
        has_latest_agent_trajectory_ = false;
        RCLCPP_WARN(this->get_logger(),
                    "Rejecting invalid AgentTrajectory id=%u",
                    msg->trajectory_id);
        return;
    }

    if (controller_ == nullptr || !has_gps_data_ || !has_odometry_data_)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Rejecting AgentTrajectory id=%u: GPS or odometry is not ready",
            msg->trajectory_id);
        return;
    }

    const auto observation = controller_->get_flight_state_observation();
    if (!observation.is_px4_state_valid || !observation.is_armed ||
        observation.is_landed)
    {
        std::lock_guard<std::mutex> lock(agent_trajectory_mutex_);
        has_latest_agent_trajectory_ = false;
        return;
    }

    const std::array<float, 3> odometry_position{
        latest_odometry_.position[0],
        latest_odometry_.position[1],
        latest_odometry_.position[2]};
    const std::array<float, 3> current_local_ned =
        controller_->get_current_position_ned();
    if (!is_finite_vector3(odometry_position) ||
        !is_finite_vector3(current_local_ned))
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Rejecting AgentTrajectory id=%u: odometry position is invalid",
            msg->trajectory_id);
        return;
    }

    const ego_swarm::GeoCoordinate own_global{
        static_cast<double>(latest_gps_.lat),
        static_cast<double>(latest_gps_.lon),
        static_cast<double>(latest_gps_.alt)};
    if (!ego_swarm::LocalNedFrame::is_valid_geodetic(own_global) ||
        msg->coordinate_frame !=
            taiphoon_agent::msg::AgentTrajectory::
                COORDINATE_FRAME_WGS84_GEODETIC ||
        msg->frame_revision != ego_swarm::GLOBAL_WGS84_FRAME_REVISION ||
        !std::isfinite(msg->knot_interval_s) ||
        !std::isfinite(msg->duration_s))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Rejecting AgentTrajectory id=%u with invalid WGS84 "
                    "frame or trajectory fields",
                    msg->trajectory_id);
        return;
    }

    if (!has_trajectory_frame_)
    {
        if (!trajectory_frame_.set_origin(own_global))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejecting AgentTrajectory id=%u: local Home invalid",
                msg->trajectory_id);
            return;
        }
        trajectory_frame_to_px4_ned_offset_ = ego_swarm::Vector3{
            static_cast<double>(current_local_ned[0]),
            static_cast<double>(current_local_ned[1]),
            static_cast<double>(current_local_ned[2])};
        has_trajectory_frame_ = true;
    }

    controllers::TimedTrajectory trajectory;
    trajectory.stamp = rclcpp::Time(
        msg->stamp.sec, msg->stamp.nanosec, RCL_ROS_TIME);
    trajectory.start_time = rclcpp::Time(
        msg->start_time.sec, msg->start_time.nanosec, RCL_ROS_TIME);
    trajectory.trajectory_id = msg->trajectory_id;
    trajectory.agent_index = msg->agent_index;
    trajectory.frame_revision = msg->frame_revision;
    trajectory.order = msg->order;
    trajectory.knot_interval_s = msg->knot_interval_s;
    trajectory.duration_s = msg->duration_s;
    trajectory.valid = msg->valid;
    if (msg->control_points.empty() ||
        msg->control_points.size() % 3U != 0U)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Rejecting AgentTrajectory id=%u: control_points must be "
                    "a flattened XYZ sequence with three floats per point",
                    msg->trajectory_id);
        return;
    }
    trajectory.control_points.reserve(msg->control_points.size() / 3U);

    for (std::size_t index = 0U;
         index < msg->control_points.size(); index += 3U)
    {
        const ego_swarm::GeoCoordinate global_point{
            msg->control_points.at(index),
            msg->control_points.at(index + 1U),
            msg->control_points.at(index + 2U)};
        ego_swarm::Vector3 local_ned_point{};
        if (!ego_swarm::LocalNedFrame::is_valid_geodetic(global_point) ||
            !trajectory_frame_.to_local_ned(global_point, local_ned_point))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Rejecting AgentTrajectory id=%u: WGS84 point is invalid",
                        msg->trajectory_id);
            return;
        }

        const ego_swarm::Vector3 local_point{
            trajectory_frame_to_px4_ned_offset_.at(0) + local_ned_point.at(0),
            trajectory_frame_to_px4_ned_offset_.at(1) + local_ned_point.at(1),
            trajectory_frame_to_px4_ned_offset_.at(2) + local_ned_point.at(2)};
        if (!ego_swarm::is_finite_vector(local_point))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Rejecting AgentTrajectory id=%u: local conversion overflow",
                        msg->trajectory_id);
            return;
        }

        const std::array<float, 3> local_point_float{
            static_cast<float>(local_point[0]),
            static_cast<float>(local_point[1]),
            static_cast<float>(local_point[2])};
        if (!is_finite_vector3(local_point_float))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Rejecting AgentTrajectory id=%u: local conversion overflow",
                        msg->trajectory_id);
            return;
        }

        trajectory.control_points.push_back(local_point_float);
    }

    if (!controller_->update_timed_trajectory(trajectory))
    {
        // 非法欄位會保留控制器內原有的有效軌跡；frame revision 變更
        // 則控制器會清除舊軌跡，下一個控制週期輸出 hold。
        if (!controller_->has_timed_trajectory())
        {
            std::lock_guard<std::mutex> lock(agent_trajectory_mutex_);
            has_latest_agent_trajectory_ = false;
        }
        return;
    }

    {
        std::lock_guard<std::mutex> lock(agent_trajectory_mutex_);
        latest_agent_trajectory_ = trajectory;
        has_latest_agent_trajectory_ = true;
    }
}

void BasicControllerNode::on_agent_command(
    const utils::ParsedCommand& cmd, const utils::CommandSource source)
{
    if (controller_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Controller not available, command ignored");
        return;
    }

    const std::string command_name = utils::agent_command_to_string(cmd.command);

    // MISSION_*／REBOOT_FC 只接受單機 topic；全體 topic 的同名命令忽略並回報。
    if (source == utils::CommandSource::BROADCAST &&
        utils::is_agent_only_command(cmd.command))
    {
        RCLCPP_WARN(this->get_logger(),
                    "%s ignored on broadcast topic; send it to /%s/command",
                    command_name.c_str(), agent_namespace_.c_str());
        publish_event(safety::AgentEventCode::COMMAND_REJECTED, command_name,
                      command_name + " is only accepted on the agent command topic");
        return;
    }

    // 參數缺少或不合法時不執行（例如 TAKEOFF abc 不再以預設高度起飛）。
    if (!utils::has_valid_command_params(cmd))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Command '%s' has missing or invalid parameters; ignored",
                    command_name.c_str());
        publish_event(safety::AgentEventCode::COMMAND_INVALID_PARAMS, command_name,
                      "Missing or invalid parameters for " + command_name);
        return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Executing command: %s (param=%.2f, has_param=%d)",
                command_name.c_str(), cmd.param_value, cmd.has_param ? 1 : 0);

    bool result = false;
    bool is_failure_reported = false;

    switch (cmd.command)
    {
        case utils::AgentCommand::MODE_RTL:
            result = controller_->switch_to_rtl_mode();
            break;

        case utils::AgentCommand::MODE_LAND:
            result = controller_->switch_to_land_mode();
            break;

        case utils::AgentCommand::MODE_OFFBOARD:
            result = controller_->switch_to_offboard_mode();
            break;

        case utils::AgentCommand::MOVE_FORWARD:
        case utils::AgentCommand::MOVE_BACKWARD:
        case utils::AgentCommand::MOVE_LEFT:
        case utils::AgentCommand::MOVE_RIGHT:
        case utils::AgentCommand::MOVE_UP:
        case utils::AgentCommand::MOVE_DOWN:
            // MOVE 的唯一 command owner 是 EgoSwarmPlannerNode；此處
            // 只回報命令已交由 planner，避免 command callback 再產生
            // 第二份 mission intent。
            RCLCPP_INFO(this->get_logger(),
                        "%s delegated to EgoSwarmPlannerNode",
                        command_name.c_str());
            result = true;
            break;

        case utils::AgentCommand::ARM:
            result = controller_->arm();
            break;

        case utils::AgentCommand::DISARM:
            result = controller_->disarm();
            break;

        case utils::AgentCommand::TAKEOFF:
            {
                float alt = cmd.has_param
                    ? cmd.param_value
                    : static_cast<float>(target_altitude_m_);
                result = controller_->takeoff(alt);
            }
            break;

        case utils::AgentCommand::HOLD:
            result = controller_->hold_current_position();
            break;

        case utils::AgentCommand::YAW:
            // YAW 是 BasicController 的相對偏航命令，不是位置規劃
            // 目標；直接更新 PX4 TrajectorySetpoint 的 yaw。
            result = controller_->rotate_yaw(cmd.param_value);
            break;

        case utils::AgentCommand::MOVE_POINT:
            // MOVE_POINT 的位置交由 Planner 產生軌跡；yaw 是絕對
            // 航向，直接交由 BasicController 保留並發布給 PX4。
            // MOVE_POINT 本身由 command callback 直接送到 planner；
            // BasicController 只保留 yaw 設定，不轉發第二份 intent。
            result = controller_->set_absolute_yaw(cmd.param4);
            break;

        case utils::AgentCommand::FORMATION_CIRCLE:
        case utils::AgentCommand::FORMATION_HORI_LINEAR:
        case utils::AgentCommand::FORMATION_VERT_LINEAR:
        case utils::AgentCommand::FORMATION_V_SHAPE:
        case utils::AgentCommand::FORMATION_CANCEL:
            // 編隊命令由 EgoSwarmPlannerNode 唯一處理；BasicController
            // 不發布 PX4 setpoint，也不建立第二份 mission intent。
            RCLCPP_INFO(this->get_logger(),
                        "Formation command delegated to EgoSwarmPlannerNode");
            result = true;
            break;

        case utils::AgentCommand::MISSION_START:
        case utils::AgentCommand::MISSION_STOP:
            // 航點任務由 EgoSwarmPlannerNode 唯一執行，結果以 mission/status 回報。
            RCLCPP_INFO(this->get_logger(),
                        "%s delegated to EgoSwarmPlannerNode (mission_id=%u)",
                        command_name.c_str(),
                        static_cast<unsigned int>(cmd.mission_id));
            result = true;
            break;

        case utils::AgentCommand::REBOOT_FC:
            result = handle_reboot_command(cmd);
            // 拒絕原因已以 COMMAND_REJECTED 回報，不再追加 COMMAND_FAILED。
            is_failure_reported = !result;
            break;

        default:
            RCLCPP_WARN(this->get_logger(),
                       "Unhandled command type");
            break;
    }

    if (result)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Command '%s' executed successfully",
                    command_name.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),
                    "Command '%s' execution failed",
                    command_name.c_str());
        if (!is_failure_reported)
        {
            publish_event(safety::AgentEventCode::COMMAND_FAILED, command_name,
                          "Command '" + command_name + "' execution failed");
        }
    }
}

}  // namespace nodes
}  // namespace taiphoon
