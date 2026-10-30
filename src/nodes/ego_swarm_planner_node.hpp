/**
 * @file ego_swarm_planner_node.hpp
 * @brief EGO-Swarm ROS2 節點，組合定位、編隊、障礙物與同儕軌跡資料
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 */

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include "taiphoon_agent/msg/agent_event.hpp"
#include "taiphoon_agent/msg/agent_state.hpp"
#include "taiphoon_agent/msg/agent_attitude.hpp"
#include "taiphoon_agent/msg/agent_trajectory.hpp"
#include "taiphoon_agent/msg/formation_epoch.hpp"
#include "taiphoon_agent/msg/ned_target.hpp"
#include "taiphoon_agent/msg/planner_status.hpp"
#include "taiphoon_agent/msg/waypoint_mission.hpp"
#include "taiphoon_agent/msg/waypoint_mission_status.hpp"

#include "ego_swarm/ContinuousSafetyValidator.hpp"
#include "ego_swarm/FormationCenterPolicy.hpp"
#include "ego_swarm/FormationConvergence.hpp"
#include "ego_swarm/FormationEpochPolicy.hpp"
#include "ego_swarm/FormationReentryPolicy.hpp"
#include "ego_swarm/FlightMissionLifecycle.hpp"
#include "ego_swarm/FormationReservation.hpp"
#include "ego_swarm/EmergencyBrakePlanner.hpp"
#include "ego_swarm/FormationAssignment.hpp"
#include "ego_swarm/FormationSlotPlanner.hpp"
#include "ego_swarm/ITrajectoryPlanner.hpp"
#include "ego_swarm/LivePeerDistancePolicy.hpp"
#include "ego_swarm/MovementFrame.hpp"
#include "ego_swarm/OptimizedEgoPlanner.hpp"
#include "ego_swarm/PeerClockSkewMonitor.hpp"
#include "ego_swarm/PeerDeconflictionPlanner.hpp"
#include "ego_swarm/PeerTrajectoryTable.hpp"
#include "ego_swarm/PlannerAcceptancePolicy.hpp"
#include "ego_swarm/PlannerFeatureFlags.hpp"
#include "ego_swarm/PlanningTargetSelector.hpp"
#include "ego_swarm/PlannerTargetSemantics.hpp"
#include "ego_swarm/PositionArrival.hpp"
#include "ego_swarm/ReplanTrigger.hpp"
#include "ego_swarm/LocalNedFrame.hpp"
#include "ego_swarm/TrajectoryFactory.hpp"
#include "ego_swarm/TrajectoryReservationPolicy.hpp"
#include "ego_swarm/WaypointMissionExecutor.hpp"
#include "formation/PeerMatcher.hpp"
#include "perception/RawPointObstacleMap.hpp"
#include "safety/AgentEventCodes.hpp"
#include "utils/CommandTypes.hpp"

namespace taiphoon
{
namespace nodes
{

constexpr double DEFAULT_EGO_PLANNER_RATE_HZ = 10.0;
constexpr double DEFAULT_FORMATION_MIN_DISTANCE_M = 2.0;
constexpr double DEFAULT_PEER_COLLISION_MIN_DISTANCE_M = 2.0;
constexpr double DEFAULT_OBSTACLE_CLEARANCE_M = 1.0;
constexpr double DEFAULT_FORMATION_RADIUS_M = 5.0;
constexpr double DEFAULT_FORMATION_SPACING_M = 3.0;
constexpr double DEFAULT_LOCAL_ARRIVAL_TOLERANCE_M = 0.5;
constexpr double DEFAULT_PLANNER_ARRIVAL_TOLERANCE_M = 1.0;
constexpr double DEFAULT_PEER_TRAJECTORY_TIMEOUT_S = 2.0;
constexpr double DEFAULT_MAX_PEER_TIME_SKEW_S = 1.0;
// 對時正常時同儕時鐘差應在數十毫秒內；超過 0.1 s 會讓同儕位置預測誤差明顯變大。
constexpr double DEFAULT_PEER_CLOCK_SKEW_WARNING_S = 0.1;
constexpr double PEER_CLOCK_SKEW_REPORT_INTERVAL_S = 30.0;
constexpr double DEFAULT_OBSTACLE_CLOUD_TIMEOUT_S = 1.0;
constexpr double DEFAULT_MAX_VELOCITY_MPS = 2.5;
constexpr double DEFAULT_MAX_ACCELERATION_MPS2 = 1.5;
constexpr double DEFAULT_MAX_JERK_MPS3 = 3.0;
constexpr double DEFAULT_MOVE_ATTITUDE_TIMEOUT_S = 1.0;
constexpr double DEFAULT_FORMATION_SLOT_SWITCH_PENALTY_M = 2.0;
constexpr int DEFAULT_FORMATION_MEMBER_STABLE_SCAN_COUNT = 2;
constexpr double FORMATION_PLAN_START_DELAY_S = 0.5;
constexpr double PEER_REPLAN_RETRY_DELAY_S = 0.5;
// SITL 與實機 callback 負載下，central finite-difference 需要保留足夠
// 的 wall-time；仍遠低於 10 Hz planner cycle，避免移動中反覆 timeout。
constexpr double DEFAULT_OPTIMIZER_TIME_BUDGET_MS = 70.0;
// 連續規劃失敗達此次數（每次失敗至少 hold 0.5 秒，約 10 秒）時升級為 ERROR 日誌。
constexpr std::size_t PLANNER_FAILURE_ESCALATION_COUNT = 20U;
constexpr int DEFAULT_OPTIMIZER_MAX_ITERATIONS = 40;
constexpr double DEFAULT_FORMATION_CROSSING_PENALTY_M = 10.0;
// 與 2026-08 調整的開發 compose 一致；0.5 秒在實機參數下會讓隊形切換的機間距離明顯變小。
constexpr double DEFAULT_RESERVATION_LEAD_TIME_S = 0.0;
constexpr double DEFAULT_RESERVATION_STEP_S = 0.5;
constexpr double DEFAULT_RESERVATION_HORIZON_S = 5.0;
constexpr double DEFAULT_PEER_DECONFLICTION_RELEASE_MARGIN_M = 1.0;
constexpr double DEFAULT_PEER_DECONFLICTION_MAX_ESCAPE_DISTANCE_M = 12.0;
/// mission/status 週期重送頻率；狀態改變時另外立即發布
constexpr double DEFAULT_MISSION_STATUS_RATE_HZ = 1.0;
/// mission_status_rate_hz 上限，超過時回到預設值以免佔用頻寬
constexpr double MAX_MISSION_STATUS_RATE_HZ = 10.0;
/// 連續規劃失敗持續超過此秒數即發布 PLANNER_FAILURE_PERSISTENT
constexpr double DEFAULT_PLANNER_FAILURE_EVENT_S = 3.0;
/// /{agent_namespace}/mission/plan 的 QoS depth（reliable、volatile）
constexpr std::size_t MISSION_PLAN_QOS_DEPTH = 10U;
/// /{agent_namespace}/mission/status 的 QoS depth（reliable、transient_local）
constexpr std::size_t MISSION_STATUS_QOS_DEPTH = 1U;

/**
 * @brief EGO-Swarm Planner ROS2 節點
 *
 * 節點只負責 ROS2 資料收發與資料轉換；局部候選軌跡計算由
 * `ego_swarm::EgoSwarmPlanner` 純 C++ 模組完成。
 */
class EgoSwarmPlannerNode : public rclcpp::Node
{
public:
    /** @brief 建立 EGO-Swarm Planner 節點。 */
    EgoSwarmPlannerNode();
    ~EgoSwarmPlannerNode() override = default;

private:
    struct PeerState
    {
        taiphoon_agent::msg::AgentState state{};
        std::chrono::steady_clock::time_point last_arrival{};
        ego_swarm::Vector3 previous_position_ned{0.0, 0.0, 0.0};
        ego_swarm::Vector3 velocity_ned{0.0, 0.0, 0.0};
        double previous_position_time_s{0.0};
        bool has_velocity{false};
        bool has_state{false};
    };

    void own_state_cb(const taiphoon_agent::msg::AgentState::SharedPtr msg);
    void own_attitude_cb(
        const taiphoon_agent::msg::AgentAttitude::SharedPtr msg);
    void peer_state_cb(
        int agent_index,
        const taiphoon_agent::msg::AgentState::SharedPtr msg);
    void ned_target_cb(const taiphoon_agent::msg::NedTarget::SharedPtr msg);
    void mission_intent_cb(const std_msgs::msg::String::SharedPtr msg);
    void agent_command_cb(const std_msgs::msg::String::SharedPtr msg);
    void all_agent_command_cb(const std_msgs::msg::String::SharedPtr msg);
    void obstacle_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void trajectory_cb(
        const taiphoon_agent::msg::AgentTrajectory::SharedPtr msg);
    void record_peer_clock_sample(
        const taiphoon_agent::msg::AgentTrajectory& msg);
    std::string peer_clock_skew_reason(
        const std::vector<int>& expected_peer_ids, double now_s) const;
    void planner_loop_cb();
    void clear_grounded_mission();
    bool is_grounded_peer(int agent_id) const;
    void scan_peers_cb();
    void formation_epoch_cb(
        const taiphoon_agent::msg::FormationEpoch::SharedPtr msg);
    /** @brief 驗證、換算並載入 GCS 上傳的航點任務。 */
    void mission_plan_cb(
        const taiphoon_agent::msg::WaypointMission::SharedPtr msg);
    /** @brief 只處理 basic_controller 發布的 PX4_RESTART_DETECTED。 */
    void agent_event_cb(const taiphoon_agent::msg::AgentEvent::SharedPtr msg);
    /** @brief 載入逾時檢查與 mission/status 週期重送。 */
    void mission_status_timer_cb();

    /**
     * @brief 處理 MISSION_START／MISSION_STOP（只來自本機 command topic）
     * @param parsed 已解析且參數合法的命令
     */
    void handle_mission_command(const utils::ParsedCommand& parsed);
    /**
     * @brief 有效的介入命令到達時中止執行中的任務並停在目前位置
     * @param command_name 寫入 status detail 的命令名稱
     */
    void abort_mission_for_command(const std::string& command_name);
    /**
     * @brief 中止任務後依目前速度煞停，煞停終點作為之後相對命令的基準
     * @param reason 寫入日誌與 PlannerStatus 的原因
     */
    void stop_mission_in_place(const std::string& reason);
    /**
     * @brief 在 planner 迴圈中推進任務：設定到期目標與檢查單段逾時
     */
    void update_waypoint_mission();
    /**
     * @brief 設定位置型 Final Target（MOVE_POINT 與航點共用）
     * @param target planner-local NED 目標
     * @return true 表示目標有限並已套用
     */
    bool apply_position_goal(const ego_swarm::Vector3& target);
    /** @brief 依 executor 目前狀態發布 mission/status。 */
    void publish_mission_status();
    /**
     * @brief 發布一次 REJECTED，executor 狀態不變
     * @param mission_id 被拒的任務識別碼
     * @param decision 拒絕原因與航點索引
     * @param waypoint_count 被拒任務的航點數
     */
    void publish_mission_rejection(
        uint32_t mission_id,
        const ego_swarm::WaypointMissionDecision& decision,
        std::size_t waypoint_count);
    void publish_mission_status_snapshot(
        const ego_swarm::WaypointMissionStatusSnapshot& snapshot);
    /** @brief 取得 MISSION_START 的開始條件（Offboard 懸停、非編隊）。 */
    ego_swarm::WaypointMissionStartConditions mission_start_conditions() const;
    /** @brief 以 AgentEvent（SOURCE_PLANNER）發布 400～402。 */
    void publish_planner_event(
        safety::AgentEventCode code,
        const std::string& command,
        const std::string& detail);
    /**
     * @brief 發布 PLANNER_COMMAND_REJECTED；參數不合法的命令已由
     * basic_controller 回報 COMMAND_INVALID_PARAMS，這裡不重複發布
     * @param parsed 被拒的命令
     * @param detail 英文拒絕原因
     */
    void report_command_rejection(
        const utils::ParsedCommand& parsed, const std::string& detail);
    /**
     * @brief 追蹤連續規劃失敗時間，超過門檻時發布一次事件
     * @param now_s planner 時間（秒）
     * @param failure_reason 最近一次失敗原因
     */
    void track_planner_failure(double now_s, const std::string& failure_reason);
    /** @brief 列出應有新鮮軌跡的同儕（已發現、非本機、未落地）。 */
    std::vector<int> collect_expected_peer_ids() const;

    bool initialize_subscribers();
    bool initialize_publishers();
    bool attitude_is_fresh() const;
    bool make_formation_proposal(
        const std::vector<int>& member_ids,
        const ego_swarm::Vector3* requested_center = nullptr);
    bool handle_group_move_command(const utils::ParsedCommand& parsed);
    bool update_formation_assignment();
    bool get_formation_target_for_agent(
        int agent_id, ego_swarm::Vector3& target) const;
    ego_swarm::FormationConvergenceResult evaluate_formation_convergence(
        double now_s) const;
    bool epoch_is_valid(
        const taiphoon_agent::msg::FormationEpoch& epoch) const;
    bool epoch_is_same_snapshot(
        const taiphoon_agent::msg::FormationEpoch& first,
        const taiphoon_agent::msg::FormationEpoch& second) const;
    bool epoch_is_higher_priority(
        const taiphoon_agent::msg::FormationEpoch& first,
        const taiphoon_agent::msg::FormationEpoch& second) const;
    bool epoch_parent_matches_current(
        const taiphoon_agent::msg::FormationEpoch& epoch) const;
    void adopt_formation_epoch(
        const taiphoon_agent::msg::FormationEpoch& epoch);
    void maybe_activate_formation_epoch();
    void set_candidate_formation_epoch(
        const taiphoon_agent::msg::FormationEpoch& epoch);
    void discard_candidate_formation_epoch(const char* reason);
    void expire_stalled_formation_epoch();
    bool is_member_view_stable() const;
    void publish_formation_epoch(uint8_t status);
    std::vector<int> stable_member_ids() const;
    /**
     * @brief 檢查指定成員數的槽位中心距離是否不小於 formation_min_distance_m
     * @param config 編隊幾何
     * @param member_count 成員數
     * @return true 表示槽位幾何有效且安全
     */
    bool is_formation_geometry_safe(
        const ego_swarm::FormationConfig& config,
        std::size_t member_count) const;
    /**
     * @brief 成員不足時解散編隊，以目前位置作為懸停目標
     * @param reason 寫入日誌的原因
     */
    void dissolve_formation_to_hold(const char* reason);
    bool formation_config_matches(
        const ego_swarm::FormationConfig& first,
        const ego_swarm::FormationConfig& second) const;
    ego_swarm::LivePeerDistanceResult live_peer_min_distance(
        const ego_swarm::Vector3& current) const;
    ego_swarm::PeerDeconflictionInput make_peer_deconfliction_input(
        const ego_swarm::Vector3& current,
        const ego_swarm::Vector3& final_target,
        double now_s,
        bool peer_data_valid) const;
    bool handle_peer_deconfliction(
        const ego_swarm::Vector3& current,
        const ego_swarm::Vector3& final_target,
        const ego_swarm::VehiclePlanningState& planning_state,
        const rclcpp::Time& now,
        bool peer_data_valid);
    bool peer_deconfliction_released(
        const ego_swarm::PeerDeconflictionInput& input) const;
    void clear_peer_deconfliction_state();
    bool publish_peer_deconfliction_hold(
        const ego_swarm::Vector3& current,
        const ego_swarm::Vector3& final_target,
        int owner_agent_id,
        const std::vector<int>& conflict_peer_ids,
        const std::string& reason,
        const rclcpp::Time& now,
        bool peer_data_valid);
    bool update_planner_frame();
    bool update_planner_to_px4_ned_transform();
    bool get_position_in_planner_frame(
        const taiphoon_agent::msg::AgentState& state,
        ego_swarm::Vector3& position) const;
    bool get_target_in_px4_local_ned(
        const ego_swarm::Vector3& target,
        ego_swarm::Vector3& local_target) const;
    /** @brief 清除不再代表目前控制狀態的 terminal hold heartbeat。 */
    void clear_terminal_hold();
    ego_swarm::PositionArrivalResult evaluate_target_arrival(
        const ego_swarm::Vector3& current_planner_ned,
        const ego_swarm::Vector3& final_target) const;
    void invalidate_active_plan();
    /** @brief 取得目前任務真正的 Final Target。 */
    bool get_current_final_target(ego_swarm::Vector3& final_target) const;
    void update_vehicle_motion_state(
        const taiphoon_agent::msg::AgentState& state);
    ego_swarm::VehiclePlanningState make_planning_state(
        const ego_swarm::Vector3& position,
        double now_s) const;
    ego_swarm::SafetyValidationResult validate_plan(
        const ego_swarm::BsplineTrajectory& trajectory,
        const ego_swarm::VehiclePlanningState& state,
        double now_s,
        double trajectory_start_time_s,
        const std::vector<ego_swarm::PeerTrajectory>* peer_snapshot = nullptr) const;
    /**
     * @param report_event false 表示預期中的停止（到達時仍在移動、任務停止），
     * 不發布 PLANNER_EMERGENCY_BRAKE
     */
    bool publish_emergency_brake(
        const ego_swarm::VehiclePlanningState& state,
        const ego_swarm::Vector3& active_target,
        const std::string& reason,
        const rclcpp::Time& now,
        bool peer_data_valid,
        bool report_event = true);
    bool publish_waiting_hold(
        const ego_swarm::Vector3& current,
        const ego_swarm::Vector3& target,
        const std::string& reason,
        const rclcpp::Time& now,
        bool peer_data_valid);
    void set_goal_from_command(const std::string& command);
    enum class TrajectoryPublishRoute : uint8_t
    {
        PEER_ONLY,
        PEER_AND_LOCAL
    };
    void publish_plan(
        const ego_swarm::PlannerResult& result,
        const ego_swarm::Vector3& origin_position,
        const rclcpp::Time& now,
        const rclcpp::Time& start_time,
        uint32_t trajectory_id,
        bool terminal_hold = false,
        TrajectoryPublishRoute route = TrajectoryPublishRoute::PEER_AND_LOCAL);
    void publish_status(
        uint8_t state,
        const std::string& reason,
        const ego_swarm::PlannerResult* result,
        bool localization_valid,
        bool peer_data_valid,
        const rclcpp::Time& stamp);
    std::string obstacle_map_status_reason(double now_s) const;
    int compute_self_index() const;

    std::string agent_namespace_{"uav_1_agent"};
    int agent_index_{1};
    /// 舊 launch 相容參數的快取；planner origin 固定使用本 Agent Home。
    int legacy_swarm_origin_agent_index_{1};
    double planning_rate_hz_{DEFAULT_EGO_PLANNER_RATE_HZ};
    double peer_trajectory_timeout_s_{DEFAULT_PEER_TRAJECTORY_TIMEOUT_S};
    double max_peer_time_skew_s_{DEFAULT_MAX_PEER_TIME_SKEW_S};
    double peer_clock_skew_warning_s_{DEFAULT_PEER_CLOCK_SKEW_WARNING_S};
    double obstacle_cloud_timeout_s_{DEFAULT_OBSTACLE_CLOUD_TIMEOUT_S};
    bool require_obstacle_cloud_{true};
    double formation_min_distance_m_{DEFAULT_FORMATION_MIN_DISTANCE_M};
    double local_arrival_tolerance_m_{DEFAULT_LOCAL_ARRIVAL_TOLERANCE_M};
    double planner_arrival_tolerance_m_{DEFAULT_PLANNER_ARRIVAL_TOLERANCE_M};
    double move_attitude_timeout_s_{DEFAULT_MOVE_ATTITUDE_TIMEOUT_S};
    double formation_slot_switch_penalty_m_ =
        DEFAULT_FORMATION_SLOT_SWITCH_PENALTY_M;
    int formation_member_stable_scan_count_ =
        DEFAULT_FORMATION_MEMBER_STABLE_SCAN_COUNT;
    std::string planner_backend_{"optimized"};
    int optimizer_max_iterations_{DEFAULT_OPTIMIZER_MAX_ITERATIONS};
    double optimizer_time_budget_ms_{DEFAULT_OPTIMIZER_TIME_BUDGET_MS};
    /// 連續規劃失敗次數；成功規劃後歸零，只用於日誌升級，不改變控制決策
    std::size_t consecutive_planner_failures_{0U};
    /// 本輪連續失敗的開始時間與是否已發布 PLANNER_FAILURE_PERSISTENT
    double planner_failure_since_s_{0.0};
    double planner_failure_last_s_{0.0};
    bool planner_failure_event_sent_{false};
    double planner_failure_event_s_{DEFAULT_PLANNER_FAILURE_EVENT_S};
    double mission_status_rate_hz_{DEFAULT_MISSION_STATUS_RATE_HZ};
    /// 航點任務狀態機；時間一律使用 steady clock 秒數
    ego_swarm::WaypointMissionExecutor waypoint_mission_executor_{};
    /// 上一次發布的任務狀態，用於判斷是否需要立即發布
    ego_swarm::WaypointMissionStatusSnapshot last_mission_status_{};
    bool has_published_mission_status_{false};
    safety::AgentEventGate event_gate_{};
    double formation_crossing_penalty_m_ =
        DEFAULT_FORMATION_CROSSING_PENALTY_M;
    double reservation_lead_time_s_{DEFAULT_RESERVATION_LEAD_TIME_S};
    double reservation_step_s_{DEFAULT_RESERVATION_STEP_S};
    double reservation_horizon_s_{DEFAULT_RESERVATION_HORIZON_S};
    bool peer_deconfliction_enabled_{true};
    ego_swarm::PlannerFeatureFlags planner_feature_flags_{};

    ego_swarm::PlannerConfig planner_config_{};
    std::unique_ptr<ego_swarm::ITrajectoryPlanner> selected_planner_{};
    ego_swarm::ContinuousSafetyValidator safety_validator_{};
    ego_swarm::EmergencyBrakePlanner emergency_brake_planner_{};
    ego_swarm::PeerDeconflictionPlanner peer_deconfliction_planner_{};
    ego_swarm::PeerTrajectoryTable peer_trajectories_{};
    /// 只供診斷：估計同儕主機時鐘差，不參與規劃或控制決策。
    ego_swarm::PeerClockSkewMonitor peer_clock_skew_monitor_{};
    std::map<int, double> peer_clock_skew_last_report_s_{};
    ego_swarm::LocalNedFrame planner_frame_{};
    ego_swarm::GeoCoordinate planner_frame_origin_{};
    /// planner-local Home 所屬的 Agent ID（本機自身）；-1 表示尚未建立。
    int planner_frame_origin_agent_index_{-1};
    uint32_t planner_frame_revision_{0U};
    /// planner-local NED 至 PX4 local NED 的固定平移是否已建立。
    bool has_planner_to_px4_ned_transform_{false};
    ego_swarm::Vector3 planner_to_px4_ned_offset_{0.0, 0.0, 0.0};

    bool has_own_state_{false};
    ego_swarm::FlightMissionLifecycle flight_mission_lifecycle_{};
    bool has_previous_state_sample_{false};
    bool has_vehicle_motion_state_{false};
    double previous_state_timestamp_s_{0.0};
    ego_swarm::Vector3 previous_position_ned_{0.0, 0.0, 0.0};
    ego_swarm::Vector3 previous_velocity_ned_{0.0, 0.0, 0.0};
    ego_swarm::VehiclePlanningState vehicle_motion_state_{};
    bool has_goal_{false};
    /// 目標到達後仍保留最後的任務目標，供下一個相對命令累加。
    bool has_position_target_{false};
    bool has_formation_{false};
    bool formation_command_pending_{false};
    bool formation_membership_change_pending_{false};
    bool formation_initial_plan_pending_{false};
    bool preserve_altitude_{false};
    bool has_active_plan_{false};
    bool emergency_brake_active_{false};
    bool waiting_for_replan_{false};
    bool peer_deconfliction_active_{false};
    int peer_deconfliction_owner_id_{0};
    std::vector<int> peer_deconfliction_conflict_peer_ids_{};
    double waiting_retry_time_s_{0.0};
    bool has_terminal_hold_{false};
    taiphoon_agent::msg::AgentState own_state_{};
    taiphoon_agent::msg::AgentAttitude own_attitude_{};
    bool has_own_attitude_{false};
    std::chrono::steady_clock::time_point attitude_last_arrival_{};
    /// 累積相對命令使用的目標 yaw；未設定前以第一筆有效 attitude 初始化。
    bool has_target_yaw_{false};
    double target_yaw_rad_{0.0};
    ego_swarm::Vector3 goal_position_{0.0, 0.0, 0.0};
    ego_swarm::PlannerResult active_plan_{};
    ego_swarm::Vector3 active_plan_target_{0.0, 0.0, 0.0};
    /// 目前 active trajectory 所屬任務的真正 Final Target。
    ego_swarm::Vector3 active_plan_final_target_{0.0, 0.0, 0.0};
    rclcpp::Time active_plan_start_time_{
        static_cast<int32_t>(0), static_cast<uint32_t>(0), RCL_ROS_TIME};
    /// 最近一次接受正常 planner candidate 的時間，用於
    /// progress replan interval。
    bool has_last_replan_time_{false};
    double last_replan_time_s_{0.0};
    uint32_t active_plan_trajectory_id_{0U};
    ego_swarm::PlannerResult terminal_hold_plan_{};
    ego_swarm::Vector3 terminal_hold_position_{0.0, 0.0, 0.0};
    rclcpp::Time terminal_hold_start_time_{
        static_cast<int32_t>(0), static_cast<uint32_t>(0), RCL_ROS_TIME};
    uint32_t terminal_hold_trajectory_id_{0U};
    perception::RawPointObstacleMap obstacle_map_{};
    ego_swarm::FormationConfig formation_config_{};
    ego_swarm::Vector3 formation_center_{0.0, 0.0, 0.0};
    /// 是否已有可供後續編隊命令沿用的 formation center target。
    bool has_formation_center_target_{false};
    std::map<int, std::size_t> formation_assignment_{};

    bool has_accepted_formation_epoch_{false};
    bool has_candidate_formation_epoch_{false};
    taiphoon_agent::msg::FormationEpoch accepted_formation_epoch_{};
    taiphoon_agent::msg::FormationEpoch candidate_formation_epoch_{};
    std::set<int> candidate_adopted_agents_{};
    /// 目前候選 epoch 建立或採用的時間，用於判斷未完成 proposal 逾時。
    std::chrono::steady_clock::time_point candidate_epoch_since_{};
    /// 已逾時的候選 epoch；heartbeat 不得再把它帶回。
    ego_swarm::ExpiredFormationEpochs expired_formation_epochs_{};
    /// 各 peer 最近一次送出 FormationEpoch 的時間，代表已準備加入編隊。
    std::map<int, std::chrono::steady_clock::time_point> formation_ready_signal_time_{};
    double last_epoch_publish_s_{0.0};
    std::vector<int> discovery_candidate_members_{};
    std::vector<int> stable_members_{};
    int discovery_same_scan_count_{0};
    std::map<int, int> discovery_absent_scan_count_{};
    std::set<int> currently_discovered_peers_{};

    std::map<int, PeerState> peers_{};
    std::map<int, rclcpp::Subscription<taiphoon_agent::msg::AgentState>::SharedPtr>
        peer_subscriptions_{};

    uint32_t trajectory_id_{0U};
    uint32_t last_published_trajectory_id_{0U};
    rclcpp::Subscription<taiphoon_agent::msg::AgentState>::SharedPtr own_state_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::AgentAttitude>::SharedPtr
        own_attitude_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::NedTarget>::SharedPtr ned_target_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_intent_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr agent_command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr all_command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::AgentTrajectory>::SharedPtr
        trajectory_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::FormationEpoch>::SharedPtr
        formation_epoch_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::WaypointMission>::SharedPtr
        mission_plan_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::AgentEvent>::SharedPtr
        agent_event_sub_;

    rclcpp::Publisher<taiphoon_agent::msg::AgentTrajectory>::SharedPtr
        trajectory_pub_;
    rclcpp::Publisher<taiphoon_agent::msg::AgentTrajectory>::SharedPtr
        local_trajectory_pub_;
    rclcpp::Publisher<taiphoon_agent::msg::PlannerStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<taiphoon_agent::msg::FormationEpoch>::SharedPtr
        formation_epoch_pub_;
    rclcpp::Publisher<taiphoon_agent::msg::WaypointMissionStatus>::SharedPtr
        mission_status_pub_;
    rclcpp::Publisher<taiphoon_agent::msg::AgentEvent>::SharedPtr events_pub_;
    rclcpp::TimerBase::SharedPtr planner_timer_;
    rclcpp::TimerBase::SharedPtr peer_scan_timer_;
    rclcpp::TimerBase::SharedPtr mission_status_timer_;
};

}  // namespace nodes
}  // namespace taiphoon
