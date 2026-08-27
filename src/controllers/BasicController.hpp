/**
 * @file BasicController.hpp
 * @brief Basic flight controller implementing IBasicFlightController
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 */

#pragma once

#include <string>
#include <memory>
#include <atomic>
#include <array>
#include <cstdint>
#include <limits>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"

#include "core/IBasicFlightController.hpp"
#include "controllers/FlightState.hpp"
#include "controllers/FlightStateResolver.hpp"

namespace taiphoon
{
namespace controllers
{

/// 進入 Offboard 前必須完成的 OffboardControlMode 心跳數。
constexpr uint32_t OFFBOARD_PRESTREAM_COUNT = 10U;

/// 模式切換後 VehicleCommand 回應超時 (毫秒)
constexpr int VEHICLE_CMD_ECHO_TIMEOUT_MS = 3000;

/// 起飛超時 (毫秒，已廢棄：達成判斷由外部節點負責)
// constexpr int TAKEOFF_TIMEOUT_MS = 30000;

/// 高度日誌記錄間隔 (控制週期數)
constexpr int ALTITUDE_LOG_INTERVAL = 20;

/// 垂直移動最低高度限制 (公尺)
constexpr float ALTITUDE_MIN_M = 1.0F;

/// RTL 完成等待超時 (毫秒)
constexpr int RTL_TIMEOUT_MS = 60000;

/// 預設 ROS2 QoS depth
constexpr int DEFAULT_QOS_DEPTH = 10;

/// 預設目標系統 ID
constexpr uint8_t DEFAULT_TARGET_SYSTEM = 1;

/// PX4 飛行狀態資料逾時時間（毫秒）
constexpr int PX4_STATE_TIMEOUT_MS = 2000;

/**
 * @brief 可由軌跡規劃器提供的定時 B-spline 軌跡資料。
 *
 * 此型別刻意不依賴 AgentTrajectory ROS2 訊息，讓 BasicController 的
 * 軌跡取樣 seam 可在訊息介面尚未產生時獨立編譯與測試。
 * `order` 定義為 B-spline 多項式階數；例如 cubic 使用 order=3。
 */
struct TimedTrajectory
{
    rclcpp::Time stamp{static_cast<int64_t>(0), RCL_ROS_TIME};
    rclcpp::Time start_time{static_cast<int64_t>(0), RCL_ROS_TIME};
    uint32_t trajectory_id{0U};
    uint32_t agent_index{0U};
    uint32_t frame_revision{0U};
    uint8_t order{0U};
    float knot_interval_s{0.0F};
    std::vector<std::array<float, 3>> control_points;
    float duration_s{0.0F};
    bool valid{false};
};

/**
 * @brief 定時軌跡取樣結果。
 */
enum class TimedTrajectorySampleResult : uint8_t
{
    VALID = 0U,
    NO_TRAJECTORY = 1U,
    NOT_STARTED = 2U,
    COMPLETED = 3U,
    INVALID = 4U
};

/**
 * @brief 基礎飛行控制器
 *
 * 實作 IBasicFlightController 介面，透過 PX4 Offboard 模式控制無人機。
 * 提供起飛、降落、返航、水平移動、模式切換等基礎飛行功能。
 */
class BasicController : public core::IBasicFlightController
{
public:
    /**
     * @brief 建構子
     *
     * @param node ROS2 節點指標 (必須不為 nullptr)
     * @param target_system PX4 MAVLink 系統 ID (uav_1=1, uav_2=2, ...)
     */
    explicit BasicController(rclcpp::Node* node,
                             uint8_t target_system = DEFAULT_TARGET_SYSTEM);

    ~BasicController() override = default;

    // ==================== IBasicFlightController 介面實作 ====================

    /** @brief 初始化所有 Publisher 與 Subscriber */
    bool initialize() override;

    /** @brief 清理控制器資源 */
    void shutdown() override;

    /** @brief 檢查控制器是否已初始化 */
    bool is_initialized() const override;

    /** @brief 發送 Arm (解鎖) 命令 */
    bool arm() override;

    /** @brief 發送 Disarm (上鎖) 命令 */
    bool disarm() override;

    /** @brief 起飛至指定高度 (公尺) */
    bool takeoff(float altitude_m) override;

    /** @brief 在空中停留指定秒數 */
    bool hold_position(float duration_s) override;

    /** @brief 降落 */
    bool land() override;

    /** @brief 執行起飛-停留-降落完整任務序列 */
    bool execute_takeoff_land_sequence(
        float altitude_m, float hover_duration_s) override;

    /** @brief 取得目前飛行狀態字串 */
    const char* get_state() const override;

    /** @brief 取得目前命令意圖的任務階段 */
    FlightState get_flight_state() const;

    /**
     * @brief 取得最新 PX4 飛行狀態觀測資料
     *
     * @return 包含有效性、解鎖、落地與原生導航模式的觀測資料
     */
    FlightStateObservation get_flight_state_observation() const;

    // ==================== 高階命令新增方法 ====================

    /**
     * @brief 切換至 RTL (Return to Launch) 模式
     *
     * PX4 會自動處理降落、落地偵測與自動上鎖
     *
     * @return true 命令發送成功
     */
    bool switch_to_rtl_mode();

    /**
     * @brief 切換至 LAND 模式
     *
     * PX4 會自動處理降落程序
     *
     * @return true 命令發送成功
     */
    bool switch_to_land_mode();

    /**
     * @brief 切換至 OFFBOARD 模式
     *
     * @return true 命令發送成功
     */
    bool switch_to_offboard_mode();

    /**
     * @brief 切換至 HOLD (位置保持) 模式
     *
     * 發送 HOLD 模式命令，使無人機在當前位置保持懸停
     *
     * @return true 命令發送成功
     */
    bool switch_to_hold_mode();

    /**
     * @brief 從目前位置向前/後移動指定距離
     *
     * @param distance_m 移動距離 (正數為向前，負數為向後)
     * @return true 移動命令設定成功
     */
    bool move_forward(float distance_m);

    /**
     * @brief 從目前位置向左/右移動指定距離
     *
     * @param distance_m 移動距離 (正數為向右，負數為向左)
     * @return true 移動命令設定成功
     */
    bool move_right(float distance_m);

    /**
     * @brief 垂直移動指定高度 (相對位移)
     *
     * 正值為上升，負值為下降。下降時不會低於 ALTITUDE_MIN_M。
     *
     * @param delta_m 高度變化量 (公尺)
     * @return true 移動命令設定成功
     */
    bool move_vertical(float delta_m) override;

    /**
     * @brief 旋轉至指定偏航角 (相對目前方向)
     *
     * 正值為順時針旋轉。角度以度為單位，內部轉為弧度。
     *
     * @param degrees 旋轉角度 (度)
     * @return true 旋轉命令設定成功
     */
    bool rotate_yaw(float degrees) override;

    /**
     * @brief 設定絕對偏航角，不改變目前的位置目標
     *
     * 供 MOVE_POINT 的 yaw 欄位使用；與 rotate_yaw() 的相對角度
     * 語意不同。PX4 NED yaw 以順時針為正。
     *
     * @param degrees 絕對偏航角 (度)
     * @return true 目標設定成功
     */
    bool set_absolute_yaw(float degrees);

    /**
     * @brief 設定 NED 絕對座標目標位置與航向
     *
     * 直接寫入目標 NED 位置，要求控制器發布一次位置設定點。
     * 不進行內部達成判斷。
     *
     * @param x NED X (前向，公尺)
     * @param y NED Y (右向，公尺)
     * @param z NED Z (向下，公尺)
     * @param yaw_deg 目標航向角 (度)，NaN 表示不控制
     * @return true 目標設定成功
     */
    bool set_ned_target(float x, float y, float z, float yaw_deg) override;

    // ==================== 公開輔助方法 ====================

    /**
     * @brief 更新車輛里程計 (由節點回呼驅動)
     *
     * @param msg VehicleOdometry 訊息
     */
    void update_odometry(const px4_msgs::msg::VehicleOdometry& msg);

    /**
     * @brief 發布 Offboard 心跳，以及必要時發布軌跡設定點 (計時器回呼)
     */
    void publish_control_commands();

    /**
     * @brief 取得目前高度
     *
     * @return float 目前高度 (公尺)
     */
    float get_current_altitude() const;

    /**
     * @brief 取得目前位置 (NED 座標)
     *
     * @return std::array<float, 3> [x, y, z]
     */
    std::array<float, 3> get_current_position_ned() const;

    /** @brief 取得目前目標 NED 位置 */
    std::array<float, 3> get_target_position_ned() const;

    /**
     * @brief 更新最新有效的定時軌跡。
     *
     * 收到非法軌跡時保留目前有效軌跡；收到座標版本變更時清除舊軌跡，
     * 讓控制迴圈至少輸出一次目前位置 hold，避免在不同座標系間跳接。
     * 新版本的第一筆軌跡會被當作版本同步訊號拒絕，後續同版本軌跡才會啟用。
     *
     * @param trajectory 規劃器輸出的定時軌跡
     * @return true 表示軌跡已儲存並可供取樣
     */
    bool update_timed_trajectory(const TimedTrajectory& trajectory);

    /**
     * @brief 依指定 ROS 時間取樣目前有效的定時軌跡。
     *
     * 使用 uniform clamped B-spline 的 De Boor 演算法。duration_s 是
     * 軌跡的有效時間窗，時間窗內不會因資料年齡增加任何安全半徑。
     *
     * @param sample_time 取樣時間
     * @param position 輸出的 NED 位置
     * @return 取樣結果；VALID 時 position 是軌跡取樣結果，COMPLETED 時
     *         position 是最後控制點，供完成後發布一次 endpoint 使用
     */
    TimedTrajectorySampleResult sample_timed_trajectory(
        const rclcpp::Time& sample_time,
        std::array<float, 3>& position) const;

    /**
     * @brief 依指定 ROS 時間取樣定時軌跡的 p/v/a。
     *
     * position 會套用已在 Node 完成的 local NED frame translation；velocity
     * 與 acceleration 是對時間的 analytic derivative，不套用位置平移。
     * 尚未開始或已過期時，輸出 derivative 會維持零向量，避免把無效的
     * feed-forward 傳給 PX4。
     *
     * @param sample_time 取樣時間
     * @param position 輸出的 NED 位置
     * @param velocity 輸出的 NED 速度
     * @param acceleration 輸出的 NED 加速度
     * @return 取樣結果
     */
    TimedTrajectorySampleResult sample_timed_trajectory(
        const rclcpp::Time& sample_time,
        std::array<float, 3>& position,
        std::array<float, 3>& velocity,
        std::array<float, 3>& acceleration) const;

    /**
     * @brief 檢查目前是否保留可取樣的定時軌跡。
     * @return true 表示緩衝區內有最新有效軌跡
     */
    bool has_timed_trajectory() const;

    /**
     * @brief 將目標重設為目前位置並進入 HOVER
     * @return true 控制器已初始化且保持目標設定成功
     */
    bool hold_current_position();

private:
    // ==================== Publisher 初始化 (單一職責) ====================

    /**
     * @brief 建立 OffboardControlMode Publisher
     *
     * 發布至 fmu/in/offboard_control_mode
     *
     * @return true 建立成功
     */
    bool create_offboard_control_mode_publisher();

    /**
     * @brief 建立 TrajectorySetpoint Publisher
     *
     * 發布至 fmu/in/trajectory_setpoint
     *
     * @return true 建立成功
     */
    bool create_trajectory_setpoint_publisher();

    /**
     * @brief 建立 VehicleCommand Publisher
     *
     * 發布至 fmu/in/vehicle_command
     *
     * @return true 建立成功
     */
    bool create_vehicle_command_publisher();

    // ==================== Subscriber 初始化 (單一職責) ====================

    /**
     * @brief 建立 VehicleCommandAck Subscriber
     *
     * 訂閱 fmu/out/vehicle_command_ack_v1，用於確認命令回應
     *
     * @return true 建立成功
     */
    bool create_vehicle_command_ack_subscriber();

    /**
     * @brief 建立 VehicleStatus Subscriber
     *
     * 訂閱 fmu/out/vehicle_status_v4，用於監控解鎖/上鎖狀態
     *
     * @return true 建立成功
     */
    bool create_vehicle_status_subscriber();

    /**
     * @brief 建立 VehicleLandDetected Subscriber
     *
     * 訂閱 fmu/out/vehicle_land_detected，用於監控實際落地狀態
     *
     * @return true 建立成功
     */
    bool create_vehicle_land_detected_subscriber();

    // ==================== 狀態機處理 (單一職責) ====================

    /**
     * @brief 處理 OFFBOARD_INIT 狀態
     *
     * 檢查 OffboardControlMode 心跳送出後的 ACK、VehicleStatus 與確認逾時。
     */
    void handle_offboard_init_state();

    /**
     * @brief 處理 TAKEOFF 狀態
     *
     * 設定上升軌跡設定點，監控高度到達狀態
     *
     * @param msg TrajectorySetpoint 訊息 (更新位置)
     */
    void handle_takeoff_state(
        px4_msgs::msg::TrajectorySetpoint& msg);

    /**
     * @brief 處理 HOVER 狀態
     *
     * 在目標高度停留指定時間後切換至 RTL
     *
     * @param msg TrajectorySetpoint 訊息 (保持高度)
     */
    void handle_hover_state(
        px4_msgs::msg::TrajectorySetpoint& msg);

    /**
     * @brief 處理 RTL 狀態
     *
     * 發送 RTL 命令，等待 PX4 完成降落並上鎖
     */
    void handle_rtl_state();

    /**
     * @brief 處理 LAND 狀態
     *
     * 發送 LAND 命令，等待 PX4 完成降落
     */
    void handle_land_state();

    /**
     * @brief 處理 OFFBOARD_ACTIVE 狀態
     *
     * Offboard 已啟用，維持心跳，等待 ARM 命令
     */
    void handle_offboard_active_state();

    /**
     * @brief 處理 MOVING 狀態
     *
     * 執行水平移動至目標位置
     *
     * @param msg TrajectorySetpoint 訊息 (更新目標位置)
     */
    void handle_moving_state(
        px4_msgs::msg::TrajectorySetpoint& msg);

    /**
     * @brief 處理 POST_RTL 狀態
     *
     * RTL 降落完成後，停止 Offboard 控制，發送 Hold 模式命令。
     */
    void handle_post_rtl_state();

    // ==================== 輔助方法 ====================

    /**
     * @brief 發送 Arm/Disarm VehicleCommand
     *
     * @param do_arm true 為 Arm，false 為 Disarm
     * @return true 發送成功
     */
    bool send_arm_command(bool do_arm);

    /**
     * @brief 發送切換至 Offboard 模式的 VehicleCommand
     *
     * PX4 1.18: command=176 (VEHICLE_CMD_DO_SET_MODE),
     * param1=1.0 (custom mode), param2=6.0 (Offboard for multicopter)
     *
     * @return true 發送成功
     */
    bool send_offboard_mode_command();

    /**
     * @brief 發送 RTL (Return to Launch) 命令
     *
     * PX4 1.18: command=20 (VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
     *
     * @return true 發送成功
     */
    bool send_rtl_command();

    /**
     * @brief 發送 LAND 命令
     *
     * PX4 1.18: command=21 (VEHICLE_CMD_NAV_LAND)
     *
     * @return true 發送成功
     */
    bool send_land_command();

    /**
     * @brief 發送 HOLD (保持位置) 模式命令
     *
     * PX4 1.18: command=176 (VEHICLE_CMD_DO_SET_MODE),
     * param1=1.0 (custom mode), param2=8.0 (Position Hold for multicopter)
     *
     * @return true 發送成功
     */
    bool send_hold_mode_command();

    /**
     * @brief 驗證設定點數值 (無 NaN/Inf)
     *
     * @param x X 座標
     * @param y Y 座標
     * @param z Z 座標
     * @return true 數值有效
     */
    bool validate_setpoint(float x, float y, float z) const;

    /**
     * @brief VehicleCommandAck 回呼 (PX4 輸出主題)
     *
     * @param msg VehicleCommandAck 訊息
     */
    void vehicle_command_ack_cb(
        const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);

    /**
     * @brief VehicleStatus 回呼 (監控解鎖/上鎖狀態)
     *
     * @param msg VehicleStatus 訊息
     */
    void vehicle_status_cb(
        const px4_msgs::msg::VehicleStatus::SharedPtr msg);

    /**
     * @brief VehicleLandDetected 回呼（監控實際落地狀態）
     *
     * @param msg VehicleLandDetected 訊息
     */
    void vehicle_land_detected_cb(
        const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

    /**
     * @brief 依 PX4 觀測資料更新已完成或逾時的命令意圖
     */
    void update_command_state_from_observation();

    /**
     * @brief 判斷 VehicleStatus 是否仍在有效時間內。
     */
    bool is_vehicle_status_fresh() const;

    /**
     * @brief 判斷 PX4 是否已實際進入 Offboard。
     */
    bool is_px4_offboard_confirmed() const;

    /**
     * @brief 判斷目前控制週期是否允許發布 OffboardControlMode 心跳。
     */
    bool should_publish_offboard_control_mode() const;

    /**
     * @brief 判斷目前控制週期是否需要發布 TrajectorySetpoint。
     */
    bool should_publish_trajectory_setpoint() const;

    /**
     * @brief 開始一次新的 Offboard 進入流程。
     */
    void begin_offboard_session();

    /**
     * @brief 停止目前 Offboard session 並清除本機定時軌跡。
     */
    void stop_offboard_session();

    /**
     * @brief 將 Offboard 進入失敗標記為 ERROR。
     * @param reason 失敗原因
     */
    void fail_offboard_entry(const char* reason);

    /**
     * @brief 清除目前保留的本機定時軌跡。
     */
    void clear_timed_trajectory();

    /**
     * @brief 完成指定識別碼的定時軌跡，避免清除較新的軌跡。
     * @param trajectory_id 已完成的軌跡識別碼
     * @return true 表示目前仍是同一條軌跡並已完成清除
     */
    bool complete_timed_trajectory(uint32_t trajectory_id);

    /**
     * @brief 取得有限的目前／目標位置，供無有效控制時產生 hold setpoint。
     * @return 優先目前位置，其次目標位置；兩者異常時回傳有限原點
     */
    std::array<float, 3> get_safe_hold_position() const;

    /**
     * @brief 驗證定時軌跡資料。
     * @param trajectory 待驗證軌跡
     * @return true 表示欄位與控制點皆為有限且可評估
     */
    static bool validate_timed_trajectory(const TimedTrajectory& trajectory);

    /**
     * @brief 判斷軌跡是否可以在目前 TAKEOFF 控制 epoch 內接管本機。
     * @param trajectory 待判斷軌跡
     * @return true 表示允許寫入本機軌跡緩衝區
     */
    bool is_timed_trajectory_allowed_in_current_epoch(
        const TimedTrajectory& trajectory) const;

    // ==================== 成員變數 ====================

    /// ROS2 節點指標
    rclcpp::Node* node_;

    // --- Publishers ---
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
        offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
        trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
        vehicle_command_pub_;

    // --- Subscribers ---
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr
        vehicle_command_ack_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
        vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
        vehicle_land_detected_sub_;

    /// PX4 MAVLink 系統 ID (uav_1=1, uav_2=2, ...)
    uint8_t target_system_{DEFAULT_TARGET_SYSTEM};

    /// 目前高度 (公尺)
    std::atomic<float> current_altitude_m_{0.0F};

    /// 目標高度 (公尺)
    std::atomic<float> target_altitude_m_{0.0F};

    /// 停留持續時間 (秒)
    std::atomic<float> hover_duration_s_{0.0F};

    /// 目前命令意圖的任務階段
    std::atomic<FlightState> flight_state_{FlightState::IDLE};

    /// 初始化完成旗標
    std::atomic<bool> is_initialized_{false};

    /// 計時相關
    rclcpp::Time offboard_start_time_;
    rclcpp::Time offboard_mode_request_time_;
    rclcpp::Time takeoff_start_time_;
    int altitude_log_counter_{0};

    /// VehicleCommandAck 確認旗標
    std::atomic<bool> offboard_cmd_echo_received_{false};
    std::atomic<bool> offboard_cmd_rejected_{false};
    std::atomic<bool> arm_cmd_echo_received_{false};

    /// Offboard 進入流程內部狀態
    std::atomic<uint32_t> offboard_prestream_count_{0U};
    std::atomic<bool> offboard_mode_request_sent_{false};
    std::atomic<bool> offboard_session_active_{false};
    std::atomic<bool> offboard_entry_failed_{false};

    /// 車輛解鎖狀態（參閱 px4_msgs/VehicleStatus）
    std::atomic<uint8_t> arming_state_{0};

    /// PX4 原生導航模式（VehicleStatus.nav_state）
    std::atomic<uint8_t> px4_nav_state_{0};

    /// PX4 實際落地狀態（VehicleLandDetected.landed）
    std::atomic<bool> is_landed_{false};

    /// 最新 VehicleStatus 回呼時間（ROS 時鐘奈秒）
    std::atomic<int64_t> last_vehicle_status_ns_{0};

    /// 最新 VehicleLandDetected 回呼時間（ROS 時鐘奈秒）
    std::atomic<int64_t> last_land_detected_ns_{0};

    /// RTL 命令已發送旗標
    bool rtl_cmd_sent_{false};

    /// LAND 命令已發送旗標
    bool land_cmd_sent_{false};

    /// 目前位置 (NED 座標系) [x, y, z]
    std::array<float, 3> current_position_ned_{0.0F, 0.0F, 0.0F};
    /// 目前偏航角（弧度，來自 odometry 四元數）
    std::atomic<float> current_yaw_rad_{0.0F};

    /// 目標位置 (NED 座標系) [x, y, z]，用於移動命令
    std::array<float, 3> target_position_ned_{0.0F, 0.0F, 0.0F};

    /// 定時軌跡緩衝區；只有 BasicController 發布其取樣結果至 PX4
    mutable std::mutex timed_trajectory_mutex_;
    TimedTrajectory timed_trajectory_{};
    bool has_timed_trajectory_{false};
    bool has_frame_revision_{false};
    uint32_t timed_trajectory_frame_revision_{0U};
    bool has_completed_trajectory_{false};
    uint32_t last_completed_trajectory_id_{0U};

    /// TAKEOFF 取得本機控制權的時間；更早建立的軌跡不可重新接管 PX4。
    rclcpp::Time timed_trajectory_epoch_start_{
        static_cast<int64_t>(0), RCL_ROS_TIME};
    bool has_timed_trajectory_epoch_{false};

    /// 一次性 TrajectorySetpoint 待發布旗標（TAKEOFF/HOLD/直接目標）。
    std::atomic<bool> trajectory_setpoint_pending_{false};

    /// 定時軌跡執行中的 TrajectorySetpoint 發布旗標。
    std::atomic<bool> trajectory_setpoint_active_{false};

    /// Offboard 控制已停止旗標 (RTL 落地後)
    bool has_offboard_stopped_{false};

    /// 目標偏航角 (弧度)，NaN 表示不控制偏航
    float target_yaw_rad_{std::numeric_limits<float>::quiet_NaN()};

    /// Hold 模式命令已發送旗標
    bool hold_cmd_sent_{false};
};

}  // namespace controllers
}  // namespace taiphoon
