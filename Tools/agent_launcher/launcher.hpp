/**
 * @file launcher.hpp
 * @brief 部署設定、映像校驗與 Compose 產生介面
 * @author Taiphoon Agent Team
 * @date 2026-10-02
 */
#pragma once
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace taiphoon::deployment
{
namespace fs = std::filesystem;
using Settings = std::map<std::string, std::string>;

struct ProcessResult
{
    int code = 0;
    std::string output;
};

/** @brief 直接執行 argv；錯誤保留於 stderr。
 * @param arguments 程序與獨立參數。
 * @param should_capture 是否擷取 stdout。
 * @return 程序回傳碼與輸出。
 */
ProcessResult run(const std::vector<std::string>& arguments, bool should_capture = false);
/** @brief 啟動無視窗背景程序，讓容器退出清理不依賴啟動器前景生命週期。
 * @param arguments 程序與獨立參數。
 */
void runDetached(const std::vector<std::string>& arguments);
/** @brief 依 Windows CRT 規則引用 argv。
 * @param value 原始 Unicode 參數。
 * @return 保留空白與反斜線語意的引用結果。
 */
std::wstring quoteWindowsArgument(const std::wstring& value);
/** @brief 依優先順序尋找 CLI；忽略空目錄與非一般檔案。
 * @param directories PATH 目錄及標準安裝目錄。
 * @param name 可執行檔名稱。
 * @return 第一個存在檔案的絕對路徑；找不到則回傳空路徑。
 */
fs::path findExecutable(const std::vector<fs::path>& directories, const std::string& name);
/** @brief 補齊舊設定的預設值並保留進階欄位。 @param settings 原始設定。 @return 完整設定。 */
Settings withDefaultSettings(const Settings& settings = {});
/** @brief 判斷 sim 或 dev 模擬情境。 @param settings 完整設定。 @return 是否啟動 PX4 模擬。 */
bool isSimulation(const Settings& settings);
/** @brief 實機是否由操作人員明確宣告場地無障礙物（obstacle_source=explicit_empty）。
 * @param settings 完整設定。
 * @return 模擬模式一律為 false；實機只有 explicit_empty 為 true。
 */
bool usesExplicitEmptyObstacles(const Settings& settings);
/** @brief 判斷是否產生供同機原生 Taiphoon_QGCS 連線的單播 DDS 設定。
 * @param settings 完整設定。
 * @param is_desktop Docker daemon 是否為 Desktop（容器位於 NAT 之後）。
 * @return Desktop 模擬情境為 true；Linux Engine 與實機維持既有網路。
 */
bool usesHostGcs(const Settings& settings, bool is_desktop);
/** @brief 只讀取 UDP 路由，不傳送封包。 @param peer 目的 IPv4。 @return 本機 IPv4；失敗為空字串。 */
std::string localIpv4ForPeer(const std::string& peer);
/** @brief 尋找原生工具的實際安裝目錄。 @return 可找到同版本 manifest 的目錄。 */
fs::path executableDirectory();
/** @brief 取得使用者設定目錄；環境覆寫供隔離測試使用。 @return 設定目錄絕對路徑。 */
fs::path configurationDirectory();
/** @brief 讀取非 shell 設定。 @param path 設定檔位置。 @return 已驗證語法的 key/value。 */
Settings readSettings(const fs::path& path);
/** @brief 保存設定。 @param path 目標位置。 @param text UTF-8 內容。 @return 無。 */
void writeText(const fs::path& path, const std::string& text);
/** @brief 驗證使用者設定並拒絕未知欄位。 @param settings 完整設定。 @return 無。 */
void validateSettings(const Settings& settings);
/** @brief 正規化 Docker daemon CPU 架構。 @param architecture 原始架構。 @return amd64 或 arm64。 */
std::string normalizeArchitecture(const std::string& architecture);
/** @brief 引用 YAML 字串並避免插值。 @param value 原始 UTF-8 字串。 @return 引用後 YAML。 */
std::string yamlString(const std::string& value);
/** @brief 計算檔案 SHA256。 @param path 檔案位置。 @return 十六進位 SHA256。 */
std::string sha256(const fs::path& path);
/** @brief 依實機／模擬情境產生服務，僅模擬啟用模擬感測資料。
 * @param settings 已驗證設定。
 * @param image 已校驗的本機映像標籤。
 * @param architecture Docker daemon 架構。
 * @param home 本機設定目錄。
 * @param is_desktop 是否需 bridge／UDP 映射。
 * @return 完整 Compose YAML。
 */
std::string renderCompose(const Settings& settings, const std::string& image,
    const std::string& architecture, const fs::path& home, bool is_desktop = false);
/** @brief 產生 CycloneDDS 設定。
 *
 * 原生 GCS 模式（Desktop 模擬）的 Agent 會主動向主機 QGCS 與 dds_peers 中每個 GCS IPv4 的
 * 固定 index 50 discovery 埠送出探索；Linux Engine 模擬仍忽略 dds_peers。
 * @param settings 已驗證設定；原生 GCS 模式的 lan_address 必須已解析為 IPv4。
 * @param is_desktop Docker daemon 是否為 Desktop。
 * @param instance 模擬 Agent 序號，決定原生 GCS 模式的單播埠段。
 * @return DDS XML。
 */
std::string renderCycloneDds(const Settings& settings, bool is_desktop = false, unsigned int instance = 1);
/** @brief 檢查版本化 Release manifest。 @param manifest 套件資訊。 @return 無。 */
void validateManifest(const Settings& manifest);

/** @brief `chronyc -c tracking` 的必要欄位。 */
struct TimeSyncStatus
{
    bool is_valid = false;
    std::string reference_id;
    std::string reference_name;
    unsigned int stratum = 0;
    /** @brief 系統時鐘相對時間來源的誤差（秒），只取絕對值判定。 */
    double system_offset_s = 0.0;
    std::string leap_status;
};

/** @brief 對時判定結果與可直接顯示的原因。 */
struct TimeSyncDecision
{
    bool is_ready = false;
    std::string reason;
};

/** @brief 解析 chrony CSV tracking；欄位不足、非數值或非有限值時 is_valid 為 false。
 * @param csv `chronyc -c tracking` 的標準輸出。
 * @return 解析結果。
 */
TimeSyncStatus parseChronyTracking(const std::string& csv);
/** @brief 判定是否已同步且誤差在門檻內。
 * @param status 已解析的 tracking。
 * @param max_offset_s 允許的系統時鐘誤差（秒），必須為正。
 * @return 是否就緒與原因。
 */
TimeSyncDecision evaluateTimeSync(const TimeSyncStatus& status, double max_offset_s);
/** @brief 將 time_sync 設定解析為實際策略；模擬共用主機時鐘，一律不檢查。
 * @param settings 已驗證設定。
 * @param is_linux_host 是否在 Linux 主機執行（可使用 chrony）。
 * @return required、warn 或 off。
 */
std::string resolveTimeSyncPolicy(const Settings& settings, bool is_linux_host);
}
