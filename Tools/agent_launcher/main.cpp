/**
 * @file main.cpp
 * @brief taiphoon_agent 跨平台啟動、部署與開發命令
 * @author Taiphoon Agent Team
 * @date 2026-10-02
 */
#include "launcher.hpp"
#include <algorithm>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace
{
using taiphoon::deployment::Settings;
namespace deployment = taiphoon::deployment;
namespace fs = std::filesystem;
constexpr unsigned int START_TIMEOUT_SECONDS = 90;
// TEST-NET-1 位址只用於查詢預設路由的本機 IPv4，不會送出封包。
constexpr const char* DEFAULT_ROUTE_PROBE = "192.0.2.1";

#ifdef _WIN32
/** @brief 雙擊 exe 時保留結果視窗；從既有終端執行不額外暫停。 */
class ConsoleLifetime
{
public:
    /** @brief 記錄是否由獨立 Console 啟動。 */
    explicit ConsoleLifetime(bool should_pause) : should_pause_(should_pause)
    {
    }
    /** @brief 讓雙擊使用者能讀取啟動結果或失敗原因。 */
    ~ConsoleLifetime()
    {
        if (should_pause_)
        {
            std::cout << "按 Enter 關閉視窗。" << std::flush;
            std::string line;
            std::getline(std::cin, line);
        }
    }
private:
    bool should_pause_;
};
#endif

/** @brief 執行必要程序，保留其錯誤訊息並傳遞失敗。 */
std::string checked(const std::vector<std::string>& arguments, bool should_capture = false)
{
    const auto result = deployment::run(arguments, should_capture);
    if (result.code != 0)
    {
        if (arguments.front() == "docker" && result.code == 127)
        {
            throw std::runtime_error("找不到 Docker CLI；請安裝 Docker Engine 與 Compose plugin（Ubuntu），"
                "或 Docker Desktop（Windows／macOS），並確認 docker 位於 PATH。設定可以保留。");
        }
        if (arguments.size() >= 2 && arguments[0] == "docker" && arguments[1] == "info")
        {
            throw std::runtime_error("無法連線至 Docker Engine；Windows／macOS 請先開啟 Docker Desktop，"
                "Ubuntu 請啟動 Docker 服務並確認目前使用者有存取權，再重新開啟本工具。設定可以保留。");
        }
        if (arguments == std::vector<std::string>{"docker", "compose", "version"})
        {
            throw std::runtime_error("Docker Compose 不可用；Ubuntu 請安裝 Compose plugin，"
                "Windows／macOS 請檢查 Docker Desktop 安裝是否完整。");
        }
        throw std::runtime_error(arguments.front() + " 執行失敗（" + std::to_string(result.code) + "）");
    }
    std::string output = result.output;
    while (!output.empty() && (output.back() == '\n' || output.back() == '\r'))
    {
        output.pop_back();
    }
    return output;
}

/** @brief 建立固定 argv 前綴，設定值不經 shell 插值。 */
std::vector<std::string> compose(const fs::path& home, const std::vector<std::string>& tail)
{
    std::vector<std::string> arguments = {"docker", "compose", "--project-name", "taiphoon_agent",
        "--project-directory", home.u8string(), "--file", (home / "compose.yaml").u8string()};
    arguments.insert(arguments.end(), tail.begin(), tail.end());
    return arguments;
}

/** @brief 讀取一項互動設定，EOF 明確停止而非採用未確認的部署值。 */
std::string prompt(const std::string& label, const std::string& fallback)
{
    std::cout << label << " [" << fallback << "]: " << std::flush;
    std::string value;
    if (!std::getline(std::cin, value))
    {
        throw std::runtime_error("設定尚未完成；請在互動終端執行 taiphoon_agent init");
    }
    return value.empty() ? fallback : value;
}

/** @brief 避免在服務執行時改寫其模式、映像或資料掛載。 */
void requireStopped(const fs::path& home)
{
    if (fs::exists(home / "compose.yaml")
        && !checked(compose(home, {"ps", "--status", "running", "--quiet"}), true).empty())
    {
        throw std::runtime_error("請先執行 taiphoon_agent stop，再修改設定或升級版本");
    }
}

/** @brief 首次設定只收集模式與必要連線資料；進階 DDS 值保存於設定檔。 */
void initialize(const fs::path& home)
{
    requireStopped(home);
    Settings settings = deployment::withDefaultSettings();
    if (fs::exists(home / "settings.ini"))
    {
        settings = deployment::withDefaultSettings(deployment::readSettings(home / "settings.ini"));
        deployment::validateSettings(settings);
    }
    const auto previous_mode = settings.at("mode");
    settings["mode"] = prompt("模式 deploy／dev／sim", previous_mode);
    if (settings.at("mode") == "dev")
    {
        settings["workspace"] = prompt("Taiphoon_Agent 原始碼目錄",
            settings.at("workspace").empty() ? fs::current_path().u8string() : settings.at("workspace"));
        settings["workspace"] = fs::absolute(fs::u8path(settings.at("workspace"))).u8string();
        settings["scenario"] = prompt("飛控 sim／real",
            previous_mode == "dev" ? settings.at("scenario") : "sim");
    }
    else
    {
        settings["scenario"] = settings.at("mode") == "sim" ? "sim" : "real";
    }
    if (deployment::isSimulation(settings))
    {
        settings["count"] = prompt("PX4＋Agent 組數（1～4）", settings.at("count"));
        settings["transport"] = "udp";
        settings["network"] = "local";
        std::cout << "模擬 Agent 透過 Docker 網路 Taiphoon_Net 互相通訊；Docker Desktop 上由本機原生"
            " Taiphoon_QGCS 經主機區網 IPv4 連線。\n";
    }
    else
    {
        settings["agent_id"] = prompt("Agent 編號（需與飛控 system ID／namespace 一致）", settings.at("agent_id"));
        settings["network"] = "wifi";
        if (settings.at("mode") == "dev")
        {
            settings["transport"] = "udp";
        }
        std::cout << "實機飛控 IPv4：" << settings.at("flight_ip") << "；Agent 主機 IPv4 自動取得。\n";
    }
    if (settings.at("network") == "wifi")
    {
        settings["dds_peers"] = prompt("GCS IPv4（選填，多個以逗號分隔）", settings.at("dds_peers"));
    }
    deployment::validateSettings(settings);
    std::ostringstream text;
    text << "# 本機設定；不會以 shell 執行。進階 DDS 設定：domain_id、lan_address、dds_peers。\n";
    for (const auto& entry : settings)
    {
        text << entry.first << '=' << entry.second << '\n';
    }
    deployment::writeText(home / "settings.ini", text.str());
    std::cout << "已保存 " << (home / "settings.ini").u8string()
        << "\n日常啟動：taiphoon_agent start\n";
}

/** @brief 只檢查可執行的平台與 Compose，不固定 context 或安裝版本。 */
struct DockerInfo
{
    std::string architecture;
    bool is_desktop = false;
};

/** @brief 取得 daemon 平台與 Desktop 網路差異。 @return 已確認的執行環境。 */
DockerInfo checkDocker()
{
    const auto description = checked({"docker", "info", "--format",
        "{{.OSType}}|{{.Architecture}}|{{.OperatingSystem}}"}, true);
    std::istringstream fields(description);
    std::string operating_system;
    std::string architecture;
    std::string distribution;
    std::getline(fields, operating_system, '|');
    std::getline(fields, architecture, '|');
    std::getline(fields, distribution);
    if (operating_system != "linux")
    {
        throw std::runtime_error("請啟動 Docker 的 Linux 容器模式");
    }
    checked({"docker", "compose", "version"}, true);
    bool is_desktop = distribution.find("Docker Desktop") != std::string::npos;
#if defined(_WIN32) || defined(__APPLE__)
    is_desktop = true;
#endif
    return {deployment::normalizeArchitecture(architecture), is_desktop};
}

/** @brief 不同儲存方式的 ID／Config 可能不同，使用全部 layer digest 核對預編譯內容。
 * @param manifest 套件內的映像描述。
 * @param prefix 映像種類與架構。
 * @param image 本機映像標籤。
 * @return 無；無法證明相同內容時停止。
 */
void verifyImageContent(const Settings& manifest, const std::string& prefix, const std::string& image)
{
    const auto actual_id = checked({"docker", "image", "inspect", "--format", "{{.Id}}", image}, true);
    if (actual_id == manifest.at(prefix + "_id"))
    {
        return;
    }
    if (manifest.count(prefix + "_layers") == 0)
    {
        throw std::runtime_error("Docker 映像摘要與套件記錄不同，舊套件缺少跨儲存方式的內容描述；"
            "請下載新版完整平台包。預期 ID：" + manifest.at(prefix + "_id") + "，本機 ID：" + actual_id);
    }
    const auto layers = checked({"docker", "image", "inspect", "--format", "{{json .RootFS.Layers}}", image}, true);
    if (layers != manifest.at(prefix + "_layers"))
    {
        throw std::runtime_error("本機同名映像的檔案層與 Release 不符：" + image
            + "；為避免使用錯誤內容，已停止啟動並保留既有映像");
    }
}

/** @brief 自包含下載包的映像自動載入；不需要私有儲存庫的第二次認證。 */
std::string ensureImage(const Settings& manifest, const std::string& kind,
    const std::string& architecture, const fs::path& package)
{
    const std::string prefix = kind + "_" + architecture;
    const std::string image = "taiphoon-agent:" + manifest.at("version") + "-" + kind + "-" + architecture;
    const std::vector<std::string> inspect = {"docker", "image", "inspect", "--format", "{{.Id}}", image};
    const auto existing = deployment::run(inspect, true);
    if (existing.code == 0)
    {
        verifyImageContent(manifest, prefix, image);
    }
    else
    {
        const std::string filename = "taiphoon-agent-" + kind + "-linux-" + architecture + ".tar.gz";
        const auto archive = package / "images" / filename;
        if (!fs::exists(archive))
        {
            throw std::runtime_error("下載包缺少 " + filename + "；請下載符合 Docker daemon 架構的完整平台包");
        }
        std::cout << "準備套件內的預編譯 Agent 映像（首次載入需要較長時間）...\n";
        if (deployment::sha256(archive) != manifest.at(prefix + "_sha256"))
        {
            throw std::runtime_error("套件內容 SHA256 不符，禁止載入；請重新下載完整平台包");
        }
        checked({"docker", "load", "--input", archive.u8string()});
        verifyImageContent(manifest, prefix, image);
    }
    if (checked({"docker", "image", "inspect", "--format", "{{.Architecture}}", image}, true) != architecture)
    {
        throw std::runtime_error("映像架構與 Docker daemon 不符");
    }
    if (checked({"docker", "image", "inspect", "--format", "{{.Os}}", image}, true) != "linux")
    {
        throw std::runtime_error("映像必須使用 Linux 作業系統");
    }
    return image;
}

/** @brief 開發命令只在 dev 模式的容器執行，部署沒有編譯依賴。 */
void buildWorkspace(const fs::path& home, bool should_test)
{
    std::vector<std::string> arguments = {"run", "--rm", "--no-deps", "--entrypoint", "/bin/bash", "agent_1",
        "-c", "set -e; source /opt/ros/humble/setup.bash; "
        "source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash; "
        "colcon build --executor sequential --packages-select taiphoon_agent "
        "--cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF"};
    if (should_test)
    {
        arguments.back() += "; colcon test --packages-select taiphoon_agent; colcon test-result --verbose";
        arguments.back() += "; cmake -S Tools/agent_launcher -B build/agent_launcher_tests "
            "-DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release; "
            "cmake --build build/agent_launcher_tests --parallel 2; "
            "ctest --test-dir build/agent_launcher_tests --output-on-failure";
    }
    checked(compose(home, arguments));
}

/** @brief 產生 Compose 前確認掛載來源與 release，不變更既有 legacy 啟動流程。 */
void prepare(const Settings& settings, const Settings& manifest, const std::string& architecture,
    const fs::path& home, const fs::path& package, bool is_desktop)
{
    if (settings.at("mode") == "dev")
    {
        const auto workspace = fs::u8path(settings.at("workspace"));
        if (!fs::exists(workspace / "package.xml") || !fs::exists(workspace / "CMakeLists.txt"))
        {
            throw std::runtime_error("原始碼目錄缺少 package.xml／CMakeLists.txt；請重新 init");
        }
    }
    if (settings.at("transport") == "serial")
    {
#if defined(_WIN32) || defined(__APPLE__)
        throw std::runtime_error("Desktop 串列裝置需另行配置 USB 轉接；請先使用 UDP 或樹莓派串列部署");
#else
        if (!fs::exists(settings.at("serial_device")))
        {
            throw std::runtime_error("找不到指定串列裝置");
        }
#endif
    }
    const std::string kind = settings.at("mode") == "dev" ? "dev" : "runtime";
    const auto image = ensureImage(manifest, kind, architecture, package);
    if (deployment::usesHostGcs(settings, is_desktop))
    {
        // 每台 Agent 使用各自的單播埠段，因此各自一份 DDS 設定。
        for (unsigned int instance = 1; instance <= std::stoul(settings.at("count")); ++instance)
        {
            deployment::writeText(home / ("cyclonedds_" + std::to_string(instance) + ".xml"),
                deployment::renderCycloneDds(settings, is_desktop, instance));
        }
    }
    else
    {
        deployment::writeText(home / "cyclonedds.xml", deployment::renderCycloneDds(settings));
    }
    if (deployment::isSimulation(settings))
    {
        if (!fs::exists(package / "sitl-entrypoint.sh"))
        {
            throw std::runtime_error("安裝包缺少 sitl-entrypoint.sh");
        }
        fs::copy_file(package / "sitl-entrypoint.sh", home / "sitl-entrypoint.sh", fs::copy_options::overwrite_existing);
        if (deployment::run({"docker", "image", "inspect", "px4io/px4-sitl:latest"}, true).code != 0)
        {
            checked({"docker", "pull", "--platform", "linux/amd64", "px4io/px4-sitl:latest"});
        }
        if (architecture == "arm64")
        {
            std::cout << "PX4 SITL 使用 AMD64 模擬；ARM64 需要 Docker 的 CPU 模擬支援，效能需另行驗證。\n";
        }
    }
    deployment::writeText(home / "compose.yaml",
        deployment::renderCompose(settings, image, architecture, home, is_desktop));
    deployment::writeText(home / "active-release.ini", "version=" + manifest.at("version") + "\narchitecture="
        + architecture + "\nmode=" + settings.at("mode") + "\nsettings_sha256="
        + deployment::sha256(home / "settings.ini") + "\n");
    checked(compose(home, {"config", "--quiet"}));
}

/** @brief 顯示程序健康與已收到的飛控狀態，避免將 running 當作飛行就緒。 */
void status(const fs::path& home, const Settings& settings)
{
    checked(compose(home, {"ps", "--all"}));
    const auto count = deployment::isSimulation(settings) ? std::stoul(settings.at("count")) : 1;
    for (unsigned int instance = 1; instance <= count; ++instance)
    {
        const auto id = deployment::isSimulation(settings) ? std::to_string(instance) : settings.at("agent_id");
        const auto service = "agent_" + std::to_string(instance);
        const auto observation = deployment::run(compose(home, {"exec", "-T", service, "/bin/bash", "-c",
            "source /opt/ros/humble/setup.bash; "
            "source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash; "
            "source /home/taiphoon/Taiphoon_Agent/install/setup.bash; "
            "timeout 5 ros2 topic echo /uav_" + id + "_agent/state taiphoon_agent/msg/AgentState "
            "--no-daemon --once --field px4_state_valid"}), true);
        std::cout << service << ": " << (observation.code == 0 && observation.output.find("True") != std::string::npos
            ? "已收到有效 PX4 狀態（仍需確認定位與感測器）" : "等待有效 PX4 狀態／服務未執行") << '\n';
    }
}

/** @brief 重複啟動不改動健康服務，部分失效則要求明確 restart。 */
void requireHealthyAgents(const fs::path& home, const Settings& settings)
{
    const auto count = deployment::isSimulation(settings) ? std::stoul(settings.at("count")) : 1;
    for (unsigned int instance = 1; instance <= count; ++instance)
    {
        const auto service = "agent_" + std::to_string(instance);
        const auto container = checked(compose(home, {"ps", "--all", "--quiet", service}), true);
        if (container.empty()
            || checked({"docker", "inspect", "--format", "{{.State.Status}} {{.State.Health.Status}}", container}, true)
                != "running healthy")
        {
            throw std::runtime_error(service + " 尚未健康；請查看 logs，確認後使用 restart");
        }
        if (deployment::isSimulation(settings)
            && checked(compose(home, {"ps", "--status", "running", "--quiet", "px4_"
                + std::to_string(instance)}), true).empty())
        {
            throw std::runtime_error("PX4 模擬器尚未執行；請查看 logs，確認後使用 restart");
        }
    }
}

/** @brief 拒絕與既有入口重複管理同一飛控；不自動停止其他功能。 */
void requireNoLegacyAgents(const Settings& settings)
{
    const auto names = checked({"docker", "ps", "--format", "{{.Names}}"}, true);
    std::istringstream stream(names);
    std::set<std::string> active;
    std::string name;
    while (std::getline(stream, name))
    {
        active.insert(name);
    }
    const auto count = deployment::isSimulation(settings) ? std::stoul(settings.at("count")) : 1;
    for (unsigned int instance = 1; instance <= count; ++instance)
    {
        const auto id = deployment::isSimulation(settings) ? std::to_string(instance) : settings.at("agent_id");
        if (active.count("uav_" + id + "_agent") != 0 || active.count("fmu" + id + "_agent") != 0)
        {
            throw std::runtime_error("已有其他入口的 Agent 管理 uav_" + id
                + "；請確認並停止既有服務後再使用新入口");
        }
    }
}

/** @brief 為此 Compose 專案執行中的完整容器 ID 啟動退出清理。
 * @param home 此次部署的設定目錄。
 */
void watchContainers(const fs::path& home)
{
    const auto ids = checked(compose(home, {"ps", "--status", "running", "--quiet"}), true);
    std::istringstream containers(ids);
    std::string container_id;
    while (containers >> container_id)
    {
        // 只綁定完整 ID，不以名稱刪除後來建立的替代容器。
        if (container_id.size() == 64 && container_id.find_first_not_of("0123456789abcdef") == std::string::npos)
        {
#ifdef _WIN32
            const auto watcher = deployment::executableDirectory() / "taiphoon_agent.exe";
#else
            const auto watcher = deployment::executableDirectory() / "taiphoon_agent";
#endif
            deployment::runDetached({watcher.u8string(), "--watch-container", container_id});
        }
    }
}

/** @brief 顯示各平台相同的最小操作入口。 */
void help()
{
    std::cout << "Taiphoon Agent " << TAIPHOON_RELEASE_VERSION << "\n"
        << "首次：taiphoon_agent init\n日常：taiphoon_agent start\n"
        << "命令：init、config、start、stop、restart、status、logs、build、test、shell\n"
        << "開發：build／test／shell 需要 dev 模式；Agent 與 ROS2 在 Docker Linux 容器執行。\n"
        << "首次自動載入套件內的 Release 映像，後續使用本機映像。啟動不會 Arm／起飛。\n";
}
}

/** @brief 原生入口；任何依賴或設定失敗以非零狀態回傳。 */
int main(int argc, char* argv[])
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    DWORD console_processes[2] = {};
    ConsoleLifetime console_lifetime(argc == 1 && GetConsoleProcessList(console_processes, 2) == 1);
#endif
    try
    {
        const std::string command = argc == 1 ? "start" : argv[1];
        if (command == "--watch-container" && argc == 3)
        {
            const std::string container_id = argv[2];
            if (container_id.size() != 64 || container_id.find_first_not_of("0123456789abcdef") != std::string::npos)
            {
                throw std::runtime_error("容器清理需要完整 Docker container ID");
            }
            const auto waited = deployment::run({"docker", "wait", container_id}, true);
            if (waited.code != 0)
            {
                return waited.code;
            }
            const auto state = deployment::run({"docker", "inspect", "--format", "{{.State.Running}}", container_id}, true);
            if (state.code == 0 && state.output.find("false") != std::string::npos)
            {
                return deployment::run({"docker", "rm", container_id}, true).code;
            }
            return state.code;
        }
        if (argc > 2)
        {
            throw std::runtime_error("命令不需要額外參數；請使用 taiphoon_agent config 設定");
        }
        if (command == "--help" || command == "help" || command == "-h")
        {
            help();
            return 0;
        }
        if (command == "--version")
        {
            std::cout << TAIPHOON_RELEASE_VERSION << '\n';
            return 0;
        }
        const std::vector<std::string> known = {"init", "config", "start", "stop", "restart", "status", "logs",
            "build", "test", "shell"};
        if (std::find(known.begin(), known.end(), command) == known.end())
        {
            throw std::runtime_error("未知命令：" + command + "；請使用 --help");
        }
        const auto home = deployment::configurationDirectory();
        if (command == "init" || command == "config")
        {
            initialize(home);
            return 0;
        }
        DockerInfo docker;
        if (!fs::exists(home / "settings.ini"))
        {
            if (command != "start")
            {
                throw std::runtime_error("尚未設定；請執行 taiphoon_agent init");
            }
            docker = checkDocker();
            initialize(home);
        }
        auto settings = deployment::withDefaultSettings(deployment::readSettings(home / "settings.ini"));
        deployment::validateSettings(settings);
        if (docker.architecture.empty())
        {
            docker = checkDocker();
        }
        const auto& architecture = docker.architecture;
        if (command == "stop" || command == "logs" || command == "status" || command == "shell")
        {
            if (!fs::exists(home / "compose.yaml"))
            {
                throw std::runtime_error("尚未啟動；請先執行 taiphoon_agent start");
            }
            if (command == "status")
            {
                status(home, settings);
            }
            else if (command == "shell")
            {
                if (settings.at("mode") != "dev")
                {
                    throw std::runtime_error("shell 需要 dev 模式");
                }
                checked(compose(home, {"exec", "agent_1", "/bin/bash"}));
            }
            else
            {
                checked(compose(home, command == "stop" ? std::vector<std::string>{"down", "--timeout", "20"}
                    : std::vector<std::string>{"logs", "--tail", "150"}));
            }
            return 0;
        }
        const auto package = deployment::executableDirectory();
        const auto manifest = deployment::readSettings(package / "release.manifest");
        deployment::validateManifest(manifest);
        if (manifest.at("version") != TAIPHOON_RELEASE_VERSION)
        {
            throw std::runtime_error("啟動工具與 release.manifest 版本不符；請重新安裝完整 Release 包");
        }
        if ((command == "build" || command == "test") && settings.at("mode") != "dev")
        {
            throw std::runtime_error("build／test 需要 dev 模式");
        }
        if (command == "start" || command == "restart")
        {
            requireNoLegacyAgents(settings);
        }
        if (command == "restart" && fs::exists(home / "compose.yaml"))
        {
            checked(compose(home, {"down", "--timeout", "20"}));
        }
        const bool is_running = fs::exists(home / "compose.yaml")
            && !checked(compose(home, {"ps", "--status", "running", "--quiet"}), true).empty();
        if (is_running)
        {
            const auto active = deployment::readSettings(home / "active-release.ini");
            if (active.at("version") != manifest.at("version") || active.at("architecture") != architecture
                || active.at("mode") != settings.at("mode")
                || active.at("settings_sha256") != deployment::sha256(home / "settings.ini"))
            {
                throw std::runtime_error("執行中服務的模式／版本不同；請先 stop 再 start");
            }
            if (command == "start")
            {
                requireHealthyAgents(home, settings);
                watchContainers(home);
                std::cout << "服務已在執行；飛控連線可用 taiphoon_agent status 查看。\n";
                return 0;
            }
        }
        else
        {
            if (!deployment::isSimulation(settings) && settings.at("network") == "wifi")
            {
                if (settings.at("lan_address").empty() || settings.at("lan_address") == "auto")
                {
                    const auto peer = settings.at("dds_peers").empty() ? settings.at("flight_ip")
                        : settings.at("dds_peers").substr(0, settings.at("dds_peers").find(','));
                    settings["lan_address"] = deployment::localIpv4ForPeer(peer);
                }
                std::cout << "Wi-Fi Agent 主機 IPv4：" << (settings.at("lan_address").empty()
                    ? "未取得；使用 DDS 自動探索" : settings.at("lan_address")) << '\n';
            }
            else if (deployment::usesHostGcs(settings, docker.is_desktop))
            {
                if (settings.at("lan_address").empty() || settings.at("lan_address") == "auto")
                {
                    settings["lan_address"] = deployment::localIpv4ForPeer(DEFAULT_ROUTE_PROBE);
                }
                if (settings.at("lan_address").empty() || settings.at("lan_address").rfind("127.", 0) == 0)
                {
                    throw std::runtime_error("無法取得主機區網 IPv4；請確認網路已連線，"
                        "或在 settings.ini 指定 lan_address 後重新啟動");
                }
                settings["dds_peers"] = "";
                std::cout << "本機 Taiphoon_QGCS 連線用主機 IPv4：" << settings.at("lan_address")
                    << "（多網卡選錯時請在 settings.ini 指定 lan_address）\n";
            }
            else
            {
                settings["lan_address"] = "";
                settings["dds_peers"] = "";
            }
            if (!deployment::isSimulation(settings) && settings.at("transport") == "udp")
            {
                std::cout << "飛控 " << settings.at("flight_ip") << " 的 uXRCE-DDS client 需連向 Agent 主機 UDP "
                    << 8887 + std::stoi(settings.at("agent_id")) << "，namespace /uav_"
                    << settings.at("agent_id") << "。\n";
            }
            prepare(settings, manifest, architecture, home, package, docker.is_desktop);
        }
        if (settings.at("mode") == "dev")
        {
            buildWorkspace(home, command == "test");
        }
        if (command == "build" || command == "test")
        {
            return 0;
        }
        checked(compose(home, {"up", "--detach", "--no-build", "--wait", "--wait-timeout",
            std::to_string(START_TIMEOUT_SECONDS)}));
        watchContainers(home);
        std::cout << "服務程序已啟動；不會自動 Arm／起飛。\n";
        if (deployment::usesHostGcs(settings, docker.is_desktop) && !settings.at("lan_address").empty()
            && settings.at("lan_address") != "auto")
        {
            std::cout << "本機 Taiphoon_QGCS 啟動參數（ROS domain " << settings.at("domain_id") << "）：\n"
                << "  --interface " << settings.at("lan_address") << " --external-address 192.168.65.254"
                << " --no-multicast --peer " << settings.at("lan_address") << " --participant-index 50\n";
        }
        std::cout << "使用 taiphoon_agent status 查看飛控連線，logs 查看日誌。\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "taiphoon_agent: " << error.what() << '\n';
        return 1;
    }
}
