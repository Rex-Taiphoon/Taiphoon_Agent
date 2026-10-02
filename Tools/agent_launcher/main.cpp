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
    Settings settings = {{"mode", "deploy"}, {"agent_id", "1"}, {"count", "4"}, {"transport", "udp"},
        {"serial_device", ""}, {"serial_baud", "921600"}, {"domain_id", "0"}, {"workspace", ""},
        {"lan_address", ""}, {"dds_peers", ""}};
    if (fs::exists(home / "settings.ini"))
    {
        settings = deployment::readSettings(home / "settings.ini");
        deployment::validateSettings(settings);
    }
    settings["mode"] = prompt("模式 deploy／dev／sim", settings.at("mode"));
    if (settings.at("mode") == "sim")
    {
        settings["count"] = prompt("模擬機數", settings.at("count"));
        settings["transport"] = "udp";
    }
    else
    {
        settings["agent_id"] = prompt("Agent 編號（需與飛控 system ID／namespace 一致）", settings.at("agent_id"));
        if (settings.at("mode") == "deploy")
        {
            settings["transport"] = prompt("飛控連線 udp／serial", settings.at("transport"));
            if (settings.at("transport") == "serial")
            {
                settings["serial_device"] = prompt("Linux 串列裝置", "/dev/ttyACM0");
                settings["serial_baud"] = prompt("串列 baud rate", settings.at("serial_baud"));
            }
        }
        else if (settings.at("mode") == "dev")
        {
            settings["transport"] = "udp";
            settings["workspace"] = prompt("Taiphoon_Agent 原始碼目錄", fs::current_path().u8string());
            settings["workspace"] = fs::absolute(fs::u8path(settings.at("workspace"))).u8string();
        }
#if defined(_WIN32) || defined(__APPLE__)
        std::cout << "實機／LAN 通訊需 Docker Desktop 4.34+ 並啟用 host networking。\n";
        settings["lan_address"] = prompt("本機 LAN IPv4（需要公告給其他 ROS2 主機）",
            settings.at("lan_address"));
        settings["dds_peers"] = prompt("DDS peer IPv4（逗號分隔；同網段可留空）", settings.at("dds_peers"));
#endif
        if (settings.at("transport") == "udp")
        {
            std::cout << "飛控 uXRCE-DDS client 應連到本機 UDP "
                << 8887 + std::stoi(settings.at("agent_id")) << "，namespace 為 /uav_"
                << settings.at("agent_id") << "。\n";
        }
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

/** @brief 選擇 Docker daemon 架構，拒絕 Windows 容器與遠端 daemon 的本機掛載。 */
std::string checkDocker()
{
    const auto operating_system = checked({"docker", "info", "--format", "{{.OSType}}"}, true);
    if (operating_system != "linux")
    {
        throw std::runtime_error("請啟動 Docker 的 Linux 容器模式");
    }
    checked({"docker", "compose", "version"}, true);
    const auto endpoint = checked({"docker", "context", "inspect", "--format",
        "{{.Endpoints.docker.Host}}"}, true);
    if (endpoint.rfind("ssh:", 0) == 0 || endpoint.rfind("tcp:", 0) == 0)
    {
        throw std::runtime_error("請使用本機 Docker context；本工具的檔案掛載不支援遠端 daemon");
    }
    return deployment::normalizeArchitecture(checked({"docker", "info", "--format", "{{.Architecture}}"}, true));
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
        if (checked(inspect, true) != manifest.at(prefix + "_id"))
        {
            throw std::runtime_error("本機同名映像與 Release 不符；請使用正確版本，工具不會覆寫該映像");
        }
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
        if (checked(inspect, true) != manifest.at(prefix + "_id"))
        {
            throw std::runtime_error("載入後的映像 ID 與 Release 不符");
        }
    }
    if (checked({"docker", "image", "inspect", "--format", "{{.Architecture}}", image}, true) != architecture)
    {
        throw std::runtime_error("映像架構與 Docker daemon 不符");
    }
    return image;
}

/** @brief 開發命令只在 dev 模式的容器執行，部署沒有編譯依賴。 */
void buildWorkspace(const fs::path& home, bool should_test)
{
    std::vector<std::string> arguments = {"run", "--rm", "--no-deps", "--entrypoint", "/bin/bash", "agent_1",
        "-c", "set -e; source /opt/ros/humble/setup.bash; "
        "source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash; "
        "colcon build --symlink-install --executor sequential --packages-select taiphoon_agent"};
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
    const fs::path& home, const fs::path& package)
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
    deployment::writeText(home / "cyclonedds.xml", deployment::renderCycloneDds(settings));
    if (settings.at("mode") == "sim")
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
    deployment::writeText(home / "compose.yaml", deployment::renderCompose(settings, image, architecture, home));
    deployment::writeText(home / "active-release.ini", "version=" + manifest.at("version") + "\narchitecture="
        + architecture + "\nmode=" + settings.at("mode") + "\nsettings_sha256="
        + deployment::sha256(home / "settings.ini") + "\n");
    checked(compose(home, {"config", "--quiet"}));
}

/** @brief 顯示程序健康與已收到的飛控狀態，避免將 running 當作飛行就緒。 */
void status(const fs::path& home, const Settings& settings)
{
    checked(compose(home, {"ps", "--all"}));
    const auto count = settings.at("mode") == "sim" ? std::stoul(settings.at("count")) : 1;
    for (unsigned int instance = 1; instance <= count; ++instance)
    {
        const auto id = settings.at("mode") == "sim" ? std::to_string(instance) : settings.at("agent_id");
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
    const auto count = settings.at("mode") == "sim" ? std::stoul(settings.at("count")) : 1;
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
        if (settings.at("mode") == "sim"
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
    const auto count = settings.at("mode") == "sim" ? std::stoul(settings.at("count")) : 1;
    for (unsigned int instance = 1; instance <= count; ++instance)
    {
        const auto id = settings.at("mode") == "sim" ? std::to_string(instance) : settings.at("agent_id");
        if (active.count("uav_" + id + "_agent") != 0 || active.count("fmu" + id + "_agent") != 0)
        {
            throw std::runtime_error("已有其他入口的 Agent 管理 uav_" + id
                + "；請確認並停止既有服務後再使用新入口");
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
        if (!fs::exists(home / "settings.ini"))
        {
            if (command != "start")
            {
                throw std::runtime_error("尚未設定；請執行 taiphoon_agent init");
            }
            initialize(home);
        }
        const auto settings = deployment::readSettings(home / "settings.ini");
        deployment::validateSettings(settings);
        const auto architecture = checkDocker();
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
                checked(compose(home, command == "stop" ? std::vector<std::string>{"stop", "--timeout", "20"}
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
            checked(compose(home, {"stop", "--timeout", "20"}));
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
                status(home, settings);
                return 0;
            }
        }
        else
        {
            prepare(settings, manifest, architecture, home, package);
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
        std::cout << "服務程序已啟動；不會自動 Arm／起飛。\n";
        status(home, settings);
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "taiphoon_agent: " << error.what() << '\n';
        return 1;
    }
}
