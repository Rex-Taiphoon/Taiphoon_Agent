/**
 * @file agent_launcher_test.cpp
 * @brief 跨平台部署入口的設定、校驗、模式與 argv 回歸測試
 * @author Taiphoon Agent Team
 * @date 2026-10-02
 */
#include "launcher.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace
{
namespace deployment = taiphoon::deployment;
namespace fs = std::filesystem;

/** @brief 所有組態均執行檢查，不依賴會被 NDEBUG 移除的 assert。 */
void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

/** @brief 確認非法輸入遭拒絕，避免接受數字前綴或注入片段。 */
template<typename Function>
void requireRejected(Function operation)
{
    bool is_rejected = false;
    try
    {
        operation();
    }
    catch (const std::exception&)
    {
        is_rejected = true;
    }
    require(is_rejected, "非法輸入未被拒絕");
}

/** @brief 建立完整合法設定以測試單一欄位的異常。 */
deployment::Settings defaults()
{
    return {{"mode", "deploy"}, {"agent_id", "1"}, {"count", "4"}, {"transport", "udp"},
        {"serial_device", ""}, {"serial_baud", "921600"}, {"domain_id", "0"}, {"workspace", ""},
        {"lan_address", ""}, {"dds_peers", ""}};
}
}

/** @brief 測試全部在 Docker 開發容器內執行，測試資料不放入 src。 */
int main()
{
    try
    {
        const auto directory = fs::temp_directory_path() / ("taiphoon-launcher-tests-"
            + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        auto settings = defaults();
        deployment::validateSettings(settings);
        for (const auto& invalid : {"0", "256", "1;exit", "1\nother=1"})
        {
            settings["agent_id"] = invalid;
            requireRejected([&settings] { deployment::validateSettings(settings); });
        }
        settings = defaults();
        settings["unknown"] = "x";
        requireRejected([&settings] { deployment::validateSettings(settings); });
        settings = defaults();
        settings["lan_address"] = "192.168.1.999";
        requireRejected([&settings] { deployment::validateSettings(settings); });
        settings["lan_address"] = "192.168.1.20";
        settings["dds_peers"] = "192.168.1.21,192.168.1.22";
        const auto xml = deployment::renderCycloneDds(settings);
        require(xml.find("ExternalNetworkAddress>192.168.1.20") != std::string::npos, "公告位址遺失");
        require(xml.find("Address=\"192.168.1.22\"") != std::string::npos, "peer 遺失");
        deployment::writeText(directory / "cyclonedds.xml", xml);

        deployment::writeText(directory / "data", "abc");
        require(deployment::sha256(directory / "data")
            == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", "SHA256 abc 錯誤");
        deployment::writeText(directory / "data", "");
        require(deployment::sha256(directory / "data")
            == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", "SHA256 空內容錯誤");
        deployment::writeText(directory / "data", std::string(1000000, 'a'));
        require(deployment::sha256(directory / "data")
            == "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0", "SHA256 多區塊錯誤");
        deployment::writeText(directory / "settings", "mode=deploy\r\nmode=sim\r\n");
        requireRejected([&directory] { deployment::readSettings(directory / "settings"); });

        settings = defaults();
        const auto deploy = deployment::renderCompose(settings, "taiphoon-agent:test", "arm64", directory);
        require(deploy.find("network_mode: host") != std::string::npos, "部署未使用 host 網路");
        require(deploy.find("ENABLE_OBSTACLE_CLOUD_SIMULATOR: \"false\"") != std::string::npos,
            "部署錯誤啟用模擬感測器");
        require(deploy.find("source: \"/source") == std::string::npos, "部署掛載原始碼");
        settings["mode"] = "dev";
        settings["workspace"] = "C:\\中文 source\\$workspace";
        const auto dev = deployment::renderCompose(settings, "taiphoon-agent:test", "amd64", directory);
        require(dev.find("build_amd64") != std::string::npos && dev.find("install_amd64") != std::string::npos,
            "開發產物未依 CPU 架構隔離");
        require(dev.find("$$workspace") != std::string::npos, "Compose 意外插值環境變數");
        settings = defaults();
        settings["mode"] = "sim";
        const auto sim = deployment::renderCompose(settings, "taiphoon-agent:test", "arm64", directory);
        require(sim.find("agent_4:") != std::string::npos && sim.find("px4_4:") != std::string::npos,
            "四機服務不完整");
        require(sim.find("condition: service_healthy") != std::string::npos, "模擬器未等待 Agent");
        require(sim.find("network_mode: host") == std::string::npos, "模擬網路未隔離");
        require(sim.find("auto_start_sequence:=true") == std::string::npos, "錯誤啟用自動起飛");
        settings["transport"] = "serial";
        settings["serial_device"] = "/dev/ttyACM0";
        requireRejected([&settings] { deployment::validateSettings(settings); });
        settings["mode"] = "deploy";
        const auto serial = deployment::renderCompose(settings, "taiphoon-agent:test", "arm64", directory);
        require(serial.find("devices:") != std::string::npos, "串列裝置未映射");
        settings["serial_device"] = "/dev/../etc/passwd";
        requireRejected([&settings] { deployment::validateSettings(settings); });

        require(deployment::normalizeArchitecture("aarch64\n") == "arm64", "ARM64 正規化錯誤");
        require(deployment::normalizeArchitecture("x86_64") == "amd64", "AMD64 正規化錯誤");
        requireRejected([] { deployment::normalizeArchitecture("armv7l"); });
        require(deployment::quoteWindowsArgument(L"C:\\path with spaces\\")
            == L"\"C:\\path with spaces\\\\\"", "Windows 尾端反斜線錯誤");
        require(deployment::quoteWindowsArgument(L"a\"b") == L"\"a\\\"b\"", "Windows 引號錯誤");
        require(deployment::quoteWindowsArgument(L"") == L"\"\"", "Windows 空參數錯誤");
#ifndef _WIN32
        const auto process = deployment::run({"/bin/printf", "%s", "$(exit 5); space ' \" 中文"}, true);
        require(process.code == 0 && process.output == "$(exit 5); space ' \" 中文", "argv 遭 shell 解譯");
        require(deployment::run({"/bin/sh", "-c", "exit 7"}, true).code == 7, "失敗碼未傳遞");
#endif
        deployment::writeText(directory / "deploy.yaml", deploy);
        deployment::writeText(directory / "dev.yaml", dev);
        deployment::writeText(directory / "sim.yaml", sim);
        deployment::writeText(directory / "serial.yaml", serial);
        std::cout << "PASS: 設定拒絕、SHA256、模式隔離、DDS、串列、Windows argv、程序失敗\n"
            << "測試產物：" << directory.u8string() << '\n';
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
