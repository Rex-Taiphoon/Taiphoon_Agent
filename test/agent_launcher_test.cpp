/**
 * @file agent_launcher_test.cpp
 * @brief 跨平台部署入口的設定、校驗、模式與 argv 回歸測試
 * @author Taiphoon Agent Team
 * @date 2026-10-02
 */
#include "launcher.hpp"
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

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

/** @brief 依 chronyc -c tracking 欄位順序組出測試資料。 */
std::string tracking(const std::string& reference_id, const std::string& name, const std::string& stratum,
    const std::string& offset, const std::string& leap_status)
{
    return reference_id + "," + name + "," + stratum + ",1760000000.123456789," + offset
        + ",-0.000003000,0.000010000,-12.345,0.001,0.050,0.003000000,0.000500000,4.0," + leap_status + "\n";
}

/** @brief obstacle_source：實機預設不提供地圖、explicit_empty 才啟用，模擬不受影響，非法值拒絕。 */
void testObstacleSource()
{
    const fs::path home = fs::temp_directory_path();
    auto settings = deployment::withDefaultSettings(defaults());
    require(settings.at("obstacle_source") == "none", "obstacle_source 缺少安全預設值");
    deployment::validateSettings(settings);
    require(!deployment::usesExplicitEmptyObstacles(settings), "預設錯誤宣告無障礙物");
    const auto safe = deployment::renderCompose(settings, "taiphoon-agent:test", "arm64", home);
    require(safe.find("ENABLE_OBSTACLE_CLOUD_SIMULATOR: \"false\"") != std::string::npos,
        "實機預設錯誤啟用空地圖");

    settings["obstacle_source"] = "explicit_empty";
    deployment::validateSettings(settings);
    require(deployment::usesExplicitEmptyObstacles(settings), "explicit_empty 未生效");
    const auto declared = deployment::renderCompose(settings, "taiphoon-agent:test", "arm64", home);
    require(declared.find("ENABLE_OBSTACLE_CLOUD_SIMULATOR: \"true\"") != std::string::npos
        && declared.find("OBSTACLE_CLOUD_MODE: explicit_empty") != std::string::npos,
        "explicit_empty 未啟用空地圖模擬器");
    settings["mode"] = "dev";
    settings["workspace"] = "/workspace";
    require(deployment::usesExplicitEmptyObstacles(settings), "dev real 未套用 explicit_empty");

    // 模擬模式維持原本的模擬器設定，不受實機宣告影響。
    auto simulation = deployment::withDefaultSettings(defaults());
    simulation["mode"] = "sim";
    simulation["obstacle_source"] = "none";
    require(!deployment::usesExplicitEmptyObstacles(simulation), "模擬模式錯誤套用實機宣告");
    require(deployment::renderCompose(simulation, "taiphoon-agent:test", "arm64", home)
        .find("ENABLE_OBSTACLE_CLOUD_SIMULATOR: \"true\"") != std::string::npos, "模擬模式關閉了模擬器");

    for (const auto& invalid : {"", "empty", "EXPLICIT_EMPTY", "sensor"})
    {
        settings = deployment::withDefaultSettings(defaults());
        settings["obstacle_source"] = invalid;
        requireRejected([&settings] { deployment::validateSettings(settings); });
    }
}

/** @brief chrony 解析、判定與 time_sync 策略的正常、邊界與異常輸入。 */
void testTimeSync()
{
    constexpr double MAX_OFFSET_S = 0.05;
    const auto client = deployment::parseChronyTracking(
        tracking("c0a80165", "192.168.1.101", "9", "0.000012345", "Normal"));
    require(client.is_valid && client.reference_id == "C0A80165" && client.stratum == 9, "客戶端 tracking 解析錯誤");
    const auto client_decision = deployment::evaluateTimeSync(client, MAX_OFFSET_S);
    require(client_decision.is_ready && client_decision.reason.find("192.168.1.101") != std::string::npos,
        "已同步客戶端未通過");

    const auto master = deployment::parseChronyTracking(tracking("7F7F0101", "", "8", "0.000000000", "Normal"));
    const auto master_decision = deployment::evaluateTimeSync(master, MAX_OFFSET_S);
    require(master_decision.is_ready && master_decision.reason.find("本機基準") != std::string::npos,
        "時間主機的本機基準未通過");

    const auto unsynced = deployment::parseChronyTracking(
        tracking("00000000", "", "0", "0.000000000", "Not synchronised"));
    require(unsynced.is_valid && !deployment::evaluateTimeSync(unsynced, MAX_OFFSET_S).is_ready,
        "未同步狀態被接受");
    require(!deployment::evaluateTimeSync(deployment::parseChronyTracking(
        tracking("C0A80165", "192.168.1.101", "16", "0.0", "Normal")), MAX_OFFSET_S).is_ready, "stratum 16 被接受");
    require(deployment::evaluateTimeSync(deployment::parseChronyTracking(
        tracking("C0A80165", "192.168.1.101", "9", "0.0", "Insert second")), MAX_OFFSET_S).is_ready,
        "閏秒公告狀態被誤判為未同步");

    require(deployment::evaluateTimeSync(deployment::parseChronyTracking(
        tracking("C0A80165", "192.168.1.101", "9", "0.050", "Normal")), MAX_OFFSET_S).is_ready, "誤差等於門檻被拒絕");
    require(!deployment::evaluateTimeSync(deployment::parseChronyTracking(
        tracking("C0A80165", "192.168.1.101", "9", "0.0501", "Normal")), MAX_OFFSET_S).is_ready, "超過門檻被接受");
    require(!deployment::evaluateTimeSync(deployment::parseChronyTracking(
        tracking("C0A80165", "192.168.1.101", "9", "-0.060", "Normal")), MAX_OFFSET_S).is_ready, "負向誤差未取絕對值");

    for (const std::string offset : {"nan", "inf", "-inf", "0.01x", ""})
    {
        require(!deployment::parseChronyTracking(tracking("C0A80165", "a", "9", offset, "Normal")).is_valid,
            "非法 System time 被接受：" + offset);
    }
    require(!deployment::parseChronyTracking("").is_valid, "空輸出被接受");
    require(!deployment::parseChronyTracking("C0A80165,a,9,0,0,0,0,0,0,0,0,0,Normal").is_valid, "欄位不足被接受");
    require(!deployment::parseChronyTracking(tracking("C0A80165", "a", "9", "0.0", "Normal")
        + tracking("C0A80165", "a", "9", "0.0", "Normal")).is_valid, "多行輸出被接受");
    require(!deployment::parseChronyTracking(tracking("XYZ", "a", "9", "0.0", "Normal")).is_valid,
        "非法 Reference ID 被接受");
    require(!deployment::evaluateTimeSync(client, 0.0).is_ready, "零門檻被接受");
    require(!deployment::evaluateTimeSync(client, std::numeric_limits<double>::quiet_NaN()).is_ready,
        "NaN 門檻被接受");
    require(!deployment::evaluateTimeSync(deployment::TimeSyncStatus{}, MAX_OFFSET_S).is_ready, "無效狀態被接受");

    auto settings = deployment::withDefaultSettings(defaults());
    require(settings.at("time_sync") == "auto" && settings.at("time_sync_max_offset_ms") == "50",
        "對時設定缺少預設值");
    deployment::validateSettings(settings);
    require(deployment::resolveTimeSyncPolicy(settings, true) == "required", "Linux 實機未強制對時");
    require(deployment::resolveTimeSyncPolicy(settings, false) == "warn", "非 Linux 實機未降為警告");
    settings["time_sync"] = "off";
    require(deployment::resolveTimeSyncPolicy(settings, true) == "off", "time_sync=off 未生效");
    settings["time_sync"] = "required";
    deployment::validateSettings(settings);
    require(deployment::resolveTimeSyncPolicy(settings, false) == "required", "time_sync=required 未生效");
    settings["mode"] = "sim";
    require(deployment::resolveTimeSyncPolicy(settings, true) == "off", "模擬共用主機時鐘卻要求對時");
    settings = deployment::withDefaultSettings(defaults());
    for (const auto& invalid : {"maybe", "", "REQUIRED"})
    {
        settings["time_sync"] = invalid;
        requireRejected([&settings] { deployment::validateSettings(settings); });
    }
    settings = deployment::withDefaultSettings(defaults());
    for (const auto& invalid : {"0", "1001", "-1", "1.5"})
    {
        settings["time_sync_max_offset_ms"] = invalid;
        requireRejected([&settings] { deployment::validateSettings(settings); });
    }
    settings["time_sync_max_offset_ms"] = "1000";
    deployment::validateSettings(settings);
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
        deployment::validateSettings(settings);
        require(deployment::withDefaultSettings(settings).at("unknown") == "x", "進階欄位未保留");
        auto migrated = deployment::withDefaultSettings({{"mode", "sim"}, {"count", "1"}});
        deployment::validateSettings(migrated);
        require(migrated.at("network") == "local" && deployment::isSimulation(migrated), "舊設定未補預設值");
        require(deployment::localIpv4ForPeer("127.0.0.1") == "127.0.0.1", "無封包路由查詢失敗");
        require(deployment::localIpv4ForPeer("not-an-ip").empty(), "非法路由目的未忽略");
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
        const auto path_directory = directory / "PATH tools";
        const auto fallback_directory = directory / fs::u8path("中文 Docker Desktop") / "resources" / "bin";
        deployment::writeText(fallback_directory / "docker.exe", "fixture");
        require(deployment::findExecutable({{}, path_directory, fallback_directory}, "docker.exe")
            == fs::absolute(fallback_directory / "docker.exe"), "缺少 PATH 時未使用安裝目錄備援");
        deployment::writeText(path_directory / "docker.exe", "fixture");
        require(deployment::findExecutable({path_directory, fallback_directory}, "docker.exe")
            == fs::absolute(path_directory / "docker.exe"), "未保留 PATH 優先順序");
        require(deployment::findExecutable({directory / "missing"}, "docker.exe").empty(),
            "不存在的 Docker CLI 被接受");
        fs::create_directories(directory / "not-a-file" / "docker.exe");
        require(deployment::findExecutable({directory / "not-a-file"}, "docker.exe").empty(),
            "目錄被錯認為 Docker CLI");
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

        deployment::Settings manifest = {{"version", "v0.0.0-review"},
            {"repository", "Rex-Taiphoon/Taiphoon_Agent"}};
        for (const std::string architecture : {"amd64", "arm64"})
        {
            for (const std::string kind : {"runtime", "dev"})
            {
                const auto prefix = kind + "_" + architecture;
                manifest[prefix + "_sha256"] = std::string(64, 'a');
                manifest[prefix + "_id"] = "sha256:" + std::string(64, 'b');
            }
        }
        deployment::validateManifest(manifest);
        manifest["runtime_arm64_config"] = "{}";
        deployment::validateManifest(manifest);
        manifest["runtime_arm64_layers"] = "[\"sha256:" + std::string(64, 'c') + "\"]";
        deployment::validateManifest(manifest);
        const auto complete_manifest = manifest;
        std::string large_layers = "[";
        for (unsigned int layer = 0; layer < 1000; ++layer)
        {
            large_layers += (layer == 0 ? "" : ",") + std::string("\"sha256:") + std::string(64, 'c') + "\"";
        }
        manifest["runtime_arm64_layers"] = large_layers + ']';
        deployment::validateManifest(manifest);
        manifest["runtime_arm64_layers"] = "[\"missing-digest\"]";
        requireRejected([&manifest] { deployment::validateManifest(manifest); });
        manifest = complete_manifest;
        manifest["runtime_arm64_config"] = "null";
        deployment::validateManifest(manifest);

        settings = defaults();
        const auto deploy = deployment::renderCompose(settings, "taiphoon-agent:test", "arm64", directory);
        require(deploy.find("network_mode: host") != std::string::npos, "部署未使用 host 網路");
        require(deploy.find("ENABLE_OBSTACLE_CLOUD_SIMULATOR: \"false\"") != std::string::npos,
            "部署錯誤啟用模擬感測器");
        require(deploy.find("source: \"/source") == std::string::npos, "部署掛載原始碼");
        testObstacleSource();
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
        require(sim.find("network_mode:") == std::string::npos, "模擬 Agent 不應共用網路空間");
        require(sim.find("PX4_AGENT_HOST: agent_4") != std::string::npos,
            "PX4 未連向對應 Agent");
        for (unsigned int instance = 1; instance <= 4; ++instance)
        {
            const auto start = sim.find("  agent_" + std::to_string(instance) + ":\n");
            const auto end = sim.find("\n  agent_", start + 1);
            const auto service = sim.substr(start, end == std::string::npos ? end : end - start);
            require(service.find("    networks:\n      - taiphoon_net") != std::string::npos,
                "模擬 Agent 未獨立接入 Taiphoon_Net");
        }
        require(sim.find("name: Taiphoon_Net\n    external: true") != std::string::npos,
            "模擬未使用外部 Taiphoon_Net");
        require(sim.find("    networks:\n      - taiphoon_net") != std::string::npos,
            "模擬 Agent 未接入 Taiphoon_Net");
        migrated["network"] = "wifi";
        migrated["lan_address"] = "192.168.1.20";
        migrated["dds_peers"] = "192.168.1.21";
        const auto simulation_xml = deployment::renderCycloneDds(migrated);
        require(simulation_xml.find("ExternalNetworkAddress") == std::string::npos
            && simulation_xml.find("192.168.1.21") == std::string::npos,
            "模擬不應公告主機位址或沿用外部 GCS peer");
        migrated["agent_id"] = "7";
        const auto single = deployment::renderCompose(migrated, "taiphoon-agent:test", "amd64", directory, true);
        require(single.find("px4_1:") != std::string::npos && single.find("px4_2:") == std::string::npos,
            "單機模擬啟動額外飛控");
        require(single.find("7410-7491") == std::string::npos, "舊 Wi-Fi 模擬設定不應映射整段主機 DDS 埠");
        require(single.find("\"192.168.1.20:7410-7433:7410-7433/udp\"") != std::string::npos
            && single.find("cyclonedds_1.xml") != std::string::npos,
            "Desktop 模擬未只映射本機 QGCS 單播埠段");
        require(single.find("name: Taiphoon_Net\n    external: true") != std::string::npos,
            "Wi-Fi 模擬未接入 Taiphoon_Net");
        // Desktop 模擬：每台 Agent 公告主機 IP、使用不重疊埠段，並向主機 QGCS 單播 discovery。
        auto host_gcs = defaults();
        host_gcs["mode"] = "sim";
        host_gcs["lan_address"] = "192.168.1.30";
        const auto engine_sim = deployment::renderCompose(host_gcs, "taiphoon-agent:test", "amd64", directory);
        require(engine_sim.find("ports:") == std::string::npos
            && engine_sim.find("cyclonedds_1.xml") == std::string::npos
            && engine_sim.find("TAIPHOON_DDS_PEER_RELAY") == std::string::npos,
            "Linux Engine 模擬不應改變既有網路設定");
        require(deployment::renderCycloneDds(host_gcs) == deployment::renderCycloneDds(host_gcs, false, 3)
            && deployment::renderCycloneDds(host_gcs).find("<MaxAutoParticipantIndex>40<") != std::string::npos
            && deployment::renderCycloneDds(host_gcs).find("ExternalNetworkAddress") == std::string::npos,
            "非 Desktop 模擬的 DDS 設定被改變");
        require(deployment::usesHostGcs(host_gcs, true) && !deployment::usesHostGcs(host_gcs, false),
            "原生 QGCS 判斷錯誤");
        const auto desktop_sim = deployment::renderCompose(host_gcs, "taiphoon-agent:test", "amd64", directory, true);
        const std::vector<std::string> ranges = {"7410-7433", "7434-7457", "7458-7481", "7482-7505"};
        for (unsigned int instance = 1; instance <= 4; ++instance)
        {
            const auto start = desktop_sim.find("  agent_" + std::to_string(instance) + ":\n");
            const auto end = desktop_sim.find("\n  agent_", start + 1);
            const auto service = desktop_sim.substr(start, end == std::string::npos ? end : end - start);
            const auto& range = ranges[instance - 1];
            require(service.find("\"192.168.1.30:" + range + ":" + range + "/udp\"") != std::string::npos,
                "Agent " + std::to_string(instance) + " 埠段映射錯誤");
            require(service.find("cyclonedds_" + std::to_string(instance) + ".xml") != std::string::npos
                && service.find("    networks:\n      - taiphoon_net") != std::string::npos,
                "Agent " + std::to_string(instance) + " 未使用專屬 DDS 設定或未接入 Taiphoon_Net");
            // 同一主機 IP 會被 CycloneDDS 改寫成本機容器 IP，必須啟用同儕轉送才能互相發現。
            require(service.find("TAIPHOON_DDS_PEER_RELAY_FORWARD: \"192.168.1.30\"") != std::string::npos
                && service.find("TAIPHOON_DDS_PEER_RELAY_INSTANCE: \"" + std::to_string(instance) + "\"")
                    != std::string::npos
                && service.find("TAIPHOON_DDS_PEER_RELAY_COUNT: \"4\"") != std::string::npos
                && service.find("TAIPHOON_DDS_PEER_RELAY_HOST_FORMAT: \"agent_{N}\"") != std::string::npos,
                "Agent " + std::to_string(instance) + " 未啟用 DDS 同儕轉送");
            const auto offset = std::to_string(10 + 24 * (instance - 1));
            const auto host_xml = deployment::renderCycloneDds(host_gcs, true, instance);
            require(host_xml.find("<ExternalNetworkAddress>192.168.1.30<") != std::string::npos
                && host_xml.find("<MaxAutoParticipantIndex>11<") != std::string::npos
                && host_xml.find("Address=\"127.0.0.1\"") != std::string::npos
                && host_xml.find("Address=\"192.168.65.254:7510\"") != std::string::npos
                && host_xml.find("<UnicastMetaOffset>" + offset + "<") != std::string::npos
                && host_xml.find("<UnicastDataOffset>" + std::to_string(11 + 24 * (instance - 1)) + "<")
                    != std::string::npos
                && host_xml.find("<AllowMulticast>true<") != std::string::npos,
                "Agent " + std::to_string(instance) + " 單播 DDS 設定錯誤");
            deployment::writeText(directory / ("cyclonedds_" + std::to_string(instance) + ".xml"), host_xml);
        }
        require(desktop_sim.find("px4_4:") != std::string::npos
            && desktop_sim.find("PX4_AGENT_HOST: agent_4") != std::string::npos, "Desktop 模擬 PX4 服務改變");
        requireRejected([&host_gcs] { deployment::renderCycloneDds(host_gcs, true, 5); });
        requireRejected([&host_gcs] { deployment::renderCycloneDds(host_gcs, true, 0); });
        // 其他電腦的 QGCS：每台 Agent 都主動送 discovery 到 GCS 的 index 50，並保留主機 QGCS。
        host_gcs["dds_peers"] = "192.168.1.40, 192.168.1.41";
        for (unsigned int instance = 1; instance <= 4; ++instance)
        {
            const auto remote_xml = deployment::renderCycloneDds(host_gcs, true, instance);
            require(remote_xml.find("<Peer Address=\"127.0.0.1\"/><Peer Address=\"192.168.65.254:7510\"/>"
                "<Peer Address=\"192.168.1.40:7510\"/><Peer Address=\"192.168.1.41:7510\"/></Peers>")
                    != std::string::npos,
                "Agent " + std::to_string(instance) + " 未主動向其他電腦的 QGCS 送出 discovery");
        }
        require(deployment::renderCycloneDds(host_gcs).find("192.168.1.40") == std::string::npos,
            "Linux Engine 模擬不應沿用外部 GCS peer");
        require(deployment::renderCompose(host_gcs, "taiphoon-agent:test", "amd64", directory, true)
                .find("\"192.168.1.30:7410-7433:7410-7433/udp\"") != std::string::npos,
            "外部 GCS peer 不應改變主機埠映射");
        for (const auto* loopback : {"127.0.0.1", "192.168.1.40,127.0.1.1"})
        {
            auto rejected = host_gcs;
            rejected["dds_peers"] = loopback;
            requireRejected([&rejected] { deployment::validateSettings(rejected); });
        }
        host_gcs["domain_id"] = "1";
        require(deployment::renderCycloneDds(host_gcs, true, 1).find("192.168.65.254:7760") != std::string::npos
            && deployment::renderCycloneDds(host_gcs, true, 1).find("192.168.1.40:7760") != std::string::npos
            && deployment::renderCompose(host_gcs, "taiphoon-agent:test", "amd64", directory, true)
                .find("192.168.1.30:7660-7683:7660-7683/udp") != std::string::npos,
            "非 0 domain 埠號未平移");
        host_gcs["dds_peers"] = "";
        for (const auto& unresolved : {"auto", ""})
        {
            host_gcs["lan_address"] = unresolved;
            requireRejected([&host_gcs] { deployment::renderCycloneDds(host_gcs, true, 1); });
            requireRejected([&host_gcs, &directory] {
                deployment::renderCompose(host_gcs, "taiphoon-agent:test", "amd64", directory, true); });
        }
        deployment::writeText(directory / "desktop-sim.yaml", desktop_sim);
        migrated["mode"] = "deploy";
        require(deployment::renderCycloneDds(migrated, true) == deployment::renderCycloneDds(migrated),
            "實機 DDS 設定受 Desktop 判斷影響");
        const auto desktop = deployment::renderCompose(migrated, "taiphoon-agent:test", "amd64", directory, true);
        require(desktop.find("8894:8894/udp") != std::string::npos, "無人機編號未對應 Desktop UDP 埠");
        require(desktop.find("network_mode: host") == std::string::npos, "Desktop 要求全域 host networking");
        require(desktop.find("px4_1:") == std::string::npos, "實機部署啟動模擬器");
        migrated["mode"] = "dev";
        migrated["scenario"] = "sim";
        migrated["workspace"] = directory.u8string();
        const auto development = deployment::renderCompose(migrated, "taiphoon-agent:test", "amd64", directory, true);
        require(development.find("px4_1:") != std::string::npos && development.find("build_amd64") != std::string::npos,
            "開發模擬缺少 PX4 或共享建置 volume");
        for (const auto& count : {"0", "5", "999"})
        {
            migrated["count"] = count;
            requireRejected([&migrated] { deployment::validateSettings(migrated); });
        }
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
        testTimeSync();
        deployment::writeText(directory / "deploy.yaml", deploy);
        deployment::writeText(directory / "dev.yaml", dev);
        deployment::writeText(directory / "sim.yaml", sim);
        deployment::writeText(directory / "serial.yaml", serial);
        std::cout << "PASS: 設定拒絕、SHA256、模式隔離、DDS、本機 QGCS 單播、串列、Windows argv、程序失敗、chrony 對時判定、障礙物來源\n"
            << "測試產物：" << directory.u8string() << '\n';
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
