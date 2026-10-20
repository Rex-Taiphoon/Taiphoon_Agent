/**
 * @file launcher.cpp
 * @brief 設定驗證、SHA256 與平台無關的部署描述
 * @author Taiphoon Agent Team
 * @date 2026-10-02
 */
#include "launcher.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <locale>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace taiphoon::deployment
{
namespace
{
constexpr unsigned int SHA_BLOCK_BYTES = 64;
constexpr unsigned int SHA_LENGTH_BYTES = 8;
constexpr unsigned int MAX_AGENTS = 4;
constexpr unsigned int MAX_SYSTEM_ID = 255;
constexpr unsigned int MAX_DOMAIN_ID = 101;
// 超過 1 秒時同儕軌跡已會被規劃器拒收，不允許設定更寬鬆的啟動門檻。
constexpr unsigned int MAX_TIME_SYNC_OFFSET_MS = 1000;
// chronyc -c tracking 欄位：0 Reference ID、1 名稱、2 stratum、4 System time、13 Leap status。
constexpr std::size_t CHRONY_TRACKING_FIELD_COUNT = 14;
constexpr std::size_t CHRONY_STRATUM_FIELD = 2;
constexpr std::size_t CHRONY_SYSTEM_TIME_FIELD = 4;
constexpr std::size_t CHRONY_LEAP_STATUS_FIELD = 13;
constexpr unsigned int CHRONY_MAX_SYNCED_STRATUM = 15;
constexpr unsigned int DDS_BASE_PORT = 7400;
constexpr unsigned int DDS_DOMAIN_PORT_STEP = 250;
constexpr unsigned int DDS_UNICAST_OFFSET = 10;
constexpr unsigned int DDS_PARTICIPANT_PORT_STEP = 2;
constexpr unsigned int DDS_LAST_PARTICIPANT_PORT = 91;
// 原生 GCS 模式每台 Agent 12 個 participant（index 0～11），各占 meta／data 兩埠。
constexpr unsigned int HOST_GCS_MAX_PARTICIPANT_INDEX = 11;
constexpr unsigned int HOST_GCS_AGENT_PORT_SPAN = (HOST_GCS_MAX_PARTICIPANT_INDEX + 1) * DDS_PARTICIPANT_PORT_STEP;
// Taiphoon_QGCS 固定 participant index；domain 0 時對應 7510，不與 4 台 Agent 埠段重疊。
constexpr unsigned int HOST_GCS_PARTICIPANT_INDEX = 50;
// Docker Desktop 的 host.docker.internal，Agent 由此主動送 discovery 給主機上的 QGCS。
constexpr const char* DESKTOP_HOST_GATEWAY = "192.168.65.254";

/** @brief 線性解析 digest 陣列，避免 Windows std::regex 對大型 manifest 耗盡堆疊。
 * @param value Docker 的緊湊 JSON layer 陣列。
 * @return 每個項目是否為有效 SHA256 digest。
 */
bool isLayerList(const std::string& value)
{
    constexpr std::size_t DIGEST_HEX_LENGTH = 64;
    const std::string prefix = "\"sha256:";
    if (value == "[]")
    {
        return true;
    }
    if (value.size() < 2 || value.front() != '[' || value.back() != ']')
    {
        return false;
    }
    std::size_t cursor = 1;
    while (cursor < value.size() - 1)
    {
        if (value.compare(cursor, prefix.size(), prefix) != 0)
        {
            return false;
        }
        cursor += prefix.size();
        if (cursor + DIGEST_HEX_LENGTH >= value.size() - 1)
        {
            return false;
        }
        for (std::size_t offset = 0; offset < DIGEST_HEX_LENGTH; ++offset)
        {
            const char character = value[cursor + offset];
            if ((character < '0' || character > '9') && (character < 'a' || character > 'f'))
            {
                return false;
            }
        }
        cursor += DIGEST_HEX_LENGTH;
        if (value[cursor++] != '"')
        {
            return false;
        }
        if (cursor == value.size() - 1)
        {
            return true;
        }
        if (value[cursor++] != ',')
        {
            return false;
        }
    }
    return false;
}

/** @brief 移除設定與程序輸出的外側空白。 */
std::string trim(const std::string& value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return {};
    }
    return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

/** @brief 驗證完整整數，而非接受帶有命令片段的數字前綴。 */
unsigned int number(const Settings& settings, const std::string& key, unsigned int maximum,
    unsigned int minimum = 1)
{
    const std::string& value = settings.at(key);
    if (!std::regex_match(value, std::regex("[0-9]{1,7}")))
    {
        throw std::runtime_error(key + " 必須是整數");
    }
    const auto parsed = std::stoul(value);
    if (parsed < minimum || parsed > maximum)
    {
        throw std::runtime_error(key + " 超出允許範圍");
    }
    return static_cast<unsigned int>(parsed);
}

/** @brief 只允許合法 IPv4，避免把使用者字串插入 DDS XML。 */
bool isIpv4(const std::string& value)
{
    if (!std::regex_match(value, std::regex("([0-9]{1,3}\\.){3}[0-9]{1,3}")))
    {
        return false;
    }
    std::istringstream stream(value);
    std::string octet;
    while (std::getline(stream, octet, '.'))
    {
        if (std::stoul(octet) > 255)
        {
            return false;
        }
    }
    return true;
}

/** @brief 列出 discovery peers，空字串代表使用 multicast。 */
std::vector<std::string> peers(const std::string& value)
{
    std::istringstream stream(value);
    std::vector<std::string> result;
    std::string peer;
    while (std::getline(stream, peer, ','))
    {
        peer = trim(peer);
        if (!isIpv4(peer))
        {
            throw std::runtime_error("dds_peers 必須是以逗號分隔的 IPv4");
        }
        result.push_back(peer);
    }
    if (!value.empty() && value.back() == ',')
    {
        throw std::runtime_error("dds_peers 不可包含空項目");
    }
    return result;
}

/** @brief 指定 domain 的 DDS 埠基準，與 CycloneDDS 預設 PB／DG 相同。 */
unsigned int ddsDomainBase(const Settings& settings)
{
    return DDS_BASE_PORT + DDS_DOMAIN_PORT_STEP * number(settings, "domain_id", MAX_DOMAIN_ID, 0);
}

/** @brief 原生 GCS 模式下第 instance 台 Agent 的單播 meta 埠偏移，data 偏移為其加一。 */
unsigned int hostGcsUnicastOffset(unsigned int instance)
{
    return DDS_UNICAST_OFFSET + HOST_GCS_AGENT_PORT_SPAN * (instance - 1);
}

/** @brief 固定 index 的 Taiphoon_QGCS 單播 discovery 埠；Agent 由此主動送出 discovery。 */
unsigned int hostGcsDiscoveryPort(const Settings& settings)
{
    return ddsDomainBase(settings) + DDS_UNICAST_OFFSET + DDS_PARTICIPANT_PORT_STEP * HOST_GCS_PARTICIPANT_INDEX;
}

/** @brief SHA256 定義的 32 位元循環右移。 */
std::uint32_t rotate(std::uint32_t value, unsigned int amount)
{
    return (value >> amount) | (value << (32 - amount));
}

/** @brief SHA256 壓縮函式；依 FIPS 180-4 計算，不依賴平台雜湊工具。 */
void compress(std::array<std::uint32_t, 8>& state, const std::array<unsigned char, SHA_BLOCK_BYTES>& block)
{
    static constexpr std::array<std::uint32_t, 64> constants = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    std::array<std::uint32_t, 64> words{};
    for (unsigned int i = 0; i < 16; ++i)
    {
        for (unsigned int j = 0; j < 4; ++j)
        {
            words[i] = (words[i] << 8) | block[i * 4 + j];
        }
    }
    for (unsigned int i = 16; i < words.size(); ++i)
    {
        const auto a = words[i - 15];
        const auto b = words[i - 2];
        words[i] = words[i - 16] + (rotate(a, 7) ^ rotate(a, 18) ^ (a >> 3))
            + words[i - 7] + (rotate(b, 17) ^ rotate(b, 19) ^ (b >> 10));
    }
    auto working = state;
    for (unsigned int i = 0; i < words.size(); ++i)
    {
        const auto a = working[0];
        const auto e = working[4];
        const auto first = working[7] + (rotate(e, 6) ^ rotate(e, 11) ^ rotate(e, 25))
            + ((e & working[5]) ^ (~e & working[6])) + constants[i] + words[i];
        const auto second = (rotate(a, 2) ^ rotate(a, 13) ^ rotate(a, 22))
            + ((a & working[1]) ^ (a & working[2]) ^ (working[1] & working[2]));
        for (unsigned int j = 7; j > 0; --j)
        {
            working[j] = working[j - 1];
        }
        working[4] += first;
        working[0] = first + second;
    }
    for (unsigned int i = 0; i < state.size(); ++i)
    {
        state[i] += working[i];
    }
}
}

Settings readSettings(const fs::path& path)
{
    std::ifstream stream(path);
    if (!stream)
    {
        throw std::runtime_error("無法讀取 " + path.u8string());
    }
    Settings settings;
    std::string line;
    while (std::getline(stream, line))
    {
        line = trim(line);
        if (line.empty() || line.front() == '#')
        {
            continue;
        }
        const auto separator = line.find('=');
        if (separator == std::string::npos)
        {
            throw std::runtime_error("設定行缺少 =");
        }
        const auto key = trim(line.substr(0, separator));
        if (!std::regex_match(key, std::regex("[a-z][a-z0-9_]*"))
            || !settings.emplace(key, trim(line.substr(separator + 1))).second)
        {
            throw std::runtime_error("非法或重複的設定欄位：" + key);
        }
    }
    if (!stream.eof())
    {
        throw std::runtime_error("讀取設定失敗");
    }
    return settings;
}

void writeText(const fs::path& path, const std::string& text)
{
    fs::create_directories(path.parent_path());
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream || !(stream << text) || !stream.flush())
    {
        throw std::runtime_error("無法保存 " + path.u8string());
    }
}

Settings withDefaultSettings(const Settings& settings)
{
    Settings complete = {{"mode", "deploy"}, {"agent_id", "1"}, {"count", "1"}, {"transport", "udp"},
        {"serial_device", ""}, {"serial_baud", "921600"}, {"domain_id", "0"}, {"workspace", ""},
        {"lan_address", "auto"}, {"dds_peers", ""}, {"scenario", "real"}, {"network", "wifi"},
        {"flight_ip", "192.168.1.14"}, {"time_sync", "auto"}, {"time_sync_max_offset_ms", "50"},
        {"obstacle_source", "none"}};
    for (const auto& entry : settings)
    {
        complete[entry.first] = entry.second;
    }
    if (settings.count("scenario") == 0 && complete.at("mode") == "sim")
    {
        complete["scenario"] = "sim";
    }
    if (settings.count("network") == 0 && isSimulation(complete))
    {
        complete["network"] = "local";
    }
    return complete;
}

bool isSimulation(const Settings& settings)
{
    return settings.at("mode") == "sim" || (settings.at("mode") == "dev"
        && settings.count("scenario") != 0 && settings.at("scenario") == "sim");
}

bool usesHostGcs(const Settings& settings, bool is_desktop)
{
    return is_desktop && isSimulation(settings);
}

void validateSettings(const Settings& settings)
{
    static const std::vector<std::string> keys = {"mode", "agent_id", "count", "transport", "serial_device",
        "serial_baud", "domain_id", "workspace", "lan_address", "dds_peers"};
    for (const auto& key : keys)
    {
        if (settings.count(key) == 0)
        {
            throw std::runtime_error("設定缺少 " + key + "；請執行 taiphoon_agent init");
        }
    }
    for (const auto& entry : settings)
    {
        if (entry.second.find_first_of("\r\n\0", 0, 3) != std::string::npos)
        {
            throw std::runtime_error("不支援的設定欄位或內容：" + entry.first);
        }
    }
    const auto& mode = settings.at("mode");
    if (mode != "deploy" && mode != "dev" && mode != "sim")
    {
        throw std::runtime_error("mode 必須是 deploy、dev 或 sim");
    }
    number(settings, "agent_id", MAX_SYSTEM_ID);
    number(settings, "count", MAX_AGENTS);
    number(settings, "domain_id", MAX_DOMAIN_ID, 0);
    number(settings, "serial_baud", 4000000);
    const auto& transport = settings.at("transport");
    if (transport != "udp" && transport != "serial")
    {
        throw std::runtime_error("transport 必須是 udp 或 serial");
    }
    if (transport == "serial" && (mode != "deploy"
        || !std::regex_match(settings.at("serial_device"), std::regex("/dev/[A-Za-z0-9_./-]+"))
        || settings.at("serial_device").find("..") != std::string::npos))
    {
        throw std::runtime_error("串列埠只適用於部署模式，必須指定 /dev/ 下的裝置");
    }
    if (mode == "dev" && settings.at("workspace").empty())
    {
        throw std::runtime_error("開發模式需要原始碼工作目錄");
    }
    if (!settings.at("lan_address").empty() && settings.at("lan_address") != "auto"
        && !isIpv4(settings.at("lan_address")))
    {
        throw std::runtime_error("lan_address 必須是本機 LAN IPv4");
    }
    for (const auto& peer : peers(settings.at("dds_peers")))
    {
        // 容器內的 loopback 指向容器本身；同機 QGCS 已由 host.docker.internal 涵蓋。
        if (isSimulation(settings) && peer.rfind("127.", 0) == 0)
        {
            throw std::runtime_error("模擬模式的 GCS IPv4 不可為 127.x；同機 Taiphoon_QGCS 不需填寫");
        }
    }
    if (settings.count("network") != 0 && settings.at("network") != "local" && settings.at("network") != "wifi")
    {
        throw std::runtime_error("network 必須是 local 或 wifi");
    }
    if (settings.count("scenario") != 0 && settings.at("scenario") != "real" && settings.at("scenario") != "sim")
    {
        throw std::runtime_error("scenario 必須是 real 或 sim");
    }
    if (settings.count("flight_ip") != 0 && !isIpv4(settings.at("flight_ip")))
    {
        throw std::runtime_error("flight_ip 必須是飛控 IPv4");
    }
    if (settings.count("time_sync") != 0)
    {
        const auto& policy = settings.at("time_sync");
        if (policy != "auto" && policy != "required" && policy != "warn" && policy != "off")
        {
            throw std::runtime_error("time_sync 必須是 auto、required、warn 或 off");
        }
    }
    if (settings.count("time_sync_max_offset_ms") != 0)
    {
        number(settings, "time_sync_max_offset_ms", MAX_TIME_SYNC_OFFSET_MS);
    }
    if (settings.count("obstacle_source") != 0 && settings.at("obstacle_source") != "none"
        && settings.at("obstacle_source") != "explicit_empty")
    {
        throw std::runtime_error("obstacle_source 必須是 none 或 explicit_empty");
    }
}

bool usesExplicitEmptyObstacles(const Settings& settings)
{
    return !isSimulation(settings) && settings.count("obstacle_source") != 0
        && settings.at("obstacle_source") == "explicit_empty";
}

std::string normalizeArchitecture(const std::string& architecture)
{
    const auto value = trim(architecture);
    if (value == "x86_64" || value == "amd64")
    {
        return "amd64";
    }
    if (value == "aarch64" || value == "arm64")
    {
        return "arm64";
    }
    throw std::runtime_error("只支援 64 位元 AMD64／ARM64 Docker daemon");
}

std::string yamlString(const std::string& value)
{
    std::string result = "\"";
    for (const char character : value)
    {
        switch (character)
        {
        case '\\': result += "\\\\"; break;
        case '"': result += "\\\""; break;
        case '\n': result += "\\n"; break;
        case '\r': result += "\\r"; break;
        case '\t': result += "\\t"; break;
        case '$': result += "$$"; break;
        default: result += character; break;
        }
    }
    return result + '"';
}

std::string sha256(const fs::path& path)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
    {
        throw std::runtime_error("無法校驗 " + path.u8string());
    }
    std::array<std::uint32_t, 8> state = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
        0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    std::array<unsigned char, SHA_BLOCK_BYTES> block{};
    std::uint64_t length = 0;
    while (true)
    {
        stream.read(reinterpret_cast<char*>(block.data()), block.size());
        const auto bytes = static_cast<unsigned int>(stream.gcount());
        length += bytes;
        if (bytes == block.size())
        {
            compress(state, block);
            continue;
        }
        if (!stream.eof())
        {
            throw std::runtime_error("讀取映像以校驗時失敗");
        }
        block[bytes] = 0x80;
        std::fill(block.begin() + bytes + 1, block.end(), 0);
        if (bytes >= SHA_BLOCK_BYTES - SHA_LENGTH_BYTES)
        {
            compress(state, block);
            block.fill(0);
        }
        const std::uint64_t bits = length * 8;
        for (unsigned int i = 0; i < SHA_LENGTH_BYTES; ++i)
        {
            block[SHA_BLOCK_BYTES - 1 - i] = static_cast<unsigned char>(bits >> (i * 8));
        }
        compress(state, block);
        break;
    }
    std::ostringstream result;
    result << std::hex << std::setfill('0');
    for (const auto word : state)
    {
        result << std::setw(8) << word;
    }
    return result.str();
}

void validateManifest(const Settings& manifest)
{
    std::vector<std::string> required = {"version", "repository"};
    for (const std::string architecture : {"amd64", "arm64"})
    {
        for (const std::string kind : {"runtime", "dev"})
        {
            required.push_back(kind + "_" + architecture + "_sha256");
            required.push_back(kind + "_" + architecture + "_id");
        }
    }
    for (const auto& key : required)
    {
        if (manifest.count(key) == 0)
        {
            throw std::runtime_error("Release 包缺少必要資料：" + key + "；請重新下載完整平台包");
        }
    }
    if (!std::regex_match(manifest.at("version"), std::regex("v[0-9]+\\.[0-9]+\\.[0-9]+([.-][A-Za-z0-9.-]+)?"))
        || manifest.at("repository") != "Rex-Taiphoon/Taiphoon_Agent")
    {
        throw std::runtime_error("Release manifest 的版本或來源非法");
    }
    for (const std::string architecture : {"amd64", "arm64"})
    {
        for (const std::string kind : {"runtime", "dev"})
        {
            const auto prefix = kind + "_" + architecture;
            if (!std::regex_match(manifest.at(prefix + "_sha256"), std::regex("[a-f0-9]{64}"))
                || !std::regex_match(manifest.at(prefix + "_id"), std::regex("sha256:[a-f0-9]{64}")))
            {
                throw std::runtime_error("Release manifest 缺少有效映像校驗資料");
            }
            const bool has_layers = manifest.count(prefix + "_layers") != 0;
            if (has_layers && !isLayerList(manifest.at(prefix + "_layers")))
            {
                throw std::runtime_error("Release manifest 的映像內容描述不完整或非法");
            }
        }
    }
}

std::string renderCycloneDds(const Settings& settings, bool is_desktop, unsigned int instance)
{
    validateSettings(settings);
    std::ostringstream xml;
    xml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        << "<CycloneDDS xmlns=\"https://cdds.io/config\"><Domain Id=\"any\"><General>"
        << "<AllowMulticast>true</AllowMulticast>";
    if (usesHostGcs(settings, is_desktop))
    {
        // Desktop 主機收不到容器 multicast：公告主機 IP＋專屬埠段，並主動向主機與 dds_peers 的 QGCS 單播 discovery。
        if (!isIpv4(settings.at("lan_address")))
        {
            throw std::runtime_error("模擬 Agent 需要主機區網 IPv4 供本機 Taiphoon_QGCS 連線；"
                "請在 settings.ini 指定 lan_address");
        }
        if (instance < 1 || instance > number(settings, "count", MAX_AGENTS))
        {
            throw std::runtime_error("模擬 Agent 序號超出 count");
        }
        const auto offset = hostGcsUnicastOffset(instance);
        const auto gcs_port = hostGcsDiscoveryPort(settings);
        xml << "<ExternalNetworkAddress>" << settings.at("lan_address") << "</ExternalNetworkAddress>"
            << "</General><Discovery><ParticipantIndex>auto</ParticipantIndex>"
            << "<MaxAutoParticipantIndex>" << HOST_GCS_MAX_PARTICIPANT_INDEX << "</MaxAutoParticipantIndex>"
            << "<Peers><Peer Address=\"127.0.0.1\"/><Peer Address=\"" << DESKTOP_HOST_GATEWAY << ':' << gcs_port
            << "\"/>";
        // 其他電腦的 QGCS 不在 Agent 埠段探測範圍內，需指定其固定 index 的 discovery 埠。
        for (const auto& peer : peers(settings.at("dds_peers")))
        {
            xml << "<Peer Address=\"" << peer << ':' << gcs_port << "\"/>";
        }
        xml << "</Peers><Ports><UnicastMetaOffset>" << offset << "</UnicastMetaOffset>"
            << "<UnicastDataOffset>" << offset + 1 << "</UnicastDataOffset></Ports>"
            << "</Discovery></Domain></CycloneDDS>\n";
        return xml.str();
    }
    if (!isSimulation(settings) && !settings.at("lan_address").empty() && settings.at("lan_address") != "auto")
    {
        xml << "<ExternalNetworkAddress>" << settings.at("lan_address") << "</ExternalNetworkAddress>";
    }
    xml << "</General><Discovery><ParticipantIndex>auto</ParticipantIndex>"
        << "<MaxAutoParticipantIndex>40</MaxAutoParticipantIndex><Peers><Peer Address=\"127.0.0.1\"/>";
    for (const auto& peer : peers(isSimulation(settings) ? "" : settings.at("dds_peers")))
    {
        xml << "<Peer Address=\"" << peer << "\"/>";
    }
    xml << "</Peers></Discovery></Domain></CycloneDDS>\n";
    return xml.str();
}

std::string renderCompose(const Settings& settings, const std::string& image,
    const std::string& architecture, const fs::path& home, bool is_desktop)
{
    validateSettings(settings);
    normalizeArchitecture(architecture);
    constexpr unsigned int XRCE_BASE_PORT = 8887;
    const std::string mode = settings.at("mode");
    const bool is_simulation = isSimulation(settings);
    const bool is_development = mode == "dev";
    const bool uses_bridge = is_simulation || is_desktop;
    const bool uses_wifi = !is_simulation
        && (settings.count("network") == 0 || settings.at("network") == "wifi");
    const bool uses_host_gcs = usesHostGcs(settings, is_desktop);
    if (uses_host_gcs && !isIpv4(settings.at("lan_address")))
    {
        throw std::runtime_error("模擬 Agent 需要主機區網 IPv4 以映射 DDS 埠；請在 settings.ini 指定 lan_address");
    }
    const unsigned int count = is_simulation ? number(settings, "count", MAX_AGENTS) : 1;
    std::ostringstream yaml;
    yaml << "# 由 taiphoon_agent 產生；進階 ROS2 參數請使用各服務的 environment。\nservices:\n";
    for (unsigned int instance = 1; instance <= count; ++instance)
    {
        const unsigned int id = is_simulation ? instance : number(settings, "agent_id", MAX_SYSTEM_ID);
        yaml << "  agent_" << instance << ":\n    image: " << yamlString(image)
            << "\n    pull_policy: never\n    restart: \"no\"\n"
            << "    working_dir: /home/taiphoon/Taiphoon_Agent\n"
            << "    entrypoint: [/home/taiphoon/runtime-entrypoint.sh]\n";
        if (!uses_bridge)
        {
            yaml << "    network_mode: host\n";
        }
        else
        {
            if (is_simulation)
            {
                yaml << "    networks:\n      - taiphoon_net\n";
            }
            yaml << "    extra_hosts:\n      - host.docker.internal:host-gateway\n";
            if (uses_wifi || !is_simulation || uses_host_gcs)
            {
                yaml << "    ports:\n";
                if (!is_simulation && settings.at("transport") == "udp")
                {
                    const auto port = std::to_string(XRCE_BASE_PORT + id);
                    yaml << "      - " << yamlString(port + ":" + port + "/udp") << '\n';
                }
                if (uses_wifi)
                {
                    const auto base = ddsDomainBase(settings);
                    const auto range = std::to_string(base + DDS_UNICAST_OFFSET) + "-"
                        + std::to_string(base + DDS_LAST_PARTICIPANT_PORT);
                    yaml << "      - " << yamlString(range + ":" + range + "/udp") << '\n';
                }
                if (uses_host_gcs)
                {
                    // 只映射本 Agent 的埠段並綁定主機區網 IP，避免多台 Agent 搶同一主機埠。
                    const auto first = ddsDomainBase(settings) + hostGcsUnicastOffset(instance);
                    const auto range = std::to_string(first) + "-"
                        + std::to_string(first + HOST_GCS_AGENT_PORT_SPAN - 1);
                    yaml << "      - " << yamlString(settings.at("lan_address") + ":" + range + ":"
                        + range + "/udp") << '\n';
                }
            }
        }
        yaml << "    environment:\n      TARGET_SYSTEM: \"" << id << "\"\n"
            << "      AGENT_NAMESPACE: uav_" << id << "_agent\n"
            << "      UXRCE_PORT: \"" << XRCE_BASE_PORT + id << "\"\n"
            << "      ROS_DOMAIN_ID: " << yamlString(settings.at("domain_id")) << "\n"
            << "      RMW_IMPLEMENTATION: rmw_cyclonedds_cpp\n"
            << "      CYCLONEDDS_URI: file:///home/taiphoon/deployment/cyclonedds.xml\n"
            << "      XRCE_TRANSPORT: " << yamlString(settings.at("transport")) << "\n"
            << "      XRCE_SERIAL_DEVICE: " << yamlString(settings.at("serial_device")) << "\n"
            << "      XRCE_SERIAL_BAUD: " << yamlString(settings.at("serial_baud")) << "\n"
            // 實機預設不提供障礙物地圖（規劃器不移動）；只有操作人員以 obstacle_source=explicit_empty
            // 明確宣告場地無障礙物時，才啟動模擬器發布空地圖。模擬模式維持原設定。
            << "      ENABLE_OBSTACLE_CLOUD_SIMULATOR: \""
            << (is_simulation || usesExplicitEmptyObstacles(settings) ? "true" : "false") << "\"\n"
            << "      OBSTACLE_CLOUD_MODE: explicit_empty\n";
        if (uses_host_gcs)
        {
            // 所有 Agent 公告同一個主機 IP，CycloneDDS 會把同儕 locator 改寫成本機容器 IP；
            // 由 runtime-entrypoint 啟動 dds_peer_relay 把同儕封包轉給 agent_<N> 容器。
            yaml << "      TAIPHOON_DDS_PEER_RELAY_FORWARD: " << yamlString(settings.at("lan_address")) << "\n"
                << "      TAIPHOON_DDS_PEER_RELAY_INSTANCE: \"" << instance << "\"\n"
                << "      TAIPHOON_DDS_PEER_RELAY_COUNT: \"" << count << "\"\n"
                << "      TAIPHOON_DDS_PEER_RELAY_HOST_FORMAT: \"agent_{N}\"\n";
        }
        yaml << "    volumes:\n      - type: bind\n        source: "
            << yamlString((home / (uses_host_gcs ? "cyclonedds_" + std::to_string(instance) + ".xml"
                : std::string("cyclonedds.xml"))).u8string())
            << "\n        target: /home/taiphoon/deployment/cyclonedds.xml\n        read_only: true\n";
        if (settings.at("transport") == "serial")
        {
            yaml << "    devices:\n      - " << yamlString(settings.at("serial_device")
                + ":" + settings.at("serial_device")) << "\n    group_add:\n      - dialout\n";
        }
        if (is_development)
        {
            yaml << "      - type: bind\n        source: " << yamlString(settings.at("workspace"))
                << "\n        target: /home/taiphoon/Taiphoon_Agent\n"
                << "      - build_" << architecture << ":/home/taiphoon/Taiphoon_Agent/build\n"
                << "      - install_" << architecture << ":/home/taiphoon/Taiphoon_Agent/install\n"
                << "      - log_" << architecture << ":/home/taiphoon/Taiphoon_Agent/log\n";
        }
    }
    if (is_simulation)
    {
        for (unsigned int instance = 1; instance <= count; ++instance)
        {
            yaml << "  px4_" << instance << ":\n    image: px4io/px4-sitl:latest\n"
                << "    platform: linux/amd64\n    restart: \"no\"\n"
                << "    networks:\n      - taiphoon_net\n"
                << "    depends_on:\n      agent_" << instance << ":\n        condition: service_healthy\n"
                << "    entrypoint: [/bin/sh, /deployment/sitl-entrypoint.sh]\n"
                << "    environment:\n      PX4_INSTANCE: \"" << instance - 1 << "\"\n"
                << "      PX4_SYS_ID: \"" << instance << "\"\n      PX4_AGENT_HOST: agent_" << instance << "\n"
                << "      PX4_AGENT_PORT: \"" << XRCE_BASE_PORT + instance << "\"\n"
                << "      PX4_PARAM_UXRCE_DDS_DOM_ID: " << yamlString(settings.at("domain_id")) << "\n"
                << "    extra_hosts:\n      - host.docker.internal:host-gateway\n"
                << "    volumes:\n      - type: bind\n        source: "
                << yamlString((home / "sitl-entrypoint.sh").u8string())
                << "\n        target: /deployment/sitl-entrypoint.sh\n        read_only: true\n";
        }
        yaml << "networks:\n  taiphoon_net:\n    name: Taiphoon_Net\n    external: true\n";
    }
    if (is_development)
    {
        yaml << "volumes:\n  build_" << architecture << ": {}\n  install_" << architecture
            << ": {}\n  log_" << architecture << ": {}\n";
    }
    return yaml.str();
}

namespace
{
/** @brief 以 C locale 完整解析有限浮點數，拒絕殘留字元、NaN 與 Inf。 */
bool parseFiniteDouble(const std::string& text, double& value)
{
    if (text.empty())
    {
        return false;
    }
    std::istringstream stream(text);
    stream.imbue(std::locale::classic());
    double parsed = 0.0;
    stream >> parsed;
    if (stream.fail() || !stream.eof() || !std::isfinite(parsed))
    {
        return false;
    }
    value = parsed;
    return true;
}

/** @brief chrony 的 Leap status 中，只有這些值代表已選定時間來源。 */
bool isSynchronizedLeapStatus(const std::string& leap_status)
{
    return leap_status == "Normal" || leap_status == "Insert second" || leap_status == "Delete second";
}

/** @brief 將 Reference ID 轉成操作人員可辨識的來源名稱。 */
std::string describeReference(const TimeSyncStatus& status)
{
    // 7F7F0101 是 chrony 以本機時鐘作為基準（local 指令）時的 Reference ID。
    if (status.reference_id == "7F7F0101")
    {
        return "本機基準（時間主機）";
    }
    return status.reference_name.empty() ? status.reference_id : status.reference_name;
}

/** @brief 以毫秒、小數三位顯示時間差。 */
std::string formatMilliseconds(double seconds)
{
    std::ostringstream text;
    text.imbue(std::locale::classic());
    text << std::fixed << std::setprecision(3) << seconds * 1000.0;
    return text.str();
}
}

TimeSyncStatus parseChronyTracking(const std::string& csv)
{
    TimeSyncStatus status;
    std::string line = csv;
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
    {
        line.pop_back();
    }
    if (line.empty() || line.find_first_of("\r\n") != std::string::npos)
    {
        return status;
    }
    std::vector<std::string> fields;
    std::size_t begin = 0;
    while (true)
    {
        const auto end = line.find(',', begin);
        fields.push_back(line.substr(begin, end == std::string::npos ? std::string::npos : end - begin));
        if (end == std::string::npos)
        {
            break;
        }
        begin = end + 1;
    }
    if (fields.size() != CHRONY_TRACKING_FIELD_COUNT
        || !std::regex_match(fields[0], std::regex("[0-9A-Fa-f]{8}"))
        || !std::regex_match(fields[CHRONY_STRATUM_FIELD], std::regex("[0-9]{1,2}")))
    {
        return status;
    }
    double offset_s = 0.0;
    if (!parseFiniteDouble(fields[CHRONY_SYSTEM_TIME_FIELD], offset_s))
    {
        return status;
    }
    status.reference_id = fields[0];
    std::transform(status.reference_id.begin(), status.reference_id.end(), status.reference_id.begin(),
        [](unsigned char character) { return static_cast<char>(std::toupper(character)); });
    status.reference_name = fields[1];
    status.stratum = static_cast<unsigned int>(std::stoul(fields[CHRONY_STRATUM_FIELD]));
    status.system_offset_s = offset_s;
    status.leap_status = fields[CHRONY_LEAP_STATUS_FIELD];
    status.is_valid = true;
    return status;
}

TimeSyncDecision evaluateTimeSync(const TimeSyncStatus& status, double max_offset_s)
{
    TimeSyncDecision decision;
    if (!std::isfinite(max_offset_s) || max_offset_s <= 0.0)
    {
        decision.reason = "對時誤差門檻無效";
        return decision;
    }
    if (!status.is_valid)
    {
        decision.reason = "無法解析 chronyc tracking 輸出";
        return decision;
    }
    if (!isSynchronizedLeapStatus(status.leap_status) || status.stratum == 0
        || status.stratum > CHRONY_MAX_SYNCED_STRATUM)
    {
        decision.reason = "尚未同步到時間來源（Leap status：" + status.leap_status
            + "）；請確認時間主機已開機且網路可達";
        return decision;
    }
    if (!std::isfinite(status.system_offset_s) || std::fabs(status.system_offset_s) > max_offset_s)
    {
        decision.reason = "系統時鐘誤差 " + formatMilliseconds(status.system_offset_s) + " ms 超過 "
            + formatMilliseconds(max_offset_s) + " ms；請等待 chrony 收斂";
        return decision;
    }
    decision.is_ready = true;
    decision.reason = "已同步：來源 " + describeReference(status) + "，stratum "
        + std::to_string(status.stratum) + "，系統時鐘誤差 " + formatMilliseconds(status.system_offset_s) + " ms";
    return decision;
}

std::string resolveTimeSyncPolicy(const Settings& settings, bool is_linux_host)
{
    if (isSimulation(settings))
    {
        return "off";
    }
    const auto policy = settings.count("time_sync") == 0 ? std::string("auto") : settings.at("time_sync");
    if (policy == "auto")
    {
        return is_linux_host ? "required" : "warn";
    }
    return policy;
}
}
