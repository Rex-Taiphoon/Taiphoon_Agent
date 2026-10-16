/**
 * @file main_dds_peer_relay.cpp
 * @brief Docker Desktop 模擬的 DDS 同儕單播轉送程式
 * @author Taiphoon Agent Team
 * @date 2026-10-16
 *
 * 只在「模擬＋主機 QGCS 單播」部署啟動。CycloneDDS 會把公告為主機 IP 的同儕
 * locator 改寫成本機容器 IP（見 utils/DdsPeerRelayPlan.hpp），本程式在本機 IP
 * 綁定其他 Agent 的埠段，將收到的 UDP datagram 原樣轉送到該埠段所屬 Agent 容器
 * 在 Docker 網路上的位址（同一埠）。
 *
 * 不經主機埠映射回送：Docker Desktop 的 UDP 埠轉送以來源位址記錄 flow，同儕
 * participant 結束後，其他 Agent 仍會持續送往舊埠而使該 flow 失效，之後在同一埠
 * 啟動的新 participant 便永遠收不到資料。直接送往容器位址只經過 bridge 網路；
 * 主機名稱無法解析時才退回主機區網 IP。
 *
 * 程式不解析 RTPS 內容、不需要特權，也不連結 ROS2；這是部署層的網路橋接，
 * 不屬於任何 Agent topic 或控制資料流。
 *
 * 用法：dds_peer_relay --bind <容器 IPv4> --forward <主機 IPv4> --domain <id>
 *                      --instance <N> --count <總數> [--peer-host-format <如 uav_{N}_agent>]
 */

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "utils/DdsPeerRelayPlan.hpp"

namespace
{

/// 單一 UDP datagram 的最大長度（位元組）
constexpr std::size_t MAX_DATAGRAM_BYTES = 65536U;
/// poll 逾時（毫秒），讓程式定期檢查結束訊號與重新解析同儕位址
constexpr int POLL_TIMEOUT_MS = 500;
/// 重新解析同儕容器位址的週期；Agent 容器重建後 IP 可能改變
constexpr std::chrono::seconds PEER_RESOLVE_PERIOD{5};
/// 轉送失敗時每隔多少次才輸出一次警告，避免洗版
constexpr unsigned long SEND_ERROR_LOG_INTERVAL = 1000UL;

volatile std::sig_atomic_t g_should_stop = 0;

/**
 * @brief 收到 SIGTERM／SIGINT 時要求主迴圈結束
 * @param signal_number 訊號編號（未使用）
 */
void stop_cb(int signal_number)
{
    (void)signal_number;
    g_should_stop = 1;
}

/// 以 RAII 持有 socket，避免提前返回時洩漏 file descriptor
class UdpSocket
{
public:
    UdpSocket() : fd_(::socket(AF_INET, SOCK_DGRAM, 0)) {}
    ~UdpSocket()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
        }
    }
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;
    UdpSocket(UdpSocket&& other) noexcept : fd_(other.fd_) { other.fd_ = -1; }
    UdpSocket& operator=(UdpSocket&&) = delete;

    /** @brief 取得 file descriptor；建立失敗時為 -1 */
    int fd() const { return fd_; }

private:
    int fd_;
};

/// 命令列參數
struct RelayOptions
{
    std::string bind_address{};
    std::string forward_address{};
    std::string peer_host_format{};
    taiphoon::utils::DdsPeerRelayConfig plan{};
};

/**
 * @brief 將十進位字串轉為無號整數
 * @param text 輸入字串
 * @param[out] value 轉換結果
 * @return true 表示整個字串都是有效數字
 */
bool parse_unsigned(const std::string& text, unsigned int& value)
{
    if (text.empty())
    {
        return false;
    }
    char* end = nullptr;
    errno = 0;
    const unsigned long parsed = std::strtoul(text.c_str(), &end, 10);
    if (errno != 0 || end == nullptr || *end != '\0' || parsed > 65535UL)
    {
        return false;
    }
    value = static_cast<unsigned int>(parsed);
    return true;
}

/**
 * @brief 解析命令列
 * @param argc 參數數量
 * @param argv 參數陣列
 * @param[out] options 解析結果
 * @return true 表示必要參數齊全且格式正確
 */
bool parse_options(int argc, char* argv[], RelayOptions& options)
{
    if (argv == nullptr)
    {
        std::cerr << "[dds_peer_relay] argv 為 nullptr\n";
        return false;
    }
    for (int index = 1; index + 1 < argc; index += 2)
    {
        const std::string key = argv[index];
        const std::string value = argv[index + 1];
        bool is_valid = true;
        if (key == "--bind")
        {
            options.bind_address = value;
        }
        else if (key == "--forward")
        {
            options.forward_address = value;
        }
        else if (key == "--peer-host-format")
        {
            options.peer_host_format = value;
        }
        else if (key == "--domain")
        {
            is_valid = parse_unsigned(value, options.plan.domain_id);
        }
        else if (key == "--instance")
        {
            is_valid = parse_unsigned(value, options.plan.instance);
        }
        else if (key == "--count")
        {
            is_valid = parse_unsigned(value, options.plan.instance_count);
        }
        else
        {
            is_valid = false;
        }
        if (!is_valid)
        {
            std::cerr << "[dds_peer_relay] 無效參數：" << key << ' ' << value << '\n';
            return false;
        }
    }
    return (argc % 2) == 1 && !options.bind_address.empty() && !options.forward_address.empty();
}

/**
 * @brief 將 IPv4 字串轉為 in_addr
 * @param address IPv4 字串
 * @param[out] out 轉換結果
 * @return true 表示位址有效
 */
bool parse_ipv4(const std::string& address, in_addr& out)
{
    return ::inet_pton(AF_INET, address.c_str(), &out) == 1;
}

/**
 * @brief 解析同儕容器在 Docker 網路上的 IPv4
 * @param host 主機名稱
 * @param[out] out 解析結果
 * @return true 表示取得 IPv4
 */
bool resolve_ipv4(const std::string& host, in_addr& out)
{
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* result = nullptr;
    if (host.empty() || ::getaddrinfo(host.c_str(), nullptr, &hints, &result) != 0 || result == nullptr)
    {
        return false;
    }
    const auto* address = reinterpret_cast<const sockaddr_in*>(result->ai_addr);
    const bool is_ok = address != nullptr;
    if (is_ok)
    {
        out = address->sin_addr;
    }
    ::freeaddrinfo(result);
    return is_ok;
}

/// 各同儕的轉送目的位址；解析失敗時沿用主機區網 IP
class PeerDestinations
{
public:
    PeerDestinations(const RelayOptions& options, const in_addr& fallback)
        : options_(options), fallback_(fallback)
    {
    }

    /** @brief 重新解析所有同儕主機名稱；位址改變時寫入日誌 */
    void refresh()
    {
        for (unsigned int peer = 1U; peer <= options_.plan.instance_count; ++peer)
        {
            if (peer == options_.plan.instance)
            {
                continue;
            }
            in_addr resolved = fallback_;
            const std::string host = taiphoon::utils::make_dds_peer_host(options_.peer_host_format, peer);
            const bool is_resolved = resolve_ipv4(host, resolved);
            const auto previous = addresses_.find(peer);
            if (previous == addresses_.end() || previous->second.s_addr != resolved.s_addr)
            {
                char text[INET_ADDRSTRLEN] = {0};
                ::inet_ntop(AF_INET, &resolved, text, sizeof(text));
                std::cerr << "[dds_peer_relay] Agent " << peer << " → " << text
                          << (is_resolved ? "" : "（無法解析容器位址，改用主機埠映射）") << '\n';
            }
            addresses_[peer] = resolved;
        }
    }

    /**
     * @brief 取得指定埠的轉送目的地
     * @param port 埠號
     * @param[out] out 目的位址
     * @return true 表示埠屬於某個同儕
     */
    bool destination_for(const std::uint16_t port, sockaddr_in& out) const
    {
        const unsigned int peer = taiphoon::utils::dds_relay_instance_for_port(options_.plan, port);
        const auto iterator = addresses_.find(peer);
        std::memset(&out, 0, sizeof(out));
        out.sin_family = AF_INET;
        out.sin_port = htons(port);
        out.sin_addr = iterator != addresses_.end() ? iterator->second : fallback_;
        return peer != 0U;
    }

private:
    const RelayOptions& options_;
    in_addr fallback_;
    std::map<unsigned int, in_addr> addresses_{};
};

/**
 * @brief 在本機 IP 綁定所有同儕埠
 * @param bind_address 本機 IPv4
 * @param ports 同儕埠
 * @param[out] receivers 已綁定的 socket
 * @param[out] poll_fds 對應的 pollfd
 * @param[out] bound_ports 對應的埠號
 */
void bind_receivers(
    const in_addr& bind_address, const std::vector<std::uint16_t>& ports, std::vector<UdpSocket>& receivers,
    std::vector<pollfd>& poll_fds, std::vector<std::uint16_t>& bound_ports)
{
    receivers.reserve(ports.size());
    for (const std::uint16_t port : ports)
    {
        UdpSocket receiver;
        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port = htons(port);
        local.sin_addr = bind_address;
        if (receiver.fd() < 0
            || ::bind(receiver.fd(), reinterpret_cast<const sockaddr*>(&local), sizeof(local)) != 0)
        {
            // 埠已被其他程序（例如 MicroXRCEAgent 的 Fast DDS participant）占用時略過；
            // 只影響對應 participant index，不影響其他埠的轉送。
            std::cerr << "[dds_peer_relay] 略過埠 " << port << "：" << std::strerror(errno) << '\n';
            continue;
        }
        poll_fds.push_back(pollfd{receiver.fd(), POLLIN, 0});
        bound_ports.push_back(port);
        receivers.push_back(std::move(receiver));
    }
}

}  // namespace

int main(int argc, char* argv[])
{
    RelayOptions options;
    in_addr bind_address{};
    in_addr forward_address{};
    if (!parse_options(argc, argv, options) || !parse_ipv4(options.bind_address, bind_address)
        || !parse_ipv4(options.forward_address, forward_address))
    {
        std::cerr << "用法：dds_peer_relay --bind <容器 IPv4> --forward <主機 IPv4> "
                     "--domain <id> --instance <N> --count <總數> [--peer-host-format uav_{N}_agent]\n";
        return 2;
    }
    std::vector<std::uint16_t> ports;
    if (!taiphoon::utils::make_dds_peer_relay_ports(options.plan, ports))
    {
        std::cerr << "[dds_peer_relay] domain／instance／count 超出支援範圍\n";
        return 2;
    }

    std::signal(SIGTERM, stop_cb);
    std::signal(SIGINT, stop_cb);

    if (ports.empty())
    {
        // 單機模擬沒有同儕；保持執行到收到結束訊號，讓 runtime 監控不把它當成異常退出。
        std::cerr << "[dds_peer_relay] 只有 1 台 Agent，不需轉送\n";
        while (g_should_stop == 0)
        {
            ::pause();
        }
        return 0;
    }

    UdpSocket sender;
    if (sender.fd() < 0)
    {
        std::cerr << "[dds_peer_relay] 無法建立轉送 socket：" << std::strerror(errno) << '\n';
        return 1;
    }

    std::vector<UdpSocket> receivers;
    std::vector<pollfd> poll_fds;
    std::vector<std::uint16_t> bound_ports;
    bind_receivers(bind_address, ports, receivers, poll_fds, bound_ports);
    if (poll_fds.empty())
    {
        std::cerr << "[dds_peer_relay] 沒有可綁定的同儕埠，結束\n";
        return 1;
    }
    std::cerr << "[dds_peer_relay] " << options.bind_address << " 轉送 " << poll_fds.size() << " 個同儕埠\n";

    PeerDestinations destinations(options, forward_address);
    destinations.refresh();
    auto next_refresh = std::chrono::steady_clock::now() + PEER_RESOLVE_PERIOD;

    std::vector<char> buffer(MAX_DATAGRAM_BYTES);
    unsigned long send_errors = 0UL;
    while (g_should_stop == 0)
    {
        if (std::chrono::steady_clock::now() >= next_refresh)
        {
            destinations.refresh();
            next_refresh = std::chrono::steady_clock::now() + PEER_RESOLVE_PERIOD;
        }
        const int ready = ::poll(poll_fds.data(), poll_fds.size(), POLL_TIMEOUT_MS);
        if (ready < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            std::cerr << "[dds_peer_relay] poll 失敗：" << std::strerror(errno) << '\n';
            return 1;
        }
        for (std::size_t index = 0U; index < poll_fds.size() && ready > 0; ++index)
        {
            if ((poll_fds.at(index).revents & POLLIN) == 0)
            {
                continue;
            }
            const ssize_t received = ::recv(poll_fds.at(index).fd, buffer.data(), buffer.size(), 0);
            sockaddr_in destination{};
            if (received <= 0 || !destinations.destination_for(bound_ports.at(index), destination))
            {
                continue;
            }
            const ssize_t sent = ::sendto(sender.fd(), buffer.data(), static_cast<std::size_t>(received), 0,
                reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
            if (sent < 0 && (send_errors++ % SEND_ERROR_LOG_INTERVAL) == 0UL)
            {
                std::cerr << "[dds_peer_relay] 轉送至埠 " << bound_ports.at(index) << " 失敗："
                          << std::strerror(errno) << '\n';
            }
        }
    }
    return 0;
}
