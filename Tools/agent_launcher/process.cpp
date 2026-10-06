/**
 * @file process.cpp
 * @brief 原生程序執行；不以 shell 解譯使用者輸入
 * @author Taiphoon Agent Team
 * @date 2026-10-02
 */
#include "launcher.hpp"
#include <array>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#ifdef _WIN32
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif
#endif

namespace taiphoon::deployment
{
namespace
{
/** @brief 確保路由查詢釋放 socket；Windows 同時管理 Winsock 初始化。 */
class RouteSocket
{
public:
#ifdef _WIN32
    SOCKET value_ = INVALID_SOCKET;
    bool is_initialized_ = false;
    /** @brief 初始化 Winsock 與唯讀路由 socket。 */
    RouteSocket()
    {
        WSADATA data{};
        is_initialized_ = WSAStartup(MAKEWORD(2, 2), &data) == 0;
        if (is_initialized_)
        {
            value_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        }
    }
    /** @brief 釋放 socket 與本次 Winsock 參照。 */
    ~RouteSocket()
    {
        if (value_ != INVALID_SOCKET)
        {
            closesocket(value_);
        }
        if (is_initialized_)
        {
            WSACleanup();
        }
    }
    /** @brief 回報資源是否可用。 @return 是否已建立 socket。 */
    bool isValid() const
    {
        return value_ != INVALID_SOCKET;
    }
#else
    int value_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    /** @brief 釋放本次路由查詢 socket。 */
    ~RouteSocket()
    {
        if (value_ >= 0)
        {
            close(value_);
        }
    }
    /** @brief 回報資源是否可用。 @return 是否已建立 socket。 */
    bool isValid() const
    {
        return value_ >= 0;
    }
#endif
    RouteSocket(const RouteSocket&) = delete;
    RouteSocket& operator=(const RouteSocket&) = delete;
#ifndef _WIN32
    /** @brief 建立 UDP socket，不連線或傳送資料。 */
    RouteSocket() = default;
#endif
};
}

std::string localIpv4ForPeer(const std::string& peer)
{
    RouteSocket route;
    if (!route.isValid())
    {
        return {};
    }
    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    constexpr unsigned short ROUTE_QUERY_PORT = 9; // UDP connect 只查路由，沒有 send。
    destination.sin_port = htons(ROUTE_QUERY_PORT);
    if (inet_pton(AF_INET, peer.c_str(), &destination.sin_addr) != 1
        || connect(route.value_, reinterpret_cast<const sockaddr*>(&destination), sizeof(destination)) != 0)
    {
        return {};
    }
    sockaddr_in local{};
#ifdef _WIN32
    int length = sizeof(local);
#else
    socklen_t length = sizeof(local);
#endif
    if (getsockname(route.value_, reinterpret_cast<sockaddr*>(&local), &length) != 0)
    {
        return {};
    }
    std::array<char, INET_ADDRSTRLEN> address{};
    if (inet_ntop(AF_INET, &local.sin_addr, address.data(), address.size()) == nullptr)
    {
        return {};
    }
    return address.data();
}

fs::path findExecutable(const std::vector<fs::path>& directories, const std::string& name)
{
    for (const auto& directory : directories)
    {
        if (directory.empty())
        {
            continue;
        }
        std::error_code error;
        const auto candidate = directory / name;
        if (fs::is_regular_file(candidate, error))
        {
            const auto absolute = fs::absolute(candidate, error);
            if (!error)
            {
                return absolute;
            }
        }
    }
    return {};
}

std::wstring quoteWindowsArgument(const std::wstring& value)
{
    std::wstring result = L"\"";
    std::size_t slashes = 0;
    for (const wchar_t character : value)
    {
        if (character == L'\\')
        {
            ++slashes;
            continue;
        }
        result.append(character == L'"' ? slashes * 2 + 1 : slashes, L'\\');
        result += character;
        slashes = 0;
    }
    result.append(slashes * 2, L'\\');
    return result + L'"';
}

#ifdef _WIN32
namespace
{
/** @brief 管理 Windows handle，包含程序建立失敗的清理路徑。 */
class Handle
{
public:
    HANDLE value_ = nullptr;
    ~Handle()
    {
        if (value_ != nullptr && value_ != INVALID_HANDLE_VALUE)
        {
            CloseHandle(value_);
        }
    }
    Handle() = default;
    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;
};

/** @brief 明確將 UTF-8 argv 轉成 Windows Unicode。 */
std::wstring widen(const std::string& value)
{
    if (value.empty())
    {
        return {};
    }
    const int size = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS,
        value.data(), static_cast<int>(value.size()), nullptr, 0);
    if (size == 0)
    {
        throw std::runtime_error("程序參數不是有效 UTF-8");
    }
    std::wstring result(static_cast<std::size_t>(size), L'\0');
    if (MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, value.data(),
        static_cast<int>(value.size()), result.data(), size) == 0)
    {
        throw std::runtime_error("Windows 參數轉換失敗");
    }
    return result;
}

/** @brief Explorer 的 PATH 可能尚未更新；備援搜尋標準安裝位置。
 * @return Docker CLI 的絕對 Unicode 路徑；缺少依賴時提供處理方式。
 */
fs::path windowsDockerExecutable()
{
    std::vector<fs::path> directories;
    const wchar_t* path_value = _wgetenv(L"PATH");
    const std::wstring search_path = path_value == nullptr ? L"" : path_value;
    std::size_t begin = 0;
    while (begin < search_path.size())
    {
        const auto end = search_path.find(L';', begin);
        auto directory = search_path.substr(begin, end - begin);
        if (directory.size() >= 2 && directory.front() == L'"' && directory.back() == L'"')
        {
            directory = directory.substr(1, directory.size() - 2);
        }
        directories.emplace_back(directory);
        if (end == std::wstring::npos)
        {
            break;
        }
        begin = end + 1;
    }
    const wchar_t* program_files = _wgetenv(L"ProgramFiles");
    if (program_files != nullptr && program_files[0] != L'\0')
    {
        directories.push_back(fs::path(program_files) / "Docker" / "Docker" / "resources" / "bin");
    }
    const wchar_t* local_data = _wgetenv(L"LOCALAPPDATA");
    if (local_data != nullptr && local_data[0] != L'\0')
    {
        directories.push_back(fs::path(local_data) / "Programs" / "DockerDesktop" / "resources" / "bin");
    }
    const auto executable = findExecutable(directories, "docker.exe");
    if (executable.empty())
    {
        throw std::runtime_error("找不到 Windows Docker CLI（docker.exe）。\n"
            "請安裝並啟動 Docker Desktop，使用 WSL2／Linux 容器後再開啟本工具。\n"
            "安裝說明：https://docs.docker.com/desktop/setup/install/windows-install/\n"
            "僅在 WSL Ubuntu 安裝 Docker Engine 無法供 Windows exe 使用。\n"
            "若已安裝於自訂位置，請將其 resources\\bin 加入 Windows PATH，再重新開啟本工具。\n"
            "已保存的設定可以保留，不需要重新選擇模式。");
    }
    return executable;
}
}
#else
namespace
{
/** @brief 以 RAII 管理 pipe，fork 失敗與父程序例外均會釋放描述符。 */
class Pipe
{
public:
    /** @brief 只有擷取 stdout 時建立 pipe。 */
    explicit Pipe(bool is_enabled)
    {
        if (is_enabled && pipe(descriptors_.data()) != 0)
        {
            throw std::runtime_error("無法建立程序輸出 pipe");
        }
    }
    /** @brief 釋放尚未關閉的兩端。 */
    ~Pipe()
    {
        closeRead();
        closeWrite();
    }
    Pipe(const Pipe&) = delete;
    Pipe& operator=(const Pipe&) = delete;
    /** @brief 取得讀端以擷取輸出。 */
    int reader() const
    {
        return descriptors_[0];
    }
    /** @brief 取得寫端以建立子程序 stdout。 */
    int writer() const
    {
        return descriptors_[1];
    }
    /** @brief 提前釋放父子程序不再使用的讀端。 */
    void closeRead()
    {
        if (descriptors_[0] >= 0)
        {
            close(descriptors_[0]);
            descriptors_[0] = -1;
        }
    }
    /** @brief 提前釋放寫端，讓讀端可以收到 EOF。 */
    void closeWrite()
    {
        if (descriptors_[1] >= 0)
        {
            close(descriptors_[1]);
            descriptors_[1] = -1;
        }
    }
private:
    std::array<int, 2> descriptors_ = {-1, -1};
};
}
#endif

void runDetached(const std::vector<std::string>& arguments)
{
    if (arguments.empty())
    {
        throw std::invalid_argument("缺少背景程序名稱");
    }
#ifdef _WIN32
    std::wstring command;
    for (const auto& argument : arguments)
    {
        if (!command.empty())
        {
            command += L' ';
        }
        command += quoteWindowsArgument(widen(argument));
    }
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    startup.dwFlags = STARTF_USESHOWWINDOW;
    startup.wShowWindow = SW_HIDE;
    PROCESS_INFORMATION information{};
    if (!CreateProcessW(nullptr, command.data(), nullptr, nullptr, FALSE,
        CREATE_NO_WINDOW, nullptr, nullptr, &startup, &information))
    {
        throw std::runtime_error("無法啟動容器退出清理程序");
    }
    CloseHandle(information.hProcess);
    CloseHandle(information.hThread);
#else
    const pid_t child = fork();
    if (child < 0)
    {
        throw std::runtime_error("無法建立容器退出清理程序");
    }
    if (child == 0)
    {
        if (setsid() < 0)
        {
            _exit(1);
        }
        const pid_t detached = fork();
        if (detached < 0)
        {
            _exit(1);
        }
        if (detached > 0)
        {
            _exit(0);
        }
        const int null_descriptor = open("/dev/null", O_RDWR);
        if (null_descriptor < 0)
        {
            _exit(1);
        }
        for (int descriptor = 0; descriptor <= 2; ++descriptor)
        {
            if (dup2(null_descriptor, descriptor) < 0)
            {
                _exit(1);
            }
        }
        if (null_descriptor > 2)
        {
            close(null_descriptor);
        }
        std::vector<char*> values;
        for (const auto& argument : arguments)
        {
            values.push_back(const_cast<char*>(argument.c_str()));
        }
        values.push_back(nullptr);
        execvp(values.front(), values.data());
        _exit(127);
    }
    int status = 0;
    if (waitpid(child, &status, 0) != child || !WIFEXITED(status) || WEXITSTATUS(status) != 0)
    {
        throw std::runtime_error("容器退出清理程序啟動失敗");
    }
#endif
}

ProcessResult run(const std::vector<std::string>& arguments, bool should_capture)
{
    if (arguments.empty())
    {
        throw std::invalid_argument("缺少程序名稱");
    }
    ProcessResult result;
#ifdef _WIN32
    const auto executable = arguments.front() == "docker" ? windowsDockerExecutable() : fs::path{};
    std::wstring command;
    for (std::size_t index = 0; index < arguments.size(); ++index)
    {
        if (!command.empty())
        {
            command += L' ';
        }
        command += quoteWindowsArgument(index == 0 && !executable.empty()
            ? executable.wstring() : widen(arguments[index]));
    }
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    Handle reader;
    Handle writer;
    if (should_capture)
    {
        SECURITY_ATTRIBUTES attributes{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
        if (!CreatePipe(&reader.value_, &writer.value_, &attributes, 0)
            || !SetHandleInformation(reader.value_, HANDLE_FLAG_INHERIT, 0))
        {
            throw std::runtime_error("無法建立程序輸出 pipe");
        }
        startup.dwFlags = STARTF_USESTDHANDLES;
        startup.hStdOutput = writer.value_;
        startup.hStdError = GetStdHandle(STD_ERROR_HANDLE);
        startup.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    }
    PROCESS_INFORMATION information{};
    if (!CreateProcessW(executable.empty() ? nullptr : executable.c_str(), command.data(), nullptr, nullptr,
        should_capture ? TRUE : FALSE, should_capture ? CREATE_NO_WINDOW : 0, nullptr, nullptr, &startup, &information))
    {
        const auto error = GetLastError();
        throw std::runtime_error("無法執行 " + arguments.front() + "（Windows 錯誤 "
            + std::to_string(error) + "）；請檢查檔案是否完整，以及系統顯示的存取／安全性提示");
    }
    Handle process;
    Handle thread;
    process.value_ = information.hProcess;
    thread.value_ = information.hThread;
    if (should_capture)
    {
        CloseHandle(writer.value_);
        writer.value_ = nullptr;
        char buffer[4096];
        DWORD bytes = 0;
        while (true)
        {
            if (!ReadFile(reader.value_, buffer, sizeof(buffer), &bytes, nullptr))
            {
                if (GetLastError() != ERROR_BROKEN_PIPE)
                {
                    throw std::runtime_error("讀取程序輸出失敗");
                }
                break;
            }
            if (bytes == 0)
            {
                break;
            }
            result.output.append(buffer, bytes);
        }
    }
    if (WaitForSingleObject(process.value_, INFINITE) != WAIT_OBJECT_0)
    {
        throw std::runtime_error("等待程序失敗");
    }
    DWORD code = 1;
    if (!GetExitCodeProcess(process.value_, &code))
    {
        throw std::runtime_error("取得程序結果失敗");
    }
    result.code = static_cast<int>(code);
#else
    std::vector<std::string> actual_arguments = arguments;
#ifdef __APPLE__
    if (arguments.front() == "docker")
    {
        std::vector<fs::path> directories;
        const char* path_value = std::getenv("PATH");
        const std::string search_path = path_value == nullptr ? "" : path_value;
        std::size_t begin = 0;
        while (begin < search_path.size())
        {
            const auto end = search_path.find(':', begin);
            directories.push_back(fs::u8path(search_path.substr(begin, end - begin)));
            if (end == std::string::npos)
            {
                break;
            }
            begin = end + 1;
        }
        const char* user_home = std::getenv("HOME");
        if (user_home != nullptr)
        {
            directories.push_back(fs::u8path(user_home) / ".docker" / "bin");
        }
        directories.emplace_back("/Applications/Docker.app/Contents/Resources/bin");
        directories.emplace_back("/usr/local/bin");
        const auto executable = findExecutable(directories, "docker");
        if (!executable.empty())
        {
            actual_arguments[0] = executable.u8string();
        }
    }
#endif
    Pipe pipe_handles(should_capture);
    bool has_read_error = false;
    const pid_t child = fork();
    if (child < 0)
    {
        throw std::runtime_error("無法建立子程序");
    }
    if (child == 0)
    {
        if (should_capture)
        {
            pipe_handles.closeRead();
            if (dup2(pipe_handles.writer(), STDOUT_FILENO) < 0)
            {
                _exit(126);
            }
            pipe_handles.closeWrite();
        }
        std::vector<char*> argv;
        for (const auto& argument : actual_arguments)
        {
            argv.push_back(const_cast<char*>(argument.c_str()));
        }
        argv.push_back(nullptr);
        execvp(argv.front(), argv.data());
        _exit(127);
    }
    if (should_capture)
    {
        pipe_handles.closeWrite();
        char buffer[4096];
        ssize_t bytes = 0;
        do
        {
            bytes = read(pipe_handles.reader(), buffer, sizeof(buffer));
            if (bytes > 0)
            {
                result.output.append(buffer, static_cast<std::size_t>(bytes));
            }
        } while (bytes > 0 || (bytes < 0 && errno == EINTR));
        has_read_error = bytes < 0;
        pipe_handles.closeRead();
    }
    int status = 0;
    pid_t waited;
    do
    {
        waited = waitpid(child, &status, 0);
    } while (waited < 0 && errno == EINTR);
    if (waited < 0)
    {
        throw std::runtime_error("等待程序失敗");
    }
    if (has_read_error)
    {
        throw std::runtime_error("讀取程序輸出失敗");
    }
    result.code = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
#endif
    return result;
}

fs::path executableDirectory()
{
#ifdef _WIN32
    std::vector<wchar_t> path(32768);
    const DWORD length = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
    if (length == 0 || length >= path.size())
    {
        throw std::runtime_error("無法取得啟動工具位置");
    }
    return fs::canonical(fs::path(std::wstring(path.data(), length))).parent_path();
#elif defined(__APPLE__)
    std::uint32_t size = 0;
    _NSGetExecutablePath(nullptr, &size);
    std::vector<char> path(size);
    if (_NSGetExecutablePath(path.data(), &size) != 0)
    {
        throw std::runtime_error("無法取得啟動工具位置");
    }
    return fs::canonical(path.data()).parent_path();
#else
    return fs::canonical("/proc/self/exe").parent_path();
#endif
}

fs::path configurationDirectory()
{
#ifdef _WIN32
    const wchar_t* custom = _wgetenv(L"TAIPHOON_AGENT_HOME");
    if (custom != nullptr && custom[0] != L'\0')
    {
        return fs::absolute(fs::path(custom));
    }
    const wchar_t* base = _wgetenv(L"LOCALAPPDATA");
    if (base == nullptr || base[0] == L'\0')
    {
        throw std::runtime_error("無法取得使用者目錄；請設定 TAIPHOON_AGENT_HOME");
    }
    return fs::path(base) / "TaiphoonAgent" / "config";
#else
    const char* custom = std::getenv("TAIPHOON_AGENT_HOME");
    if (custom != nullptr && custom[0] != '\0')
    {
        return fs::absolute(fs::u8path(custom));
    }
    const char* base = std::getenv("HOME");
    if (base == nullptr || base[0] == '\0')
    {
        throw std::runtime_error("無法取得使用者目錄；請設定 TAIPHOON_AGENT_HOME");
    }
    return fs::u8path(base) / ".config" / "taiphoon_agent";
#endif
}
}
