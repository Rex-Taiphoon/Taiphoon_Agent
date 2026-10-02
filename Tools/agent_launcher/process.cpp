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
#include <windows.h>
#else
#include <cerrno>
#include <sys/wait.h>
#include <unistd.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif
#endif

namespace taiphoon::deployment
{
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

ProcessResult run(const std::vector<std::string>& arguments, bool should_capture)
{
    if (arguments.empty())
    {
        throw std::invalid_argument("缺少程序名稱");
    }
    ProcessResult result;
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
    if (!CreateProcessW(nullptr, command.data(), nullptr, nullptr,
        should_capture ? TRUE : FALSE, 0, nullptr, nullptr, &startup, &information))
    {
        throw std::runtime_error("無法執行 " + arguments.front() + "；請檢查安裝與 PATH");
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
        for (const auto& argument : arguments)
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
