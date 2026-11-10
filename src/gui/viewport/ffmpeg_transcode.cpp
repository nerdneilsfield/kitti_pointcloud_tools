#include "gui/viewport/ffmpeg_transcode.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;
#endif

namespace kpt::gui {
namespace {

enum class SpawnOutcome { Completed, Failed, Cancelled };

struct SpawnResult {
  SpawnOutcome outcome = SpawnOutcome::Failed;
  std::string message;
};

#if defined(_WIN32)

bool isExecutable(const std::filesystem::path &candidate) {
  std::error_code error;
  return std::filesystem::is_regular_file(candidate, error);
}

void appendPathCandidates(std::vector<std::filesystem::path> &out) {
  const DWORD length = ::GetEnvironmentVariableW(L"PATH", nullptr, 0);
  if (length == 0)
    return;
  std::wstring value(length, L'\0');
  const DWORD copied =
      ::GetEnvironmentVariableW(L"PATH", value.data(), length);
  if (copied == 0 || copied >= length)
    return;
  value.resize(copied);
  std::size_t begin = 0;
  while (begin < value.size()) {
    const std::size_t end = value.find(L';', begin);
    const std::wstring entry = value.substr(begin, end - begin);
    if (!entry.empty())
      out.emplace_back(entry);
    if (end == std::wstring::npos)
      break;
    begin = end + 1;
  }
}

// Quotes one argument for CreateProcessW's command line, following the
// backslash/quote rules documented for CommandLineToArgvW.
std::wstring quoteArgument(const std::wstring &value) {
  std::wstring quoted = L"\"";
  std::size_t backslashes = 0;
  for (const wchar_t character : value) {
    if (character == L'\\') {
      ++backslashes;
      continue;
    }
    if (character == L'"') {
      quoted.append(backslashes * 2U + 1U, L'\\');
      quoted.push_back(L'"');
      backslashes = 0;
      continue;
    }
    quoted.append(backslashes, L'\\');
    backslashes = 0;
    quoted.push_back(character);
  }
  quoted.append(backslashes * 2U, L'\\');
  quoted.push_back(L'"');
  return quoted;
}

struct HandleCloser {
  HANDLE handle = INVALID_HANDLE_VALUE;
  ~HandleCloser() {
    if (handle != INVALID_HANDLE_VALUE && handle != nullptr)
      ::CloseHandle(handle);
  }
};

SpawnResult runFfmpeg(const std::filesystem::path &ffmpeg,
                      const std::vector<std::filesystem::path> &args,
                      std::stop_token stop) {
  std::wstring command_line;
  for (const auto &arg : args) {
    if (!command_line.empty())
      command_line.push_back(L' ');
    command_line += quoteArgument(arg.wstring());
  }

  // CreateFileW takes a non-const LPSECURITY_ATTRIBUTES.
  SECURITY_ATTRIBUTES inheritable{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
  HandleCloser null_in{::CreateFileW(
      L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, &inheritable,
      OPEN_EXISTING, 0, nullptr)};
  HandleCloser null_out{::CreateFileW(
      L"NUL", GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, &inheritable,
      OPEN_EXISTING, 0, nullptr)};

  STARTUPINFOW startup{};
  startup.cb = sizeof(startup);
  if (null_in.handle != INVALID_HANDLE_VALUE &&
      null_out.handle != INVALID_HANDLE_VALUE) {
    startup.dwFlags |= STARTF_USESTDHANDLES;
    startup.hStdInput = null_in.handle;
    startup.hStdOutput = null_out.handle;
    startup.hStdError = null_out.handle;
  }

  PROCESS_INFORMATION process{};
  if (::CreateProcessW(ffmpeg.c_str(), command_line.data(), nullptr, nullptr,
                       TRUE, CREATE_NO_WINDOW, nullptr, nullptr, &startup,
                       &process) == FALSE) {
    return {SpawnOutcome::Failed,
            "cannot start ffmpeg (error " +
                std::to_string(static_cast<int>(::GetLastError())) + ")"};
  }
  HandleCloser process_handle{process.hProcess};
  HandleCloser thread_handle{process.hThread};

  while (true) {
    const DWORD waited = ::WaitForSingleObject(process.hProcess, 50);
    if (waited == WAIT_OBJECT_0)
      break;
    if (waited == WAIT_FAILED) {
      ::TerminateProcess(process.hProcess, 1);
      ::WaitForSingleObject(process.hProcess, INFINITE);
      return {SpawnOutcome::Failed, "cannot wait for ffmpeg"};
    }
    if (stop.stop_requested()) {
      // Only the child this call created.
      ::TerminateProcess(process.hProcess, 1);
      ::WaitForSingleObject(process.hProcess, INFINITE);
      return {SpawnOutcome::Cancelled, {}};
    }
  }

  DWORD exit_code = 1;
  if (::GetExitCodeProcess(process.hProcess, &exit_code) == FALSE)
    return {SpawnOutcome::Failed, "cannot read the ffmpeg exit status"};
  if (exit_code != 0)
    return {SpawnOutcome::Failed,
            "ffmpeg exited with status " + std::to_string(exit_code)};
  return {SpawnOutcome::Completed, {}};
}

#else

bool isExecutable(const std::filesystem::path &candidate) {
  std::error_code error;
  if (!std::filesystem::is_regular_file(candidate, error))
    return false;
  return ::access(candidate.c_str(), X_OK) == 0;
}

void appendPathCandidates(std::vector<std::filesystem::path> &out) {
  const char *path = std::getenv("PATH");
  if (path == nullptr)
    return;
  std::string_view remaining(path);
  while (true) {
    const std::size_t colon = remaining.find(':');
    const std::string_view entry = remaining.substr(0, colon);
    if (!entry.empty())
      out.emplace_back(std::string(entry));
    if (colon == std::string_view::npos)
      break;
    remaining.remove_prefix(colon + 1);
  }
}

struct FileActions {
  posix_spawn_file_actions_t actions{};
  bool initialized = false;

  FileActions() : initialized(posix_spawn_file_actions_init(&actions) == 0) {}
  ~FileActions() {
    if (initialized)
      posix_spawn_file_actions_destroy(&actions);
  }
  FileActions(const FileActions &) = delete;
  FileActions &operator=(const FileActions &) = delete;

  bool redirect(int descriptor, const char *path, int flags) {
    return initialized &&
           posix_spawn_file_actions_addopen(&actions, descriptor, path, flags,
                                            0) == 0;
  }
};

SpawnResult runFfmpeg(const std::filesystem::path &ffmpeg,
                      const std::vector<std::filesystem::path> &args,
                      std::stop_token stop) {
  FileActions actions;
  if (!actions.initialized)
    return {SpawnOutcome::Failed, "cannot initialize ffmpeg spawn actions"};
  if (!actions.redirect(STDIN_FILENO, "/dev/null", O_RDONLY) ||
      !actions.redirect(STDOUT_FILENO, "/dev/null", O_WRONLY) ||
      !actions.redirect(STDERR_FILENO, "/dev/null", O_WRONLY))
    return {SpawnOutcome::Failed, "cannot redirect the ffmpeg process"};

  std::vector<std::string> owned;
  owned.reserve(args.size());
  for (const auto &arg : args)
    owned.push_back(arg.string());
  std::vector<char *> argv;
  argv.reserve(owned.size() + 1U);
  for (auto &arg : owned)
    argv.push_back(arg.data());
  argv.push_back(nullptr);

  pid_t pid = -1;
  const int spawn_error = posix_spawn(&pid, ffmpeg.c_str(), &actions.actions,
                                      nullptr, argv.data(), environ);
  if (spawn_error != 0)
    return {SpawnOutcome::Failed,
            "cannot start ffmpeg: " +
                std::generic_category().message(spawn_error)};

  int status = 0;
  while (true) {
    const pid_t waited = ::waitpid(pid, &status, WNOHANG);
    if (waited == pid)
      break;
    if (waited < 0) {
      if (errno == EINTR)
        continue;
      const int wait_error = errno;
      ::kill(pid, SIGKILL);
      ::waitpid(pid, &status, 0);
      return {SpawnOutcome::Failed,
              "cannot wait for ffmpeg: " +
                  std::generic_category().message(wait_error)};
    }
    if (stop.stop_requested()) {
      // Only the child this call spawned is signalled.
      ::kill(pid, SIGKILL);
      ::waitpid(pid, &status, 0);
      return {SpawnOutcome::Cancelled, {}};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (WIFEXITED(status)) {
    if (WEXITSTATUS(status) == 0)
      return {SpawnOutcome::Completed, {}};
    return {SpawnOutcome::Failed,
            "ffmpeg exited with status " +
                std::to_string(WEXITSTATUS(status))};
  }
  if (WIFSIGNALED(status))
    return {SpawnOutcome::Failed, "ffmpeg was terminated by signal " +
                                      std::to_string(WTERMSIG(status))};
  return {SpawnOutcome::Failed, "ffmpeg terminated unexpectedly"};
}

#endif

// The output file name keeps the .mp4 extension: ffmpeg selects the muxer
// from it. AtomicReplace needs the same directory, so the temporary stays
// beside the destination.
std::filesystem::path makeTemporaryPath(const std::filesystem::path &mp4) {
  static std::atomic<std::uint64_t> sequence{0};
  const auto stamp =
      std::chrono::steady_clock::now().time_since_epoch().count();
  // Keep the stem as a native path component so non-ASCII names survive on
  // Windows; only the ASCII suffix and extension are converted.
  std::filesystem::path temporary = mp4.parent_path() / mp4.stem();
  temporary += std::filesystem::path(".part-" + std::to_string(stamp) + "-" +
                                     std::to_string(sequence.fetch_add(1U)) +
                                     mp4.extension().string());
  return temporary;
}

void removeTemporary(const std::filesystem::path &temporary) {
  std::error_code ignored;
  std::filesystem::remove(temporary, ignored);
}

} // namespace

std::optional<std::filesystem::path> findFfmpegExecutable() {
  std::vector<std::filesystem::path> directories;
  appendPathCandidates(directories);
#if defined(__APPLE__)
  // A packaged .app started from Finder inherits no shell PATH, so the
  // Homebrew prefixes that actually hold ffmpeg are searched explicitly.
  directories.emplace_back("/opt/homebrew/bin");
  directories.emplace_back("/usr/local/bin");
#endif
#if defined(_WIN32)
  constexpr const char *kExecutableName = "ffmpeg.exe";
#else
  constexpr const char *kExecutableName = "ffmpeg";
#endif
  for (const auto &directory : directories) {
    const std::filesystem::path candidate = directory / kExecutableName;
    if (isExecutable(candidate))
      return candidate;
  }
  return std::nullopt;
}

RecordingResult
transcodeAviToMp4(const std::filesystem::path &ffmpeg,
                  const std::filesystem::path &avi,
                  const std::filesystem::path &mp4, bool overwrite,
                  std::stop_token stop) {
  std::error_code error;
  if (!std::filesystem::is_regular_file(avi, error))
    return {RecordingStatus::Failed, "AVI recording is unavailable"};
  if (stop.stop_requested())
    return {RecordingStatus::Cancelled, "operation cancelled"};
  // Early exit only: avoid spawning ffmpeg when the destination already exists
  // and must not be replaced. publishRecordedFile repeats this check when the
  // temporary file is ready.
  if (!overwrite && std::filesystem::exists(mp4, error))
    return {RecordingStatus::Skipped, "destination already exists"};

  const std::filesystem::path temporary = makeTemporaryPath(mp4);
  // Arguments stay paths so the Windows branch can quote native wide text;
  // flag values such as pad=... are plain ASCII and survive unchanged.
  const std::vector<std::filesystem::path> args{
      ffmpeg,
      "-nostdin",
      "-hide_banner",
      "-loglevel",
      "error",
      "-y",
      "-i",
      avi,
      // libx264 rejects odd dimensions, and recorded viewports can be odd.
      "-vf",
      "pad=ceil(iw/2)*2:ceil(ih/2)*2",
      "-c:v",
      "libx264",
      "-pix_fmt",
      "yuv420p",
      "-movflags",
      "+faststart",
      temporary};

  const SpawnResult spawned = runFfmpeg(ffmpeg, args, stop);
  if (spawned.outcome == SpawnOutcome::Cancelled) {
    removeTemporary(temporary);
    return {RecordingStatus::Cancelled, "operation cancelled"};
  }
  if (spawned.outcome == SpawnOutcome::Failed) {
    removeTemporary(temporary);
    return {RecordingStatus::Failed, spawned.message};
  }

  // A zero exit status is not enough: ffmpeg also exits 0 when it declines to
  // overwrite, and a failed encode can leave an empty file behind.
  const auto produced = std::filesystem::file_size(temporary, error);
  if (error || produced == 0U) {
    removeTemporary(temporary);
    return {RecordingStatus::Failed, "ffmpeg produced no output"};
  }
  // publishRecordedFile owns the overwrite policy and the atomic replace; it
  // also removes `temporary` when the destination appeared meanwhile.
  return publishRecordedFile(temporary, mp4, overwrite);
}

} // namespace kpt::gui
