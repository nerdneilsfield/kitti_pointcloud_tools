#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#endif

#include "platform/settings_store.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <system_error>
#include <utility>

#if defined(_WIN32)
#include <windows.h>

#include <bcrypt.h>
#elif defined(__APPLE__)
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#else
#include <fcntl.h>
#include <sys/random.h>
#include <unistd.h>
#endif

namespace kpt::platform {
namespace {

PlatformError settingsError(std::string message,
                            std::error_code system_error = {}) {
  return {PlatformErrorCode::SettingsIoFailed, std::move(message),
          system_error};
}

std::uint64_t processId() {
#if defined(_WIN32)
  return static_cast<std::uint64_t>(GetCurrentProcessId());
#else
  return static_cast<std::uint64_t>(getpid());
#endif
}

PlatformResult<std::string> randomNonce() {
  std::array<std::uint64_t, 2> words{};
#if defined(_WIN32)
  const NTSTATUS status = BCryptGenRandom(
      nullptr, reinterpret_cast<PUCHAR>(words.data()),
      static_cast<ULONG>(sizeof(words)), BCRYPT_USE_SYSTEM_PREFERRED_RNG);
  if (status < 0) {
    return settingsError("cannot obtain secure settings nonce",
                         {static_cast<int>(status), std::system_category()});
  }
#elif defined(__APPLE__)
  arc4random_buf(words.data(), sizeof(words));
#else
  std::size_t offset = 0;
  while (offset < sizeof(words)) {
    const auto count =
        getrandom(reinterpret_cast<unsigned char *>(words.data()) + offset,
                  sizeof(words) - offset, 0);
    if (count < 0 && errno == EINTR)
      continue;
    if (count <= 0) {
      return settingsError("cannot obtain secure settings nonce",
                           {errno, std::generic_category()});
    }
    offset += static_cast<std::size_t>(count);
  }
#endif
  std::ostringstream encoded;
  encoded << std::hex;
  for (const auto word : words)
    encoded << word;
  return encoded.str();
}

PlatformResult<void> writeExclusive(const std::filesystem::path &file,
                                    std::string_view contents) {
#if defined(_WIN32)
  const HANDLE handle =
      CreateFileW(file.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW,
                  FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH, nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    return settingsError(
        "cannot exclusively create temporary settings file",
        {static_cast<int>(GetLastError()), std::system_category()});
  }

  std::size_t offset = 0;
  bool ok = true;
  while (offset < contents.size()) {
    const auto remaining = contents.size() - offset;
    const DWORD request = static_cast<DWORD>(
        std::min<std::size_t>(remaining, static_cast<std::size_t>(MAXDWORD)));
    DWORD written = 0;
    if (WriteFile(handle, contents.data() + offset, request, &written,
                  nullptr) == FALSE ||
        written == 0) {
      ok = false;
      break;
    }
    offset += written;
  }
  if (ok)
    ok = FlushFileBuffers(handle) != FALSE;
  const DWORD write_error = ok ? ERROR_SUCCESS : GetLastError();
  CloseHandle(handle);
  if (!ok) {
    std::error_code ignored;
    std::filesystem::remove(file, ignored);
    return settingsError(
        "cannot write temporary settings file",
        {static_cast<int>(write_error), std::system_category()});
  }
#else
  const int descriptor =
      ::open(file.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (descriptor < 0) {
    return settingsError("cannot exclusively create temporary settings file",
                         {errno, std::generic_category()});
  }

  std::size_t offset = 0;
  bool ok = true;
  int write_error = 0;
  while (offset < contents.size()) {
    const ssize_t written =
        ::write(descriptor, contents.data() + offset, contents.size() - offset);
    if (written < 0 && errno == EINTR)
      continue;
    if (written <= 0) {
      ok = false;
      write_error = errno;
      break;
    }
    offset += static_cast<std::size_t>(written);
  }
  if (ok && ::fsync(descriptor) != 0) {
    ok = false;
    write_error = errno;
  }
  if (::close(descriptor) != 0 && ok) {
    ok = false;
    write_error = errno;
  }
  if (!ok) {
    std::error_code ignored;
    std::filesystem::remove(file, ignored);
    return settingsError("cannot write temporary settings file",
                         {write_error, std::generic_category()});
  }
#endif
  return {};
}

class FileSettingsStore final : public SettingsStore {
public:
  FileSettingsStore(std::filesystem::path ini_file,
                    std::unique_ptr<detail::AtomicReplace> atomic_replace)
      : ini_file_(std::move(ini_file)),
        atomic_replace_(std::move(atomic_replace)) {}

  PlatformResult<std::optional<std::string>> loadIni() const override {
    std::error_code error;
    const bool exists = std::filesystem::exists(ini_file_, error);
    if (error)
      return settingsError("cannot inspect settings file", error);
    if (!exists)
      return std::optional<std::string>{};
    if (!std::filesystem::is_regular_file(ini_file_, error) || error)
      return settingsError("settings path is not a regular file", error);

    std::ifstream input(ini_file_, std::ios::binary);
    if (!input)
      return settingsError("cannot open settings file for reading");

    std::ostringstream contents;
    contents << input.rdbuf();
    if (input.bad())
      return settingsError("cannot read settings file");
    return std::optional<std::string>{contents.str()};
  }

  PlatformResult<void> saveIniAtomically(std::string_view contents) override {
    std::error_code error;
    std::filesystem::create_directories(ini_file_.parent_path(), error);
    if (error)
      return settingsError("cannot create settings directory", error);

    std::filesystem::path temporary;
    bool created = false;
    PlatformError create_error;
    for (int attempt = 0; attempt < 32; ++attempt) {
      auto nonce = randomNonce();
      if (!nonce)
        return std::move(nonce).error();
      temporary = ini_file_;
      temporary += ".tmp." + std::to_string(processId()) + "." +
                   std::move(nonce).value();
      auto written = writeExclusive(temporary, contents);
      if (written) {
        created = true;
        break;
      }
      create_error = std::move(written).error();
      if (create_error.system_error !=
          std::make_error_code(std::errc::file_exists)) {
#if defined(_WIN32)
        if (create_error.system_error.value() != ERROR_FILE_EXISTS &&
            create_error.system_error.value() != ERROR_ALREADY_EXISTS)
          return create_error;
#else
        return create_error;
#endif
      }
    }
    if (!created)
      return create_error;

    auto replaced = atomic_replace_->replace(temporary, ini_file_);
    if (!replaced) {
      std::filesystem::remove(temporary, error);
      return std::move(replaced).error();
    }
    return {};
  }

private:
  std::filesystem::path ini_file_;
  std::unique_ptr<detail::AtomicReplace> atomic_replace_;
};

class UnavailableSettingsStore final : public SettingsStore {
public:
  explicit UnavailableSettingsStore(PlatformError error)
      : error_(std::move(error)) {}

  PlatformResult<std::optional<std::string>> loadIni() const override {
    return error_;
  }

  PlatformResult<void> saveIniAtomically(std::string_view) override {
    return error_;
  }

private:
  PlatformError error_;
};

} // namespace

std::unique_ptr<SettingsStore>
makeSettingsStore(std::filesystem::path ini_file,
                  std::unique_ptr<detail::AtomicReplace> atomic_replace) {
  return std::make_unique<FileSettingsStore>(std::move(ini_file),
                                             std::move(atomic_replace));
}

std::unique_ptr<SettingsStore>
makeUnavailableSettingsStore(PlatformError error) {
  return std::make_unique<UnavailableSettingsStore>(std::move(error));
}

} // namespace kpt::platform
