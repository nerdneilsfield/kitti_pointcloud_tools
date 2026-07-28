#include "platform/settings_store.hpp"

#include <atomic>
#include <fstream>
#include <sstream>
#include <utility>

namespace kpt::platform {
namespace {

PlatformError settingsError(std::string message,
                            std::error_code system_error = {}) {
  return {PlatformErrorCode::SettingsIoFailed, std::move(message),
          system_error};
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

    const auto serial = next_temp_.fetch_add(1, std::memory_order_relaxed);
    auto temporary = ini_file_;
    temporary += ".tmp." + std::to_string(serial);

    {
      std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
      if (!output)
        return settingsError("cannot open temporary settings file");
      output.write(contents.data(),
                   static_cast<std::streamsize>(contents.size()));
      output.flush();
      if (!output) {
        output.close();
        std::filesystem::remove(temporary, error);
        return settingsError("cannot write temporary settings file");
      }
    }

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
  static std::atomic_uint64_t next_temp_;
};

std::atomic_uint64_t FileSettingsStore::next_temp_{0};

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
