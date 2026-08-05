#include "platform/services.hpp"

#include "platform/settings_store.hpp"
#include "platform/utf8_path.hpp"

#include <cstdlib>
#include <cerrno>
#include <utility>

#include <sys/stat.h>
#include <unistd.h>

namespace kpt::platform {
std::unique_ptr<Fonts> createLinuxFonts();

namespace {

class PosixPlatformLifetime final : public PlatformLifetime {};

PlatformError configurationError(std::string message,
                                 std::error_code system_error = {}) {
  return {PlatformErrorCode::ConfigurationDirectoryUnavailable,
          std::move(message), system_error};
}

PlatformResult<std::filesystem::path> environmentPath(const char *name,
                                                      const char *value) {
  auto decoded = pathFromUtf8(value);
  if (!decoded) {
    auto error = std::move(decoded).error();
    error.code = PlatformErrorCode::EnvironmentDecodeFailed;
    error.message = std::string("invalid UTF-8 in ") + name;
    return error;
  }
  return std::move(decoded).value();
}

class LinuxPaths final : public Paths {
public:
  PlatformResult<std::filesystem::path> configDirectory() const override {
    std::filesystem::path base;
    if (const char *xdg = std::getenv("XDG_CONFIG_HOME");
        xdg != nullptr && *xdg != '\0') {
      auto decoded = environmentPath("XDG_CONFIG_HOME", xdg);
      if (!decoded)
        return std::move(decoded).error();
      if (decoded.value().is_absolute())
        base = std::move(decoded).value();
    }

    if (base.empty()) {
      const char *home = std::getenv("HOME");
      if (home == nullptr || *home == '\0')
        return configurationError("HOME is unavailable");
      auto decoded = environmentPath("HOME", home);
      if (!decoded)
        return std::move(decoded).error();
      if (!decoded.value().is_absolute())
        return configurationError("HOME is not an absolute path");
      base = std::move(decoded).value() / ".config";
    }

    auto directory = base / "kpt";
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error)
      return configurationError("cannot create configuration directory", error);
    const auto status = std::filesystem::symlink_status(directory, error);
    if (error || std::filesystem::is_symlink(status) ||
        !std::filesystem::is_directory(status))
      return configurationError("configuration path is not a directory", error);
    if (::chmod(directory.c_str(), S_IRWXU) != 0)
      return configurationError("cannot restrict configuration directory",
                               {errno, std::generic_category()});
    struct stat information{};
    if (::lstat(directory.c_str(), &information) != 0 ||
        !S_ISDIR(information.st_mode) || information.st_uid != ::geteuid() ||
        (information.st_mode & (S_IRWXG | S_IRWXO)) != 0) {
      return configurationError(
          "configuration directory is not owner-only and user-owned");
    }
    return directory;
  }
};

} // namespace

PlatformResult<Services> createServices() {
  Services services;
  services.platform_lifetime = std::make_unique<PosixPlatformLifetime>();
  services.paths = std::make_unique<LinuxPaths>();
  services.fonts = createLinuxFonts();

  auto config_directory = services.paths->configDirectory();
  if (config_directory) {
    services.settings = makeSettingsStore(
        config_directory.value() / "imgui.ini", detail::createAtomicReplace());
  } else {
    services.settings =
        makeUnavailableSettingsStore(std::move(config_directory).error());
  }
  return services;
}

} // namespace kpt::platform
