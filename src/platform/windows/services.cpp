#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include "platform/services.hpp"

#include "platform/settings_store.hpp"

#include <objbase.h>
#include <shlobj.h>
#include <windows.h>

#include <cassert>
#include <memory>
#include <utility>

namespace kpt::platform {
std::unique_ptr<Fonts> createWindowsFonts();

namespace {

struct CoTaskMemoryDeleter {
  void operator()(wchar_t *value) const noexcept {
    if (value != nullptr)
      CoTaskMemFree(value);
  }
};

std::error_code hresultError(HRESULT result) {
  return {static_cast<int>(result), std::system_category()};
}

PlatformError initializationError(HRESULT result) {
  return {PlatformErrorCode::PlatformInitializationFailed,
          "cannot initialize the Windows COM apartment", hresultError(result)};
}

PlatformError configurationError(std::string message, HRESULT result) {
  return {PlatformErrorCode::ConfigurationDirectoryUnavailable,
          std::move(message), hresultError(result)};
}

PlatformError configurationError(std::string message,
                                 std::error_code system_error) {
  return {PlatformErrorCode::ConfigurationDirectoryUnavailable,
          std::move(message), system_error};
}

class WindowsPlatformLifetime final : public PlatformLifetime {
public:
  WindowsPlatformLifetime(bool owns_apartment, DWORD thread_id)
      : owns_apartment_(owns_apartment), thread_id_(thread_id) {}

  ~WindowsPlatformLifetime() override {
    if (!owns_apartment_)
      return;
    assert(GetCurrentThreadId() == thread_id_);
    if (GetCurrentThreadId() == thread_id_)
      CoUninitialize();
  }

private:
  bool owns_apartment_;
  DWORD thread_id_;
};

PlatformResult<std::unique_ptr<PlatformLifetime>> createPlatformLifetime() {
  const HRESULT result = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
  if (result == S_OK || result == S_FALSE) {
    std::unique_ptr<PlatformLifetime> lifetime =
        std::make_unique<WindowsPlatformLifetime>(true, GetCurrentThreadId());
    return lifetime;
  }
  if (result == RPC_E_CHANGED_MODE) {
    std::unique_ptr<PlatformLifetime> lifetime =
        std::make_unique<WindowsPlatformLifetime>(false, GetCurrentThreadId());
    return lifetime;
  }
  return initializationError(result);
}

class WindowsPaths final : public Paths {
public:
  PlatformResult<std::filesystem::path> configDirectory() const override {
    PWSTR allocated_path = nullptr;
    const HRESULT result = SHGetKnownFolderPath(
        FOLDERID_RoamingAppData, KF_FLAG_DEFAULT, nullptr, &allocated_path);
    // The API may assign output storage even when returning failure.
    std::unique_ptr<wchar_t, CoTaskMemoryDeleter> owned_path(allocated_path);
    if (FAILED(result))
      return configurationError("Roaming AppData is unavailable", result);

    const std::filesystem::path base(owned_path.get());
    const auto directory = base / L"kpt";

    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error)
      return configurationError("cannot create configuration directory", error);
    if (!std::filesystem::is_directory(directory, error) || error)
      return configurationError("configuration path is not a directory", error);
    return directory;
  }
};

} // namespace

PlatformResult<Services> createServices() {
  Services services;
  auto lifetime = createPlatformLifetime();
  if (!lifetime)
    return std::move(lifetime).error();
  services.platform_lifetime = std::move(lifetime).value();
  services.paths = std::make_unique<WindowsPaths>();
  services.fonts = createWindowsFonts();

  auto config_directory = services.paths->configDirectory();
  if (config_directory) {
    services.settings = makeSettingsStore(
        config_directory.value() / L"imgui.ini", detail::createAtomicReplace());
  } else {
    services.settings =
        makeUnavailableSettingsStore(std::move(config_directory).error());
  }
  return services;
}

} // namespace kpt::platform
