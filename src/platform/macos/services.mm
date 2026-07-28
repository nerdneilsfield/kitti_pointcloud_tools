#include "platform/services.hpp"

#include "platform/settings_store.hpp"

#import <Foundation/Foundation.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace kpt::platform {
std::unique_ptr<Fonts> createMacFonts();

namespace {

class MacPlatformLifetime final : public PlatformLifetime {};

std::string utf8Description(NSError *error) {
  if (error == nil)
    return {};
  const char *description = error.localizedDescription.UTF8String;
  return description == nullptr ? std::string{} : std::string(description);
}

PlatformError configurationError(std::string message, NSError *error = nil) {
  std::error_code system_error;
  if (error != nil && [error.domain isEqualToString:NSPOSIXErrorDomain] &&
      error.code >= std::numeric_limits<int>::min() &&
      error.code <= std::numeric_limits<int>::max()) {
    system_error = {static_cast<int>(error.code), std::generic_category()};
  } else {
    const auto description = utf8Description(error);
    if (!description.empty())
      message += ": " + description;
  }
  return {PlatformErrorCode::ConfigurationDirectoryUnavailable,
          std::move(message), system_error};
}

PlatformResult<std::filesystem::path> nativePathFromFileUrl(NSURL *url) {
  if (url == nil || !url.fileURL)
    return configurationError("Application Support URL is not file-backed");

  const auto cf_url = reinterpret_cast<CFURLRef>(url);
  CFStringRef path = CFURLCopyFileSystemPath(cf_url, kCFURLPOSIXPathStyle);
  if (path == nullptr)
    return configurationError("cannot inspect Application Support path");

  const CFIndex maximum =
      CFStringGetMaximumSizeOfFileSystemRepresentation(path);
  CFRelease(path);
  if (maximum <= 0)
    return configurationError("Application Support path is empty");

  std::vector<UInt8> bytes(static_cast<std::size_t>(maximum));
  if (!CFURLGetFileSystemRepresentation(cf_url, true, bytes.data(),
                                        static_cast<CFIndex>(bytes.size()))) {
    return configurationError(
        "cannot convert Application Support URL to a native path");
  }
  return std::filesystem::path(reinterpret_cast<const char *>(bytes.data()));
}

class MacPaths final : public Paths {
public:
  PlatformResult<std::filesystem::path> configDirectory() const override {
    @autoreleasepool {
      NSFileManager *manager = NSFileManager.defaultManager;
      NSError *error = nil;
      NSURL *base = [manager URLForDirectory:NSApplicationSupportDirectory
                                    inDomain:NSUserDomainMask
                           appropriateForURL:nil
                                      create:YES
                                       error:&error];
      if (base == nil)
        return configurationError("Application Support is unavailable", error);

      NSURL *directory = [base URLByAppendingPathComponent:@"kpt"
                                               isDirectory:YES];
      error = nil;
      if (![manager createDirectoryAtURL:directory
              withIntermediateDirectories:YES
                               attributes:nil
                                    error:&error]) {
        return configurationError("cannot create Application Support directory",
                                  error);
      }
      return nativePathFromFileUrl(directory);
    }
  }
};

} // namespace

PlatformResult<Services> createServices() {
  @autoreleasepool {
    Services services;
    services.platform_lifetime = std::make_unique<MacPlatformLifetime>();
    services.paths = std::make_unique<MacPaths>();
    services.fonts = createMacFonts();

    auto config_directory = services.paths->configDirectory();
    if (config_directory) {
      services.settings =
          makeSettingsStore(config_directory.value() / "imgui.ini",
                            detail::createAtomicReplace());
    } else {
      services.settings =
          makeUnavailableSettingsStore(std::move(config_directory).error());
    }
    return services;
  }
}

} // namespace kpt::platform
