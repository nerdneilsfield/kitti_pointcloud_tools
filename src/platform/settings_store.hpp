#pragma once

#include "platform/detail/atomic_replace.hpp"
#include "platform/services.hpp"

#include <filesystem>
#include <memory>

namespace kpt::platform {

[[nodiscard]] std::unique_ptr<SettingsStore>
makeSettingsStore(std::filesystem::path ini_file,
                  std::unique_ptr<detail::AtomicReplace> atomic_replace);

[[nodiscard]] std::unique_ptr<SettingsStore>
makeUnavailableSettingsStore(PlatformError error);

} // namespace kpt::platform
