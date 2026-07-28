#pragma once

#include "platform/error.hpp"

#include <filesystem>
#include <memory>

namespace kpt::platform::detail {

class AtomicReplace {
public:
  virtual ~AtomicReplace() = default;

  [[nodiscard]] virtual PlatformResult<void>
  replace(const std::filesystem::path &source,
          const std::filesystem::path &destination) = 0;
};

[[nodiscard]] std::unique_ptr<AtomicReplace> createAtomicReplace();

} // namespace kpt::platform::detail
