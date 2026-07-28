#include "platform/detail/atomic_replace.hpp"

#include <utility>

namespace kpt::platform::detail {
namespace {

class PosixAtomicReplace final : public AtomicReplace {
public:
  PlatformResult<void>
  replace(const std::filesystem::path &source,
          const std::filesystem::path &destination) override {
    std::error_code error;
    std::filesystem::rename(source, destination, error);
    if (error) {
      return PlatformError{PlatformErrorCode::SettingsIoFailed,
                           "cannot atomically replace settings file", error};
    }
    return {};
  }
};

} // namespace

std::unique_ptr<AtomicReplace> createAtomicReplace() {
  return std::make_unique<PosixAtomicReplace>();
}

} // namespace kpt::platform::detail
