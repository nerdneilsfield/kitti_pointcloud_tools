#pragma once

#include <stdexcept>

namespace kpt {

class OperationCancelled final : public std::runtime_error {
public:
  OperationCancelled() : std::runtime_error("operation cancelled") {}
};

} // namespace kpt
