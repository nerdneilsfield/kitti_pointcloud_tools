#pragma once

#include <cstddef>
#include <cstdint>

namespace kpt::io_detail {

inline constexpr std::size_t kMaxPointCount = 20'000'000U;
inline constexpr std::size_t kMaxHeaderBytes = 1024U * 1024U;
inline constexpr std::size_t kMaxTextLineBytes = 64U * 1024U;
inline constexpr std::uint64_t kMaxBodyBytes = std::uint64_t{512} << 20U;

} // namespace kpt::io_detail
