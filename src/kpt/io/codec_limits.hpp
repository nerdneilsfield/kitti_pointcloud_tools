#pragma once

#include <cstddef>
#include <cstdint>

namespace kpt::io_detail {

#ifndef KPT_MAX_POINT_COUNT
#define KPT_MAX_POINT_COUNT 20000000U
#endif

#ifndef KPT_MAX_BODY_BYTES
#define KPT_MAX_BODY_BYTES (512ULL << 20U)
#endif

inline constexpr std::size_t kMaxPointCount = KPT_MAX_POINT_COUNT;
inline constexpr std::size_t kMaxHeaderBytes = 1024U * 1024U;
inline constexpr std::size_t kMaxTextLineBytes = 64U * 1024U;
inline constexpr std::uint64_t kMaxBodyBytes = KPT_MAX_BODY_BYTES;

} // namespace kpt::io_detail
