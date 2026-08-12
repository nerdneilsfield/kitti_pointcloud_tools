#include "kpt/io/pts_codec.hpp"
#include "kpt/cancellation.hpp"
#include "kpt/io/codec_limits.hpp"
#include "kpt/io/ascii_float_parser.hpp"
#include "platform/utf8_path.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <charconv>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace kpt::io_detail {
namespace {

std::string displayPath(const std::filesystem::path &path) {
  auto converted = platform::pathToUtf8(path);
  return converted ? std::move(converted).value() : "<invalid-native-path>";
}

struct PtsRow {
  float vals[7];
  int count;
};

bool parsePtsLine(std::string_view line, PtsRow &out) {
  out.count = 0;
  std::istringstream ss{std::string(line)};
  for (int i = 0; i < 7; ++i) {
    double v;
    if (!(ss >> v))
      break;
    out.vals[i] = static_cast<float>(v);
    ++out.count;
  }
  return out.count >= 3;
}

bool containsNonFiniteNumber(std::string_view line) {
  while (!line.empty()) {
    while (!line.empty() && (line.front() == ' ' || line.front() == '\t'))
      line.remove_prefix(1);
    const auto end = line.find_first_of(" \t\r");
    const auto token = line.substr(0, end);
    double value = 0.0;
    const auto parsed = parseAsciiFloating(token, value);
    if (parsed.ec == std::errc{} && parsed.ptr == token.data() + token.size() &&
        !std::isfinite(value))
      return true;
    if (end == std::string_view::npos)
      break;
    line.remove_prefix(end);
  }
  return false;
}

bool readBoundedLine(std::istream &input, std::string &line,
                     const std::filesystem::path &path) {
  line.clear();
  char value = '\0';
  while (input.get(value)) {
    if (value == '\n')
      return true;
    if (line.size() == kMaxTextLineBytes)
      throw std::runtime_error("PTS parse error: line exceeds 64 KiB: " +
                               displayPath(path));
    line.push_back(value);
  }
  if (input.bad())
    throw std::runtime_error("PTS read error: " + displayPath(path));
  return !line.empty();
}

bool parseCount(std::string_view line, std::size_t &count) {
  while (!line.empty() && (line.back() == ' ' || line.back() == '\t' ||
                           line.back() == '\r'))
    line.remove_suffix(1);
  std::uint64_t value = 0;
  const auto parsed =
      std::from_chars(line.data(), line.data() + line.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != line.data() + line.size() ||
      value > std::numeric_limits<std::size_t>::max())
    return false;
  count = static_cast<std::size_t>(value);
  return true;
}

} // namespace

void loadPts(std::istream &input, const std::filesystem::path &path,
             PointCloudIRGB &cloud, bool &has_color, bool &has_intensity,
             std::stop_token stop) {
  std::string line;
  std::size_t line_number = 0;
  std::size_t skipped = 0;
  bool first_line = true;
  std::optional<std::size_t> declared_count;
  int expected_columns = 0;

  PointCloudIRGB parsed;
  parsed.has_noise = false;

  while (readBoundedLine(input, line, path)) {
    ++line_number;
    if (line_number % 10000 == 0 && stop.stop_requested())
      throw OperationCancelled();

    auto sv = std::string_view(line);
    while (!sv.empty() && (sv.front() == ' ' || sv.front() == '\t' ||
                           sv.front() == '\r'))
      sv.remove_prefix(1);
    if (sv.empty() || sv.front() == '#')
      continue;

    if (first_line) {
      first_line = false;
      std::size_t count = 0;
      if (parseCount(sv, count)) {
        if (count > kMaxPointCount)
          throw std::runtime_error(
              "PTS parse error: point count exceeds limit: " +
              displayPath(path));
        declared_count = count;
        parsed.reserve(count);
        continue;
      }
      if (sv.find_first_of(" \t\r") == std::string_view::npos) {
        double invalid_count = 0.0;
        const auto parsed_count = parseAsciiFloating(sv, invalid_count);
        if (parsed_count.ec == std::errc{} &&
            parsed_count.ptr == sv.data() + sv.size())
          throw std::runtime_error("PTS parse error: invalid declared point "
                                   "count: " +
                                   displayPath(path));
      }
    }

    if (containsNonFiniteNumber(sv))
      throw std::runtime_error("PTS parse error: non-finite value: " +
                               displayPath(path));

    PtsRow row;
    if (!parsePtsLine(sv, row)) {
      if (++skipped <= 50)
        spdlog::warn("skip {}:{}: need >= 3 numeric columns",
                     displayPath(path), line_number);
      continue;
    }

    if (row.count != 3 && row.count != 4 && row.count != 6 && row.count != 7)
      throw std::runtime_error("PTS parse error: unsupported column count: " +
                               displayPath(path));
    if (expected_columns == 0)
      expected_columns = row.count;
    else if (row.count != expected_columns)
      throw std::runtime_error("PTS parse error: inconsistent column count: " +
                               displayPath(path));
    for (int index = 0; index < row.count; ++index) {
      if (!std::isfinite(row.vals[index]))
        throw std::runtime_error("PTS parse error: non-finite value: " +
                                 displayPath(path));
    }

    PointT pt;
    pt.x = row.vals[0];
    pt.y = row.vals[1];
    pt.z = row.vals[2];
    if (row.count == 4 || row.count == 7)
      pt.intensity = row.vals[3];
    if (row.count == 6 || row.count == 7) {
      const int color_offset = row.count == 6 ? 3 : 4;
      pt.r = static_cast<std::uint8_t>(
          std::clamp(row.vals[color_offset], 0.0F, 255.0F));
      pt.g = static_cast<std::uint8_t>(
          std::clamp(row.vals[color_offset + 1], 0.0F, 255.0F));
      pt.b = static_cast<std::uint8_t>(
          std::clamp(row.vals[color_offset + 2], 0.0F, 255.0F));
    }
    if (parsed.size() == kMaxPointCount)
      throw std::runtime_error("PTS parse error: point count exceeds limit: " +
                               displayPath(path));
    parsed.points.push_back(pt);
  }

  if (declared_count && *declared_count != parsed.size())
    throw std::runtime_error("PTS parse error: declared point count mismatch: " +
                             displayPath(path));
  parsed.width = parsed.points.size();
  parsed.height = 1;
  cloud = std::move(parsed);
  has_intensity = expected_columns == 4 || expected_columns == 7;
  has_color = expected_columns == 6 || expected_columns == 7;
  if (skipped > 50)
    spdlog::warn("... {} more skipped lines in {}", skipped - 50,
                 displayPath(path));
}

void savePts(std::ostream &output, const std::filesystem::path &path,
             const PointCloudIRGB &cloud, std::stop_token stop) {
  if (cloud.size() > kMaxPointCount)
    throw std::runtime_error("PTS write error: point count exceeds limit: " +
                             displayPath(path));
  output << cloud.size() << '\n';
  output << std::setprecision(6);
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    if (i % 10000 == 0 && stop.stop_requested())
      throw OperationCancelled();
    const auto &p = cloud.points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(p.z) || !std::isfinite(p.intensity))
      throw std::runtime_error("PTS write error: non-finite point: " +
                               displayPath(path));
    output << p.x << ' ' << p.y << ' ' << p.z;
    output << ' ' << p.intensity;
    output << ' ' << static_cast<int>(p.r) << ' '
           << static_cast<int>(p.g) << ' '
           << static_cast<int>(p.b);
    output << '\n';
  }
  if (!output)
    throw std::runtime_error("PTS write error: data: " + displayPath(path));
}

} // namespace kpt::io_detail
