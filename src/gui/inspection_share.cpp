#include "gui/inspection_share.hpp"

#include "kpt/cancellation.hpp"
#include "kpt/io/ascii_float_parser.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <unordered_set>
#include <utility>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace kpt::gui {
namespace {

constexpr std::uintmax_t kMaxShareBytes = std::uintmax_t{4} << 20U;
constexpr std::string_view kPathSourcePrefix = "path:";

// Review Share v2 is intentionally pinned to the public native enum values.
// Do not serialize renderer-local shader-mode indexes here: those are allowed
// to differ across endpoints while this persisted contract is not.
static_assert(static_cast<int>(ColorBy::Intensity) == 0);
static_assert(static_cast<int>(ColorBy::RGB) == 1);
static_assert(static_cast<int>(ColorBy::Z) == 2);
static_assert(static_cast<int>(ColorBy::Label) == 3);
static_assert(static_cast<int>(ColorBy::None) == 4);
static_assert(static_cast<int>(ColorMap::Turbo) == 0);
static_assert(static_cast<int>(ColorMap::Autumn) == 9);

void setError(std::string *error, std::string message) {
  if (error != nullptr) {
    *error = std::move(message);
  }
}

[[nodiscard]] bool validCamera(const CameraSnapshot &camera) noexcept {
  ViewportModel model;
  return model.setCameraSnapshot(camera);
}

[[nodiscard]] bool validAffine(const Eigen::Affine3d &transform) noexcept {
  const Eigen::Vector4d expected_bottom_row{0.0, 0.0, 0.0, 1.0};
  return transform.matrix().allFinite() &&
         transform.matrix().row(3).isApprox(expected_bottom_row.transpose());
}

[[nodiscard]] bool finite(const Eigen::Vector3d &value) noexcept {
  return value.allFinite();
}

[[nodiscard]] bool isPathSourceKey(std::string_view source_key) noexcept {
  return source_key.size() >= kPathSourcePrefix.size() &&
         source_key.substr(0, kPathSourcePrefix.size()) == kPathSourcePrefix;
}

[[nodiscard]] bool validRelativeSourcePath(
    const std::filesystem::path &path) {
  if (path.empty() || path.is_absolute() || path.has_root_name() ||
      path.has_root_directory()) {
    return false;
  }
  // A share file may only name sources beneath its own directory. Checking
  // components rather than only lexically_normal() also rejects a leading
  // "..", which otherwise remains normalized and could escape that root.
  for (const auto &component : path) {
    if (component == "..") {
      return false;
    }
  }
  const auto normalized = path.lexically_normal();
  const auto raw = path.generic_string();
  return raw != "." && !raw.empty() && normalized.generic_string() == raw;
}

[[nodiscard]] std::filesystem::path absoluteSharePath(
    const std::filesystem::path &path) {
  if (path.empty()) {
    throw std::invalid_argument("share file path must not be empty");
  }
  std::error_code error;
  const auto absolute = std::filesystem::absolute(path, error);
  if (error) {
    throw std::system_error(error, "cannot resolve share file path");
  }
  return absolute.lexically_normal();
}

[[nodiscard]] std::optional<std::filesystem::path> relativePathForSource(
    std::string_view source_key, const std::filesystem::path &share_file) {
  if (!isPathSourceKey(source_key)) {
    return std::nullopt;
  }
  const std::filesystem::path source{
      std::string{source_key.substr(kPathSourcePrefix.size())}};
  if (!source.is_absolute()) {
    // A foreign platform's path: key is still a valid logical identity, but
    // never becomes a local location merely because this process re-exports.
    return std::nullopt;
  }
  const auto relative =
      source.lexically_relative(absoluteSharePath(share_file).parent_path());
  if (!validRelativeSourcePath(relative)) {
    return std::nullopt;
  }
  return relative;
}

template <typename Number>
void appendNumber(std::string &output, Number value) {
  if constexpr (std::is_integral_v<Number>) {
    char buffer[64];
    const auto [end, error] =
        std::to_chars(std::begin(buffer), std::end(buffer), value);
    if (error != std::errc{}) {
      throw std::runtime_error("cannot serialize share number");
    }
    output.append(buffer, end);
  } else if (!io_detail::appendAsciiFloating(output, value)) {
    throw std::runtime_error("cannot serialize share number");
  }
}

[[nodiscard]] std::string escape(std::string_view value) {
  std::string result;
  result.reserve(value.size());
  for (const char character : value) {
    switch (character) {
    case '"': result += "\\\""; break;
    case '\\': result += "\\\\"; break;
    case '\b': result += "\\b"; break;
    case '\f': result += "\\f"; break;
    case '\n': result += "\\n"; break;
    case '\r': result += "\\r"; break;
    case '\t': result += "\\t"; break;
    default:
      if (static_cast<unsigned char>(character) < 0x20U) {
        constexpr char digits[] = "0123456789abcdef";
        result += "\\u00";
        result += digits[(static_cast<unsigned char>(character) >> 4U) & 0x0fU];
        result += digits[static_cast<unsigned char>(character) & 0x0fU];
      } else {
        result += character;
      }
    }
  }
  return result;
}

void appendVector(std::string &output, const Eigen::Vector3d &value) {
  output += '[';
  for (int index = 0; index < 3; ++index) {
    if (index != 0) output += ',';
    appendNumber(output, value[index]);
  }
  output += ']';
}

void appendVector(std::string &output, const Eigen::Vector3f &value) {
  output += '[';
  for (int index = 0; index < 3; ++index) {
    if (index != 0) output += ',';
    appendNumber(output, value[index]);
  }
  output += ']';
}

void appendMatrix(std::string &output, const Eigen::Matrix4d &value) {
  output += '[';
  for (int row = 0; row < 4; ++row) {
    if (row != 0) output += ',';
    output += '[';
    for (int column = 0; column < 4; ++column) {
      if (column != 0) output += ',';
      appendNumber(output, value(row, column));
    }
    output += ']';
  }
  output += ']';
}

void appendCamera(std::string &output, const CameraSnapshot &camera) {
  output += "{\"target\":";
  appendVector(output, camera.target);
  output += ",\"rotation_center\":";
  appendVector(output, camera.rotation_center);
  output += ",\"camera_to_world\":[";
  for (int row = 0; row < 3; ++row) {
    if (row != 0) output += ',';
    output += '[';
    for (int column = 0; column < 3; ++column) {
      if (column != 0) output += ',';
      appendNumber(output, camera.camera_to_world(row, column));
    }
    output += ']';
  }
  output += "],\"distance\":";
  appendNumber(output, camera.distance);
  output += ",\"fov_y_degrees\":";
  appendNumber(output, camera.fov_y_degrees);
  output += '}';
}

void appendStyle(std::string &output, const LayerStyle &style) {
  output += "{\"color_by\":";
  appendNumber(output, static_cast<int>(style.color_by));
  output += ",\"color_map\":";
  appendNumber(output, static_cast<int>(style.color_map));
  output += ",\"point_size\":";
  appendNumber(output, style.point_size);
  output += ",\"opacity\":";
  appendNumber(output, style.opacity);
  output += ",\"scalar_min\":";
  appendNumber(output, style.scalar_min);
  output += ",\"scalar_max\":";
  appendNumber(output, style.scalar_max);
  output += ",\"fixed_color\":";
  appendVector(output, style.fixed_color);
  output += ",\"noise_color\":";
  appendVector(output, style.noise_color);
  output += ",\"highlight_noise\":";
  output += style.highlight_noise ? "true" : "false";
  output += ",\"intensity_equalize\":";
  output += style.intensity_equalize ? "true" : "false";
  output += '}';
}

void validateDocument(const InspectionShareDocument &document) {
  std::unordered_set<std::string> layer_keys;
  layer_keys.reserve(document.layers.size());
  for (const InspectionShareLayer &layer : document.layers) {
    if (!isCanonicalSourceKey(layer.source_key) ||
        !layer_keys.insert(layer.source_key).second ||
        !validAffine(layer.local_to_world) || !isValidLayerStyle(layer.style)) {
      throw std::invalid_argument("share layer is invalid");
    }
    if (layer.relative_source_path.has_value() &&
        !validRelativeSourcePath(*layer.relative_source_path)) {
      throw std::invalid_argument("share source path must be normalized relative");
    }
  }
  if (document.roi.has_value() &&
      (!finite(document.roi->minimum()) || !finite(document.roi->maximum()))) {
    throw std::invalid_argument("share ROI is invalid");
  }
  for (const InspectionShareMeasurement &measurement : document.measurements) {
    const bool has_second_source = measurement.second_source_key.has_value();
    const bool has_second_world = measurement.second_world.has_value();
    if (!isCanonicalSourceKey(measurement.first_source_key) ||
        !finite(measurement.first_world) ||
        has_second_source != has_second_world ||
        (has_second_source &&
         (!isCanonicalSourceKey(*measurement.second_source_key) ||
          !finite(*measurement.second_world)))) {
      throw std::invalid_argument("share measurement is invalid");
    }
  }
  std::unordered_set<std::string> bookmark_names;
  bookmark_names.reserve(document.bookmarks.size());
  for (const CameraBookmark &bookmark : document.bookmarks) {
    if (bookmark.name().empty() || !bookmark_names.insert(bookmark.name()).second ||
        !validCamera(bookmark.camera())) {
      throw std::invalid_argument("share bookmark is invalid");
    }
  }
}

[[nodiscard]] std::string serialize(const InspectionShareDocument &document) {
  validateDocument(document);
  std::string output = "{\"schema_version\":";
  appendNumber(output, InspectionShareFile::kSchemaVersion);
  output += ",\"layers\":[";
  bool first = true;
  for (const InspectionShareLayer &layer : document.layers) {
    if (!first) output += ',';
    first = false;
    output += "{\"source_key\":\"" + escape(layer.source_key) +
              "\",\"source_path\":";
    if (layer.relative_source_path.has_value()) {
      output += "\"" + escape(layer.relative_source_path->generic_string()) +
                "\"";
    } else {
      output += "null";
    }
    output += ",\"local_to_world\":";
    appendMatrix(output, layer.local_to_world.matrix());
    output += ",\"style\":";
    appendStyle(output, layer.style);
    output += ",\"visible\":";
    output += layer.visible ? "true" : "false";
    output += '}';
  }
  output += "],\"roi\":";
  if (document.roi.has_value()) {
    output += "{\"minimum\":";
    appendVector(output, document.roi->minimum());
    output += ",\"maximum\":";
    appendVector(output, document.roi->maximum());
    output += '}';
  } else {
    output += "null";
  }
  output += ",\"measurements\":[";
  first = true;
  for (const InspectionShareMeasurement &measurement : document.measurements) {
    if (!first) output += ',';
    first = false;
    output += "{\"first_source_key\":\"" +
              escape(measurement.first_source_key) + "\",\"first_world\":";
    appendVector(output, measurement.first_world);
    output += ",\"second_source_key\":";
    if (measurement.second_source_key.has_value()) {
      output += "\"" + escape(*measurement.second_source_key) + "\"";
    } else {
      output += "null";
    }
    output += ",\"second_world\":";
    if (measurement.second_world.has_value()) {
      appendVector(output, *measurement.second_world);
    } else {
      output += "null";
    }
    output += '}';
  }
  output += "],\"bookmarks\":[";
  first = true;
  for (const CameraBookmark &bookmark : document.bookmarks) {
    if (!first) output += ',';
    first = false;
    output += "{\"name\":\"" + escape(bookmark.name()) + "\",\"camera\":";
    appendCamera(output, bookmark.camera());
    output += '}';
  }
  output += "]}\n";
  return output;
}

class Parser {
public:
  explicit Parser(std::string_view input) : input_(input) {}

  [[nodiscard]] InspectionShareDocument parse() {
    expect('{');
    expectKey("schema_version");
    const int schema_version = integer();
    if (schema_version == 1) {
      fail("inspection share schema v1 is unsupported; re-export as v2");
    }
    if (schema_version != InspectionShareFile::kSchemaVersion) {
      fail("unsupported inspection share schema version");
    }
    expect(',');
    expectKey("layers");
    const auto layers = layerArray();
    expect(',');
    expectKey("roi");
    const auto roi = roiValue();
    expect(',');
    expectKey("measurements");
    const auto measurements = measurementArray();
    expect(',');
    expectKey("bookmarks");
    const auto bookmarks = bookmarkArray();
    expect('}');
    whitespace();
    if (position_ != input_.size()) fail("trailing JSON data");
    InspectionShareDocument result{layers, roi, measurements, bookmarks};
    validateDocument(result);
    return result;
  }

private:
  [[noreturn]] void fail(std::string_view message) const {
    throw std::invalid_argument(std::string(message));
  }

  void whitespace() {
    while (position_ < input_.size() &&
           (input_[position_] == ' ' || input_[position_] == '\n' ||
            input_[position_] == '\r' || input_[position_] == '\t')) {
      ++position_;
    }
  }

  void expect(char value) {
    whitespace();
    if (position_ == input_.size() || input_[position_] != value) {
      fail("invalid inspection share JSON");
    }
    ++position_;
  }

  [[nodiscard]] bool consume(char value) {
    whitespace();
    if (position_ == input_.size() || input_[position_] != value) {
      return false;
    }
    ++position_;
    return true;
  }

  [[nodiscard]] bool null() {
    whitespace();
    if (input_.substr(position_, 4) != "null") {
      return false;
    }
    position_ += 4;
    return true;
  }

  void expectKey(std::string_view key) {
    if (string() != key) fail("unexpected inspection share field");
    expect(':');
  }

  [[nodiscard]] std::string string() {
    whitespace();
    if (position_ == input_.size() || input_[position_++] != '"') {
      fail("expected JSON string");
    }
    std::string result;
    while (position_ < input_.size()) {
      const char value = input_[position_++];
      if (value == '"') return result;
      if (static_cast<unsigned char>(value) < 0x20U) {
        fail("control character in JSON string");
      }
      if (value != '\\') {
        result += value;
        continue;
      }
      if (position_ == input_.size()) fail("incomplete JSON escape");
      const char escaped = input_[position_++];
      switch (escaped) {
      case '"': result += '"'; break;
      case '\\': result += '\\'; break;
      case '/': result += '/'; break;
      case 'b': result += '\b'; break;
      case 'f': result += '\f'; break;
      case 'n': result += '\n'; break;
      case 'r': result += '\r'; break;
      case 't': result += '\t'; break;
      case 'u': {
        const auto hex = [this](char digit) -> unsigned int {
          if (digit >= '0' && digit <= '9') {
            return static_cast<unsigned int>(digit - '0');
          }
          if (digit >= 'a' && digit <= 'f') {
            return static_cast<unsigned int>(digit - 'a' + 10);
          }
          if (digit >= 'A' && digit <= 'F') {
            return static_cast<unsigned int>(digit - 'A' + 10);
          }
          fail("invalid JSON Unicode escape");
        };
        if (input_.size() - position_ < 4U) fail("incomplete JSON Unicode escape");
        unsigned int codepoint = 0;
        for (int index = 0; index < 4; ++index) {
          codepoint = (codepoint << 4U) | hex(input_[position_++]);
        }
        if (codepoint >= 0xd800U && codepoint <= 0xdfffU) {
          fail("JSON surrogate escapes are unsupported");
        }
        if (codepoint <= 0x7fU) {
          result += static_cast<char>(codepoint);
        } else if (codepoint <= 0x7ffU) {
          result += static_cast<char>(0xc0U | (codepoint >> 6U));
          result += static_cast<char>(0x80U | (codepoint & 0x3fU));
        } else {
          result += static_cast<char>(0xe0U | (codepoint >> 12U));
          result += static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3fU));
          result += static_cast<char>(0x80U | (codepoint & 0x3fU));
        }
        break;
      }
      default: fail("unsupported JSON escape");
      }
    }
    fail("unterminated JSON string");
  }

  [[nodiscard]] double number() {
    whitespace();
    double value = 0.0;
    const char *begin = input_.data() + position_;
    const auto [parsed, error] =
        io_detail::parseJsonFloatingPrefix(input_.substr(position_), value);
    if (error != std::errc{} || parsed == begin || !std::isfinite(value)) {
      fail("expected finite JSON number");
    }
    position_ = static_cast<std::size_t>(parsed - input_.data());
    return value;
  }

  [[nodiscard]] float floatNumber() {
    const double value = number();
    if (std::abs(value) >
        static_cast<double>((std::numeric_limits<float>::max)())) {
      fail("JSON number does not fit float");
    }
    return static_cast<float>(value);
  }

  [[nodiscard]] int integer() {
    whitespace();
    const char *begin = input_.data() + position_;
    const char *end = input_.data() + input_.size();
    int value = 0;
    const auto [parsed, error] = std::from_chars(begin, end, value);
    if (error != std::errc{} || parsed == begin) {
      fail("expected integer");
    }
    position_ = static_cast<std::size_t>(parsed - input_.data());
    return value;
  }

  [[nodiscard]] bool boolean() {
    whitespace();
    if (input_.substr(position_, 4) == "true") {
      position_ += 4;
      return true;
    }
    if (input_.substr(position_, 5) == "false") {
      position_ += 5;
      return false;
    }
    fail("expected JSON boolean");
  }

  [[nodiscard]] Eigen::Vector3d vector3() {
    expect('[');
    Eigen::Vector3d value;
    for (int index = 0; index < 3; ++index) {
      if (index != 0) expect(',');
      value[index] = number();
    }
    expect(']');
    return value;
  }

  [[nodiscard]] Eigen::Vector3f vector3f() {
    expect('[');
    Eigen::Vector3f value;
    for (int index = 0; index < 3; ++index) {
      if (index != 0) expect(',');
      value[index] = floatNumber();
    }
    expect(']');
    return value;
  }

  [[nodiscard]] Eigen::Affine3d affine() {
    expect('[');
    Eigen::Matrix4d matrix;
    for (int row = 0; row < 4; ++row) {
      if (row != 0) expect(',');
      expect('[');
      for (int column = 0; column < 4; ++column) {
        if (column != 0) expect(',');
        matrix(row, column) = number();
      }
      expect(']');
    }
    expect(']');
    return Eigen::Affine3d{matrix};
  }

  [[nodiscard]] CameraSnapshot camera() {
    expect('{');
    CameraSnapshot result;
    expectKey("target");
    result.target = vector3();
    expect(',');
    expectKey("rotation_center");
    result.rotation_center = vector3();
    expect(',');
    expectKey("camera_to_world");
    expect('[');
    for (int row = 0; row < 3; ++row) {
      if (row != 0) expect(',');
      expect('[');
      for (int column = 0; column < 3; ++column) {
        if (column != 0) expect(',');
        result.camera_to_world(row, column) = floatNumber();
      }
      expect(']');
    }
    expect(']');
    expect(',');
    expectKey("distance");
    result.distance = number();
    expect(',');
    expectKey("fov_y_degrees");
    result.fov_y_degrees = floatNumber();
    expect('}');
    if (!validCamera(result)) fail("share bookmark camera is invalid");
    return result;
  }

  [[nodiscard]] LayerStyle style() {
    expect('{');
    LayerStyle result;
    expectKey("color_by");
    const int color_by = integer();
    expect(',');
    expectKey("color_map");
    const int color_map = integer();
    if (color_by < static_cast<int>(ColorBy::Intensity) ||
        color_by > static_cast<int>(ColorBy::None) || color_map < 0 ||
        color_map > static_cast<int>(ColorMap::Autumn)) {
      fail("share layer colour enum is invalid");
    }
    result.color_by = static_cast<ColorBy>(color_by);
    result.color_map = static_cast<ColorMap>(color_map);
    expect(',');
    expectKey("point_size");
    result.point_size = floatNumber();
    expect(',');
    expectKey("opacity");
    result.opacity = floatNumber();
    expect(',');
    expectKey("scalar_min");
    result.scalar_min = floatNumber();
    expect(',');
    expectKey("scalar_max");
    result.scalar_max = floatNumber();
    expect(',');
    expectKey("fixed_color");
    result.fixed_color = vector3f();
    expect(',');
    expectKey("noise_color");
    result.noise_color = vector3f();
    expect(',');
    expectKey("highlight_noise");
    result.highlight_noise = boolean();
    expect(',');
    expectKey("intensity_equalize");
    result.intensity_equalize = boolean();
    expect('}');
    if (!isValidLayerStyle(result)) fail("share layer style is invalid");
    return result;
  }

  [[nodiscard]] std::optional<std::filesystem::path> optionalPath() {
    if (null()) return std::nullopt;
    const std::filesystem::path value{string()};
    if (!validRelativeSourcePath(value)) {
      fail("share source path must be normalized relative");
    }
    return value;
  }

  [[nodiscard]] InspectionShareLayer layer() {
    expect('{');
    InspectionShareLayer result;
    expectKey("source_key");
    result.source_key = string();
    expect(',');
    expectKey("source_path");
    result.relative_source_path = optionalPath();
    expect(',');
    expectKey("local_to_world");
    result.local_to_world = affine();
    expect(',');
    expectKey("style");
    result.style = style();
    expect(',');
    expectKey("visible");
    result.visible = boolean();
    expect('}');
    return result;
  }

  [[nodiscard]] std::vector<InspectionShareLayer> layerArray() {
    expect('[');
    std::vector<InspectionShareLayer> result;
    if (!consume(']')) {
      do {
        result.push_back(layer());
      } while (consume(','));
      expect(']');
    }
    return result;
  }

  [[nodiscard]] std::optional<RoiBox> roiValue() {
    if (null()) return std::nullopt;
    expect('{');
    expectKey("minimum");
    const auto minimum = vector3();
    expect(',');
    expectKey("maximum");
    const auto maximum = vector3();
    expect('}');
    return RoiBox{minimum, maximum};
  }

  [[nodiscard]] InspectionShareMeasurement measurement() {
    expect('{');
    InspectionShareMeasurement result;
    expectKey("first_source_key");
    result.first_source_key = string();
    expect(',');
    expectKey("first_world");
    result.first_world = vector3();
    expect(',');
    expectKey("second_source_key");
    if (!null()) {
      result.second_source_key = string();
    }
    expect(',');
    expectKey("second_world");
    if (!null()) {
      result.second_world = vector3();
    }
    expect('}');
    return result;
  }

  [[nodiscard]] std::vector<InspectionShareMeasurement> measurementArray() {
    expect('[');
    std::vector<InspectionShareMeasurement> result;
    if (!consume(']')) {
      do {
        result.push_back(measurement());
      } while (consume(','));
      expect(']');
    }
    return result;
  }

  [[nodiscard]] CameraBookmark bookmark() {
    expect('{');
    expectKey("name");
    const std::string name = string();
    expect(',');
    expectKey("camera");
    const CameraSnapshot value = camera();
    expect('}');
    return CameraBookmark{name, value};
  }

  [[nodiscard]] std::vector<CameraBookmark> bookmarkArray() {
    expect('[');
    std::vector<CameraBookmark> result;
    if (!consume(']')) {
      do {
        result.push_back(bookmark());
      } while (consume(','));
      expect(']');
    }
    return result;
  }

  std::string_view input_;
  std::size_t position_ = 0;
};

[[nodiscard]] std::optional<std::string> readFile(
    const std::filesystem::path &path, std::string *error) {
  std::error_code status_error;
  if (!std::filesystem::exists(path, status_error)) {
    setError(error, status_error ? status_error.message() : "share file does not exist");
    return std::nullopt;
  }
  const auto size = std::filesystem::file_size(path, status_error);
  if (status_error || size > kMaxShareBytes) {
    setError(error, status_error ? status_error.message()
                                 : "inspection share file exceeds 4 MiB");
    return std::nullopt;
  }
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    setError(error, "cannot open inspection share file");
    return std::nullopt;
  }
  std::string content(static_cast<std::size_t>(size), '\0');
  file.read(content.data(), static_cast<std::streamsize>(content.size()));
  if (!file && !content.empty()) {
    setError(error, "cannot read inspection share file");
    return std::nullopt;
  }
  return content;
}

[[nodiscard]] std::string temporarySuffix() {
  static std::atomic<std::uint64_t> sequence{0};
  std::random_device random;
  std::ostringstream encoded;
  encoded << std::hex << random() << random() << random() << '.'
          << sequence.fetch_add(1, std::memory_order_relaxed);
  return encoded.str();
}

struct ExclusiveWriteResult {
  std::error_code error;
  bool cancelled = false;
};

[[nodiscard]] ExclusiveWriteResult
writeExclusive(const std::filesystem::path &path, std::string_view contents,
               std::stop_token stop) {
  if (stop.stop_requested()) {
    return {{}, true};
  }
#if defined(_WIN32)
  const HANDLE handle = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr,
                                     CREATE_NEW,
                                     FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH,
                                     nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    return {{static_cast<int>(GetLastError()), std::system_category()}, false};
  }
  std::size_t offset = 0;
  bool written = true;
  bool cancelled = false;
  while (offset < contents.size()) {
    if (stop.stop_requested()) {
      cancelled = true;
      break;
    }
    const DWORD request = static_cast<DWORD>(std::min<std::size_t>(
        contents.size() - offset, static_cast<std::size_t>(MAXDWORD)));
    DWORD count = 0;
    if (WriteFile(handle, contents.data() + offset, request, &count, nullptr) ==
            FALSE ||
        count == 0) {
      written = false;
      break;
    }
    offset += count;
  }
  if (written && !cancelled && stop.stop_requested()) {
    cancelled = true;
  }
  if (written && !cancelled) written = FlushFileBuffers(handle) != FALSE;
  const DWORD error = written ? ERROR_SUCCESS : GetLastError();
  CloseHandle(handle);
  if (!written || cancelled) {
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    if (cancelled) {
      return {{}, true};
    }
    return {{static_cast<int>(error), std::system_category()}, false};
  }
  return {};
#else
  const int descriptor =
      ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (descriptor < 0) return {{errno, std::generic_category()}, false};
  std::size_t offset = 0;
  int write_error = 0;
  bool cancelled = false;
  while (offset < contents.size()) {
    if (stop.stop_requested()) {
      cancelled = true;
      break;
    }
    const ssize_t count =
        ::write(descriptor, contents.data() + offset, contents.size() - offset);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) {
      write_error = errno;
      break;
    }
    offset += static_cast<std::size_t>(count);
  }
  if (write_error == 0 && !cancelled && stop.stop_requested()) {
    cancelled = true;
  }
  if (write_error == 0 && !cancelled && ::fsync(descriptor) != 0) {
    write_error = errno;
  }
  if (::close(descriptor) != 0 && write_error == 0) write_error = errno;
  if (write_error != 0 || cancelled) {
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    if (cancelled) {
      return {{}, true};
    }
    return {{write_error, std::generic_category()}, false};
  }
  return {};
#endif
}

[[nodiscard]] bool isAlreadyExists(const std::error_code &error) {
  if (error == std::errc::file_exists) return true;
#if defined(_WIN32)
  return error.value() == ERROR_FILE_EXISTS || error.value() == ERROR_ALREADY_EXISTS;
#else
  return false;
#endif
}

[[nodiscard]] std::error_code publishFile(const std::filesystem::path &source,
                                          const std::filesystem::path &destination,
                                          bool overwrite) {
#if defined(_WIN32)
  DWORD flags = MOVEFILE_WRITE_THROUGH;
  if (overwrite) {
    flags |= MOVEFILE_REPLACE_EXISTING;
  }
  if (MoveFileExW(source.c_str(), destination.c_str(),
                  flags) == FALSE) {
    return {static_cast<int>(GetLastError()), std::system_category()};
  }
  return {};
#else
  if (!overwrite) {
    if (::link(source.c_str(), destination.c_str()) != 0) {
      return {errno, std::generic_category()};
    }
    if (::unlink(source.c_str()) != 0) {
      // link() already made the destination visible atomically. Reporting a
      // failure here would claim the save failed despite a valid published
      // document; leave an orphaned temporary for later cleanup instead.
    }
    return {};
  }
  std::error_code error;
  std::filesystem::rename(source, destination, error);
  return error;
#endif
}

} // namespace

InspectionShareFile::InspectionShareFile(std::filesystem::path file)
    : file_(std::move(file)) {}

const std::filesystem::path &InspectionShareFile::path() const noexcept {
  return file_;
}

bool InspectionShareFile::load(InspectionShareDocument &document,
                               std::string *error) const {
  const auto content = readFile(file_, error);
  if (!content.has_value()) return false;
  try {
    InspectionShareDocument parsed = Parser(*content).parse();
    document = std::move(parsed);
    return true;
  } catch (const std::exception &exception) {
    setError(error, exception.what());
    return false;
  }
}

InspectionShareSaveResult
InspectionShareFile::save(const InspectionShareDocument &document,
                          bool overwrite, std::stop_token stop) const {
  std::filesystem::path temporary;
  try {
    if (stop.stop_requested()) {
      return {InspectionShareSaveStatus::Cancelled, "operation cancelled"};
    }
    const std::string content = serialize(document);
    if (stop.stop_requested()) {
      return {InspectionShareSaveStatus::Cancelled, "operation cancelled"};
    }
    if (file_.empty()) {
      throw std::invalid_argument("share file path must not be empty");
    }
    std::error_code filesystem_error;
    if (!file_.parent_path().empty()) {
      std::filesystem::create_directories(file_.parent_path(), filesystem_error);
      if (filesystem_error) {
        return {InspectionShareSaveStatus::Failed, filesystem_error.message()};
      }
    }
    if (stop.stop_requested()) {
      return {InspectionShareSaveStatus::Cancelled, "operation cancelled"};
    }
    ExclusiveWriteResult write_result;
    bool created = false;
    for (int attempt = 0; attempt < 32; ++attempt) {
      temporary = file_;
      temporary += ".tmp." + temporarySuffix();
      write_result = writeExclusive(temporary, content, stop);
      if (write_result.cancelled) {
        return {InspectionShareSaveStatus::Cancelled, "operation cancelled"};
      }
      if (!write_result.error) {
        created = true;
        break;
      }
      if (!isAlreadyExists(write_result.error)) break;
    }
    if (!created) {
      return {InspectionShareSaveStatus::Failed,
              "cannot create exclusive temporary inspection share file: " +
                  write_result.error.message()};
    }
    if (stop.stop_requested()) {
      std::filesystem::remove(temporary, filesystem_error);
      return {InspectionShareSaveStatus::Cancelled, "operation cancelled"};
    }
    const std::error_code publish_error =
        publishFile(temporary, file_, overwrite);
    if (publish_error) {
      std::filesystem::remove(temporary, filesystem_error);
      if (!overwrite && isAlreadyExists(publish_error)) {
        return {InspectionShareSaveStatus::Skipped,
                "destination already exists"};
      }
      return {InspectionShareSaveStatus::Failed,
              "cannot atomically publish inspection share file: " +
                  publish_error.message()};
    }
    return {InspectionShareSaveStatus::Written, {}};
  } catch (const std::exception &exception) {
    std::error_code ignored;
    if (!temporary.empty()) {
      std::filesystem::remove(temporary, ignored);
    }
    return {InspectionShareSaveStatus::Failed, exception.what()};
  }
}

InspectionShareDocument InspectionShareFile::capture(
    const Scene &scene, const InspectionSettings &settings,
    const std::filesystem::path &share_file) {
  InspectionShareDocument document;
  document.layers.reserve(scene.layers().size());
  for (const CloudLayer &layer : scene.layers()) {
    document.layers.push_back(
        {layer.sourceKey(), relativePathForSource(layer.sourceKey(), share_file),
         layer.localToWorld(), layer.style(), layer.visible()});
  }
  document.roi = scene.roi();
  document.measurements.reserve(scene.measurements().size());
  for (const Measurement &measurement : scene.measurements()) {
    document.measurements.push_back(
        {measurement.firstSourceKey(), measurement.firstWorld(),
         measurement.secondSourceKey(), measurement.secondWorld()});
  }
  document.bookmarks = settings.bookmarks();
  validateDocument(document);
  return document;
}

std::optional<std::filesystem::path> InspectionShareFile::resolveSourcePath(
    const std::filesystem::path &share_file,
    const InspectionShareLayer &layer) {
  try {
    if (!layer.relative_source_path.has_value() ||
        !isCanonicalSourceKey(layer.source_key) ||
        !validRelativeSourcePath(*layer.relative_source_path)) {
      return std::nullopt;
    }
    return (absoluteSharePath(share_file).parent_path() /
            *layer.relative_source_path)
        .lexically_normal();
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

} // namespace kpt::gui
