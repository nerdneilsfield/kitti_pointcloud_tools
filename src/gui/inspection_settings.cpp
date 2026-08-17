#include "gui/inspection_settings.hpp"

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
#include <fstream>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <string_view>
#include <system_error>
#include <type_traits>

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

constexpr std::uintmax_t kMaxSettingsBytes = std::uintmax_t{1} << 20U;

void setError(std::string *error, std::string message) {
  if (error != nullptr) {
    *error = std::move(message);
  }
}

[[nodiscard]] bool validSnapshot(const CameraSnapshot &camera) noexcept {
  // Keep persisted-camera acceptance identical to ViewportModel. In
  // particular, finite double values can still overflow renderer float space.
  ViewportModel model;
  return model.setCameraSnapshot(camera);
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

template <typename Number>
void appendNumber(std::string &output, Number value) {
  if constexpr (std::is_integral_v<Number>) {
    char buffer[64];
    const auto [end, error] =
        std::to_chars(std::begin(buffer), std::end(buffer), value);
    if (error != std::errc{}) {
      throw std::runtime_error("cannot serialize inspection camera number");
    }
    output.append(buffer, end);
  } else if (!io_detail::appendAsciiFloating(output, value)) {
    throw std::runtime_error("cannot serialize inspection camera number");
  }
}

void appendVector(std::string &output, const Eigen::Vector3d &value) {
  output += '[';
  for (int index = 0; index < 3; ++index) {
    if (index != 0) output += ',';
    appendNumber(output, value[index]);
  }
  output += ']';
}

[[nodiscard]] std::string serialize(const InspectionSettings &settings) {
  std::string output = "{\"schema_version\":1,\"bookmarks\":[";
  bool first = true;
  for (const CameraBookmark &bookmark : settings.bookmarks()) {
    if (!validSnapshot(bookmark.camera())) {
      throw std::invalid_argument("bookmark camera violates viewport contract");
    }
    if (!first) output += ',';
    first = false;
    const CameraSnapshot &camera = bookmark.camera();
    output += "{\"name\":\"" + escape(bookmark.name()) + "\",\"camera\":{";
    output += "\"target\":";
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
    output += "}}";
  }
  output += "]}\n";
  return output;
}

class Parser {
public:
  explicit Parser(std::string_view input) : input_(input) {}

  [[nodiscard]] InspectionSettings parse() {
    expect('{');
    expectKey("schema_version");
    const int version = integer();
    if (version != InspectionSettingsFile::kSchemaVersion) {
      fail("unsupported inspection settings schema version");
    }
    expect(',');
    expectKey("bookmarks");
    expect('[');
    InspectionSettings settings;
    if (!consume(']')) {
      do {
        settings.saveBookmark(bookmark());
      } while (consume(','));
      expect(']');
    }
    expect('}');
    whitespace();
    if (position_ != input_.size()) fail("trailing JSON data");
    settings.clearHistory();
    return settings;
  }

private:
  [[noreturn]] void fail(std::string_view message) const {
    throw std::invalid_argument(std::string(message));
  }
  void whitespace() {
    while (position_ < input_.size() &&
           (input_[position_] == ' ' || input_[position_] == '\n' ||
            input_[position_] == '\r' || input_[position_] == '\t')) ++position_;
  }
  void expect(char value) {
    whitespace();
    if (position_ == input_.size() || input_[position_] != value) fail("invalid inspection settings JSON");
    ++position_;
  }
  [[nodiscard]] bool consume(char value) {
    whitespace();
    if (position_ == input_.size() || input_[position_] != value) return false;
    ++position_;
    return true;
  }
  void expectKey(std::string_view key) {
    if (string() != key) fail("unexpected inspection settings field");
    expect(':');
  }
  [[nodiscard]] std::string string() {
    whitespace();
    if (position_ == input_.size() || input_[position_++] != '"') fail("expected JSON string");
    std::string result;
    while (position_ < input_.size()) {
      const char value = input_[position_++];
      if (value == '"') return result;
      if (static_cast<unsigned char>(value) < 0x20U) fail("control character in JSON string");
      if (value != '\\') { result += value; continue; }
      if (position_ == input_.size()) fail("incomplete JSON escape");
      const char escaped = input_[position_++];
      switch (escaped) {
      case '"': result += '"'; break; case '\\': result += '\\'; break;
      case '/': result += '/'; break; case 'b': result += '\b'; break;
      case 'f': result += '\f'; break; case 'n': result += '\n'; break;
      case 'r': result += '\r'; break; case 't': result += '\t'; break;
      case 'u': {
        const auto hex = [this](char digit) -> unsigned int {
          if (digit >= '0' && digit <= '9') return static_cast<unsigned int>(digit - '0');
          if (digit >= 'a' && digit <= 'f') return static_cast<unsigned int>(digit - 'a' + 10);
          if (digit >= 'A' && digit <= 'F') return static_cast<unsigned int>(digit - 'A' + 10);
          fail("invalid JSON Unicode escape");
        };
        if (input_.size() - position_ < 4U) fail("incomplete JSON Unicode escape");
        unsigned int codepoint = 0;
        for (int index = 0; index < 4; ++index)
          codepoint = (codepoint << 4U) | hex(input_[position_++]);
        // Supporting half a surrogate pair would silently corrupt text. Reject
        // both halves until full pair decoding is deliberately introduced.
        if (codepoint >= 0xd800U && codepoint <= 0xdfffU)
          fail("JSON surrogate escapes are unsupported");
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
  [[nodiscard]] int integer() {
    whitespace();
    const char *begin = input_.data() + position_;
    const char *end = input_.data() + input_.size();
    int value = 0;
    const auto [parsed, error] = std::from_chars(begin, end, value);
    if (error != std::errc{} || parsed == begin) fail("expected integer schema version");
    position_ = static_cast<std::size_t>(parsed - input_.data());
    return value;
  }
  [[nodiscard]] Eigen::Vector3d vector() {
    expect('[');
    Eigen::Vector3d value;
    for (int index = 0; index < 3; ++index) {
      if (index != 0) expect(',');
      value[index] = number();
    }
    expect(']');
    return value;
  }
  [[nodiscard]] CameraBookmark bookmark() {
    expect('{'); expectKey("name"); const std::string name = string();
    expect(','); expectKey("camera"); expect('{');
    CameraSnapshot camera;
    expectKey("target"); camera.target = vector();
    expect(','); expectKey("rotation_center"); camera.rotation_center = vector();
    expect(','); expectKey("camera_to_world"); expect('[');
    for (int row = 0; row < 3; ++row) {
      if (row != 0) expect(',');
      expect('[');
      for (int column = 0; column < 3; ++column) {
        if (column != 0) expect(',');
        camera.camera_to_world(row, column) = static_cast<float>(number());
      }
      expect(']');
    }
    expect(']'); expect(','); expectKey("distance"); camera.distance = number();
    expect(','); expectKey("fov_y_degrees"); camera.fov_y_degrees = static_cast<float>(number());
    expect('}'); expect('}');
    if (!validSnapshot(camera)) fail("bookmark camera violates viewport contract");
    return CameraBookmark(name, camera);
  }
  std::string_view input_;
  std::size_t position_ = 0;
};

[[nodiscard]] std::optional<std::string> readFile(const std::filesystem::path &path,
                                                   std::string *error) {
  std::error_code status_error;
  if (!std::filesystem::exists(path, status_error)) {
    if (status_error) setError(error, status_error.message());
    return status_error ? std::nullopt : std::optional<std::string>{""};
  }
  const auto size = std::filesystem::file_size(path, status_error);
  if (status_error || size > kMaxSettingsBytes) {
    setError(error, status_error ? status_error.message() : "inspection settings file exceeds 1 MiB");
    return std::nullopt;
  }
  std::ifstream file(path, std::ios::binary);
  if (!file) { setError(error, "cannot open inspection settings file"); return std::nullopt; }
  std::string content(static_cast<std::size_t>(size), '\0');
  file.read(content.data(), static_cast<std::streamsize>(content.size()));
  if (!file && !content.empty()) { setError(error, "cannot read inspection settings file"); return std::nullopt; }
  return content;
}

[[nodiscard]] std::string temporarySuffix() {
  static std::atomic<std::uint64_t> sequence{0};
  std::random_device random;
  std::ostringstream encoded;
  encoded << std::hex << random() << random() << random() << random() << '.'
          << sequence.fetch_add(1, std::memory_order_relaxed);
  return encoded.str();
}

[[nodiscard]] std::error_code writeExclusive(const std::filesystem::path &path,
                                              std::string_view contents) {
#if defined(_WIN32)
  const HANDLE handle = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr,
                                     CREATE_NEW,
                                     FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH,
                                     nullptr);
  if (handle == INVALID_HANDLE_VALUE)
    return {static_cast<int>(GetLastError()), std::system_category()};
  std::size_t offset = 0;
  bool written = true;
  while (offset < contents.size()) {
    const DWORD request = static_cast<DWORD>(std::min<std::size_t>(
        contents.size() - offset, static_cast<std::size_t>(MAXDWORD)));
    DWORD count = 0;
    if (WriteFile(handle, contents.data() + offset, request, &count, nullptr) == FALSE ||
        count == 0) {
      written = false;
      break;
    }
    offset += count;
  }
  if (written) written = FlushFileBuffers(handle) != FALSE;
  const DWORD error = written ? ERROR_SUCCESS : GetLastError();
  CloseHandle(handle);
  if (!written) {
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    return {static_cast<int>(error), std::system_category()};
  }
  return {};
#else
  const int descriptor = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC,
                                0600);
  if (descriptor < 0) return {errno, std::generic_category()};
  std::size_t offset = 0;
  int write_error = 0;
  while (offset < contents.size()) {
    const ssize_t count = ::write(descriptor, contents.data() + offset,
                                  contents.size() - offset);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) { write_error = errno; break; }
    offset += static_cast<std::size_t>(count);
  }
  if (write_error == 0 && ::fsync(descriptor) != 0) write_error = errno;
  if (::close(descriptor) != 0 && write_error == 0) write_error = errno;
  if (write_error != 0) {
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    return {write_error, std::generic_category()};
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

[[nodiscard]] std::error_code replaceFile(const std::filesystem::path &source,
                                           const std::filesystem::path &destination) {
#if defined(_WIN32)
  if (MoveFileExW(source.c_str(), destination.c_str(),
                  MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) == FALSE)
    return {static_cast<int>(GetLastError()), std::system_category()};
  return {};
#else
  std::error_code error;
  std::filesystem::rename(source, destination, error);
  return error;
#endif
}

} // namespace

InspectionSettingsFile::InspectionSettingsFile(std::filesystem::path file)
    : file_(std::move(file)) {}

const std::filesystem::path &InspectionSettingsFile::path() const noexcept { return file_; }

bool InspectionSettingsFile::load(InspectionSettings &settings, std::string *error) const {
  const auto content = readFile(file_, error);
  if (!content.has_value()) return false;
  if (content->empty()) { settings = {}; return true; }
  try {
    InspectionSettings parsed = Parser(*content).parse();
    settings = std::move(parsed);
    return true;
  } catch (const std::exception &exception) {
    setError(error, exception.what());
    return false;
  }
}

bool InspectionSettingsFile::save(const InspectionSettings &settings, std::string *error) const {
  try {
    const std::string content = serialize(settings);
    std::error_code filesystem_error;
    if (!file_.parent_path().empty()) {
      std::filesystem::create_directories(file_.parent_path(), filesystem_error);
      if (filesystem_error) { setError(error, filesystem_error.message()); return false; }
    }
    std::filesystem::path temporary;
    std::error_code write_error;
    bool created = false;
    for (int attempt = 0; attempt < 32; ++attempt) {
      temporary = file_;
      temporary += ".tmp." + temporarySuffix();
      write_error = writeExclusive(temporary, content);
      if (!write_error) {
        created = true;
        break;
      }
      if (!isAlreadyExists(write_error)) break;
    }
    if (!created) {
      setError(error, "cannot create exclusive temporary inspection settings file: " +
                          write_error.message());
      return false;
    }
    const std::error_code replace_error = replaceFile(temporary, file_);
    if (replace_error) {
      std::filesystem::remove(temporary, filesystem_error);
      setError(error, "cannot atomically replace inspection settings file: " +
                          replace_error.message());
      return false;
    }
    return true;
  } catch (const std::exception &exception) {
    setError(error, exception.what());
    return false;
  }
}

} // namespace kpt::gui
