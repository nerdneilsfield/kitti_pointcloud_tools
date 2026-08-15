#include "gui/inspection_settings.hpp"

#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string_view>
#include <system_error>

namespace kpt::gui {
namespace {

constexpr std::uintmax_t kMaxSettingsBytes = std::uintmax_t{1} << 20U;

void setError(std::string *error, std::string message) {
  if (error != nullptr) {
    *error = std::move(message);
  }
}

[[nodiscard]] bool finiteSnapshot(const CameraSnapshot &camera) noexcept {
  return camera.target.allFinite() && camera.rotation_center.allFinite() &&
         camera.camera_to_world.allFinite() && std::isfinite(camera.distance) &&
         std::isfinite(camera.fov_y_degrees);
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
  char buffer[64];
  const auto [end, error] = std::to_chars(std::begin(buffer), std::end(buffer), value,
                                          std::chars_format::general);
  if (error != std::errc{}) {
    throw std::runtime_error("cannot serialize inspection camera number");
  }
  output.append(buffer, end);
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
    if (!finiteSnapshot(bookmark.camera())) {
      throw std::invalid_argument("bookmark camera contains non-finite values");
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
      default: fail("unsupported JSON escape");
      }
    }
    fail("unterminated JSON string");
  }
  [[nodiscard]] double number() {
    whitespace();
    const char *begin = input_.data() + position_;
    const char *end = input_.data() + input_.size();
    double value = 0.0;
    const auto [parsed, error] = std::from_chars(begin, end, value, std::chars_format::general);
    if (error != std::errc{} || parsed == begin || !std::isfinite(value)) fail("expected finite JSON number");
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
    if (!finiteSnapshot(camera)) fail("non-finite bookmark camera");
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
    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    const std::filesystem::path temporary = file_.string() + ".tmp." + std::to_string(nonce);
    {
      std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
      if (!file) { setError(error, "cannot create temporary inspection settings file"); return false; }
      file.write(content.data(), static_cast<std::streamsize>(content.size()));
      file.flush();
      if (!file) { std::filesystem::remove(temporary, filesystem_error); setError(error, "cannot write temporary inspection settings file"); return false; }
    }
    std::filesystem::rename(temporary, file_, filesystem_error);
    if (filesystem_error) {
      std::filesystem::remove(temporary, filesystem_error);
      setError(error, "cannot atomically replace inspection settings file: " + filesystem_error.message());
      return false;
    }
    return true;
  } catch (const std::exception &exception) {
    setError(error, exception.what());
    return false;
  }
}

} // namespace kpt::gui
