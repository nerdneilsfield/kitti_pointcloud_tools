#include "kpt/workflow/workflow.hpp"

#include "kpt/io/io.hpp"
#include "kpt/label/label.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <map>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <system_error>

namespace kpt::workflow {
namespace {

std::mutex conversion_commit_mutex;

bool decodeUtf8(std::string_view input, std::u32string &output) {
  output.clear();
  for (std::size_t cursor = 0; cursor < input.size();) {
    const auto first = static_cast<unsigned char>(input[cursor]);
    char32_t code_point = 0;
    std::size_t length = 0;
    if (first <= 0x7fU) {
      code_point = first;
      length = 1;
    } else if (first >= 0xc2U && first <= 0xdfU) {
      code_point = first & 0x1fU;
      length = 2;
    } else if (first >= 0xe0U && first <= 0xefU) {
      code_point = first & 0x0fU;
      length = 3;
    } else if (first >= 0xf0U && first <= 0xf4U) {
      code_point = first & 0x07U;
      length = 4;
    } else {
      return false;
    }

    if (cursor + length > input.size()) {
      return false;
    }
    for (std::size_t offset = 1; offset < length; ++offset) {
      const auto continuation =
          static_cast<unsigned char>(input[cursor + offset]);
      if ((continuation & 0xc0U) != 0x80U) {
        return false;
      }
      code_point = (code_point << 6U) | (continuation & 0x3fU);
    }

    const bool overlong =
        (length == 2 && code_point < 0x80U) ||
        (length == 3 && code_point < 0x800U) ||
        (length == 4 && code_point < 0x10000U);
    const bool surrogate = code_point >= 0xd800U && code_point <= 0xdfffU;
    if (overlong || surrogate || code_point > 0x10ffffU) {
      return false;
    }
    output.push_back(code_point);
    cursor += length;
  }
  return true;
}

bool matchCharacterClass(std::u32string_view pattern, char32_t value,
                         std::size_t &consumed) {
  std::size_t cursor = 1;
  bool negate = false;
  if (cursor < pattern.size() &&
      (pattern[cursor] == '!' || pattern[cursor] == '^')) {
    negate = true;
    ++cursor;
  }

  bool matched = false;
  bool has_member = false;
  for (; cursor < pattern.size() && pattern[cursor] != ']'; ++cursor) {
    has_member = true;
    const char32_t first = pattern[cursor];
    if (cursor + 2 < pattern.size() && pattern[cursor + 1] == '-' &&
        pattern[cursor + 2] != ']') {
      const char32_t last = pattern[cursor + 2];
      matched = matched || (first <= value && value <= last);
      cursor += 2;
    } else {
      matched = matched || first == value;
    }
  }

  if (!has_member || cursor == pattern.size()) {
    consumed = 1;
    return value == '[';
  }
  consumed = cursor + 1;
  return negate ? !matched : matched;
}

bool matchDecodedGlob(std::u32string_view pattern,
                      std::u32string_view value) {
  std::size_t pattern_cursor = 0;
  std::size_t value_cursor = 0;
  std::size_t star_pattern = std::string_view::npos;
  std::size_t star_value = 0;

  while (value_cursor < value.size()) {
    if (pattern_cursor < pattern.size() && pattern[pattern_cursor] == '*') {
      star_pattern = ++pattern_cursor;
      star_value = value_cursor;
      continue;
    }
    if (pattern_cursor < pattern.size() &&
        pattern[pattern_cursor] == '?') {
      ++pattern_cursor;
      ++value_cursor;
      continue;
    }
    if (pattern_cursor + 1 < pattern.size() &&
        pattern[pattern_cursor] == '\\' &&
        pattern[pattern_cursor + 1] == value[value_cursor]) {
      pattern_cursor += 2;
      ++value_cursor;
      continue;
    }
    if (pattern_cursor < pattern.size() &&
        pattern[pattern_cursor] == '[') {
      std::size_t consumed = 0;
      if (matchCharacterClass(pattern.substr(pattern_cursor),
                              value[value_cursor], consumed)) {
        pattern_cursor += consumed;
        ++value_cursor;
        continue;
      }
    } else if (pattern_cursor < pattern.size() &&
               pattern[pattern_cursor] == value[value_cursor]) {
      ++pattern_cursor;
      ++value_cursor;
      continue;
    }

    if (star_pattern == std::string_view::npos) {
      return false;
    }
    pattern_cursor = star_pattern;
    value_cursor = ++star_value;
  }

  while (pattern_cursor < pattern.size() && pattern[pattern_cursor] == '*') {
    ++pattern_cursor;
  }
  return pattern_cursor == pattern.size();
}

// Glob syntax and filenames are UTF-8. Metacharacters are ASCII, while
// literals, '?', and bracket classes operate on Unicode code points. Invalid
// UTF-8 in either operand never matches, including a wildcard-only pattern.
bool matchGlob(std::string_view pattern, std::string_view value) {
  std::u32string decoded_pattern;
  std::u32string decoded_value;
  if (!decodeUtf8(pattern, decoded_pattern) ||
      !decodeUtf8(value, decoded_value)) {
    return false;
  }
  return matchDecodedGlob(decoded_pattern, decoded_value);
}

std::filesystem::path temporaryPath(const std::filesystem::path &output) {
  static thread_local std::mt19937_64 generator(std::random_device{}());
  const auto token = generator();
  return output.parent_path() /
         (output.stem().string() + ".kpt-tmp-" + std::to_string(token) +
          output.extension().string());
}

PointCloudIRGBPtr readPoses(const std::filesystem::path &path, std::uint8_t r,
                            std::uint8_t g, std::uint8_t b) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open poses: " + path.string());
  }
  auto cloud = std::make_shared<PointCloudIRGB>();
  std::string line;
  while (std::getline(input, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    std::array<float, 12> values{};
    bool complete = true;
    for (auto &value : values) {
      if (!(row >> value)) {
        complete = false;
        break;
      }
    }
    if (!complete)
      continue;
    PointT point{};
    point.x = values[3];
    point.y = values[7];
    point.z = values[11];
    point.r = r;
    point.g = g;
    point.b = b;
    point.intensity = 1.0F;
    cloud->push_back(point);
  }
  return cloud;
}

} // namespace

std::vector<std::filesystem::path> enumerate(const std::filesystem::path &dir,
                                             const std::string &glob) {
  if (!std::filesystem::is_directory(dir)) {
    throw std::runtime_error("not a directory: " + dir.string());
  }

  std::vector<std::filesystem::path> files;
  for (const auto &entry : std::filesystem::directory_iterator(dir)) {
    if (!entry.is_regular_file())
      continue;
    const auto filename = entry.path().filename().string();
    if (matchGlob(glob, filename)) {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

BatchPlan makeBatchPlan(const BatchConvertOptions &options) {
  BatchPlan plan;
  const auto extension = "." + toString(options.output_format);
  std::map<std::filesystem::path, std::filesystem::path> claimed_outputs;

  std::vector<std::filesystem::path> inputs;
  try {
    inputs = enumerate(options.input_dir, options.glob);
  } catch (const std::exception &error) {
    plan.error = error.what();
    return plan;
  }

  for (const auto &input : inputs) {
    auto output = options.output_dir / (input.stem().string() + extension);
    output = output.lexically_normal();

    if (const auto found = claimed_outputs.find(output);
        found != claimed_outputs.end()) {
      plan.rejected.push_back({OperationStatus::Failed, input, output,
                               "duplicate output path (already claimed by " +
                                   found->second.string() + ")",
                               0});
      continue;
    }
    claimed_outputs.emplace(output, input);
    plan.requests.push_back(
        {input, output, options.ascii_flavor, options.overwrite});
  }
  return plan;
}

OperationResult convert(const ConversionRequest &request) {
  OperationResult result;
  result.input = request.input;
  result.output = request.output;

  try {
    if (std::filesystem::exists(request.output) && !request.overwrite) {
      result.status = OperationStatus::Skipped;
      result.message = "output exists";
      return result;
    }

    if (!request.output.parent_path().empty()) {
      std::filesystem::create_directories(request.output.parent_path());
    }
    const auto cloud = kpt::load(request.input);
    const auto temporary = temporaryPath(request.output);
    try {
      kpt::save(temporary, *cloud, request.ascii_flavor);
      {
        std::lock_guard commit_lock(conversion_commit_mutex);
        if (std::filesystem::exists(request.output) && !request.overwrite) {
          std::error_code ignored;
          std::filesystem::remove(temporary, ignored);
          result.status = OperationStatus::Skipped;
          result.message = "output exists";
          return result;
        }
        std::filesystem::rename(temporary, request.output);
      }
    } catch (...) {
      std::error_code ignored;
      std::filesystem::remove(temporary, ignored);
      throw;
    }

    result.status = OperationStatus::Succeeded;
    result.point_count = cloud->size();
    result.message = "converted";
  } catch (const std::exception &error) {
    result.status = OperationStatus::Failed;
    result.message = error.what();
  }
  return result;
}

SequenceSource::SequenceSource(SequenceOptions options)
    : options_(std::move(options)),
      files_(enumerate(options_.input_dir, options_.glob)) {
  if (options_.label_dir) {
    label_map_ = kpt::rangeNetLabelMap();
    rgb_map_ = kpt::rgbLabelMap();
  }
}

SequenceFrame SequenceSource::load(std::size_t index) const {
  if (index >= files_.size()) {
    throw std::out_of_range("sequence frame index out of range");
  }

  auto cloud = kpt::load(files_[index]);
  if (options_.label_dir) {
    const auto label_path =
        *options_.label_dir / (files_[index].stem().string() + ".label");
    cloud = kpt::applyLabel(cloud, kpt::loadLabel(label_path),
                            label_map_, rgb_map_);
  }
  return {index, files_[index], std::move(cloud)};
}

PointCloudIRGBPtr SequenceSource::trajectory() const {
  auto combined = std::make_shared<PointCloudIRGB>();
  if (options_.poses) {
    *combined += *readPoses(*options_.poses, 255, 0, 0);
  }
  if (options_.poses2) {
    *combined += *readPoses(*options_.poses2, 0, 255, 0);
  }
  return combined;
}

} // namespace kpt::workflow
