#include "kpt/workflow/workflow.hpp"
#include "kpt/cancellation.hpp"

#include "kpt/io/io.hpp"
#include "kpt/label/label.hpp"
#include "platform/utf8_path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace kpt::workflow {
namespace {

constexpr std::uintmax_t kMaxPoseFileBytes = std::uintmax_t{64} << 20U;
constexpr std::size_t kMaxPoseRows = 2'000'000U;
constexpr std::size_t kMaxPoseLineBytes = 4096U;
constexpr std::size_t kMaxBatchFiles = 100'000U;

std::string displayPath(const std::filesystem::path &path) {
  auto converted = platform::pathToUtf8(path);
  return converted ? std::move(converted).value() : "<invalid-native-path>";
}

std::filesystem::path utf8Path(std::string_view value) {
  auto converted = platform::pathFromUtf8(value);
  if (!converted)
    throw std::runtime_error(converted.error().message);
  return std::move(converted).value();
}

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

    const bool overlong = (length == 2 && code_point < 0x80U) ||
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

bool matchDecodedGlob(std::u32string_view pattern, std::u32string_view value) {
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
    if (pattern_cursor < pattern.size() && pattern[pattern_cursor] == '?') {
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
    if (pattern_cursor < pattern.size() && pattern[pattern_cursor] == '[') {
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

struct PosesReadResult {
  PointCloudIRGBPtr cloud;
  std::size_t skipped_rows = 0;
};

PosesReadResult readPoses(const std::filesystem::path &path, std::uint8_t r,
                          std::uint8_t g, std::uint8_t b,
                          std::stop_token stop = std::stop_token{}) {
  std::error_code type_error;
  const auto status = std::filesystem::symlink_status(path, type_error);
  if (type_error || std::filesystem::is_symlink(status) ||
      !std::filesystem::is_regular_file(status)) {
    throw std::runtime_error("poses is not a regular file: " +
                             displayPath(path));
  }
  const auto bytes = std::filesystem::file_size(path, type_error);
  if (type_error || bytes > kMaxPoseFileBytes)
    throw std::runtime_error("poses file exceeds 64 MiB limit: " +
                             displayPath(path));
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open poses: " + displayPath(path));
  }
  PosesReadResult result{std::make_shared<PointCloudIRGB>(), 0};
  std::string line;
  result.cloud->reserve(std::min<std::size_t>(kMaxPoseRows, bytes / 64U));
  while (std::getline(input, line)) {
    if (stop.stop_requested())
      break;
    if (line.size() > kMaxPoseLineBytes)
      throw std::runtime_error("pose line exceeds 4 KiB limit: " +
                               displayPath(path));
    if (result.cloud->size() + result.skipped_rows >= kMaxPoseRows)
      throw std::runtime_error("pose row count exceeds limit: " +
                               displayPath(path));
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
    std::string trailing;
    if (!complete || (row >> trailing) ||
        !std::ranges::all_of(
            values, [](float value) { return std::isfinite(value); })) {
      ++result.skipped_rows;
      continue;
    }
    PointT point{};
    point.x = values[3];
    point.y = values[7];
    point.z = values[11];
    point.r = r;
    point.g = g;
    point.b = b;
    point.intensity = 1.0F;
    result.cloud->push_back(point);
  }
  if (input.bad())
    throw std::runtime_error("cannot read poses: " + displayPath(path));
  return result;
}

} // namespace

std::vector<std::filesystem::path> enumerate(const std::filesystem::path &dir,
                                             const std::string &glob) {
  std::error_code directory_error;
  const auto directory_status =
      std::filesystem::symlink_status(dir, directory_error);
  if (directory_error || std::filesystem::is_symlink(directory_status) ||
      !std::filesystem::is_directory(directory_status)) {
    throw std::runtime_error("not a directory: " + displayPath(dir));
  }

  std::vector<std::filesystem::path> files;
  for (const auto &entry : std::filesystem::directory_iterator(dir)) {
    std::error_code entry_error;
    const auto status = entry.symlink_status(entry_error);
    if (entry_error || std::filesystem::is_symlink(status) ||
        !std::filesystem::is_regular_file(status))
      continue;
    const auto filename_result = platform::pathToUtf8(entry.path().filename());
    if (!filename_result)
      continue;
    const auto &filename = filename_result.value();
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
  if (inputs.size() > kMaxBatchFiles) {
    plan.error = "input file count exceeds 100000 limit";
    return plan;
  }

  for (const auto &input : inputs) {
    auto output_name = input.stem();
    output_name += utf8Path(extension).native();
    auto output = options.output_dir / output_name;
    output = output.lexically_normal();

    if (const auto found = claimed_outputs.find(output);
        found != claimed_outputs.end()) {
      plan.rejected.push_back({OperationStatus::Failed, input, output,
                               "duplicate output path (already claimed by " +
                                   displayPath(found->second) + ")",
                               0});
      continue;
    }
    claimed_outputs.emplace(output, input);
    plan.requests.push_back(
        {input, output, options.ascii_flavor, options.overwrite});
  }
  return plan;
}

OperationResult convert(const ConversionRequest &request,
                        std::stop_token stop) {
  OperationResult result;
  result.input = request.input;
  result.output = request.output;

  try {
    if (stop.stop_requested()) {
      result.status = OperationStatus::Cancelled;
      result.message = "cancelled";
      return result;
    }
    if (!request.output.parent_path().empty()) {
      std::filesystem::create_directories(request.output.parent_path());
    }
    const auto cloud = kpt::load(request.input, stop);
    const auto written = kpt::saveAtomic(
        request.output, *cloud, request.overwrite, request.ascii_flavor, stop);
    if (written == kpt::CloudWriteStatus::Skipped) {
      result.status = OperationStatus::Skipped;
      result.message = "output exists";
      return result;
    }

    result.status = OperationStatus::Succeeded;
    result.point_count = cloud->size();
    if (result.message.empty())
      result.message = "converted";
  } catch (const OperationCancelled &) {
    result.status = OperationStatus::Cancelled;
    result.message = "cancelled";
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

SequenceSource::SequenceSource(SequenceOptions options,
                               std::vector<std::filesystem::path> files)
    : options_(std::move(options)), files_(std::move(files)) {
  std::sort(files_.begin(), files_.end());
  if (options_.label_dir) {
    label_map_ = kpt::rangeNetLabelMap();
    rgb_map_ = kpt::rgbLabelMap();
  }
}

SequenceFrame SequenceSource::load(std::size_t index,
                                   std::stop_token stop) const {
  if (index >= files_.size()) {
    throw std::out_of_range("sequence frame index out of range");
  }

  auto cloud = kpt::load(files_[index], stop);
  if (options_.label_dir) {
    if (stop.stop_requested())
      throw OperationCancelled();
    auto label_name = files_[index].stem();
    label_name += utf8Path(".label").native();
    const auto label_path = *options_.label_dir / label_name;
    auto labels = kpt::loadLabel(label_path, stop);
    cloud = kpt::applyLabel(cloud, labels, label_map_, rgb_map_, false, stop);
  }
  return {index, files_[index], std::move(cloud)};
}

PointCloudIRGBPtr SequenceSource::trajectory() const {
  auto combined = std::make_shared<PointCloudIRGB>();
  if (options_.poses) {
    *combined += *readPoses(*options_.poses, 255, 0, 0).cloud;
  }
  if (options_.poses2) {
    *combined += *readPoses(*options_.poses2, 0, 255, 0).cloud;
  }
  return combined;
}

SequenceTrajectory
SequenceSource::trajectoryBestEffort(std::stop_token stop) const {
  SequenceTrajectory result{std::make_shared<PointCloudIRGB>(), {}};
  const auto append = [&result, stop](const std::filesystem::path &path,
                                      std::uint8_t red, std::uint8_t green,
                                      std::uint8_t blue) {
    if (stop.stop_requested())
      return;
    try {
      auto poses = readPoses(path, red, green, blue, stop);
      if (stop.stop_requested())
        return;
      if (result.cloud->empty()) {
        result.cloud = std::move(poses.cloud);
      } else {
        std::size_t index = 0;
        for (const auto &point : *poses.cloud) {
          if ((index++ % 4096U) == 0U && stop.stop_requested())
            return;
          result.cloud->push_back(point);
        }
      }
      if (poses.skipped_rows != 0) {
        result.warnings.push_back("Trajectory input contains " +
                                  std::to_string(poses.skipped_rows) +
                                  " malformed row(s): " + displayPath(path));
      }
    } catch (const std::bad_alloc &) {
      throw;
    } catch (const std::exception &error) {
      result.warnings.push_back("Trajectory input ignored: " +
                                displayPath(path) + ": " + error.what());
    } catch (...) {
      result.warnings.push_back(
          "Trajectory input ignored: " + displayPath(path) + ": unknown error");
    }
  };
  if (options_.poses)
    append(*options_.poses, 255, 0, 0);
  if (options_.poses2)
    append(*options_.poses2, 0, 255, 0);
  return result;
}

} // namespace kpt::workflow
