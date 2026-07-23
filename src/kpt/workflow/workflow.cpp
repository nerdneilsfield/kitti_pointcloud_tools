#include "kpt/workflow/workflow.hpp"

#include "kpt/io/io.hpp"
#include "kpt/label/label.hpp"

#include <algorithm>
#include <array>
#include <fnmatch.h>
#include <fstream>
#include <map>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace kpt::workflow {
namespace {

std::mutex conversion_commit_mutex;

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
    if (fnmatch(glob.c_str(), filename.c_str(), 0) == 0) {
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

  for (const auto &input : enumerate(options.input_dir, options.glob)) {
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
      files_(enumerate(options_.input_dir, options_.glob)) {}

SequenceFrame SequenceSource::load(std::size_t index) const {
  if (index >= files_.size()) {
    throw std::out_of_range("sequence frame index out of range");
  }

  auto cloud = kpt::load(files_[index]);
  if (options_.label_dir) {
    const auto label_path =
        *options_.label_dir / (files_[index].stem().string() + ".label");
    cloud = kpt::applyLabel(cloud, kpt::loadLabel(label_path),
                            kpt::rangeNetLabelMap(), kpt::rgbLabelMap());
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
