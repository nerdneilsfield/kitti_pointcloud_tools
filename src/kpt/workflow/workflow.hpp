#pragma once

#include "kpt/io/format.hpp"
#include "kpt/types.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace kpt::workflow {

enum class OperationStatus { Succeeded, Skipped, Failed };

struct ConversionRequest {
  std::filesystem::path input;
  std::filesystem::path output;
  std::optional<Format> ascii_flavor;
  bool overwrite = false;
};

struct OperationResult {
  OperationStatus status = OperationStatus::Failed;
  std::filesystem::path input;
  std::filesystem::path output;
  std::string message;
  std::size_t point_count = 0;
};

struct BatchConvertOptions {
  std::filesystem::path input_dir;
  std::filesystem::path output_dir;
  std::string glob = "*";
  Format output_format = Format::PCD;
  std::optional<Format> ascii_flavor;
  bool overwrite = false;
};

struct BatchPlan {
  std::vector<ConversionRequest> requests;
  std::vector<OperationResult> rejected;
};

std::vector<std::filesystem::path> enumerate(const std::filesystem::path &dir,
                                             const std::string &glob);

BatchPlan makeBatchPlan(const BatchConvertOptions &options);

OperationResult convert(const ConversionRequest &request);

struct SequenceOptions {
  std::filesystem::path input_dir;
  std::string glob = "*";
  std::optional<std::filesystem::path> label_dir;
  std::optional<std::filesystem::path> poses;
  std::optional<std::filesystem::path> poses2;
};

struct SequenceFrame {
  std::size_t index = 0;
  std::filesystem::path path;
  PointCloudIRGBPtr cloud;
};

class SequenceSource {
public:
  explicit SequenceSource(SequenceOptions options);

  [[nodiscard]] std::size_t size() const { return files_.size(); }
  [[nodiscard]] bool empty() const { return files_.empty(); }
  [[nodiscard]] const std::vector<std::filesystem::path> &files() const {
    return files_;
  }

  SequenceFrame load(std::size_t index) const;
  PointCloudIRGBPtr trajectory() const;

private:
  SequenceOptions options_;
  std::vector<std::filesystem::path> files_;
};

} // namespace kpt::workflow
