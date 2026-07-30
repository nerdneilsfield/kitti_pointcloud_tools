#include <catch2/catch.hpp>

#include "kpt/io/io.hpp"
#include "kpt/workflow/workflow.hpp"

#include <filesystem>
#include <fstream>
#include <functional>
#include <random>
#include <thread>

namespace fs = std::filesystem;

namespace {

struct TempDirectory {
  static std::string token() {
    static std::mt19937_64 generator(std::random_device{}());
    return std::to_string(generator());
  }

  fs::path path = fs::temp_directory_path() / ("kpt-workflow-" + token());
  TempDirectory() { fs::create_directories(path); }
  ~TempDirectory() {
    std::error_code ignored;
    fs::remove_all(path, ignored);
  }
};

void writeXyz(const fs::path &path, float x = 1.0F) {
  std::ofstream output(path);
  output << x << " 2 3\n";
}

} // namespace

TEST_CASE("workflow enumerate filters and sorts regular files", "[workflow]") {
  TempDirectory temp;
  writeXyz(temp.path / "b.xyz");
  writeXyz(temp.path / "a.xyz");
  writeXyz(temp.path / "ignored.pcd");
  fs::create_directory(temp.path / "directory.xyz");

  const auto files = kpt::workflow::enumerate(temp.path, "*.xyz");
  REQUIRE(files.size() == 2);
  REQUIRE(files[0].filename() == "a.xyz");
  REQUIRE(files[1].filename() == "b.xyz");
}

TEST_CASE("workflow supports an empty directory", "[workflow]") {
  TempDirectory temp;
  REQUIRE(kpt::workflow::enumerate(temp.path, "*").empty());
}

TEST_CASE("sequence source accepts and sorts an explicit file catalog",
          "[workflow][web]") {
  TempDirectory temp;
  const auto first = temp.path / "0001.xyz";
  const auto second = temp.path / "0002.xyz";
  writeXyz(first, 1.0F);
  writeXyz(second, 2.0F);

  kpt::workflow::SequenceOptions options;
  options.input_dir = temp.path / "virtual";
  kpt::workflow::SequenceSource sequence(std::move(options),
                                         {second, first});

  REQUIRE(sequence.size() == 2);
  REQUIRE(sequence.files()[0] == first);
  REQUIRE(sequence.files()[1] == second);
  REQUIRE(sequence.load(1).cloud->points[0].x == 2.0F);
}

TEST_CASE("workflow glob matching is portable", "[workflow]") {
  TempDirectory temp;
  writeXyz(temp.path / "scan-1.xyz");
  writeXyz(temp.path / "scan-a.xyz");
  writeXyz(temp.path / "scan-12.xyz");
  writeXyz(temp.path / "scan-*.xyz");

  const auto single_character =
      kpt::workflow::enumerate(temp.path, "scan-?.xyz");
  REQUIRE(single_character.size() == 3);

  const auto digit = kpt::workflow::enumerate(temp.path, "scan-[0-9].xyz");
  REQUIRE(digit.size() == 1);
  REQUIRE(digit.front().filename() == "scan-1.xyz");

  const auto non_digit = kpt::workflow::enumerate(temp.path, "scan-[!0-9].xyz");
  REQUIRE(non_digit.size() == 2);
  REQUIRE(non_digit.back().filename() == "scan-a.xyz");

  const auto escaped = kpt::workflow::enumerate(temp.path, R"(scan-\*.xyz)");
  REQUIRE(escaped.size() == 1);
  REQUIRE(escaped.front().filename() == "scan-*.xyz");
}

TEST_CASE("workflow glob matching operates on UTF-8 code points",
          "[workflow]") {
  TempDirectory temp;
  writeXyz(temp.path / "点云.xyz");
  writeXyz(temp.path / "点图.xyz");
  writeXyz(temp.path / "地图.xyz");

  const auto literal = kpt::workflow::enumerate(temp.path, "点云.xyz");
  REQUIRE(literal.size() == 1);
  REQUIRE(literal.front().filename().u8string() == u8"点云.xyz");

  const auto single_code_point = kpt::workflow::enumerate(temp.path, "点?.xyz");
  REQUIRE(single_code_point.size() == 2);

  const auto character_class =
      kpt::workflow::enumerate(temp.path, "[点地]图.xyz");
  REQUIRE(character_class.size() == 2);
}

TEST_CASE("workflow glob rejects invalid UTF-8 patterns", "[workflow]") {
  TempDirectory temp;
  writeXyz(temp.path / "valid.xyz");
  const std::string invalid_pattern{"*\xC3\x28*", 4};
  REQUIRE(kpt::workflow::enumerate(temp.path, invalid_pattern).empty());
}

#ifndef _WIN32
TEST_CASE("workflow glob rejects invalid UTF-8 filenames", "[workflow]") {
  TempDirectory temp;
  std::string invalid_name = "bad-";
  invalid_name.append("\xC3\x28", 2);
  invalid_name += ".xyz";
  writeXyz(temp.path / invalid_name);
  writeXyz(temp.path / "valid.xyz");

  const auto wildcard = kpt::workflow::enumerate(temp.path, "*");
  REQUIRE(wildcard.size() == 1);
  REQUIRE(wildcard.front().filename() == "valid.xyz");
}
#endif

TEST_CASE("batch plan reports a missing input directory", "[workflow]") {
  TempDirectory temp;
  kpt::workflow::BatchConvertOptions options;
  options.input_dir = temp.path / "missing";
  options.output_dir = temp.path / "output";

  const auto plan = kpt::workflow::makeBatchPlan(options);
  REQUIRE(plan.error);
  REQUIRE(plan.requests.empty());
  REQUIRE(plan.rejected.empty());
}

TEST_CASE("batch plan rejects duplicate output stems", "[workflow]") {
  TempDirectory temp;
  const auto input = temp.path / "input";
  const auto output = temp.path / "output";
  fs::create_directory(input);
  writeXyz(input / "same.xyz");
  std::ofstream(input / "same.xyzi") << "1 2 3 4\n";

  kpt::workflow::BatchConvertOptions options;
  options.input_dir = input;
  options.output_dir = output;
  options.output_format = kpt::Format::PCD;
  const auto plan = kpt::workflow::makeBatchPlan(options);

  REQUIRE(plan.requests.size() == 1);
  REQUIRE(plan.rejected.size() == 1);
  REQUIRE(plan.rejected[0].message.find("duplicate output path") !=
          std::string::npos);
}

TEST_CASE("atomic conversion skips then overwrites existing output",
          "[workflow]") {
  TempDirectory temp;
  const auto input = temp.path / "input.xyz";
  const auto output = temp.path / "output.xyz";
  writeXyz(input, 4.0F);
  writeXyz(output, 9.0F);

  kpt::workflow::ConversionRequest request{input, output, kpt::Format::XYZ,
                                           false};
  auto result = kpt::workflow::convert(request);
  REQUIRE(result.status == kpt::workflow::OperationStatus::Skipped);
  REQUIRE(kpt::load(output)->points[0].x == 9.0F);

  request.overwrite = true;
  result = kpt::workflow::convert(request);
  REQUIRE(result.status == kpt::workflow::OperationStatus::Succeeded);
  REQUIRE(kpt::load(output)->points[0].x == 4.0F);

  for (const auto &entry : fs::directory_iterator(temp.path)) {
    REQUIRE(entry.path().filename().string().find(".kpt-tmp-") ==
            std::string::npos);
  }
}

TEST_CASE("concurrent no-overwrite conversions commit only once",
          "[workflow]") {
  TempDirectory temp;
  const auto input = temp.path / "input.xyz";
  const auto output = temp.path / "output.pcd";
  writeXyz(input);
  const kpt::workflow::ConversionRequest request{input, output, std::nullopt,
                                                 false};
  kpt::workflow::OperationResult first;
  kpt::workflow::OperationResult second;
  std::jthread first_thread([&] { first = kpt::workflow::convert(request); });
  std::jthread second_thread([&] { second = kpt::workflow::convert(request); });
  first_thread.join();
  second_thread.join();

  const int succeeded =
      (first.status == kpt::workflow::OperationStatus::Succeeded ? 1 : 0) +
      (second.status == kpt::workflow::OperationStatus::Succeeded ? 1 : 0);
  const int skipped =
      (first.status == kpt::workflow::OperationStatus::Skipped ? 1 : 0) +
      (second.status == kpt::workflow::OperationStatus::Skipped ? 1 : 0);
  REQUIRE(succeeded == 1);
  REQUIRE(skipped == 1);
  REQUIRE(kpt::load(output)->size() == 1);
}

TEST_CASE("sequence source loads frames and validates bounds", "[workflow]") {
  TempDirectory temp;
  writeXyz(temp.path / "0001.xyz", 1.0F);
  writeXyz(temp.path / "0002.xyz", 2.0F);

  kpt::workflow::SequenceOptions options;
  options.input_dir = temp.path;
  options.glob = "*.xyz";
  kpt::workflow::SequenceSource sequence(std::move(options));
  REQUIRE(sequence.size() == 2);
  REQUIRE(sequence.load(1).cloud->points[0].x == 2.0F);
  REQUIRE_THROWS_AS(sequence.load(2), std::out_of_range);
  REQUIRE(sequence.trajectory()->empty());
}

TEST_CASE("sequence source loads colored KITTI trajectories", "[workflow]") {
  TempDirectory temp;
  writeXyz(temp.path / "0001.xyz");
  const auto poses = temp.path / "poses.txt";
  std::ofstream(poses) << "1 0 0 4 0 1 0 5 0 0 1 6\n";

  kpt::workflow::SequenceOptions options;
  options.input_dir = temp.path;
  options.glob = "*.xyz";
  options.poses = poses;
  kpt::workflow::SequenceSource sequence(std::move(options));
  const auto trajectory = sequence.trajectory();

  REQUIRE(trajectory->size() == 1);
  REQUIRE(trajectory->points[0].x == 4.0F);
  REQUIRE(trajectory->points[0].y == 5.0F);
  REQUIRE(trajectory->points[0].z == 6.0F);
  REQUIRE(trajectory->points[0].r == 255);
}

TEST_CASE("sequence source preserves valid best-effort trajectories",
          "[workflow]") {
  TempDirectory temp;
  writeXyz(temp.path / "0001.xyz");
  const auto valid = temp.path / "poses.txt";
  std::ofstream(valid) << "1 0 0 4 0 1 0 5 0 0 1 6\n";

  kpt::workflow::SequenceOptions options;
  options.input_dir = temp.path;
  options.glob = "*.xyz";
  options.poses = valid;
  options.poses2 = temp.path / "missing.txt";
  const kpt::workflow::SequenceSource sequence(std::move(options));
  const auto trajectory = sequence.trajectoryBestEffort();

  REQUIRE(trajectory.cloud->size() == 1);
  REQUIRE(trajectory.cloud->points[0].r == 255);
  REQUIRE(trajectory.warnings.size() == 1);
  REQUIRE(trajectory.warnings.front().find("missing.txt") != std::string::npos);
}

TEST_CASE("sequence source reports malformed trajectory rows", "[workflow]") {
  TempDirectory temp;
  writeXyz(temp.path / "0001.xyz");
  const auto poses = temp.path / "poses.txt";
  std::ofstream(poses) << "malformed\n"
                       << "1 0 0 4 0 1 0 5 0 0 1 6 garbage\n"
                       << "1 0 0 nan 0 1 0 5 0 0 1 6\n"
                       << "1 0 0 4 0 1 0 5 0 0 1 6\n";

  kpt::workflow::SequenceOptions options;
  options.input_dir = temp.path;
  options.glob = "*.xyz";
  options.poses = poses;
  const kpt::workflow::SequenceSource sequence(std::move(options));
  const auto trajectory = sequence.trajectoryBestEffort();

  REQUIRE(trajectory.cloud->size() == 1);
  REQUIRE(trajectory.warnings.size() == 1);
  REQUIRE(trajectory.warnings.front().find("3 malformed row") !=
          std::string::npos);
}

TEST_CASE("sequence trajectory loading observes cancellation", "[workflow]") {
  TempDirectory temp;
  writeXyz(temp.path / "0001.xyz");
  const auto poses = temp.path / "poses.txt";
  std::ofstream(poses) << "1 0 0 4 0 1 0 5 0 0 1 6\n";

  kpt::workflow::SequenceOptions options;
  options.input_dir = temp.path;
  options.glob = "*.xyz";
  options.poses = poses;
  const kpt::workflow::SequenceSource sequence(std::move(options));
  std::stop_source cancellation;
  cancellation.request_stop();
  const auto trajectory =
      sequence.trajectoryBestEffort(cancellation.get_token());

  REQUIRE(trajectory.cloud->empty());
  REQUIRE(trajectory.warnings.empty());
}

TEST_CASE("sequence source applies matching semantic labels", "[workflow]") {
  TempDirectory temp;
  const auto frames = temp.path / "frames";
  const auto labels = temp.path / "labels";
  fs::create_directory(frames);
  fs::create_directory(labels);
  writeXyz(frames / "0001.xyz");
  writeXyz(frames / "0002.xyz", 2.0F);
  {
    std::ofstream output(labels / "0001.label", std::ios::binary);
    const int label = 40;
    output.write(reinterpret_cast<const char *>(&label), sizeof(label));
  }
  {
    std::ofstream output(labels / "0002.label", std::ios::binary);
    const int label = 44;
    output.write(reinterpret_cast<const char *>(&label), sizeof(label));
  }

  kpt::workflow::SequenceOptions options;
  options.input_dir = frames;
  options.glob = "*.xyz";
  options.label_dir = labels;
  kpt::workflow::SequenceSource sequence(std::move(options));
  const auto first = sequence.load(0);
  const auto second = sequence.load(1);

  REQUIRE(first.cloud->size() == 1);
  REQUIRE(first.cloud->points[0].intensity == 1.0F);
  REQUIRE(second.cloud->size() == 1);
  REQUIRE(second.cloud->points[0].intensity == 1.0F);
}

TEST_CASE("conversion failure is returned instead of thrown", "[workflow]") {
  TempDirectory temp;
  kpt::workflow::ConversionRequest request;
  request.input = temp.path / "missing.xyz";
  request.output = temp.path / "out.pcd";
  const auto result = kpt::workflow::convert(request);
  REQUIRE(result.status == kpt::workflow::OperationStatus::Failed);
  REQUIRE_FALSE(result.message.empty());
}

TEST_CASE("conversion cancellation is distinct from failure", "[workflow]") {
  TempDirectory temp;
  const auto input = temp.path / "input.xyz";
  const auto output = temp.path / "out.pcd";
  writeXyz(input);
  kpt::workflow::ConversionRequest request{input, output, std::nullopt, true};
  std::stop_source cancellation;
  cancellation.request_stop();

  const auto result = kpt::workflow::convert(request, cancellation.get_token());

  REQUIRE(result.status == kpt::workflow::OperationStatus::Cancelled);
  REQUIRE_FALSE(fs::exists(output));
}
