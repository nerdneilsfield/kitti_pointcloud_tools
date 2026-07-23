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

TEST_CASE("sequence source applies matching semantic labels", "[workflow]") {
  TempDirectory temp;
  const auto frames = temp.path / "frames";
  const auto labels = temp.path / "labels";
  fs::create_directory(frames);
  fs::create_directory(labels);
  writeXyz(frames / "0001.xyz");
  {
    std::ofstream output(labels / "0001.label", std::ios::binary);
    const int label = 40;
    output.write(reinterpret_cast<const char *>(&label), sizeof(label));
  }

  kpt::workflow::SequenceOptions options;
  options.input_dir = frames;
  options.glob = "*.xyz";
  options.label_dir = labels;
  kpt::workflow::SequenceSource sequence(std::move(options));
  const auto frame = sequence.load(0);

  REQUIRE(frame.cloud->size() == 1);
  REQUIRE(frame.cloud->points[0].intensity == 1.0F);
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
