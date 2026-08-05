#include "kpt/io/conversion_options.hpp"
#include "kpt/workflow/workflow.hpp"
#include "platform/utf8_path.hpp"
#include <iostream>
#include <popl.hpp>
#include <spdlog/spdlog.h>

namespace {
std::string displayPath(const std::filesystem::path &path) {
  auto value = kpt::platform::pathToUtf8(path);
  return value ? std::move(value).value() : "<invalid-native-path>";
}
} // namespace

int main(int argc, char *argv[]) {
  popl::OptionParser op("kpt_batch_convert: batch convert a directory");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level", "", 2);
  auto in_dir =
      op.add<popl::Value<std::string>>("i", "input-dir", "input directory", "");
  auto out_dir = op.add<popl::Value<std::string>>("o", "output-dir",
                                                  "output directory", "");
  auto to = op.add<popl::Value<std::string>>(
      "t", "to", "bin|pcd|ply|xyz|xyzi|xyzrgb|xyzrgbi", "");
  auto glob = op.add<popl::Value<std::string>>(
      "g", "glob", "fnmatch pattern, default *", "*");
  auto flavor = op.add<popl::Value<std::string>>("", "ascii-flavor",
                                                 "override ascii flavor", "");
  op.parse(argc, argv);
  if (help->is_set()) {
    std::cout << op << "\n";
    return 0;
  }
  try {
    kpt::io::validateLogLevel(log_level->value());
  } catch (const std::invalid_argument &error) {
    spdlog::error("{}", error.what());
    return 1;
  }
  // set log level (same switch as kpt_convert) - factor into helper if desired
  switch (log_level->value()) {
  case 0:
    spdlog::set_level(spdlog::level::err);
    break;
  case 1:
    spdlog::set_level(spdlog::level::warn);
    break;
  case 2:
    spdlog::set_level(spdlog::level::info);
    break;
  case 3:
    spdlog::set_level(spdlog::level::debug);
    break;
  }

  if (in_dir->value().empty() || out_dir->value().empty() ||
      to->value().empty()) {
    std::cerr << "required: --input-dir --output-dir --to\n";
    return 1;
  }
  std::optional<kpt::Format> target_fmt;
  std::string t = to->value();
  if (t == "bin")
    target_fmt = kpt::Format::Bin;
  else if (t == "pcd")
    target_fmt = kpt::Format::PCD;
  else if (t == "ply")
    target_fmt = kpt::Format::PLY;
  else if (t == "xyz")
    target_fmt = kpt::Format::XYZ;
  else if (t == "xyzi")
    target_fmt = kpt::Format::XYZI;
  else if (t == "xyzrgb")
    target_fmt = kpt::Format::XYZRGB;
  else if (t == "xyzrgbi")
    target_fmt = kpt::Format::XYZRGBI;
  else {
    spdlog::error("unknown --to: {}", t);
    return 1;
  }

  std::optional<kpt::Format> af;
  try {
    af = kpt::io::parseAsciiFlavor(flavor->value());
    kpt::io::validateAsciiFlavor(*target_fmt, af);
  } catch (const std::invalid_argument &error) {
    spdlog::error("{}", error.what());
    return 1;
  }
  kpt::workflow::BatchConvertOptions options;
  options.input_dir = in_dir->value();
  options.output_dir = out_dir->value();
  options.glob = glob->value();
  options.output_format = *target_fmt;
  options.ascii_flavor = af;
  options.overwrite = true; // Preserve the historical CLI behavior.
  try {
    const auto plan = kpt::workflow::makeBatchPlan(options);
    if (plan.error) {
      spdlog::error("{}", *plan.error);
      return 1;
    }
    std::filesystem::create_directories(options.output_dir);
    int ok = 0, fail = 0;
    for (const auto &rejected : plan.rejected) {
      spdlog::warn("fail: {} : {}", displayPath(rejected.input),
                   rejected.message);
      ++fail;
    }
    for (const auto &request : plan.requests) {
      const auto result = kpt::workflow::convert(request);
      if (result.status == kpt::workflow::OperationStatus::Succeeded) {
        spdlog::info("ok: {} -> {}", displayPath(result.input),
                     displayPath(result.output));
        ++ok;
      } else {
        spdlog::warn("fail: {} : {}", displayPath(result.input),
                     result.message);
        ++fail;
      }
    }
    spdlog::info("done: {} ok, {} fail", ok, fail);
    return fail == 0 ? 0 : 1;
  } catch (const std::exception &error) {
    spdlog::error("{}", error.what());
    return 1;
  }
}
