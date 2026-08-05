#include "kpt/io/conversion_options.hpp"
#include "kpt/io/io.hpp"
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <iostream>

int main(int argc, char* argv[]) {
  popl::OptionParser op("kpt_convert: single-file point cloud converter");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level", "0=err 1=warn 2=info 3=debug", 2);
  auto flavor = op.add<popl::Value<std::string>>("", "ascii-flavor",
              "xyz|xyzi|xyzrgb|xyzrgbi (only for ascii output)", "");
  auto overwrite = op.add<popl::Switch>("", "overwrite",
                                        "replace existing output");
  try {
    op.parse(argc, argv);
  } catch (const std::exception &error) {
    spdlog::error("argument error: {}", error.what());
    return 2;
  }
  if (help->is_set()) { std::cout << op << "\n"; return 0; }
  try {
    kpt::io::validateLogLevel(log_level->value());
  } catch (const std::invalid_argument &error) {
    spdlog::error("{}", error.what());
    return 1;
  }
  switch (log_level->value()) {
    case 0: spdlog::set_level(spdlog::level::err); break;
    case 1: spdlog::set_level(spdlog::level::warn); break;
    case 2: spdlog::set_level(spdlog::level::info); break;
    case 3: spdlog::set_level(spdlog::level::debug); break;
  }
  auto positional = op.non_option_args();
  if (positional.size() != 2) {
    std::cerr << "usage: kpt_convert <input> <output> [--ascii-flavor ...]\n";
    return 1;
  }

  try {
    const auto af = kpt::io::parseAsciiFlavor(flavor->value());
    kpt::io::validateAsciiFlavor(positional[1], af);
    auto cloud = kpt::load(positional[0]);
    const auto status = kpt::saveAtomic(positional[1], *cloud,
                                        overwrite->is_set(), af);
    if (status == kpt::CloudWriteStatus::Skipped) {
      spdlog::error("output exists; pass --overwrite to replace: {}",
                    positional[1]);
      return 2;
    }
    spdlog::info("converted {} -> {} ({} points)", positional[0], positional[1], cloud->size());
  } catch (const std::exception& e) {
    spdlog::error("{}", e.what());
    return 1;
  }
  return 0;
}
