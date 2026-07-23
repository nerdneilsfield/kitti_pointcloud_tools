#include "kpt/io/io.hpp"
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <iostream>

int main(int argc, char* argv[]) {
  popl::OptionParser op("pc_convert: single-file point cloud converter");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level", "0=err 1=warn 2=info 3=debug", 2);
  auto flavor = op.add<popl::Value<std::string>>("", "ascii-flavor",
              "xyz|xyzi|xyzrgb|xyzrgbi (only for ascii output)", "");
  op.parse(argc, argv);
  if (help->is_set()) { std::cout << op << "\n"; return 0; }
  switch (log_level->value()) {
    case 0: spdlog::set_level(spdlog::level::err); break;
    case 1: spdlog::set_level(spdlog::level::warn); break;
    case 2: spdlog::set_level(spdlog::level::info); break;
    case 3: spdlog::set_level(spdlog::level::debug); break;
  }
  auto positional = op.non_option_args();
  if (positional.size() < 2) { std::cerr << "usage: pc_convert <input> <output> [--ascii-flavor ...]\n"; return 1; }

  try {
    auto cloud = kpt::load(positional[0]);
    std::optional<kpt::Format> af;
    if (flavor->is_set() && !flavor->value().empty()) {
      std::string f = flavor->value();
      if (f == "xyz") af = kpt::Format::XYZ;
      else if (f == "xyzi") af = kpt::Format::XYZI;
      else if (f == "xyzrgb") af = kpt::Format::XYZRGB;
      else if (f == "xyzrgbi") af = kpt::Format::XYZRGBI;
      else { spdlog::error("unknown ascii-flavor: {}", f); return 1; }
    }
    kpt::save(positional[1], *cloud, af);
    spdlog::info("converted {} -> {} ({} points)", positional[0], positional[1], cloud->size());
  } catch (const std::exception& e) {
    spdlog::error("{}", e.what());
    return 1;
  }
  return 0;
}
