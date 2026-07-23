#include "kpt/io/io.hpp"
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <iostream>
#include <filesystem>
#include <fnmatch.h>
#include <algorithm>

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
  popl::OptionParser op("pc_batch_convert: batch convert a directory");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level", "", 2);
  auto in_dir = op.add<popl::Value<std::string>>("i", "input-dir", "input directory", "");
  auto out_dir = op.add<popl::Value<std::string>>("o", "output-dir", "output directory", "");
  auto to = op.add<popl::Value<std::string>>("t", "to", "bin|pcd|ply|xyz|xyzi|xyzrgb|xyzrgbi", "");
  auto glob = op.add<popl::Value<std::string>>("g", "glob", "fnmatch pattern, default *", "*");
  auto flavor = op.add<popl::Value<std::string>>("", "ascii-flavor", "override ascii flavor", "");
  op.parse(argc, argv);
  if (help->is_set()) { std::cout << op << "\n"; return 0; }
  // set log level (same switch as pc_convert) - factor into helper if desired
  switch (log_level->value()) { case 0: spdlog::set_level(spdlog::level::err); break; case 1: spdlog::set_level(spdlog::level::warn); break; case 2: spdlog::set_level(spdlog::level::info); break; case 3: spdlog::set_level(spdlog::level::debug); break; }

  if (in_dir->value().empty() || out_dir->value().empty() || to->value().empty()) {
    std::cerr << "required: --input-dir --output-dir --to\n"; return 1;
  }
  std::optional<kpt::Format> target_fmt;
  std::string t = to->value();
  if (t == "bin") target_fmt = kpt::Format::Bin;
  else if (t == "pcd") target_fmt = kpt::Format::PCD;
  else if (t == "ply") target_fmt = kpt::Format::PLY;
  else if (t == "xyz") target_fmt = kpt::Format::XYZ;
  else if (t == "xyzi") target_fmt = kpt::Format::XYZI;
  else if (t == "xyzrgb") target_fmt = kpt::Format::XYZRGB;
  else if (t == "xyzrgbi") target_fmt = kpt::Format::XYZRGBI;
  else { spdlog::error("unknown --to: {}", t); return 1; }

  std::optional<kpt::Format> af;
  if (flavor->is_set() && !flavor->value().empty()) {
    std::string f = flavor->value();
    if (f == "xyz") af = kpt::Format::XYZ;
    else if (f == "xyzi") af = kpt::Format::XYZI;
    else if (f == "xyzrgb") af = kpt::Format::XYZRGB;
    else if (f == "xyzrgbi") af = kpt::Format::XYZRGBI;
  }
  std::string ext = "." + t;
  fs::create_directories(out_dir->value());

  std::vector<fs::path> files;
  for (const auto& e : fs::directory_iterator(in_dir->value())) {
    if (fnmatch(glob->value().c_str(), e.path().filename().c_str(), 0) == 0)
      files.push_back(e.path());
  }
  std::sort(files.begin(), files.end());

  int ok = 0, fail = 0;
  for (const auto& f : files) {
    try {
      auto cloud = kpt::load(f);
      fs::path out = fs::path(out_dir->value()) / (f.stem().string() + ext);
      kpt::save(out, *cloud, af);
      spdlog::info("ok: {} -> {}", f.string(), out.string());
      ++ok;
    } catch (const std::exception& e) {
      spdlog::warn("fail: {} : {}", f.string(), e.what());
      ++fail;
    }
  }
  spdlog::info("done: {} ok, {} fail", ok, fail);
  return fail == 0 ? 0 : 1;
}
