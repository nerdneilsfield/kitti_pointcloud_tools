#include "kpt/io/io.hpp"
#include "kpt/viewer/viewer.hpp"
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <iostream>
#include <sstream>

int main(int argc, char* argv[]) {
  popl::OptionParser op("pc_viewer: single-frame interactive viewer");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level", "", 2);
  auto colorby = op.add<popl::Value<std::string>>("c", "colorby", "intensity|rgb|z|none", "intensity");
  auto psize = op.add<popl::Value<int>>("s", "point-size", "", 3);
  auto bg = op.add<popl::Value<std::string>>("b", "bg", "r,g,b (0-1)", "0,0,0");
  op.parse(argc, argv);
  if (help->is_set()) { std::cout << op << "\n"; return 0; }
  switch (log_level->value()) { case 0: spdlog::set_level(spdlog::level::err); break; case 1: spdlog::set_level(spdlog::level::warn); break; case 2: spdlog::set_level(spdlog::level::info); break; case 3: spdlog::set_level(spdlog::level::debug); break; }

  auto pos = op.non_option_args();
  if (pos.empty()) { std::cerr << "usage: pc_viewer <file> [options]\n"; return 1; }

  kpt::ColorBy cb = kpt::ColorBy::Intensity;
  std::string c = colorby->value();
  if (c == "rgb") cb = kpt::ColorBy::RGB;
  else if (c == "z") cb = kpt::ColorBy::Z;
  else if (c == "none") cb = kpt::ColorBy::None;

  Eigen::Vector3f bgv(0,0,0);
  { std::stringstream ss(bg->value()); char sep; ss >> bgv.x() >> sep >> bgv.y() >> sep >> bgv.z(); }

  try {
    auto cloud = kpt::load(pos[0]);
    kpt::ViewerOpts opts; opts.colorby = cb; opts.point_size = psize->value(); opts.bg = bgv;
    kpt::InteractiveViewer viewer(opts);
    viewer.show(cloud, pos[0]);
    viewer.spin();
  } catch (const std::exception& e) {
    spdlog::error("{}", e.what());
    return 1;
  }
  return 0;
}
