#include "kpt/player/player.hpp"
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <iostream>
#include <sstream>

static kpt::ColorBy parseColorBy(const std::string& s) {
  if (s == "rgb") return kpt::ColorBy::RGB;
  if (s == "z") return kpt::ColorBy::Z;
  if (s == "label") return kpt::ColorBy::Label;
  if (s == "none") return kpt::ColorBy::None;
  return kpt::ColorBy::Intensity;
}

static kpt::View parseView(const std::string& s) {
  if (s == "front") return kpt::View::Front;
  if (s == "right") return kpt::View::Right;
  if (s == "back") return kpt::View::Back;
  if (s == "left") return kpt::View::Left;
  if (s == "top") return kpt::View::Top;
  if (s == "bottom") return kpt::View::Bottom;
  if (s == "toprightfront") return kpt::View::TopRightFront;
  if (s == "topleftfront") return kpt::View::TopLeftFront;
  if (s == "botrightfront") return kpt::View::BotRightFront;
  if (s == "botleftfront") return kpt::View::BotLeftFront;
  throw std::runtime_error("unknown view: " + s);
}

int main(int argc, char* argv[]) {
  popl::OptionParser op("pc_player: sequence player");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level", "", 2);
  auto in_dir = op.add<popl::Value<std::string>>("i", "input-dir", "input directory", "");
  auto glob = op.add<popl::Value<std::string>>("g", "glob", "fnmatch pattern", "*");
  auto label_dir = op.add<popl::Value<std::string>>("", "label-dir", "semantic label dir", "");
  auto poses = op.add<popl::Value<std::string>>("", "poses", "pose csv file", "");
  auto poses2 = op.add<popl::Value<std::string>>("", "poses2", "second pose csv", "");
  auto colorby = op.add<popl::Value<std::string>>("c", "colorby", "intensity|rgb|z|label|none", "intensity");
  auto psize = op.add<popl::Value<int>>("s", "point-size", "", 3);
  auto snap = op.add<popl::Value<std::string>>("", "snapshot", "output prefix for per-frame PNGs", "");
  auto snap_w = op.add<popl::Value<int>>("", "snapshot-w", "", 640);
  auto snap_h = op.add<popl::Value<int>>("", "snapshot-h", "", 480);
  auto snap_fov = op.add<popl::Value<float>>("", "snapshot-fov", "", 120.0f);
  auto snap_views = op.add<popl::Value<std::string>>("", "snapshot-views", "all|front,right,...", "all");
  auto fps = op.add<popl::Value<int>>("f", "fps", "", 10);
  op.parse(argc, argv);
  if (help->is_set()) { std::cout << op << "\n"; return 0; }
  switch (log_level->value()) { case 0: spdlog::set_level(spdlog::level::err); break; case 1: spdlog::set_level(spdlog::level::warn); break; case 2: spdlog::set_level(spdlog::level::info); break; case 3: spdlog::set_level(spdlog::level::debug); break; }

  if (in_dir->value().empty()) { std::cerr << "required: --input-dir\n"; return 1; }

  kpt::PlayerOpts opts;
  opts.input_dir = in_dir->value();
  opts.glob = glob->value();
  if (label_dir->is_set() && !label_dir->value().empty()) opts.label_dir = label_dir->value();
  if (poses->is_set() && !poses->value().empty()) opts.poses = poses->value();
  if (poses2->is_set() && !poses2->value().empty()) opts.poses2 = poses2->value();
  opts.colorby = parseColorBy(colorby->value());
  opts.point_size = psize->value();
  if (snap->is_set() && !snap->value().empty()) {
    opts.snapshot_prefix = snap->value();
    opts.render_opts.width = snap_w->value();
    opts.render_opts.height = snap_h->value();
    opts.render_opts.fov = snap_fov->value();
    std::string vs = snap_views->value();
    if (vs != "all") {
      opts.render_opts.views.clear();
      std::stringstream ss(vs); std::string item;
      while (std::getline(ss, item, ',')) {
        opts.render_opts.views.push_back(parseView(item));
      }
    }
  }
  opts.fps = fps->value();

  try {
    kpt::SequencePlayer player(opts);
    player.run();
  } catch (const std::exception& e) {
    spdlog::error("{}", e.what());
    return 1;
  }
  return 0;
}
