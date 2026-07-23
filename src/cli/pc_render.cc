#include "kpt/io/io.hpp"
#include "kpt/render/render.hpp"
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>

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
  popl::OptionParser op("pc_render: multi-view PNG snapshot");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level", "", 2);
  auto prefix = op.add<popl::Value<std::string>>("o", "output-prefix", "output filename prefix", "");
  auto w = op.add<popl::Value<int>>("", "width", "", 640);
  auto h = op.add<popl::Value<int>>("", "height", "", 480);
  auto fov = op.add<popl::Value<float>>("", "fov", "field of view degrees", 120.0f);
  auto views = op.add<popl::Value<std::string>>("", "views", "all|front,right,...", "all");
  op.parse(argc, argv);
  if (help->is_set()) { std::cout << op << "\n"; return 0; }
  switch (log_level->value()) { case 0: spdlog::set_level(spdlog::level::err); break; case 1: spdlog::set_level(spdlog::level::warn); break; case 2: spdlog::set_level(spdlog::level::info); break; case 3: spdlog::set_level(spdlog::level::debug); break; }

  auto pos = op.non_option_args();
  if (pos.empty() || prefix->value().empty()) {
    std::cerr << "usage: pc_render <file> -o <prefix> [options]\n"; return 1;
  }

  try {
    auto cloud = kpt::load(pos[0]);
    kpt::RenderOpts opts; opts.width = w->value(); opts.height = h->value(); opts.fov = fov->value();
    std::string vs = views->value();
    if (vs != "all") {
      opts.views.clear();
      std::stringstream ss(vs); std::string item;
      while (std::getline(ss, item, ',')) opts.views.push_back(parseView(item));
    }
    auto results = kpt::renderMultiView(cloud, opts);
    for (const auto& r : results) {
      std::string fn = prefix->value() + "_" + r.view_name + ".png";
      cv::imwrite(fn, r.image);
      spdlog::info("wrote {}", fn);
    }
  } catch (const std::exception& e) {
    spdlog::error("{}", e.what());
    return 1;
  }
  return 0;
}
