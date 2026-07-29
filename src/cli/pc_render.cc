#include "kpt/io/io.hpp"
#include "kpt/render/render.hpp"
#include <cmath>
#include <iostream>
#include <popl.hpp>
#include <spdlog/spdlog.h>
#include <sstream>

static std::size_t visiblePixelCount(const kpt::ImageRGB8 &image) {
  std::size_t count = 0;
  for (std::size_t offset = 0; offset < image.pixels().size(); offset += 3) {
    if (image.pixels()[offset] != 0 || image.pixels()[offset + 1] != 0 ||
        image.pixels()[offset + 2] != 0) {
      ++count;
    }
  }
  return count;
}

static kpt::View parseView(const std::string &s) {
  if (s == "front")
    return kpt::View::Front;
  if (s == "right")
    return kpt::View::Right;
  if (s == "back")
    return kpt::View::Back;
  if (s == "left")
    return kpt::View::Left;
  if (s == "top")
    return kpt::View::Top;
  if (s == "bottom")
    return kpt::View::Bottom;
  if (s == "toprightfront")
    return kpt::View::TopRightFront;
  if (s == "topleftfront")
    return kpt::View::TopLeftFront;
  if (s == "botrightfront")
    return kpt::View::BotRightFront;
  if (s == "botleftfront")
    return kpt::View::BotLeftFront;
  throw std::runtime_error("unknown view: " + s);
}

static kpt::RenderColorMode parseColorMode(const std::string &value) {
  if (value == "auto")
    return kpt::RenderColorMode::Auto;
  if (value == "rgb")
    return kpt::RenderColorMode::RGB;
  if (value == "intensity")
    return kpt::RenderColorMode::Intensity;
  if (value == "z")
    return kpt::RenderColorMode::Z;
  if (value == "solid")
    return kpt::RenderColorMode::Solid;
  throw std::runtime_error("unknown color mode: " + value);
}

static kpt::RenderProjection parseProjection(const std::string &value) {
  if (value == "orthographic")
    return kpt::RenderProjection::Orthographic;
  if (value == "perspective")
    return kpt::RenderProjection::Perspective;
  throw std::runtime_error("unknown projection: " + value);
}

int main(int argc, char *argv[]) {
  popl::OptionParser op("pc_render: multi-view PNG snapshot");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level", "", 2);
  auto prefix = op.add<popl::Value<std::string>>("o", "output-prefix",
                                                 "output filename prefix", "");
  auto w = op.add<popl::Value<int>>("", "width", "", 640);
  auto h = op.add<popl::Value<int>>("", "height", "", 480);
  auto fov =
      op.add<popl::Value<float>>("", "fov", "field of view degrees", 120.0f);
  auto projection = op.add<popl::Value<std::string>>(
      "", "projection", "orthographic|perspective", "orthographic");
  auto trim_percent = op.add<popl::Value<float>>(
      "", "trim-percent", "outlier percentage removed from each axis tail",
      1.0F);
  auto views = op.add<popl::Value<std::string>>("", "views",
                                                "all|front,right,...", "all");
  auto color_by = op.add<popl::Value<std::string>>(
      "", "color-by", "auto|rgb|intensity|z|solid", "auto");
  op.parse(argc, argv);
  if (help->is_set()) {
    std::cout << op << "\n";
    return 0;
  }
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

  auto pos = op.non_option_args();
  if (pos.empty() || prefix->value().empty()) {
    std::cerr << "usage: pc_render <file> -o <prefix> [options]\n";
    return 1;
  }

  try {
    kpt::RenderOpts opts;
    opts.width = w->value();
    opts.height = h->value();
    opts.fov = fov->value();
    opts.projection = parseProjection(projection->value());
    opts.trim_percent = trim_percent->value();
    opts.color_mode = parseColorMode(color_by->value());
    if (!std::isfinite(opts.trim_percent) || opts.trim_percent < 0.0F ||
        opts.trim_percent >= 50.0F) {
      throw std::runtime_error("--trim-percent must be in [0, 50)");
    }
    if (fov->is_set() &&
        opts.projection != kpt::RenderProjection::Perspective) {
      throw std::runtime_error("--fov requires --projection perspective");
    }
    std::string vs = views->value();
    if (vs != "all") {
      opts.views.clear();
      std::stringstream ss(vs);
      std::string item;
      while (std::getline(ss, item, ','))
        opts.views.push_back(parseView(item));
    }
    auto cloud = kpt::load(pos[0]);
    spdlog::info("rendering {} points into {} view(s) at {}x{}, "
                 "projection={}, trim={}%, color={}",
                 cloud->size(), opts.views.size(), opts.width, opts.height,
                 kpt::renderProjectionName(opts.projection), opts.trim_percent,
                 kpt::renderColorModeName(opts.color_mode));
    auto results = kpt::renderMultiView(cloud, opts);
    if (!results.empty()) {
      const auto &stats = results.front().cloud_stats;
      const float retained_ratio =
          stats.finite_points == 0U
              ? 0.0F
              : 100.0F * static_cast<float>(stats.retained_points) /
                    static_cast<float>(stats.finite_points);
      spdlog::info("bounds LxWxH: input={:.2f}x{:.2f}x{:.2f} m, "
                   "framed={:.2f}x{:.2f}x{:.2f} m; retained {}/{} ({:.1f}%)",
                   stats.input_dimensions[0], stats.input_dimensions[1],
                   stats.input_dimensions[2], stats.framed_dimensions[0],
                   stats.framed_dimensions[1], stats.framed_dimensions[2],
                   stats.retained_points, stats.finite_points, retained_ratio);
    }
    for (const auto &r : results) {
      std::string fn = prefix->value() + "_" + r.view_name + ".png";
      static_cast<void>(kpt::writeImageAtomic(fn, r.image, true));
      const auto visible_pixels = visiblePixelCount(r.image);
      if (visible_pixels == 0) {
        spdlog::warn("wrote {} but it contains no visible pixels", fn);
      } else {
        spdlog::info("wrote {} ({} visible pixels)", fn, visible_pixels);
      }
    }
  } catch (const std::exception &e) {
    spdlog::error("{}", e.what());
    return 1;
  }
  return 0;
}
