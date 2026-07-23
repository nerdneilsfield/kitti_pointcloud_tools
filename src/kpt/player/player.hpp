#pragma once
#include "kpt/types.hpp"
#include "kpt/render/render.hpp"
#include <filesystem>
#include <optional>
#include <string>

namespace kpt {

struct PlayerOpts {
  std::filesystem::path input_dir;
  std::string glob = "*";
  std::optional<std::filesystem::path> label_dir;
  std::optional<std::filesystem::path> poses, poses2;
  ColorBy colorby = ColorBy::Intensity;
  int point_size = 3;
  std::optional<std::string> snapshot_prefix;
  RenderOpts render_opts;
  int fps = 10;
};

class SequencePlayer {
 public:
  explicit SequencePlayer(const PlayerOpts& opts);
  void run();
 private:
  PlayerOpts opts_;
};

}  // namespace kpt
