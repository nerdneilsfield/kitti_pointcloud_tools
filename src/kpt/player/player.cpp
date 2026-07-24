#include "kpt/player/player.hpp"

#include "kpt/render/render.hpp"
#include "kpt/viewer/viewer.hpp"
#include "kpt/workflow/workflow.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <thread>

namespace kpt {

SequencePlayer::SequencePlayer(const PlayerOpts& opts) : opts_(opts) {}

void SequencePlayer::run() {
  workflow::SequenceOptions source_options;
  source_options.input_dir = opts_.input_dir;
  source_options.glob = opts_.glob;
  source_options.label_dir = opts_.label_dir;
  source_options.poses = opts_.poses;
  source_options.poses2 = opts_.poses2;
  workflow::SequenceSource source(std::move(source_options));

  spdlog::info("player: {} files in {}", source.size(),
               opts_.input_dir.string());
  if (source.empty()) return;

  ViewerOpts vopts;
  vopts.colorby = opts_.colorby;
  vopts.point_size = opts_.point_size;
  InteractiveViewer viewer(vopts);

  // Optional trajectory viewers (poses / poses2).
  std::unique_ptr<InteractiveViewer> traj_viewer;
  bool have_traj = false;
  if (opts_.poses || opts_.poses2) {
    ViewerOpts topts;
    topts.colorby = ColorBy::RGB;
    topts.point_size = 2;
    traj_viewer = std::make_unique<InteractiveViewer>(topts);
    have_traj = true;
  }

  PointCloudIRGBPtr trajectory;
  if (have_traj) {
    try {
      trajectory = source.trajectory();
      if (!trajectory->empty()) {
        traj_viewer->show(trajectory, "Trajectory");
      }
    } catch (const std::exception& error) {
      spdlog::warn("player: failed to read poses: {}", error.what());
      have_traj = false;
      traj_viewer.reset();
    }
  }

  int frame = 0;
  for (std::size_t index = 0; index < source.size(); ++index) {
    auto loaded = source.load(index);
    const auto& f = loaded.path;
    const auto& cloud = loaded.cloud;
    viewer.show(cloud, "Frame " + std::to_string(frame));

    if (opts_.snapshot_prefix) {
      auto results = kpt::renderMultiView(cloud, opts_.render_opts);
      for (const auto& r : results) {
        const auto output =
            *opts_.snapshot_prefix + "_" + f.stem().string() + "_" +
            r.view_name + ".png";
        static_cast<void>(kpt::writeImageAtomic(output, r.image, true));
      }
    }

    int delay_ms = 1000 / std::max(1, opts_.fps);
    viewer.raw()->spinOnce(delay_ms);
    if (have_traj) traj_viewer->raw()->spinOnce(delay_ms);
    // No double wait: spinOnce(delay) + sleep(delay/2), not delay+delay.
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms / 2));
    ++frame;
  }

  // Only block on interactive spin when not in headless snapshot mode.
  if (!opts_.snapshot_prefix) {
    viewer.spin();
    if (have_traj) traj_viewer->spin();
  }
}

}  // namespace kpt
