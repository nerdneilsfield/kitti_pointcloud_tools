#include "kpt/player/player.hpp"

#include "kpt/io/io.hpp"
#include "kpt/viewer/viewer.hpp"
#include "kpt/label/label.hpp"
#include "kpt/render/render.hpp"

// Vendored rapidcsv.h is not sign-conversion-clean; suppress warnings while
// parsing it so the project's -Werror -Wsign-conversion does not fail the
// build on third-party code we do not own.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"
#include "rapidcsv.h"
#pragma GCC diagnostic pop

#include <spdlog/spdlog.h>

#include <fnmatch.h>

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>

namespace kpt {

namespace {

std::vector<std::filesystem::path> enumerate(const std::filesystem::path& dir,
                                             const std::string& glob) {
  std::vector<std::filesystem::path> files;
  for (const auto& e : std::filesystem::directory_iterator(dir)) {
    if (fnmatch(glob.c_str(), e.path().filename().c_str(), 0) == 0)
      files.push_back(e.path());
  }
  std::sort(files.begin(), files.end());
  return files;
}

// Read KITTI poses.txt (12 floats/row, row-major 3x4 [R|t]; translation at
// cols 3,7,11). Returns a trajectory cloud colored with the given RGB.
PointCloudIRGBPtr readPoses(const std::filesystem::path& poses_path,
                            uint8_t r, uint8_t g, uint8_t b) {
  auto cloud = std::make_shared<PointCloudIRGB>();
  try {
    rapidcsv::Document doc(poses_path.string(),
                           rapidcsv::LabelParams(-1, -1));
    for (size_t i = 0; i < doc.GetRowCount(); ++i) {
      auto row = doc.GetRow<float>(i);
      if (row.size() < 12) continue;
      PointT p;
      p.x = row[3];
      p.y = row[7];
      p.z = row[11];
      p.r = r; p.g = g; p.b = b;
      p.intensity = 1.0f;
      cloud->push_back(p);
    }
  } catch (const std::exception& e) {
    spdlog::warn("player: failed to read poses {}: {}", poses_path.string(),
                 e.what());
  }
  return cloud;
}

}  // namespace

SequencePlayer::SequencePlayer(const PlayerOpts& opts) : opts_(opts) {}

void SequencePlayer::run() {
  auto files = enumerate(opts_.input_dir, opts_.glob);
  spdlog::info("player: {} files in {}", files.size(),
               opts_.input_dir.string());
  if (files.empty()) return;

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

  auto lm = kpt::rangeNetLabelMap();
  auto rm = kpt::rgbLabelMap();

  int frame = 0;
  for (const auto& f : files) {
    auto cloud = kpt::load(f);
    if (opts_.label_dir) {
      auto label_path = *opts_.label_dir / (f.stem().string() + ".label");
      auto labels = kpt::loadLabel(label_path);
      cloud = kpt::applyLabel(cloud, labels, lm, rm);
    }
    viewer.show(cloud, "Frame " + std::to_string(frame));

    if (have_traj) {
      PointCloudIRGBPtr traj = std::make_shared<PointCloudIRGB>();
      if (opts_.poses) {
        auto t = readPoses(*opts_.poses, 255, 0, 0);
        *traj += *t;
      }
      if (opts_.poses2) {
        auto t = readPoses(*opts_.poses2, 0, 255, 0);
        *traj += *t;
      }
      if (!traj->empty()) {
        traj_viewer->show(traj, "Trajectory");
      }
    }

    if (opts_.snapshot_prefix) {
      auto results = kpt::renderMultiView(cloud, opts_.render_opts);
      for (const auto& r : results) {
        cv::imwrite(*opts_.snapshot_prefix + "_" + f.stem().string() + "_" +
                        r.view_name + ".png",
                    r.image);
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
