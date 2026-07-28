#include "cli/legacy_gui_options.hpp"

#include <catch2/catch.hpp>

#include <array>
#include <string_view>
#include <vector>

namespace {

template <std::size_t Size>
auto viewer(const std::array<std::string_view, Size> &args) {
  return kpt::cli::parseViewerArgs(args);
}

template <std::size_t Size>
auto player(const std::array<std::string_view, Size> &args) {
  return kpt::cli::parsePlayerArgs(args);
}

} // namespace

TEST_CASE("legacy viewer parser preserves defaults and UTF-8 input",
          "[cli][viewer]") {
  const auto parsed = viewer(std::array<std::string_view, 1>{"资料/点云.pcd"});

  REQUIRE(parsed);
  CHECK(parsed.value->input_file_utf8 == "资料/点云.pcd");
  CHECK(parsed.value->log_level == 2);
  CHECK(parsed.value->style.color_by == kpt::ColorBy::Intensity);
  CHECK(parsed.value->style.point_size == 3.0F);
  CHECK(parsed.value->style.background ==
        std::array<float, 3>{0.0F, 0.0F, 0.0F});
}

TEST_CASE("legacy viewer parser maps short and long options", "[cli][viewer]") {
  const auto parsed = viewer(std::array<std::string_view, 9>{
      "-l", "3", "--colorby=rgb", "-s", "5", "--bg", "0.1, 0.2,1", "--",
      "-frame.bin"});

  REQUIRE(parsed);
  CHECK(parsed.value->input_file_utf8 == "-frame.bin");
  CHECK(parsed.value->log_level == 3);
  CHECK(parsed.value->style.color_by == kpt::ColorBy::RGB);
  CHECK(parsed.value->style.point_size == 5.0F);
  CHECK(parsed.value->style.background ==
        std::array<float, 3>{0.1F, 0.2F, 1.0F});
}

TEST_CASE("legacy parsers accept historical leading plus numbers",
          "[cli][viewer][player]") {
  const auto viewer_options = viewer(std::array<std::string_view, 5>{
      "--log-level", "+3", "--point-size", "+5", "frame.bin"});
  REQUIRE(viewer_options);
  CHECK(viewer_options.value->log_level == 3);
  CHECK(viewer_options.value->style.point_size == 5.0F);

  const auto player_options = player(
      std::array<std::string_view, 4>{"--input-dir", "frames", "--fps", "+10"});
  REQUIRE(player_options);
  CHECK(player_options.value->fps == 10);
}

TEST_CASE("legacy viewer help does not require input", "[cli][viewer]") {
  const auto parsed = viewer(std::array<std::string_view, 1>{"--help"});

  REQUIRE(parsed);
  CHECK(parsed.value->help);
  CHECK_FALSE(kpt::cli::viewerUsage().empty());
}

TEST_CASE("legacy viewer parser rejects malformed values", "[cli][viewer]") {
  const auto missing = viewer(std::array<std::string_view, 0>{});
  const auto extra =
      viewer(std::array<std::string_view, 2>{"one.bin", "two.bin"});
  const auto bad_log =
      viewer(std::array<std::string_view, 3>{"--log-level", "4", "one.bin"});
  const auto bad_color =
      viewer(std::array<std::string_view, 3>{"--colorby", "label", "one.bin"});
  const auto bad_size =
      viewer(std::array<std::string_view, 3>{"--point-size", "0", "one.bin"});
  const auto fractional_size =
      viewer(std::array<std::string_view, 3>{"--point-size", "1.5", "one.bin"});
  const auto bad_bg =
      viewer(std::array<std::string_view, 3>{"--bg", "0,2,0", "one.bin"});
  const auto unknown =
      viewer(std::array<std::string_view, 2>{"--wat", "one.bin"});

  CHECK_FALSE(missing);
  CHECK_FALSE(extra);
  CHECK_FALSE(bad_log);
  CHECK_FALSE(bad_color);
  CHECK_FALSE(bad_size);
  CHECK_FALSE(fractional_size);
  CHECK_FALSE(bad_bg);
  CHECK_FALSE(unknown);
  CHECK(unknown.error == "unknown option: --wat");
}

TEST_CASE("legacy player parser preserves interactive defaults",
          "[cli][player]") {
  const auto parsed =
      player(std::array<std::string_view, 2>{"--input-dir", "序列/帧"});

  REQUIRE(parsed);
  CHECK(parsed.value->input_dir_utf8 == "序列/帧");
  CHECK(parsed.value->glob == "*");
  CHECK(parsed.value->log_level == 2);
  CHECK(parsed.value->fps == 10);
  CHECK(parsed.value->style.color_by == kpt::ColorBy::Intensity);
  CHECK(parsed.value->style.point_size == 3.0F);
  CHECK_FALSE(parsed.value->snapshot);
}

TEST_CASE("legacy player parser maps sequence and display options",
          "[cli][player]") {
  const auto parsed = player(std::array<std::string_view, 16>{
      "-i", "序列", "-g", "*.pcd", "--label-dir", "标签", "--poses",
      "轨迹一.csv", "--poses2", "轨迹二.csv", "-c", "label", "-s", "7", "-f",
      "24"});

  REQUIRE(parsed);
  CHECK(parsed.value->input_dir_utf8 == "序列");
  CHECK(parsed.value->glob == "*.pcd");
  REQUIRE(parsed.value->label_dir_utf8);
  CHECK(*parsed.value->label_dir_utf8 == "标签");
  REQUIRE(parsed.value->poses_utf8);
  CHECK(*parsed.value->poses_utf8 == "轨迹一.csv");
  REQUIRE(parsed.value->poses2_utf8);
  CHECK(*parsed.value->poses2_utf8 == "轨迹二.csv");
  CHECK(parsed.value->style.color_by == kpt::ColorBy::Label);
  CHECK(parsed.value->style.point_size == 7.0F);
  CHECK(parsed.value->fps == 24);
}

TEST_CASE("legacy player parser preserves snapshot contract", "[cli][player]") {
  const auto parsed = player(std::array<std::string_view, 12>{
      "--input-dir", "frames", "--snapshot", "输出/前缀", "--snapshot-w", "800",
      "--snapshot-h", "600", "--snapshot-fov", "90", "--snapshot-views",
      "front, top,botleftfront"});

  REQUIRE(parsed);
  REQUIRE(parsed.value->snapshot);
  const auto &snapshot = *parsed.value->snapshot;
  CHECK(snapshot.output_prefix_utf8 == "输出/前缀");
  CHECK(snapshot.width == 800);
  CHECK(snapshot.height == 600);
  CHECK(snapshot.fov == 90.0F);
  CHECK(snapshot.overwrite);
  CHECK(snapshot.views == std::vector<kpt::View>{kpt::View::Front,
                                                 kpt::View::Top,
                                                 kpt::View::BotLeftFront});
}

TEST_CASE("legacy player snapshot all retains historical view order",
          "[cli][player]") {
  const auto parsed = player(std::array<std::string_view, 4>{
      "-i", "frames", "--snapshot", "out/frame"});

  REQUIRE(parsed);
  REQUIRE(parsed.value->snapshot);
  CHECK(parsed.value->snapshot->views ==
        std::vector<kpt::View>{
            kpt::View::Front, kpt::View::Right, kpt::View::Back,
            kpt::View::Left, kpt::View::Top, kpt::View::Bottom,
            kpt::View::TopRightFront, kpt::View::TopLeftFront,
            kpt::View::BotRightFront, kpt::View::BotLeftFront});
}

TEST_CASE("legacy player help does not require input", "[cli][player]") {
  const auto parsed = player(std::array<std::string_view, 1>{"-h"});

  REQUIRE(parsed);
  CHECK(parsed.value->help);
  CHECK_FALSE(kpt::cli::playerUsage().empty());
}

TEST_CASE("legacy player parser rejects unsafe values", "[cli][player]") {
  const auto missing = player(std::array<std::string_view, 0>{});
  const auto positional = player(std::array<std::string_view, 1>{"frames"});
  const auto bad_log = player(
      std::array<std::string_view, 4>{"-i", "frames", "--log-level", "-1"});
  const auto bad_color = player(
      std::array<std::string_view, 4>{"-i", "frames", "--colorby", "rainbow"});
  const auto bad_fps =
      player(std::array<std::string_view, 4>{"-i", "frames", "--fps", "0"});
  const auto excessive_fps =
      player(std::array<std::string_view, 4>{"-i", "frames", "--fps", "121"});
  const auto bad_width = player(
      std::array<std::string_view, 4>{"-i", "frames", "--snapshot-w", "0"});
  const auto bad_fov = player(
      std::array<std::string_view, 4>{"-i", "frames", "--snapshot-fov", "180"});
  const auto bad_view = player(std::array<std::string_view, 4>{
      "-i", "frames", "--snapshot-views", "front,sideways"});
  const auto unknown =
      player(std::array<std::string_view, 3>{"-i", "frames", "--wat"});
  const auto huge_png = player(std::array<std::string_view, 8>{
      "-i", "frames", "--snapshot", "out", "--snapshot-w", "715827882",
      "--snapshot-h", "2"});

  CHECK_FALSE(missing);
  CHECK_FALSE(positional);
  CHECK_FALSE(bad_log);
  CHECK_FALSE(bad_color);
  CHECK_FALSE(bad_fps);
  CHECK_FALSE(excessive_fps);
  CHECK_FALSE(bad_width);
  CHECK_FALSE(bad_fov);
  CHECK_FALSE(bad_view);
  CHECK_FALSE(unknown);
  CHECK(unknown.error == "unknown option: --wat");
  CHECK_FALSE(huge_png);
}
