#include "kpt/io/format.hpp"
#include "kpt/io/io.hpp"
#include <array>
#include <catch2/catch.hpp>
#include <filesystem>
#include <fstream>
#include <random>
#include <span>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

static const fs::path data_dir = "test/data";

fs::path uniqueTempPath(const fs::path &name) {
  static std::mt19937_64 generator(std::random_device{}());
  auto result = name.stem();
  result += "-" + std::to_string(generator());
  result += name.extension().native();
  return fs::temp_directory_path() / result;
}

TEST_CASE("load bin", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.bin");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].x == 1.0f);
  REQUIRE(cloud->points[0].intensity == 0.5f);
  REQUIRE(cloud->points[0].r == 0);
}

TEST_CASE("memory decode preserves format schema", "[io][memory]") {
  const auto check = [](const fs::path &path, bool color, bool intensity) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    REQUIRE(input);
    const auto size = static_cast<std::size_t>(input.tellg());
    std::vector<std::byte> bytes(size);
    input.seekg(0);
    input.read(reinterpret_cast<char *>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
    REQUIRE(input);
    const auto decoded = kpt::decode(bytes, path.filename().string());
    REQUIRE(decoded.cloud->size() == 3);
    REQUIRE(decoded.schema.has_color == color);
    REQUIRE(decoded.schema.has_intensity == intensity);
  };
  check(data_dir / "tiny.bin", false, true);
  check(data_dir / "tiny.xyz", false, false);
  check(data_dir / "tiny.xyzi", false, true);
  check(data_dir / "tiny.xyzrgb", true, false);
  check(data_dir / "tiny.xyzrgbi", true, true);
  const auto decodeText = [](std::string_view text, std::string_view name) {
    return kpt::decode(
        {reinterpret_cast<const std::byte *>(text.data()), text.size()}, name);
  };
  const auto pcd = decodeText(
      "VERSION 0.7\nFIELDS x y z rgb intensity\nSIZE 4 4 4 4 4\n"
      "TYPE F F F F F\nCOUNT 1 1 1 1 1\nWIDTH 1\nHEIGHT 1\n"
      "POINTS 1\nDATA ascii\n1 2 3 0 0\n",
      "memory.pcd");
  REQUIRE(pcd.schema.has_color);
  REQUIRE(pcd.schema.has_intensity);
  const auto ply = decodeText(
      "ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\n"
      "property float y\nproperty float z\nproperty uchar red\n"
      "property uchar green\nproperty uchar blue\nproperty float intensity\n"
      "end_header\n1 2 3 0 0 0 0\n",
      "memory.ply");
  REQUIRE(ply.schema.has_color);
  REQUIRE(ply.schema.has_intensity);
}

TEST_CASE("memory decode handles binary PCD stream", "[io][memory]") {
  const auto path = fs::path("data/000123.pcd");
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  REQUIRE(input);
  std::vector<std::byte> bytes(static_cast<std::size_t>(input.tellg()));
  input.seekg(0);
  input.read(reinterpret_cast<char *>(bytes.data()),
             static_cast<std::streamsize>(bytes.size()));
  REQUIRE(input);
  const auto decoded = kpt::decode(bytes, path.filename().string());
  REQUIRE(decoded.cloud->size() == 125980);
  REQUIRE_FALSE(decoded.schema.has_color);
  REQUIRE(decoded.schema.has_intensity);
}

TEST_CASE("native load observes cancellation", "[io]") {
  std::stop_source cancellation;
  cancellation.request_stop();
  REQUIRE_THROWS_WITH(
      kpt::load(data_dir / "tiny.bin", cancellation.get_token()),
      Catch::Contains("operation cancelled"));
}

TEST_CASE("native save observes cancellation before publication", "[io]") {
  const auto output = uniqueTempPath("kpt-cancelled-save.pcd");
  auto cloud = kpt::load(data_dir / "tiny.bin");
  std::stop_source cancellation;
  cancellation.request_stop();
  REQUIRE_THROWS_WITH(kpt::saveAtomic(output, *cloud, true, std::nullopt,
                                      cancellation.get_token()),
                      Catch::Contains("operation cancelled"));
  REQUIRE_FALSE(fs::exists(output));
}

TEST_CASE("load xyz auto-detect 3 col", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.xyz");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].x == 1.0f);
  REQUIRE(cloud->points[0].intensity == 0.0f);
}

TEST_CASE("load xyzi auto-detect 4 col", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.xyzi");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].intensity == 0.5f);
}

TEST_CASE("load xyzrgb auto-detect 6 col", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.xyzrgb");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].r == 255);
  REQUIRE(cloud->points[0].g == 0);
}

TEST_CASE("load xyzrgbi auto-detect 7 col", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.xyzrgbi");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].r == 255);
  REQUIRE(cloud->points[0].intensity == 0.5f);
}

TEST_CASE("detect unknown throws", "[io]") {
  REQUIRE_THROWS_AS(kpt::detect("foo.zzz"), std::runtime_error);
}

TEST_CASE("load missing file throws", "[io]") {
  REQUIRE_THROWS_AS(kpt::load("nope_xyz_does_not_exist.bin"),
                    std::runtime_error);
}

TEST_CASE("ascii load skips bad lines, keeps good ones", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny_bad.xyz");
  // 4 lines: 3 ok, 1 short (1 col), 1 odd (5 col) — wait, fixture: 3col ok /
  // 2col skip / 5col skip / 3col ok => 2 ok
  REQUIRE(cloud->size() == 2);
  REQUIRE(cloud->points[0].x == 1.0f);
  REQUIRE(cloud->points[1].x == 1.0f);
}

TEST_CASE("round-trip bin", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.bin");
  fs::path out = "build/rt.bin";
  kpt::save(out, *cloud);
  auto cloud2 = kpt::load(out);
  REQUIRE(cloud2->size() == cloud->size());
  REQUIRE(cloud2->points[1].intensity == 0.6f);
  fs::remove(out);
}

TEST_CASE("round-trip xyzrgbi explicit flavor", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.xyzrgbi");
  fs::path out = "build/rt.xyzrgbi";
  kpt::save(out, *cloud, kpt::Format::XYZRGBI);
  auto cloud2 = kpt::load(out);
  REQUIRE(cloud2->size() == cloud->size());
  REQUIRE(cloud2->points[0].x == Approx(cloud->points[0].x));
  REQUIRE(cloud2->points[0].y == Approx(cloud->points[0].y));
  REQUIRE(cloud2->points[0].z == Approx(cloud->points[0].z));
  REQUIRE(cloud2->points[0].r == cloud->points[0].r);
  REQUIRE(cloud2->points[0].g == cloud->points[0].g);
  REQUIRE(cloud2->points[0].b == cloud->points[0].b);
  REQUIRE(cloud2->points[0].intensity == Approx(cloud->points[0].intensity));
  fs::remove(out);
}

TEST_CASE("save bin drops rgb", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.xyzrgb");
  fs::path out = "build/drop.bin";
  kpt::save(out, *cloud);
  auto back = kpt::load(out);
  REQUIRE(back->points[0].r == 0); // rgb lost
  REQUIRE(back->points[0].x == 1.0f);
  fs::remove(out);
}

TEST_CASE("round-trip supports UTF-8 directories and filenames", "[io]") {
  const auto directory = uniqueTempPath("点云工具测试");
  const auto output = directory / "中文点云.xyz";
  fs::create_directories(directory);

  const auto cloud = kpt::load(data_dir / "tiny.xyz");
  kpt::save(output, *cloud);
  const auto loaded = kpt::load(output);
  REQUIRE(loaded->size() == cloud->size());

  std::error_code ignored;
  fs::remove_all(directory, ignored);
}

TEST_CASE("native Unicode paths round-trip through structured codecs",
          "[io][unicode]") {
  const auto directory = uniqueTempPath(fs::path(u8"点云工具-native-io"));
  fs::create_directories(directory);
  const auto cloud = kpt::load(data_dir / "tiny.xyzrgbi");

  for (const auto &filename :
       {fs::path(u8"中文点云.pcd"), fs::path(u8"中文点云.ply")}) {
    const auto output = directory / filename;
    kpt::save(output, *cloud);
    const auto loaded = kpt::load(output);
    REQUIRE(loaded->size() == cloud->size());
    REQUIRE(loaded->points[0].x == Approx(cloud->points[0].x));
    REQUIRE(loaded->points[0].intensity == Approx(cloud->points[0].intensity));
  }

  std::error_code ignored;
  fs::remove_all(directory, ignored);
}

TEST_CASE("text extension selects exact column schema", "[io][text]") {
  const auto path = uniqueTempPath("kpt-schema.xyzi");
  {
    std::ofstream output(path);
    output << "1 2 3\n";
    output << "4 5 6 0.75\n";
    output << "7 8 9 1.0 trailing\n";
  }
  const auto cloud = kpt::load(path);
  REQUIRE(cloud->size() == 1);
  REQUIRE(cloud->points[0].x == 4.0F);
  REQUIRE(cloud->points[0].intensity == Approx(0.75F));
  fs::remove(path);
}

TEST_CASE("all supported formats participate in conversion", "[io]") {
  kpt::PointCloudIRGB source;
  kpt::PointT point;
  point.x = 1.25F;
  point.y = -2.5F;
  point.z = 3.75F;
  point.r = 17;
  point.g = 34;
  point.b = 51;
  point.intensity = 0.625F;
  source.push_back(point);

  const auto directory = uniqueTempPath("kpt-native-conversion-matrix");
  fs::create_directories(directory);
  const std::array<std::pair<kpt::Format, const char *>, 7> formats{{
      {kpt::Format::Bin, "point.bin"},
      {kpt::Format::PCD, "point.pcd"},
      {kpt::Format::PLY, "point.ply"},
      {kpt::Format::XYZ, "point.xyz"},
      {kpt::Format::XYZI, "point.xyzi"},
      {kpt::Format::XYZRGB, "point.xyzrgb"},
      {kpt::Format::XYZRGBI, "point.xyzrgbi"},
  }};

  const auto keepsRgb = [](kpt::Format format) {
    return format == kpt::Format::PCD || format == kpt::Format::PLY ||
           format == kpt::Format::XYZRGB || format == kpt::Format::XYZRGBI;
  };
  const auto keepsIntensity = [](kpt::Format format) {
    return format == kpt::Format::Bin || format == kpt::Format::PCD ||
           format == kpt::Format::PLY || format == kpt::Format::XYZI ||
           format == kpt::Format::XYZRGBI;
  };

  for (const auto &[input_format, input_filename] : formats) {
    const auto input = directory / ("source-" + std::string(input_filename));
    kpt::save(input, source, input_format);
    const auto decoded_input = kpt::load(input);

    for (const auto &[output_format, output_filename] : formats) {
      const auto output =
          directory /
          ("from-" + std::to_string(static_cast<int>(input_format)) + "-" +
           std::string(output_filename));
      kpt::save(output, *decoded_input, output_format);
      const auto loaded = kpt::load(output);
      INFO(input_filename << " -> " << output_filename);
      REQUIRE(loaded->size() == 1);
      REQUIRE(loaded->points[0].x == Approx(point.x));
      REQUIRE(loaded->points[0].y == Approx(point.y));
      REQUIRE(loaded->points[0].z == Approx(point.z));
      REQUIRE(
          loaded->points[0].r ==
          (keepsRgb(input_format) && keepsRgb(output_format) ? point.r : 0));
      REQUIRE(
          loaded->points[0].intensity ==
          Approx(keepsIntensity(input_format) && keepsIntensity(output_format)
                     ? point.intensity
                     : 0.0F));
    }
  }

  std::error_code ignored;
  fs::remove_all(directory, ignored);
}

TEST_CASE("native readers reject inputs above resource limits", "[io]") {
  const auto long_text = uniqueTempPath("kpt-long-line.xyz");
  {
    std::ofstream output(long_text);
    output << std::string(64U * 1024U + 1U, '1');
  }
  REQUIRE_THROWS_WITH(kpt::load(long_text),
                      Catch::Contains("text line exceeds 64 KiB"));
  fs::remove(long_text);

  const auto carriage_return_text = uniqueTempPath("kpt-long-cr-line.xyz");
  {
    std::ofstream output(carriage_return_text, std::ios::binary);
    output << std::string(64U * 1024U + 1U, '\r');
  }
  REQUIRE_THROWS_WITH(kpt::load(carriage_return_text),
                      Catch::Contains("text line exceeds 64 KiB"));
  fs::remove(carriage_return_text);

  const auto embedded_carriage_return = uniqueTempPath("kpt-embedded-cr.xyz");
  {
    std::ofstream output(embedded_carriage_return, std::ios::binary);
    output << "1\r2 3\n";
  }
  const auto embedded = kpt::load(embedded_carriage_return);
  REQUIRE(embedded->size() == 1);
  REQUIRE(embedded->points[0].x == 1.0F);
  REQUIRE(embedded->points[0].y == 2.0F);
  REQUIRE(embedded->points[0].z == 3.0F);
  fs::remove(embedded_carriage_return);

  const auto oversized_bin = uniqueTempPath("kpt-oversized-sparse.bin");
  {
    std::ofstream output(oversized_bin, std::ios::binary);
    output.seekp(320000015);
    output.put('\0');
  }
  REQUIRE_THROWS_WITH(kpt::load(oversized_bin),
                      Catch::Contains("point count exceeds limit"));
  fs::remove(oversized_bin);
}

TEST_CASE("save rejects extension and explicit format disagreement", "[io]") {
  const auto output = uniqueTempPath("kpt-format-mismatch.pcd");
  {
    std::ofstream existing(output);
    existing << "preserve me";
  }
  kpt::PointCloudIRGB cloud;
  REQUIRE_THROWS_WITH(kpt::save(output, cloud, kpt::Format::XYZI),
                      Catch::Contains("does not match file extension"));
  std::ifstream preserved(output);
  std::string contents;
  std::getline(preserved, contents);
  REQUIRE(contents == "preserve me");
  fs::remove(output);
}
