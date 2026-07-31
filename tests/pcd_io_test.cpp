#include "kpt/io/pcd_codec.hpp"

#include <catch2/catch.hpp>

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

struct TempFile {
  fs::path path;

  explicit TempFile(std::string_view name)
      : path([name] {
          const fs::path requested(name);
          auto filename = fs::path(
              "kpt-pcd-手写-fixture-" + requested.stem().string() + "-" +
              std::to_string(std::mt19937_64(std::random_device{}())()));
          filename += requested.extension().native();
          return fs::temp_directory_path() / filename;
        }()) {}

  ~TempFile() {
    std::error_code ignored;
    fs::remove(path, ignored);
  }
};

void appendU32(std::vector<char> &bytes, std::uint32_t value) {
  for (unsigned shift : {0U, 8U, 16U, 24U})
    bytes.push_back(static_cast<char>((value >> shift) & 0xffU));
}

void appendFloat(std::vector<char> &bytes, float value) {
  appendU32(bytes, std::bit_cast<std::uint32_t>(value));
}

void writeFixture(const fs::path &path, std::string_view header,
                  const std::vector<char> &body = {}) {
  std::ofstream output(path, std::ios::binary);
  REQUIRE(output);
  output.write(header.data(), static_cast<std::streamsize>(header.size()));
  output.write(body.data(), static_cast<std::streamsize>(body.size()));
  REQUIRE(output);
}

std::vector<char> literalLzf(const std::vector<char> &plain) {
  std::vector<char> compressed;
  for (std::size_t offset = 0; offset < plain.size();) {
    const auto count = std::min<std::size_t>(32, plain.size() - offset);
    compressed.push_back(static_cast<char>(count - 1));
    const auto first = plain.begin() + static_cast<std::ptrdiff_t>(offset);
    const auto last = first + static_cast<std::ptrdiff_t>(count);
    compressed.insert(compressed.end(), first, last);
    offset += count;
  }
  return compressed;
}

} // namespace

TEST_CASE("PCD ASCII maps reordered fields, COUNT and RGB aliases",
          "[io][pcd]") {
  STATIC_REQUIRE(sizeof(kpt::PointT) == 20);
  TempFile file("ascii.pcd");
  constexpr std::string_view fixture = "# independent golden fixture\n"
                                       "VERSION .7\n"
                                       "FIELDS z junk x rgb y red reflectance is_noise\n"
                                       "SIZE 4 2 4 4 4 1 4 1\n"
                                       "TYPE F U F U F U F U\n"
                                       "COUNT 1 2 1 1 1 1 1 1\n"
                                       "WIDTH 2\n"
                                       "HEIGHT 1\n"
                                       "POINTS 2\n"
                                       "DATA ascii\n"
                                       "3 10 11 1 66051 2 9 0.5 1\n"
                                       "6 12 13 4 263430 5 8 0.75 0\n";
  writeFixture(file.path, fixture);

  kpt::PointCloudIRGB cloud;
  kpt::io_detail::loadPcd(file.path, cloud);

  REQUIRE(cloud.size() == 2);
  REQUIRE(cloud.has_noise);
  CHECK(cloud.points[0].x == Approx(1.0F));
  CHECK(cloud.points[0].y == Approx(2.0F));
  CHECK(cloud.points[0].z == Approx(3.0F));
  CHECK(cloud.points[0].r == 9); // separate red overrides packed 1
  CHECK(cloud.points[0].g == 2);
  CHECK(cloud.points[0].b == 3);
  CHECK(cloud.points[0].intensity == Approx(0.5F));
  CHECK(cloud.points[0].noise == 1);
  CHECK(cloud.points[1].r == 8);
  CHECK(cloud.points[1].g == 5);
  CHECK(cloud.points[1].b == 6);
  CHECK(cloud.points[1].noise == 0);
}

TEST_CASE("PCD binary reads arbitrary field order and packed float RGB",
          "[io][pcd]") {
  TempFile file("binary.pcd");
  constexpr std::string_view header = "VERSION .7\n"
                                      "FIELDS intensity z rgb noise_class x y ignored\n"
                                      "SIZE 4 4 4 1 4 4 2\n"
                                      "TYPE F F F U F F U\n"
                                      "COUNT 1 1 1 1 1 1 2\n"
                                      "WIDTH 1\n"
                                      "HEIGHT 1\n"
                                      "POINTS 1\n"
                                      "DATA binary\n";
  std::vector<char> body;
  appendFloat(body, 0.25F);
  appendFloat(body, 3.0F);
  appendU32(body, 0x00112233U); // F32 field uses these bits, not its value
  body.push_back(3);
  appendFloat(body, 1.0F);
  appendFloat(body, 2.0F);
  body.insert(body.end(), {1, 0, 2, 0});
  writeFixture(file.path, header, body);

  kpt::PointCloudIRGB cloud;
  kpt::io_detail::loadPcd(file.path, cloud);

  REQUIRE(cloud.size() == 1);
  REQUIRE(cloud.has_noise);
  CHECK(cloud.points[0].x == Approx(1.0F));
  CHECK(cloud.points[0].y == Approx(2.0F));
  CHECK(cloud.points[0].z == Approx(3.0F));
  CHECK(cloud.points[0].r == 0x11);
  CHECK(cloud.points[0].g == 0x22);
  CHECK(cloud.points[0].b == 0x33);
  CHECK(cloud.points[0].intensity == Approx(0.25F));
  CHECK(cloud.points[0].noise == 3);
}

TEST_CASE("PCD binary_compressed decodes LZF field-major storage",
          "[io][pcd]") {
  TempFile file("compressed.pcd");
  constexpr std::string_view header = "VERSION .7\n"
                                      "FIELDS x y z rgb intensity noise\n"
                                      "SIZE 4 4 4 4 4 1\n"
                                      "TYPE F F F F F U\n"
                                      "COUNT 1 1 1 1 1 1\n"
                                      "WIDTH 2\n"
                                      "HEIGHT 1\n"
                                      "POINTS 2\n"
                                      "DATA binary_compressed\n";

  // PCD compressed bodies are structure-of-arrays: xx yy zz rgb-rgb ii nn.
  std::vector<char> plain;
  for (float value : {1.0F, 4.0F, 2.0F, 5.0F, 3.0F, 6.0F})
    appendFloat(plain, value);
  appendU32(plain, 0x00ff0000U);
  appendU32(plain, 0x0000ff00U);
  appendFloat(plain, 0.5F);
  appendFloat(plain, 0.75F);
  plain.push_back(0);
  plain.push_back(2);
  const auto compressed = literalLzf(plain);
  std::vector<char> body;
  appendU32(body, static_cast<std::uint32_t>(compressed.size()));
  appendU32(body, static_cast<std::uint32_t>(plain.size()));
  body.insert(body.end(), compressed.begin(), compressed.end());
  writeFixture(file.path, header, body);

  kpt::PointCloudIRGB cloud;
  kpt::io_detail::loadPcd(file.path, cloud);

  REQUIRE(cloud.size() == 2);
  REQUIRE(cloud.has_noise);
  CHECK(cloud.points[0].x == Approx(1.0F));
  CHECK(cloud.points[0].y == Approx(2.0F));
  CHECK(cloud.points[0].z == Approx(3.0F));
  CHECK(cloud.points[0].r == 255);
  CHECK(cloud.points[1].x == Approx(4.0F));
  CHECK(cloud.points[1].y == Approx(5.0F));
  CHECK(cloud.points[1].z == Approx(6.0F));
  CHECK(cloud.points[1].g == 255);
  CHECK(cloud.points[1].intensity == Approx(0.75F));
  CHECK(cloud.points[0].noise == 0);
  CHECK(cloud.points[1].noise == 2);
}

TEST_CASE("PCD LZF decoder handles overlapping back references", "[io][pcd]") {
  TempFile file("compressed-backref.pcd");
  constexpr std::string_view header = "VERSION .7\n"
                                      "FIELDS x y z\n"
                                      "SIZE 4 4 4\n"
                                      "TYPE F F F\n"
                                      "WIDTH 2\n"
                                      "HEIGHT 1\n"
                                      "POINTS 2\n"
                                      "DATA binary_compressed\n";
  std::vector<char> body;
  appendU32(body, 5);
  appendU32(body, 24);
  // One zero literal, then copy it 23 times using an overlapping offset-zero
  // reference. This sequence is hand-derived from the LZF wire format.
  body.insert(body.end(), {0, 0, static_cast<char>(0xe0), 14, 0});
  writeFixture(file.path, header, body);

  kpt::PointCloudIRGB cloud;
  kpt::io_detail::loadPcd(file.path, cloud);

  REQUIRE(cloud.size() == 2);
  CHECK(cloud.points[0].x == 0.0F);
  CHECK(cloud.points[1].z == 0.0F);
}

TEST_CASE("PCD rejects malformed schemas and truncated bodies", "[io][pcd]") {
  SECTION("count overflow") {
    TempFile file("overflow.pcd");
    constexpr std::string_view fixture = "VERSION .7\n"
                                         "FIELDS x y z\n"
                                         "SIZE 4 4 4\n"
                                         "TYPE F F F\n"
                                         "COUNT 1 1 18446744073709551615\n"
                                         "WIDTH 2\n"
                                         "HEIGHT 1\n"
                                         "POINTS 2\n"
                                         "DATA binary\n";
    writeFixture(file.path, fixture);
    kpt::PointCloudIRGB cloud;
    REQUIRE_THROWS_WITH(kpt::io_detail::loadPcd(file.path, cloud),
                        Catch::Matchers::Contains("COUNT"));
  }

  SECTION("missing coordinate") {
    TempFile file("missing-z.pcd");
    constexpr std::string_view fixture = "VERSION .7\n"
                                         "FIELDS x y\n"
                                         "SIZE 4 4\n"
                                         "TYPE F F\n"
                                         "WIDTH 0\n"
                                         "HEIGHT 1\n"
                                         "POINTS 0\n"
                                         "DATA ascii\n";
    writeFixture(file.path, fixture);
    kpt::PointCloudIRGB cloud;
    REQUIRE_THROWS_WITH(kpt::io_detail::loadPcd(file.path, cloud),
                        Catch::Matchers::Contains("x, y and z"));
  }

  SECTION("truncated binary") {
    TempFile file("truncated.pcd");
    constexpr std::string_view header = "VERSION .7\n"
                                        "FIELDS x y z\n"
                                        "SIZE 4 4 4\n"
                                        "TYPE F F F\n"
                                        "WIDTH 1\n"
                                        "HEIGHT 1\n"
                                        "POINTS 1\n"
                                        "DATA binary\n";
    std::vector<char> body;
    appendFloat(body, 1.0F);
    writeFixture(file.path, header, body);
    kpt::PointCloudIRGB cloud;
    cloud.has_noise = true;
    auto original = kpt::PointT{};
    original.noise = 7;
    cloud.push_back(original);
    REQUIRE_THROWS_WITH(kpt::io_detail::loadPcd(file.path, cloud),
                        Catch::Matchers::Contains("truncated"));
    REQUIRE(cloud.has_noise);
    REQUIRE(cloud.size() == 1);
    REQUIRE(cloud.points[0].noise == 7);
  }

  SECTION("noise must be unsigned byte") {
    TempFile file("invalid-noise.pcd");
    constexpr std::string_view fixture = "VERSION .7\n"
                                         "FIELDS x y z noise\n"
                                         "SIZE 4 4 4 1\n"
                                         "TYPE F F F I\n"
                                         "WIDTH 0\n"
                                         "HEIGHT 1\n"
                                         "POINTS 0\n"
                                         "DATA ascii\n";
    writeFixture(file.path, fixture);
    kpt::PointCloudIRGB cloud;
    REQUIRE_THROWS_WITH(kpt::io_detail::loadPcd(file.path, cloud),
                        Catch::Matchers::Contains("noise must be U8"));
  }

  SECTION("noise aliases are unique") {
    TempFile file("duplicate-noise.pcd");
    constexpr std::string_view fixture = "VERSION .7\n"
                                         "FIELDS x y z noise is_noise\n"
                                         "SIZE 4 4 4 1 1\n"
                                         "TYPE F F F U U\n"
                                         "WIDTH 0\n"
                                         "HEIGHT 1\n"
                                         "POINTS 0\n"
                                         "DATA ascii\n";
    writeFixture(file.path, fixture);
    kpt::PointCloudIRGB cloud;
    REQUIRE_THROWS_WITH(kpt::io_detail::loadPcd(file.path, cloud),
                        Catch::Matchers::Contains("duplicate mapped field noise"));
  }
}

TEST_CASE("PCD writer emits portable little-endian binary", "[io][pcd]") {
  TempFile file("writer-中文.pcd");
  kpt::PointCloudIRGB cloud;
  cloud.push_back(
      kpt::PointT{1.0F, 2.0F, 3.0F, 0x12, 0x34, 0x56, 0, 0.5F});

  kpt::io_detail::savePcd(file.path, cloud);

  std::ifstream input(file.path, std::ios::binary);
  REQUIRE(input);
  input.seekg(0, std::ios::end);
  const auto length = input.tellg();
  REQUIRE(length >= 0);
  input.seekg(0, std::ios::beg);
  std::string all(static_cast<std::size_t>(length), '\0');
  input.read(all.data(), static_cast<std::streamsize>(all.size()));
  REQUIRE(input);
  const auto marker = all.find("DATA binary\n");
  REQUIRE(marker != std::string::npos);
  const auto body = marker + std::string("DATA binary\n").size();
  REQUIRE(all.size() == body + 20);
  CHECK(static_cast<unsigned char>(all[body + 12]) == 0x56);
  CHECK(static_cast<unsigned char>(all[body + 13]) == 0x34);
  CHECK(static_cast<unsigned char>(all[body + 14]) == 0x12);
  CHECK(static_cast<unsigned char>(all[body + 15]) == 0x00);

  kpt::PointCloudIRGB loaded;
  kpt::io_detail::loadPcd(file.path, loaded);
  REQUIRE(loaded.size() == 1);
  CHECK(loaded.points[0].x == Approx(1.0F));
  CHECK(loaded.points[0].r == 0x12);
  CHECK(loaded.points[0].g == 0x34);
  CHECK(loaded.points[0].b == 0x56);
  CHECK(loaded.points[0].intensity == Approx(0.5F));
}

TEST_CASE("PCD writer preserves optional noise labels", "[io][pcd]") {
  TempFile file("writer-noise.pcd");
  kpt::PointCloudIRGB cloud;
  cloud.has_noise = true;
  auto valid = kpt::PointT{1.0F, 2.0F, 3.0F};
  valid.noise = 0;
  cloud.push_back(valid);
  auto noise = kpt::PointT{4.0F, 5.0F, 6.0F};
  noise.noise = 255;
  cloud.push_back(noise);

  kpt::io_detail::savePcd(file.path, cloud);

  std::ifstream input(file.path, std::ios::binary);
  REQUIRE(input);
  input.seekg(0, std::ios::end);
  const auto length = input.tellg();
  REQUIRE(length >= 0);
  input.seekg(0, std::ios::beg);
  std::string all(static_cast<std::size_t>(length), '\0');
  input.read(all.data(), static_cast<std::streamsize>(all.size()));
  REQUIRE(input);
  CHECK(all.find("FIELDS x y z rgb intensity noise\n") != std::string::npos);
  CHECK(all.find("SIZE 4 4 4 4 4 1\n") != std::string::npos);
  CHECK(all.find("TYPE F F F F F U\n") != std::string::npos);

  kpt::PointCloudIRGB loaded;
  kpt::io_detail::loadPcd(file.path, loaded);
  REQUIRE(loaded.has_noise);
  REQUIRE(loaded.size() == 2);
  CHECK(loaded.points[0].noise == 0);
  CHECK(loaded.points[1].noise == 255);
}

TEST_CASE("PCD accepts repository corpus with trailing binary bytes",
          "[io][pcd][compat]") {
  kpt::PointCloudIRGB cloud;
  REQUIRE_NOTHROW(kpt::io_detail::loadPcd("data/000123.pcd", cloud));
  REQUIRE(cloud.size() == 125980);
  CHECK(cloud.width == 125980);
  CHECK(cloud.height == 1);
}

TEST_CASE("PCD preserves organized shape and viewpoint", "[io][pcd]") {
  TempFile input_file("metadata.pcd");
  constexpr std::string_view fixture = "VERSION .7\r\n"
                                       "FIELDS x y z\r\n"
                                       "SIZE 4 4 4\r\n"
                                       "TYPE F F F\r\n"
                                       "WIDTH 2\r\n"
                                       "HEIGHT 2\r\n"
                                       "VIEWPOINT 1 2 3 0.5 0.1 0.2 0.3\r\n"
                                       "POINTS 4\r\n"
                                       "DATA ascii\r\n"
                                       "0 0 0\r\n1 1 1\r\n2 2 2\r\n3 3 3\r\n";
  writeFixture(input_file.path, fixture);

  kpt::PointCloudIRGB cloud;
  kpt::io_detail::loadPcd(input_file.path, cloud);
  REQUIRE(cloud.size() == 4);
  CHECK(cloud.width == 2);
  CHECK(cloud.height == 2);
  CHECK(cloud.viewpoint[0] == Approx(1.0F));
  CHECK(cloud.viewpoint[3] == Approx(0.5F));

  TempFile output_file("metadata-roundtrip.pcd");
  kpt::io_detail::savePcd(output_file.path, cloud);
  kpt::PointCloudIRGB loaded;
  kpt::io_detail::loadPcd(output_file.path, loaded);
  CHECK(loaded.width == 2);
  CHECK(loaded.height == 2);
  CHECK(loaded.viewpoint == cloud.viewpoint);
}

TEST_CASE("PCD ASCII supports packed F32 RGB, default COUNT and NaN",
          "[io][pcd]") {
  TempFile file("ascii-packed-float.pcd");
  std::ofstream output(file.path, std::ios::binary);
  REQUIRE(output);
  output.imbue(std::locale::classic());
  output << "VERSION .7\n"
            "FIELDS x y z rgb\n"
            "SIZE 4 4 4 4\n"
            "TYPE F F F F\n"
            "WIDTH 1\n"
            "HEIGHT 1\n"
            "POINTS 1\n"
            "DATA ascii\n"
         << "nan 2 3 "
         << std::setprecision(std::numeric_limits<float>::max_digits10)
         << std::bit_cast<float>(std::uint32_t{0x00aabbcc}) << '\n';
  output.close();

  kpt::PointCloudIRGB cloud;
  kpt::io_detail::loadPcd(file.path, cloud);
  REQUIRE(cloud.size() == 1);
  CHECK(std::isnan(cloud.points[0].x));
  CHECK(cloud.points[0].r == 0xaa);
  CHECK(cloud.points[0].g == 0xbb);
  CHECK(cloud.points[0].b == 0xcc);
}

TEST_CASE("PCD handles empty compressed clouds", "[io][pcd]") {
  TempFile file("empty-compressed.pcd");
  constexpr std::string_view header = "VERSION .7\n"
                                      "FIELDS x y z\n"
                                      "SIZE 4 4 4\n"
                                      "TYPE F F F\n"
                                      "WIDTH 0\n"
                                      "HEIGHT 1\n"
                                      "POINTS 0\n"
                                      "DATA binary_compressed\n";
  std::vector<char> body;
  appendU32(body, 0);
  appendU32(body, 0);
  writeFixture(file.path, header, body);

  kpt::PointCloudIRGB cloud;
  REQUIRE_NOTHROW(kpt::io_detail::loadPcd(file.path, cloud));
  CHECK(cloud.empty());
  CHECK(cloud.width == 0);
}

TEST_CASE("PCD bounds directives and ASCII tokens", "[io][pcd][security]") {
  SECTION("duplicate directive") {
    TempFile file("duplicate-version.pcd");
    constexpr std::string_view fixture = "VERSION .7\n"
                                         "VERSION .7\n"
                                         "FIELDS x y z\n"
                                         "SIZE 4 4 4\n"
                                         "TYPE F F F\n"
                                         "WIDTH 0\n"
                                         "HEIGHT 1\n"
                                         "POINTS 0\n"
                                         "DATA ascii\n";
    writeFixture(file.path, fixture);
    kpt::PointCloudIRGB cloud;
    REQUIRE_THROWS_WITH(kpt::io_detail::loadPcd(file.path, cloud),
                        Catch::Matchers::Contains("duplicate"));
  }

  SECTION("overlong ASCII token") {
    TempFile file("long-token.pcd");
    const std::string fixture = "VERSION .7\n"
                                "FIELDS x y z\n"
                                "SIZE 4 4 4\n"
                                "TYPE F F F\n"
                                "WIDTH 1\n"
                                "HEIGHT 1\n"
                                "POINTS 1\n"
                                "DATA ascii\n" +
                                std::string(257, '1') + " 2 3\n";
    writeFixture(file.path, fixture);
    kpt::PointCloudIRGB cloud;
    REQUIRE_THROWS_WITH(kpt::io_detail::loadPcd(file.path, cloud),
                        Catch::Matchers::Contains("256 bytes"));
  }

  SECTION("compressed size cannot dwarf output") {
    TempFile file("compressed-bound.pcd");
    constexpr std::string_view header = "VERSION .7\n"
                                        "FIELDS x y z\n"
                                        "SIZE 4 4 4\n"
                                        "TYPE F F F\n"
                                        "WIDTH 1\n"
                                        "HEIGHT 1\n"
                                        "POINTS 1\n"
                                        "DATA binary_compressed\n";
    std::vector<char> body;
    appendU32(body, 100);
    appendU32(body, 12);
    body.resize(body.size() + 100);
    writeFixture(file.path, header, body);
    kpt::PointCloudIRGB cloud;
    REQUIRE_THROWS_WITH(kpt::io_detail::loadPcd(file.path, cloud),
                        Catch::Matchers::Contains("LZF bound"));
  }
}

TEST_CASE("PCD rejects F64 values outside float range", "[io][pcd][security]") {
  TempFile file("f64-range.pcd");
  constexpr std::string_view fixture = "VERSION .7\n"
                                       "FIELDS x y z\n"
                                       "SIZE 8 8 8\n"
                                       "TYPE F F F\n"
                                       "COUNT 1 1 1\n"
                                       "WIDTH 1\n"
                                       "HEIGHT 1\n"
                                       "POINTS 1\n"
                                       "DATA ascii\n"
                                       "1e300 0 0\n";
  writeFixture(file.path, fixture);
  kpt::PointCloudIRGB cloud;
  REQUIRE_THROWS_WITH(kpt::io_detail::loadPcd(file.path, cloud),
                      Catch::Contains("invalid point value in x"));
}

TEST_CASE("PointCloud self append is defined and flattens metadata",
          "[types]") {
  kpt::PointCloudIRGB cloud;
  cloud.push_back(kpt::PointT{1.0F, 2.0F, 3.0F});
  cloud.width = 1;
  cloud.height = 1;
  cloud += cloud;
  REQUIRE(cloud.size() == 2);
  CHECK(cloud.points[1].x == 1.0F);
  CHECK(cloud.width == 2);
  CHECK(cloud.height == 1);
}
