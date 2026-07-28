#include "kpt/io/ply_codec.hpp"

#include <catch2/catch.hpp>

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

class TemporaryDirectory {
public:
  TemporaryDirectory() {
    static std::uint64_t next = 0;
    path = std::filesystem::temp_directory_path() /
           ("kpt-ply-codec-" + std::to_string(++next));
    std::filesystem::create_directories(path);
  }

  ~TemporaryDirectory() {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
  }

  TemporaryDirectory(const TemporaryDirectory &) = delete;
  TemporaryDirectory &operator=(const TemporaryDirectory &) = delete;

  std::filesystem::path path;
};

void writeText(const std::filesystem::path &path, std::string_view text) {
  std::ofstream output(path, std::ios::binary);
  REQUIRE(output);
  output.write(text.data(), static_cast<std::streamsize>(text.size()));
  REQUIRE(output);
}

template <typename T>
void appendEndian(std::vector<std::byte> &bytes, T value, bool big_endian) {
  static_assert(std::is_trivially_copyable_v<T>);
  auto encoded = std::bit_cast<std::array<std::byte, sizeof(T)>>(value);
  const bool host_big = std::endian::native == std::endian::big;
  if (host_big != big_endian) {
    for (std::size_t left = 0, right = encoded.size() - 1; left < right;
         ++left, --right)
      std::swap(encoded[left], encoded[right]);
  }
  bytes.insert(bytes.end(), encoded.begin(), encoded.end());
}

void writeBinary(const std::filesystem::path &path, std::string_view header,
                 const std::vector<std::byte> &payload) {
  std::ofstream output(path, std::ios::binary);
  REQUIRE(output);
  output.write(header.data(), static_cast<std::streamsize>(header.size()));
  output.write(reinterpret_cast<const char *>(payload.data()),
               static_cast<std::streamsize>(payload.size()));
  REQUIRE(output);
}

} // namespace

TEST_CASE("PLY ASCII reader handles generic elements, lists and reordered "
          "vertex properties",
          "[io][ply]") {
  TemporaryDirectory temporary;
  const auto input = temporary.path / "reordered-ascii.ply";
  writeText(input, "ply\r\n"
                   "format ascii 1.0\r\n"
                   "comment hand-written golden\r\n"
                   "element face 1\r\n"
                   "property list uchar int vertex_indices\r\n"
                   "element vertex 2\r\n"
                   "property uchar blue\r\n"
                   "property double z\r\n"
                   "property short red\r\n"
                   "property float x\r\n"
                   "property uchar green\r\n"
                   "property float y\r\n"
                   "property double intensity\r\n"
                   "property float confidence\r\n"
                   "end_header\r\n"
                   "3 0 1 1\r\n"
                   "30 3.5 10 1.25 20 2.25 0.75 99\r\n"
                   "3 -6.5 1 -4.25 2 -5.25 -0.5 42\r\n");

  kpt::PointCloudIRGB cloud;
  kpt::io_detail::loadPly(input, cloud);

  REQUIRE(cloud.size() == 2);
  CHECK(cloud.points[0].x == Approx(1.25F));
  CHECK(cloud.points[0].y == Approx(2.25F));
  CHECK(cloud.points[0].z == Approx(3.5F));
  CHECK(cloud.points[0].r == 10);
  CHECK(cloud.points[0].g == 20);
  CHECK(cloud.points[0].b == 30);
  CHECK(cloud.points[0].intensity == Approx(0.75F));
  CHECK(cloud.points[1].x == Approx(-4.25F));
  CHECK(cloud.points[1].intensity == Approx(-0.5F));
}

TEST_CASE("PLY binary big-endian reader handles aliases, type variation and "
          "vertex list properties",
          "[io][ply]") {
  TemporaryDirectory temporary;
  const auto input = temporary.path / "big-endian.ply";
  constexpr std::string_view header = "ply\n"
                                      "format binary_big_endian 1.0\n"
                                      "element material 1\n"
                                      "property uint id\n"
                                      "property list uchar float weights\n"
                                      "element vertex 1\n"
                                      "property double intensity\n"
                                      "property ushort g\n"
                                      "property float y\n"
                                      "property uchar b\n"
                                      "property double x\n"
                                      "property short r\n"
                                      "property float z\n"
                                      "property list uchar int neighbors\n"
                                      "end_header\n";
  std::vector<std::byte> payload;
  appendEndian(payload, std::uint32_t{7}, true);
  appendEndian(payload, std::uint8_t{2}, true);
  appendEndian(payload, 1.5F, true);
  appendEndian(payload, 2.5F, true);
  appendEndian(payload, 9.25, true);
  appendEndian(payload, std::uint16_t{22}, true);
  appendEndian(payload, -2.0F, true);
  appendEndian(payload, std::uint8_t{33}, true);
  appendEndian(payload, 1.0, true);
  appendEndian(payload, std::int16_t{11}, true);
  appendEndian(payload, 3.0F, true);
  appendEndian(payload, std::uint8_t{2}, true);
  appendEndian(payload, std::int32_t{4}, true);
  appendEndian(payload, std::int32_t{5}, true);
  writeBinary(input, header, payload);

  kpt::PointCloudIRGB cloud;
  kpt::io_detail::loadPly(input, cloud);

  REQUIRE(cloud.size() == 1);
  CHECK(cloud.points[0].x == Approx(1.0F));
  CHECK(cloud.points[0].y == Approx(-2.0F));
  CHECK(cloud.points[0].z == Approx(3.0F));
  CHECK(cloud.points[0].r == 11);
  CHECK(cloud.points[0].g == 22);
  CHECK(cloud.points[0].b == 33);
  CHECK(cloud.points[0].intensity == Approx(9.25F));
}

TEST_CASE("PLY writer emits binary little-endian data readable from Unicode "
          "paths",
          "[io][ply][unicode]") {
  TemporaryDirectory temporary;
  const auto output = temporary.path / std::filesystem::path(u8"中文点云.ply");
  kpt::PointCloudIRGB source;
  source.push_back({1.25F, -2.5F, 3.75F, 12, 34, 56, 0.625F});
  source.push_back({-4.0F, 5.0F, -6.0F, 78, 90, 123, -0.25F});

  kpt::io_detail::savePly(output, source);

  std::ifstream raw(output, std::ios::binary);
  REQUIRE(raw);
  const std::vector<char> encoded{std::istreambuf_iterator<char>(raw),
                                  std::istreambuf_iterator<char>()};
  const std::string encoded_text(encoded.begin(), encoded.end());
  CHECK(encoded_text.starts_with("ply\nformat binary_little_endian 1.0\n"));
  const auto payload_begin = encoded_text.find("end_header\n");
  REQUIRE(payload_begin != std::string::npos);
  const auto offset = payload_begin + std::string_view("end_header\n").size();
  std::vector<std::byte> expected_first_point;
  appendEndian(expected_first_point, source.points[0].x, false);
  appendEndian(expected_first_point, source.points[0].y, false);
  appendEndian(expected_first_point, source.points[0].z, false);
  appendEndian(expected_first_point, source.points[0].r, false);
  appendEndian(expected_first_point, source.points[0].g, false);
  appendEndian(expected_first_point, source.points[0].b, false);
  appendEndian(expected_first_point, source.points[0].intensity, false);
  REQUIRE(encoded.size() >= offset + expected_first_point.size());
  CHECK(std::memcmp(encoded.data() + static_cast<std::ptrdiff_t>(offset),
                    expected_first_point.data(),
                    expected_first_point.size()) == 0);

  kpt::PointCloudIRGB loaded;
  kpt::io_detail::loadPly(output, loaded);
  REQUIRE(loaded.size() == source.size());
  CHECK(loaded.points[0].x == Approx(source.points[0].x));
  CHECK(loaded.points[0].r == source.points[0].r);
  CHECK(loaded.points[0].intensity == Approx(source.points[0].intensity));
  CHECK(loaded.points[1].z == Approx(source.points[1].z));
  CHECK(loaded.points[1].b == source.points[1].b);
}

TEST_CASE("PLY reader rejects malformed and truncated inputs transactionally",
          "[io][ply]") {
  TemporaryDirectory temporary;

  const auto missing_coordinate = temporary.path / "missing-z.ply";
  writeText(missing_coordinate, "ply\n"
                                "format ascii 1.0\n"
                                "element vertex 1\n"
                                "property float x\n"
                                "property float y\n"
                                "end_header\n"
                                "1 2\n");
  kpt::PointCloudIRGB cloud;
  cloud.push_back({9, 8, 7, 6, 5, 4, 3});
  CHECK_THROWS(kpt::io_detail::loadPly(missing_coordinate, cloud));
  REQUIRE(cloud.size() == 1);
  CHECK(cloud.points[0].x == Approx(9));

  const auto truncated = temporary.path / "truncated.ply";
  constexpr std::string_view header = "ply\n"
                                      "format binary_little_endian 1.0\n"
                                      "element vertex 1\n"
                                      "property float x\n"
                                      "property float y\n"
                                      "property float z\n"
                                      "end_header\n";
  std::vector<std::byte> partial;
  appendEndian(partial, 1.0F, false);
  appendEndian(partial, 2.0F, false);
  writeBinary(truncated, header, partial);
  CHECK_THROWS(kpt::io_detail::loadPly(truncated, cloud));
  REQUIRE(cloud.size() == 1);

  const auto negative_list = temporary.path / "negative-list.ply";
  writeText(negative_list, "ply\n"
                           "format ascii 1.0\n"
                           "element face 1\n"
                           "property list char int vertices\n"
                           "element vertex 0\n"
                           "property float x\n"
                           "property float y\n"
                           "property float z\n"
                           "end_header\n"
                           "-1\n");
  CHECK_THROWS(kpt::io_detail::loadPly(negative_list, cloud));

  const auto overflow = temporary.path / "overflow-count.ply";
  writeText(overflow, "ply\n"
                      "format ascii 1.0\n"
                      "element vertex 184467440737095516160\n"
                      "property float x\n"
                      "property float y\n"
                      "property float z\n"
                      "end_header\n");
  CHECK_THROWS(kpt::io_detail::loadPly(overflow, cloud));
}

TEST_CASE("PLY reader bounds adversarial header and payload work",
          "[io][ply][limits]") {
  TemporaryDirectory temporary;
  kpt::PointCloudIRGB cloud;

  SECTION("header line length") {
    const auto input = temporary.path / "long-line.ply";
    std::string contents =
        "ply\nformat ascii 1.0\ncomment " + std::string(65'537, 'x');
    contents +=
        "\nelement vertex 0\nproperty float x\nproperty float y\nproperty "
        "float z\nend_header\n";
    writeText(input, contents);
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }

  SECTION("total header bytes") {
    const auto input = temporary.path / "large-header.ply";
    std::string contents = "ply\nformat ascii 1.0\n";
    for (int line = 0; line < 18; ++line)
      contents += "comment " + std::string(60'000, 'x') + "\n";
    contents +=
        "element vertex 0\nproperty float x\nproperty float y\nproperty "
        "float z\nend_header\n";
    writeText(input, contents);
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }

  SECTION("element count") {
    const auto input = temporary.path / "many-elements.ply";
    std::string contents = "ply\nformat ascii 1.0\n";
    for (int element = 0; element < 1025; ++element)
      contents += "element item" + std::to_string(element) + " 0\n";
    contents +=
        "element vertex 0\nproperty float x\nproperty float y\nproperty "
        "float z\nend_header\n";
    writeText(input, contents);
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }

  SECTION("properties per element") {
    const auto input = temporary.path / "many-properties.ply";
    std::string contents =
        "ply\nformat ascii 1.0\nelement vertex 0\nproperty float x\nproperty "
        "float y\nproperty float z\n";
    for (int property = 3; property < 1025; ++property)
      contents += "property float p" + std::to_string(property) + "\n";
    contents += "end_header\n";
    writeText(input, contents);
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }

  SECTION("total records") {
    const auto input = temporary.path / "many-records.ply";
    writeText(input, "ply\n"
                     "format ascii 1.0\n"
                     "element metadata 100000000\n"
                     "element vertex 1\n"
                     "property float x\n"
                     "property float y\n"
                     "property float z\n"
                     "end_header\n");
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }

  SECTION("vertex records") {
    const auto input = temporary.path / "many-vertices.ply";
    writeText(input, "ply\n"
                     "format ascii 1.0\n"
                     "element vertex 100000001\n"
                     "property float x\n"
                     "property float y\n"
                     "property float z\n"
                     "end_header\n");
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }

  SECTION("minimum scalar work") {
    const auto input = temporary.path / "scalar-budget.ply";
    writeText(input, "ply\n"
                     "format ascii 1.0\n"
                     "element metadata 100000000\n"
                     "property float a\n"
                     "property float b\n"
                     "property float c\n"
                     "property float d\n"
                     "property float e\n"
                     "property float f\n"
                     "element vertex 0\n"
                     "property float x\n"
                     "property float y\n"
                     "property float z\n"
                     "end_header\n");
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }

  SECTION("list items") {
    const auto input = temporary.path / "large-list.ply";
    writeText(input, "ply\n"
                     "format ascii 1.0\n"
                     "element face 1\n"
                     "property list uint int vertices\n"
                     "element vertex 0\n"
                     "property float x\n"
                     "property float y\n"
                     "property float z\n"
                     "end_header\n"
                     "100000001\n");
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }

  SECTION("ASCII token length") {
    const auto input = temporary.path / "long-token.ply";
    std::string contents = "ply\n"
                           "format ascii 1.0\n"
                           "element vertex 1\n"
                           "property float x\n"
                           "property float y\n"
                           "property float z\n"
                           "end_header\n";
    contents += std::string(257, '1') + " 2 3\n";
    writeText(input, contents);
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }
}

TEST_CASE("PLY reader rejects ambiguous fields, invalid scalar ranges and "
          "trailing payload",
          "[io][ply]") {
  TemporaryDirectory temporary;
  kpt::PointCloudIRGB cloud;

  SECTION("duplicate coordinate") {
    const auto input = temporary.path / "duplicate-x.ply";
    writeText(input, "ply\n"
                     "format ascii 1.0\n"
                     "element vertex 0\n"
                     "property float x\n"
                     "property float x\n"
                     "property float y\n"
                     "property float z\n"
                     "end_header\n");
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }

  SECTION("duplicate color alias") {
    const auto input = temporary.path / "duplicate-red.ply";
    writeText(input, "ply\n"
                     "format ascii 1.0\n"
                     "element vertex 0\n"
                     "property float x\n"
                     "property float y\n"
                     "property float z\n"
                     "property uchar red\n"
                     "property uchar r\n"
                     "end_header\n");
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }

  SECTION("float32 range applies to unknown properties") {
    const auto input = temporary.path / "float32-range.ply";
    writeText(input, "ply\n"
                     "format ascii 1.0\n"
                     "element vertex 1\n"
                     "property float x\n"
                     "property float y\n"
                     "property float z\n"
                     "property float confidence\n"
                     "end_header\n"
                     "1 2 3 1e100\n");
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }

  SECTION("fractional color") {
    const auto input = temporary.path / "fractional-red.ply";
    writeText(input, "ply\n"
                     "format ascii 1.0\n"
                     "element vertex 1\n"
                     "property float x\n"
                     "property float y\n"
                     "property float z\n"
                     "property float red\n"
                     "end_header\n"
                     "1 2 3 1.5\n");
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }

  SECTION("trailing ASCII token") {
    const auto input = temporary.path / "trailing-ascii.ply";
    writeText(input, "ply\n"
                     "format ascii 1.0\n"
                     "element vertex 1\n"
                     "property float x\n"
                     "property float y\n"
                     "property float z\n"
                     "end_header\n"
                     "1 2 3\n"
                     "unexpected\n");
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }

  SECTION("trailing binary byte") {
    const auto input = temporary.path / "trailing-binary.ply";
    constexpr std::string_view header = "ply\n"
                                        "format binary_little_endian 1.0\n"
                                        "element vertex 1\n"
                                        "property float x\n"
                                        "property float y\n"
                                        "property float z\n"
                                        "end_header\n";
    std::vector<std::byte> payload;
    appendEndian(payload, 1.0F, false);
    appendEndian(payload, 2.0F, false);
    appendEndian(payload, 3.0F, false);
    payload.push_back(std::byte{0x7f});
    writeBinary(input, header, payload);
    CHECK_THROWS(kpt::io_detail::loadPly(input, cloud));
  }
}
