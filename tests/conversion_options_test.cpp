#include "kpt/io/conversion_options.hpp"

#include <catch2/catch.hpp>

TEST_CASE("ASCII flavor parser rejects unknown values", "[cli][convert]") {
  CHECK_FALSE(kpt::io::parseAsciiFlavor(""));
  CHECK(kpt::io::parseAsciiFlavor("xyz") == kpt::Format::XYZ);
  CHECK(kpt::io::parseAsciiFlavor("xyzrgbi") == kpt::Format::XYZRGBI);
  CHECK_THROWS_AS(kpt::io::parseAsciiFlavor("csv"), std::invalid_argument);
}

TEST_CASE("ASCII flavor must agree with output schema", "[cli][convert]") {
  const auto xyzi = kpt::io::parseAsciiFlavor("xyzi");
  CHECK_NOTHROW(kpt::io::validateAsciiFlavor("cloud.xyzi", xyzi));
  CHECK_NOTHROW(
      kpt::io::validateAsciiFlavor(kpt::Format::XYZI, xyzi));
  CHECK_THROWS_AS(kpt::io::validateAsciiFlavor("cloud.xyz", xyzi),
                  std::invalid_argument);
  CHECK_THROWS_AS(kpt::io::validateAsciiFlavor("cloud.pcd", xyzi),
                  std::invalid_argument);
  CHECK_THROWS_AS(kpt::io::validateAsciiFlavor("cloud.txt", xyzi),
                  std::invalid_argument);
  CHECK_THROWS_AS(
      kpt::io::validateAsciiFlavor(kpt::Format::PCD, xyzi),
      std::invalid_argument);
}

TEST_CASE("conversion log level has a closed range", "[cli][convert]") {
  CHECK_NOTHROW(kpt::io::validateLogLevel(0));
  CHECK_NOTHROW(kpt::io::validateLogLevel(3));
  CHECK_THROWS_AS(kpt::io::validateLogLevel(-1), std::invalid_argument);
  CHECK_THROWS_AS(kpt::io::validateLogLevel(4), std::invalid_argument);
}
