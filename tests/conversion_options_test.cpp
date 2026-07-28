#include "cli/conversion_options.hpp"

#include <catch2/catch.hpp>

TEST_CASE("ASCII flavor parser rejects unknown values", "[cli][convert]") {
  CHECK_FALSE(kpt::cli::parseAsciiFlavor(""));
  CHECK(kpt::cli::parseAsciiFlavor("xyz") == kpt::Format::XYZ);
  CHECK(kpt::cli::parseAsciiFlavor("xyzrgbi") == kpt::Format::XYZRGBI);
  CHECK_THROWS_AS(kpt::cli::parseAsciiFlavor("csv"), std::invalid_argument);
}

TEST_CASE("ASCII flavor must agree with output schema", "[cli][convert]") {
  const auto xyzi = kpt::cli::parseAsciiFlavor("xyzi");
  CHECK_NOTHROW(kpt::cli::validateAsciiFlavor("cloud.xyzi", xyzi));
  CHECK_NOTHROW(
      kpt::cli::validateAsciiFlavor(kpt::Format::XYZI, xyzi));
  CHECK_THROWS_AS(kpt::cli::validateAsciiFlavor("cloud.xyz", xyzi),
                  std::invalid_argument);
  CHECK_THROWS_AS(kpt::cli::validateAsciiFlavor("cloud.pcd", xyzi),
                  std::invalid_argument);
  CHECK_THROWS_AS(kpt::cli::validateAsciiFlavor("cloud.txt", xyzi),
                  std::invalid_argument);
  CHECK_THROWS_AS(
      kpt::cli::validateAsciiFlavor(kpt::Format::PCD, xyzi),
      std::invalid_argument);
}

TEST_CASE("conversion log level has a closed range", "[cli][convert]") {
  CHECK_NOTHROW(kpt::cli::validateLogLevel(0));
  CHECK_NOTHROW(kpt::cli::validateLogLevel(3));
  CHECK_THROWS_AS(kpt::cli::validateLogLevel(-1), std::invalid_argument);
  CHECK_THROWS_AS(kpt::cli::validateLogLevel(4), std::invalid_argument);
}
