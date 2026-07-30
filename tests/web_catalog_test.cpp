#include <catch2/catch.hpp>

#include "gui/web/catalog.hpp"

using kpt::gui::web::validateCatalog;

TEST_CASE("web catalog sorts supported point clouds", "[web][catalog]") {
  const auto result = validateCatalog(
      {"/clouds/0007.xyzrgbi", "/clouds/0006.xyzrgb",
       "/clouds/0005.xyzi", "/clouds/0004.xyz", "/clouds/0003.ply",
       "/clouds/0002.pcd", "/clouds/0001.bin"},
      {});
  REQUIRE(result);
  REQUIRE(result.clouds[0].filename() == "0001.bin");
  REQUIRE(result.clouds[6].filename() == "0007.xyzrgbi");
}

TEST_CASE("web catalog rejects duplicate basenames", "[web][catalog]") {
  const auto result =
      validateCatalog({"/a/0001.pcd", "/b/0001.pcd"}, {});
  REQUIRE_FALSE(result);
  REQUIRE(result.error.find("Duplicate") != std::string::npos);

  const auto ambiguous =
      validateCatalog({"/a/0001.pcd", "/b/0001.bin"}, {});
  REQUIRE_FALSE(ambiguous);
  REQUIRE(ambiguous.error.find("stem") != std::string::npos);
}

TEST_CASE("web catalog requires one semantic label per frame",
          "[web][catalog]") {
  const auto missing =
      validateCatalog({"/clouds/0001.bin", "/clouds/0002.bin"},
                      {"/labels/0001.label"});
  REQUIRE_FALSE(missing);
  REQUIRE(missing.error.find("0002") != std::string::npos);

  const auto complete =
      validateCatalog({"/clouds/0002.bin", "/clouds/0001.bin"},
                      {"/labels/0001.label", "/labels/0002.label"});
  REQUIRE(complete);
}

TEST_CASE("web catalog rejects unsupported extensions", "[web][catalog]") {
  REQUIRE_FALSE(validateCatalog({"/clouds/0001.las"}, {}));
  REQUIRE_FALSE(
      validateCatalog({"/clouds/0001.bin"}, {"/labels/0001.txt"}));
}
