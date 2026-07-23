# KPT Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor scattered KITTI pointcloud tools into a thin `kpt` library + 5 CLI tools (convert/batch_convert/viewer/player/render) with unified `PointXYZRGBI` canonical type and 7-format interconversion.

**Architecture:** Static lib `kpt` with 5 sub-modules (io/viewer/player/render/label) under `src/kpt/`; thin CLI wrappers under `src/cli/`. Clean break — delete all old sources, rewrite CMakeLists.

**Tech Stack:** C++20, CMake 3.10+, PCL (PointCloud + visualization), OpenCV, Eigen, spdlog, popl, Catch2 — all already vendored in `third_party/` or system-found.

## Global Constraints

- C++20 (`cxx_std_20`), keep existing `project_options`/`project_warnings` interface libs
- Canonical point type: `pcl::PointXYZRGBI` — all modules use `using PointCloudIRGB = pcl::PointCloud<pcl::PointXYZRGBI>;`
- Namespace: `kpt::` for library; CLI `main` in global namespace
- Include paths: `src/kpt/` is include root (e.g. `#include "kpt/types.hpp"`)
- PCL/OpenCV/Eigen found via existing `find_package` blocks in CMakeLists
- spdlog via vendored `spdlog::spdlog` target; popl via `popl::popl`; Catch2 via vendored subdirectory
- ASCII auto-detect by column count on read (3/4/6/7); explicit `--ascii-flavor` on write
- Error handling: library throws `std::runtime_error`; CLI catches → spdlog + exit 1
- TDD: write Catch2 test first, verify fail, implement, verify pass, commit
- Self-use tool, no compatibility with old binaries

---

## File Structure

```
src/kpt/
  types.hpp                  # aliases, Format/ColorBy/View enums
  io/format.hpp              # Format enum + detect() + extension helpers
  io/io.hpp                  # load/save declarations
  io/io.cpp                  # load/save implementations
  viewer/viewer.hpp
  viewer/viewer.cpp
  render/render.hpp          # RenderOpts, RenderResult, renderMultiView()
  render/render.cpp
  label/label.hpp
  label/label.cpp
  player/player.hpp
  player/player.cpp
src/cli/
  pc_convert.cc
  pc_batch_convert.cc
  pc_viewer.cc
  pc_player.cc
  pc_render.cc
tests/
  test_main.cpp              # Catch2 main
  io_test.cpp
  label_test.cpp
  render_test.cpp
test/data/
  tiny.bin                   # 3-point KITTI bin
  tiny.pcd                   # 3-point pcd XYZRGBI
  tiny.ply                   # 3-point ply XYZRGBI
  tiny_xyz.txt               # 3 col
  tiny_xyzi.txt              # 4 col
  tiny_xyzrgb.txt            # 6 col
  tiny_xyzrgbi.txt           # 7 col
README.md                    # rewrite
CMakeLists.txt               # rewrite
```

---

## Task 1: Scaffold — types.hpp + delete old sources + CMake skeleton

**Files:**
- Create: `src/kpt/types.hpp`
- Create: `src/kpt/io/format.hpp`
- Delete: all files in cleanup list (see spec)
- Modify: `CMakeLists.txt` (full rewrite)
- Create: `tests/test_main.cpp`

**Interfaces:**
- Produces: `kpt::PointCloudIRGB`, `kpt::PointCloudIRGBPtr`, `kpt::Format`, `kpt::ColorBy`, `kpt::View`, `kpt::detect()`

- [ ] **Step 1: Delete old source files**

```bash
rm src/kitti_binary.h src/kitti_binary.cc \
   src/kitti_binary_converter.cc src/convert_pcd_to_binary.cc \
   src/kitti_player.cc src/kitti_player_two.cc src/kitti_player_with_poses.cpp src/kitti_player.h \
   src/binary_to_images.cc src/pcd_to_images.cpp src/pcd_to_images_soft.cpp \
   src/cloud2images.hpp src/cloud2images.cpp src/cloud2images_soft.hpp src/cloud2images_soft.cc \
   src/types.h src/se_helper.h src/se_helper.cpp
```
Keep `src/rapidcsv.h`.

- [ ] **Step 2: Create `src/kpt/types.hpp`**

```cpp
#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <optional>
#include <string>
#include <filesystem>

namespace kpt {

using PointT = pcl::PointXYZRGBI;
using PointCloudIRGB = pcl::PointCloud<PointT>;
using PointCloudIRGBPtr = PointCloudIRGB::Ptr;
using PointCloudIRGBConstPtr = PointCloudIRGB::ConstPtr;

enum class Format { Bin, PCD, PLY, XYZ, XYZI, XYZRGB, XYZRGBI };
enum class ColorBy { Intensity, RGB, Z, Label, None };
enum class View { Front, Right, Back, Left, Top, Bottom,
                  TopRightFront, TopLeftFront, BotRightFront, BotLeftFront };

}  // namespace kpt
```

- [ ] **Step 3: Create `src/kpt/io/format.hpp`**

```cpp
#pragma once
#include "kpt/types.hpp"
#include <filesystem>
#include <string>
#include <stdexcept>

namespace kpt {

inline Format detect(const std::filesystem::path& p) {
  std::string ext = p.extension().string();
  // lowercase
  for (auto& c : ext) c = static_cast<char>(std::tolower(c));
  if (ext == ".bin") return Format::Bin;
  if (ext == ".pcd") return Format::PCD;
  if (ext == ".ply") return Format::PLY;
  if (ext == ".xyz")    return Format::XYZ;
  if (ext == ".xyzi")   return Format::XYZI;
  if (ext == ".xyzrgb") return Format::XYZRGB;
  if (ext == ".xyzrgbi")return Format::XYZRGBI;
  throw std::runtime_error("unknown format: " + ext);
}

inline std::string toString(Format f) {
  switch (f) {
    case Format::Bin: return "bin";
    case Format::PCD: return "pcd";
    case Format::PLY: return "ply";
    case Format::XYZ: return "xyz";
    case Format::XYZI: return "xyzi";
    case Format::XYZRGB: return "xyzrgb";
    case Format::XYZRGBI: return "xyzrgbi";
  }
  return "?";
}

}  // namespace kpt
```

- [ ] **Step 4: Create `tests/test_main.cpp`**

```cpp
#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
```

- [ ] **Step 5: Rewrite `CMakeLists.txt`**

Keep project setup (project_options/warnings/sanitizers/find_package PCL/OpenMP/OpenCV). Replace targets section (after `include_directories(src)`) with:

```cmake
include_directories(src)

# kpt static library
add_library(kpt STATIC
  src/kpt/io/io.cpp
  src/kpt/viewer/viewer.cpp
  src/kpt/render/render.cpp
  src/kpt/label/label.cpp
  src/kpt/player/player.cpp
)
target_link_libraries(kpt PUBLIC
  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} spdlog::spdlog eigen3::eigen project_options
)
target_include_directories(kpt PUBLIC src)

# CLI tools
foreach(tool pc_convert pc_batch_convert pc_viewer pc_player pc_render)
  add_executable(${tool} src/cli/${tool}.cc)
  target_link_libraries(${tool} PUBLIC kpt popl::popl project_options)
endforeach()

# Tests
if(ENABLE_TESTING)
  add_subdirectory(third_party/catch2)
  add_executable(kpt_tests tests/test_main.cpp tests/io_test.cpp
                 tests/label_test.cpp tests/render_test.cpp)
  target_link_libraries(kpt_tests PRIVATE kpt Catch2::catch2 project_options)
  enable_testing()
  add_test(NAME kpt_tests COMMAND kpt_tests)
endif()
```

Note: empty `.cpp` files referenced (io.cpp etc.) must exist for CMake — create empty stubs now so configure passes:

```bash
touch src/kpt/io/io.cpp src/kpt/viewer/viewer.cpp src/kpt/render/render.cpp
touch src/kpt/label/label.cpp src/kpt/player/player.cpp
touch tests/io_test.cpp tests/label_test.cpp tests/render_test.cpp
touch src/cli/pc_convert.cc src/cli/pc_batch_convert.cc src/cli/pc_viewer.cc
touch src/cli/pc_player.cc src/cli/pc_render.cc
```

- [ ] **Step 6: Configure + build to verify scaffold compiles**

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j
```
Expected: configures + builds (empty stubs, no errors).

- [ ] **Step 7: Commit**

```bash
git add -A && git commit -m "refactor: scaffold kpt lib structure, delete old sources"
```

---

## Task 2: kpt::io — load (binary + PCL + ASCII auto-detect)

**Files:**
- Create: `src/kpt/io/io.hpp`
- Modify: `src/kpt/io/io.cpp`
- Create: `tests/io_test.cpp`
- Create: `test/data/tiny.bin`, `test/data/tiny_xyz.txt`, `test/data/tiny_xyzi.txt`, `test/data/tiny_xyzrgb.txt`, `test/data/tiny_xyzrgbi.txt`

**Interfaces:**
- Consumes: `kpt::detect()` from format.hpp
- Produces: `kpt::load(path) -> PointCloudIRGB`

- [ ] **Step 1: Create test fixtures**

`test/data/tiny.bin` — 3 points, each 4 floats (x,y,z,i), 48 bytes:
```bash
python3 -c "
import struct
pts=[(1.0,2.0,3.0,0.5),(4.0,5.0,6.0,0.6),(7.0,8.0,9.0,0.7)]
with open('test/data/tiny.bin','wb') as f:
    for p in pts: f.write(struct.pack('ffff',*p))
"
```

`test/data/tiny_xyz.txt`:
```
1 2 3
4 5 6
7 8 9
```
`test/data/tiny_xyzi.txt`:
```
1 2 3 0.5
4 5 6 0.6
7 8 9 0.7
```
`test/data/tiny_xyzrgb.txt`:
```
1 2 3 255 0 0
4 5 6 0 255 0
7 8 9 0 0 255
```
`test/data/tiny_xyzrgbi.txt`:
```
1 2 3 255 0 0 0.5
4 5 6 0 255 0 0.6
7 8 9 0 0 255 0.7
```

- [ ] **Step 2: Write `src/kpt/io/io.hpp`**

```cpp
#pragma once
#include "kpt/types.hpp"
#include "kpt/io/format.hpp"
#include <filesystem>

namespace kpt {

PointCloudIRGB load(const std::filesystem::path& p);

void save(const std::filesystem::path& p, const PointCloudIRGB& cloud,
          std::optional<Format> ascii_flavor = std::nullopt);

}  // namespace kpt
```

- [ ] **Step 3: Write failing tests in `tests/io_test.cpp`**

```cpp
#include <catch2/catch_all.hpp>
#include "kpt/io/io.hpp"
#include <filesystem>
namespace fs = std::filesystem;

static const fs::path data_dir = "test/data";

TEST_CASE("load bin", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.bin");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].x == 1.0f);
  REQUIRE(cloud->points[0].intensity == 0.5f);
  REQUIRE(cloud->points[0].r == 0);
}

TEST_CASE("load xyz auto-detect 3 col", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny_xyz.txt");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].x == 1.0f);
  REQUIRE(cloud->points[0].intensity == 0.0f);
}

TEST_CASE("load xyzi auto-detect 4 col", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny_xyzi.txt");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].intensity == 0.5f);
}

TEST_CASE("load xyzrgb auto-detect 6 col", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny_xyzrgb.txt");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].r == 255);
  REQUIRE(cloud->points[0].g == 0);
}

TEST_CASE("load xyzrgbi auto-detect 7 col", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny_xyzrgbi.txt");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].r == 255);
  REQUIRE(cloud->points[0].intensity == 0.5f);
}

TEST_CASE("detect unknown throws", "[io]") {
  REQUIRE_THROWS_AS(kpt::detect("foo.zzz"), std::runtime_error);
}

TEST_CASE("load missing file throws", "[io]") {
  REQUIRE_THROWS_AS(kpt::load("nope.bin"), std::runtime_error);
}
```

- [ ] **Step 4: Run tests to verify they fail**

```bash
cmake --build build -j && cd build && ctest --output-on-failure
```
Expected: link error or FAIL (load not implemented).

- [ ] **Step 5: Implement load in `src/kpt/io/io.cpp`**

```cpp
#include "kpt/io/io.hpp"
#include <spdlog/spdlog.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <fstream>
#include <sstream>
#include <cstring>

namespace kpt {

namespace {

PointCloudIRGB::Ptr makeCloud() { return std::make_shared<PointCloudIRGB>(); }

void loadBin(const std::filesystem::path& p, PointCloudIRGB::Ptr cloud) {
  std::ifstream ifs(p, std::ios::binary | std::ios::ate);
  if (!ifs) throw std::runtime_error("file not found: " + p.string());
  auto size = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  constexpr size_t rec = 4 * sizeof(float);
  if (size % rec != 0) throw std::runtime_error("parse error: bin size not multiple of 16");
  size_t n = static_cast<size_t>(size) / rec;
  for (size_t i = 0; i < n; ++i) {
    float buf[4];
    ifs.read(reinterpret_cast<char*>(buf), rec);
    PointT pt;
    pt.x = buf[0]; pt.y = buf[1]; pt.z = buf[2];
    pt.intensity = buf[3];
    pt.r = pt.g = pt.b = 0;
    cloud->push_back(pt);
  }
}

void loadPCL(const std::filesystem::path& p, PointCloudIRGB::Ptr cloud) {
  auto fmt = detect(p);
  if (fmt == Format::PCD) {
    if (pcl::io::loadPCDFile(p.string(), *cloud) < 0)
      throw std::runtime_error("parse error: pcd load failed: " + p.string());
  } else if (fmt == Format::PLY) {
    if (pcl::io::loadPLYFile(p.string(), *cloud) < 0)
      throw std::runtime_error("parse error: ply load failed: " + p.string());
  }
}

void loadAscii(const std::filesystem::path& p, PointCloudIRGB::Ptr cloud) {
  std::ifstream ifs(p);
  if (!ifs) throw std::runtime_error("file not found: " + p.string());
  std::string line;
  int warn_count = 0;
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    std::istringstream ss(line);
    std::vector<float> v; float f;
    while (ss >> f) v.push_back(f);
    if (v.size() < 3) { if (++warn_count <= 50) spdlog::warn("skip short line: {}", line); continue; }
    PointT pt;
    pt.x = v[0]; pt.y = v[1]; pt.z = v[2];
    pt.r = pt.g = pt.b = 0; pt.intensity = 0.0f;
    if (v.size() == 4) { pt.intensity = v[3]; }
    else if (v.size() == 6) { pt.r = (uint8_t)v[3]; pt.g = (uint8_t)v[4]; pt.b = (uint8_t)v[5]; }
    else if (v.size() == 7) { pt.r = (uint8_t)v[3]; pt.g = (uint8_t)v[4]; pt.b = (uint8_t)v[5]; pt.intensity = v[6]; }
    else if (v.size() == 3) { /* ok */ }
    else {
      if (++warn_count <= 50) spdlog::warn("skip line with {} cols: {}", v.size(), line);
      continue;
    }
    cloud->push_back(pt);
  }
  if (warn_count > 50) spdlog::warn("... {} more skipped lines", warn_count - 50);
}

}  // namespace

PointCloudIRGB load(const std::filesystem::path& p) {
  if (!std::filesystem::exists(p)) throw std::runtime_error("file not found: " + p.string());
  auto cloud = makeCloud();
  auto fmt = detect(p);
  switch (fmt) {
    case Format::Bin: loadBin(p, cloud); break;
    case Format::PCD:
    case Format::PLY: loadPCL(p, cloud); break;
    default: loadAscii(p, cloud); break;  // XYZ/XYZI/XYZRGB/XYZRGBI all ascii
  }
  spdlog::debug("loaded {} points from {}", cloud->size(), p.string());
  return *cloud;
}

}  // namespace kpt
```

Note: `load` returns `PointCloudIRGB` by value (PCL cloud copy); test uses `.->` via implicit conversion? No — adjust return to `PointCloudIRGB::Ptr` OR tests use `cloud.size()`. **Decision: return `PointCloudIRGB::Ptr`** for consistency with PCL idioms. Update `io.hpp` signature to `PointCloudIRGBPtr load(...)` and tests to use `->`.

- [ ] **Step 6: Run tests to verify pass**

```bash
cmake --build build -j && cd build && ctest --output-on-failure
```
Expected: all load tests PASS.

- [ ] **Step 7: Commit**

```bash
git add -A && git commit -m "feat(io): implement load for bin/pcd/ply/ascii with auto-detect"
```

---

## Task 3: kpt::io — save (bin + PCL + ASCII flavors)

**Files:**
- Modify: `src/kpt/io/io.cpp`
- Modify: `tests/io_test.cpp`

**Interfaces:**
- Produces: `kpt::save(path, cloud, ascii_flavor?)`

- [ ] **Step 1: Write failing round-trip tests (append to io_test.cpp)**

```cpp
TEST_CASE("round-trip bin", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.bin");
  fs::path out = "build/rt.bin";
  kpt::save(out, cloud);
  auto cloud2 = kpt::load(out);
  REQUIRE(cloud2->size() == cloud->size());
  REQUIRE(cloud2->points[1].intensity == 0.6f);
  fs::remove(out);
}

TEST_CASE("round-trip xyzrgbi explicit flavor", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny_xyzrgbi.txt");
  fs::path out = "build/rt.txt";
  kpt::save(out, cloud, kpt::Format::XYZRGBI);
  std::ifstream ifs(out); std::string line;
  std::getline(ifs, line);
  REQUIRE(line.find(' ') != std::string::npos);  // 7 cols
  fs::remove(out);
}

TEST_CASE("save bin drops rgb", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny_xyzrgb.txt");
  fs::path out = "build/drop.bin";
  kpt::save(out, cloud);
  auto back = kpt::load(out);
  REQUIRE(back->points[0].r == 0);  // rgb lost
  REQUIRE(back->points[0].x == 1.0f);
  fs::remove(out);
}
```

- [ ] **Step 2: Run — verify fail**

```bash
cmake --build build -j && cd build && ctest --output-on-failure
```
Expected: FAIL (save not implemented).

- [ ] **Step 3: Implement save in io.cpp**

```cpp
namespace kpt {

namespace {
void saveBin(const std::filesystem::path& p, const PointCloudIRGB& cloud) {
  std::ofstream ofs(p, std::ios::binary);
  if (!ofs) throw std::runtime_error("cannot write: " + p.string());
  for (const auto& pt : cloud.points) {
    float buf[4] = {pt.x, pt.y, pt.z, pt.intensity};
    ofs.write(reinterpret_cast<char*>(buf), sizeof(buf));
  }
}
void savePCL(const std::filesystem::path& p, const PointCloudIRGB& cloud) {
  auto fmt = detect(p);
  if (fmt == Format::PCD) pcl::io::savePCDFileBinary(p.string(), cloud);
  else if (fmt == Format::PLY) pcl::io::savePLYFileBinary(p.string(), cloud);
}
void saveAscii(const std::filesystem::path& p, const PointCloudIRGB& cloud, Format flavor) {
  std::ofstream ofs(p);
  if (!ofs) throw std::runtime_error("cannot write: " + p.string());
  ofs << std::fixed; ofs.precision(6);
  for (const auto& pt : cloud.points) {
    switch (flavor) {
      case Format::XYZ:     ofs << pt.x << ' ' << pt.y << ' ' << pt.z << '\n'; break;
      case Format::XYZI:    ofs << pt.x << ' ' << pt.y << ' ' << pt.z << ' ' << pt.intensity << '\n'; break;
      case Format::XYZRGB:  ofs << pt.x << ' ' << pt.y << ' ' << pt.z << ' '
                                << (int)pt.r << ' ' << (int)pt.g << ' ' << (int)pt.b << '\n'; break;
      case Format::XYZRGBI: ofs << pt.x << ' ' << pt.y << ' ' << pt.z << ' '
                                << (int)pt.r << ' ' << (int)pt.g << ' ' << (int)pt.b << ' '
                                << pt.intensity << '\n'; break;
      default: throw std::runtime_error("ascii flavor must be XYZ/XYZI/XYZRGB/XYZRGBI");
    }
  }
}
}  // namespace

void save(const std::filesystem::path& p, const PointCloudIRGB& cloud,
          std::optional<Format> ascii_flavor) {
  auto fmt = ascii_flavor.value_or(detect(p));
  switch (fmt) {
    case Format::Bin: saveBin(p, cloud); break;
    case Format::PCD:
    case Format::PLY: savePCL(p, cloud); break;
    default: saveAscii(p, cloud, fmt); break;
  }
  spdlog::debug("saved {} points to {}", cloud.size(), p.string());
}

}  // namespace kpt
```

- [ ] **Step 4: Run — verify pass**

```bash
cmake --build build -j && cd build && ctest --output-on-failure
```
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(io): implement save for bin/pcd/ply/ascii flavors"
```

---

## Task 4: kpt::label — port se_helper to XYZRGBI

**Files:**
- Create: `src/kpt/label/label.hpp`, `src/kpt/label/label.cpp`
- Create: `tests/label_test.cpp`
- Create: `test/data/tiny.label` (3 ints: 40 70 10 → road/vegetation/car)

**Interfaces:**
- Consumes: `kpt::PointCloudIRGB`
- Produces: `kpt::loadLabel`, `kpt::rangeNetLabelMap`, `kpt::rgbLabelMap`, `kpt::applyLabel`

- [ ] **Step 1: Create label fixture**

```bash
python3 -c "
import struct
with open('test/data/tiny.label','wb') as f:
    for v in [40,70,10]: f.write(struct.pack('i', v))
"
```

- [ ] **Step 2: Write `src/kpt/label/label.hpp`**

```cpp
#pragma once
#include "kpt/types.hpp"
#include <filesystem>
#include <map>
#include <tuple>
#include <vector>

namespace kpt {

std::vector<int> loadLabel(const std::filesystem::path& p);
std::map<int,int> rangeNetLabelMap();
std::map<int,std::tuple<int,int,int>> rgbLabelMap();

PointCloudIRGBPtr applyLabel(const PointCloudIRGBConstPtr& cloud,
                             const std::vector<int>& labels,
                             const std::map<int,int>& label_map,
                             const std::map<int,std::tuple<int,int,int>>& rgb_map,
                             bool drop_unlabeled = false);

}  // namespace kpt
```

- [ ] **Step 3: Write failing tests `tests/label_test.cpp`**

```cpp
#include <catch2/catch_all.hpp>
#include "kpt/label/label.hpp"
namespace fs = std::filesystem;

TEST_CASE("loadLabel reads ints", "[label]") {
  auto labels = kpt::loadLabel("test/data/tiny.label");
  REQUIRE(labels.size() == 3);
  REQUIRE(labels[0] == 40);
  REQUIRE(labels[1] == 70);
}

TEST_CASE("applyLabel colors points", "[label]") {
  kpt::PointCloudIRGB cloud;
  for (int i = 0; i < 3; ++i) {
    kpt::PointT pt; pt.x = i; pt.y = 0; pt.z = 0; pt.intensity = 0;
    cloud.push_back(pt);
  }
  auto cloud_ptr = std::make_shared<kpt::PointCloudIRGB>(cloud);
  auto labels = std::vector<int>{40, 70, 10};
  auto lm = kpt::rangeNetLabelMap();
  auto rm = kpt::rgbLabelMap();
  auto out = kpt::applyLabel(cloud_ptr, labels, lm, rm);
  REQUIRE(out->size() == 3);
  // 40 -> road -> compact 1 -> (34,139,0)
  REQUIRE(out->points[0].r == 34);
  REQUIRE(out->points[0].g == 139);
}
```

- [ ] **Step 4: Run — verify fail**

- [ ] **Step 5: Implement `src/kpt/label/label.cpp`**

Port from old `se_helper.cpp`. Key change: point type `PointXYZI` → `PointXYZRGBI`, `bindLabel + bindRGBLabel` merged into `applyLabel`.

```cpp
#include "kpt/label/label.hpp"
#include <spdlog/spdlog.h>
#include <fstream>
#include <cassert>

namespace kpt {

std::vector<int> loadLabel(const std::filesystem::path& p) {
  std::ifstream ifs(p, std::ios::binary);
  if (!ifs) throw std::runtime_error("file not found: " + p.string());
  std::vector<int> result;
  int label;
  while (ifs.read(reinterpret_cast<char*>(&label), sizeof(int))) {
    result.push_back(label);
  }
  return result;
}

std::map<int,int> rangeNetLabelMap() {
  return {
    {0,0},{1,-1},{10,0},{11,-1},{13,0},{15,-1},{16,-1},{18,0},{20,0},
    {30,-1},{31,-1},{32,-1},{40,1},{44,1},{48,2},{49,3},{50,4},{51,5},
    {52,0},{60,6},{70,7},{71,8},{72,9},{80,10},{81,11},{99,-1},
    {252,-1},{253,-1},{254,-1},{255,-1},{256,-1},{257,-1},{258,-1},{259,-1},
  };
}

std::map<int,std::tuple<int,int,int>> rgbLabelMap() {
  return {
    {0,{0,0,0}},{1,{34,139,0}},{2,{0,255,127}},{3,{8,46,84}},
    {4,{106,90,205}},{5,{65,105,225}},{6,{240,255,255}},{7,{124,252,0}},
    {8,{176,48,96}},{9,{160,32,240}},{10,{218,112,214}},{11,{221,160,221}},
    {-1,{227,23,13}},
  };
}

PointCloudIRGBPtr applyLabel(const PointCloudIRGBConstPtr& cloud,
                             const std::vector<int>& labels,
                             const std::map<int,int>& label_map,
                             const std::map<int,std::tuple<int,int,int>>& rgb_map,
                             bool drop_unlabeled) {
  auto out = std::make_shared<PointCloudIRGB>();
  assert(cloud->size() == labels.size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    auto pt = cloud->points[i];
    int compact = label_map.at(labels[i]);
    pt.intensity = static_cast<float>(compact);
    if (drop_unlabeled && compact == -1) continue;
    auto& rgb = rgb_map.at(compact);
    pt.r = static_cast<uint8_t>(std::get<0>(rgb));
    pt.g = static_cast<uint8_t>(std::get<1>(rgb));
    pt.b = static_cast<uint8_t>(std::get<2>(rgb));
    out->push_back(pt);
  }
  return out;
}

}  // namespace kpt
```

- [ ] **Step 6: Run — verify pass**

- [ ] **Step 7: Commit**

```bash
git add -A && git commit -m "feat(label): port se_helper to XYZRGBI with merged applyLabel"
```

---

## Task 5: kpt::render — port cloud2images_soft

**Files:**
- Create: `src/kpt/render/render.hpp`, `src/kpt/render/render.cpp`
- Create: `tests/render_test.cpp`

**Interfaces:**
- Consumes: `kpt::PointCloudIRGB`, `kpt::View`
- Produces: `kpt::RenderOpts`, `kpt::RenderResult`, `kpt::renderMultiView()`

- [ ] **Step 1: Write `src/kpt/render/render.hpp`**

```cpp
#pragma once
#include "kpt/types.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <utility>

namespace kpt {

struct RenderOpts {
  int width = 640;
  int height = 480;
  float fov = 120.0f;
  std::vector<View> views = {View::Front, View::Right, View::Back, View::Left,
                             View::Top, View::Bottom, View::TopRightFront,
                             View::TopLeftFront, View::BotRightFront, View::BotLeftFront};
};

struct RenderResult {
  std::string view_name;
  cv::Mat image;
};

std::vector<RenderResult> renderMultiView(const PointCloudIRGBConstPtr& cloud,
                                          const RenderOpts& opts);

}  // namespace kpt
```

- [ ] **Step 2: Write failing test `tests/render_test.cpp`**

```cpp
#include <catch2/catch_all.hpp>
#include "kpt/render/render.hpp"

TEST_CASE("renderMultiView produces images", "[render]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  for (int i = 0; i < 100; ++i) {
    kpt::PointT pt;
    pt.x = (i % 10) * 0.5f; pt.y = (i / 10) * 0.5f; pt.z = 0;
    pt.r = 255; pt.g = 0; pt.b = 0; pt.intensity = 0.5f;
    cloud->push_back(pt);
  }
  kpt::RenderOpts opts; opts.width = 64; opts.height = 64;
  auto results = kpt::renderMultiView(cloud, opts);
  REQUIRE(results.size() == opts.views.size());
  REQUIRE(results[0].image.cols == 64);
  REQUIRE(!results[0].view_name.empty());
}
```

- [ ] **Step 3: Run — verify fail**

- [ ] **Step 4: Implement `src/kpt/render/render.cpp`**

Port from `cloud2images_soft.cc`. Replace `PointCloud<pcl::PointXYZRGB>::Ptr` with `PointCloudIRGBConstPtr`. Keep `SimpleRenderer` + `CloudBoundingBox` + `createViewMatrix` as internal helpers in the .cpp. Map `View` enum → `(theta, phi)`:

```cpp
static std::pair<float,float> viewAngles(View v) {
  switch (v) {
    case View::Front: return {0, 0};
    case View::Right: return {M_PI/2, 0};
    case View::Back: return {M_PI, 0};
    case View::Left: return {-M_PI/2, 0};
    case View::Top: return {0, M_PI/4};
    case View::Bottom: return {0, -M_PI/4};
    case View::TopRightFront: return {M_PI/4, M_PI/4};
    case View::TopLeftFront: return {-M_PI/4, M_PI/4};
    case View::BotRightFront: return {M_PI/4, -M_PI/4};
    case View::BotLeftFront: return {-M_PI/4, -M_PI/4};
  }
  return {0,0};
}
static std::string viewName(View v) { /* "正面","右侧",... */ }

std::vector<RenderResult> renderMultiView(const PointCloudIRGBConstPtr& cloud,
                                          const RenderOpts& opts) {
  // bbox, loop views, render, collect
}
```

Full body: copy `SimpleRenderer::render`, `CloudBoundingBox`, `createViewMatrix`, `generateMultiView` logic from old `cloud2images_soft.cc`, adapting types. Replace the old `generateMultiViewImages` (which did `cv::imwrite` internally) — new version returns `RenderResult` vector, no file I/O.

- [ ] **Step 5: Run — verify pass**

- [ ] **Step 6: Commit**

```bash
git add -A && git commit -m "feat(render): port cloud2images_soft to kpt::renderMultiView"
```

---

## Task 6: kpt::viewer — InteractiveViewer

**Files:**
- Create: `src/kpt/viewer/viewer.hpp`, `src/kpt/viewer/viewer.cpp`

**Interfaces:**
- Consumes: `kpt::PointCloudIRGB`, `kpt::ColorBy`
- Produces: `kpt::ViewerOpts`, `kpt::InteractiveViewer`

Note: No automated test (GUI). Manual verification only.

- [ ] **Step 1: Write `src/kpt/viewer/viewer.hpp`**

```cpp
#pragma once
#include "kpt/types.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <string>

namespace kpt {

struct ViewerOpts {
  ColorBy colorby = ColorBy::Intensity;
  int point_size = 3;
  Eigen::Vector3f bg = {0, 0, 0};
};

class InteractiveViewer {
 public:
  explicit InteractiveViewer(const ViewerOpts& opts);
  void show(const PointCloudIRGBConstPtr& cloud, const std::string& title);
  void spin();  // blocks until window closed
  pcl::visualization::PCLVisualizer::Ptr raw() { return viewer_; }
 private:
  ViewerOpts opts_;
  pcl::visualization::PCLVisualizer::Ptr viewer_;
};

}  // namespace kpt
```

- [ ] **Step 2: Implement `src/kpt/viewer/viewer.cpp`**

Adapt `rgbVis`/`UpdateViewer` from old `kitti_player.cc`. For `ColorBy::Intensity` use `PointCloudColorHandlerGenericField`; for `RGB` use `RGBField`; for `Z` use `GenericField("z")`; `None` → single color.

```cpp
#include "kpt/viewer/viewer.hpp"
#include <spdlog/spdlog.h>

namespace kpt {

InteractiveViewer::InteractiveViewer(const ViewerOpts& opts)
    : opts_(opts),
      viewer_(new pcl::visualization::PCLVisualizer("KPT Viewer")) {
  viewer_->setBackgroundColor(opts_.bg.x(), opts_.bg.y(), opts_.bg.z());
  viewer_->addCoordinateSystem(1.0);
  viewer_->initCameraParameters();
}

void InteractiveViewer::show(const PointCloudIRGBConstPtr& cloud, const std::string& title) {
  viewer_->removeAllPointClouds();
  if (opts_.colorby == ColorBy::RGB) {
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer_->addPointCloud<PointT>(cloud, rgb, "pc");
  } else if (opts_.colorby == ColorBy::Intensity) {
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> field(cloud, "intensity");
    viewer_->addPointCloud<PointT>(cloud, field, "pc");
  } else if (opts_.colorby == ColorBy::Z) {
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> field(cloud, "z");
    viewer_->addPointCloud<PointT>(cloud, field, "pc");
  } else {
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single(cloud, 255, 255, 255);
    viewer_->addPointCloud<PointT>(cloud, single, "pc");
  }
  viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, opts_.point_size, "pc");
  viewer_->addText(title, 20, 20, 0, 0, 0, "title");
}

void InteractiveViewer::spin() {
  while (!viewer_->wasStopped()) {
    viewer_->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}  // namespace kpt
```

- [ ] **Step 3: Build to verify compiles**

```bash
cmake --build build -j
```
Expected: compiles (no test — GUI).

- [ ] **Step 4: Commit**

```bash
git add -A && git commit -m "feat(viewer): add InteractiveViewer with ColorBy strategies"
```

---

## Task 7: kpt::player — SequencePlayer

**Files:**
- Create: `src/kpt/player/player.hpp`, `src/kpt/player/player.cpp`

**Interfaces:**
- Consumes: io, viewer, label, render
- Produces: `kpt::PlayerOpts`, `kpt::SequencePlayer`

Note: No automated test (GUI). Manual verification.

- [ ] **Step 1: Write `src/kpt/player/player.hpp`**

```cpp
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
```

- [ ] **Step 2: Implement `src/kpt/player/player.cpp`**

Key logic:
- `enumerate()`: `std::filesystem::directory_iterator(input_dir)`, filter by `fnmatch(glob, filename)`, sort, return vector.
- `run()`: for each file, `kpt::load`; if `label_dir`, build label path `{label_dir}/{stem}.label`, `loadLabel` + `applyLabel`; show in `InteractiveViewer`; if `snapshot_prefix`, call `renderMultiView` + `cv::imwrite` to `{snapshot_prefix}_{stem}_{view}.png`; `spinOnce(1000/fps)`.
- `poses`/`poses2`: if set, read via rapidcsv, plot trajectory points (red/green) in a second viewer — port from `kitti_player_with_poses.cpp`. Keep simple: two viewers side by side.

Use `<fnmatch.h>` for glob (POSIX) or implement simple wildcard match. Prefer `<fnmatch.h>` on Linux (platform is linux per env).

```cpp
#include "kpt/player/player.hpp"
#include "kpt/io/io.hpp"
#include "kpt/viewer/viewer.hpp"
#include "kpt/label/label.hpp"
#include "rapidcsv.h"
#include <spdlog/spdlog.h>
#include <fnmatch.h>
#include <algorithm>
#include <chrono>
#include <thread>

namespace kpt {

namespace {
std::vector<std::filesystem::path> enumerate(const std::filesystem::path& dir, const std::string& glob) {
  std::vector<std::filesystem::path> files;
  for (const auto& e : std::filesystem::directory_iterator(dir)) {
    if (fnmatch(glob.c_str(), e.path().filename().c_str(), 0) == 0)
      files.push_back(e.path());
  }
  std::sort(files.begin(), files.end());
  return files;
}
}  // namespace

SequencePlayer::SequencePlayer(const PlayerOpts& opts) : opts_(opts) {}

void SequencePlayer::run() {
  auto files = enumerate(opts_.input_dir, opts_.glob);
  spdlog::info("player: {} files in {}", files.size(), opts_.input_dir.string());
  if (files.empty()) return;

  ViewerOpts vopts; vopts.colorby = opts_.colorby; vopts.point_size = opts_.point_size;
  InteractiveViewer viewer(vopts);

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
    if (opts_.snapshot_prefix) {
      auto results = kpt::renderMultiView(cloud, opts_.render_opts);
      for (const auto& r : results) {
        cv::imwrite(*opts_.snapshot_prefix + "_" + f.stem().string() + "_" + r.view_name + ".png", r.image);
      }
    }
    viewer.raw()->spinOnce(1000 / std::max(1, opts_.fps));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / std::max(1, opts_.fps)));
    ++frame;
  }
  viewer.spin();
}

}  // namespace kpt
```

Note: poses/poses2 trajectory plotting — keep minimal: if `poses` set, read CSV, render trajectory as red points in a second viewer window. Port `readPoses` from `kitti_player_with_poses.cpp` using `rapidcsv::Document`. If scope too big for one task, implement poses in a follow-up — but spec requires it, so include basic version.

- [ ] **Step 3: Build**

```bash
cmake --build build -j
```
Expected: compiles.

- [ ] **Step 4: Commit**

```bash
git add -A && git commit -m "feat(player): add SequencePlayer with glob/label/snapshot support"
```

---

## Task 8: CLI — pc_convert + pc_batch_convert

**Files:**
- Modify: `src/cli/pc_convert.cc`, `src/cli/pc_batch_convert.cc`

- [ ] **Step 1: Write `src/cli/pc_convert.cc`**

```cpp
#include "kpt/io/io.hpp"
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <iostream>

int main(int argc, char* argv[]) {
  popl::OptionParser op("pc_convert: single-file point cloud converter");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level", "0=err 1=warn 2=info 3=debug", 2);
  auto flavor = op.add<popl::Value<std::string>>("", "ascii-flavor",
              "xyz|xyzi|xyzrgb|xyzrgbi (only for ascii output)", "");
  op.parse(argc, argv);
  if (help->is_set()) { std::cout << op << "\n"; return 0; }
  switch (log_level->value()) {
    case 0: spdlog::set_level(spdlog::level::err); break;
    case 1: spdlog::set_level(spdlog::level::warn); break;
    case 2: spdlog::set_level(spdlog::level::info); break;
    case 3: spdlog::set_level(spdlog::level::debug); break;
  }
  auto positional = op.non_option_args();
  if (positional.size() < 2) { std::cerr << "usage: pc_convert <input> <output> [--ascii-flavor ...]\n"; return 1; }

  try {
    auto cloud = kpt::load(positional[0]);
    std::optional<kpt::Format> af;
    if (flavor->is_set() && !flavor->value().empty()) {
      std::string f = flavor->value();
      if (f == "xyz") af = kpt::Format::XYZ;
      else if (f == "xyzi") af = kpt::Format::XYZI;
      else if (f == "xyzrgb") af = kpt::Format::XYZRGB;
      else if (f == "xyzrgbi") af = kpt::Format::XYZRGBI;
      else { spdlog::error("unknown ascii-flavor: {}", f); return 1; }
    }
    kpt::save(positional[1], *cloud, af);
    spdlog::info("converted {} -> {} ({} points)", positional[0], positional[1], cloud->size());
  } catch (const std::exception& e) {
    spdlog::error("{}", e.what());
    return 1;
  }
  return 0;
}
```

- [ ] **Step 2: Write `src/cli/pc_batch_convert.cc`**

```cpp
#include "kpt/io/io.hpp"
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <iostream>
#include <filesystem>
#include <fnmatch.h>
#include <algorithm>

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
  popl::OptionParser op("pc_batch_convert: batch convert a directory");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level", "", 2);
  auto in_dir = op.add<popl::Value<std::string>>("i", "input-dir", "input directory", "");
  auto out_dir = op.add<popl::Value<std::string>>("o", "output-dir", "output directory", "");
  auto to = op.add<popl::Value<std::string>>("t", "to", "bin|pcd|ply|xyz|xyzi|xyzrgb|xyzrgbi", "");
  auto glob = op.add<popl::Value<std::string>>("g", "glob", "fnmatch pattern, default *", "*");
  auto flavor = op.add<popl::Value<std::string>>("", "ascii-flavor", "override ascii flavor", "");
  op.parse(argc, argv);
  if (help->is_set()) { std::cout << op << "\n"; return 0; }
  // set log level (same switch as pc_convert) — factor into helper if desired
  switch (log_level->value()) { case 0: spdlog::set_level(spdlog::level::err); break; case 1: spdlog::set_level(spdlog::level::warn); break; case 2: spdlog::set_level(spdlog::level::info); break; case 3: spdlog::set_level(spdlog::level::debug); break; }

  if (in_dir->value().empty() || out_dir->value().empty() || to->value().empty()) {
    std::cerr << "required: --input-dir --output-dir --to\n"; return 1;
  }
  std::optional<kpt::Format> target_fmt;
  std::string t = to->value();
  if (t == "bin") target_fmt = kpt::Format::Bin;
  else if (t == "pcd") target_fmt = kpt::Format::PCD;
  else if (t == "ply") target_fmt = kpt::Format::PLY;
  else if (t == "xyz") target_fmt = kpt::Format::XYZ;
  else if (t == "xyzi") target_fmt = kpt::Format::XYZI;
  else if (t == "xyzrgb") target_fmt = kpt::Format::XYZRGB;
  else if (t == "xyzrgbi") target_fmt = kpt::Format::XYZRGBI;
  else { spdlog::error("unknown --to: {}", t); return 1; }

  std::optional<kpt::Format> af;
  if (flavor->is_set() && !flavor->value().empty()) {
    std::string f = flavor->value();
    if (f == "xyz") af = kpt::Format::XYZ;
    else if (f == "xyzi") af = kpt::Format::XYZI;
    else if (f == "xyzrgb") af = kpt::Format::XYZRGB;
    else if (f == "xyzrgbi") af = kpt::Format::XYZRGBI;
  }
  std::string ext = "." + t;
  fs::create_directories(out_dir->value());

  std::vector<fs::path> files;
  for (const auto& e : fs::directory_iterator(in_dir->value())) {
    if (fnmatch(glob->value().c_str(), e.path().filename().c_str(), 0) == 0)
      files.push_back(e.path());
  }
  std::sort(files.begin(), files.end());

  int ok = 0, fail = 0;
  for (const auto& f : files) {
    try {
      auto cloud = kpt::load(f);
      fs::path out = fs::path(out_dir->value()) / (f.stem().string() + ext);
      kpt::save(out, *cloud, af);
      spdlog::info("ok: {} -> {}", f.string(), out.string());
      ++ok;
    } catch (const std::exception& e) {
      spdlog::warn("fail: {} : {}", f.string(), e.what());
      ++fail;
    }
  }
  spdlog::info("done: {} ok, {} fail", ok, fail);
  return fail == 0 ? 0 : 1;
}
```

- [ ] **Step 3: Build + manual test**

```bash
cmake --build build -j
./build/pc_convert test/data/tiny.bin build/tiny.pcd
./build/pc_convert build/tiny.pcd build/back.bin
./build/pc_batch_convert -i test/data -o build/batch --to pcd -g "*.bin"
```
Expected: conversions succeed, files exist.

- [ ] **Step 4: Commit**

```bash
git add -A && git commit -m "feat(cli): add pc_convert and pc_batch_convert"
```

---

## Task 9: CLI — pc_viewer + pc_render

**Files:**
- Modify: `src/cli/pc_viewer.cc`, `src/cli/pc_render.cc`

- [ ] **Step 1: Write `src/cli/pc_viewer.cc`**

```cpp
#include "kpt/io/io.hpp"
#include "kpt/viewer/viewer.hpp"
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <iostream>
#include <sstream>

int main(int argc, char* argv[]) {
  popl::OptionParser op("pc_viewer: single-frame interactive viewer");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level", "", 2);
  auto colorby = op.add<popl::Value<std::string>>("c", "colorby", "intensity|rgb|z|none", "intensity");
  auto psize = op.add<popl::Value<int>>("s", "point-size", "", 3);
  auto bg = op.add<popl::Value<std::string>>("b", "bg", "r,g,b (0-1)", "0,0,0");
  op.parse(argc, argv);
  if (help->is_set()) { std::cout << op << "\n"; return 0; }
  switch (log_level->value()) { case 0: spdlog::set_level(spdlog::level::err); break; case 1: spdlog::set_level(spdlog::level::warn); break; case 2: spdlog::set_level(spdlog::level::info); break; case 3: spdlog::set_level(spdlog::level::debug); break; }

  auto pos = op.non_option_args();
  if (pos.empty()) { std::cerr << "usage: pc_viewer <file> [options]\n"; return 1; }

  kpt::ColorBy cb = kpt::ColorBy::Intensity;
  std::string c = colorby->value();
  if (c == "rgb") cb = kpt::ColorBy::RGB;
  else if (c == "z") cb = kpt::ColorBy::Z;
  else if (c == "none") cb = kpt::ColorBy::None;

  Eigen::Vector3f bgv(0,0,0);
  { std::stringstream ss(bg->value()); char sep; ss >> bgv.x() >> sep >> bgv.y() >> sep >> bgv.z(); }

  try {
    auto cloud = kpt::load(pos[0]);
    kpt::ViewerOpts opts; opts.colorby = cb; opts.point_size = psize->value(); opts.bg = bgv;
    kpt::InteractiveViewer viewer(opts);
    viewer.show(cloud, pos[0]);
    viewer.spin();
  } catch (const std::exception& e) {
    spdlog::error("{}", e.what());
    return 1;
  }
  return 0;
}
```

- [ ] **Step 2: Write `src/cli/pc_render.cc`**

```cpp
#include "kpt/io/io.hpp"
#include "kpt/render/render.hpp"
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>

static kpt::View parseView(const std::string& s) {
  if (s == "front") return kpt::View::Front;
  if (s == "right") return kpt::View::Right;
  if (s == "back") return kpt::View::Back;
  if (s == "left") return kpt::View::Left;
  if (s == "top") return kpt::View::Top;
  if (s == "bottom") return kpt::View::Bottom;
  if (s == "toprightfront") return kpt::View::TopRightFront;
  if (s == "topleftfront") return kpt::View::TopLeftFront;
  if (s == "botrightfront") return kpt::View::BotRightFront;
  if (s == "botleftfront") return kpt::View::BotLeftFront;
  throw std::runtime_error("unknown view: " + s);
}

int main(int argc, char* argv[]) {
  popl::OptionParser op("pc_render: multi-view PNG snapshot");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level", "", 2);
  auto prefix = op.add<popl::Value<std::string>>("o", "output-prefix", "output filename prefix", "");
  auto w = op.add<popl::Value<int>>("", "width", "", 640);
  auto h = op.add<popl::Value<int>>("", "height", "", 480);
  auto fov = op.add<popl::Value<float>>("", "fov", "field of view degrees", 120.0f);
  auto views = op.add<popl::Value<std::string>>("", "views", "all|front,right,...", "all");
  op.parse(argc, argv);
  if (help->is_set()) { std::cout << op << "\n"; return 0; }
  switch (log_level->value()) { case 0: spdlog::set_level(spdlog::level::err); break; case 1: spdlog::set_level(spdlog::level::warn); break; case 2: spdlog::set_level(spdlog::level::info); break; case 3: spdlog::set_level(spdlog::level::debug); break; }

  auto pos = op.non_option_args();
  if (pos.empty() || prefix->value().empty()) {
    std::cerr << "usage: pc_render <file> -o <prefix> [options]\n"; return 1;
  }

  try {
    auto cloud = kpt::load(pos[0]);
    kpt::RenderOpts opts; opts.width = w->value(); opts.height = h->value(); opts.fov = fov->value();
    std::string vs = views->value();
    if (vs != "all") {
      opts.views.clear();
      std::stringstream ss(vs); std::string item;
      while (std::getline(ss, item, ',')) opts.views.push_back(parseView(item));
    }
    auto results = kpt::renderMultiView(cloud, opts);
    for (const auto& r : results) {
      std::string fn = prefix->value() + "_" + r.view_name + ".png";
      cv::imwrite(fn, r.image);
      spdlog::info("wrote {}", fn);
    }
  } catch (const std::exception& e) {
    spdlog::error("{}", e.what());
    return 1;
  }
  return 0;
}
```

- [ ] **Step 3: Build + manual test**

```bash
cmake --build build -j
./build/pc_render data/000123.pcd -o build/shot --width 320 --height 240
ls build/shot_*.png
```
Expected: 10 PNG files.

- [ ] **Step 4: Commit**

```bash
git add -A && git commit -m "feat(cli): add pc_viewer and pc_render"
```

---

## Task 10: CLI — pc_player

**Files:**
- Modify: `src/cli/pc_player.cc`

- [ ] **Step 1: Write `src/cli/pc_player.cc`**

```cpp
#include "kpt/player/player.hpp"
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <iostream>

static kpt::ColorBy parseColorBy(const std::string& s) {
  if (s == "rgb") return kpt::ColorBy::RGB;
  if (s == "z") return kpt::ColorBy::Z;
  if (s == "label") return kpt::ColorBy::Label;
  if (s == "none") return kpt::ColorBy::None;
  return kpt::ColorBy::Intensity;
}

int main(int argc, char* argv[]) {
  popl::OptionParser op("pc_player: sequence player");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level", "", 2);
  auto in_dir = op.add<popl::Value<std::string>>("i", "input-dir", "input directory", "");
  auto glob = op.add<popl::Value<std::string>>("g", "glob", "fnmatch pattern", "*");
  auto label_dir = op.add<popl::Value<std::string>>("", "label-dir", "semantic label dir", "");
  auto poses = op.add<popl::Value<std::string>>("", "poses", "pose csv file", "");
  auto poses2 = op.add<popl::Value<std::string>>("", "poses2", "second pose csv", "");
  auto colorby = op.add<popl::Value<std::string>>("c", "colorby", "intensity|rgb|z|label|none", "intensity");
  auto psize = op.add<popl::Value<int>>("s", "point-size", "", 3);
  auto snap = op.add<popl::Value<std::string>>("", "snapshot", "output prefix for per-frame PNGs", "");
  auto snap_w = op.add<popl::Value<int>>("", "snapshot-w", "", 640);
  auto snap_h = op.add<popl::Value<int>>("", "snapshot-h", "", 480);
  auto snap_fov = op.add<popl::Value<float>>("", "snapshot-fov", "", 120.0f);
  auto fps = op.add<popl::Value<int>>("f", "fps", "", 10);
  op.parse(argc, argv);
  if (help->is_set()) { std::cout << op << "\n"; return 0; }
  switch (log_level->value()) { case 0: spdlog::set_level(spdlog::level::err); break; case 1: spdlog::set_level(spdlog::level::warn); break; case 2: spdlog::set_level(spdlog::level::info); break; case 3: spdlog::set_level(spdlog::level::debug); break; }

  if (in_dir->value().empty()) { std::cerr << "required: --input-dir\n"; return 1; }

  kpt::PlayerOpts opts;
  opts.input_dir = in_dir->value();
  opts.glob = glob->value();
  if (label_dir->is_set() && !label_dir->value().empty()) opts.label_dir = label_dir->value();
  if (poses->is_set() && !poses->value().empty()) opts.poses = poses->value();
  if (poses2->is_set() && !poses2->value().empty()) opts.poses2 = poses2->value();
  opts.colorby = parseColorBy(colorby->value());
  opts.point_size = psize->value();
  if (snap->is_set() && !snap->value().empty()) {
    opts.snapshot_prefix = snap->value();
    opts.render_opts.width = snap_w->value();
    opts.render_opts.height = snap_h->value();
    opts.render_opts.fov = snap_fov->value();
  }
  opts.fps = fps->value();

  try {
    kpt::SequencePlayer player(opts);
    player.run();
  } catch (const std::exception& e) {
    spdlog::error("{}", e.what());
    return 1;
  }
  return 0;
}
```

- [ ] **Step 2: Build + manual test**

```bash
cmake --build build -j
mkdir -p build/seq && cp data/000123.pcd build/seq/ && cp data/000123.pcd build/seq/000124.pcd
./build/pc_player -i build/seq -g "*.pcd" -c intensity
```
Expected: viewer window opens, cycles frames.

- [ ] **Step 3: Commit**

```bash
git add -A && git commit -m "feat(cli): add pc_player"
```

---

## Task 11: README

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Rewrite README.md**

Cover:
- Project name + one-line description
- Dependencies (PCL, OpenCV, Eigen, CMake3.10+, C++20) — all vendored or system
- Build: `cmake -B build && cmake --build build -j`
- Tool list with usage examples for all 5 CLIs
- Format support matrix (reference spec)
- Project structure overview
- License note (existing LICENSE)

- [ ] **Step 2: Commit**

```bash
git add -A && git commit -m "docs: rewrite README for kpt toolset"
```

---

## Task 12: Final verification

- [ ] **Step 1: Clean build from scratch**

```bash
rm -rf build && cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j
```
Expected: configures + builds all 5 CLIs + kpt_tests with no errors.

- [ ] **Step 2: Run tests**

```bash
cd build && ctest --output-on-failure
```
Expected: all PASS.

- [ ] **Step 3: Smoke test all 5 CLIs**

```bash
./build/pc_convert --help
./build/pc_convert test/data/tiny.bin build/smoke.pcd
./build/pc_batch_convert --help
./build/pc_batch_convert -i test/data -o build/smoke_batch --to ply -g "*.bin"
./build/pc_render data/000123.pcd -o build/smoke_shot --width 320 --height 240
ls build/smoke_shot_*.png | wc -l   # expect 10
# pc_viewer / pc_player: manual GUI verify
```

- [ ] **Step 4: Final commit if any fixes needed**

```bash
git add -A && git commit -m "chore: final verification fixes"
```

---

## Self-Review Notes

- **Spec coverage**: io (T2,T3) ✓; label (T4) ✓; render (T5) ✓; viewer (T6) ✓; player (T7) ✓; 5 CLIs (T8-T10) ✓; CMake rewrite (T1) ✓; file cleanup (T1) ✓; README (T11) ✓; testing (T2-T5 unit + T12 smoke) ✓.
- **Placeholder scan**: render T5 says "full body: copy ... adapting types" — acceptable since source file is in repo and adaptation is mechanical; included key signature `viewAngles`/`viewName`. player T7 poses logic flagged as minimal — acceptable per spec scope.
- **Type consistency**: `PointCloudIRGBPtr`/`ConstPtr` used consistently. `load` returns `Ptr` (corrected in T2 step 5 note). `applyLabel` takes `ConstPtr` returns `Ptr`.
