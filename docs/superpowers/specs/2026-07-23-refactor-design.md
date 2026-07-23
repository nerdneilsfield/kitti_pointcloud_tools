# Refactor Design: KITTI Pointcloud Tools

Date: 2026-07-23
Status: Approved (rev.2 — 9 corrections applied)

## Goal

重构为薄库 + 五件套 CLI 的点云工具集。支持 bin/pcd/ply/xyz 四大类（ASCII 细分 xyz/xyzi/xyzrgb/xyzrgbi）互转，单帧交互 viewer，文件夹序列 player，多视角快照 render。

## Context

现状：十余零碎工具散落 `src/`，点型不统一（PointXYZI / PointXYZRGB 混用），CLI 解析方式不一（popl / 裸 argc），`binary_to_images.cc` 已坏。自用工具，无兼容包袱，clean break。

## Architecture

库 `kpt`（static lib）拆五子模块，置于 `src/kpt/`，命名空间 `kpt::`。CLI 置于 `src/cli/`，各含仅一 `main`。

```
src/kpt/
  types.hpp          # PointCloudIRGB, Format enum, ColorBy enum, View enum
  io/
    format.hpp       # enum class Format { Bin, PCD, PLY, XYZ, XYZI, XYZRGB, XYZRGBI }
    io.hpp / io.cpp  # detect / load / save
  viewer/
    viewer.hpp/cpp   # InteractiveViewer: 单帧 PCLVisualizer
  player/
    player.hpp/cpp   # SequencePlayer: directory_iterator + glob filter
  render/
    render.hpp/cpp   # 由 cloud2images_soft 移植
  label/
    label.hpp/cpp    # 由 se_helper 移植，PointXYZRGBI canonical
src/cli/
  pc_convert.cc
  pc_batch_convert.cc
  pc_viewer.cc
  pc_player.cc
  pc_render.cc
```

依赖：`io` 独立；`label` 独立；`render` 依赖 `io`(types)；`viewer` 依赖 `io`；`player` 依赖 `io`+`viewer`+`label`+`render`。CLI 各依赖对应模块。

## Canonical Point Type

`pcl::PointXYZRGBI` — 含 x y z rgb intensity，PCL 原生支持。

## Format Support Matrix

读入（→ canonical XYZRGBI）：

| 格式 | 识别方式 | 字段映射 |
|------|---------|---------|
| bin (KITTI) | `.bin` | x y z intensity；RGB 置黑 |
| pcd | `.pcd` | PCL load，取 xyz/rgb/intensity，缺字段补默认 |
| ply | `.ply` | PCL load，同上 |
| xyz | `.xyz` + 3 列 | x y z；I=0 RGB=黑 |
| xyzi | `.xyzi` + 4 列 | x y z intensity |
| xyzrgb | `.xyzrgb` + 6 列 | x y z r g b；I=0 |
| xyzrgbi | `.xyzrgbi` + 7 列 | x y z r g b intensity |

ASCII 读入按列数 auto-detect 子格式（3/4/6/7），扩展名仅初步分派。

写出（canonical → 目标）：

| 格式 | 写出字段 | 丢什么 |
|------|---------|--------|
| bin | x y z i | RGB |
| pcd | 全 XYZRGBI | 无 |
| ply | 全 XYZRGBI | 无 |
| xyz | x y z | I + RGB |
| xyzi | x y z i | RGB |
| xyzrgb | x y z r g b | I |
| xyzrgbi | 全 | 无 |

ASCII 写子格式由 `--ascii-flavor {xyz|xyzi|xyzrgb|xyzrgbi}` 显式指定；默认按输出扩展名推。

## Module Interfaces

### kpt::types
```cpp
enum class Format { Bin, PCD, PLY, XYZ, XYZI, XYZRGB, XYZRGBI };
enum class ColorBy { Intensity, RGB, Z, Label, None };
enum class View { Front, Right, Back, Left, Top, Bottom,
                  TopRightFront, TopLeftFront, BotRightFront, BotLeftFront };
// ViewFlag bitmask / "all" 由 CLI 解析层处理，RenderOpts 直接接 std::vector<View>
```

### kpt::io
```cpp
Format detect(const std::filesystem::path& p);  // 仅按扩展名，不检查存在性
// 抛 std::runtime_error("unknown format: <ext>") 当扩展名未知
// 不抛文件不存在 — 那是 load 的职责
PointCloudIRGB load(const std::filesystem::path& p);
// 抛 std::runtime_error("file not found: <path>") / ("parse error: <detail>")
void save(const std::filesystem::path& p, const PointCloudIRGB& cloud,
          std::optional<Format> ascii_flavor = std::nullopt);
// 抛 std::runtime_error("cannot write: <path>") 当路径不可写
```

### kpt::viewer
```cpp
struct ViewerOpts { ColorBy colorby; int point_size; Eigen::Vector3f bg; };
class InteractiveViewer {
 public:
  InteractiveViewer(const ViewerOpts& opts);
  void show(const PointCloudIRGB::ConstPtr& cloud, const std::string& title);
  void spin();  // blocks until window closed
};
```

### kpt::render
```cpp
struct RenderOpts {
  int width, height; float fov;
  std::vector<View> views;  // 默认全部 10 视角
};
struct RenderResult { std::string view_name; cv::Mat image; };
std::vector<RenderResult>
renderMultiView(const PointCloudIRGB::ConstPtr& cloud, const RenderOpts& opts);
// 模块仅负责渲染，不写文件。CLI 层负责 cv::imwrite。
```

### kpt::label
```cpp
std::vector<int> loadLabel(const std::filesystem::path& p);
std::map<int,int> rangeNetLabelMap();       // 原始 label -> 紧凑 id
std::map<int,std::tuple<int,int,int>> rgbLabelMap();  // 紧凑 id -> RGB
PointCloudIRGB::Ptr applyLabel(const PointCloudIRGB::ConstPtr& cloud,
                               const std::vector<int>& labels,
                               const std::map<int,int>& label_map,
                               const std::map<int,std::tuple<int,int,int>>& rgb_map);
// 一步到位：label_map 映射原始 label -> 紧凑 id，rgb_map 映射紧凑 id -> RGB，
// 写入 cloud.rgb + cloud.intensity。丢弃 -1 紧凑 id 的点（保留可选 flag）。
```

### kpt::player
```cpp
struct PlayerOpts {
  std::filesystem::path input_dir, label_dir;
  std::string glob;  // 唯一过滤手段，支持简化 glob (fnmatch 风格)，默认 "*"
  std::optional<std::filesystem::path> poses, poses2;
  ColorBy colorby; int point_size;
  std::optional<std::string> snapshot_prefix;
  RenderOpts render_opts;  // snapshot 启用时使用，否则忽略
  int fps;
};
class SequencePlayer {
 public:
  SequencePlayer(const PlayerOpts& opts);
  void run();  // 内部 enumerate + 播放循环 + 可选 snapshot，单阶段接口
};
```
注：`prefix`/`suffix` 删除，统一用 `glob`（fnmatch 风格，如 `*.bin` / `frame_*.pcd`）。

注：`enumerate()` 公开方法删除，`run()` 内部完成枚举 + 播放，简化接口。

## CLI Toolset

统一 `popl` 解析，统一 `--log-level {err|warn|info|debug}` flag，统一 `-h/--help` 由 popl 自动生成并打印所有选项后 exit 0。

### pc_convert — 单文件转换
```
pc_convert <input> <output> [--ascii-flavor xyz|xyzi|xyzrgb|xyzrgbi] [-h]
```

### pc_batch_convert — 批量转换
```
pc_batch_convert --input-dir <dir> --output-dir <dir>
  --to bin|pcd|ply|xyz|xyzi|xyzrgb|xyzrgbi
  [--glob <pattern>]      # 默认 "*"
  [--ascii-flavor ...]    # 仅 ASCII 目标生效
  [-h]
```
枚举 input-dir，按 glob（fnmatch 风格）过滤，逐个转。单文件失败 warn + 继续，末尾汇总成功/失败计数。

### pc_viewer — 单帧交互
```
pc_viewer <file>
  [--colorby intensity|rgb|z|none]
  [--point-size <n>] [--bg r,g,b]
  [-h]
```

### pc_player — 序列播放
```
pc_player --input-dir <dir>
  [--glob <pattern>]      # 默认 "*"
  [--label-dir <dir>] [--poses <file>] [--poses2 <file>]
  [--colorby intensity|rgb|z|label] [--point-size <n>]
  [--snapshot <out_prefix>] [--snapshot-w <n>] [--snapshot-h <n>]
  [--snapshot-fov <deg>] [--snapshot-views all|front,right,back,...]
  [--fps <n>]
  [-h]
```
`--label-dir`/`--poses` 接 `kpt::label` 做语义着色与轨迹叠加。`--snapshot*` 系列映射入 `PlayerOpts.render_opts`。

### pc_render — 多视角快照
```
pc_render <file> --output-prefix <prefix>
  [--width <n>] [--height <n>] [--fov <deg>]
  [--views all|front,right,back,left,top,bottom,toprightfront,...]
  [-h]
```
无头可跑。CLI 层调用 `renderMultiView` 后自行 `cv::imwrite`，文件名 `<prefix>_<view_name>.png`。

## Error Handling

- `detect` 抛 `std::runtime_error("unknown format: <ext>")` 当扩展名未知；不检查文件存在性（那是 `load` 职责）。
- `load` 抛 `std::runtime_error("file not found: <path>")` 或 `("parse error: <detail>")`。
- `save` 抛 `std::runtime_error("cannot write: <path>")` 当路径不可写。
- CLI 层 catch → `spdlog::error` + exit 1。
- ASCII load 遇列数 < 3 或不一致：跳过该行 + `spdlog::warn` 计数。列数非 3/4/6/7 报错中止。warn 超过 50 条后停止逐条输出，末尾汇总总数防刷屏。
- 批量单文件失败：warn + 继续，末尾汇总成功/失败计数。

## Testing

Catch2（已 vendored），`ENABLE_TESTING` 控构建。

- `io`：round-trip 各格式 load→save→load 比较点数与坐标容差。bin↔pcd↔ply↔xyzrgbi 全链路。ASCII auto-detect 列数判别。
- `label`：loadLabel fixture，applyLabel 着色正确性。
- `render`：synthetic cloud，renderMultiView 返回非空、尺寸正确。
- `viewer`/`player`：不测，手动验证。
- fixture：复用 `data/000123.pcd`，另造 `test/data/` 小文件。

## File Cleanup

**删除**（被新五件套取代）：
- `src/kitti_binary_converter.cc`, `src/convert_pcd_to_binary.cc`
- `src/kitti_player.cc`, `src/kitti_player_two.cc`, `src/kitti_player_with_poses.cpp`, `src/kitti_player.h`
- `src/binary_to_images.cc`（已坏）
- `src/pcd_to_images.cpp`, `src/pcd_to_images_soft.cpp`
- `src/kitti_binary.h`, `src/kitti_binary.cc`
- `src/se_helper.h`, `src/se_helper.cpp`
- `src/cloud2images.hpp`, `src/cloud2images.cpp`, `src/cloud2images_soft.hpp`, `src/cloud2images_soft.cc`
- `src/types.h`

**保留**：
- `src/rapidcsv.h`（vendor，player poses 读取用）
- `third_party/` 全部，`cmake/` 全部，`data/000123.pcd`

**CMakeLists.txt 重写**：删旧 targets，新增 `kpt` static lib + 五 CLI exe + `kpt_tests`。一次性 clean break，git history 保留旧文件可追溯。
