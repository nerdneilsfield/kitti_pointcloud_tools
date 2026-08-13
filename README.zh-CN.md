# kitti_pointcloud_tools（kpt）

简体中文 | [English](README.md)

**一套点云工具，从命令行延伸到桌面、浏览器与 VS Code。**

KPT 可转换、查看、播放与渲染 KITTI 风格点云，且不依赖 PCL。十一种原生
codec、桌面 workbench、无窗口 renderer、WebAssembly viewer 与 VS Code extension
共用同一套无外部依赖的点云模型。

| 需求 | 从这里开始 |
|---|---|
| 转换单个点云 | `kpt_convert input.bin output.pcd` |
| 本机查看或播放 | `kpt_gui` / `kpt_player` |
| 在 CI 或服务器导出 PNG | `kpt_render input.bin -o frame` |
| 在编辑器浏览点云 | 安装 VSIX extension |

## 功能

- 5 个 CLI 工具：单文件转换、批量转换、交互查看、序列播放、多视角渲染
- 11 种格式：`bin`、`pcd`、`ply`、`las`、`pts`、`obj`、`npy`、
  `xyz`、`xyzi`、`xyzrgb`、`xyzrgbi`
- 自有 KITTI BIN、PCD、PLY、LAS、PTS、OBJ、NPY 与文本 reader/writer
- 统一点类型 `kpt::PointXYZRGBI`：x、y、z、rgb、intensity
- `kpt_render` 无窗口多视角 PNG 渲染
- `kpt_player` 支持 semantic label、两组 pose CSV、播放及逐帧 snapshot
- 可选 Dear ImGui workbench：Viewer、Player、Convert、Batch Convert、Render
- 静态 WebAssembly workbench：WebGL2 Viewer 与 Player
- VS Code + Three.js extension：查看、序列播放、semantic label、pose 与转换
- 中文文件名、中文目录与平台原生配置目录

## 平台状态

| 平台 | 架构 | GUI backend | 状态 |
|---|---|---|---|
| Linux | x86-64 | OpenGL 3.3 / X11 | 已实现并验证 |
| Windows 10 1903+/11 | x86-64 | OpenGL 3.3 | 已实现；已启用 CI 编译打包 |
| macOS 13+ | arm64、x86-64 | Metal | 已实现；已启用双架构 CI 编译打包 |
| 浏览器 | WASM | WebGL2 | 已实现；Chrome smoke 已验证 |

Release workflow 分别在 Apple Silicon 与 Intel runner 上编译，再审计合并后的
universal bundle 架构、依赖、布局及 ad-hoc 签名。

浏览器构建使用 Emscripten pthreads，部署必须启用跨源隔离。构建、启动及
headers 说明见 [WebAssembly / WebGL2 构建](docs/web-build.md)。

## 安装 Release

从 [GitHub Releases](https://github.com/nerdneilsfield/kitti_pointcloud_tools/releases)
下载匹配平台的产物：

- **Ubuntu**：按系统版本选择 `ubuntu22.04`、`ubuntu24.04` 或 `ubuntu26.04`
  DEB，以 `sudo apt install ./kitti-pointcloud-tools_*.deb` 安装。
- **Windows**：解压 portable x64 ZIP，运行 `kpt_gui.exe`。内嵌的
  `kpt-workbench.ico` 会用于 Explorer、任务栏与窗口图标。
- **macOS**：打开 universal DMG，将 **KPT Workbench** 拖入 Applications。
- **VS Code**：Extensions → `…` → **Install from VSIX…**，选择 VSIX 文件。

全部桌面图标从同一 SVG 源生成：
[`packaging/icons/kpt-workbench.svg`](packaging/icons/kpt-workbench.svg)。
Release 构建据此产生 Linux SVG/PNG、Windows 多尺寸 `.ico` 与 macOS `.icns`；
勿手改生成文件。

## 依赖

### 通用依赖

- CMake 3.21 或更新版本
- Ninja、Git
- C++20 编译器：
  - Linux：GCC 11+ 或 Clang 14+
  - Windows：Visual Studio 2022 / MSVC
  - macOS：Xcode 14+ 所带 Apple Clang

`ccache` 为可选项。CMake 会从 `PATH` 自动发现它，并缓存 C/C++ 编译结果；适合
反复本地构建，Linux CI 亦会启用。链接与测试执行不在其缓存范围内。

项目会优先查找系统 Eigen；找不到时使用 `third_party/` 内的 vendored Eigen。
仅构建转换工具时，不需要安装 PCL、OpenCV、VTK、OpenGL、GLFW、Fontconfig
或 Freetype。

下列依赖已 vendored，无须单独安装：

- spdlog、popl、Catch2 v2、Eigen、rapidcsv
- stb image writer；`stb_image` 仅供测试
- Dear ImGui `v1.92.8-docking`、ImGuiFileDialog `v0.6.8`
- GLFW `3.4`，仅 `KPT_BUILD_GUI=ON` 时构建

### Linux 安装依赖

Ubuntu 22.04/24.04/26.04 的 GUI/X11 构建：

```bash
sudo apt update
sudo apt install build-essential cmake ninja-build git pkg-config \
  ccache \
  libfreetype-dev libfontconfig1-dev \
  libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev \
  libxcursor-dev libxi-dev xvfb
cmake --version  # 必须 >= 3.21
```

仅构建转换工具：

```bash
sudo apt install build-essential cmake ninja-build git
```

`linux-vcpkg-*` presets 会由 vcpkg 提供 Freetype 与 Fontconfig，但 vendored
GLFW 仍需主机上的 X11/OpenGL development packages。

### Windows 安装依赖

1. 安装 Visual Studio 2022。
2. 勾选 **Desktop development with C++**、MSVC、Windows SDK、CMake tools
   与 Ninja。
3. 安装 Git。
4. clone 并 bootstrap vcpkg。

在已激活 x64 环境的 **Developer PowerShell for VS 2022** 中执行：

```powershell
git clone https://github.com/microsoft/vcpkg C:\src\vcpkg
C:\src\vcpkg\bootstrap-vcpkg.bat
$env:VCPKG_ROOT = "C:\src\vcpkg"
```

Windows executables 内嵌 UTF-8 code page 与 long-path manifest。自有 codec
直接使用 `std::filesystem::path`；Win32 边界使用 wide API。

### macOS 安装依赖

```bash
xcode-select --install
brew install cmake ninja git
git clone https://github.com/microsoft/vcpkg "$HOME/src/vcpkg"
"$HOME/src/vcpkg/bootstrap-vcpkg.sh"
export VCPKG_ROOT="$HOME/src/vcpkg"
```

Homebrew 仅是安装 CMake/Ninja 的一种方式；任何 CMake 3.21+ 与支持 C++20
的 Xcode toolchain 均可。

Ubuntu 可执行 `sudo apt install ccache`，macOS 可执行 `brew install ccache`。
配置后用 `ccache --show-stats` 查看统计；CMake 找到时会输出 `Using ccache:`。

## 各平台构建

### Linux：系统依赖

Debug：

```bash
cmake --preset linux-system-debug
cmake --build --preset linux-system-debug
xvfb-run -a ctest --preset linux-system-debug
./build/linux-system-debug/kpt_gui
```

Release 将 preset 改为 `linux-system-release`。

### Linux：vcpkg

```bash
export VCPKG_ROOT=/absolute/path/to/vcpkg
cmake --preset linux-vcpkg-debug
cmake --build --preset linux-vcpkg-debug
xvfb-run -a ctest --preset linux-vcpkg-debug
```

Release preset 为 `linux-vcpkg-release`。Linux GUI 当前强制使用 X11；
Wayland session 需要 XWayland。

### Windows x64：vcpkg

先进入已激活 MSVC x64 environment 的终端：

```powershell
$env:VCPKG_ROOT = "C:\src\vcpkg"
cmake --preset windows-x64-vcpkg-debug
cmake --build --preset windows-x64-vcpkg-debug
ctest --preset windows-x64-vcpkg-debug
.\build\windows-x64-vcpkg-debug\kpt_gui.exe
```

Release preset 为 `windows-x64-vcpkg-release`。打包 workflow 会在 Windows
runner 上构建该 preset、生成 ZIP，并审计其中命令与 runtime dependencies。

### macOS arm64

```bash
export VCPKG_ROOT="$HOME/src/vcpkg"
cmake --preset macos-arm64-vcpkg-debug
cmake --build --preset macos-arm64-vcpkg-debug
ctest --preset macos-arm64-vcpkg-debug
```

Intel Mac 使用 `macos-x64-vcpkg-debug`。二者均有对应 `-release` preset。
arm64 preset 会构建完整 Metal GUI 与自动化 backend tests。

### 仅构建转换工具

此模式不构建 renderer、GUI 或 tests：

```bash
cmake -S . -B build/convert-only -G Ninja \
  -DKPT_BUILD_RENDER=OFF -DKPT_BUILD_GUI=OFF -DKPT_BUILD_TESTS=OFF
cmake --build build/convert-only --target kpt_convert kpt_batch_convert
```

### 仅构建无窗口渲染

此模式可构建 `kpt_render` 与 display-free `kpt_player --snapshot`，不链接
GLFW/OpenGL：

```bash
cmake -S . -B build/headless -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DKPT_BUILD_RENDER=ON -DKPT_BUILD_GUI=OFF -DKPT_BUILD_TESTS=OFF
cmake --build build/headless --target kpt_render kpt_player
```

### Preset 速查

| 平台 | Debug | Release | GUI |
|---|---|---|---|
| Linux system | `linux-system-debug` | `linux-system-release` | OpenGL/X11，已验证 |
| Linux vcpkg | `linux-vcpkg-debug` | `linux-vcpkg-release` | OpenGL/X11 |
| Windows x64 | `windows-x64-vcpkg-debug` | `windows-x64-vcpkg-release` | OpenGL，CI 编译打包 |
| macOS arm64 | `macos-arm64-vcpkg-debug` | `macos-arm64-vcpkg-release` | Metal，已验证 |
| macOS x64 | `macos-x64-vcpkg-debug` | `macos-x64-vcpkg-release` | Metal，Intel CI 编译打包 |

更完整的平台说明、字体、配置目录及验证证据见
[跨平台构建指南](docs/cross-platform-build.md)。

## 打包与发布

根目录 `VERSION` 是 CMake、DEB、DMG、Windows ZIP、VSIX 的统一版本真源。升版时执行：

```bash
./tools/set-version.sh 0.2.5
```

Ubuntu 可复现打包依赖 Docker：

```bash
./tools/package-deb.sh 22.04
./tools/package-deb.sh 24.04
./tools/package-deb.sh 26.04
```

macOS 13 或更高版本配置 `VCPKG_ROOT` 后，以 `./tools/package-macos.sh` 生成
ad-hoc 签名的 universal2 DMG。Windows 配置 Visual Studio 2022 与
`VCPKG_ROOT` 后，在 PowerShell 运行 `./tools/package-windows.ps1` 生成
portable x64 ZIP。激活 Emscripten 6.0.5 后，以
`./tools/package-vsix.sh` 生成
VSIX。所有产物写入 `artifacts/`。

Release 含 Ubuntu 22.04、24.04、26.04 三个 amd64 DEB、universal2 DMG、
Windows x64 ZIP、VSIX，以及覆盖上述六包的 `SHA256SUMS`。
GitHub 会将上传 DEB 文件名中的 `~` 规范为 `.`；包内 Debian version 仍使用
`-1~ubuntuXX.04`。

推送与 `VERSION` 一致的 `vX.Y.Z` tag 后，CI 会完成全部平台编译打包及轻量产物
审计，继而自动发布 GitHub Release。手动 workflow 仅上传 CI artifacts，不创建 Release。
macOS DMG 仅作 ad-hoc 签名，未 notarize；Gatekeeper 提示来源不明时，使用
Finder 右键菜单的“打开”。

本地打包脚本默认仍执行完整测试与 smoke checks。Release CI 设置
`KPT_PACKAGE_VERIFY=0`，publish jobs 仅负责编译打包。

## 工具

preset 构建产物位于 `build/<preset-name>/`。下列 `./build/<tool>` 为简写。
所有工具支持 `-h,--help` 与 `-l,--log-level`：
`0=error`、`1=warning`、`2=info`、`3=debug`，默认 `2`。GUI 启动信息、
OpenGL adapter、framebuffer、文件加载与任务结果会输出到终端；成功运行时
`0`、`1` 级别本就应保持安静。

### kpt_convert：单文件转换

```bash
./build/kpt_convert input.bin output.pcd
./build/kpt_convert input.pcd output.xyz
./build/kpt_convert input.pcd output.xyzi --ascii-flavor xyzi
```

位置参数为 `<input> <output>`。`--ascii-flavor` 仅接受
`xyz|xyzi|xyzrgb|xyzrgbi`，且必须与输出扩展名相同。

### kpt_batch_convert：批量转换

```bash
./build/kpt_batch_convert -i data/velodyne -o out_pcd -t pcd
./build/kpt_batch_convert -i data -o out_bin -t bin -g '*.pcd'
```

| 参数 | 含义 |
|---|---|
| `-i,--input-dir DIR` | 输入目录 |
| `-o,--output-dir DIR` | 输出目录；不存在时创建 |
| `-t,--to FMT` | 目标格式 |
| `-g,--glob PAT` | 匹配模式，默认 `*` |

### kpt_viewer：交互查看

使用与 `kpt_gui` 相同的 GPU renderer 与 workbench，不使用 PCL：

```bash
./build/kpt_viewer data/frame.pcd --colorby intensity --point-size 3
```

### kpt_player：序列播放

```bash
./build/kpt_player -i data/velodyne -g '*.bin' --fps 10
./build/kpt_player -i data/velodyne --snapshot out/frame \
  --snapshot-views front,top
```

`--snapshot PREFIX` 会改用无窗口 CPU renderer，导出 PNG 后退出。
snapshot 默认采用 robust 正交构图；`--snapshot-trim-percent 0` 可保留全部
有限点，透视效果可用
`--snapshot-projection perspective --snapshot-fov 120`。

### kpt_render：无窗口多视角 PNG

```bash
./build/kpt_render data/000123.pcd -o frame
./build/kpt_render frame.bin -o shot --views front,top \
  --color-by z --width 1920 --height 1080
```

输出名为 `<prefix>_<view>.png`。PNG 最大为 32 Mi pixels。
`--color-by auto` 依次选择有效 RGB、归一化 intensity 灰度、中性灰；
亦可显式选择 `rgb`、`intensity`、蓝绿红 Z 高度渐变或 `solid` 中性灰。
点云没有有效 RGB 时显式 `rgb` 会给出明确错误。CLI 会报告每张图的
visible pixel 数，若为零则告警。默认逐轴裁除最低及最高各 1% 的
order-statistic 尾部，丢弃范围外点，再以 5% margin 正交适配各视角；
日志会报告裁剪前后长宽高及保留点比例。

常用选项：

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `-o,--output-prefix P` | — | 输出文件名前缀（必填） |
| `--width N` | `640` | 图像宽度 |
| `--height N` | `480` | 图像高度 |
| `--projection MODE` | `orthographic` | `orthographic` 或 `perspective` |
| `--trim-percent P` | `1` | 每轴两端裁剪百分比；`0` 为禁用 |
| `--fov DEG` | `120` | 透视视场角；仅可配合 perspective |
| `--views LIST` | `all` | `all` 或逗号分隔视角 |
| `--color-by MODE` | `auto` | `auto`、`rgb`、`intensity`、`z`、`solid` |

旧构图可用 `--projection perspective --trim-percent 0 --fov 120`。

### kpt_gui：统一 workbench

```bash
./build/linux-system-debug/kpt_gui
xvfb-run -a ./build/linux-system-debug/kpt_gui --smoke-test
```

包含 Viewer、Player、Convert、Batch Convert、Render 五个 dockable tools；
Player 支持 seek、reset、正放、倒放与 loop。
viewport 采用 CloudCompare 的 object-centered 鼠标逻辑：左键拖动为绕所选
pivot 的轨迹球旋转，Shift+左键为 roll，右键为屏幕平面平移，中键点击拾取新的
旋转中心，滚轮为缩放；指针拖出 viewport 后仍保持捕获。另支持 fit、颜色模式、
point size 与
CloudCompare 的八个标准 view presets。

## 格式支持

统一内存类型为 `kpt::PointXYZRGBI`。格式转换只保留输入输出双方均能表达的
字段；缺失字段置零。

| 格式 | 扩展名 | 读取 | 写入 |
|---|---|---|---|
| KITTI BIN | `.bin` | little-endian float32 XYZI | 同格式 |
| PCD 0.7 | `.pcd` | ASCII、binary、LZF `binary_compressed` | little-endian binary |
| PLY 1.0 | `.ply` | ASCII、binary little/big endian | binary little-endian |
| LAS | `.las` | LAS 1.0–1.4 legacy point format 0–5；拒绝 LAZ 与 format 6–10 | LAS 1.2 format 0/2，坐标精度 0.01 |
| PTS | `.pts` | 点数 header + 一致的 3/4/6/7 列 | 点数 header + 7 列 XYZI RGB |
| OBJ | `.obj` | 点 vertex，支持 normalized RGB；忽略 mesh topology | 点 vertex + normalized RGB，不写 face |
| NPY | `.npy` | NumPy v1/v2、C-order、二维 float32、3/4/6/7 列 | little-endian float32 `(N, 7)` |
| XYZ | `.xyz` | x y z | x y z |
| XYZI | `.xyzi` | x y z intensity | 同格式 |
| XYZRGB | `.xyzrgb` | x y z r g b | 同格式 |
| XYZRGBI | `.xyzrgbi` | x y z r g b intensity | 同格式 |

详细 alias、字段损失规则与 parser 安全上限见
[Native point-cloud I/O design](docs/superpowers/specs/2026-07-28-native-pointcloud-io-design.md)。

`kpt_convert`、批量转换、原生 Viewer/Player、浏览器 workbench 与 VS Code
extension 均按上述十一种扩展名路由。`.label` 与 pose 属于序列附属输入，不是点云
codec；当前不支持 LAZ。

## 中文路径、字体与配置

UI 文件路径使用 UTF-8，并在平台边界转换为 native path。可用绝对字体路径
覆盖 CJK 字体：

```bash
export KPT_CJK_FONT=/absolute/path/to/NotoSansCJK-Regular.ttc
```

```powershell
$env:KPT_CJK_FONT = "C:\Fonts\思源黑体.ttc"
```

Dear ImGui 设置文件：

| 平台 | 路径 |
|---|---|
| Linux | `$XDG_CONFIG_HOME/kpt/imgui.ini`，否则 `$HOME/.config/kpt/imgui.ini` |
| Windows | `%APPDATA%\kpt\imgui.ini` |
| macOS | `~/Library/Application Support/kpt/imgui.ini` |

## 项目结构

```text
kitti_pointcloud_tools/
├── CMakeLists.txt
├── CMakePresets.json
├── cmake/                 # toolchain、warning、sanitizer
├── third_party/           # vendored dependencies
├── src/
│   ├── kpt/               # 点类型、codec、label、workflow、CPU renderer
│   ├── gui/               # portable app/model 与 GPU runtime
│   ├── platform/          # Linux/Windows/macOS path/font/settings
│   └── cli/               # CLI 入口
└── tests/
```

`kpt_core` 含自有点类型与 codec；`kpt_render` 增加 CPU renderer 与 stb PNG；
GUI 每次只链接一个 backend-specific target。

## License

BSD 3-Clause，见 [LICENSE](LICENSE)。Copyright (c) 2020, DengQi。
