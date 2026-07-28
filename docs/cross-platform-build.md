# Cross-platform build guide

This guide documents supported preset entry points and separates implemented
source support from results actually verified on target hardware.

## Support status

| Platform | Architecture | GUI backend | Source status | Verification |
|---|---|---|---|---|
| Linux | x86-64 | OpenGL 3.3 | Implemented; initial GUI is X11-only | Verified with system packages on 2026-07-28 |
| Windows 10 1903+/11 | x86-64 | OpenGL 3.3 | Implemented | Not run on a Windows host |
| macOS 13+ | arm64, x86-64 | Metal | Planned; implementation is gated | Not built or run on a Mac |

The macOS presets intentionally set `KPT_BUILD_GUI=OFF`. Selecting Metal with
the current source fails at configure time instead of producing a partial GUI.
Direct3D 11 is not a selectable backend.

## Common prerequisites

- CMake 3.21 or newer;
- Ninja (validated with 1.11.1);
- a C++20 compiler;
- Git;
- Eigen, plus OpenCV when `KPT_BUILD_RENDER=ON` or `KPT_BUILD_GUI=ON`;
- Freetype and platform GUI development libraries when building tests or the
  GUI.

The committed presets write to `build/<preset-name>`. Debug and release use
separate directories. Tests are enabled by default.

`KPT_BUILD_RENDER` defaults to `ON` and builds `pc_render`. Setting it to
`OFF` omits that executable and its render tests. OpenCV is still required
when the GUI is enabled because the Render panel shares `kpt_render`.
Conversion-only builds with render, GUI, and tests disabled do not discover
OpenCV.

vcpkg presets use manifest mode and the port baseline pinned in `vcpkg.json`.
OpenCV lives in the opt-in manifest `render` feature; committed vcpkg presets
enable it to match their renderer/GUI options. A conversion-only vcpkg
configure must also set `VCPKG_MANIFEST_FEATURES` to an empty value.
Install and bootstrap vcpkg, then expose its checkout:

```bash
export VCPKG_ROOT=/absolute/path/to/vcpkg
```

```powershell
$env:VCPKG_ROOT = "C:\absolute\path\to\vcpkg"
```

The toolchain wrappers reject an unset or invalid `VCPKG_ROOT`. To avoid a
global environment variable, create the ignored `CMakeUserPresets.json` and
inherit the desired project configure preset:

```json
{
  "version": 3,
  "configurePresets": [
    {
      "name": "linux-vcpkg-debug-local",
      "inherits": "linux-vcpkg-debug",
      "environment": {
        "VCPKG_ROOT": "/absolute/path/to/vcpkg"
      }
    }
  ]
}
```

Use the local name for configure; build and test can target its generated
directory directly, or define matching user build/test presets.

## Native point-cloud I/O

`kpt_core` uses plain in-tree point types and native BIN, text, PCD and PLY
codecs. It neither discovers nor links PCL or VTK. The retired `pc_viewer` and
`pc_player` executables are replaced by the Viewer and Player panels in
`pc_gui`.

| Family | Reader | Writer |
|---|---|---|
| KITTI BIN | little-endian float32 XYZI | same |
| XYZ text family | exact 3/4/6/7-column schema selected by extension | locale-independent text with float round-trip precision |
| PCD 0.7 | ASCII, binary, LZF binary-compressed | little-endian binary |
| PLY 1.0 | ASCII, binary little-endian, binary big-endian | binary little-endian |

Detailed field aliases, loss rules and parser limits are specified in
[`2026-07-28-native-pointcloud-io-design.md`](superpowers/specs/2026-07-28-native-pointcloud-io-design.md).

When both `KPT_BUILD_GUI=OFF` and `KPT_BUILD_TESTS=OFF`, KPT also skips direct
Fontconfig/Freetype discovery and does not create `kpt_platform`. OpenCV is
required only when the separately scoped renderer or GUI Render panel is
built.

## Linux

The system-package preset enables the GUI and tests, so it requires GCC or
Clang plus OpenCV, Freetype, Fontconfig, OpenGL, and X11 development
packages. On Ubuntu, a representative package set is:

```bash
sudo apt install build-essential cmake ninja-build pkg-config \
  libopencv-dev libfreetype-dev libfontconfig1-dev \
  libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev \
  libxcursor-dev libxi-dev xvfb
```

Confirm that the distribution's `cmake --version` is at least 3.21.

System packages:

```bash
cmake --preset linux-system-debug
cmake --build --preset linux-system-debug
xvfb-run -a ctest --preset linux-system-debug
xvfb-run -a ./build/linux-system-debug/pc_gui --smoke-test
```

Release uses `linux-system-release`. The vcpkg alternatives are:

```bash
export VCPKG_ROOT=/absolute/path/to/vcpkg
cmake --preset linux-vcpkg-debug
cmake --build --preset linux-vcpkg-debug
xvfb-run -a ctest --preset linux-vcpkg-debug
```

Linux GUI support currently forces GLFW X11 and disables Wayland. A native
Wayland session therefore needs XWayland; native Wayland is outside this
milestone.

## Windows

Requirements:

- Windows 10 version 1903 or newer, or Windows 11;
- Visual Studio 2022 with Desktop development with C++;
- CMake and Ninja on `PATH`;
- a bootstrapped vcpkg checkout.

Open an x64 Native Tools Command Prompt for VS, or otherwise activate the MSVC
x64 developer environment. Then run:

```powershell
$env:VCPKG_ROOT = "C:\src\vcpkg"
cmake --preset windows-x64-vcpkg-debug
cmake --build --preset windows-x64-vcpkg-debug
ctest --preset windows-x64-vcpkg-debug
```

Release uses `windows-x64-vcpkg-release`. The preset builds the OpenGL
workbench.

Every executable embeds `activeCodePage=UTF-8` and `longPathAware=true`.
Windows platform services use wide Win32 APIs and read `KPT_CJK_FONT` through
the wide environment. Native point-cloud codecs open
`std::filesystem::path` directly and do not cross a narrow third-party
filename ABI. Windows 10 version 1903 remains the declared product minimum;
native Windows execution has not yet verified it.

This path has source-level and configure-contract coverage, but no Windows
configure, build, test, driver, or file-dialog result has been recorded yet.

## macOS

Requirements:

- macOS 13 or newer;
- Xcode command-line tools (`xcode-select --install`);
- CMake, Ninja, and a bootstrapped vcpkg checkout.

On Apple Silicon:

```bash
export VCPKG_ROOT="$HOME/src/vcpkg"
cmake --preset macos-arm64-vcpkg-debug
cmake --build --preset macos-arm64-vcpkg-debug
ctest --preset macos-arm64-vcpkg-debug
```

On a real Intel Mac:

```bash
export VCPKG_ROOT="$HOME/src/vcpkg"
cmake --preset macos-x64-vcpkg-debug
cmake --build --preset macos-x64-vcpkg-debug
ctest --preset macos-x64-vcpkg-debug
```

The current presets build core/headless targets only. The Metal renderer,
`.metallib` packaging, GLFW/CAMetalLayer runtime, contract readback, Retina,
sleep/wake, and repeated-lifecycle verification remain unimplemented. An
Apple Silicon cross-build or Rosetta run cannot replace Intel GPU verification
for the x86-64 release row.

## Chinese filenames, font override, and settings

UI strings cross the filesystem boundary as UTF-8. Platform code converts them
to native paths, and dialog tests cover current, parent, grandparent, sibling,
and Chinese names. Platform-specific round trips still require execution on
each target OS before cross-platform completion can be claimed.

The application uses a platform-specific font service. `KPT_CJK_FONT` has
highest priority; without it, Fontconfig, DirectWrite, or Core Text performs
native discovery. Set the override to an absolute `.ttf`, `.ttc`, or other
supported local font file:

```bash
export KPT_CJK_FONT=/absolute/path/to/NotoSansCJK-Regular.ttc
```

```powershell
$env:KPT_CJK_FONT = "C:\Fonts\思源黑体.ttc"
```

Missing CJK coverage is non-fatal; the GUI logs the override instruction and
continues with its default font.

Dear ImGui settings are loaded and atomically saved at:

| Platform | Settings file |
|---|---|
| Linux | `$XDG_CONFIG_HOME/kpt/imgui.ini`, or `$HOME/.config/kpt/imgui.ini` |
| Windows | `%APPDATA%\kpt\imgui.ini` |
| macOS | `~/Library/Application Support/kpt/imgui.ini` |

If the configuration directory or settings file is unavailable, persistence
is disabled for that run without preventing startup.

## Known gates and limitations

- Native Linux GUI support is X11-only.
- Windows support is implemented but unverified on target hardware.
- macOS native services exist in source, but Metal GUI support is not
  implemented and macOS builds are unverified.
- vcpkg dependency resolution and cold-build time are unverified in this
  environment because `VCPKG_ROOT` was unavailable.
- CI automation, installers, signing, and macOS notarization are deferred.

## Acceptance evidence

Only executed results appear as verified. Historical pre-native-codec evidence
remains in `docs/build-baseline.md`.

| Platform/preset | Environment | Configure | Build | Tests | GUI/backend evidence |
|---|---|---:|---:|---:|---|
| `linux-system-debug` | Ubuntu 24.04 x86-64; Linux 6.17.0; CMake 4.3.2; Ninja 1.11.1; GCC 13.3.0; OpenCV 4.6.0; Fontconfig 2.15.0; Freetype 2.13.2 | Pass, fresh, 2026-07-28 | Pass, 84/84 Ninja edges | Pass, 7/7 under `xvfb-run` | Native codecs built; `pc_gui --smoke-test` passed; selected OpenGL backend |
| `linux-vcpkg-debug` | No `VCPKG_ROOT` on available host | Not run | Not run | Not run | Not run |
| `windows-x64-vcpkg-debug` | Available host is Linux, not Windows | Not run | Not run | Not run | Source support only |
| `macos-arm64-vcpkg-debug` | Available host is Linux, not macOS; no Xcode tools | Not run | Not run | Not run | Metal not implemented |
| `macos-x64-vcpkg-debug` | Available host is Linux, not macOS; no Xcode tools | Not run | Not run | Not run | Requires suitable Intel Mac |

The post-native-codec Linux acceptance used:

```bash
cmake --fresh --preset linux-system-debug
cmake --build --preset linux-system-debug -j2
xvfb-run -a ctest --preset linux-system-debug
xvfb-run -a ./build/linux-system-debug/pc_gui --smoke-test
```

A separate `KPT_BUILD_RENDER=OFF`, `KPT_BUILD_GUI=OFF` conversion-only
configure and 21-edge build completed without OpenCV discovery. A
render-disabled, GUI-disabled test build compiled `kpt_tests` without
`render_test.cpp` or OpenCV and passed its native codec/CLI-helper tests.

Architectural boundary searches found no PCL source, target, manifest or link
entry; no OS/GPU conditionals in `App`, jobs, core/workflow/I/O, or portable
viewport code; no raw GPU type in App/portable viewport headers; and no
locale-dependent path conversion in portable GUI code. These are Linux
source/build facts, not Windows or macOS execution evidence.
