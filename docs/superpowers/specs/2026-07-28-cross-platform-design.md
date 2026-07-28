# KPT Cross-Platform Build and GUI Architecture

> **Superseded dependency sections:** KPT now uses native point-cloud types
> and codecs. PCL/core/viewer statements below remain historical design
> context; see `2026-07-28-native-pointcloud-io-design.md` for the current
> data and I/O architecture.

Date: 2026-07-28

Status: Proposed

Target platforms: Linux, Windows 10 version 1903 or later/Windows 11, macOS 13+
Target architectures: x86-64 on Linux/Windows, arm64 and x86-64 on macOS

## 1. Executive Summary

KPT can become a three-platform project without changing its point-cloud
workflow or file-format model. The current platform coupling is concentrated in
the GUI executable, OpenGL point renderer, configuration/font discovery, and
one CMake block. The proposed design separates those concerns behind narrow
interfaces. Here, “platform-neutral” means free of OS/window/GPU APIs; it does
not mean dependency-free. The canonical point-cloud types remain PCL types and
the core also uses OpenCV and vendored spdlog.

The principal decisions are:

1. Use checked-in `CMakePresets.json` for reproducible project configurations
   and ignored `CMakeUserPresets.json` for machine-local paths such as
   `VCPKG_ROOT`.
2. Give each platform preset an explicit toolchain entry point. Toolchain files
   select compiler/SDK/architecture and integrate vcpkg; they do not select GUI
   behavior.
3. Use a vcpkg manifest for reproducible PCL and OpenCV builds on Windows and
   macOS. Keep a `linux-system` preset for distribution packages and permit
   macOS system packages as a developer convenience.
4. Separate platform services, GUI runtime, viewport model, and GPU renderer.
   Do not create a single “god” platform interface and do not spread `#ifdef`
   through `App`.
5. Keep GLFW as the initial window/input platform backend on all three systems.
   Select the renderer at configure time:
   - Linux: OpenGL 3.3.
   - Windows: OpenGL 3.3 first; Direct3D 11 remains a designed extension.
   - macOS: Metal.
6. Move camera, bounds, color-mode, and interaction behavior into a
   renderer-independent viewport model. OpenGL and Metal consume the same
   immutable frame description.
7. Build one graphics backend into each `pc_gui` binary. Runtime backend
   switching is out of scope.

This design deliberately treats “toolchain” and “graphics backend” as separate
axes:

```text
CMake preset
  ├─ toolchain: compiler + SDK + architecture + dependency provider
  └─ cache option: KPT_GUI_BACKEND={auto,opengl,metal}
```

## 2. Goals

- Configure, build, and test through named presets on Linux, Windows, and
  macOS.
- Preserve a system-package build for Linux users.
- Make Windows setup deterministic through vcpkg manifest mode.
- Provide a native Metal implementation for the macOS workbench.
- Keep core I/O, workflow, jobs, and headless rendering free of OS, window, and
  graphics-API conditionals. PCL/OpenCV remain explicit library dependencies.
- Preserve UTF-8 filenames at the UI boundary while using native filesystem
  paths internally.
- Make platform logic and viewport behavior independently testable.

## 3. Non-Goals

- Mobile, WebAssembly, Android, or iOS support.
- A plugin ABI for third-party renderers.
- Selecting OpenGL/Metal/D3D11 at runtime.
- Replacing PCL, OpenCV, Dear ImGui, GLFW, or ImGuiFileDialog.
- Porting `pc_viewer` and `pc_player` away from PCLVisualizer in the first
  milestone.
- Packaging installers, code signing, or macOS notarization in the first
  milestone.
- Wayland support in the first milestone; the initial Linux GUI remains X11.
- CI automation is intentionally deferred to follow-up work. This document
  specifies local presets, tests, and acceptance criteria only.
- Abstracting standard C++ facilities merely because implementations differ
  internally. `std::filesystem`, `std::thread`, and normal file I/O remain
  direct dependencies.

## 4. Current-State Audit

### 4.1 Existing strengths

- `kpt::workflow`, `kpt::io`, job scheduling, labels, and headless rendering
  contain little OS-specific code and already use filesystem paths. They are
  not dependency-free: `PointCloudIRGB` is a registered PCL type, `kpt` links
  PCL and OpenCV, and logging uses vendored spdlog.
- Platform-specific GUI work is localized under `src/gui`.
- Dear ImGui, GLFW, and ImGuiFileDialog are already vendored.
- The vendored Dear ImGui tree already contains GLFW, OpenGL3, Metal, Win32,
  and Direct3D 11 backends.
- `PointRenderer` already groups its public operations around viewport
  behavior, but its roughly 25 methods mix model, interaction, GPU, and test
  concerns. The split therefore changes `App` composition explicitly.
- GUI smoke tests and renderer behavior tests already exist on Linux.

### 4.2 Current coupling

| Location | Coupling | Consequence |
|---|---|---|
| `CMakeLists.txt` | Rejects every non-Linux GUI build | Windows/macOS cannot configure |
| `CMakeLists.txt` | Forces GLFW X11 and links `OpenGL::GL` | No Cocoa/Win32 window backend |
| `src/gui/pc_gui.cc` | Owns GLFW, OpenGL, ImGui lifecycle | Entry point cannot vary by renderer |
| `src/gui/pc_gui.cc` | Includes Linux `<GL/glcorearb.h>` | Does not compile on Windows/macOS |
| `src/gui/pc_gui.cc` | Uses XDG/HOME and Linux font paths | Settings and CJK font discovery are non-portable |
| `src/gui/point_renderer.*` | Owns camera and raw OpenGL resources together | Metal port would duplicate UI behavior |
| `src/gui/app.cpp` | Casts OpenGL texture integer to `ImTextureID` | Texture semantics leak into UI |
| GUI tests | Link raw OpenGL and use `xvfb-run` | No backend-neutral contract test |

### 4.3 Dependency observations

- PCL officially describes itself as cross-platform and documents Windows,
  macOS, and Linux deployment.
- PCL recommends vcpkg for Windows and Homebrew for macOS. For this project,
  vcpkg presets give a reproducible dependency graph, while Homebrew can remain
  an unpinned local option.
- PCL visualization brings VTK and substantially increases dependency build
  time. CMake targets should therefore distinguish core/headless tools from
  PCLVisualizer-based tools.
- The canonical point type in `src/kpt/types.hpp` directly depends on PCL.
  `kpt_core` therefore retains PCL common/I/O. PCL visualization moves behind
  an optional target. Provider-owned `pcl_io` transitive dependencies remain
  intact; depending on PCL packaging, these can include a broad VTK graph.
- The current project searches for OpenMP and mutates global compiler/linker
  flags, but no source uses an OpenMP pragma or runtime symbol. Phase 0 removes
  this inactive global dependency instead of inventing platform-specific
  OpenMP setup.
- The current global `include_directories`, `link_directories`, and
  `${PCL_LIBRARIES}` pattern obscures transitive requirements. Migration should
  prefer component-scoped `find_package` calls and imported targets where the
  installed PCL version exposes them.

## 5. Literature and Documentation Findings

The research used current upstream documentation rather than secondary build
guides.

### 5.1 CMake Presets

CMake introduced presets to share common configure settings. It explicitly
distinguishes checked-in `CMakePresets.json` from untracked,
machine-specific `CMakeUserPresets.json`. Configure presets support
`toolchainFile`, inheritance, host conditions, and build/test presets. The
`toolchainFile` field takes precedence over
`CMAKE_TOOLCHAIN_FILE`.

Design consequence:

- Check in all reproducible presets.
- Ignore `CMakeUserPresets.json`.
- Put `VCPKG_ROOT`, local SDK overrides, and private cache endpoints only in
  user presets or environment variables.
- Raise the project minimum to CMake 3.21 and use preset schema version 3.
  Version 3 provides `toolchainFile` and conditions; build/test presets arrived
  in version 2. Do not require workflow presets, which would raise the floor to
  CMake 3.25 and exclude Ubuntu 22.04's packaged CMake 3.22.

Source: [CMake Presets manual](https://cmake.org/cmake/help/latest/manual/cmake-presets.7.html).

### 5.2 vcpkg toolchain and manifest mode

vcpkg integrates through a CMake toolchain file. In manifest mode, the
toolchain detects `vcpkg.json` and installs the declared dependency graph.
Variables affecting vcpkg must be set before `project()`. Microsoft recommends
presets for selecting the toolchain and recommends `CMakeUserPresets.json` when
the vcpkg root is machine-specific.

vcpkg also defines a chainload mechanism:
`VCPKG_CHAINLOAD_TOOLCHAIN_FILE`. This is the supported way to combine its
dependency integration with a custom compiler/SDK toolchain.

Design consequence:

- Platform toolchain entry files set the target triplet and chainloaded native
  toolchain, then enter `vcpkg.cmake`.
- Do not set a vcpkg toolchain after `project()`.
- Pin `builtin-baseline` in `vcpkg.json`.
- Treat triplet/CRT/linkage changes as ABI changes that require a clean
  configure.

Sources:

- [vcpkg in CMake projects](https://learn.microsoft.com/en-us/vcpkg/users/buildsystems/cmake-integration)
- [vcpkg repository and toolchain implementation](https://github.com/microsoft/vcpkg)

### 5.3 Dear ImGui backend split

Dear ImGui defines two backend roles:

- Platform backend: input, cursor, timing, clipboard, and windows.
- Renderer backend: font/texture resources and draw-data rendering.

Its official examples include GLFW + OpenGL, GLFW + Metal, Win32 + D3D11, and
other combinations. The upstream guidance recommends reusing standard backends
instead of writing replacements.

Design consequence:

- Reuse `imgui_impl_glfw` on all initial targets.
- Reuse `imgui_impl_opengl3` on Linux/Windows.
- Reuse `imgui_impl_metal` on macOS.
- KPT only implements its point-cloud viewport renderer, not an ImGui renderer.

Sources:

- [Dear ImGui backend architecture](https://github.com/ocornut/imgui/blob/master/docs/BACKENDS.md)
- [Dear ImGui integration examples](https://github.com/ocornut/imgui/wiki/Getting-Started)

### 5.4 GLFW

GLFW supplies native Win32, Cocoa, X11/Wayland, context, and no-client-API
window paths. Native handles are available when a graphics API needs the
underlying platform window.

Design consequence:

- Keep a common GLFW event/input layer.
- OpenGL hosts request an OpenGL context.
- Metal hosts request `GLFW_NO_API`, access the Cocoa window, and attach a
  Metal-backed view/layer.
- Native-handle access stays inside the selected GUI runtime implementation.

Sources:

- [GLFW window guide](https://www.glfw.org/docs/latest/window.html)
- [GLFW native access](https://www.glfw.org/docs/3.0/group__native.html)
- [GLFW build guide](https://www.glfw.org/docs/latest/build.html)

### 5.5 Metal presentation model

Apple’s documented model renders through `MTKView` or a `CAMetalLayer`, obtains
a drawable, encodes render commands into a command buffer, presents the
drawable, and commits the buffer. Rendering to an offscreen texture does not
display it automatically.

Therefore viewport renderers may encode into offscreen textures, but only the
runtime can order those passes before Dear ImGui, target the drawable, present,
and commit the shared frame command buffer.

Design consequence:

- The Metal runtime owns device, command queue, drawable, and final ImGui pass.
- Each viewport renderer owns offscreen color/depth textures.
- A frame uses one runtime-owned command buffer; viewport passes execute before
  the final ImGui pass.
- `App` never owns `MTLDevice`, `CAMetalDrawable`, or command buffers.

Sources:

- [Using Metal to draw a view’s contents](https://developer.apple.com/documentation/metal/using-metal-to-draw-a-view%27s-contents)
- [Creating a custom Metal view](https://developer.apple.com/documentation/Metal/creating-a-custom-metal-view)
- [CAMetalLayer](https://developer.apple.com/documentation/QuartzCore/CAMetalLayer)

### 5.6 Platform data and fonts

- Linux configuration belongs under `$XDG_CONFIG_HOME`, falling back to
  `$HOME/.config`.
- Windows new code should query known folders with
  `SHGetKnownFolderPath`, not concatenate environment variables.
- macOS application support data belongs under the Foundation-provided
  Application Support directory.
- Fontconfig provides system font matching on Linux.
- DirectWrite exposes the Windows system font collection.
- Core Text provides font descriptors, matching, substitution, and font URLs on
  macOS.

Design consequence:

- Replace hard-coded font filenames with per-platform font resolvers.
- Keep `KPT_CJK_FONT` as the highest-priority explicit override.
- Return a native `std::filesystem::path`, plus face index for TTC collections.

Sources:

- [XDG Base Directory Specification](https://specifications.freedesktop.org/basedir/)
- [Windows known folders](https://learn.microsoft.com/en-us/windows/win32/shell/known-folders)
- [macOS Application Support directory](https://developer.apple.com/documentation/foundation/url/applicationsupportdirectory)
- [Fontconfig developer reference](https://fontconfig.pages.freedesktop.org/fontconfig/fontconfig-devel/)
- [DirectWrite system font collection](https://learn.microsoft.com/en-us/windows/win32/api/dwrite/nf-dwrite-idwritefactory-getsystemfontcollection)
- [Core Text overview](https://developer.apple.com/documentation/coretext/)

## 6. Architecture

### 6.1 Layering

```text
┌─────────────────────────────────────────────────────────────┐
│ App: tools, panels, workflows                               │
│ Depends on ImGui + abstract viewport/runtime services       │
├─────────────────────────────────────────────────────────────┤
│ Viewport model                 │ Jobs / UI event queue       │
│ Cloud vertices, bounds, camera, interaction, render style   │
│ Pure C++/Eigen; no GLFW, GL, Metal, Win32, Cocoa            │
├───────────────────────────┬─────────────────────────────────┤
│ GUI runtime              │ Platform services               │
│ window + frame lifecycle │ config paths + fonts + UTF-8    │
├───────────────────────────┼─────────────────────────────────┤
│ OpenGL runtime/renderer   │ Linux: XDG + Fontconfig        │
│ Metal runtime/renderer    │ Windows: Known Folder + DWrite │
│ future D3D11 renderer     │ macOS: Foundation + Core Text  │
├───────────────────────────┴─────────────────────────────────┤
│ GLFW / Dear ImGui backends / OS graphics APIs               │
├─────────────────────────────────────────────────────────────┤
│ PCL common/I/O + OpenCV + spdlog (core data/workflow layer) │
└─────────────────────────────────────────────────────────────┘
```

Dependencies point downward only. Platform services know nothing about ImGui or
point clouds. GPU renderers know nothing about file dialogs, jobs, or workflow.
PCL is an explicit data-layer dependency, not an OS abstraction.

### 6.2 Proposed source tree

```text
src/
  platform/
    services.hpp
    settings_store.hpp
    utf8_path.hpp
    linux/
      services.cpp
      fonts.cpp
    windows/
      services.cpp
      fonts.cpp
    macos/
      services.mm
      fonts.mm

  gui/
    app.hpp / app.cpp
    jobs/
      job_system.hpp / job_system.cpp
    viewport/
      model.hpp / model.cpp
      pcl_adapter.hpp / pcl_adapter.cpp
      renderer.hpp
      render_types.hpp
      test_access.hpp               # test-support target only
    runtime/
      runtime.hpp
      factory.hpp
      glfw_opengl_runtime.cpp
      glfw_metal_runtime.mm
    backend/
      opengl/
        point_renderer.hpp / point_renderer.cpp
      metal/
        point_renderer.hpp / point_renderer.mm
        point_shaders.metal
```

Platform implementations are selected by CMake. They are not all compiled and
selected at runtime. A `dx11/` subtree and cache value are added only if Phase 5
is approved and implemented.

### 6.3 Platform services

Avoid a broad `Platform` interface. Use interface segregation:

```cpp
namespace kpt::platform {

enum class PlatformErrorCode {
  ConfigurationDirectoryUnavailable,
  EnvironmentDecodeFailed,
  FontFileUnavailable,
  InvalidUtf8,
  NarrowApiEncodingUnavailable,
  PlatformInitializationFailed,
  SettingsIoFailed
};

struct PlatformError {
  PlatformErrorCode code;
  std::string message;
  std::error_code system_error;
};

// C++20 project utility implemented with std::variant. This is not
// std::expected, which is C++23.
template <class T>
using PlatformResult = Result<T, PlatformError>;

struct FontFace {
  std::filesystem::path file;
  int face_index = 0;
};

class Paths {
public:
  virtual ~Paths() = default;
  [[nodiscard]] virtual PlatformResult<std::filesystem::path>
  configDirectory() const = 0;
};

class Fonts {
public:
  virtual ~Fonts() = default;
  [[nodiscard]] virtual PlatformResult<std::optional<FontFace>>
  matchUiFont(std::u32string_view required_characters) const = 0;
};

class SettingsStore {
public:
  virtual ~SettingsStore() = default;
  [[nodiscard]] virtual PlatformResult<std::optional<std::string>>
  loadIni() const = 0;
  [[nodiscard]] virtual PlatformResult<void>
  saveIniAtomically(std::string_view contents) = 0;
};

class PlatformLifetime {
public:
  virtual ~PlatformLifetime() = default;
};

struct Services {
  // First member, therefore destroyed last.
  std::unique_ptr<PlatformLifetime> platform_lifetime;
  std::unique_ptr<Paths> paths;
  std::unique_ptr<Fonts> fonts;
  std::unique_ptr<SettingsStore> settings;
};

PlatformResult<Services> createServices();

} // namespace kpt::platform
```

`main` handles service-bootstrap failure and then owns `Services`; it constructs
`GuiRuntime` and `App` afterward. Services and runtime outlive `App` and both
renderers. `PlatformLifetime` is declared first in the aggregate so it is
destroyed last. POSIX implementations use a no-op lifetime; Windows uses it
for the COM apartment. Tests inject fake `Paths` and `Fonts`.
`Services` is only a composition-root bundle: methods are never added to it.
Each future capability gets its own interface after a real call site exists.
Clipboard, notifications, shell-open, and power management are not included
preemptively.

#### Platform implementations

| Service | Linux | Windows | macOS |
|---|---|---|---|
| Config directory | XDG spec | `SHGetKnownFolderPath(FOLDERID_RoamingAppData)` | Foundation Application Support |
| CJK font | Fontconfig match | DirectWrite collection/match | Core Text descriptor match |
| Explicit override | `KPT_CJK_FONT` | `KPT_CJK_FONT` | `KPT_CJK_FONT` |

Windows reads `KPT_CJK_FONT` with `_wgetenv` and constructs a native path
directly; it must not pass a possibly non-ASCII override through `std::getenv`.
The Windows platform bootstrap owns a UI-thread COM apartment initialized
before Known Folder/service calls and destroyed after services, runtime, and
App. This follows the Known Folders requirement to call `CoInitializeEx`
before `SHGetKnownFolderPath`; initialization/uninitialization is balanced on
the same thread. `S_OK` and `S_FALSE` are both owned and balanced;
`RPC_E_CHANGED_MODE` reuses the caller's apartment without uninitializing it;
every other HRESULT makes `createServices()` return
`PlatformInitializationFailed`.
The DirectWrite implementation obtains the matched `IDWriteFontFace` files and
uses the local font-file loader to resolve a filesystem path. Because packaged,
remote, or custom-loader fonts need not have a local path, Phase 1 includes a
spike and returns no match rather than inventing a filename. Core Text likewise
requires a descriptor with a resolvable font URL. Core Text has no public
collection face-index attribute, so the macOS adapter enumerates the local file
with FreeType, verifies required glyphs, and aligns exactly one face by its
PostScript name. Missing, non-local, or ambiguous matches return no match.
`FontFace::face_index` is assigned to `ImFontConfig::FontNo` before
`AddFontFromFileTTF`.

### 6.4 UTF-8 and native filesystem paths

Rules:

1. UI strings and logs are UTF-8 `std::string`.
2. Filesystem APIs receive `std::filesystem::path`.
3. Conversion occurs once at UI/platform boundaries through:

```cpp
PlatformResult<std::filesystem::path> pathFromUtf8(std::string_view value);
PlatformResult<std::string> pathToUtf8(
    const std::filesystem::path &value);
```

4. Win32 OS calls use wide-character APIs.
5. POSIX implementations preserve UTF-8 bytes.
6. No call site uses locale-dependent narrow Win32 APIs.
7. `setlocale(LC_ALL, "")` may format UI dates, but filesystem correctness must
   not depend on process locale.

All call sites follow this boundary table:

| Flow | Required conversion |
|---|---|
| ImGuiFileDialog UTF-8 result → filesystem | `pathFromUtf8` |
| Native path → dialog initial directory/display/log | `pathToUtf8` |
| Windows environment override → filesystem | `_wgetenv` → native `path` |
| POSIX environment override → filesystem | UTF-8 bytes → `pathFromUtf8` |
| Native path → Dear ImGui filename API | `pathToUtf8`, with owned string lifetime |

PCL 1.15 is a documented exception at the third-party ABI boundary: its
PCD/PLY readers accept only `std::string` and call narrow CRT file APIs on
Windows. Every first-party executable therefore embeds the Windows 10 1903+
`activeCodePage=UTF-8` manifest, and every PCL filename first passes through
`pathToUtf8ForNarrowApi`, which checks `GetACP() == CP_UTF8` and fails with
`NarrowApiEncodingUnavailable` when the process contract is missing.
`/utf-8` is also part of the MSVC/clang-cl first-party compile contract. This
is why Windows 10 1903 is the minimum; passing a UTF-8 string to PCL without
that process contract is insufficient. Native streams remain the path-bearing
API for first-party BIN/ASCII data. OpenCV image output uses `imencode`
followed by a native-path stream.

`dialog_paths` remains platform-neutral, but its public string contract is
UTF-8; constructing `fs::path(current)` or returning `path.string()` at its UI
boundary is forbidden. Tests cover selecting siblings and ancestors outside
the process working directory, plus round trips through paths and filenames
containing Chinese characters.

The vendored Dear ImGui 1.92.8 already converts UTF-8 filenames to wide strings
and calls `_wfopen` on MSVC Windows. The application still uses manual settings
persistence to make ownership and failure explicit:

1. Set `io.IniFilename = nullptr`.
2. Read/write the ini file through a `SettingsStore` using native
   `std::filesystem::path`.
3. Load with `ImGui::LoadIniSettingsFromMemory`.
4. When `io.WantSaveIniSettings` is set, call
   `ImGui::SaveIniSettingsToMemory`, persist atomically, then clear the flag.

Failure to obtain or write the configuration directory disables persistence,
logs `PlatformError`, and does not prevent startup.
`loadIni()` distinguishes “file not created yet” from I/O failure;
`saveIniAtomically()` writes a sibling temporary file and replaces the target
through the platform-safe implementation. Temporary names contain the process
ID and a random nonce, are opened with exclusive-create semantics, and are
written/flushed through that same handle before replacement. Concurrent
processes cannot truncate or reuse one another's temporary file. Invalid UTF-8
is a structured boundary error, never replacement-character data loss.

### 6.5 Viewport model

The current `PointRenderer` combines four responsibilities:

- cloud-to-vertex conversion;
- bounds and scalar range calculation;
- camera/input state;
- OpenGL resource and shader management.

Split it into a PCL adapter, pure viewport model, and GPU renderer.

```cpp
struct ViewportVertex {
  Eigen::Vector3f position;
  Eigen::Vector3f color;
  float intensity;
};

struct ViewportStyle {
  ColorBy color_by = ColorBy::Intensity;
  float point_size = 3.0F;
  Eigen::Vector3f background = Eigen::Vector3f::Zero();
  float scalar_min = 0.0F;
  float scalar_max = 1.0F;
};

struct ViewportFrame {
  Eigen::Matrix4f view_projection;
  ViewportStyle style;
};

struct ViewportCloudSnapshot {
  std::vector<ViewportVertex> vertices;
  CloudBounds bounds;
  std::uint64_t revision;
};

struct PixelExtent;

class ViewportModel {
public:
  void setCloud(std::shared_ptr<const ViewportCloudSnapshot>, CameraUpdate);
  void fit();
  void orbit(float dx, float dy);
  void pan(float dx, float dy);
  void zoom(float delta);
  void setView(View);
  void setStyle(ViewportStyle);

  [[nodiscard]] std::shared_ptr<const ViewportCloudSnapshot> cloud() const;
  [[nodiscard]] ViewportFrame frame(PixelExtent physical_pixels) const;
  [[nodiscard]] const CloudBounds &bounds() const;
  [[nodiscard]] std::uint64_t cloudRevision() const;
};

std::shared_ptr<const ViewportCloudSnapshot>
makeViewportCloudSnapshot(const PointCloudIRGBConstPtr &cloud,
                          std::uint64_t request_generation);
```

`makeViewportCloudSnapshot` lives in `kpt_viewport_pcl_adapter`, the sole
viewport target that sees PCL. The model owns no PCL or GPU handle. The UI
thread assigns a strictly increasing request generation, starting at 1, before
dispatching a job; revision 0 means no uploaded cloud. A completion older than
the newest accepted/requested generation is discarded. Camera and style
mutations do not change the cloud revision. Existing bounds and camera tests
move here and run without a graphics context.

`JobSystem` workers may load and transform PCL data, then post one immutable
snapshot through `UiEvents`. Creation, mutation, upload, render, and destruction
of both `ViewportModel` and `ViewportRenderer` occur only on the UI thread.
When several completed jobs queue before a frame, the UI keeps the newest
revision and performs at most one upload. No renderer method is thread-safe.
`upload` copies or finishes encoding a copy before its input span expires; it
never retains the span.

### 6.6 Viewport renderer

```cpp
enum class BackendKind { OpenGL, Metal };

struct PixelExtent {
  int width;
  int height;
};

struct ViewportTexture {
  ImTextureRef ref;
  ImVec2 uv0{0.0F, 0.0F};
  ImVec2 uv1{1.0F, 1.0F};
};

enum class RendererErrorCode {
  BackendMismatch,
  ResourceCreationFailed,
  EncodingFailed
};

struct RendererError {
  RendererErrorCode code;
  std::string message;
};

class FrameContext {
public:
  virtual ~FrameContext() = default;
  [[nodiscard]] virtual BackendKind backendKind() const noexcept = 0;
};

class ViewportRenderer {
public:
  virtual ~ViewportRenderer() = default;

  virtual Result<void, RendererError>
  upload(std::span<const ViewportVertex> vertices,
         std::uint64_t revision) = 0;
  virtual Result<void, RendererError>
  resize(PixelExtent physical_pixels) = 0;
  virtual Result<void, RendererError>
  render(const ViewportFrame &, FrameContext &) = 0;

  [[nodiscard]] virtual ViewportTexture texture() const = 0;
  [[nodiscard]] virtual PixelExtent extent() const = 0;
  [[nodiscard]] virtual BackendKind backendKind() const noexcept = 0;
};
```

The vendored Dear ImGui 1.92 API consumes `ImTextureRef`, not the legacy
`ImTextureID` directly. Backend-specific construction remains inside each
implementation:

- OpenGL: texture name encoded as required by `imgui_impl_opengl3`.
- Metal: `id<MTLTexture>` encoded as required by `imgui_impl_metal`.

The interface does not expose `GLuint` or `MTLTexture`. `ViewportTexture`
standardizes ImGui's top-left UI convention, so `App` contains no backend UV
flip. The texture and its `ImTextureRef` stay valid from draw-list recording
through `renderAndPresent`; resize occurs at a frame boundary and retains any
replaced texture until that frame's GPU work completes.

A zero width or height means “viewport suspended”: `resize` succeeds without
creating a zero-sized GPU resource, `render` is skipped, and `App` does not
record `ImGui::Image`. Restoring a positive extent recreates resources. Failed
upload/resize/render operations preserve the last valid renderer state where
possible and propagate `RendererError`; they are never reduced to a log-only
side channel.

`FrameContext` makes command encoding explicit without putting Objective-C
types in portable headers. The selected runtime creates both the context and
compatible renderers. `render` asserts a mismatched `BackendKind` in debug and
returns `RendererError` in release. The Metal-private context gives
its renderer access to the active command buffer/encoder factory; the
OpenGL-private context asserts that the expected context is current. No global
or thread-local command buffer is permitted. Context construction,
native-handle inspection, activation, and invalidation remain private to the
selected runtime/concrete renderer; test support uses an explicit friend
adapter. A caller retaining the returned base reference cannot reactivate it
after presentation.

### 6.7 GUI runtime

The runtime owns the window, graphics device/context, Dear ImGui platform and
renderer backends, and frame presentation.

```cpp
struct GuiRuntimeOptions {
  int width = 1440;
  int height = 900;
  std::string title = "KPT Workbench";
  bool visible = true;
  bool persist_settings = true;
  platform::Fonts *fonts = nullptr;
  platform::SettingsStore *settings = nullptr;
};

struct FramebufferMetrics {
  ImVec2 logical_size;
  PixelExtent framebuffer_size;
  ImVec2 scale;
};

enum class GuiErrorCode {
  InvalidOptions,
  InvalidState,
  WindowSystemUnavailable,
  GraphicsDeviceUnavailable,
  RendererCreationFailed,
  PresentationFailed,
  UnsupportedBackend,
  ShaderCompilationFailed,
  BackendMismatch
};

struct GuiError {
  GuiErrorCode code;
  std::string message;
  std::error_code system_error;
};

class GuiRuntime {
public:
  virtual ~GuiRuntime() = default;

  virtual Result<void, GuiError> initialize(const GuiRuntimeOptions &) = 0;
  virtual bool shouldClose() const = 0;
  virtual void pollEvents() = 0;
  virtual Result<std::reference_wrapper<FrameContext>, GuiError>
  beginFrame() = 0;
  virtual Result<void, GuiError> renderAndPresent() = 0;
  virtual void shutdown() noexcept = 0;
  [[nodiscard]] virtual FramebufferMetrics framebufferMetrics() const = 0;

  [[nodiscard]] virtual Result<std::unique_ptr<ViewportRenderer>, GuiError>
  createViewportRenderer() = 0;
};

std::unique_ptr<GuiRuntime> createGuiRuntime();
```

`Fonts` and `SettingsStore` are borrowed initialization services. Their
`Services` owner outlives the runtime. The runtime is the sole Dear ImGui font
and ini-settings owner: it loads once during initialization, observes
`WantSaveIniSettings` after each rendered frame, and performs the final flush
before destroying the ImGui context. The entry point never calls Dear ImGui
settings APIs. A load or save failure disables persistence for the rest of that
runtime instance without preventing startup or presentation.

The factory is implemented once per selected backend target. CMake validates
that exactly one backend target is linked and generates
`KPT_ACTIVE_GUI_BACKEND`; duplicate strong factory definitions would normally
be a link error, not a silent selection mechanism.

`App` owns two complete viewport sessions:

```cpp
struct ViewportSession {
  ViewportModel model;
  std::unique_ptr<ViewportRenderer> renderer;
  std::uint64_t uploaded_revision = 0;
};

App::App(ViewportSession viewport, ViewportSession trajectory);
Result<void, AppError>
App::draw(FrameContext &frame_context, FramebufferMetrics metrics);
```

The runtime constructs compatible renderers; `main` moves them into `App`.
Each UI frame has one ordering contract:

```text
apply queued model mutations
  → for main, then trajectory:
      upload only if cloud revision changed
      resize to physical pixel extent
      render(model.frame(), frame_context)
      ImGui::Image(renderer.texture().ref, logical size, uv0, uv1)
  → final ImGui pass
```

An accepted empty snapshot still advances its positive cloud revision and is
uploaded once. For the optional trajectory viewport it then suspends the
renderer with a zero extent and records no `ImGui::Image`, ensuring stale GPU
content is not retained when a sequence has no trajectory.

Projection aspect uses the physical pixel extent. ImGui layout uses logical
units. `FramebufferMetrics` is the conversion authority; on Retina, the Metal
runtime sets both the layer `contentsScale` and `drawableSize` from it. Metrics
are refreshed each frame and on monitor/window-scale callbacks, so moving a
window between displays cannot retain a stale scale. Dear ImGui platform
multi-viewports remain disabled in the first milestone; one GLFW window is the
only DPI authority.

Runtime lifecycle is a state machine:

```text
Created → Initialized → FrameActive → Initialized → Shutdown
```

Only one frame may be active. A successful `beginFrame` returns a context valid
until the matching `renderAndPresent`; nested frames and renderer use outside
that interval return structured errors. `shutdown` is idempotent and safe after
partial initialization. Every success or failure exit from
`renderAndPresent` invalidates the context and closes the active frame. If
`App::draw` fails, composition still calls `renderAndPresent` before reporting
the App error, so no frame remains active. Destruction waits for or safely
retires in-flight GPU resources.

### 6.8 OpenGL implementation

The OpenGL implementation is the compatibility bridge for Linux and Windows.

- Use GLFW OpenGL contexts.
- Use an explicit application GL loader such as GLAD. Dear ImGui’s embedded
  OpenGL loader is private to its backend and must not be relied on by KPT.
- Request OpenGL 3.3 core on Linux/Windows.
- Keep GLSL 330 shaders.
- Own VAO, VBO, program, framebuffer, color texture, and depth buffer inside
  `OpenGLPointRenderer`.
- Encode the current OpenGL vertical flip in `ViewportTexture::uv0/uv1`;
  `App` always consumes the normalized texture contract.

macOS OpenGL may remain an opt-in diagnostic preset during migration, but it is
not the final supported macOS backend because OpenGL is deprecated there.

### 6.9 Metal implementation

The Metal runtime is Objective-C++ and follows the official GLFW + Metal and
Apple presentation models.

#### Runtime ownership

- `GLFWwindow` created with `GLFW_CLIENT_API = GLFW_NO_API`.
- Cocoa native window/view access is confined to `glfw_metal_runtime.mm`.
- `id<MTLDevice>`, `id<MTLCommandQueue>`, Metal-backed view/layer.
- Dear ImGui GLFW platform backend and Metal renderer backend.
- One active command buffer per UI frame.
- Drawable acquisition and presentation.

#### Viewport ownership

- Immutable private-storage vertex buffer, replaced only when
  `cloudRevision()` changes.
- Uniform/ring buffer for view-projection and style.
- Private-storage BGRA8 color texture with shader-read and render-target usage.
- Private-storage depth texture.
- Point render pipeline state and depth-stencil state.
- No drawable ownership.

`ViewportVertex` is a CPU/model contract, not a GPU ABI. The Metal backend
packs it into a private trivially-copyable POD with explicit 16-byte fields;
compile-time size and offset assertions must match the MSL declaration. Eigen
object representation is never copied directly into a Metal buffer.

#### Point primitive and shader assets

The first Metal renderer issues `MTLPrimitiveTypePoint`. Its MSL vertex output
provides `float point_size [[point_size]]`; Apple documents point size as
undefined when that attribute is absent. The shared initial contract clamps
point size to `[1, 64]` pixels; the Phase 4 device spike must lower that portable
maximum if the minimum supported Mac cannot satisfy it. Both initial backends
render round points by discarding fragments outside the point-sprite radius,
preserving current OpenGL behavior. If testing finds
unacceptable device variance, a later measured change may expand each point
into an instanced camera-facing quad; this is not silently substituted during
Phase 4.

`point_shaders.metal` is compiled to a `.metallib` as part of the selected
backend target. Ninja builds use `xcrun -sdk macosx metal` followed by
`metallib`; Xcode builds use the native Metal source build rule. CMake declares
the library as a generated target output, copies it beside the executable (or
into the app bundle resources), and the runtime resolves it relative to the
executable/bundle rather than the working directory. Shader build and load
failures produce `ShaderCompilationFailed` with tool/runtime diagnostics.

#### Frame sequence

```text
poll events
  → acquire drawable / command buffer
  → ImGui new frame
  → App::draw(frame_context, framebuffer_metrics)
      → viewport model snapshots
      → offscreen point-cloud render passes, main then trajectory
      → ImGui::Image(offscreen texture)
  → ImGui::Render
  → final ImGui pass to drawable
  → present drawable
  → commit command buffer
```

The first Metal implementation uses one command buffer per frame and no
asynchronous resource streaming. Snapshot upload initially copies on the UI
thread and may stall on very large clouds; queued revisions are coalesced.
Staging/ring-buffer uploads follow profiling, not design speculation.
Both viewport renderers encode into that same command buffer and may neither
commit nor create a second one. Replaced textures, vertex/uniform buffers, and
other referenced objects enter a bounded retirement queue released by command
buffer completion handlers; normal presentation does not wait synchronously
for each frame.

All Metal-native types and `MetalFrameContext` construction remain in
backend-private Objective-C++ headers. Portable contracts expose only
`FrameContext`; test access uses the same explicit friend boundary as the
OpenGL backend.

### 6.10 Windows renderer evolution

Phase one uses GLFW + OpenGL to reduce simultaneous risk. The abstractions
support a later D3D11 implementation:

- either keep GLFW for input/window and obtain the Win32 window handle;
- or introduce a Win32 platform runtime while preserving `GuiRuntime`.

D3D11 is not required for initial Windows support. It becomes justified if
driver reliability, deployment, or remote-desktop behavior proves materially
better than OpenGL. Until that implementation lands, `dx11` is neither a
documented cache value nor an accepted configure value.

## 7. Build and Dependency Design

### 7.1 CMake options

```cmake
set(KPT_GUI_BACKEND "auto" CACHE STRING
    "GUI renderer: auto, opengl, metal")
set_property(CACHE KPT_GUI_BACKEND PROPERTY STRINGS
             auto opengl metal)

option(KPT_BUILD_GUI "Build Dear ImGui workbench" OFF)
option(KPT_BUILD_PCL_VIEWERS "Build PCLVisualizer CLI tools" ON)
option(KPT_BUILD_TESTS "Build tests" ON)
```

`auto` resolves as:

| Platform | Backend |
|---|---|
| Linux | `opengl` |
| Windows | `opengl` |
| macOS | `metal` |

The configure logic validates membership rather than relying on the advisory
`STRINGS` property. Unknown values, including the not-yet-implemented `dx11`,
fail with an explicit message. Phase 5 adds `dx11` to both validation and
`STRINGS` only in the same change that adds its target.

Vendored GLFW options are host-scoped:

```cmake
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
  set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
endif()
```

Windows and macOS do not receive Linux cache variables; GLFW selects Win32 and
Cocoa respectively. Wayland is a recorded non-goal, not an accidental global
setting.

### 7.2 Target decomposition

Proposed targets:

```text
kpt_core                 types, I/O, labels, workflow
kpt_render               CPU/headless OpenCV rendering
kpt_pcl_viewer           PCLVisualizer-dependent viewer/player
kpt_platform             exactly one OS implementation
kpt_jobs                 JobSystem and UiEvents
kpt_gui_contracts        POD render types + renderer/runtime interfaces
kpt_gui_model            PCL-free model; depends on contracts
kpt_viewport_pcl_adapter PCL cloud to immutable viewport snapshot
kpt_gui_backend_opengl   contracts + GLFW/OpenGL runtime/renderer
kpt_gui_backend_metal    contracts + GLFW/Metal runtime/renderer
kpt_gui_app              model + contracts + jobs; no backend dependency
kpt_gui_test_support     backend-private readback fixtures, tests only
pc_gui                   links app + platform + exactly one backend
```

Benefits:

- Core/headless tools retain PCL common/I/O and its provider-declared
  transitive dependencies, plus OpenCV where used, but do not request or
  directly link PCLVisualizer, GLFW, Metal, or OpenGL.
- A CLI-only configuration (`KPT_BUILD_GUI=OFF`,
  `KPT_BUILD_TESTS=OFF`) does not create `kpt_platform` or directly discover
  Fontconfig/Freetype; provider-owned PCL metadata may still do so
  transitively.
- Core tests do not need a graphics context.
- PCL visualization can be disabled independently.
- Platform and renderer contract tests link only what they need.

Phase 0 first splits today's monolithic `kpt` target. `kpt_core` contains the
canonical PCL point type and non-visualization components; `kpt_pcl_viewer`
contains `viewer.cpp`, `player.cpp`, and the PCL visualization/VTK rendering
link.
`KPT_BUILD_PCL_VIEWERS=OFF` must remove the visualization component from
`find_package(PCL ...)` and from the link graph. This split precedes any Windows
core acceptance test.

The migration also replaces global `include_directories`, `link_directories`,
and flag mutation with target-scoped include/link/compile options. It removes
the obsolete `CMAKE_POLICY_VERSION_MINIMUM 3.5`, conditions
`CATCH_CONFIG_NO_POSIX_SIGNALS` on POSIX, and defines warning policies for
GCC/Clang, AppleClang, MSVC, and clang-cl. OpenMP is removed unless a future
target has an actual pragma/runtime call.

### 7.3 Toolchain layout

```text
cmake/
  toolchains/
    linux-system.cmake
    linux-vcpkg.cmake
    windows-x64-vcpkg.cmake
    macos-arm64-vcpkg.cmake
    macos-x64-vcpkg.cmake
  triplets/
    kpt-x64-windows.cmake
    kpt-arm64-osx.cmake
    kpt-x64-osx.cmake
```

Rules:

- Presets select the entry toolchain file.
- Toolchain files contain compiler/SDK/architecture/dependency setup only.
- `KPT_GUI_BACKEND` is a preset cache variable, never a toolchain decision.
- vcpkg entry files validate `VCPKG_ROOT`, select an overlay triplet, and use
  vcpkg’s documented chainload mechanism when a native compiler toolchain is
  needed.
- Toolchain files are idempotent because CMake may evaluate them more than
  once.
- No toolchain path is hard-coded to a developer’s home directory.

Example entry shape:

```cmake
include_guard(GLOBAL)

if(NOT DEFINED ENV{VCPKG_ROOT})
  message(FATAL_ERROR "VCPKG_ROOT is required by this preset")
endif()

set(VCPKG_TARGET_TRIPLET "kpt-arm64-osx" CACHE STRING "")
set(VCPKG_OVERLAY_TRIPLETS
    "${CMAKE_CURRENT_LIST_DIR}/../triplets" CACHE PATH "")

# Optional custom compiler/SDK setup:
if(DEFINED KPT_NATIVE_TOOLCHAIN_FILE)
  set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE
      "${KPT_NATIVE_TOOLCHAIN_FILE}" CACHE FILEPATH "")
endif()

include("$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
```

Architecture and SDK values live in the matching overlay/native toolchain, not
in the top-level `CMakeLists.txt`. Overlay triplets pin target architecture,
`VCPKG_CRT_LINKAGE`, `VCPKG_LIBRARY_LINKAGE`, and the applicable
`VCPKG_CMAKE_SYSTEM_NAME`; changing any is an ABI change requiring a clean
build tree. Windows Ninja presets require an activated matching MSVC developer
environment and fail early if `cl` is unavailable. This is a documented local
prerequisite.

### 7.4 Presets

Committed configure presets:

```text
linux-system-debug
linux-system-release
linux-vcpkg-debug
linux-vcpkg-release
windows-x64-vcpkg-debug
windows-x64-vcpkg-release
macos-arm64-vcpkg-debug
macos-arm64-vcpkg-release
macos-x64-vcpkg-debug
macos-x64-vcpkg-release
```

Generator policy:

- Reproducible command-line presets use Ninja and an explicit
  `CMAKE_BUILD_TYPE`.
- Windows users first activate the MSVC developer environment, then invoke the
  same Ninja commands. This avoids coupling project presets to one Visual
  Studio generator version.
- Developers who require a Visual Studio or Xcode project add a local
  generator override in `CMakeUserPresets.json`.

Each has matching build and test presets. CMake 3.21-compatible commands are:

```bash
cmake --preset linux-system-debug
cmake --build --preset linux-system-debug
ctest --preset linux-system-debug
```

The same three commands apply to each platform preset. Host conditions hide
inapplicable presets. Workflow presets are deliberately omitted because they
require preset schema 6/CMake 3.25.

Example user preset:

```json
{
  "version": 3,
  "configurePresets": [
    {
      "name": "local-windows",
      "inherits": "windows-x64-vcpkg-debug",
      "environment": {
        "VCPKG_ROOT": "C:/src/vcpkg"
      }
    }
  ]
}
```

`CMakeUserPresets.json` is added to `.gitignore`.

### 7.5 vcpkg manifest

The manifest pins a `builtin-baseline`. Initial direct dependencies are PCL and
OpenCV. Project features prevent the heaviest visualization dependencies from
infecting core-only builds.

Conceptual shape:

```json
{
  "name": "kitti-pointcloud-tools",
  "version-string": "0.1.0",
  "builtin-baseline": "<pinned-vcpkg-commit>",
  "dependencies": [
    {
      "name": "pcl",
      "default-features": false
    },
    {
      "name": "opencv4",
      "default-features": false,
      "features": ["png"]
    }
  ],
  "features": {
    "pcl-viewers": {
      "dependencies": [
        {
          "name": "pcl",
          "default-features": false,
          "features": ["visualization"]
        }
      ]
    }
  }
}
```

The exact OpenCV and PCL features must be validated against the pinned baseline
during implementation; this design does not assume that feature names remain
stable across arbitrary vcpkg revisions.

Presets set `VCPKG_MANIFEST_FEATURES=pcl-viewers` only when
`KPT_BUILD_PCL_VIEWERS=ON`; each committed preset sets both values explicitly
so dependency resolution cannot disagree with the CMake target option.

## 8. Testing Strategy

### 8.1 Test pyramid

#### Platform-neutral unit tests

- Bounds and scalar ranges.
- Camera fit/orbit/pan/zoom/view presets.
- Cloud-to-vertex conversion.
- UTF-8/native path round trips.
- Dialog initial/result normalization.
- App behavior with fake viewport renderers.

These run on every OS without a GPU context.

#### Platform service contract tests

The same test suite runs against each selected implementation:

- config directory is absolute and writable or returns a structured error;
- explicit `KPT_CJK_FONT` override wins;
- a resolver result points to a readable font;
- required Chinese glyphs can be loaded;
- UTF-8 filename round trip preserves text.

#### Renderer contract tests

Each backend renders a deterministic synthetic cloud and validates:

- requested dimensions;
- at least one non-background pixel in a fixed center neighborhood after
  `fit`;
- color-mode changes;
- resize resource recreation;
- empty and non-finite clouds;
- repeated create/destroy without resource errors.

The test-only interface is explicit:

```cpp
struct Rgba8Image {
  PixelExtent extent;
  std::vector<std::uint8_t> pixels;
  std::size_t bytes_per_row;
};

class RendererTestAccess {
public:
  virtual ~RendererTestAccess() = default;
  virtual Result<Rgba8Image, RendererError>
  readColor(const ViewportRenderer &) = 0;
};

struct RendererTestFixture {
  std::unique_ptr<ViewportRenderer> renderer;
  std::unique_ptr<RendererTestAccess> readback;
};
```

Each backend test-support target returns this pair. The portable
`gui/viewport/test_access.hpp` contains only the backend-neutral interface and
fixture ownership. Context helpers, fixture factories, and concrete adapters
live in backend-private test-support headers and sources. The adapter is a
friend of the concrete renderer and performs `glReadPixels` for OpenGL. For
Metal, the adapter encodes a blit from the renderer's private
color texture to a row-aligned shared buffer in the same test command buffer,
commits once, waits for test completion, and reads that buffer. Production
textures remain private; production `ViewportRenderer` exposes neither raw
handle nor readback. Backend test-support targets, such as
`kpt_gui_opengl_test_support`, are never linked into `pc_gui`.

The contract is behavioral, not bit-exact. Extent and row layout are exact.
Background comparison uses 8-bit channel distance with tolerance 3; the center
test inspects a 5×5 neighborhood. Color-mode tests compare region statistics
with a declared minimum separation, not individual pixels. Depth format,
floating-point rounding, point rasterization edges, and texture origin may
differ across GPUs so long as the normalized behavior passes.
An empty cloud produces background only. Non-finite vertices are ignored and
must not poison bounds, matrices, depth, or subsequent valid draws. Failed
renderer results fail the test immediately.

#### Runtime smoke tests

- Linux OpenGL: `xvfb-run` with software rendering.
- Windows OpenGL: hidden GLFW window.
- macOS Metal: hidden/offscreen-capable startup where possible; otherwise a
  minimal visible-window lifecycle test.

Smoke tests validate startup, one frame, shutdown, and no backend error.
They do not call renderer readback from `App`. Pure model tests always run.
Backend contract tests run only for the selected backend:
`kpt_opengl_renderer_tests` links OpenGL, while
`kpt_metal_renderer_tests` links Metal and no OpenGL target. Phase 4 requires
the lower-level Metal offscreen contract test; hidden/visible GLFW lifecycle is
automatic only if the implementation spike proves it reliable, otherwise it is
a documented manual acceptance item.

## 9. Error Handling and Diagnostics

Platform/backend initialization uses the `PlatformError`, `GuiError`, and
`RendererError` result types defined above. No parallel `bool` or
`GuiInitError` channel exists.

Rules:

- Configure-time unsupported combinations fail in CMake.
- Runtime device/window failures produce one diagnostic with backend, OS,
  adapter/device where available, and nested system error.
- Missing CJK font is non-fatal: use default font and log the
  `KPT_CJK_FONT` override instruction.
- Configuration-directory failure disables persistence but does not prevent
  startup.
- Shader compilation errors include shader name and compiler log.
- Destructors and shutdown paths do not throw.

## 10. Migration Plan

### Phase 0: Baseline and build hygiene

- Record current Linux build/test results.
- Raise minimum CMake to 3.21 and select preset schema 3.
- Split `kpt_core` from `kpt_pcl_viewer` before cross-platform validation.
- Replace global includes, link directories, and flags with target-scoped
  usage requirements; remove inactive global OpenMP setup and the obsolete
  policy-version override.
- Add presets without changing user-visible runtime behavior.
- Add `CMakeUserPresets.json` to `.gitignore`.
- Add a pinned vcpkg manifest and validate a core/headless Windows configure
  with `KPT_BUILD_PCL_VIEWERS=OFF`.

Acceptance:

- Existing Linux commands still work.
- Linux configure/build/test presets pass locally.
- Windows core/headless CLI preset configures and builds without requesting or
  directly linking PCLVisualizer; PCL I/O transitive dependencies remain.

### Phase 1: Platform services

- Add `kpt_platform`.
- Move config-directory and font resolution out of `pc_gui.cc`.
- Add UTF-8 boundary helpers.
- Replace direct ImGui ini path ownership with `SettingsStore`.
- Implement Linux, Windows, and macOS service backends.
- Add service contract tests.

Acceptance:

- No Linux path or font literal remains in `pc_gui.cc`.
- Chinese path I/O tests pass on all three systems.
- Dialog tests select files in sibling and ancestor directories independent of
  the working directory.
- GUI starts without a CJK font and reports a useful warning.

### Phase 2: Viewport model extraction

- Move bounds, vertices, camera, and style to `ViewportModel`.
- Add the PCL-to-immutable-snapshot adapter and UI-thread/revision contract.
- Introduce `ViewportRenderer`, `FrameContext`, and a minimal OpenGL context
  adapter around the still-existing entry-point lifecycle.
- Rename the existing implementation to `OpenGLPointRenderer`.
- Make `App` own two model+renderer sessions.
- Replace raw texture integer access with `ViewportTexture::ref`.
- Move `centerPixelVisible` and all readback out of production/App code into
  backend test support.

Acceptance:

- Existing Linux GUI behavior is unchanged.
- Camera/model tests need no OpenGL context.
- OpenGL renderer passes the renderer contract suite.
- `App` contains no PCL, GPU handle, or backend-specific UV conversion.

### Phase 3: GUI runtime extraction

- Introduce `GuiRuntime`.
- Move ownership of the Phase 2 `FrameContext` into `GuiRuntime`; add structured
  initialization/frame errors and logical/framebuffer metrics.
- Move GLFW/OpenGL/ImGui lifecycle from `pc_gui.cc`.
- Reduce `pc_gui.cc` to option parsing, factories, and main loop.
- Add GLAD for KPT’s OpenGL calls.
- Enable GLFW Win32 build.

Acceptance:

- Linux OpenGL smoke test passes.
- Windows OpenGL smoke test passes.
- `App` contains no GLFW or OpenGL symbols.

### Phase 4: macOS Metal

- Add Objective-C++ runtime and Metal point renderer.
- Add MSL shaders.
- Compile/package `.metallib`; implement `[[point_size]]` circular-point
  rendering.
- Use GLFW no-API window and Dear ImGui Metal backend.
- Integrate and revalidate the Core Text/Foundation services delivered in
  Phase 1.
- Add Metal contract tests and the spike-selected lifecycle smoke test.

Acceptance:

- Native arm64 macOS configure/build/test passes from preset.
- Viewer and trajectory textures render correctly.
- Resize, Retina framebuffer scale, sleep/wake, and repeated startup/shutdown
  are manually verified.
- Metal contract readback passes its behavioral tolerances.
- Metal test targets and `pc_gui` link no OpenGL target or framework.
- No OpenGL library is linked by the Metal preset.

### Phase 5: Optional D3D11

- Evaluate D3D11 using measured Windows OpenGL failures or deployment needs.
- Implement only if evidence justifies it.
- Add the `dx11` cache value only with a complete runtime/renderer and contract
  target.

## 11. Risks and Mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| PCL/VTK vcpkg cold build is slow | Long first-time setup | Split viewer feature; binary caching remains future work |
| PCL imported targets vary by version | Link failures | Pin baseline; validate component targets; retain compatibility adapter temporarily |
| Metal texture lifetime differs from GL integer names | Invalid ImGui image | Return `ImTextureRef`; retain old resources through presentation |
| Retina logical/framebuffer sizes diverge | Blurry or clipped viewport | Runtime reports both sizes/scale; renderer and projection use physical pixels |
| Objective-C++ leaks into C++ headers | Portability erosion | PImpl and `.mm` implementation files; opaque interfaces |
| Font collection returns TTC or non-local face | Wrong glyph/no path | `FontNo`, local-file resolution, deterministic two-face TTC contract, explicit override fallback |
| `std::filesystem::path::string()` on Windows | Mojibake/path failures | Boundary table, `_wgetenv`, UTF-8 helpers, wide Win32 APIs, UTF-8 process manifest for PCL |
| Hidden GPU window differs by platform | Flaky smoke tests | Separate model tests from runtime smoke; platform-specific harness |
| Mega-abstraction accumulates unrelated APIs | Hard-to-test design | Interface segregation; add services only at real call sites |
| Backends drift in visual behavior | Inconsistent UI | Shared model; behavior thresholds rather than pixel identity |
| UI-thread GPU upload stalls | Dropped frames on large clouds | Coalesce revisions; profile before staging uploads |

## 12. Alternatives Considered

### 12.1 Scatter `#ifdef` through current files

Rejected. It minimizes initial file count but couples platform, renderer, and
application behavior. It also makes tests platform-exclusive.

Allowed use: one source-selection block in CMake and small implementation-local
compiler guards.

### 12.2 OpenGL on every platform permanently

Rejected as the final architecture because OpenGL is deprecated on macOS.

Accepted as a migration bridge for Windows and as an optional macOS diagnostic
backend.

### 12.3 Native window systems on every platform immediately

Rejected. GLFW already supplies the platform/input role and Dear ImGui provides
an official GLFW backend. Replacing it with X11/Wayland, Win32, and Cocoa
implementations triples unrelated work.

### 12.4 One `Platform` interface containing all OS and GPU behavior

Rejected. Configuration paths, fonts, window lifecycle, and point rendering
change for different reasons and have different test strategies.

### 12.5 Compile all GPU backends and choose at runtime

Rejected for the initial design. It increases binary dependencies, complicates
Objective-C++ and framework linkage, and expands the test matrix. One preset
produces one backend-specific GUI binary.

### 12.6 Use only system packages on every platform

Rejected because Windows and macOS dependency versions would vary by machine.
Linux system packages remain supported; vcpkg presets provide the reproducible
alternative.

## 13. Definition of Done

Cross-platform support is complete when:

- `cmake --preset`, `cmake --build --preset`, and `ctest --preset` are the
  documented CMake 3.21-compatible build path.
- Linux, Windows, and macOS presets pass from clean target-platform machines.
- Core and headless tools neither request nor directly link PCLVisualizer or
  GUI dependencies; remaining PCL I/O provider dependencies, OpenCV, and
  spdlog dependencies are explicit.
- `App`, workflow, jobs, and I/O contain no OS or graphics API conditionals.
- Linux/Windows OpenGL and macOS Metal satisfy the same behavior contract with
  documented tolerance; no bit-exact cross-GPU promise is made.
- UTF-8 Chinese paths and filenames pass round-trip tests on all platforms.
- Configuration and CJK font discovery use native platform services.
- macOS Metal links no OpenGL framework.
- Viewport model/renderers obey the documented UI-thread, frame-context,
  revision, DPI, and texture-lifetime contracts.
- Unsupported backend/platform combinations fail at configure time.
- The README contains per-platform preset commands and known limitations.

## 14. Open Questions Requiring Implementation Spikes

1. Which pinned vcpkg baseline yields a stable PCL + VTK graph on all target
   platforms?
2. Does the selected PCL package expose consistent imported component targets,
   or is a temporary CMake compatibility target required?
3. Does GLFW + Metal with the current vendored versions behave correctly under
   Retina resize and macOS sleep/wake? Basic Cocoa-window/CAMetalLayer
   attachment is already demonstrated by the vendored GLFW+Metal example and
   does not require a GLFW patch.
4. Can macOS execute a reliable hidden-window Metal smoke test, or should the
   automated test use a lower-level offscreen device test plus manual window
   verification?
5. Can DirectWrite/Core Text matching reliably resolve a local font file and
   TTC face index for the target system collections? If not, the resolver
   returns no match and directs users to `KPT_CJK_FONT`.

Each question should be answered by a small build or runtime spike before the
corresponding migration phase is committed.

## 15. Product Decision Before Windows Release

Decide whether the first Windows release ships PCLVisualizer executables or
defers them until the core and ImGui workbench are stable. This is a product
scope decision, not an implementation spike, and does not block the
`KPT_BUILD_PCL_VIEWERS=OFF` architecture.
