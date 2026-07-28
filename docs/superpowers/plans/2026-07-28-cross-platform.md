# KPT Cross-Platform Implementation Plan

> **Superseded dependency sections:** The completed plan below records the
> former PCL-based architecture. Native point-cloud types/codecs and retirement
> of the PCLVisualizer tools are specified in
> `../specs/2026-07-28-native-pointcloud-io-design.md`.

> **For agentic workers:** Execute tasks in order. Use checkbox (`- [ ]`)
> tracking. Do not start a platform-gated task without access to that target
> platform. Keep each task independently buildable and commit only after its
> stated verification passes.

**Goal:** Build KPT core/headless tools and the Dear ImGui workbench on Linux,
Windows, and macOS; preserve Chinese paths and filenames; keep Linux/Windows on
OpenGL initially and use Metal on macOS.

**Architecture:** Separate direct PCLVisualizer use from the PCL-based core;
preserve provider-owned PCL I/O dependencies; select
compiler/dependencies through presets and toolchain files; isolate native paths,
fonts, and settings behind platform services; split viewport state from GPU
encoding; make runtime-owned `FrameContext` explicit; link exactly one GUI
backend into each `pc_gui`.

**Tech stack:** C++20, CMake 3.21+, CMake Presets schema 3, Ninja, vcpkg
manifest mode, PCL, OpenCV, Eigen, spdlog, Catch2 v2, GLFW, Dear ImGui 1.92.8,
ImGuiFileDialog, OpenGL 3.3, Metal/MSL.

**Source design:**
`docs/superpowers/specs/2026-07-28-cross-platform-design.md`

**Review audit:**
`docs/superpowers/specs/2026-07-28-cross-platform-review-audit.md`

## Global Constraints

- CI automation is out of scope.
- Initial backend values are only `auto`, `opengl`, and `metal`; do not expose
  `dx11`.
- Toolchain files select compiler, SDK, architecture, and dependency provider.
  They never select the GUI backend.
- “Core platform-neutral” means no OS/window/GPU APIs. Core still depends on
  PCL common/I/O, OpenCV where used, and spdlog.
- `KPT_BUILD_PCL_VIEWERS=OFF` must prevent requesting or directly linking
  `pcl_visualization`. Provider-owned `pcl_io` transitive dependencies remain
  intact; some PCL packages attach a broader VTK graph to that imported target.
- UI strings are UTF-8. Filesystem work uses `std::filesystem::path`.
- Windows environment/path APIs use wide strings. Never use `path.string()` at
  a UI boundary or `std::getenv` for a possibly non-ASCII Windows path.
- Windows support starts at Windows 10 version 1903 because PCL's narrow
  filename ABI relies on an embedded `activeCodePage=UTF-8` process manifest.
- MSVC and clang-cl first-party targets inherit `/utf-8`.
- `ViewportModel`, `ViewportRenderer`, `GuiRuntime`, and GPU resources are
  UI-thread-only.
- `FrameContext` is valid only between `beginFrame()` and
  `renderAndPresent()`.
- Renderer contract tests compare behavior with declared tolerances, never
  bit-exact pixels across GPUs.
- Follow TDD where a test is runnable on the current host: add the failing test,
  observe the intended failure, implement, then rerun.
- Preserve unrelated worktree changes. Stage explicit paths, not `git add -A`.
- Commands below are portable canonical commands. When executing in an
  environment that provides the optional `rtk` wrapper, the agent may prefix
  commands locally; Windows/macOS users and clean build hosts must not need
  `rtk`.
- Every task configures its own `build/task-NN` directory unless it explicitly
  uses a named preset. Never infer success from a previous task's CMake cache.
- To resume after interruption: inspect the task checkbox and explicit commit,
  discard no user changes, reconfigure that task's own build directory, rerun
  its stated verification, then continue. A platform-gated task may remain
  pending while an independent branch proceeds.

## Execution Dependency DAG

Task numbers describe the document, not one mandatory cross-machine sequence:

```text
Linux critical path:
1 → 2 → 3 → 4 → 5 → 6 → 9 → 10 → 11 → 12 → 15 (Linux portion)

Platform-service gates after Task 6:
6 → 7 (Windows services)
6 → 8 (macOS services)

Final platform backends:
7 + 12 → 13 (Windows OpenGL)
8 + 12 → 14 (macOS Metal)

Final acceptance:
13 + 14 + Linux portion → 15 (all-platform completion)
```

Tasks 7 and 8 may run in parallel with Linux Tasks 9–12, or later when target
machines become available. Their absence never blocks Linux architectural
work. Task 15 may record partial Linux evidence, but its completion checkbox
remains open until all target-platform gates pass.

## Intended Target Graph

```text
kpt_core
  ├─ kpt_render
  ├─ kpt_pcl_viewer                # optional PCL visualization/VTK
  └─ kpt_viewport_pcl_adapter

kpt_gui_contracts
  ├─ kpt_gui_model
  ├─ kpt_gui_backend_opengl
  └─ kpt_gui_backend_metal

kpt_jobs
kpt_common                         # Result and dependency-free shared helpers
kpt_platform                       # exactly one OS implementation
kpt_gui_app                        # model + contracts + jobs

pc_gui
  └─ app + platform + exactly one backend
```

---

## Task 1: Capture Baseline and Add Build-Graph Assertions

**Files:**

- Create: `docs/build-baseline.md`
- Create: `cmake/KptTargetAssertions.cmake`
- Modify: `CMakeLists.txt`

**Produces:**

- Reproducible record of current Linux configure/build/test behavior.
- Pure CMake helper tests for future backend resolution; no production option
  is wired in this baseline task.

- [x] **Step 1: Record current environment**

```bash
cmake --version
c++ --version
cmake -S . -B build/cross-platform-baseline \
  -DCMAKE_BUILD_TYPE=Debug \
  -DKPT_BUILD_GUI=ON
cmake --build build/cross-platform-baseline -j
ctest --test-dir build/cross-platform-baseline --output-on-failure
```

Write compiler, CMake, dependency versions, commands, passing test count, and
known GUI requirements into `docs/build-baseline.md`.

- [x] **Step 2: Add a pure helper self-test**

Add a `cmake -P` self-test that calls the resolver with explicit input values,
including an unknown backend, without assuming `KPT_GUI_BACKEND` already exists
in the top-level project. Describe the future API:

```cmake
kpt_resolve_gui_backend(
  REQUESTED "${KPT_GUI_BACKEND}"
  SYSTEM "${CMAKE_SYSTEM_NAME}"
  OUT_VAR KPT_ACTIVE_GUI_BACKEND)
```

- [x] **Step 3: Implement assertion helpers**

Allowed initial values:

```text
auto opengl metal
```

Rules:

- Linux: `auto/opengl`
- Windows: `auto/opengl`
- Apple: `auto/metal`, plus explicit diagnostic `opengl` only if enabled by a
  separate migration option
- all unknown values, including `dx11`: configure-time fatal error

Production option creation and target selection are deliberately deferred to
Task 3, when the relevant CMake branch is modified.

- [x] **Step 4: Verify**

```bash
cmake -P cmake/KptTargetAssertions.cmake
cmake -S . -B build/task-01 -DKPT_BUILD_GUI=OFF
cmake --build build/task-01 -j
ctest --test-dir build/task-01 --output-on-failure
```

- [x] **Step 5: Commit**

```bash
git add CMakeLists.txt cmake/KptTargetAssertions.cmake docs/build-baseline.md
git commit -m "test(build): capture cross-platform baseline"
```

---

## Task 2: Split Core, Headless Render, and PCLVisualizer Targets

**Files:**

- Modify: `CMakeLists.txt`
- Modify: `src/cli/pc_viewer.cc`
- Modify: `src/cli/pc_player.cc`
- Modify only if required by target boundaries:
  `src/kpt/viewer/*`, `src/kpt/player/*`

**Produces:**

- `kpt_core`: types, I/O, labels, workflow.
- `kpt_render`: CPU/OpenCV rendering.
- `kpt_pcl_viewer`: viewer/player and PCL visualization/VTK.
- `KPT_BUILD_PCL_VIEWERS`.
- Temporary source-compatible `kpt` alias while all in-tree consumers migrate.

- [x] **Step 1: Add a failing core-only graph check**

Configure with:

```bash
cmake -S . -B build/task-02-core \
  -DKPT_BUILD_GUI=OFF \
  -DKPT_BUILD_PCL_VIEWERS=OFF
```

Before implementation, confirm the link/configure graph still requests PCL
visualization or builds viewer/player.

- [x] **Step 2: Replace monolithic `kpt`**

Move sources into target-scoped lists. `kpt_core` may link PCL common/I/O,
Eigen, and spdlog; `kpt_render` links actual OpenCV components. Only
`kpt_pcl_viewer` may request or directly link PCL visualization. Installed PCL
1.14 `pcl_io` itself requires VTK for PCD/PLY and its package metadata may
conservatively propagate additional VTK modules.

Never pass an unfiltered `${PCL_LIBRARIES}` aggregate to `kpt_core`. Link the
`pcl_common` and `pcl_io` imported targets so static PCL/vcpkg builds preserve
all necessary transitive dependencies. Do not reconstruct those dependencies
from raw `PCL_*_LIBRARY` paths merely to trim a provider's metadata.

Keep:

```cmake
add_library(kpt ALIAS kpt_core)
```

while migrating every root-CMake consumer (`pc_convert`, `pc_batch_convert`,
`pc_render`, tests, and optional viewer/player) to its precise target.
`pc_render` links `kpt_render`; viewer/player link `kpt_pcl_viewer`.

- [x] **Step 3: Gate viewer executables**

Build `pc_viewer` and `pc_player` only when
`KPT_BUILD_PCL_VIEWERS=ON`. Keep other CLI tools available.

- [x] **Step 4: Verify both feature states**

```bash
cmake --build build/task-02-core -j
ctest --test-dir build/task-02-core --output-on-failure

cmake -S . -B build/task-02-full \
  -DCMAKE_BUILD_TYPE=Debug \
  -DKPT_BUILD_PCL_VIEWERS=ON
cmake --build build/task-02-full -j
ctest --test-dir build/task-02-full --output-on-failure
```

Inspect the core-only graph. Expected: no `pcl_visualization` target and no
`libpcl_visualization` link entry. Do not reject VTK entries inherited through
the provider's `pcl_io` imported target.

Make the check executable rather than visual:

```bash
set -o pipefail
cmake --build build/task-02-core --verbose \
  | tee build/task-02-core/build-commands.log
if rg -i 'pcl_visualization' \
  build/task-02-core/build-commands.log; then
  echo "core-only graph unexpectedly links pcl_visualization" >&2
  exit 1
fi
```

Also generate/inspect CMake's Graphviz output when imported-target linkage is
not visible in the selected generator's command log.

- [x] **Step 5: Commit**

```bash
git add CMakeLists.txt src/cli/pc_viewer.cc src/cli/pc_player.cc \
  src/kpt/viewer src/kpt/player
git commit -m "refactor(build): isolate pcl visualizer targets"
```

---

## Task 3: Modernize CMake Hygiene

**Files:**

- Modify: `CMakeLists.txt`
- Modify: `cmake/CompilerWarnings.cmake`
- Modify: `tests/CMakeLists.txt` if tests are extracted from the root file

**Produces:**

- CMake 3.21 minimum.
- Target-scoped includes, links, options, and definitions.
- Compiler-family warning policy.
- No inactive global OpenMP mutation.

- [x] **Step 1: Add configure assertions**

Wire `KPT_GUI_BACKEND` to Task 1's resolver. Task 3 validates the selected
backend value and rejects unavailable implementations. The exactly-one-backend
**target** assertion cannot exist honestly until backend targets are introduced:
add it in Task 10 and wire it to the final executable in Task 12.
Then verify:

- `CMAKE_POLICY_VERSION_MINIMUM` is absent;
- Windows does not receive `CATCH_CONFIG_NO_POSIX_SIGNALS`;
- GUI-disabled KPT targets do not directly find/link OpenGL or GLFW. Provider
  metadata for the required `pcl_io` imported target may transitively include
  VTK modules whose own interfaces mention OpenGL; that is not a KPT GUI
  dependency and is not grounds to discard PCL's required transitive linkage.

For global `include_directories`/`link_directories`, delete their invocations
and retain a code-review/search checklist; do not build a brittle CMake
introspection framework.

- [x] **Step 2: Raise the minimum**

Use:

```cmake
cmake_minimum_required(VERSION 3.21)
```

Delete `set(CMAKE_POLICY_VERSION_MINIMUM 3.5)`.

- [x] **Step 3: Make dependencies target-scoped**

Replace global directories/flags with:

```cmake
target_include_directories(...)
target_link_libraries(...)
target_compile_options(...)
target_compile_definitions(...)
```

Remove `find_package(OpenMP)` and global OpenMP flags because no current source
uses OpenMP.

- [x] **Step 4: Complete warning policy**

Cover GCC, Clang, AppleClang, MSVC, and clang-cl without treating vendored
headers as project warnings. Condition `CATCH_CONFIG_NO_POSIX_SIGNALS` on
POSIX.

- [x] **Step 5: Verify**

```bash
cmake -S . -B build/task-03 -DCMAKE_BUILD_TYPE=Debug
cmake --build build/task-03 -j
ctest --test-dir build/task-03 --output-on-failure
```

- [x] **Step 6: Commit**

```bash
git add CMakeLists.txt cmake/CompilerWarnings.cmake tests
git commit -m "refactor(build): make dependencies target scoped"
```

---

## Task 4: Add Presets, Toolchains, Triplets, and vcpkg Manifest

**Files:**

- Create: `CMakePresets.json`
- Create: `vcpkg.json`
- Create: `cmake/toolchains/linux-system.cmake`
- Create: `cmake/toolchains/linux-vcpkg.cmake`
- Create: `cmake/toolchains/windows-x64-vcpkg.cmake`
- Create: `cmake/toolchains/macos-arm64-vcpkg.cmake`
- Create: `cmake/toolchains/macos-x64-vcpkg.cmake`
- Create: `cmake/triplets/kpt-x64-windows.cmake`
- Create: `cmake/triplets/kpt-arm64-osx.cmake`
- Create: `cmake/triplets/kpt-x64-osx.cmake`
- Modify: `.gitignore`

**Produces:**

- Preset schema 3 with symmetric debug/release presets.
- Pinned vcpkg baseline.
- Explicit compiler/dependency entry point per platform.

- [x] **Step 1: Add preset-schema validation**

Create schema-3 configure/build/test presets:

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

Do not add workflow presets.

- [x] **Step 2: Implement toolchain wrappers**

Every vcpkg wrapper selects a target triplet. Linux uses vcpkg's built-in
`x64-linux`; Windows and macOS select one project overlay triplet. Each wrapper:

- validates `VCPKG_ROOT`;
- sets the project overlay directory when it selects a project triplet;
- optionally sets `VCPKG_CHAINLOAD_TOOLCHAIN_FILE`;
- includes vcpkg before `project()`;
- is idempotent;
- contains no developer-home path.

- [x] **Step 3: Pin triplet ABI**

Triplets explicitly set architecture, `VCPKG_CRT_LINKAGE`,
`VCPKG_LIBRARY_LINKAGE`, and `VCPKG_CMAKE_SYSTEM_NAME` where applicable.

- [ ] **Step 4: Add and resolve manifest**

Pin `builtin-baseline`. Keep PCL visualization in a `pcl-viewers` feature.
Validate actual PCL/OpenCV feature names against that exact baseline instead of
copying the conceptual design blindly.

This is a hard evidence gate: if the pinned baseline cannot resolve the
required non-visualization PCL/OpenCV features, stop Task 4, record the attempted
baseline, command, elapsed time, and resolver error in
`docs/build-baseline.md`, and choose another documented baseline. Never guess a
feature name merely to make the manifest parse. Record core-only and
viewer-enabled cold configure/build times separately; slow viewer/VTK builds do
not justify contaminating the core graph.

The manifest and pinned feature names are present, but this checkbox remains
open until a real vcpkg checkout resolves the pin and both feature states.

Each preset must explicitly keep these aligned:

```text
KPT_BUILD_PCL_VIEWERS
VCPKG_MANIFEST_FEATURES
```

At this task, Windows/macOS presets keep `KPT_BUILD_GUI=OFF` so the not-yet
implemented backends do not break incremental builds. Tasks 12–14 enable GUI
in the relevant final presets when their backend exists.
Windows and macOS presets also default `KPT_BUILD_PCL_VIEWERS=OFF` and omit the
`pcl-viewers` manifest feature. Enabling those executables is an explicit
product choice after the core/workbench path passes; it is never a first-build
default.

- [x] **Step 5: Ignore user presets**

Add only `CMakeUserPresets.json` to `.gitignore`; keep
`CMakePresets.json` tracked.

- [x] **Step 6: Verify Linux system preset**

```bash
cmake --list-presets
cmake --preset linux-system-debug
cmake --build --preset linux-system-debug
ctest --preset linux-system-debug
```

The Linux system preset is verified. `linux-vcpkg-debug` remains an explicit
unverified gate when `VCPKG_ROOT` is unavailable; it is not implied by this
checkbox.

- [x] **Step 7: Commit**

```bash
git add .gitignore CMakePresets.json vcpkg.json cmake/toolchains \
  cmake/triplets
git commit -m "feat(build): add cross-platform presets and toolchains"
```

---

## Task 5: Fix UTF-8 Path Boundaries and Ancestor Selection

**Files:**

- Create: `src/common/result.hpp`
- Create: `src/platform/utf8_path.hpp`
- Create: `src/platform/utf8_path_posix.cpp`
- Modify: `src/gui/dialog_paths.hpp`
- Modify: `src/gui/dialog_paths.cpp`
- Modify: file-dialog call sites in `src/gui/app.cpp`
- Modify: point-cloud/workflow/render path call sites under `src/kpt/`
- Create: `tests/utf8_path_test.cpp`
- Modify: `tests/gui_test.cpp`
- Modify: `CMakeLists.txt`
- Create: `cmake/windows-utf8.manifest`

**Produces:**

- Dependency-free `kpt_common` target with C++20 `Result<T, E>` based on
  `std::variant`.
- Explicit UTF-8/native-path conversion.
- File-dialog paths independent of process working directory.
- Chinese filename/path round-trip coverage.

- [x] **Step 1: Write failing UTF-8 tests**

Cover:

- `路径/点云/帧 0001.pcd`;
- UTF-8 → native path → UTF-8 identity;
- invalid UTF-8 returns `InvalidUtf8`;
- absolute paths remain absolute;
- no conversion uses locale-dependent `path.string()` semantics.

- [x] **Step 2: Write failing dialog-path tests**

With a synthetic tree `/A/B/C` and process/UI start in `C`, cover selecting:

- a file in `C`;
- a file in `B`;
- a file in `A`;
- a sibling of `C`;
- a relative dialog result resolved against the dialog's displayed directory,
  not the process working directory;
- Chinese directory and filename components.

Tests must use temporary directories, not assume literal `/A`.

- [x] **Step 3: Implement `Result` and conversions**

Required signatures:

```cpp
PlatformResult<std::filesystem::path>
pathFromUtf8(std::string_view value);

PlatformResult<std::string>
pathToUtf8(const std::filesystem::path &value);
```

Windows implementation uses UTF-8↔UTF-16 conversion and native wide paths.
POSIX validates/preserves UTF-8 bytes.

Task 5 implements the interface and POSIX source only. It declares, but does
not pretend to verify, the Windows contract. Task 7 adds
`src/platform/utf8_path_windows.cpp` and runs the same contract on Windows.
Native POSIX filesystem paths may contain arbitrary bytes, but conversion into
the UI UTF-8 contract rejects invalid byte sequences with `InvalidUtf8`.

- [x] **Step 4: Repair dialog normalization**

Define:

```text
dialog current directory + dialog result → normalized native absolute path
```

Never prepend the executable/current-working directory to an already absolute
result. Never discard `..` before anchoring it to the dialog directory.

- [x] **Step 5: Route every call site**

All ImGuiFileDialog strings enter/leave through the UTF-8 helpers. Replace
direct `fs::path(string)`, `.string()`, and ad-hoc concatenation at UI
boundaries.

Before a job/workflow request is queued, every editable UI path is decoded
with `pathFromUtf8`; decode failure is logged and no job is submitted. Native
paths return to UI/job names/logs only through `pathToUtf8`.

PCL 1.15 PCD/PLY readers expose only a narrow filename ABI. Windows
executables embed `activeCodePage=UTF-8` and PCL call sites use
`pathToUtf8`; the platform minimum is Windows 10 version 1903. OpenCV output
uses `imencode` plus native `ofstream`, and tests exercise real Chinese PCD,
PLY, and PNG I/O paths.

Keep ImGuiFileDialog 0.6.8 compiled with `USE_STD_FILESYSTEM`. Its vendored
Windows `stringToPath`/`pathToString` already use `UTF8Decode`/`UTF8Encode`;
the `dirent` fallback must not be compiled. Add an integration harness—not only
a normalization unit test—that creates a temporary Chinese directory, lets
ImGuiFileDialog enumerate it, and verifies `GetCurrentPath`,
`GetFilePathName`, and selection strings round-trip as UTF-8. Run the same
harness on Windows in Task 7.

- [x] **Step 6: Verify**

```bash
cmake -S . -B build/task-05 \
  -DCMAKE_BUILD_TYPE=Debug \
  -DKPT_BUILD_GUI=ON
cmake --build build/task-05 -j
build/task-05/kpt_tests "[utf8]"
build/task-05/kpt_gui_tests "[dialog]"
```

- [x] **Step 7: Commit**

```bash
git add src/common/result.hpp src/platform/utf8_path.hpp \
  src/platform/utf8_path_posix.cpp src/gui/dialog_paths.hpp \
  src/gui/dialog_paths.cpp src/gui/app.cpp tests/utf8_path_test.cpp \
  tests/gui_test.cpp CMakeLists.txt
git commit -m "fix(gui): preserve unicode and ancestor dialog paths"
```

---

## Task 6: Define Platform Services and Implement Linux Backend

**Files:**

- Create: `src/platform/services.hpp`
- Create: `src/platform/settings_store.hpp`
- Create: `src/platform/linux/services.cpp`
- Create: `src/platform/linux/fonts.cpp`
- Create: `src/platform/common/settings_store.cpp`
- Modify: `src/gui/pc_gui.cc`
- Create: `tests/platform_services_test.cpp`
- Modify: `CMakeLists.txt`

**Produces:**

- `Paths`, `Fonts`, `SettingsStore`, and `Services`.
- XDG config path, Fontconfig CJK lookup, native-path ini persistence.

`kpt_platform` and its contract tests build when `KPT_BUILD_GUI=OFF`; platform
services must not be hidden behind the GUI executable option.

- [x] **Step 1: Write service contract tests**

Cover:

- config directory is absolute or returns a structured error;
- an isolated unique subdirectory can be created and removed;
- absent ini is distinct from read failure;
- ini save uses sibling-temp + atomic replacement;
- sibling temps use PID + random nonce and exclusive creation; competing
  writers leave no residue;
- `KPT_CJK_FONT` wins;
- returned font path is readable and face index loads required glyphs;
- no-match font resolution is non-fatal.

The glyph-loading assertion runs only when a deterministic
`KPT_CJK_FONT` fixture is supplied or Fontconfig first reports a usable system
match. A host with no CJK font must pass the explicit no-match/non-fatal branch,
not fail because of its environment.

- [x] **Step 2: Define interfaces**

Use the signatures in design §6.3. `Fonts::matchUiFont` returns
`PlatformResult<std::optional<FontFace>>`, not a bare optional.

- [x] **Step 3: Implement Linux paths/fonts**

- config: `$XDG_CONFIG_HOME/kpt`, otherwise `$HOME/.config/kpt`;
- font: Fontconfig match for required CJK characters;
- override: UTF-8 `KPT_CJK_FONT`;
- no hard-coded distro font path.

Discover and link Fontconfig through `PkgConfig::Fontconfig` (or an equivalent
imported target supplied by the host package). Do not copy raw global
`pkg-config` flags into every target.
The vcpkg manifest declares `fontconfig` directly on Linux rather than relying
on a transitive package edge. FreeType is direct where override/TTC
verification uses it.

- [x] **Step 4: Implement manual ImGui ini persistence**

Set `io.IniFilename = nullptr`. Use:

```cpp
ImGui::LoadIniSettingsFromMemory(...)
ImGui::SaveIniSettingsToMemory(...)
io.WantSaveIniSettings
```

The store owns native paths. Save failures disable persistence but do not abort
startup.

- [x] **Step 5: Inject services**

`main` owns services; they outlive `App`. Tests can inject fakes. Remove XDG,
HOME, and font literals from `pc_gui.cc`.

No `createServicesForTests()` global is needed: `Services` is a composition
bundle and the App/composition-root constructor accepts it or references to its
narrow interfaces. Tests construct the bundle directly from fake
`Paths`/`Fonts`/`SettingsStore`.

- [x] **Step 6: Verify**

```bash
cmake -S . -B build/task-06 \
  -DCMAKE_BUILD_TYPE=Debug \
  -DKPT_BUILD_GUI=ON
cmake --build build/task-06 -j
ctest --test-dir build/task-06 --output-on-failure \
  -R "platform|utf8|dialog|gui"
```

- [x] **Step 7: Commit**

```bash
git add src/platform src/gui/pc_gui.cc \
  tests/platform_services_test.cpp CMakeLists.txt
git commit -m "feat(platform): add linux paths fonts and settings"
```

---

## Task 7: Implement Windows Platform Services

**Platform gate:** Windows 10 version 1903 or later/Windows 11 x64, activated
MSVC developer environment.

**Files:**

- Create: `src/platform/windows/services.cpp`
- Create: `src/platform/windows/fonts.cpp`
- Create: `src/platform/windows/atomic_replace.cpp`
- Create: `src/platform/utf8_path_windows.cpp`
- Create: `tests/dialog_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `src/platform/services.hpp`
- Modify: `src/gui/pc_gui.cc`
- Modify: `tests/gui_test.cpp`
- Modify: `tests/utf8_path_test.cpp`
- Modify: `tests/platform_services_test.cpp`
- Create: `cmake/windows-utf8.manifest`

**Produces:**

- Known Folder config path.
- Unicode environment override.
- DirectWrite font resolution with TTC face index.

- [x] **Step 1: Add Windows contract cases**

Use temporary Chinese paths and filenames. Set a Chinese `KPT_CJK_FONT`
override through the wide environment API.
Run the Task 5 UTF-8 and real ImGuiFileDialog integration contracts; this is
the first task allowed to claim the Windows half passes.

The common font contract constructs a deterministic two-face TTC at runtime
from repository text fixtures: face 0 lacks the requested glyph, face 1 owns
it, and the resolver must return `face_index == 1`. It does not depend on a
host font collection.

The contract cases are present in commit `4212c9a`; their native Windows
execution remains unverified and is deliberately tracked by Step 5.

- [x] **Step 2: Own the Windows COM apartment**

Add a small UI-thread RAII apartment owned by the Windows platform bootstrap;
it is created before `SHGetKnownFolderPath`/services and outlives services,
runtime, and App. Call:

```cpp
CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
```

Balance `S_OK`/`S_FALSE` with `CoUninitialize` on the same thread. Handle
`RPC_E_CHANGED_MODE` as “already initialized with another model” without
calling `CoUninitialize`; propagate other failures. Platform-service tests use
the same RAII fixture. The ownership is required by the Known Folders API,
independent of whether a particular DirectWrite call happens to work without
it.

- [x] **Step 3: Implement config path**

Use `SHGetKnownFolderPath(FOLDERID_RoamingAppData)` and append the KPT
directory as a native path. Wrap the output allocation immediately so it is
freed even if the API returns failure after assigning storage.

- [x] **Step 4: Implement override and font lookup**

- read override through `_wgetenv`;
- match required glyphs through DirectWrite;
- obtain `IDWriteFontFace` files and a local file loader;
- return native file path plus collection face index;
- return no match for non-local/custom-loader fonts;
- do not fabricate a path.

`Services` owns a first-member `PlatformLifetime`, so native bootstrap state
is destroyed last. `createServices()` returns `PlatformResult<Services>`;
initialization failures cannot be hidden in partially populated pointers.

Dear ImGui core, ImGuiFileDialog, and `kpt_dialog_tests` are renderer/window
independent targets. They build with `KPT_BUILD_GUI=OFF`, allowing this task's
real Chinese-directory enumeration contract to run in the Windows headless
preset. Platform-specific source selection keeps Linux builds free of Windows
headers and libraries.

- [ ] **Step 5: Verify preset**

```powershell
cmake --preset windows-x64-vcpkg-debug -DKPT_BUILD_GUI=OFF
cmake --build --preset windows-x64-vcpkg-debug
ctest --preset windows-x64-vcpkg-debug
```

Expected: core/headless and platform tests pass with viewers disabled according
to the preset. No mojibake in assertion diagnostics.

- [x] **Step 6: Commit**

Commit all Task 7 files listed above as:

```powershell
git commit -m "feat(platform): add windows paths fonts and settings"
```

Recorded implementation commit: `4212c9a`.

---

## Task 8: Implement macOS Platform Services

**Platform gate:** macOS 13+, first arm64, then x86-64.

**Files:**

- Create: `src/platform/macos/services.mm`
- Create: `src/platform/macos/fonts.mm`
- Move: `src/platform/linux/atomic_replace.cpp` to
  `src/platform/posix/atomic_replace.cpp`
- Modify: `CMakeLists.txt`
- Modify: `CMakePresets.json`
- Modify: `cmake/triplets/kpt-arm64-osx.cmake`
- Modify: `cmake/triplets/kpt-x64-osx.cmake`
- Modify: `vcpkg.json`
- Modify: `tests/platform_services_test.cpp`

**Produces:**

- Application Support settings path.
- Core Text CJK lookup with file URL and face index.

- [x] **Step 1: Add macOS service cases**

Reuse the common contract with temporary Chinese paths. Add a TTC-face test
when the host font collection supplies one; otherwise use an explicit fixture.

The contract cases are present in commit `5704e94`; their native arm64 and
x86-64 execution remains unverified and is deliberately tracked by Step 4.

- [x] **Step 2: Implement native services**

- Foundation Application Support directory;
- Core Text descriptor matching;
- resolvable font URL → native path;
- collection face index;
- clean no-match result for non-file-backed faces;
- `KPT_CJK_FONT` remains highest priority.
- initialize FreeType once per `matchUiFont` call and propagate initialization
  failure before enumerating Core Text candidates;

- [x] **Step 3: Keep Objective-C++ private**

No Foundation/Core Text type may appear in public C++ headers. Use `.mm`
implementation files and standard C++ return types.

FreeType is a direct dependency because Core Text does not expose a public
collection face-index attribute. The implementation verifies each local face
and aligns it with the Core Text PostScript name; an ambiguous or
non-file-backed match returns no match rather than guessing. Linux and macOS
share the POSIX atomic-replace implementation. Both macOS presets and vcpkg
triplets set a 13.0 deployment target.

The deterministic two-face TTC override contract is shared by all three
platform test builds. Linux executes it in the current review; Windows and
macOS execution remains gated by their native-host verification steps.

- [ ] **Step 4: Verify both architectures**

```bash
cmake --preset macos-arm64-vcpkg-debug -DKPT_BUILD_GUI=OFF
cmake --build --preset macos-arm64-vcpkg-debug
ctest --preset macos-arm64-vcpkg-debug

cmake --preset macos-x64-vcpkg-debug -DKPT_BUILD_GUI=OFF
cmake --build --preset macos-x64-vcpkg-debug
ctest --preset macos-x64-vcpkg-debug
```

Run x64 commands on x64 hardware or an explicitly supported cross-build host;
do not report them passed merely because arm64 passed. Cross-compilation or a
Rosetta smoke run is supplemental; final x86-64 service acceptance requires an
Intel Mac.

- [x] **Step 5: Commit**

```bash
git add src/platform/macos CMakeLists.txt tests/platform_services_test.cpp
git commit -m "feat(platform): add macos paths fonts and settings"
```

Recorded implementation commit: `5704e94`.

---

## Task 9: Extract Viewport Contracts, Model, and PCL Adapter

**Files:**

- Create: `src/kpt/core_types.hpp`
- Modify: `src/kpt/types.hpp`
- Create: `src/gui/viewport/render_types.hpp`
- Create: `src/gui/viewport/renderer.hpp`
- Create: `src/gui/viewport/model.hpp`
- Create: `src/gui/viewport/model.cpp`
- Create: `src/gui/viewport/pcl_adapter.hpp`
- Create: `src/gui/viewport/pcl_adapter.cpp`
- Create: `tests/viewport_model_test.cpp`
- Modify: `CMakeLists.txt`

**Produces:**

- `kpt_gui_contracts`.
- PCL-free `kpt_gui_model`.
- `kpt_viewport_pcl_adapter`.
- Revision and UI-thread-neutral snapshot semantics.
- Dear ImGui core target separated from GLFW/renderer backend targets.

- [x] **Step 1: Write failing model tests**

Cover:

- bounds and scalar range;
- fit/orbit/pan/zoom/view presets;
- style mutation;
- empty and non-finite input;
- camera/style do not bump cloud revision;
- a newer snapshot supersedes an older generation;
- revision 0 represents no cloud.

- [x] **Step 2: Define shared render types**

Add `ViewportVertex`, `CloudBounds`, `ViewportStyle`, `ViewportFrame`,
`ViewportCloudSnapshot`, `PixelExtent`, `ViewportTexture`, `BackendKind`,
`RendererError`, and opaque `FrameContext`.

Use `ImTextureRef` directly.

Move dependency-free `Format`, `ColorBy`, and `View` from PCL-bearing
`src/kpt/types.hpp` into `src/kpt/core_types.hpp`; `types.hpp` includes that
header and then adds only PCL point declarations/aliases. GUI model/contracts
include `core_types.hpp`, never `types.hpp`.

Fix the renderer contract now, before either backend:

```cpp
class ViewportRenderer {
public:
  virtual Result<void, RendererError>
  upload(std::span<const ViewportVertex>, std::uint64_t revision) = 0;
  virtual Result<void, RendererError> resize(PixelExtent) = 0;
  virtual Result<void, RendererError>
  render(const ViewportFrame &, FrameContext &) = 0;
  virtual ViewportTexture texture() const = 0;
};
```

Fake renderers implement the identical context-taking signature.

Because `ViewportTexture` contains `ImTextureRef`, split the current monolithic
`imgui` CMake target into `imgui_core` (Dear ImGui core sources/headers only)
and backend targets. `kpt_gui_contracts` has a public/header dependency on
`imgui_core`; it must not link GLFW, OpenGL, or Metal. A temporary compatibility
target may preserve the old Linux GUI until Task 12.

- [x] **Step 3: Implement PCL-free model**

The model accepts `shared_ptr<const ViewportCloudSnapshot>`. It has no PCL,
GLFW, OpenGL, Metal, Win32, or Cocoa include.

- [x] **Step 4: Implement PCL adapter**

Convert `PointCloudIRGBConstPtr` into an immutable snapshot. Ignore non-finite
vertices. The UI allocates request generation before dispatch.

- [x] **Step 5: Prove target boundaries**

Inspect compile/link commands:

- `kpt_gui_model`: no PCL;
- `kpt_viewport_pcl_adapter`: PCL allowed;
- `kpt_gui_contracts`/model may see ImGui core headers but no GLFW or renderer
  backend;
- neither target links a GPU API.

- [x] **Step 6: Verify**

```bash
cmake -S . -B build/task-09 \
  -DCMAKE_BUILD_TYPE=Debug \
  -DKPT_BUILD_GUI=ON
cmake --build build/task-09 -j
ctest --test-dir build/task-09 --output-on-failure \
  -R "viewport_model"
```

- [x] **Step 7: Commit**

```bash
git add src/kpt/core_types.hpp src/kpt/types.hpp src/gui/viewport \
  tests/viewport_model_test.cpp CMakeLists.txt
git commit -m "refactor(gui): extract platform-neutral viewport model"
```

---

## Task 10: Refactor OpenGL Renderer and Add Test-Only Readback

**Files:**

- Create: `src/gui/backend/opengl/point_renderer.hpp`
- Create: `src/gui/backend/opengl/point_renderer.cpp`
- Create: `src/gui/viewport/test_access.hpp`
- Create: `src/gui/backend/opengl/test_access.cpp`
- Create: `third_party/glad/include/glad/gl.h`
- Create: `third_party/glad/include/KHR/khrplatform.h`
- Create: `third_party/glad/src/gl.c`
- Create: `third_party/glad/README.kpt.md`
- Create: `tests/opengl_renderer_test.cpp`
- Modify temporarily: `src/gui/point_renderer.hpp`
- Modify temporarily: `src/gui/point_renderer.cpp`
- Modify: `CMakeLists.txt`

**Produces:**

- `OpenGLPointRenderer : ViewportRenderer`.
- `RendererTestAccess` isolated from production.
- Behavior-based OpenGL renderer contract.

- [x] **Step 1: Write failing renderer contract**

Synthetic fixture verifies:

- exact requested positive extent;
- zero extent suspends rendering;
- a 5×5 center neighborhood contains a non-background pixel after fit;
- color modes differ by region statistics;
- resize recreates resources;
- empty cloud is background-only;
- non-finite vertices do not poison later draws;
- repeated create/destroy succeeds.
- hostile ambient OpenGL scissor, blend, depth-write/depth-function,
  color-mask, and rasterizer-discard state cannot suppress the viewport and is
  restored exactly after render.

Use channel-distance tolerance 3. Do not compare complete golden images.

- [x] **Step 2: Implement the renderer interface**

Implement the exact Task 9 signature, especially:

```cpp
Result<void, RendererError>
render(const ViewportFrame &frame, FrameContext &context) override;
```

`upload` and `resize` also return `Result`. `upload` copies before its span
expires. Renderer owns VAO, VBO, shader program, FBO, color/depth attachments.
`OpenGLPointRenderer` validates `BackendKind::OpenGL` and that the expected GL
context is current; it does not obtain context through a global.
`OpenGLFrameContext` construction, activation, invalidation, and native-window
inspection are private to the runtime and renderer. Tests obtain contexts only
through `RendererTestAccess`, so production callers cannot revive a context
after `renderAndPresent`.

Now that the first independent backend target exists, add
`kpt_assert_exactly_one_target(...)` to `KptTargetAssertions.cmake` with a pure
CMake self-test. Apply it to the selected backend target set; this is the
target-level acceptance deliberately deferred from Task 3.

Keep the old `PointRenderer` as a temporary compatibility facade combining
`ViewportModel` and `OpenGLPointRenderer`, so current `App` remains buildable
between Tasks 10 and 11. Mark it migration-only; do not add new behavior to the
facade.

- [x] **Step 3: Add GL loader**

Vendor checked-in GLAD 2 generated files under `third_party/glad`; record exact
GLAD generator version, API/profile (`gl:core=3.3`), extensions, generation
command, and upstream hash in `README.kpt.md`. Normal builds do not download or
regenerate code.

KPT renderer translation units include `<glad/gl.h>` and initialize GLAD from
`glfwGetProcAddress`. Dear ImGui's OpenGL3 backend retains its vendored,
symbol-prefixed internal loader; do not define
`IMGUI_IMPL_OPENGL_LOADER_CUSTOM`. This avoids hidden include-order coupling
while making clear that KPT never calls the ImGui-private loader.

Commit this dependency boundary separately after a loader-only compile test:

```bash
git add third_party/glad third_party/glad/README.kpt.md CMakeLists.txt
git commit -m "build(gui): vendor glad opengl loader"
```

- [x] **Step 4: Normalize texture output**

Return:

```cpp
ViewportTexture{ImTextureRef{...}, uv0, uv1}
```

Encode OpenGL's vertical orientation in `uv0/uv1`; remove backend flip logic
from `App`.

- [x] **Step 5: Isolate readback**

`RendererTestAccess` is a friend of the concrete renderer in a backend-private
header. Only `kpt_gui_test_support` calls `glReadPixels`.

The new production renderer has no `centerPixelVisible`. The temporary
compatibility facade may forward the legacy smoke call only until Task 11;
do not expose it through `ViewportRenderer`.

- [x] **Step 6: Verify**

```bash
cmake -S . -B build/task-10 \
  -DCMAKE_BUILD_TYPE=Debug \
  -DKPT_BUILD_GUI=ON \
  -DKPT_GUI_BACKEND=opengl
cmake --build build/task-10 -j
xvfb-run -a ctest --test-dir build/task-10 --output-on-failure \
  -R "viewport_model|opengl_renderer|gui"
```

- [x] **Step 7: Commit**

```bash
git add src/gui/backend/opengl src/gui/viewport/test_access.hpp \
  src/gui/point_renderer.hpp src/gui/point_renderer.cpp \
  tests/opengl_renderer_test.cpp CMakeLists.txt
git commit -m "refactor(gui): isolate opengl viewport renderer"
```

---

## Task 11: Recompose App Around Two Viewport Sessions

**Files:**

- Modify: `src/gui/app.hpp`
- Modify: `src/gui/app.cpp`
- Delete: `src/gui/point_renderer.hpp`
- Delete: `src/gui/point_renderer.cpp`
- Move: `src/gui/job_system.*` → `src/gui/jobs/job_system.*`
- Create or move: `src/gui/jobs/ui_events.*`
- Modify: `tests/gui_test.cpp`
- Modify: `CMakeLists.txt`

**Produces:**

- `kpt_jobs`.
- Two `ViewportSession` instances in `App`.
- Deterministic frame order and out-of-order job rejection.

- [x] **Step 1: Extend fake-renderer tests**

Assert frame order:

```text
apply UI events
App::draw(shared_frame_context)
  → main upload? → main resize
      → main render(shared_frame_context) → main Image
  → trajectory upload? → trajectory resize
      → trajectory render(shared_frame_context) → trajectory Image
```

Also test:

- camera/style change does not upload;
- multiple completed jobs upload only newest generation;
- old completion cannot replace new cloud;
- zero extent skips render/Image;
- renderer errors reach the caller.
- App smoke behavior no longer depends on pixel readback.
- a real `App`/ImGui harness drains controlled `UiEvents` completions before
  draw, rejects stale sequence and request generations, and passes the same
  context to main then trajectory;
- a non-empty trajectory becoming an empty positive-revision snapshot uploads
  the empty snapshot, resizes its renderer to zero, and emits no Image.

- [x] **Step 2: Introduce `ViewportSession`**

```cpp
struct ViewportSession {
  ViewportModel model;
  std::unique_ptr<ViewportRenderer> renderer;
  std::uint64_t uploaded_revision = 0;
};
```

`App` owns main and trajectory sessions. It does not construct a backend.
Remove the temporary Task 10 compatibility facade after all call sites use the
sessions.
Delete the legacy `centerPixelVisible` App smoke path; renderer pixels are
observed only through test support.

Fix the App entry point:

```cpp
Result<void, AppError> App::draw(FrameContext &frame_context);
```

The same context instance is passed to both renderers. Fake-renderer tests
receive a fake `FrameContext`; no overload without context exists.

Task 12 does not own the context yet. For this independently green transition,
the existing `pc_gui.cc` GL loop creates one stack-scoped
`OpenGLFrameContext` after making the GLFW context current, passes it to
`App::draw`, and invalidates it before buffer swap. Task 12 moves that exact
ownership into `GuiRuntime`; Task 11 must not introduce another hidden context
source.

- [x] **Step 3: Enforce thread ownership**

Workers may load/transform PCL and create immutable snapshots. They post
results to `UiEvents`. Models/renderers are created, mutated, used, and
destroyed on the UI thread only.

- [x] **Step 4: Move jobs into their own target**

`kpt_jobs` must not depend on the viewport model or GPU backend. Keep event
payloads immutable.

Add `kpt_gui_app` as an explicit library target containing `app.cpp`; it links
model, contracts, jobs, ImGui core, and ImGuiFileDialog, but no concrete GPU
backend. `pc_gui` links it and supplies renderer instances at composition time.

Transitional `pc_gui --smoke-test` means: create sessions, install a synthetic
snapshot, execute one old-loop GL frame with the explicit temporary context,
and exit without error. It makes no pixel assertion; pixel visibility belongs
only to `opengl_renderer_test`.

- [x] **Step 5: Verify**

```bash
cmake -S . -B build/task-11 \
  -DCMAKE_BUILD_TYPE=Debug \
  -DKPT_BUILD_GUI=ON \
  -DKPT_GUI_BACKEND=opengl
cmake --build build/task-11 -j
ctest --test-dir build/task-11 --output-on-failure \
  -R "gui|viewport_model"
```

- [x] **Step 6: Commit**

```bash
git add src/gui/app.hpp src/gui/app.cpp src/gui/jobs \
  tests/gui_test.cpp CMakeLists.txt
git add -u src/gui/job_system.hpp src/gui/job_system.cpp \
  src/gui/point_renderer.hpp src/gui/point_renderer.cpp
git commit -m "refactor(gui): compose app from viewport sessions"
```

---

## Task 12: Extract OpenGL GUI Runtime

**Files:**

- Create: `src/gui/runtime/runtime.hpp`
- Create: `src/gui/runtime/factory.hpp`
- Create: `src/gui/runtime/glfw_opengl_runtime.cpp`
- Modify: `src/gui/pc_gui.cc`
- Create: `tests/opengl_runtime_smoke_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `CMakePresets.json`

**Produces:**

- `GuiRuntime`.
- Runtime-owned window, ImGui lifecycle, framebuffer metrics, and
  `OpenGLFrameContext`.
- Thin `pc_gui.cc`.

When linking final `pc_gui`, invoke Task 10's exactly-one-backend target
assertion over every available backend target, then link only
`KPT_ACTIVE_GUI_BACKEND`. This is the executable-level half of Task 3's
deferred acceptance.

- [x] **Step 1: Write runtime state-machine tests**

Cover:

- initialize failure has `GuiError`;
- only one active frame;
- context invalid outside an active frame;
- `createViewportRenderer` returns a structured failure;
- partial initialization can shut down;
- shutdown is idempotent;
- one-frame startup/shutdown smoke path.
- resize/event/frame-boundary framebuffer metric refresh;
- settings load exactly once, dirty-frame save, final shutdown flush, and
  permanent persistence disablement after a save failure.

- [x] **Step 2: Define runtime API**

Implement design §6.7:

```cpp
Result<void, GuiError> initialize(...);
Result<std::reference_wrapper<FrameContext>, GuiError> beginFrame();
Result<void, GuiError> renderAndPresent();
Result<std::unique_ptr<ViewportRenderer>, GuiError>
createViewportRenderer();
FramebufferMetrics framebufferMetrics() const;
```

- [x] **Step 3: Move GLFW/OpenGL/ImGui lifecycle**

Move window/context creation, Dear ImGui init/new-frame/render/shutdown, event
polling, and presentation out of `pc_gui.cc`.

- [x] **Step 4: Implement DPI metrics**

Refresh logical size, framebuffer size, and scale every frame and on GLFW
callbacks. First milestone keeps Dear ImGui platform multi-viewports disabled.
Projection and render targets use physical pixels; layout uses logical units.
Task 12 changes the transitional Task 11 entry point to
`App::draw(FrameContext &, FramebufferMetrics)`; `App` no longer reads
`ImGuiIO::DisplayFramebufferScale` as a second DPI authority.

- [x] **Step 5: Thin the entry point**

`pc_gui.cc` retains option parsing, service/runtime factories, composition, main
loop, diagnostic reporting, and exit status. It contains no raw GLFW/OpenGL
symbol.

- [x] **Step 6: Verify Linux**

Enable `KPT_BUILD_GUI=ON` and `KPT_GUI_BACKEND=opengl` in the final Linux GUI
presets before verification.

```bash
cmake -S . -B build/task-12 \
  -DCMAKE_BUILD_TYPE=Debug \
  -DKPT_BUILD_GUI=ON \
  -DKPT_GUI_BACKEND=opengl
cmake --build build/task-12 -j
xvfb-run -a ctest --test-dir build/task-12 --output-on-failure
```

- [x] **Step 7: Commit**

```bash
git add src/gui/runtime src/gui/pc_gui.cc \
  tests/opengl_runtime_smoke_test.cpp CMakeLists.txt CMakePresets.json
git commit -m "refactor(gui): extract glfw opengl runtime"
```

---

## Task 13: Enable and Verify Windows OpenGL GUI

**Platform gate:** Windows 10/11 x64 with MSVC and vcpkg preset.

**Files:**

- Modify: `CMakeLists.txt`
- Modify: `CMakePresets.json`
- Modify as failures require:
  `src/gui/runtime/glfw_opengl_runtime.cpp`,
  `src/gui/backend/opengl/*`
- Modify: `tests/opengl_runtime_smoke_test.cpp`

**Produces:**

- GLFW Win32 + OpenGL 3.3 `pc_gui`.
- Hidden-window renderer/runtime smoke tests on Windows.

- [x] **Step 1: Scope GLFW platform options**

Only Linux sets:

```cmake
GLFW_BUILD_X11=ON
GLFW_BUILD_WAYLAND=OFF
```

Windows receives neither variable and uses GLFW Win32. Metal/Cocoa work remains
outside this task.

- [ ] **Step 2: Configure the Windows preset**

Enable `KPT_BUILD_GUI=ON` and `KPT_GUI_BACKEND=opengl` in the Windows presets.

From an activated x64 MSVC developer shell:

```powershell
cmake --preset windows-x64-vcpkg-debug
```

Fix compile errors without adding Win32 conditionals to `App`, workflow, jobs,
or I/O.

> **Status note (2026-07-28):** The preset enablement
> (`KPT_BUILD_GUI=ON`, `KPT_GUI_BACKEND=opengl` for `windows-x64-vcpkg-*`)
> landed in commit `2d0284d`. The box remains unchecked because the
> configure/build/compile-fix commands below require a Windows host that was
> not available; the preset change is a static source edit, not a verified
> configure pass.

- [ ] **Step 3: Run full Windows verification**

```powershell
cmake --build --preset windows-x64-vcpkg-debug
ctest --preset windows-x64-vcpkg-debug
```

Explicit manual fixture:

- open a point cloud in a Chinese path;
- choose a file in the parent and grandparent directory;
- restart and confirm ini persistence;
- load CJK font override from a Chinese path.

Record command outputs, tool/dependency versions, pass/fail results, and manual
fixture evidence in the Windows row of `docs/cross-platform-build.md`; a
checkbox without that evidence is not acceptance.

Platform gate remains closed on the 2026-07-28 Linux implementation host:
Windows SDK, MSVC, and `VCPKG_ROOT` are unavailable. The presets and source
selection were made Windows-ready, but Steps 2-4 remain unchecked until their
commands and manual fixtures run on Windows.

Static evidence on that host is limited to the portable resolver self-test:
Windows `auto`/`opengl` resolve to `opengl`, Windows `metal` is rejected, the
Windows debug/release presets select the OpenGL GUI with PCL viewers disabled,
and Linux's X11/Wayland cache variables are guarded by
`CMAKE_SYSTEM_NAME STREQUAL "Linux"`. A Linux system-preset rebuild plus the
complete seven-test suite under `xvfb-run` passed after these changes. None of
this substitutes for Windows compilation or the manual fixture.

- [ ] **Step 4: Verify invalid backend rejection**

```powershell
cmake -S . -B build/reject-metal -DKPT_BUILD_GUI=ON -DKPT_GUI_BACKEND=metal
```

Expected: explicit configure error.

- [x] **Step 5: Commit**

```powershell
git add CMakeLists.txt src/gui/runtime/glfw_opengl_runtime.cpp \
  src/gui/backend/opengl tests/opengl_runtime_smoke_test.cpp CMakePresets.json
git commit -m "feat(gui): enable windows opengl workbench"
```

---

## Task 14: Implement macOS Metal Renderer and Runtime

**Platform gate:** macOS 13+ with Xcode command-line tools.

**Gate record (2026-07-28):** The available host is Linux x86-64. It has no
`xcrun` or `xcodebuild`, and `VCPKG_ROOT` is unset. A read-only inspection
confirmed that the vendored Dear ImGui GLFW+Metal example uses
`GLFW_NO_API`, `CAMetalLayer`, one command buffer, and encode-before-present
ordering. This is source evidence, not a macOS execution result. Configuring
this host with `KPT_BUILD_GUI=ON` and `KPT_GUI_BACKEND=metal` fails explicitly
with `Backend 'metal' is not supported on Linux`, as required. No Objective-C++
implementation, vcpkg resolution, Xcode/Metal shader compilation, GPU
contract, or lifecycle check can be truthfully completed on this host.
Consequently every implementation and verification checkbox below remains
open.

**Files:**

- Create: `src/gui/backend/metal/point_renderer.hpp`
- Create: `src/gui/backend/metal/point_renderer.mm`
- Create: `src/gui/backend/metal/test_access.mm`
- Create: `src/gui/backend/metal/point_shaders.metal`
- Create: `src/gui/runtime/glfw_metal_runtime.mm`
- Create: `tests/metal_renderer_test.mm`
- Create: `tests/metal_runtime_smoke_test.mm` if spike proves reliable
- Modify: `CMakeLists.txt`
- Modify: `CMakePresets.json`

**Produces:**

- Metal offscreen point renderer.
- Runtime-owned device, queue, command buffer, drawable, and final ImGui pass.
- `.metallib` build/package path.
- Metal behavior contract.

- [ ] **Step 1: Run focused spikes**

Before implementation, record:

- selected vcpkg baseline resolves PCL/OpenCV for arm64;
- vendored GLFW+Metal example survives Retina resize and sleep/wake;
- hidden GLFW lifecycle is reliable enough for an automatic smoke test;
- `[1,64]` point-size range works on the minimum supported Mac.

Basic Cocoa handle and `CAMetalLayer` attachment need no spike; the vendored
example already demonstrates them.

- [ ] **Step 2: Write failing Metal renderer contract**

Reuse the same behavioral fixture and thresholds as OpenGL. Implement
`RendererTestAccess::readColor` by encoding a blit from the renderer's
`MTLStorageModePrivate` color texture into a row-aligned
`MTLStorageModeShared` buffer. Encode render and readback into the test
fixture's single command buffer, commit once, wait for completion, then copy
the shared bytes into `Rgba8Image`. Do not call `getBytes` on the private
production texture and do not weaken production storage mode for tests.

- [ ] **Step 3: Implement MSL point rendering**

- draw `MTLPrimitiveTypePoint`;
- output `float point_size [[point_size]]`;
- render round points by discarding fragments outside the point-coordinate
  radius, matching the OpenGL behavior contract;
- BGRA8 color target with shader-read/render-target usage;
- use `MTLStorageModePrivate` for renderer-owned color, depth, and immutable
  vertex resources;
- depth target;
- ignore non-finite input at snapshot creation;
- pack `ViewportVertex` into a Metal-private, trivially-copyable GPU POD with
  explicit 16-byte fields and compile-time size/offset assertions matching the
  MSL declaration; never upload Eigen object representation directly;
- use one immutable vertex buffer per uploaded revision and retire replaced
  buffers only after every command buffer that references them completes.

- [ ] **Step 4: Build and package `.metallib`**

For Ninja:

```text
xcrun -sdk macosx metal
xcrun -sdk macosx metallib
```

For Xcode, use native Metal source handling. Declare generated outputs and copy
the library beside the executable or into bundle resources. Resolve relative
to executable/bundle, never working directory.

- [ ] **Step 5: Implement `MetalFrameContext` and runtime**

Runtime owns:

- `GLFWwindow` with `GLFW_NO_API`;
- Cocoa window and `CAMetalLayer`;
- device and command queue;
- one command buffer per frame;
- drawable acquisition/presentation;
- Dear ImGui GLFW platform and Metal renderer backends.

Renderer receives command encoding access only through active
`FrameContext`. No global/thread-local command buffer.
Both main and trajectory renderers receive the same `MetalFrameContext` and
therefore encode into the same runtime-owned `MTLCommandBuffer`. A renderer may
create/end only its own offscreen encoder; it must never commit, wait, present,
or allocate a second frame command buffer.
`MetalFrameContext` is defined in a backend-private Objective-C++ header,
constructible only by the runtime and the backend-private test adapter. No
`id<MTL...>` type appears in a portable header.

- [ ] **Step 6: Implement frame order and resource lifetime**

```text
begin command buffer
→ main offscreen pass
→ trajectory offscreen pass
→ ImGui final drawable pass
→ present
→ commit
```

Retain replaced textures, vertex buffers, uniform buffers, and pipeline-owned
objects until every command buffer that recorded them completes. Use command
buffer completion handlers (or an equivalent bounded retirement queue), not a
per-frame `waitUntilCompleted`. Set `contentsScale` and `drawableSize` from
current physical metrics.
Pass that same command buffer to `ImGui_ImplMetal_RenderDrawData`, matching the
vendored official GLFW+Metal example's encode-before-present ordering.

- [ ] **Step 7: Split Metal tests from OpenGL**

Metal preset must not compile/link:

- `OpenGL::GL`;
- OpenGL renderer tests;
- OpenGL runtime smoke target;
- macOS OpenGL framework.

Keep `find_package(OpenGL)`, GLAD, `imgui_impl_opengl3`, and every OpenGL
backend target inside the OpenGL selection branch. Add configure-time target
assertions showing that a Metal selection links exactly
`kpt_gui_backend_metal` and that its transitive link interface contains no
OpenGL target or framework.

Mandatory automation is the lower-level offscreen contract. Add lifecycle
smoke only if Step 1 proved it reliable; otherwise keep the exact lifecycle
check in the manual acceptance list.

- [ ] **Step 8: Verify arm64**

Enable `KPT_BUILD_GUI=ON` and `KPT_GUI_BACKEND=metal` in the macOS presets.

```bash
cmake --preset macos-arm64-vcpkg-debug
cmake --build --preset macos-arm64-vcpkg-debug
ctest --preset macos-arm64-vcpkg-debug
```

Manual acceptance:

- both viewport textures render;
- resize and monitor-scale changes remain sharp;
- sleep/wake recovers;
- repeated startup/shutdown succeeds;
- Chinese parent/ancestor file selection works;
- no OpenGL framework appears in the link result.

- [ ] **Step 9: Verify x86-64**

```bash
cmake --preset macos-x64-vcpkg-debug
cmake --build --preset macos-x64-vcpkg-debug
ctest --preset macos-x64-vcpkg-debug
```

- [ ] **Step 10: Commit**

```bash
git add src/gui/backend/metal src/gui/runtime/glfw_metal_runtime.mm \
  tests/metal_renderer_test.mm tests/metal_runtime_smoke_test.mm \
  CMakeLists.txt CMakePresets.json
git commit -m "feat(gui): add macos metal workbench"
```

Omit `tests/metal_runtime_smoke_test.mm` from staging when the spike selects
manual lifecycle verification.

---

## Task 15: Documentation and Final Three-Platform Acceptance

**Acceptance record (2026-07-28):** A clean Linux system-preset configure and
114-edge build passed; all seven tests and the independent GUI smoke command
passed under Xvfb. Boundary searches and the generated link statement verified
portable-source separation and selection of only the OpenGL backend. The
available host is Linux x86-64 with no `VCPKG_ROOT`, Windows toolchain, or
Xcode tools. Windows, macOS, and every vcpkg row therefore remain explicitly
unverified; Task 14's Metal implementation gate remains open.

**Files:**

- Modify: `README.md`
- Modify: `docs/build-baseline.md`
- Create: `docs/cross-platform-build.md`
- Modify only if reality differs:
  `docs/superpowers/specs/2026-07-28-cross-platform-design.md`

**Produces:**

- Exact local commands and prerequisites for each platform.
- Known limitations and spike results.
- Final Definition-of-Done evidence.

- [x] **Step 1: Write build documentation**

Document:

- CMake 3.21+, Ninja, compiler prerequisites;
- Linux system-package and vcpkg presets;
- activated MSVC developer shell and Windows presets;
- Xcode command-line tools and macOS presets;
- `VCPKG_ROOT` through environment or untracked user preset;
- `KPT_BUILD_PCL_VIEWERS`;
- supported GUI backend per platform;
- X11-only initial Linux GUI;
- CJK override and settings locations;
- known Metal lifecycle limitations, if any.

- [x] **Step 2: Run Linux clean acceptance**

Use a fresh directory:

```bash
cmake --preset linux-system-debug
cmake --build --preset linux-system-debug
ctest --preset linux-system-debug
```

Then run the OpenGL smoke test under `xvfb-run`.

- [ ] **Step 3: Run Windows clean acceptance**

```powershell
cmake --preset windows-x64-vcpkg-debug
cmake --build --preset windows-x64-vcpkg-debug
ctest --preset windows-x64-vcpkg-debug
```

Record exact tool and dependency versions in the acceptance document.

- [ ] **Step 4: Run macOS clean acceptance**

```bash
cmake --preset macos-arm64-vcpkg-debug
cmake --build --preset macos-arm64-vcpkg-debug
ctest --preset macos-arm64-vcpkg-debug
```

Repeat x86-64 on a suitable host.

Release acceptance for x86-64 requires a real Intel Mac. An Apple Silicon
cross-compile or Rosetta run is useful supplemental compile/smoke evidence but
does not replace Intel hardware verification, especially for Metal driver
behavior.

- [x] **Step 5: Audit architectural boundaries**

Search:

```bash
rg -n \
  '_WIN32|WIN32|__APPLE__|APPLE|__linux__|CMAKE_SYSTEM|<GL/|<Metal/|windows\\.h' \
  src/gui/app.cpp src/gui/jobs src/kpt src/gui/viewport
rg -n 'GLuint|MTLTexture|ID3D11|MTLCommandBuffer' \
  src/gui/app.cpp src/gui/viewport
rg -n 'path\\.string\\(\\)|std::getenv' src/gui src/platform
```

Expected:

- no OS/GPU conditional in App, jobs, workflow, or I/O;
- no raw GPU type in portable viewport/App headers;
- no forbidden Windows path boundary;
- exactly one backend linked by each GUI preset.

- [x] **Step 6: Run formatting and repository checks**

```bash
git diff --check
cmake --build --preset linux-system-debug
ctest --preset linux-system-debug
```

Run platform-specific equivalents on their hosts. Do not mark an unavailable
platform verified.

- [x] **Step 7: Reconcile design with implementation**

If a spike forced a different contract, update design and audit with evidence
before calling the work complete. Do not silently diverge.

- [x] **Step 8: Commit**

```bash
git add README.md docs/build-baseline.md docs/cross-platform-build.md \
  docs/superpowers/specs/2026-07-28-cross-platform-design.md
git commit -m "docs: add cross-platform build guide"
```

## Completion Checklist

- [x] Linux system preset configures, builds, and tests.
- [ ] Windows vcpkg preset configures, builds, and tests.
- [ ] macOS arm64 vcpkg preset configures, builds, and tests.
- [ ] macOS x86-64 vcpkg preset configures, builds, and tests on a suitable
  host.
- [x] Core/headless build neither requests nor directly links PCLVisualizer;
  provider-owned PCL I/O transitive dependencies remain intact.
- [ ] Windows and Linux OpenGL satisfy the renderer behavior contract.
- [ ] macOS Metal satisfies the same behavior contract.
- [ ] Metal preset links no OpenGL target/framework.
- [ ] Chinese path and filename round trips pass on all three platforms.
- [x] Parent, grandparent, and sibling dialog selection tests pass.
- [ ] Settings and CJK font discovery use native services.
- [ ] UI-thread, revision, DPI, frame-context, and texture-lifetime contracts
  are enforced by tests.
- [x] Unsupported backend/platform combinations fail during configure.
- [x] README and build guide match commands that were actually run.
