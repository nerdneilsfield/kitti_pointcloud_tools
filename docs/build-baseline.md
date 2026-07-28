# Cross-platform build baseline

Captured on 2026-07-28 before target and platform refactoring.

> **Historical record:** This snapshot predates KPT's native point-cloud
> codecs. PCL versions and targets below describe the old build only; the
> current source neither discovers nor links PCL, and the legacy
> `pc_viewer`/interactive `pc_player` commands now delegate to the same native
> workbench renderer as `pc_gui`.

## Host and toolchain

- Host: Ubuntu 24.04, Linux 6.17.0, x86_64
- CMake: 4.3.2
- Compiler: GCC/G++ 13.3.0
- PCL: 1.14.0, from system packages
- OpenCV: 4.6.0, from system packages
- GLFW: vendored 3.4.0
- Dear ImGui: vendored 1.92.8 docking
- Window system during smoke test: forwarded X11 (`DISPLAY=localhost:11.0`);
  no Wayland display

## Commands

```bash
cmake -S . -B build/cross-platform-baseline \
  -DCMAKE_BUILD_TYPE=Debug \
  -DKPT_BUILD_GUI=ON
cmake --build build/cross-platform-baseline -j
ctest --test-dir build/cross-platform-baseline --output-on-failure
```

Configure and build completed. CTest ran three tests: `kpt_tests` and
`kpt_gui_tests` passed; `pc_gui_smoke` failed. The baseline is therefore two
of three tests passing.

`pc_gui_smoke` aborted in Dear ImGui font setup:

```text
ImFontAtlas::AddFont: Cannot use MergeMode with an explicit reference size
when the destination font used an implicit reference size
```

This is a known pre-refactor failure, not caused by the cross-platform target
work. GUI configure additionally requires Linux/X11, OpenGL, and the vendored
GLFW tree. The current CMake configuration rejects GUI builds on non-Linux
hosts and forces GLFW's X11 backend.

## Historical vcpkg manifest baseline

At the time of this snapshot, the manifest pinned vcpkg commit
`99e82d9c9f0b281ec11fba48cc8434574a2b6e66`. Inspection of the port manifests
at that exact commit established:

- `pcl` is version `1.15.1#1`;
- its `visualization` feature depends on `vtk[opengl]`;
- `opencv4` is version `4.12.0#7`;
- its `png` feature is available.

This host had no `VCPKG_ROOT`, so no vcpkg resolver, cold configure, or cold
build was run. Those timings and resolver results remain unverified; the
Linux system-package preset is the only Task 4 build tested here.

## Task 14 macOS Metal gate

Task 14 was reached on 2026-07-28, but this host is Linux x86-64 and has no
`xcrun` or `xcodebuild`; `VCPKG_ROOT` is also unset. Therefore none of these
results exists yet:

- arm64/x86-64 macOS vcpkg dependency resolution;
- Objective-C++ or `.metal` compilation;
- Metal offscreen renderer contract execution;
- Retina, sleep/wake, or repeated lifecycle verification;
- proof from a macOS link result that no OpenGL framework is present.

Read-only inspection of
`third_party/imgui-1.92.8-docking/examples/example_glfw_metal/main.mm`
confirmed the intended `GLFW_NO_API` + `CAMetalLayer` integration and its
single-command-buffer encode/present/commit order. This does not substitute
for running the example.

The available negative configure probe was:

```bash
cmake -S . -B /tmp/kpt-task14-probe \
  -DKPT_BUILD_GUI=ON \
  -DKPT_GUI_BACKEND=metal
```

It failed as intended with:

```text
Backend 'metal' is not supported on Linux
```

No Metal `.mm` or MSL implementation was added on this non-Apple host. Task 14
remains open until a macOS 13+ machine with Xcode command-line tools and the
pinned vcpkg baseline can execute every implementation and verification step.

## Final Linux acceptance after Tasks 1-13

A clean `linux-system-debug` preset build was run on 2026-07-28. The explicit
`build/linux-system-debug` directory was removed first, then regenerated.

```bash
cmake --preset linux-system-debug
cmake --build --preset linux-system-debug -j2
xvfb-run -a ctest --preset linux-system-debug
xvfb-run -a ./build/linux-system-debug/pc_gui --smoke-test
```

Configure passed, all 114 Ninja build edges passed, all seven CTest tests
passed under Xvfb, and the independent GUI smoke command exited successfully.
This resolves the historical pre-refactor font-atlas smoke failure recorded
above.

The final pre-native-codec host evidence was CMake 4.3.2, Ninja 1.11.1, GCC
13.3.0, OpenCV 4.6.0, Fontconfig 2.15.0, and Freetype 2.13.2 on Ubuntu 24.04
x86-64. PCL 1.14.0 was installed and linked by that historical revision.
The generated `pc_gui` link statement contained
`libkpt_gui_backend_opengl.a` and no Metal backend.

No Windows, macOS, or vcpkg acceptance was performed on this Linux host.
Detailed commands, prerequisites, status, and evidence are maintained in
`docs/cross-platform-build.md`.

Native-codec acceptance is intentionally not appended to this baseline.
Current verification belongs in `docs/cross-platform-build.md` after a clean
configure, build and test run of the PCL-free source.
