# Cross-platform build baseline

Captured on 2026-07-28 before target and platform refactoring.

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

## vcpkg manifest baseline

The manifest pins vcpkg commit
`99e82d9c9f0b281ec11fba48cc8434574a2b6e66`. Inspection of the port manifests
at that exact commit established:

- `pcl` is version `1.15.1#1`;
- its `visualization` feature depends on `vtk[opengl]`;
- `opencv4` is version `4.12.0#7`;
- its `png` feature is available.

This host had no `VCPKG_ROOT`, so no vcpkg resolver, cold configure, or cold
build was run. Those timings and resolver results remain unverified; the
Linux system-package preset is the only Task 4 build tested here.
