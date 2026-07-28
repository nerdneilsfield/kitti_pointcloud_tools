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
