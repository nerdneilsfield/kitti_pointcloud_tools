# Cross-platform build rules

- macOS is a first-class target. Treat Apple Clang, libc++, Metal, deployment
  targets, filesystem behavior, and GUI backends as distinct from Linux.
- Never rely on a compiler's default C or C++ language mode. Every target,
  including vendored or support libraries built by this project, must declare
  its required standard explicitly (for example `cxx_std_20`).
- Any CMake, renderer, GUI, packaging, or platform change must preserve macOS
  compatibility. Run the relevant macOS preset or let the macOS CI job finish
  before claiming such work complete.
- Do not replace Apple-specific behavior with Linux-only flags, paths, tools,
  or assumptions. Keep platform branches explicit and tested.
