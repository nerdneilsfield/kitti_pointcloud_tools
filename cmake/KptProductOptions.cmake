option(BUILD_SHARED_LIBS "Enable compilation of shared libraries" OFF)
option(KPT_BUILD_TESTS "Enable kpt test builds" ON)
option(KPT_BUILD_GUI "Enable the Dear ImGui workbench" OFF)
option(KPT_BUILD_RENDER "Build the native headless renderer and kpt_render" ON)
option(KPT_BUILD_WEB "Build the Emscripten WebGL workbench" OFF)
option(KPT_BUILD_WASM_DECODER "Build the decoder-only Emscripten module" OFF)
option(KPT_ENABLE_PACKAGING "Enable native install and package metadata" OFF)
option(ENABLE_TOOLS "Enable the tools" ON)
set(KPT_GUI_BACKEND "auto" CACHE STRING
  "GUI renderer: auto, opengl, metal, webgl")
set_property(CACHE KPT_GUI_BACKEND PROPERTY STRINGS auto opengl metal webgl)

if(KPT_BUILD_WEB AND KPT_BUILD_WASM_DECODER)
  message(FATAL_ERROR
    "KPT_BUILD_WEB and KPT_BUILD_WASM_DECODER are separate Emscripten products")
endif()

# Product modes override their incompatible siblings in this configure only.
# Do not FORCE these choices into CMakeCache.txt: switching back to a native
# preset must restore the user's original cache choices rather than inherit a
# Web/WASM product's implementation details.
if(KPT_BUILD_WEB)
  if(NOT EMSCRIPTEN)
    message(FATAL_ERROR "KPT_BUILD_WEB requires the Emscripten toolchain")
  endif()
  set(KPT_BUILD_GUI ON)
  set(KPT_BUILD_RENDER OFF)
  set(KPT_BUILD_TESTS OFF)
  set(ENABLE_TOOLS OFF)
  add_compile_options(-pthread -fexceptions)
  add_link_options(-pthread -fexceptions)
endif()

if(KPT_BUILD_WASM_DECODER)
  if(NOT EMSCRIPTEN)
    message(FATAL_ERROR
      "KPT_BUILD_WASM_DECODER requires the Emscripten toolchain")
  endif()
  set(KPT_BUILD_GUI OFF)
  set(KPT_BUILD_RENDER OFF)
  set(KPT_BUILD_TESTS OFF)
  set(ENABLE_TOOLS OFF)
  # The decoder is deliberately single-threaded. CMake 4.4 no longer detects
  # Emscripten's placeholder pthread library unless -pthread is enabled, while
  # vendored spdlog requires Threads even for its synchronous configuration.
  set(CMAKE_HAVE_LIBC_PTHREAD ON CACHE INTERNAL "" FORCE)
  set(CMAKE_THREAD_LIBS_INIT "" CACHE STRING "" FORCE)
  set(CMAKE_USE_PTHREADS_INIT ON CACHE INTERNAL "" FORCE)
endif()

# Validate the public cache value even for headless configurations. Platform
# resolution is intentionally deferred until a GUI runtime is requested.
kpt_validate_gui_backend(
  REQUESTED "${KPT_GUI_BACKEND}"
  OUT_VAR KPT_VALIDATED_GUI_BACKEND)
