cmake_minimum_required(VERSION 3.21)

#set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" KPT_VERSION
  LIMIT_COUNT 1 ENCODING UTF-8)
if(NOT KPT_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
  message(FATAL_ERROR "VERSION must contain one X.Y.Z semantic version")
endif()

# Set the project name to your project name, my project isn't very descriptive
project(KITTI_POINTCLOUD_TOOLS VERSION "${KPT_VERSION}" LANGUAGES CXX C)

# Every C++ target, including vendored/support targets that do not consume our
# project_options interface, must compile in the same language mode. Apple
# Clang defaults those targets to C++98 otherwise; that breaks modern ImGui and
# can silently diverge from Linux's newer compiler defaults.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(APPLE)
  # Keep Objective-C++ targets aligned with the C++ ABI and language mode.
  set(CMAKE_OBJCXX_STANDARD 20)
  set(CMAKE_OBJCXX_STANDARD_REQUIRED ON)
  set(CMAKE_OBJCXX_EXTENSIONS OFF)
endif()

# Git commit hash (short) for About dialog
execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE KPT_GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(NOT KPT_GIT_COMMIT_HASH)
  set(KPT_GIT_COMMIT_HASH "unknown")
endif()

# CMake honors SOURCE_DATE_EPOCH in string(TIMESTAMP), making release metadata
# reproducible. Reject malformed input rather than silently emitting local time.
if(DEFINED ENV{SOURCE_DATE_EPOCH} AND
   NOT "$ENV{SOURCE_DATE_EPOCH}" MATCHES "^[0-9]+$")
  message(FATAL_ERROR "SOURCE_DATE_EPOCH must be a non-negative Unix timestamp")
endif()
# Build timestamp (UTC)
string(TIMESTAMP KPT_BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S UTC" UTC)

include(cmake/StandardProjectSettings.cmake)
include(cmake/KptIcons.cmake)
include(cmake/KptTargetAssertions.cmake)
include(cmake/i18n.cmake)

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_20)
if(APPLE)
  # Xcode 25 and earlier guard C++20 stop_token/jthread behind libc++'s
  # experimental library switch. The implementation is linked statically,
  # preserving the macOS 13 deployment target.
  target_compile_options(project_options INTERFACE -fexperimental-library)
  target_link_options(project_options INTERFACE -fexperimental-library)
endif()
if(WIN32)
  # Keep Windows SDK compatibility macros consistent across first-party targets,
  # including translation units that receive Windows headers transitively.
  target_compile_definitions(project_options INTERFACE
    NOMINMAX WIN32_LEAN_AND_MEAN)
endif()
if(MSVC)
  # Source files, narrow literals, diagnostics, and third-party UTF-8 APIs must
  # agree on one encoding under both cl.exe and clang-cl.
  target_compile_options(project_options INTERFACE /utf-8)
  target_compile_definitions(project_options INTERFACE KPT_MSVC_UTF8=1)
endif()
kpt_assert_msvc_utf8(project_options)
function(kpt_enable_windows_utf8_manifest target)
  if(NOT MSVC)
    return()
  endif()
  # Keep process narrow-string boundaries UTF-8 for vendored UI libraries and
  # diagnostics. Core filesystem I/O uses native std::filesystem::path values.
  target_link_options("${target}" PRIVATE
    /MANIFEST:EMBED
    "/MANIFESTINPUT:${CMAKE_CURRENT_SOURCE_DIR}/cmake/windows-utf8.manifest")
endfunction()

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

# sanitizer options if supported by compiler
include(cmake/Sanitizers.cmake)
enable_sanitizers(project_options)

# enable doxygen
#include(cmake/Doxygen.cmake)
#enable_doxygen()

# allow for static analysis options
include(cmake/StaticAnalyzers.cmake)

# # catch cmake
# include(cmake/Catch.cmake)

include(cmake/KptProductOptions.cmake)

#find_package(spdlog REQUIRED)
#find_package(yaml-cpp CONFIG REQUIRED)

#find_package(sioclient REQUIRED)

add_subdirectory(third_party)

if(EMSCRIPTEN)
  add_library(kpt_common STATIC
    src/platform/native_file_web.cpp
    src/platform/utf8_path_posix.cpp)
  target_compile_features(kpt_common PUBLIC cxx_std_20)
  target_include_directories(kpt_common PUBLIC src)
elseif(WIN32)
  add_library(kpt_common STATIC
    src/platform/native_file_windows.cpp
    src/platform/utf8_path_windows.cpp)
  target_compile_features(kpt_common PUBLIC cxx_std_20)
  target_include_directories(kpt_common PUBLIC src)
else()
  add_library(kpt_common STATIC
    src/platform/native_file_posix.cpp
    src/platform/utf8_path_posix.cpp)
  target_compile_features(kpt_common PUBLIC cxx_std_20)
  target_include_directories(kpt_common PUBLIC src)
endif()

# i18n: JSON source files -> embedded C++ translation maps
set(KPT_I18N_GENERATED "${CMAKE_CURRENT_BINARY_DIR}/generated/i18n_generated.cpp")
kpt_generate_i18n("${CMAKE_CURRENT_SOURCE_DIR}/i18n" "${KPT_I18N_GENERATED}")
add_library(kpt_i18n STATIC
  src/i18n/i18n.cpp
  "${KPT_I18N_GENERATED}")
target_compile_features(kpt_i18n PUBLIC cxx_std_20)
target_include_directories(kpt_i18n PUBLIC src)
target_link_libraries(kpt_i18n PRIVATE project_options project_warnings)

if(KPT_BUILD_GUI OR KPT_BUILD_TESTS)
  if(EMSCRIPTEN)
    add_library(kpt_platform STATIC
      src/platform/web/services.cpp)
    target_compile_features(kpt_platform PUBLIC cxx_std_20)
    target_include_directories(kpt_platform PUBLIC src)
    target_link_libraries(kpt_platform
      PUBLIC kpt_common
      PRIVATE project_options project_warnings)
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(PkgConfig REQUIRED)
    find_package(Freetype REQUIRED)
    pkg_check_modules(Fontconfig REQUIRED IMPORTED_TARGET fontconfig)

    add_library(kpt_platform STATIC
      src/platform/common/settings_store.cpp
      src/platform/posix/atomic_replace.cpp
      src/platform/linux/fonts.cpp
      src/platform/linux/services.cpp)
    target_compile_features(kpt_platform PUBLIC cxx_std_20)
    target_include_directories(kpt_platform PUBLIC src)
    target_link_libraries(kpt_platform
      PUBLIC kpt_common
      PRIVATE
        Freetype::Freetype
        PkgConfig::Fontconfig
        project_options
        project_warnings)
  elseif(APPLE)
    enable_language(OBJCXX)
    find_package(Freetype REQUIRED)
    find_library(KPT_FOUNDATION_FRAMEWORK Foundation REQUIRED)
    find_library(KPT_CORETEXT_FRAMEWORK CoreText REQUIRED)

    add_library(kpt_platform STATIC
      src/platform/common/settings_store.cpp
      src/platform/posix/atomic_replace.cpp
      src/platform/macos/fonts.mm
      src/platform/macos/services.mm)
    target_compile_features(kpt_platform PUBLIC cxx_std_20)
    set_target_properties(kpt_platform PROPERTIES
      OBJCXX_STANDARD 20
      OBJCXX_STANDARD_REQUIRED YES
      OBJCXX_EXTENSIONS NO)
    target_include_directories(kpt_platform PUBLIC src)
    target_link_libraries(kpt_platform
      PUBLIC kpt_common
      PRIVATE
        Freetype::Freetype
        ${KPT_FOUNDATION_FRAMEWORK}
        ${KPT_CORETEXT_FRAMEWORK}
        project_options
        project_warnings)
  elseif(WIN32)
    add_library(kpt_platform STATIC
      src/platform/common/settings_store.cpp
      src/platform/windows/atomic_replace.cpp
      src/platform/windows/fonts.cpp
      src/platform/windows/services.cpp)
    target_compile_features(kpt_platform PUBLIC cxx_std_20)
    target_include_directories(kpt_platform PUBLIC src)
    target_compile_definitions(kpt_platform PRIVATE
      NOMINMAX UNICODE _UNICODE WIN32_LEAN_AND_MEAN)
    target_link_libraries(kpt_platform
      PUBLIC kpt_common
      PRIVATE bcrypt ole32 shell32 dwrite project_options project_warnings)
  endif()
endif()


# add_subdirectory(third_party)

# Very basic PCH example
option(ENABLE_PCH "Enable Precompiled Headers" OFF)
if (ENABLE_PCH)
  # This sets a global PCH parameter, each project will build its own PCH, which
  # is a good idea if any #define's change
  # 
  # 
  target_precompile_headers(project_options INTERFACE <chrono> <vector> <string> <map> <utility>)
endif()


add_library(kpt_codec STATIC
  src/kpt/io/io.cpp
  src/kpt/io/pcd_codec.cpp
  src/kpt/io/ply_codec.cpp
  src/kpt/io/las_codec.cpp
  src/kpt/io/pts_codec.cpp
  src/kpt/io/obj_codec.cpp
  src/kpt/io/npy_codec.cpp
)
target_link_libraries(kpt_codec
  PUBLIC
    kpt_common
    spdlog::spdlog
    project_options
    project_warnings
)
target_include_directories(kpt_codec SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include
)
target_include_directories(kpt_codec PUBLIC src)

add_library(kpt_core STATIC
  src/kpt/label/label.cpp
  src/kpt/workflow/workflow.cpp
)
target_link_libraries(kpt_core
  PUBLIC
    kpt_codec
    spdlog::spdlog
    project_options
    project_warnings
)
# Mark vendored spdlog include as SYSTEM to avoid -Werror on its headers
target_include_directories(kpt_core SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include
)
target_include_directories(kpt_core PUBLIC src)

if(KPT_BUILD_WASM_DECODER)
  add_executable(kpt_wasm_decoder
    bindings/wasm/kpt_wasm_decoder.cpp)
  target_link_libraries(kpt_wasm_decoder PRIVATE
    kpt_core project_options project_warnings)
  target_compile_options(kpt_wasm_decoder PRIVATE -fexceptions)
  target_link_options(kpt_wasm_decoder PRIVATE
    -fexceptions
    -sMODULARIZE=1
    -sEXPORT_NAME=createKptDecoder
    -sENVIRONMENT=web,worker,node
    -sALLOW_MEMORY_GROWTH=1
    -sINITIAL_MEMORY=67108864
    -sMAXIMUM_MEMORY=2147483648
    "-sINCOMING_MODULE_JS_API=['locateFile','wasmBinary']"
    "-sEXPORTED_RUNTIME_METHODS=['ccall','UTF8ToString','HEAPF32','HEAPU8']"
    -sASSERTIONS=$<IF:$<CONFIG:Debug>,1,0>)
  set_target_properties(kpt_wasm_decoder PROPERTIES
    OUTPUT_NAME kpt_decoder
    SUFFIX ".js"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/decoder")
endif()

# Temporary source-compatible name while in-tree consumers move to precise
# targets. This aliases the core only; it does not silently enable renderers.
add_library(kpt ALIAS kpt_core)

if(KPT_BUILD_RENDER OR KPT_BUILD_GUI)
  add_library(kpt_render_core STATIC
    src/kpt/render/render.cpp
    src/kpt/render/detail/stb_png.cpp
  )
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set_source_files_properties(src/kpt/render/detail/stb_png.cpp
      PROPERTIES COMPILE_OPTIONS "-Wno-null-dereference")
  endif()
  target_link_libraries(kpt_render_core
    PUBLIC
      kpt_core
      eigen3::eigen
  )
  target_include_directories(kpt_render_core PUBLIC src)
  target_include_directories(kpt_render_core SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/stb
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)

  add_library(kpt_legacy_gui_options STATIC
    src/cli/legacy_gui_options.cpp)
  target_include_directories(kpt_legacy_gui_options PUBLIC src)
  target_link_libraries(kpt_legacy_gui_options
    PRIVATE project_options project_warnings)

  add_library(kpt_legacy_player_snapshot STATIC
    src/cli/legacy_player_snapshot.cpp)
  target_include_directories(kpt_legacy_player_snapshot PUBLIC src)
  target_link_libraries(kpt_legacy_player_snapshot
    PUBLIC kpt_core
    PRIVATE kpt_render_core project_options project_warnings)
endif()

# Dear ImGui core and ImGuiFileDialog stay independent of the native window
# and renderer backends. This keeps UTF-8/dialog contracts runnable in
# headless platform presets.
if(KPT_BUILD_GUI OR KPT_BUILD_TESTS)
  add_library(imgui_core STATIC
    third_party/imgui-1.92.8-docking/imgui.cpp
    third_party/imgui-1.92.8-docking/imgui_demo.cpp
    third_party/imgui-1.92.8-docking/imgui_draw.cpp
    third_party/imgui-1.92.8-docking/imgui_tables.cpp
    third_party/imgui-1.92.8-docking/imgui_widgets.cpp
    third_party/imgui-1.92.8-docking/misc/cpp/imgui_stdlib.cpp)
  target_include_directories(imgui_core SYSTEM PUBLIC
    third_party/imgui-1.92.8-docking)
  # Apple Clang defaults vendored .cpp files to C++98 unless the target
  # declares its language level. ImGui 1.92 uses constexpr, so make the
  # requirement explicit instead of relying on Linux compiler defaults.
  target_compile_features(imgui_core PUBLIC cxx_std_20)

  if(NOT EMSCRIPTEN)
    add_library(imgui_file_dialog STATIC
      third_party/ImGuiFileDialog-0.6.8/ImGuiFileDialog.cpp)
    target_compile_features(imgui_file_dialog PUBLIC cxx_std_20)
    target_compile_definitions(imgui_file_dialog PUBLIC
      USE_STD_FILESYSTEM
      USE_PLACES_FEATURE
      PLACES_PANE_DEFAULT_SHOWN)
    target_include_directories(imgui_file_dialog PUBLIC
      third_party/ImGuiFileDialog-0.6.8)
    target_link_libraries(imgui_file_dialog PUBLIC imgui_core)
  endif()

  add_library(kpt_gui_contracts INTERFACE)
  target_compile_features(kpt_gui_contracts INTERFACE cxx_std_20)
  target_include_directories(kpt_gui_contracts INTERFACE src)
  if(TARGET Eigen3::Eigen)
    get_target_property(KPT_SYSTEM_EIGEN_INCLUDE_DIRS
      Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(kpt_gui_contracts SYSTEM INTERFACE
      ${KPT_SYSTEM_EIGEN_INCLUDE_DIRS})
  endif()
  target_link_libraries(kpt_gui_contracts INTERFACE
    imgui_core eigen3::eigen)

  add_library(kpt_gui_model STATIC
    src/gui/viewport/model.cpp)
  target_include_directories(kpt_gui_model PUBLIC src)
  target_link_libraries(kpt_gui_model
    PUBLIC kpt_gui_contracts
    PRIVATE project_options project_warnings)

  add_library(kpt_scene STATIC
    src/gui/scene/scene.cpp)
  target_include_directories(kpt_scene PUBLIC src)
  target_link_libraries(kpt_scene
    PUBLIC eigen3::eigen kpt_gui_model
    PRIVATE project_options project_warnings)

  # Pure world-to-viewport annotations. Keeping this independent of ImGui
  # lets native, WebGL, and tests share exact measurement visibility rules.
  add_library(kpt_measurement_overlay STATIC
    src/gui/measurement_overlay.cpp)
  target_include_directories(kpt_measurement_overlay PUBLIC src)
  target_link_libraries(kpt_measurement_overlay
    PUBLIC kpt_scene
    PRIVATE project_options project_warnings)

  add_library(kpt_roi_filter STATIC
    src/gui/roi_filter.cpp)
  target_include_directories(kpt_roi_filter PUBLIC src)
  target_link_libraries(kpt_roi_filter
    PUBLIC kpt_scene
    PRIVATE project_options project_warnings)

  add_library(kpt_inspection_settings STATIC
    src/gui/inspection_settings.cpp)
  target_include_directories(kpt_inspection_settings PUBLIC src)
  target_link_libraries(kpt_inspection_settings
    PUBLIC kpt_scene
    PRIVATE project_options project_warnings)

  add_library(kpt_inspection_share STATIC
    src/gui/inspection_share.cpp)
  target_include_directories(kpt_inspection_share PUBLIC src)
  target_link_libraries(kpt_inspection_share
    PUBLIC kpt_scene
    PRIVATE project_options project_warnings)

  add_library(kpt_inspection_export STATIC
    src/gui/inspection_export.cpp)
  target_include_directories(kpt_inspection_export PUBLIC src)
  target_link_libraries(kpt_inspection_export
    PUBLIC kpt_core
    PRIVATE project_options project_warnings)

  # Renderer capture occurs on the UI thread; RGB conversion and PNG file I/O
  # live here so App can hand a CPU-owned frame to JobSystem without exposing
  # backend-specific framebuffer types.
  if(TARGET kpt_render_core)
    add_library(kpt_viewport_capture STATIC
      src/gui/viewport/capture.cpp)
    target_include_directories(kpt_viewport_capture PUBLIC src)
    target_link_libraries(kpt_viewport_capture
      PUBLIC kpt_gui_contracts kpt_render_core
      PRIVATE project_options project_warnings)
  endif()

  add_library(kpt_viewport_cloud_adapter STATIC
    src/gui/viewport/cloud_adapter.cpp)
  target_include_directories(kpt_viewport_cloud_adapter PUBLIC src)
  target_link_libraries(kpt_viewport_cloud_adapter
    PUBLIC kpt_gui_contracts kpt_core
    PRIVATE project_options project_warnings)

  # Pure review-scene adapter.  Backend implementations adopt its render list
  # independently; this target does not replace the established single-cloud
  # viewport path.
  add_library(kpt_scene_render_adapter STATIC
    src/gui/scene/render_adapter.cpp)
  target_include_directories(kpt_scene_render_adapter PUBLIC src)
  target_link_libraries(kpt_scene_render_adapter
    PUBLIC kpt_scene kpt_viewport_cloud_adapter
    PRIVATE project_options project_warnings)

  # Compatibility compositor used by the native workbench while concrete
  # backends still expose the established single-cloud renderer contract.
  add_library(kpt_scene_viewport_compositor STATIC
    src/gui/viewport/scene_compositor.cpp)
  target_include_directories(kpt_scene_viewport_compositor PUBLIC src)
  target_link_libraries(kpt_scene_viewport_compositor
    PUBLIC kpt_scene_render_adapter
    PRIVATE project_options project_warnings)

  add_library(kpt_jobs STATIC
    src/gui/jobs/job_system.cpp
    src/gui/jobs/ui_events.cpp)
  target_include_directories(kpt_jobs PUBLIC src)
  target_link_libraries(kpt_jobs PRIVATE kpt_i18n project_options project_warnings)
endif()

# Optional Dear ImGui workbench. All three GUI dependencies are vendored so
# configure/build remains fully offline.
if(KPT_BUILD_GUI)
  kpt_resolve_gui_backend(
    REQUESTED "${KPT_VALIDATED_GUI_BACKEND}"
    SYSTEM "${CMAKE_SYSTEM_NAME}"
    OUT_VAR KPT_ACTIVE_GUI_BACKEND)

  if(APPLE)
    # GLFW's Cocoa sources are Objective-C (.m), not Objective-C++.
    enable_language(OBJC)
  endif()

  if(KPT_ACTIVE_GUI_BACKEND STREQUAL "opengl" AND
     NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" AND NOT WIN32)
    message(FATAL_ERROR
      "KPT_GUI_BACKEND=${KPT_ACTIVE_GUI_BACKEND} resolves successfully for "
      "${CMAKE_SYSTEM_NAME}, but the current OpenGL runtime supports "
      "Linux/X11 and Windows/Win32 only")
  endif()

  if(EMSCRIPTEN)
    add_library(glfw INTERFACE)
    target_compile_options(glfw INTERFACE --use-port=contrib.glfw3)
    target_link_options(glfw INTERFACE --use-port=contrib.glfw3)
  else()
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
      # These are Linux product choices. Leaving them undefined elsewhere lets
      # GLFW select its native Win32 or Cocoa platform without cache pollution.
      set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
      set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
    endif()
    add_subdirectory(third_party/glfw-3.4 EXCLUDE_FROM_ALL)
  endif()

  if(KPT_ACTIVE_GUI_BACKEND STREQUAL "opengl")
    find_package(OpenGL REQUIRED)

    # KPT owns this loader. Dear ImGui keeps its symbol-prefixed internal
    # loader; do not couple them through IMGUI_IMPL_OPENGL_LOADER_CUSTOM.
    add_library(kpt_glad STATIC third_party/glad/src/gl.c)
    target_include_directories(kpt_glad SYSTEM PUBLIC
      third_party/glad/include)
    set_target_properties(kpt_glad PROPERTIES
      C_STANDARD 11
      C_STANDARD_REQUIRED YES
      C_EXTENSIONS NO)

    add_library(imgui_glfw_opengl STATIC
      third_party/imgui-1.92.8-docking/backends/imgui_impl_glfw.cpp
      third_party/imgui-1.92.8-docking/backends/imgui_impl_opengl3.cpp)
    target_include_directories(imgui_glfw_opengl PUBLIC
      third_party/imgui-1.92.8-docking)
    target_link_libraries(imgui_glfw_opengl PUBLIC
      imgui_core glfw OpenGL::GL)

    add_library(kpt_gui_backend_opengl STATIC
      src/gui/backend/opengl/point_renderer.cpp
      src/gui/runtime/glfw_opengl_runtime.cpp)
    target_include_directories(kpt_gui_backend_opengl PUBLIC src)
    target_link_libraries(kpt_gui_backend_opengl
      PUBLIC kpt_gui_contracts
      PRIVATE kpt_glad glfw imgui_glfw_opengl kpt_platform spdlog::spdlog
        project_options project_warnings)
    target_include_directories(kpt_gui_backend_opengl SYSTEM PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
  elseif(KPT_ACTIVE_GUI_BACKEND STREQUAL "metal")
    if(NOT APPLE)
      message(FATAL_ERROR "Metal backend requires an Apple target")
    endif()
    enable_language(OBJCXX)
    find_library(KPT_METAL_FRAMEWORK Metal REQUIRED)
    find_library(KPT_COCOA_FRAMEWORK Cocoa REQUIRED)
    find_library(KPT_QUARTZCORE_FRAMEWORK QuartzCore REQUIRED)
    find_library(KPT_FOUNDATION_FRAMEWORK Foundation REQUIRED)
    find_program(KPT_XCRUN_EXECUTABLE xcrun REQUIRED)

    set(KPT_METAL_AIR
      "${CMAKE_CURRENT_BINARY_DIR}/kpt_point_shaders.air")
    set(KPT_METAL_LIBRARY
      "${CMAKE_CURRENT_BINARY_DIR}/kpt_point_shaders.metallib")
    add_custom_command(
      OUTPUT "${KPT_METAL_AIR}"
      COMMAND "${KPT_XCRUN_EXECUTABLE}" -sdk macosx metal -c
        "${CMAKE_CURRENT_SOURCE_DIR}/src/gui/backend/metal/point_shaders.metal"
        -o "${KPT_METAL_AIR}"
      DEPENDS src/gui/backend/metal/point_shaders.metal
      COMMENT "Compiling KPT Metal point shaders"
      VERBATIM)
    add_custom_command(
      OUTPUT "${KPT_METAL_LIBRARY}"
      COMMAND "${KPT_XCRUN_EXECUTABLE}" -sdk macosx metallib
        "${KPT_METAL_AIR}" -o "${KPT_METAL_LIBRARY}"
      DEPENDS "${KPT_METAL_AIR}"
      COMMENT "Linking KPT Metal shader library"
      VERBATIM)
    add_custom_target(kpt_metal_shaders DEPENDS "${KPT_METAL_LIBRARY}")
    function(kpt_copy_metal_library target)
      add_dependencies(${target} kpt_metal_shaders)
      add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${KPT_METAL_LIBRARY}"
          "$<TARGET_FILE_DIR:${target}>/kpt_point_shaders.metallib"
        VERBATIM)
    endfunction()

    add_library(imgui_glfw_metal STATIC
      third_party/imgui-1.92.8-docking/backends/imgui_impl_glfw.cpp
      third_party/imgui-1.92.8-docking/backends/imgui_impl_metal.mm)
    target_include_directories(imgui_glfw_metal PUBLIC
      third_party/imgui-1.92.8-docking)
    target_compile_options(imgui_glfw_metal PRIVATE
      $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)
    target_link_libraries(imgui_glfw_metal PUBLIC
      imgui_core glfw ${KPT_METAL_FRAMEWORK})

    add_library(kpt_gui_backend_metal STATIC
      src/gui/backend/metal/point_renderer.mm
      src/gui/runtime/glfw_metal_runtime.mm)
    target_include_directories(kpt_gui_backend_metal PUBLIC src)
    target_compile_options(kpt_gui_backend_metal PRIVATE
      $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)
    target_link_libraries(kpt_gui_backend_metal
      PUBLIC kpt_gui_contracts
      PRIVATE glfw imgui_glfw_metal kpt_platform spdlog::spdlog
        ${KPT_METAL_FRAMEWORK} ${KPT_COCOA_FRAMEWORK}
        ${KPT_QUARTZCORE_FRAMEWORK} ${KPT_FOUNDATION_FRAMEWORK}
        project_options project_warnings)
    target_include_directories(kpt_gui_backend_metal SYSTEM PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
  elseif(KPT_ACTIVE_GUI_BACKEND STREQUAL "webgl")
    add_library(imgui_glfw_webgl STATIC
      third_party/imgui-1.92.8-docking/backends/imgui_impl_glfw.cpp
      third_party/imgui-1.92.8-docking/backends/imgui_impl_opengl3.cpp)
    target_include_directories(imgui_glfw_webgl PUBLIC
      third_party/imgui-1.92.8-docking)
    target_link_libraries(imgui_glfw_webgl PUBLIC imgui_core glfw)

    add_library(kpt_gui_backend_webgl STATIC
      src/gui/backend/opengl/point_renderer.cpp
      src/gui/runtime/glfw_webgl_runtime.cpp)
    target_include_directories(kpt_gui_backend_webgl PUBLIC src)
    target_link_libraries(kpt_gui_backend_webgl
      PUBLIC kpt_gui_contracts
      PRIVATE glfw imgui_glfw_webgl kpt_platform spdlog::spdlog
        project_options project_warnings)
    target_include_directories(kpt_gui_backend_webgl SYSTEM PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
  endif()

  if(EMSCRIPTEN)
    add_library(kpt_gui_app STATIC
      src/gui/app.cpp
      src/gui/web/bridge.cpp
      src/gui/web/catalog.cpp
      src/gui/viewport/session.cpp)
    target_compile_definitions(kpt_gui_app PRIVATE KPT_WEB_BUILD=1)
  else()
    add_library(kpt_gui_app STATIC
      src/gui/app.cpp
      src/gui/dialog_paths.cpp
      src/gui/runner.cpp
      src/gui/viewport/session.cpp)
  endif()
  target_include_directories(kpt_gui_app PUBLIC src)
  target_include_directories(kpt_gui_app SYSTEM PRIVATE
    third_party/imgui-1.92.8-docking
    third_party/spdlog/include)
  if(NOT EMSCRIPTEN)
    target_include_directories(kpt_gui_app SYSTEM PRIVATE
      third_party/ImGuiFileDialog-0.6.8)
  endif()
  target_link_libraries(kpt_gui_app
    PUBLIC kpt_gui_contracts kpt_gui_model kpt_scene kpt_measurement_overlay kpt_inspection_settings kpt_jobs kpt_core project_options
    PRIVATE kpt_common kpt_render_core kpt_platform kpt_i18n
      kpt_viewport_cloud_adapter kpt_scene_render_adapter
      kpt_scene_viewport_compositor project_warnings)
  target_compile_definitions(kpt_gui_app PRIVATE
    KPT_VERSION_STRING="${KPT_VERSION}"
    KPT_GIT_HASH="${KPT_GIT_COMMIT_HASH}"
    KPT_BUILD_TIME="${KPT_BUILD_TIMESTAMP}")
  if(NOT EMSCRIPTEN)
    target_link_libraries(kpt_gui_app PRIVATE
      imgui_file_dialog
      kpt_roi_filter
      kpt_inspection_export
      kpt_inspection_share
      kpt_viewport_capture)
  endif()

  # Create only the selected, implemented backend target. Adding an
  # unselected placeholder target is intentionally an error: one executable
  # must contain exactly one concrete backend.
  kpt_assert_exactly_one_target(
    ACTIVE "kpt_gui_backend_${KPT_ACTIVE_GUI_BACKEND}"
    CANDIDATES kpt_gui_backend_opengl kpt_gui_backend_metal
      kpt_gui_backend_webgl)
  set(KPT_ACTIVE_GUI_BACKEND_TARGET
      "kpt_gui_backend_${KPT_ACTIVE_GUI_BACKEND}")

  if(EMSCRIPTEN)
    add_executable(kpt_web src/gui/web/pc_web.cc)
    target_link_libraries(kpt_web PRIVATE
      kpt_platform kpt_gui_app ${KPT_ACTIVE_GUI_BACKEND_TARGET})
    target_compile_definitions(kpt_web PRIVATE
      KPT_WEB_BUILD=1
      KPT_VERSION_STRING="${KPT_VERSION}"
      KPT_GIT_HASH="${KPT_GIT_COMMIT_HASH}"
      KPT_BUILD_TIME="${KPT_BUILD_TIMESTAMP}")
    target_include_directories(kpt_web SYSTEM PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
    set_target_properties(kpt_web PROPERTIES
      OUTPUT_NAME index
      SUFFIX ".html"
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/site"
      LINK_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/web/shell.html;${CMAKE_CURRENT_SOURCE_DIR}/web/kpt_web.js;${CMAKE_CURRENT_SOURCE_DIR}/web/assets/NotoSansSC-VF.ttf")
    target_compile_options(kpt_web PRIVATE -pthread)
    target_link_options(kpt_web PRIVATE
      -pthread
      --shell-file=${CMAKE_CURRENT_SOURCE_DIR}/web/shell.html
      --pre-js=${CMAKE_CURRENT_SOURCE_DIR}/web/kpt_web.js
      --preload-file
        ${CMAKE_CURRENT_SOURCE_DIR}/web/assets/NotoSansSC-VF.ttf@/assets/NotoSansSC-VF.ttf
      -sUSE_WEBGL2=1
      -sFULL_ES3=1
      -sPTHREAD_POOL_SIZE=4
      -sINITIAL_MEMORY=268435456
      -sMAXIMUM_MEMORY=2147483648
      -sALLOW_MEMORY_GROWTH=1
      -Wno-pthreads-mem-growth
      -sSTACK_SIZE=5242880
      -sNO_EXIT_RUNTIME=1
      # The browser bridge owns transient UTF-8 and RGBA test buffers. Keep
      # allocation explicit in the public Module ABI rather than depending on
      # Emscripten's version-dependent implicit runtime exports.
      "-sEXPORTED_FUNCTIONS=['_main','_malloc','_free']"
      "-sEXPORTED_RUNTIME_METHODS=['ccall','FS','HEAPU8','UTF8ToString','stringToNewUTF8','lengthBytesUTF8']"
      -sASSERTIONS=$<IF:$<CONFIG:Debug>,1,0>)
  else()
    add_executable(kpt_gui src/gui/pc_gui.cc)
    if(APPLE)
      set_target_properties(kpt_gui PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_BUNDLE_NAME "KPT Workbench"
        MACOSX_BUNDLE_GUI_IDENTIFIER
          "io.github.nerdneilsfield.kitti-pointcloud-tools"
        MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
        MACOSX_BUNDLE_INFO_PLIST
          "${CMAKE_CURRENT_SOURCE_DIR}/packaging/macos/Info.plist.in")
    endif()
    kpt_enable_windows_utf8_manifest(kpt_gui)
    if(WIN32)
      kpt_attach_windows_icon(kpt_gui)
    endif()
    target_link_libraries(kpt_gui PRIVATE
      kpt_platform kpt_gui_app ${KPT_ACTIVE_GUI_BACKEND_TARGET})
    target_include_directories(kpt_gui SYSTEM PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
    target_compile_definitions(kpt_gui PRIVATE
      KPT_ACTIVE_GUI_BACKEND="${KPT_ACTIVE_GUI_BACKEND}")

    add_executable(kpt_viewer src/cli/pc_viewer.cc)
    kpt_enable_windows_utf8_manifest(kpt_viewer)
    target_link_libraries(kpt_viewer PRIVATE
      kpt_legacy_gui_options kpt_platform kpt_gui_app
      ${KPT_ACTIVE_GUI_BACKEND_TARGET})
    target_include_directories(kpt_viewer SYSTEM PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)

    if(KPT_ACTIVE_GUI_BACKEND STREQUAL "metal")
      kpt_copy_metal_library(kpt_gui)
      kpt_copy_metal_library(kpt_viewer)
    endif()
  endif()

endif()

if((KPT_BUILD_RENDER OR KPT_BUILD_GUI) AND NOT EMSCRIPTEN)
  add_executable(kpt_player src/cli/pc_player.cc)
  kpt_enable_windows_utf8_manifest(kpt_player)
  target_link_libraries(kpt_player PRIVATE
    kpt_legacy_gui_options kpt_legacy_player_snapshot kpt_common
    project_options project_warnings)
  target_compile_definitions(kpt_player PRIVATE
    KPT_HAS_GUI=$<BOOL:${KPT_BUILD_GUI}>)
  if(KPT_BUILD_GUI)
    target_link_libraries(kpt_player PRIVATE
      kpt_platform kpt_gui_app ${KPT_ACTIVE_GUI_BACKEND_TARGET})
    if(KPT_ACTIVE_GUI_BACKEND STREQUAL "metal")
      kpt_copy_metal_library(kpt_player)
    endif()
  else()
    # Keep snapshot-only player genuinely display-free. Traverse target
    # dependencies so an intermediate helper cannot conceal a display library.
    set(snapshot_link_queue kpt_player)
    set(snapshot_link_visited)
    while(snapshot_link_queue)
      list(POP_FRONT snapshot_link_queue snapshot_target)
      if(snapshot_target IN_LIST snapshot_link_visited OR
         NOT TARGET ${snapshot_target})
        continue()
      endif()
      list(APPEND snapshot_link_visited ${snapshot_target})
      get_target_property(snapshot_alias ${snapshot_target} ALIASED_TARGET)
      if(snapshot_alias)
        list(APPEND snapshot_link_queue ${snapshot_alias})
      endif()
      foreach(snapshot_property LINK_LIBRARIES INTERFACE_LINK_LIBRARIES)
        get_target_property(snapshot_links ${snapshot_target}
          ${snapshot_property})
        if(NOT snapshot_links)
          continue()
        endif()
        foreach(snapshot_link IN LISTS snapshot_links)
          if(snapshot_link MATCHES
              "kpt_gui|glfw|OpenGL::|Fontconfig|Freetype|X11|Metal|Cocoa|imgui|kpt_glad")
            message(FATAL_ERROR
              "GUI-off kpt_player link closure is not display-free: "
              "${snapshot_target}: ${snapshot_link}")
          endif()
          string(GENEX_STRIP "${snapshot_link}" snapshot_dependency)
          if(snapshot_dependency AND TARGET ${snapshot_dependency})
            list(APPEND snapshot_link_queue ${snapshot_dependency})
          endif()
          # GENEX_STRIP intentionally removes conditional payloads. Recover
          # target-looking tokens so LINK_ONLY/BUILD_INTERFACE helpers are
          # still traversed.
          string(REGEX MATCHALL "[A-Za-z0-9_:+.-]+"
            snapshot_link_tokens "${snapshot_link}")
          foreach(snapshot_token IN LISTS snapshot_link_tokens)
            if(TARGET ${snapshot_token})
              list(APPEND snapshot_link_queue ${snapshot_token})
            endif()
          endforeach()
        endforeach()
      endforeach()
      foreach(snapshot_property LINK_OPTIONS INTERFACE_LINK_OPTIONS)
        get_target_property(snapshot_options ${snapshot_target}
          ${snapshot_property})
        if(NOT snapshot_options)
          continue()
        endif()
        foreach(snapshot_option IN LISTS snapshot_options)
          string(TOLOWER "${snapshot_option}" snapshot_option_lower)
          if(snapshot_option_lower MATCHES
              "opengl|(^|[^a-z])-lgl([^a-z]|$)|x11|fontconfig|freetype|metal|cocoa|glfw|imgui")
            message(FATAL_ERROR
              "GUI-off kpt_player link options are not display-free: "
              "${snapshot_target}: ${snapshot_option}")
          endif()
        endforeach()
      endforeach()
    endwhile()
  endif()
  target_include_directories(kpt_player SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
endif()

# CLI tools
if(ENABLE_TOOLS)
  foreach(tool kpt_convert kpt_batch_convert kpt_info)
    string(REPLACE "kpt_" "pc_" tool_source "${tool}")
    add_executable(${tool} src/cli/${tool_source}.cc)
    kpt_enable_windows_utf8_manifest(${tool})
    target_link_libraries(${tool} PUBLIC
      kpt_core popl::popl project_options project_warnings)
    # Vendored spdlog exposes its include dir as PUBLIC non-system via the
    # spdlog target; promote to SYSTEM here so -Werror=shadow does not fire on
    # its bundled fmt headers (same workaround kpt itself uses above).
    target_include_directories(${tool} SYSTEM PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
  endforeach()
endif()

if(KPT_BUILD_RENDER)
  add_executable(kpt_render src/cli/pc_render.cc)
  kpt_enable_windows_utf8_manifest(kpt_render)
  target_link_libraries(kpt_render PUBLIC
    kpt_render_core popl::popl project_options project_warnings)
  target_include_directories(kpt_render SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
endif()

if(KPT_ENABLE_PACKAGING)
  include(GNUInstallDirs)
  set(KPT_PACKAGE_EXECUTABLES
    kpt_gui
    kpt_viewer
    kpt_player
    kpt_convert
    kpt_batch_convert
    kpt_info
    kpt_render)
  foreach(KPT_PACKAGE_EXECUTABLE IN LISTS KPT_PACKAGE_EXECUTABLES)
    if(NOT TARGET "${KPT_PACKAGE_EXECUTABLE}")
      message(FATAL_ERROR
        "KPT_ENABLE_PACKAGING requires target ${KPT_PACKAGE_EXECUTABLE}")
    endif()
  endforeach()

  if(APPLE)
    install(TARGETS kpt_gui BUNDLE DESTINATION "." COMPONENT Runtime)
    install(TARGETS
      kpt_viewer kpt_player kpt_convert kpt_batch_convert kpt_info kpt_render
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
      COMPONENT Runtime)
  elseif(WIN32)
    install(TARGETS ${KPT_PACKAGE_EXECUTABLES}
      RUNTIME_DEPENDENCY_SET kpt_windows_runtime_dependencies
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
      COMPONENT Runtime)
    install(RUNTIME_DEPENDENCY_SET kpt_windows_runtime_dependencies
      DIRECTORIES
        "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin"
      PRE_EXCLUDE_REGEXES
        "api-ms-.*"
        "ext-ms-.*"
      POST_EXCLUDE_REGEXES
        ".*[Ww][Ii][Nn][Dd][Oo][Ww][Ss][/\\\\][Ss][Yy][Ss][Tt][Ee][Mm]32[/\\\\].*"
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
      COMPONENT Runtime)
  else()
    install(TARGETS ${KPT_PACKAGE_EXECUTABLES}
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
      COMPONENT Runtime)
  endif()
  install(FILES LICENSE THIRD_PARTY_NOTICES.md README.md README.zh-CN.md VERSION
    DESTINATION "${CMAKE_INSTALL_DATADIR}/doc/kitti-pointcloud-tools"
    COMPONENT Runtime)

  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    kpt_generate_linux_icons(KPT_LINUX_ICON_FILES)
    install(FILES packaging/linux/kitti-pointcloud-tools.desktop
      DESTINATION "${CMAKE_INSTALL_DATADIR}/applications"
      COMPONENT Runtime)
    install(FILES packaging/icons/kpt-workbench.svg
      DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps"
      RENAME kpt-workbench.svg
      COMPONENT Runtime)
    foreach(size IN ITEMS 16 32 48 64 128 256)
      install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/icons/${size}x${size}/apps/kpt-workbench.png"
        DESTINATION
          "${CMAKE_INSTALL_DATADIR}/icons/hicolor/${size}x${size}/apps"
        COMPONENT Runtime)
    endforeach()
    include(packaging/CPackDeb.cmake)
  elseif(WIN32)
    include(packaging/CPackWindows.cmake)
  endif()
endif()

# Tests
if(KPT_BUILD_TESTS)
  add_executable(kpt_tests tests/test_main.cpp tests/io_test.cpp
                 tests/pcd_io_test.cpp tests/ply_io_test.cpp
                 tests/conversion_options_test.cpp tests/label_test.cpp
                 tests/workflow_test.cpp tests/utf8_path_test.cpp
                 tests/web_catalog_test.cpp src/gui/web/catalog.cpp)
  kpt_enable_windows_utf8_manifest(kpt_tests)
  if(TARGET kpt_platform)
    target_sources(kpt_tests PRIVATE tests/platform_services_test.cpp)
    target_link_libraries(kpt_tests PRIVATE kpt_platform)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
      target_link_libraries(kpt_tests PRIVATE
        Freetype::Freetype PkgConfig::Fontconfig)
    elseif(APPLE)
      target_link_libraries(kpt_tests PRIVATE Freetype::Freetype)
    endif()
  endif()
  target_link_libraries(kpt_tests PRIVATE
    kpt_core Catch2 project_options project_warnings)
  if(KPT_BUILD_RENDER)
    target_sources(kpt_tests PRIVATE tests/render_test.cpp)
    target_link_libraries(kpt_tests PRIVATE kpt_render_core)
    target_include_directories(kpt_tests SYSTEM PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party/stb)
  endif()
  # Catch2 v2's __COUNTER__ feature triggers Clang 22's pedantic C2y
  # extension diagnostic at each TEST_CASE call site.
  target_compile_definitions(kpt_tests PRIVATE CATCH_CONFIG_NO_COUNTER)
  # Vendored Catch2 v2 has a glibc>=2.34 MINSIGSTKSZ constexpr bug; disable POSIX signals.
  if(UNIX)
    target_compile_definitions(kpt_tests PRIVATE
      CATCH_CONFIG_NO_POSIX_SIGNALS)
  endif()
  enable_testing()
  add_test(NAME kpt_tests COMMAND kpt_tests WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  set(KPT_CLI_CONTRACT_ARGS
    -DKPT_INFO=$<TARGET_FILE:kpt_info>)
  if(TARGET kpt_render)
    list(APPEND KPT_CLI_CONTRACT_ARGS
      -DKPT_RENDER=$<TARGET_FILE:kpt_render>)
  endif()
  add_test(NAME kpt_cli_contract
    COMMAND ${CMAKE_COMMAND} ${KPT_CLI_CONTRACT_ARGS}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/cli_contract_test.cmake)

  add_executable(kpt_dialog_tests
    tests/test_main.cpp
    tests/dialog_test.cpp
    src/gui/dialog_paths.cpp)
  kpt_enable_windows_utf8_manifest(kpt_dialog_tests)
  target_link_libraries(kpt_dialog_tests PRIVATE
    kpt_common Catch2 imgui_file_dialog project_options project_warnings)
  target_include_directories(kpt_dialog_tests SYSTEM PRIVATE
    third_party/imgui-1.92.8-docking
    third_party/ImGuiFileDialog-0.6.8)
  target_compile_definitions(kpt_dialog_tests PRIVATE CATCH_CONFIG_NO_COUNTER)
  if(UNIX)
    target_compile_definitions(kpt_dialog_tests PRIVATE
      CATCH_CONFIG_NO_POSIX_SIGNALS)
  endif()
  add_test(NAME kpt_dialog_tests COMMAND kpt_dialog_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    add_executable(viewport_model_tests
      tests/test_main.cpp
      tests/viewport_model_test.cpp
      tests/viewport_projection_test.cpp)
    kpt_enable_windows_utf8_manifest(viewport_model_tests)
    target_link_libraries(viewport_model_tests PRIVATE
    kpt_gui_model kpt_viewport_cloud_adapter Catch2
    project_options project_warnings)
  target_compile_definitions(viewport_model_tests PRIVATE
    CATCH_CONFIG_NO_COUNTER)
  if(UNIX)
    target_compile_definitions(viewport_model_tests PRIVATE
      CATCH_CONFIG_NO_POSIX_SIGNALS)
  endif()
  add_test(NAME viewport_model_tests COMMAND viewport_model_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  add_executable(scene_tests
    tests/test_main.cpp
    tests/scene_test.cpp)
  kpt_enable_windows_utf8_manifest(scene_tests)
  target_link_libraries(scene_tests PRIVATE
    kpt_scene Catch2 project_options project_warnings)
  target_compile_definitions(scene_tests PRIVATE CATCH_CONFIG_NO_COUNTER)
  if(UNIX)
    target_compile_definitions(scene_tests PRIVATE
      CATCH_CONFIG_NO_POSIX_SIGNALS)
  endif()
  add_test(NAME scene_tests COMMAND scene_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  add_executable(scene_render_adapter_tests
    tests/test_main.cpp
    tests/scene_render_adapter_test.cpp)
  kpt_enable_windows_utf8_manifest(scene_render_adapter_tests)
  target_link_libraries(scene_render_adapter_tests PRIVATE
    kpt_scene_render_adapter kpt_scene_viewport_compositor Catch2 project_options project_warnings)
  target_compile_definitions(scene_render_adapter_tests PRIVATE
    CATCH_CONFIG_NO_COUNTER)
  if(UNIX)
    target_compile_definitions(scene_render_adapter_tests PRIVATE
      CATCH_CONFIG_NO_POSIX_SIGNALS)
  endif()
  add_test(NAME scene_render_adapter_tests COMMAND scene_render_adapter_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  add_executable(measurement_overlay_tests
    tests/test_main.cpp
    tests/measurement_overlay_test.cpp)
  kpt_enable_windows_utf8_manifest(measurement_overlay_tests)
  target_link_libraries(measurement_overlay_tests PRIVATE
    kpt_measurement_overlay Catch2 project_options project_warnings)
  target_compile_definitions(measurement_overlay_tests PRIVATE
    CATCH_CONFIG_NO_COUNTER)
  if(UNIX)
    target_compile_definitions(measurement_overlay_tests PRIVATE
      CATCH_CONFIG_NO_POSIX_SIGNALS)
  endif()
  add_test(NAME measurement_overlay_tests COMMAND measurement_overlay_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  add_executable(roi_filter_tests
    tests/test_main.cpp
    tests/roi_filter_test.cpp)
  kpt_enable_windows_utf8_manifest(roi_filter_tests)
  target_link_libraries(roi_filter_tests PRIVATE
    kpt_roi_filter Catch2 project_options project_warnings)
  target_compile_definitions(roi_filter_tests PRIVATE CATCH_CONFIG_NO_COUNTER)
  if(UNIX)
    target_compile_definitions(roi_filter_tests PRIVATE
      CATCH_CONFIG_NO_POSIX_SIGNALS)
  endif()
  add_test(NAME roi_filter_tests COMMAND roi_filter_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  add_executable(inspection_settings_tests
    tests/test_main.cpp
    tests/inspection_settings_test.cpp)
  kpt_enable_windows_utf8_manifest(inspection_settings_tests)
  target_link_libraries(inspection_settings_tests PRIVATE
    kpt_inspection_settings Catch2 project_options project_warnings)
  target_compile_definitions(inspection_settings_tests PRIVATE
    CATCH_CONFIG_NO_COUNTER)
  if(UNIX)
    target_compile_definitions(inspection_settings_tests PRIVATE
      CATCH_CONFIG_NO_POSIX_SIGNALS)
  endif()
  add_test(NAME inspection_settings_tests COMMAND inspection_settings_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  add_executable(inspection_share_tests
    tests/test_main.cpp
    tests/inspection_share_test.cpp)
  kpt_enable_windows_utf8_manifest(inspection_share_tests)
  target_link_libraries(inspection_share_tests PRIVATE
    kpt_inspection_share Catch2 project_options project_warnings)
  target_compile_definitions(inspection_share_tests PRIVATE CATCH_CONFIG_NO_COUNTER)
  if(UNIX)
    target_compile_definitions(inspection_share_tests PRIVATE
      CATCH_CONFIG_NO_POSIX_SIGNALS)
  endif()
  add_test(NAME inspection_share_tests COMMAND inspection_share_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  add_executable(inspection_export_tests
    tests/test_main.cpp
    tests/inspection_export_test.cpp)
  kpt_enable_windows_utf8_manifest(inspection_export_tests)
  target_link_libraries(inspection_export_tests PRIVATE
    kpt_inspection_export Catch2 project_options project_warnings)
  target_compile_definitions(inspection_export_tests PRIVATE
    CATCH_CONFIG_NO_COUNTER)
  if(UNIX)
    target_compile_definitions(inspection_export_tests PRIVATE
      CATCH_CONFIG_NO_POSIX_SIGNALS)
  endif()
  add_test(NAME inspection_export_tests COMMAND inspection_export_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  if(TARGET kpt_viewport_capture)
    add_executable(viewport_capture_tests
      tests/test_main.cpp
      tests/viewport_capture_test.cpp)
    kpt_enable_windows_utf8_manifest(viewport_capture_tests)
    target_link_libraries(viewport_capture_tests PRIVATE
      kpt_viewport_capture Catch2 project_options project_warnings)
    target_compile_definitions(viewport_capture_tests PRIVATE
      CATCH_CONFIG_NO_COUNTER)
    if(UNIX)
      target_compile_definitions(viewport_capture_tests PRIVATE
        CATCH_CONFIG_NO_POSIX_SIGNALS)
    endif()
    add_test(NAME viewport_capture_tests COMMAND viewport_capture_tests
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  endif()

  if(KPT_BUILD_RENDER OR KPT_BUILD_GUI)
    add_executable(legacy_gui_cli_tests
      tests/test_main.cpp
      tests/legacy_gui_options_test.cpp
      tests/legacy_player_snapshot_test.cpp)
    kpt_enable_windows_utf8_manifest(legacy_gui_cli_tests)
    target_link_libraries(legacy_gui_cli_tests PRIVATE
      kpt_legacy_gui_options kpt_legacy_player_snapshot Catch2
      project_options project_warnings)
    target_compile_definitions(legacy_gui_cli_tests PRIVATE
      CATCH_CONFIG_NO_COUNTER)
    if(UNIX)
      target_compile_definitions(legacy_gui_cli_tests PRIVATE
        CATCH_CONFIG_NO_POSIX_SIGNALS)
    endif()
    add_test(NAME legacy_gui_cli_tests COMMAND legacy_gui_cli_tests
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    add_test(NAME kpt_player_compat
      COMMAND ${CMAKE_COMMAND}
        -DKPT_PLAYER=$<TARGET_FILE:kpt_player>
        -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
        -DHAS_GUI=${KPT_BUILD_GUI}
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/pc-player-compat-中文
        -P ${CMAKE_SOURCE_DIR}/tests/pc_player_compat_test.cmake)
  endif()

  if(KPT_BUILD_GUI)
    if(KPT_ACTIVE_GUI_BACKEND STREQUAL "opengl")
      # Compile a separate backend variant so fault-injection entry points
      # never enter the production kpt_gui link graph.
      add_library(kpt_gui_backend_opengl_testable STATIC
        src/gui/backend/opengl/point_renderer.cpp
        src/gui/runtime/glfw_opengl_runtime.cpp)
      target_include_directories(kpt_gui_backend_opengl_testable PUBLIC src)
      target_compile_definitions(kpt_gui_backend_opengl_testable PRIVATE
        KPT_GUI_RUNTIME_TEST_SUPPORT)
      target_link_libraries(kpt_gui_backend_opengl_testable
        PUBLIC kpt_gui_contracts
        PRIVATE kpt_glad glfw imgui_glfw_opengl kpt_platform spdlog::spdlog
          project_options project_warnings)
      target_include_directories(kpt_gui_backend_opengl_testable SYSTEM PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)

      add_library(kpt_gui_opengl_test_support STATIC
        src/gui/backend/opengl/test_support.cpp)
      target_include_directories(kpt_gui_opengl_test_support PUBLIC src)
      target_link_libraries(kpt_gui_opengl_test_support
        PUBLIC kpt_gui_backend_opengl_testable
        PRIVATE kpt_glad project_options project_warnings)

      add_executable(opengl_renderer_tests
        tests/test_main.cpp
        tests/opengl_renderer_test.cpp)
      kpt_enable_windows_utf8_manifest(opengl_renderer_tests)
      target_link_libraries(opengl_renderer_tests PRIVATE
        kpt_gui_model kpt_gui_opengl_test_support kpt_glad Catch2 glfw
        project_options project_warnings)
      target_compile_definitions(opengl_renderer_tests PRIVATE
        CATCH_CONFIG_NO_COUNTER)
      if(UNIX)
        target_compile_definitions(opengl_renderer_tests PRIVATE
          CATCH_CONFIG_NO_POSIX_SIGNALS)
      endif()
      add_test(NAME opengl_renderer_tests COMMAND opengl_renderer_tests
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

      add_executable(opengl_runtime_smoke_tests
        tests/test_main.cpp
        tests/opengl_runtime_smoke_test.cpp)
      kpt_enable_windows_utf8_manifest(opengl_runtime_smoke_tests)
      target_link_libraries(opengl_runtime_smoke_tests PRIVATE
        kpt_gui_app kpt_gui_opengl_test_support kpt_glad Catch2 glfw
        project_options project_warnings)
      target_compile_definitions(opengl_runtime_smoke_tests PRIVATE
        CATCH_CONFIG_NO_COUNTER)
      if(UNIX)
        target_compile_definitions(opengl_runtime_smoke_tests PRIVATE
          CATCH_CONFIG_NO_POSIX_SIGNALS)
      endif()
      add_test(NAME opengl_runtime_smoke_tests
        COMMAND opengl_runtime_smoke_tests
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    elseif(KPT_ACTIVE_GUI_BACKEND STREQUAL "metal")
      add_library(kpt_gui_backend_metal_testable STATIC
        src/gui/backend/metal/point_renderer.mm
        src/gui/runtime/glfw_metal_runtime.mm)
      target_include_directories(kpt_gui_backend_metal_testable PUBLIC src)
      target_compile_definitions(kpt_gui_backend_metal_testable PRIVATE
        KPT_GUI_RUNTIME_TEST_SUPPORT)
      target_compile_options(kpt_gui_backend_metal_testable PRIVATE
        $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)
      target_link_libraries(kpt_gui_backend_metal_testable
        PUBLIC kpt_gui_contracts
        PRIVATE glfw imgui_glfw_metal kpt_platform spdlog::spdlog
          ${KPT_METAL_FRAMEWORK} ${KPT_COCOA_FRAMEWORK}
          ${KPT_QUARTZCORE_FRAMEWORK} ${KPT_FOUNDATION_FRAMEWORK}
          project_options project_warnings)
      target_include_directories(kpt_gui_backend_metal_testable SYSTEM PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)

      add_library(kpt_gui_metal_test_support STATIC
        src/gui/backend/metal/test_support.mm)
      target_include_directories(kpt_gui_metal_test_support PUBLIC src)
      target_compile_options(kpt_gui_metal_test_support PRIVATE
        $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)
      target_link_libraries(kpt_gui_metal_test_support
        PUBLIC kpt_gui_backend_metal_testable
        PRIVATE ${KPT_METAL_FRAMEWORK} project_options project_warnings)

      add_executable(metal_renderer_tests
        tests/test_main.cpp
        tests/metal_renderer_test.mm)
      target_compile_options(metal_renderer_tests PRIVATE
        $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)
      target_link_libraries(metal_renderer_tests PRIVATE
        kpt_gui_model kpt_gui_metal_test_support Catch2
        project_options project_warnings)
      target_compile_definitions(metal_renderer_tests PRIVATE
        CATCH_CONFIG_NO_COUNTER CATCH_CONFIG_NO_POSIX_SIGNALS)
      add_test(NAME metal_renderer_tests COMMAND metal_renderer_tests
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
      kpt_copy_metal_library(metal_renderer_tests)

      add_executable(metal_runtime_smoke_tests
        tests/test_main.cpp
        tests/metal_runtime_smoke_test.mm)
      target_compile_options(metal_runtime_smoke_tests PRIVATE
        $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)
      target_link_libraries(metal_runtime_smoke_tests PRIVATE
        kpt_gui_app kpt_gui_metal_test_support Catch2 glfw
        project_options project_warnings)
      target_compile_definitions(metal_runtime_smoke_tests PRIVATE
        CATCH_CONFIG_NO_COUNTER CATCH_CONFIG_NO_POSIX_SIGNALS)
      add_test(NAME metal_runtime_smoke_tests
        COMMAND metal_runtime_smoke_tests
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
      kpt_copy_metal_library(metal_runtime_smoke_tests)
    endif()

    add_executable(kpt_gui_tests
      tests/test_main.cpp
      tests/gui_test.cpp)
    kpt_enable_windows_utf8_manifest(kpt_gui_tests)
    target_link_libraries(kpt_gui_tests PRIVATE
      kpt_gui_app kpt_viewport_cloud_adapter kpt_jobs Catch2
      project_options project_warnings)
    target_include_directories(kpt_gui_tests SYSTEM PRIVATE
      third_party/imgui-1.92.8-docking
      third_party/glfw-3.4/include)
    target_compile_definitions(kpt_gui_tests PRIVATE CATCH_CONFIG_NO_COUNTER)
    if(UNIX)
      target_compile_definitions(kpt_gui_tests PRIVATE
        CATCH_CONFIG_NO_POSIX_SIGNALS)
    endif()
    add_test(NAME kpt_gui_tests COMMAND kpt_gui_tests
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    find_program(XVFB_RUN_EXECUTABLE xvfb-run)
    if(KPT_ACTIVE_GUI_BACKEND STREQUAL "opengl" AND XVFB_RUN_EXECUTABLE)
      add_test(NAME kpt_gui_smoke
        COMMAND ${XVFB_RUN_EXECUTABLE} -a $<TARGET_FILE:kpt_gui> --smoke-test)
      set_tests_properties(kpt_gui_smoke PROPERTIES
        ENVIRONMENT "LIBGL_ALWAYS_SOFTWARE=1")
    endif()
  endif()
endif()
