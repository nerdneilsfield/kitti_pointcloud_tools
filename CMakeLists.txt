cmake_minimum_required(VERSION 3.21)

#set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})

# Set the project name to your project name, my project isn't very descriptive
project(KITTI_POINTCLOUD_TOOLS CXX C)
include(cmake/StandardProjectSettings.cmake)
include(cmake/KptTargetAssertions.cmake)

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_20)
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
  # PCL 1.15 accepts only narrow filename strings and eventually calls the
  # CRT _open API. Windows 10 1903+ honors this embedded process contract,
  # making those narrow filenames UTF-8 rather than the legacy ACP.
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

option(BUILD_SHARED_LIBS "Enable compilation of shared libraries" OFF)
option(KPT_BUILD_TESTS "Enable kpt test builds" ON)
option(KPT_BUILD_GUI "Enable the Dear ImGui workbench" OFF)
option(KPT_BUILD_PCL_VIEWERS "Build PCLVisualizer CLI tools" ON)
option(ENABLE_TOOLS "Enable the tools" ON)
set(KPT_GUI_BACKEND "auto" CACHE STRING "GUI renderer: auto, opengl, metal")
set_property(CACHE KPT_GUI_BACKEND PROPERTY STRINGS auto opengl metal)

# Validate the public cache value even for headless configurations. Platform
# resolution is intentionally deferred until a GUI runtime is requested.
kpt_validate_gui_backend(
  REQUESTED "${KPT_GUI_BACKEND}"
  OUT_VAR KPT_VALIDATED_GUI_BACKEND)

#find_package(spdlog REQUIRED)
#find_package(yaml-cpp CONFIG REQUIRED)

#find_package(sioclient REQUIRED)

add_subdirectory(third_party)

if(WIN32)
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

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
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


set(KPT_PCL_COMPONENTS common io)
if(KPT_BUILD_PCL_VIEWERS)
  list(APPEND KPT_PCL_COMPONENTS visualization)
endif()
find_package(PCL REQUIRED COMPONENTS ${KPT_PCL_COMPONENTS})
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

foreach(KPT_REQUIRED_PCL_TARGET IN ITEMS pcl_common pcl_io)
  if(NOT TARGET ${KPT_REQUIRED_PCL_TARGET})
    message(FATAL_ERROR
      "PCL package did not define required imported target "
      "${KPT_REQUIRED_PCL_TARGET}")
  endif()
endforeach()

add_library(kpt_core STATIC
  src/kpt/io/io.cpp
  src/kpt/label/label.cpp
  src/kpt/workflow/workflow.cpp
)
target_link_libraries(kpt_core
  PUBLIC
    kpt_common
    pcl_common
    spdlog::spdlog
    eigen3::eigen
    project_options
    project_warnings
  PRIVATE
    pcl_io
)
# Mark vendored spdlog include as SYSTEM to avoid -Werror on its headers
target_include_directories(kpt_core SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include
)
target_include_directories(kpt_core PUBLIC src)

# Temporary source-compatible name while in-tree consumers move to precise
# targets. This aliases the core only; it does not silently enable renderers.
add_library(kpt ALIAS kpt_core)

add_library(kpt_render STATIC
  src/kpt/render/render.cpp
)
target_link_libraries(kpt_render
  PUBLIC
    kpt_core
    opencv_core
    opencv_imgcodecs
)
target_include_directories(kpt_render PUBLIC src)

if(KPT_BUILD_PCL_VIEWERS)
  if(NOT TARGET pcl_visualization)
    message(FATAL_ERROR
      "KPT_BUILD_PCL_VIEWERS requires PCL's pcl_visualization target")
  endif()

  add_library(kpt_pcl_viewer STATIC
    src/kpt/viewer/viewer.cpp
    src/kpt/player/player.cpp
  )
  target_link_libraries(kpt_pcl_viewer
    PUBLIC
      kpt_core
      kpt_render
      pcl_visualization
  )
  target_include_directories(kpt_pcl_viewer PUBLIC src)
  target_include_directories(kpt_pcl_viewer SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
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

  add_library(imgui_file_dialog STATIC
    third_party/ImGuiFileDialog-0.6.8/ImGuiFileDialog.cpp)
  target_compile_features(imgui_file_dialog PUBLIC cxx_std_20)
  target_compile_definitions(imgui_file_dialog PRIVATE USE_STD_FILESYSTEM)
  target_include_directories(imgui_file_dialog PUBLIC
    third_party/ImGuiFileDialog-0.6.8)
  target_link_libraries(imgui_file_dialog PUBLIC imgui_core)

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

  add_library(kpt_viewport_pcl_adapter STATIC
    src/gui/viewport/pcl_adapter.cpp)
  target_include_directories(kpt_viewport_pcl_adapter PUBLIC src)
  target_link_libraries(kpt_viewport_pcl_adapter
    PUBLIC kpt_gui_contracts kpt_core
    PRIVATE project_options project_warnings)

  add_library(kpt_jobs STATIC
    src/gui/jobs/job_system.cpp
    src/gui/jobs/ui_events.cpp)
  target_include_directories(kpt_jobs PUBLIC src)
  target_link_libraries(kpt_jobs PRIVATE project_options project_warnings)
endif()

# Optional Dear ImGui workbench. All three GUI dependencies are vendored so
# configure/build remains fully offline.
if(KPT_BUILD_GUI)
  kpt_resolve_gui_backend(
    REQUESTED "${KPT_VALIDATED_GUI_BACKEND}"
    SYSTEM "${CMAKE_SYSTEM_NAME}"
    OUT_VAR KPT_ACTIVE_GUI_BACKEND)

  if(KPT_ACTIVE_GUI_BACKEND STREQUAL "metal")
    message(FATAL_ERROR
      "KPT_GUI_BACKEND=metal is selected for ${CMAKE_SYSTEM_NAME}, but the "
      "Metal runtime is not implemented yet")
  endif()

  if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR
      "KPT_GUI_BACKEND=${KPT_ACTIVE_GUI_BACKEND} resolves successfully for "
      "${CMAKE_SYSTEM_NAME}, but the current OpenGL runtime supports "
      "Linux/X11 only")
  endif()

  find_package(OpenGL REQUIRED)

  # KPT owns this loader. Dear ImGui keeps its symbol-prefixed internal loader;
  # do not couple either implementation through IMGUI_IMPL_OPENGL_LOADER_CUSTOM.
  add_library(kpt_glad STATIC third_party/glad/src/gl.c)
  target_include_directories(kpt_glad SYSTEM PUBLIC
    third_party/glad/include)
  set_target_properties(kpt_glad PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO)

  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
  set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
  add_subdirectory(third_party/glfw-3.4 EXCLUDE_FROM_ALL)

  add_library(imgui_glfw_opengl STATIC
    third_party/imgui-1.92.8-docking/backends/imgui_impl_glfw.cpp
    third_party/imgui-1.92.8-docking/backends/imgui_impl_opengl3.cpp)
  target_include_directories(imgui_glfw_opengl PUBLIC
    third_party/imgui-1.92.8-docking)
  target_link_libraries(imgui_glfw_opengl PUBLIC
    imgui_core glfw OpenGL::GL)

  add_library(kpt_gui_backend_opengl STATIC
    src/gui/backend/opengl/point_renderer.cpp)
  target_include_directories(kpt_gui_backend_opengl PUBLIC src)
  target_link_libraries(kpt_gui_backend_opengl
    PUBLIC kpt_gui_contracts
    PRIVATE kpt_glad glfw project_options project_warnings)

  add_library(kpt_gui_app STATIC
    src/gui/app.cpp
    src/gui/dialog_paths.cpp)
  target_include_directories(kpt_gui_app PUBLIC src)
  target_include_directories(kpt_gui_app SYSTEM PRIVATE
    third_party/imgui-1.92.8-docking
    third_party/ImGuiFileDialog-0.6.8
    third_party/spdlog/include)
  target_link_libraries(kpt_gui_app
    PUBLIC kpt_gui_contracts kpt_gui_model kpt_jobs kpt_core
    PRIVATE kpt_common kpt_render kpt_platform
      kpt_viewport_pcl_adapter imgui_file_dialog
      project_options project_warnings)

  kpt_assert_exactly_one_target(
    ACTIVE "kpt_gui_backend_${KPT_ACTIVE_GUI_BACKEND}"
    CANDIDATES kpt_gui_backend_opengl kpt_gui_backend_metal)

  add_executable(pc_gui src/gui/pc_gui.cc)
  kpt_enable_windows_utf8_manifest(pc_gui)
  target_link_libraries(pc_gui PRIVATE
    kpt_gui_app imgui_glfw_opengl
    kpt_gui_backend_opengl OpenGL::GL
    project_options project_warnings)
  target_include_directories(pc_gui SYSTEM PRIVATE
    third_party/imgui-1.92.8-docking
    third_party/ImGuiFileDialog-0.6.8
    third_party/glfw-3.4/include
    third_party/spdlog/include)
endif()

# CLI tools
foreach(tool pc_convert pc_batch_convert)
  add_executable(${tool} src/cli/${tool}.cc)
  kpt_enable_windows_utf8_manifest(${tool})
  target_link_libraries(${tool} PUBLIC
    kpt_core popl::popl project_options project_warnings)
  # Vendored spdlog exposes its include dir as PUBLIC non-system via the spdlog
  # target; promote to SYSTEM here so -Werror=shadow does not fire on its
  # bundled fmt headers (same workaround kpt itself uses above).
  target_include_directories(${tool} SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
endforeach()

add_executable(pc_render src/cli/pc_render.cc)
kpt_enable_windows_utf8_manifest(pc_render)
target_link_libraries(pc_render PUBLIC
  kpt_render popl::popl project_options project_warnings)
target_include_directories(pc_render SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)

if(KPT_BUILD_PCL_VIEWERS)
  foreach(tool pc_viewer pc_player)
    add_executable(${tool} src/cli/${tool}.cc)
    kpt_enable_windows_utf8_manifest(${tool})
    target_link_libraries(${tool} PUBLIC
      kpt_pcl_viewer popl::popl project_options project_warnings)
    target_include_directories(${tool} SYSTEM PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
  endforeach()
endif()

# Tests
if(KPT_BUILD_TESTS)
  add_executable(kpt_tests tests/test_main.cpp tests/io_test.cpp
                 tests/label_test.cpp tests/render_test.cpp
                 tests/workflow_test.cpp tests/utf8_path_test.cpp)
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
    kpt_core kpt_render Catch2 project_options project_warnings)
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
    tests/viewport_model_test.cpp)
  target_link_libraries(viewport_model_tests PRIVATE
    kpt_gui_model kpt_viewport_pcl_adapter Catch2
    project_options project_warnings)
  target_compile_definitions(viewport_model_tests PRIVATE
    CATCH_CONFIG_NO_COUNTER)
  if(UNIX)
    target_compile_definitions(viewport_model_tests PRIVATE
      CATCH_CONFIG_NO_POSIX_SIGNALS)
  endif()
  add_test(NAME viewport_model_tests COMMAND viewport_model_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  if(KPT_BUILD_GUI)
    add_library(kpt_gui_test_support STATIC
      src/gui/backend/opengl/test_access.cpp)
    target_include_directories(kpt_gui_test_support PUBLIC src)
    target_link_libraries(kpt_gui_test_support
      PUBLIC kpt_gui_backend_opengl
      PRIVATE kpt_glad project_options project_warnings)

    add_executable(opengl_renderer_tests
      tests/test_main.cpp
      tests/opengl_renderer_test.cpp)
    target_link_libraries(opengl_renderer_tests PRIVATE
      kpt_gui_model kpt_gui_test_support kpt_glad Catch2 glfw
      project_options project_warnings)
    target_compile_definitions(opengl_renderer_tests PRIVATE
      CATCH_CONFIG_NO_COUNTER)
    if(UNIX)
      target_compile_definitions(opengl_renderer_tests PRIVATE
        CATCH_CONFIG_NO_POSIX_SIGNALS)
    endif()
    add_test(NAME opengl_renderer_tests COMMAND opengl_renderer_tests
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

    add_executable(kpt_gui_tests
      tests/test_main.cpp
      tests/gui_test.cpp)
    kpt_enable_windows_utf8_manifest(kpt_gui_tests)
    target_link_libraries(kpt_gui_tests PRIVATE
      kpt_gui_app kpt_viewport_pcl_adapter kpt_jobs Catch2
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
    if(XVFB_RUN_EXECUTABLE)
      add_test(NAME pc_gui_smoke
        COMMAND ${XVFB_RUN_EXECUTABLE} -a $<TARGET_FILE:pc_gui> --smoke-test)
      set_tests_properties(pc_gui_smoke PROPERTIES
        ENVIRONMENT "LIBGL_ALWAYS_SOFTWARE=1")
    endif()
  endif()
endif()
