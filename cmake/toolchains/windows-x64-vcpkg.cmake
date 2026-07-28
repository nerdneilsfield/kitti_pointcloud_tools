include_guard(GLOBAL)

if(NOT CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
  message(FATAL_ERROR
    "windows-x64-vcpkg.cmake is a native Windows toolchain entry point")
endif()

if(NOT DEFINED ENV{VCPKG_ROOT} OR "$ENV{VCPKG_ROOT}" STREQUAL "")
  message(FATAL_ERROR "VCPKG_ROOT must name a vcpkg checkout")
endif()

file(TO_CMAKE_PATH "$ENV{VCPKG_ROOT}" KPT_VCPKG_ROOT)
set(KPT_VCPKG_TOOLCHAIN
  "${KPT_VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
if(NOT EXISTS "${KPT_VCPKG_TOOLCHAIN}")
  message(FATAL_ERROR
    "VCPKG_ROOT does not contain scripts/buildsystems/vcpkg.cmake: "
    "${KPT_VCPKG_ROOT}")
endif()

get_filename_component(KPT_CMAKE_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(VCPKG_OVERLAY_TRIPLETS "${KPT_CMAKE_ROOT}/triplets" CACHE STRING
  "Project vcpkg overlay triplets")
set(VCPKG_TARGET_TRIPLET "kpt-x64-windows" CACHE STRING
  "vcpkg target triplet")
include("${KPT_VCPKG_TOOLCHAIN}")
