include_guard(GLOBAL)

if(NOT CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR
    "linux-vcpkg.cmake is a native Linux toolchain entry point")
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

set(VCPKG_TARGET_TRIPLET "x64-linux" CACHE STRING
  "vcpkg target triplet")
include("${KPT_VCPKG_TOOLCHAIN}")
