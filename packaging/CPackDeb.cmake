if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "CPackDeb.cmake is Linux-only")
endif()

set(KPT_PACKAGE_DISTRO "" CACHE STRING
  "Ubuntu package target: ubuntu22.04, ubuntu24.04, or ubuntu26.04")
if(NOT KPT_PACKAGE_DISTRO MATCHES "^ubuntu(22\\.04|24\\.04|26\\.04)$")
  message(FATAL_ERROR
    "KPT_PACKAGE_DISTRO must be ubuntu22.04, ubuntu24.04, or ubuntu26.04")
endif()

execute_process(
  COMMAND dpkg --print-architecture
  OUTPUT_VARIABLE KPT_DEBIAN_ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY)
if(NOT KPT_DEBIAN_ARCHITECTURE STREQUAL "amd64")
  message(FATAL_ERROR "Release DEB packaging currently supports amd64 only")
endif()

set(CPACK_GENERATOR DEB)
set(CPACK_PACKAGE_NAME "kitti-pointcloud-tools")
set(CPACK_PACKAGE_VENDOR "DengQi")
set(CPACK_PACKAGE_CONTACT "DengQi <nerdneilsfield@users.noreply.github.com>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
  "KITTI point-cloud viewer, player, converter, and renderer")
set(CPACK_PACKAGE_HOMEPAGE_URL
  "https://github.com/nerdneilsfield/kitti_pointcloud_tools")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_FILE_NAME
  "kitti-pointcloud-tools_${PROJECT_VERSION}-1~${KPT_PACKAGE_DISTRO}_amd64")
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_STRIP_FILES TRUE)
set(CPACK_COMPONENTS_ALL Runtime)

set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
set(CPACK_DEBIAN_PACKAGE_SECTION "graphics")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_RELEASE "1~${KPT_PACKAGE_DISTRO}")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
# GLFW resolves these libraries dynamically, so dpkg-shlibdeps cannot discover
# them from DT_NEEDED entries.
set(CPACK_DEBIAN_PACKAGE_DEPENDS
  "libgl1, libx11-6, libxrandr2, libxinerama1, libxcursor1, libxi6")

include(CPack)
