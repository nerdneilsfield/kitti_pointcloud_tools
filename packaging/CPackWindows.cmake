if(NOT WIN32)
  message(FATAL_ERROR "CPackWindows.cmake is Windows-only")
endif()

include(GNUInstallDirs)
set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION "${CMAKE_INSTALL_BINDIR}")
include(InstallRequiredSystemLibraries)
if(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS)
  install(PROGRAMS ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS}
    DESTINATION "${CMAKE_INSTALL_BINDIR}"
    COMPONENT Runtime)
endif()

set(CPACK_GENERATOR ZIP)
set(CPACK_PACKAGE_NAME "kitti-pointcloud-tools")
set(CPACK_PACKAGE_VENDOR "DengQi")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
  "KITTI point-cloud viewer, player, converter, and renderer")
set(CPACK_PACKAGE_HOMEPAGE_URL
  "https://github.com/nerdneilsfield/kitti_pointcloud_tools")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_FILE_NAME
  "kitti-pointcloud-tools-${PROJECT_VERSION}-windows-x64")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
set(CPACK_COMPONENTS_ALL Runtime)
set(CPACK_STRIP_FILES OFF)
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

include(CPack)
