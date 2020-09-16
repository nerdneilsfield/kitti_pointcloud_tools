cmake_minimum_required(VERSION 3.10)

set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})

# Set the project name to your project name, my project isn't very descriptive
project(KITTI_POINTCLOUD_TOOLS CXX C)
include(cmake/StandardProjectSettings.cmake)

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

if(UNIX AND NOT APPLE)
# standard compiler warnings
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)
endif()

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
option(ENABLE_TESTING "Enable Test Builds" ON)
option(ENABLE_TOOLS "Enable the tools" ON)

#find_package(spdlog REQUIRED)
#find_package(yaml-cpp CONFIG REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    message("OpenMP founded!")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}" "${OpenMP_CXX_FLAGS}" )
endif()
#find_package(sioclient REQUIRED)

add_subdirectory(third_party)


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


find_package(PCL REQUIRED COMPONENT common io)

if(${PCL_FOUND})
  message(STATUS "pcd founded!")
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif()

add_library(kitti_binary src/kitti_binary.cc)
target_link_libraries(kitti_binary PUBLIC ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} spdlog::spdlog project_options)


add_executable(kitti_binary_converter src/kitti_binary_converter.cc)
target_link_libraries(kitti_binary_converter PUBLIC kitti_binary popl::popl project_options)
