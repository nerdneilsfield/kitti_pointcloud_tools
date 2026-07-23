cmake_minimum_required(VERSION 3.10)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

#set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})

# Set the project name to your project name, my project isn't very descriptive
project(KITTI_POINTCLOUD_TOOLS CXX C)
include(cmake/StandardProjectSettings.cmake)

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_20)

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


find_package(PCL REQUIRED)

if(${PCL_FOUND})
  message(STATUS "pcd founded!")
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif()

find_package(OpenCV REQUIRED)
if(${OpenCV_FOUND})
  message(STATUS "OpenCV founded!")
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIBRARY_DIRS})
  add_definitions(${OpenCV_DEFINITIONS})
endif()

include_directories(src)

# kpt static library
add_library(kpt STATIC
  src/kpt/io/io.cpp
  src/kpt/viewer/viewer.cpp
  src/kpt/render/render.cpp
  src/kpt/label/label.cpp
  src/kpt/player/player.cpp
)
target_link_libraries(kpt PUBLIC
  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} spdlog::spdlog eigen3::eigen project_options project_warnings
)
# Mark vendored spdlog include as SYSTEM to avoid -Werror on its headers
target_include_directories(kpt SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include
)
target_include_directories(kpt PUBLIC src)

# CLI tools
foreach(tool pc_convert pc_batch_convert pc_viewer pc_player pc_render)
  add_executable(${tool} src/cli/${tool}.cc)
  target_link_libraries(${tool} PUBLIC kpt popl::popl project_options project_warnings)
  # Vendored spdlog exposes its include dir as PUBLIC non-system via the spdlog
  # target; promote to SYSTEM here so -Werror=shadow does not fire on its
  # bundled fmt headers (same workaround kpt itself uses above).
  target_include_directories(${tool} SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
endforeach()

# Tests
if(ENABLE_TESTING)
  add_executable(kpt_tests tests/test_main.cpp tests/io_test.cpp
                 tests/label_test.cpp tests/render_test.cpp)
  target_link_libraries(kpt_tests PRIVATE kpt Catch2 project_options project_warnings)
  # Vendored Catch2 v2 has a glibc>=2.34 MINSIGSTKSZ constexpr bug; disable POSIX signals.
  target_compile_definitions(kpt_tests PRIVATE CATCH_CONFIG_NO_POSIX_SIGNALS)
  enable_testing()
  add_test(NAME kpt_tests COMMAND kpt_tests WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()