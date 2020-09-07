cmake_minimum_required(VERSION 3.0)

project(toml)

# Prepare "Catch2" library for other executables
add_library(toml INTERFACE)
add_library(toml::toml ALIAS toml)
target_include_directories(toml INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})