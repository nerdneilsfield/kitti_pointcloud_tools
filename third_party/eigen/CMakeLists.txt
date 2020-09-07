# Copyright 2019 Smart And Reconfigurable System Lab of ShanghaiTech University

cmake_minimum_required(VERSION 3.0)

project(eigen3)

# Prepare "Catch2" library for other executables
add_library(eigen3 INTERFACE)
add_library(eigen3::eigen ALIAS eigen3)
target_include_directories(eigen3 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})