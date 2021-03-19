cmake_minimum_required(VERSION 3.0)

project(rapidcsv)

# Prepare "Catch2" library for other executables
add_library(rapidcsv INTERFACE)
add_library(rapidcsv::csv ALIAS rapidcsv)
target_include_directories(rapidcsv INTERFACE ${CMAKE_CURRENT_SOURCE})
#target_precompile_headers(rapidcsv INTERFACE rapidcsv.h)