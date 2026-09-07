cmake_minimum_required(VERSION 3.14)
project(alnslike LANGUAGES CXX)

add_library(alnslike INTERFACE)
add_library(alnslike::alnslike ALIAS alnslike) 
target_include_directories(alnslike INTERFACE include)
target_compile_features(alnslike INTERFACE cxx_std_20)

option(ALNSLIKE_BUILD_EXAMPLES "Build examples" OFF)
if(ALNSLIKE_BUILD_EXAMPLES)
    add_subdirectory(examples/minimal)
endif()