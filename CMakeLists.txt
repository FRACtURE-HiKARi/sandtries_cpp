cmake_minimum_required(VERSION 3.8)
project(Sandtries)
aux_source_directory(./src SOURCE_DIR)
include_directories(./inc)
include_directories(D:\\MSVS\\MicrosoftVisualStudio\\VC\\Auxiliary\\VS\\include)

add_executable(main ${SOURCE_DIR})
