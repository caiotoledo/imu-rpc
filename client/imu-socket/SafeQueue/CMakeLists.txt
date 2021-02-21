cmake_minimum_required (VERSION 3.8)
set(LIBRARY_NAME safequeue)

set(inc_dir "./include/")

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE ${inc_dir})
