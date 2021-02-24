cmake_minimum_required (VERSION 3.8)
set(LIBRARY_NAME logger)

set(inc_dir "./include/")
set(src_dir "./src/")
file(GLOB src_files "${src_dir}/*.cpp")

add_library(${LIBRARY_NAME} STATIC ${src_files})
target_include_directories(${LIBRARY_NAME} PRIVATE ${src_dir})
target_include_directories(${LIBRARY_NAME} PUBLIC ${inc_dir})

target_link_libraries(${LIBRARY_NAME} PUBLIC singletoninstancer)

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    # Enable max level logger
    target_compile_definitions(${LIBRARY_NAME} PUBLIC -DLEVEL_DEBUG)
else()
    # Show only warnings and errors in Release mode
    target_compile_definitions(${LIBRARY_NAME} PUBLIC -DLEVEL_WARNING)
endif()
