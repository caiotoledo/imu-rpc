set(LIBRARY_NAME timer)

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE "./include/")

target_link_libraries(${LIBRARY_NAME} INTERFACE singletoninstancer)
