set(LIBRARY_NAME dbustypes)

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE "./include/")
