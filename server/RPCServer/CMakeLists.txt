set(LIBRARY_NAME rpcserver)

set(src_dir "./src/")
set(inc_dir "./include/")
file(GLOB src_files "${src_dir}/*.cpp")

add_library(${LIBRARY_NAME} STATIC ${src_files})

# Link DBus-cxx
target_link_libraries(${LIBRARY_NAME} PRIVATE ${DBUSCXX_LIBRARIES})
target_include_directories(${LIBRARY_NAME} PUBLIC ${DBUSCXX_INCLUDE_DIRS})

target_link_libraries(${LIBRARY_NAME} PRIVATE logger)
target_link_libraries(${LIBRARY_NAME} PRIVATE dbustypes)

target_include_directories(${LIBRARY_NAME} PRIVATE ${src_dir})
target_include_directories(${LIBRARY_NAME} PUBLIC ${inc_dir})
