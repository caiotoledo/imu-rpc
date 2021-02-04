set(LIBRARY_NAME imumath)

set(src_dir "./src/")
set(inc_dir "./include/")
file(GLOB src_files "${src_dir}/*.cpp")

add_library(${LIBRARY_NAME} STATIC ${src_files})

target_link_libraries(${LIBRARY_NAME} PRIVATE imuabstraction)

target_link_libraries(${LIBRARY_NAME} PRIVATE logger)
target_link_libraries(${LIBRARY_NAME} PRIVATE pthread)

# Common Types between server and client
target_link_libraries(${LIBRARY_NAME} PUBLIC dbustypes)

target_include_directories(${LIBRARY_NAME} PRIVATE ${src_dir})
target_include_directories(${LIBRARY_NAME} PUBLIC ${inc_dir})

add_subdirectory(tests)
