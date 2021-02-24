set(LIBRARY_NAME imuserver)

set(src_dir "./src/")
set(inc_dir "./include/")
file(GLOB src_files "${src_dir}/*.cpp")

add_library(${LIBRARY_NAME} STATIC ${src_files})

target_link_libraries(${LIBRARY_NAME} PRIVATE logger)

target_link_libraries(${LIBRARY_NAME} PUBLIC rpcserver)
target_link_libraries(${LIBRARY_NAME} PUBLIC imuabstraction)
target_link_libraries(${LIBRARY_NAME} PUBLIC imumath)

target_include_directories(${LIBRARY_NAME} PRIVATE ${src_dir})
target_include_directories(${LIBRARY_NAME} PUBLIC ${inc_dir})
