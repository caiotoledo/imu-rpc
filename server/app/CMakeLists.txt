set(SERVER_NAME imu-daemon)

set(server_src_dir "./src/")
file(GLOB server_src_files "${server_src_dir}/*.cpp")

add_executable(${SERVER_NAME} ${server_src_files})

# Link with Logger
target_link_libraries(${SERVER_NAME} PRIVATE logger)

target_link_libraries(${SERVER_NAME} PRIVATE imuserver)

install(TARGETS imu-daemon RUNTIME DESTINATION bin)
