set(SERVER_NAME imu-daemon)

set(IMUDAEMON_VERSION_MAJOR ${IMURPC_VERSION_MAJOR})
set(IMUDAEMON_VERSION_MINOR ${IMURPC_VERSION_MINOR})
set(IMUDAEMON_VERSION_PATCH ${IMURPC_VERSION_PATCH})

set(server_src_dir "./src/")
file(GLOB server_src_files "${server_src_dir}/*.cpp")

add_executable(${SERVER_NAME} ${server_src_files})

# Set Daemon Version
target_compile_definitions(${SERVER_NAME} PRIVATE VERSION_MAJOR=${IMUDAEMON_VERSION_MAJOR})
target_compile_definitions(${SERVER_NAME} PRIVATE VERSION_MINOR=${IMUDAEMON_VERSION_MINOR})
target_compile_definitions(${SERVER_NAME} PRIVATE VERSION_PATCH=${IMUDAEMON_VERSION_PATCH})

# Link with Logger
target_link_libraries(${SERVER_NAME} PRIVATE logger)

target_link_libraries(${SERVER_NAME} PRIVATE imuserver)

install(TARGETS ${SERVER_NAME} RUNTIME DESTINATION bin)
