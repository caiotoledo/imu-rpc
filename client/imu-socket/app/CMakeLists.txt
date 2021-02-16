set(CLIENT_NAME imu-socket)

set(IMUSOCKET_VERSION_MAJOR ${IMURPC_VERSION_MAJOR})
set(IMUSOCKET_VERSION_MINOR ${IMURPC_VERSION_MINOR})
set(IMUSOCKET_VERSION_PATCH ${IMURPC_VERSION_PATCH})

set(client_src_dir "./src/")
file(GLOB client_src_files "${client_src_dir}/*.cpp")

add_executable(${CLIENT_NAME} ${client_src_files})

# Set IMU Command Line Interface Version
target_compile_definitions(${CLIENT_NAME} PRIVATE VERSION_MAJOR=${IMUSOCKET_VERSION_MAJOR})
target_compile_definitions(${CLIENT_NAME} PRIVATE VERSION_MINOR=${IMUSOCKET_VERSION_MINOR})
target_compile_definitions(${CLIENT_NAME} PRIVATE VERSION_PATCH=${IMUSOCKET_VERSION_PATCH})

target_include_directories(${CLIENT_NAME} PRIVATE ${client_src_dir})

# Link with Logger
target_link_libraries(${CLIENT_NAME} PRIVATE logger)

# IMU Client RPC
target_link_libraries(${CLIENT_NAME} PRIVATE imuclient)

# Link with SocketHandler
target_link_libraries(${CLIENT_NAME} PRIVATE sockethandler)
# Link with SafeQueue
target_link_libraries(${CLIENT_NAME} PRIVATE safequeue)

install(TARGETS ${CLIENT_NAME} RUNTIME DESTINATION bin)
