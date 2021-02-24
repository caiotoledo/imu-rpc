set(CLIENT_NAME imu-cli)

set(IMUCLIENT_VERSION_MAJOR ${IMURPC_VERSION_MAJOR})
set(IMUCLIENT_VERSION_MINOR ${IMURPC_VERSION_MINOR})
set(IMUCLIENT_VERSION_PATCH ${IMURPC_VERSION_PATCH})

set(client_src_dir "./src/")
file(GLOB client_src_files "${client_src_dir}/*.cpp")

add_executable(${CLIENT_NAME} ${client_src_files})

# Set IMU Command Line Interface Version
target_compile_definitions(${CLIENT_NAME} PRIVATE VERSION_MAJOR=${IMUCLIENT_VERSION_MAJOR})
target_compile_definitions(${CLIENT_NAME} PRIVATE VERSION_MINOR=${IMUCLIENT_VERSION_MINOR})
target_compile_definitions(${CLIENT_NAME} PRIVATE VERSION_PATCH=${IMUCLIENT_VERSION_PATCH})

target_include_directories(${CLIENT_NAME} PRIVATE ${client_src_dir})

# Link with Logger
target_link_libraries(${CLIENT_NAME} PRIVATE logger)

target_link_libraries(${CLIENT_NAME} PRIVATE imuclient)

install(TARGETS ${CLIENT_NAME} RUNTIME DESTINATION bin)
