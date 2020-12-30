set(CLIENT_NAME imu-client)

set(client_src_dir "./src/")
file(GLOB client_src_files "${client_src_dir}/*.cpp")

add_executable(${CLIENT_NAME} ${client_src_files})

target_include_directories(${CLIENT_NAME} PRIVATE ${client_src_dir})

# Link with Logger
target_link_libraries(${CLIENT_NAME} PRIVATE logger)

target_link_libraries(${CLIENT_NAME} PRIVATE imuclient)

install(TARGETS imu-client RUNTIME DESTINATION bin)
