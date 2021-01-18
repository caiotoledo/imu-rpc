set(LIBRARY_NAME imuclient)

set(IMUCLIENT_VERSION_MAJOR 0)
set(IMUCLIENT_VERSION_MINOR 0)
set(IMUCLIENT_VERSION_PATCH 0)

set(src_dir "./src/")
set(inc_dir "./include/")
file(GLOB src_files "${src_dir}/*.cpp")
file(GLOB public_hdr_files "${inc_dir}/*.hpp")

add_library(${LIBRARY_NAME} SHARED ${src_files})

# Link DBus-cxx
target_link_libraries(${LIBRARY_NAME} PRIVATE ${DBUSCXX_LIBRARIES})
target_include_directories(${LIBRARY_NAME} PUBLIC ${DBUSCXX_INCLUDE_DIRS})

target_link_libraries(${LIBRARY_NAME} PRIVATE logger)
target_link_libraries(${LIBRARY_NAME} PRIVATE dbustypes)

target_include_directories(${LIBRARY_NAME} PRIVATE ${src_dir})
target_include_directories(${LIBRARY_NAME} PUBLIC ${inc_dir})

# Common Types between server and client
target_link_libraries(${LIBRARY_NAME} PUBLIC dbustypes)

# Set up library version
set_target_properties(${LIBRARY_NAME} PROPERTIES
                        SOVERSION ${IMUCLIENT_VERSION_MAJOR}
                        VERSION ${IMUCLIENT_VERSION_MAJOR}.${IMUCLIENT_VERSION_MINOR}.${IMUCLIENT_VERSION_PATCH})

# Configure install paths
install(TARGETS ${LIBRARY_NAME}
          LIBRARY DESTINATION lib)
install(FILES ${public_hdr_files}
          DESTINATION include/imu-dbus)
