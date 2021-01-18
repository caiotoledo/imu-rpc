set(LIBRARY_NAME dbustypes)
set(inc_dir "./include/")

file(GLOB public_hdr_files "${inc_dir}/*.hpp")

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE "${inc_dir}")

install(FILES ${public_hdr_files}
          DESTINATION include/imu-dbus)
