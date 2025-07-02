cmake_minimum_required(VERSION 3.5)
project(serial_vendor)

add_library(serial_vendor
  src/serial.cc
  src/impl/unix.cc
  src/impl/win.cc
  src/impl/list_ports/list_ports_linux.cc
  src/impl/list_ports/list_ports_osx.cc
  src/impl/list_ports/list_ports_win.cc
)

target_include_directories(serial_vendor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(serial_vendor PUBLIC SERIAL_PORT_V8STDINT_H)

install(TARGETS serial_vendor DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
