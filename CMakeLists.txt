cmake_minimum_required(VERSION 3.16)
project(pypilot_servo_protocol LANGUAGES CXX)

option(PYPILOT_SERVO_PROTOCOL_BUILD_TESTS "Build tests" ON)
option(PYPILOT_SERVO_PROTOCOL_BUILD_EXAMPLES "Build Linux examples" ON)

add_library(pypilot_servo_protocol INTERFACE)
target_include_directories(pypilot_servo_protocol INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(pypilot_servo_protocol INTERFACE cxx_std_11)

if(PYPILOT_SERVO_PROTOCOL_BUILD_EXAMPLES AND UNIX AND NOT APPLE)
  add_executable(pypilot-servo-monitor examples/linux/monitor.cpp)
  target_link_libraries(pypilot-servo-monitor PRIVATE pypilot_servo_protocol)
  target_compile_definitions(pypilot-servo-monitor PRIVATE PYPILOT_SERVO_PROTOCOL_ENABLE_LINUX_SERIAL)

  add_executable(pypilot-servo-send-command examples/linux/send_command.cpp)
  target_link_libraries(pypilot-servo-send-command PRIVATE pypilot_servo_protocol)
  target_compile_definitions(pypilot-servo-send-command PRIVATE PYPILOT_SERVO_PROTOCOL_ENABLE_LINUX_SERIAL)
endif()

if(PYPILOT_SERVO_PROTOCOL_BUILD_TESTS)
  enable_testing()
  add_executable(test_packet tests/test_packet.cpp)
  target_link_libraries(test_packet PRIVATE pypilot_servo_protocol)
  add_test(NAME test_packet COMMAND test_packet)
endif()
