cmake_minimum_required(VERSION 3.5)
project(ros2_ddboat)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
add_subdirectory(third_party/serial)

add_executable(gps_node src/gps_node.cpp)
target_link_libraries(gps_node serial_vendor)
ament_target_dependencies(gps_node rclcpp sensor_msgs)

add_executable(arduino_node src/arduino_node.cpp)
target_link_libraries(arduino_node serial_vendor)
ament_target_dependencies(arduino_node rclcpp geometry_msgs)

add_executable(encoders_node src/encoders_node.cpp)
target_link_libraries(encoders_node serial_vendor)
ament_target_dependencies(encoders_node rclcpp std_msgs)

add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node rclcpp sensor_msgs)

add_executable(temp_node src/temp_node.cpp)
ament_target_dependencies(temp_node rclcpp sensor_msgs)

add_executable(radio_node src/radio_node.cpp)
target_link_libraries(radio_node serial_vendor)
ament_target_dependencies(radio_node rclcpp std_msgs)

install(TARGETS gps_node arduino_node encoders_node imu_node temp_node radio_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
