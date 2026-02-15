cmake_minimum_required(VERSION 3.5)
project(imu_serial_reader)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

include_directories(include)

add_executable(imu_serial_receiver_node
  src/main.cpp
  src/ImuSerialReceiver.cpp
  src/filter.cpp
  src/stateMachine.cpp
  src/slider.cpp
)

ament_target_dependencies(imu_serial_receiver_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs)

target_link_libraries(imu_serial_receiver_node Boost::system pthread)
target_link_libraries(imu_serial_receiver_node
  Qt5::Widgets
  ${rclcpp_LIBRARIES}
)
install(TARGETS
  imu_serial_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
