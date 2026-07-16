cmake_minimum_required(VERSION 3.5)
project(serial_def_sdk)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(def_msg REQUIRED)
find_package(vision_msg REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pb_rm_interfaces REQUIRED)

#find_package(serial REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(./include/serial)
include_directories(./uart)
aux_source_directory(./src DIR_SRCS)
aux_source_directory(. L_SRCS)
aux_source_directory(./uart UART_SRCS)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(uart 
  ./src/base_controller.cpp
  #./uart/main.cpp
  ./uart/SerialDriver.cpp
  ./uart/uart_interface.cpp
  #src/ArraryList.c
  ./uart/DataType.c
  #src/Crc.c
  ./uart/crc8.c
  ./uart/crc16.c
  ./uart/SeaskyProtocol.cpp
  ./uart/seasky_communicator.cpp
  #src/main.cpp
)
add_executable(talker src/publisher.cpp)

ament_target_dependencies(uart
        rclcpp
        std_msgs
        def_msg
        vision_msg
        pb_rm_interfaces
        geometry_msgs
        nav_msgs
        tf2_ros
        tf2
        sensor_msgs
        example_interfaces
        std_srvs 
      )
#链接动态库
target_link_libraries(uart
        libpthread.so
        libXtst.so
        libX11.so
        )

 install(TARGETS
        uart
        talker
        DESTINATION lib/${PROJECT_NAME}
        )

  install( DIRECTORY
      launch
  DESTINATION share/${PROJECT_NAME}
  )

  ament_target_dependencies(
  talker
  "rclcpp"
  "std_msgs"
  "def_msg"
  "geometry_msgs"
  
)
ament_package()
