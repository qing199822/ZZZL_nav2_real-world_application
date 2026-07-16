cmake_minimum_required(VERSION 3.8)
project(pb_rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# 设置你要编译的消息文件路径
set(msg_files
  "msg/GameStatus.msg"
  "msg/RobotStatus.msg"
)

# 生成接口文件 (重点：DEPENDENCIES 必须加上 geometry_msgs)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime geometry_msgs)

ament_package()
