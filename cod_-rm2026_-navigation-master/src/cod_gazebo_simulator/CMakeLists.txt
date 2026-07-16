cmake_minimum_required(VERSION 3.8)
project(cod_gazebo_simulator)

find_package(ament_cmake REQUIRED)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_model_path.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_resource_path.dsv.in"
)

install(DIRECTORY
  launch
  config
  resource
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  launch/_sim_common.py
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
