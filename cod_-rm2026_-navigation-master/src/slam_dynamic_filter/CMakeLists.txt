cmake_minimum_required(VERSION 3.8)
project(slam_dynamic_filter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/sdfctl scripts/slam_dynamic_filter_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/sdfctl DESTINATION bin)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_add_pytest_test(
  test_constants
  test/test_constants.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)
ament_add_pytest_test(
  test_metadata
  test/test_metadata.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)
ament_add_pytest_test(
  test_evidence_scan_launch
  test/test_evidence_scan_launch.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)
ament_add_pytest_test(
  test_preflight
  test/test_preflight.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)
ament_add_pytest_test(
  test_recorder
  test/test_recorder.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)
ament_add_pytest_test(
  test_bag_reader
  test/test_bag_reader.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)
ament_add_pytest_test(
  test_metrics
  test/test_metrics.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)
ament_add_pytest_test(
  test_gate
  test/test_gate.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)
ament_add_pytest_test(
  test_cli
  test/test_cli.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)
ament_add_pytest_test(
  test_session_filter
  test/test_session_filter.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)
ament_add_pytest_test(
  test_mapping_filter_launch
  test/test_mapping_filter_launch.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)
ament_add_pytest_test(
  test_online_node
  test/test_online_node.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)
ament_add_pytest_test(
  test_production_mapping_opt_in
  test/test_production_mapping_opt_in.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)

ament_package()
