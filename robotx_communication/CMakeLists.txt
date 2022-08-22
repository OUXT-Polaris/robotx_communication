cmake_minimum_required(VERSION 3.8)
project(robotx_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(technical_director_network_bridge_component SHARED
  src/technical_director_network_bridge_component.cpp
)
target_compile_definitions(technical_director_network_bridge_component
  PRIVATE "ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_BUILDING_DLL")
rclcpp_components_register_nodes(technical_director_network_bridge_component
  "robotx_communication::TechnicalDirectorNetworkBridgeComponent")

ament_auto_add_executable(technical_director_network_bridge_node
  src/technical_director_network_bridge_node.cpp
)
target_link_libraries(technical_director_network_bridge_node
  technical_director_network_bridge_component
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
