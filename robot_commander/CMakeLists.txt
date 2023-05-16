cmake_minimum_required(VERSION 3.8)
project(robot_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(competitor_interfaces REQUIRED)

# Location of header files
include_directories(include)

# Dependencies for the floor robot commander
set(FLOOR_ROBOT_DEPENDENCIES
  rclcpp 
  moveit_ros_planning_interface
  ariac_msgs
  competitor_interfaces
  shape_msgs
)

foreach(Dependency IN ITEMS ${FLOOR_ROBOT_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# Install the config directory to the package share directory
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(floor_robot_commander src/floor_robot_commander.cpp)
ament_target_dependencies(floor_robot_commander ${FLOOR_ROBOT_DEPENDENCIES})

install(TARGETS
  floor_robot_commander
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/floor_robot_main.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
