cmake_minimum_required(VERSION 3.8)
project(enpm809e_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/WeatherStation.msg"
)

set(srv_files
  "srv/AddTwoInts.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}

  DEPENDENCIES
  builtin_interfaces
  example_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
