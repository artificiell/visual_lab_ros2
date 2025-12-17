cmake_minimum_required(VERSION 3.8)
project(visual_lab_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define message and service files
set(msg_files
  "msg/Point.msg"
  "msg/Body.msg"
  "msg/BodyTracking.msg"
)
set(srv_files
  "srv/SetScreenBackground.srv"
  "srv/SetScreenImage.srv"
  "srv/UnsetScreenImage.srv"
  "srv/InstructionPrompt.srv"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Build test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
