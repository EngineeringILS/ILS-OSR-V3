cmake_minimum_required(VERSION 3.8)
project(sparkcan_hardware_interface)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

# Sparkcan Inclusion/Checking Logic
# IF BUILD FAILS RUN tools/cpp_prereqs/global_install_sparkcan.sh !!!
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sparkcan REQUIRED)

pluginlib_export_plugin_description_file(hardware_interface sparkcan_hardware_interface.xml)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(
  include
)

# Add sparkcan hardware interface shared library
add_library(${PROJECT_NAME} SHARED
  src/sparkcan_hardware_interface.cpp
  src/sparkcan_unit.cpp
  src/motor_joint.cpp
)

target_link_libraries(${PROJECT_NAME} sparkcan)

# Add library dependencies (removed sparkcan since it is not an ament package)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp  
  pluginlib 
  hardware_interface
)


# Install the shared library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the plugin description xml file
install(FILES
  sparkcan_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
