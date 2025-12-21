cmake_minimum_required(VERSION 3.8)
project(ros2_serial_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(BUILD_ROS2 ON)
get_filename_component(COMMON_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../common" ABSOLUTE)
add_subdirectory(${COMMON_LIB_PATH} ${CMAKE_CURRENT_BINARY_DIR}/common_build)

add_library(ros2_serial_driver src/ros2_serial_driver.cpp)
target_compile_features(ros2_serial_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ros2_serial_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ros2_serial_driver PRIVATE "ROS2_SERIAL_DRIVER_BUILDING_LIBRARY")

target_link_libraries(${PROJECT_NAME} 
  PRIVATE
  common_lib
)



install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ros2_serial_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ros2_serial_driver
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
