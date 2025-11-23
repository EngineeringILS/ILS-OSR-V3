# Set the minimum CMake version
# 3.16 is a good, modern choice compatible with ROS2 Humble and ESP-IDF 5.x
cmake_minimum_required(VERSION 3.16)

# Define the project name
project(lunabotics_common CXX)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 1. Define the library ---
# This creates a single library target named 'common_lib'.
# We list all the .cpp files that need to be compiled into it.
add_library(common_lib STATIC
    # --- Source Files (.cpp) ---
    src/drivers/FakeIMU.cpp
    # Add new .cpp files here as you create them:
    # src/drivers/Bno055.cpp
    # src/utils/pid_controller.cpp
)

# --- 2. Expose the include directory ---
# This command "attaches" the 'include' folder to the 'common_lib' target.
#
# By setting it to PUBLIC, any other project that links against
# 'common_lib' will AUTOMATICALLY get 'common/include'
# added to its include path.
target_include_directories(common_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# This should ideally test the fake_imu.cpp program.
if(BUILD_TESTING)
    # --- ADD THIS ---
    # Tell CMake to enable CTest
    enable_testing()

    # Create an executable named 'test_runner' from our new test file
    add_executable(test_runner
        test/testFakeIMU.cpp
    )

    # Link the 'test_runner' against our 'common_lib'
    target_link_libraries(test_runner PRIVATE common_lib)

    # --- AND ADD THIS ---
    # Register the executable 'test_runner' as a test named 'test_runner'
    add_test(NAME test_runner COMMAND test_runner)
endif()