# Define cpp sources once for all platforms:

set(SRCS
    src/drivers/FakeIMU.cpp
    # Add new .cpp files here as you create them:
    # src/drivers/Bno055.cpp
    # src/utils/pid_controller.cpp
)
# INFO: Header Files are Automatically included in each .cpp, the VScode configuration does not know this, you need to manually set settings.json when developing headers.

# Platform Specific Compilation Instructions:

# ESP-IDF Compilation:
if(DEFINED IDF_TARGET)
    # ESP-IDF component mode: Register as an IDF component
    idf_component_register(
        SRCS ${SRCS}
        INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    # Set C++17 for the component target, may need to be changed depending on features invoked in the common library.
    target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# TODO: Implement ROS2 target handling (if needed).
#if (ROS2)

# Standalone Compilation:
else()
    cmake_minimum_required(VERSION 3.16)
    project(lunabotics_common CXX)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    add_library(common_lib STATIC ${SRCS})
    target_include_directories(common_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if(BUILD_TESTING)
        enable_testing()
        add_executable(test_runner test/testFakeIMU.cpp)
        target_link_libraries(test_runner PRIVATE common_lib)
        add_test(NAME test_runner COMMAND test_runner)
    endif()
endif()