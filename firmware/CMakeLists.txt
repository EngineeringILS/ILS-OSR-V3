# Define the Cmake Version
cmake_minimum_required(VERSION 3.5)
# Append the common library as an Extra Component
list(APPEND EXTRA_COMPONENT_DIRS ../common)
# Invoke the ESP-IDF CMake Tools (Avoids OS Level Errors)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Invoke the ESP-IDF firmware Project
project(firmware)