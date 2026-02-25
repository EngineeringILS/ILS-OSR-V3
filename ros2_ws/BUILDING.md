### Project Build Guide:
 - ROS2 uses ament_cmake() which has strict requirements for internal project and file structure.
 - To include `/common`, several methods (below) were developed to ensure proper compilation.
 - To provide proper syntax highlighting, use the provided `.vscode` setup (below).


### Package Setup:
- This guide covers how to setup a ROS2 Package and modify the `CMakeLists.txt` of the created package to properly include `common`:
1. Generate a Library Package from the ros2_ws root directory:
```Bash
# Set working directory as the ros2_ws root.
# Generating a new package with ament_cmake:
ros2 pkg create $PackageName --build-type ament_cmake --library-name $LibraryName
```
2. Modify the `CMakeLists.txt` of the created package:
- Insert below `find_packages()`:
```makefile
set(BUILD_ROS2 ON)
get_filename_component(COMMON_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../common" ABSOLUTE)
add_subdirectory(${COMMON_LIB_PATH} ${CMAKE_CURRENT_BINARY_DIR}/common_build)
```

- Insert below `target_compile_definitions()`
```makefile
target_link_libraries(${PROJECT_NAME} 
  PRIVATE
  common_lib
)
```

3. Include the created library in the end-user node's `package.xml`:
- Add inside of <depend> setup:
```xml
<depend>$LibaryName</depend>
```

4. Include the created libray in the end-user node's `CMakeLists.txt`:
- Insert below `find_packages()`:
```makefile
find_package(ros2_serial_driver REQUIRED)
```
- Insert into `ament_target_dependencies()`:
```makefile
$LibraryName
```

5. The Node should now properly reference the Package `$LibraryName` which properly references `/common`.

>**Note**: This guide was created with great difficultly, and it may not be entirely correct, if any issues result, please reference the [ROS2 documentation](https://docs.ros.org/en/humble/index.html) for more information.

>**Note**: This guide summarized the process used to sucessfully setup `ros2_serial_driver` (a library pakcage requiring `common`) and `esp32_bridge` (a node requiring `ros2_serial_driver`), referencing the `CMakeLists.txt` of both may provide better insight.

### Intellisense Setup:
1. Create a .vscode/ folder inside `ros2_ws`
```bash 
mkdir .vscode
```
2. Create `c_cpp_properties.json` with the following content:
```json
{
    "configurations": [
        {
            "name": "ROS 2 Humble (colcon)",
            "compileCommands": "${workspaceFolder}/build/compile_commands.json",
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```
3. Build the workspace from `ros2_ws`:
```bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

4. Intellisense should now be working, create a GitHub issue if it continues to have problems.