cmake_minimum_required(VERSION 3.10)
project(voxeland_disambiguation)

SET(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-fsanitize=undefined -O0 -g")

add_compile_definitions(USING_ROS=1)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# Disambiguation node
file(GLOB_RECURSE SOURCES "src/disambiguation/*.cpp")
ament_auto_add_library(voxeland_disambiguation SHARED
    ${SOURCES}
)

include_directories(voxeland_disambiguation/include 3rdparty)

rclcpp_components_register_node(voxeland_disambiguation
    PLUGIN "voxeland_disambiguation::VoxelandDisambiguation"
    EXECUTABLE voxeland_disambiguation_node
)

# Install 
install(
   DIRECTORY launch params
   DESTINATION share/${PROJECT_NAME} 
)

# Install 
ament_auto_package()
