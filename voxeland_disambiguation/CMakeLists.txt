cmake_minimum_required(VERSION 3.10)
project(voxeland_disambiguation)

SET(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-fsanitize=undefined -O0 -g")

find_package(ament_cmake REQUIRED)
find_package(OpenMP REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Disambiguation node
file(GLOB_RECURSE SOURCES "src/disambiguation/*.cpp")
ament_auto_add_library(voxeland_disambiguation SHARED
    ${SOURCES}
)

target_link_libraries(voxeland_disambiguation
OpenMP::OpenMP_CXX
)

# Install 
install(
   DIRECTORY launch
   DESTINATION share/${PROJECT_NAME} 
)

ament_package()
