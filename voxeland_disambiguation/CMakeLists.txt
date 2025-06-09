cmake_minimum_required(VERSION 3.10)
project(voxeland_disambiguation)

SET(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-fsanitize=undefined -O0 -g")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Buscar recursivamente todos los .cpp en voxeland/test/disambiguation
  file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/test/disambiguation/*.cpp
  )

  foreach(test_source ${TEST_SOURCES})
    # Obtener el nombre base del archivo sin extensión
    get_filename_component(test_name ${test_source} NAME_WE)

    ament_add_gtest(${PROJECT_NAME}_${test_name} ${test_source})
    target_include_directories(${PROJECT_NAME}_${test_name} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${PROJECT_NAME}_${test_name} voxeland_disambiguation)
  endforeach()
endif()

add_compile_definitions(USING_ROS=1)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(voxeland REQUIRED)
ament_auto_find_build_dependencies()


# Disambiguation node
file(GLOB_RECURSE SOURCES "src/disambiguation/*.cpp")
ament_auto_add_library(voxeland_disambiguation SHARED
    ${SOURCES}
)
ament_target_dependencies(voxeland_disambiguation
    voxeland
)

include_directories(include 3rdparty)

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
