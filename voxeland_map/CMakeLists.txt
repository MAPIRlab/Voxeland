
add_library(voxeland_map
    include/voxeland_map/pcl_utils.hpp
    include/voxeland_map/cell_types.hpp
    include/voxeland_map/probabilistic_map.hpp
    include/voxeland_map/probabilistic_map_templated.hpp
    include/voxeland_map/semantics.hpp
    src/pcl_utils.cpp
    src/semantics.cpp
)

if(NOT TARGET fmt::fmt)
add_subdirectory(3rdparty/fmt)
endif()

get_directory_property(MYDEFS COMPILE_DEFINITIONS)


if(MYDEFS MATCHES "USING_ROS")
    ament_target_dependencies(voxeland_map rclcpp)
endif()

target_compile_definitions(voxeland_map PRIVATE
    ${PCL_DEFINITIONS})

target_include_directories(voxeland_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rdparty>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(voxeland_map
    bonxai_core
    fmt
    ${PCL_LIBRARIES})

####################################################

if(benchmark_FOUND AND octomap_FOUND)
    add_subdirectory(benchmark)
endif()
