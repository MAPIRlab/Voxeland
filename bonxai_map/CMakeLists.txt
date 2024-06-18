
add_library(bonxai_map
    include/bonxai_map/pcl_utils.hpp
    include/bonxai_map/cell_types.hpp
    include/bonxai_map/probabilistic_map.hpp
    include/bonxai_map/probabilistic_map_templated.hpp
    include/bonxai_map/semantics.hpp
    src/pcl_utils.cpp
    src/semantics.cpp
)

if(NOT TARGET fmt::fmt)
add_subdirectory(3rdparty/fmt)
endif()

get_directory_property(MYDEFS COMPILE_DEFINITIONS)


if(MYDEFS MATCHES "USING_ROS")
    ament_target_dependencies(bonxai_map rclcpp)
endif()

target_compile_definitions(bonxai_map PRIVATE
    ${PCL_DEFINITIONS})

target_include_directories(bonxai_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(bonxai_map
    bonxai_core
    fmt
    ${PCL_LIBRARIES})

####################################################

if(benchmark_FOUND AND octomap_FOUND)
    add_subdirectory(benchmark)
endif()
