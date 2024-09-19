cmake_minimum_required(VERSION 3.10)
project(voxeland)

SET(CMAKE_BUILD_TYPE "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")

  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_RELEASE "-fno-omit-frame-pointer -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-fno-omit-frame-pointer -O0 -g")

#---- Add the subdirectory cmake ----
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(PCL REQUIRED COMPONENTS common io)

#find_package(benchmark QUIET)
find_package(octomap QUIET)
find_package(LZ4 QUIET)
find_package(Eigen3 QUIET)

#---- Use Address sanitizer if compiled in Debug mode ----
option(BONXAI_SANITIZE "Add address sanitizer when compiling in Debug mode" ON)

if(BONXAI_SANITIZE)
    set (CMAKE_CXX_DEBUG_FLAGS "${CMAKE_CXX_DEBUG_FLAGS}  -fsanitize=address")
    set (CMAKE_LINKER_DEBUG_FLAGS "${CMAKE_LINKER_DEBUG_FLAGS}  -fsanitize=address")
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wpedantic -fno-omit-frame-pointer)
endif()

add_compile_definitions(USING_ROS=1)

####################################################

# Default behavior
find_package(rclcpp)
find_package(ament_cmake QUIET)
find_package(OpenMP REQUIRED)

add_subdirectory(bonxai_core)
add_subdirectory(voxeland_map)

if ( ament_cmake_FOUND )

    project(voxeland)

    # Add colcon-specific configuration here
    message(STATUS "Building with colcon")

    find_package(ament_cmake_auto REQUIRED)
    find_package(segmentation_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(vision_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    ament_auto_find_build_dependencies()

    include_directories(voxeland/include 3rdparty)

    ament_auto_add_library(voxeland_server SHARED
      voxeland/src/voxeland_server.cpp
    )

    target_link_libraries(voxeland_server
      voxeland_map
      ${PCL_LIBRARIES}
      OpenMP::OpenMP_CXX
    )

    rclcpp_components_register_node(voxeland_server
      PLUGIN "voxeland_server::VoxelandServer"
      EXECUTABLE voxeland_server_node
    )

    # Messages and services
    find_package(rosidl_default_generators REQUIRED)
    rosidl_generate_interfaces(${PROJECT_NAME}
        voxeland/msg/ClassDistribution.msg
        voxeland/srv/GetClassDistributions.srv
        DEPENDENCIES geometry_msgs vision_msgs
    )
    ament_export_dependencies(rosidl_default_runtime)
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(voxeland_server "${cpp_typesupport_target}") 


    ament_auto_package(
      INSTALL_TO_SHARE
      voxeland/launch 
      voxeland/rviz 
      voxeland/params 
    )

else()

  message(STATUS "Building with cmake")
  
endif()
