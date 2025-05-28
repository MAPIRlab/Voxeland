cmake_minimum_required(VERSION 3.10)
project(voxeland)

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_VERBOSE_MAKEFILE ON) #Outputs the compiler command to the build log file

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")

  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-fsanitize=undefined -O0 -g")

#---- Add the subdirectory cmake ----
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wpedantic -fno-omit-frame-pointer)
endif()


if(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_definitions(VXL_ENABLE_ASSERTS=1)
endif()



####################################################

find_package(PCL REQUIRED COMPONENTS common io)

#find_package(benchmark QUIET)
find_package(octomap QUIET)
find_package(LZ4 QUIET)
find_package(Eigen3 QUIET)

# Default behavior
find_package(rclcpp)
find_package(ament_cmake QUIET)
find_package(OpenMP REQUIRED)

add_subdirectory(bonxai_core)
add_subdirectory(voxeland_map)

if ( ament_cmake_FOUND )

    project(voxeland)

    add_compile_definitions(USING_ROS=1)

    # Add colcon-specific configuration here
    message(STATUS "Building with colcon")

    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()

    include_directories(voxeland/include 3rdparty)
    ament_export_libraries(voxeland_map)
    ament_export_include_directories(include)

    # Server node
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
    
    # Install 
    ament_auto_package(
      INSTALL_TO_SHARE
      voxeland/launch 
      voxeland/rviz 
      voxeland/params 
    )

    # Include directories and library for downstream packages
    install(
        DIRECTORY bonxai_core/include/bonxai
        DESTINATION include
    )

    install(
        DIRECTORY voxeland_map/include/voxeland_map
        DESTINATION include
    )

    install(
        TARGETS voxeland_map 
        DESTINATION lib
    )

else()

  message(STATUS "Building with cmake")
  
endif()
