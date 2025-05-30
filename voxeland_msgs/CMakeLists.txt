cmake_minimum_required(VERSION 3.10)
project(voxeland_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
# Messages and services
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
    msg/ClassDistribution.msg
    srv/GetClassDistributions.srv
    srv/LoadMap.srv
    DEPENDENCIES geometry_msgs vision_msgs
)
ament_export_dependencies(rosidl_default_runtime)



ament_package()
