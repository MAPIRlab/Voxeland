#pragma once

#if USING_ROS
    #include <rclcpp/logging.hpp>
    #define BONXAI_INFO(...) RCLCPP_INFO(rclcpp::get_logger("Bonxai"), __VA_ARGS__)
#else
    #define BONXAI_INFO(...) printf(__VA_ARGS__)
#endif
