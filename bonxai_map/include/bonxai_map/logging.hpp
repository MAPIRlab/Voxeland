#pragma once

#include <fmt/format.h>

#if USING_ROS
#include <rclcpp/logging.hpp>
#define BONXAI_INFO(...) RCLCPP_INFO(rclcpp::get_logger("Bonxai"), "%s", fmt::format(__VA_ARGS__).c_str())
#else
#define BONXAI_INFO(...) fmt::print(__VA_ARGS__)
#endif
