#pragma once

#include <fmt/format.h>

#if USING_ROS
#include <rclcpp/logging.hpp>
#define VXL_INFO(...) RCLCPP_INFO(rclcpp::get_logger("VXL"), "%s", fmt::format(__VA_ARGS__).c_str())
#define VXL_WARN(...) RCLCPP_WARN(rclcpp::get_logger("VXL"), "%s", fmt::format(__VA_ARGS__).c_str())
#define VXL_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("VXL"), "%s", fmt::format(__VA_ARGS__).c_str())
#else
#include <fmt/color.h>
#define VXL_INFO(...) fmt::print("[Info] {}", fmt::format(__VA_ARGS__))
#define VXL_WARN(...) fmt::print("[Warn] {}", fmt::format(fmt::fg(fmt::terminal_color::yellow), __VA_ARGS__))
#define VXL_ERROR(...) fmt::print("[Error] {}", fmt::format(fmt::fg(fmt::terminal_color::red), __VA_ARGS__))
#endif
