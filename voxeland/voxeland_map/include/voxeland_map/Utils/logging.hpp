#pragma once

#include <fmt/color.h>
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



//-------------------------
    // ASSERTS
//-------------------------

// Asserts will raise SIGTRAP if condition fails. If you have a debugger, that will stop it in the appropriate line. Otherwise, the program ends.
#if VXL_ENABLE_ASSERTS
#include <signal.h>
#define VXL_ASSERT_MSG(cnd, ...)                                                                                         \
    {                                                                                                                    \
        if (!(cnd))                                                                                                      \
        {                                                                                                                \
            VXL_ERROR("{0}:     At {1}",                                                                                 \
                      fmt::format(                                                                                       \
                          fmt::bg(fmt::terminal_color::red) | fmt::fg(fmt::terminal_color::white) | fmt::emphasis::bold, \
                          fmt::format(__VA_ARGS__)),                                                                     \
                      fmt::format(fmt::emphasis::bold, "{0}:{1}", __FILE__, __LINE__));                                  \
            raise(SIGTRAP);                                                                                              \
        }                                                                                                                \
    }

#define VXL_ASSERT_BLOCK(...) __VA_ARGS__
#else

#define VXL_ASSERT_MSG(cnd, msg)
#define VXL_ASSERT_BLOCK(...)

#endif

#define VXL_ASSERT(cnd) VXL_ASSERT_MSG(cnd, "")