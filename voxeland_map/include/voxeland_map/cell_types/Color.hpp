#pragma once
#include "Common.hpp"

namespace voxeland
{
    struct Color
    {
        using PointCloudType = pcl::PointCloud<pcl::PointXYZRGB>;
        uint8_t r;
        uint8_t g;
        uint8_t b;

        Color() = default;
        Color(const Color& other) = default;

        Color(uint8_t _r, uint8_t _g, uint8_t _b)
            : r(_r)
            , g(_g)
            , b(_b)
        {}

        void update(const pcl::PointXYZRGB& pcl)
        {
            r = pcl.r;
            g = pcl.g;
            b = pcl.b;
        }

        Color toColor() { return Color(*this); }

        std::string toPLY(const Bonxai::Point3D& point) { return fmt::format("{} {}\n", XYZtoPLY(point), RGBtoPLY(*this)); }

        static std::string getHeaderPLY() { return fmt::format("{}\n{}", getXYZheader(), getRGBheader()); }

        static Color FromHex(uint32_t hexColor) { return Color((hexColor >> 16) & 0xFF, (hexColor >> 8) & 0xFF, hexColor & 0xFF); }
    };

    inline std::string RGBtoPLY(const Color& rgb)
    {
        return fmt::format("{} {} {}", rgb.r, rgb.g, rgb.b);
    }

}  // namespace voxeland