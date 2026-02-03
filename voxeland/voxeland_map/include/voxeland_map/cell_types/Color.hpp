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

    inline std::uint32_t serializeColor(Color color)
    {
        return (std::uint32_t)color.r << 16 | (std::uint32_t)color.g << 8 | (std::uint32_t)color.b;
    }

    inline Color valueToColor(double val, double lowLimit, double highLimit)
    {
        double r, g, b;
        double range = (highLimit - lowLimit) / 4;

        if (val < lowLimit + range)
        {
            double t = (val - lowLimit) / range;
            r = 0;
            g = std::lerp(0., 255., t);
            b = 255;
        }
        else if (val < lowLimit + 2 * range)
        {
            val -= range;
            double t = (val - lowLimit) / range;
            r = 0;
            g = 255;
            b = std::lerp(255., 0., t);
        }
        else if (val < lowLimit + 3 * range)
        {
            val -= 2 * range;
            double t = (val - lowLimit) / range;
            r = std::lerp(0., 255., t);
            g = 255;
            b = 0;
        }
        else if(val < lowLimit + 4 * range)
        {
            val -= 3 * range;
            double t = (val - lowLimit) / range;
            r = 255;
            g = std::lerp(255., 0., t);
            b = 0;
        } 
        else
        {
            r = 255;
            g = 0;
            b = 255;
        }
        return Color(r, g, b);
    }

}  // namespace voxeland