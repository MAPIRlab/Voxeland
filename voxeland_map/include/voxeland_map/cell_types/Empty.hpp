#pragma once
#include "Color.hpp"

namespace voxeland
{
    struct Empty
    {
        using PointCloudType = pcl::PointCloud<pcl::PointXYZ>;
        Empty() = default;

        void update(const pcl::PointXYZ& pcl) {}

        Color toColor() { return Color(255, 255, 255); }

        std::string toPLY(const Bonxai::Point3D& point) { return fmt::format("{}\n", XYZtoPLY(point)); }

        static std::string getHeaderPLY() { return getXYZheader(); }
    };
}  // namespace voxeland