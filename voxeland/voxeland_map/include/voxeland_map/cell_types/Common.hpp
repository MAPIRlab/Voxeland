
#include <fmt/format.h>
#include <pcl/io/pcd_io.h>

#include <map>
#include <voxeland_map/dirichlet.hpp>
#include <voxeland_map/pcl_utils.hpp>
#include <voxeland_map/semantics.hpp>

#include "voxeland_map/probabilistic_map.hpp"

namespace voxeland
{
    struct Color;
    inline std::string XYZtoPLY(const Bonxai::Point3D& point);
    inline std::string RGBtoPLY(const Color& rgb);
    inline std::string getXYZheader();
    inline std::string getRGBheader();

    inline std::string XYZtoPLY(const Bonxai::Point3D& point)
    {
        return fmt::format("{} {} {}", point.x, point.y, point.z);
    }

    inline std::string getXYZheader()
    {
        return "property float x\n"
               "property float y\n"
               "property float z";
    }

    inline std::string getRGBheader()
    {
        return "property uchar red\n"
               "property uchar green\n"
               "property uchar blue";
    }

    // create a specialization for any DataT you want to be able to describe in the GUI
    template <typename DataT>
    std::string GetVoxelDescription(DataT& voxel)
    {
        return "GetVoxelDescription() not implemented for this type of voxel";
    }
}  // namespace voxeland