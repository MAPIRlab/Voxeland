#pragma once
#include <string>
#include <eigen3/Eigen/Core>
#include "bonxai/bonxai.hpp"

#define PCL_NO_PRECOMPILE
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace Bonxai
{

bool ReadPointsFromPCD(const std::string& filepath,
                       std::vector<Eigen::Vector3d>& points);

bool ReadPointsFromPCD(const std::string& filepath, std::vector<Point3D>& points);

void WritePointsFromPCD(const std::string& filepath,
                        const std::vector<Eigen::Vector3d>& points);

void WritePointsFromPCD(const std::string& filepath,
                        const std::vector<Bonxai::Point3D>& points);

void WritePointsFromPCD(const std::string& filepath,
                        const std::vector<Bonxai::CoordT>& points);

}  // namespace Bonxai

namespace pcl {

struct PointXYZSemantics{

    PCL_ADD_POINT4D;              
    int32_t instance_id;
    
};

struct PointXYZRGBSemantics{
    
    PCL_ADD_POINT4D; 

    uint8_t b;
    uint8_t g;
    uint8_t r;

    uint8_t instance_id;
};

}

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZSemantics,           
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (int32_t, instance_id, instance_id)
                                   )



POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZRGBSemantics,           // here we assume a XYZ + "test" (as fields)
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint8_t, b, b)
                                  (uint8_t, g, g)
                                  (uint8_t, r, r)
                                  (uint8_t, instance_id, instance_id)
                                  )


