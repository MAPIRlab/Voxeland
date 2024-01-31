#pragma once
#include <string>
#include <eigen3/Eigen/Core>
#include "bonxai/bonxai.hpp"

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

namespace pcl
{

 struct PointXYZSemantics{
    float x;
    float y;
    float z;

    uint8_t instance_id;
 };

struct PointXYZRGBSemantics{
    float x;
    float y;
    float z;

    uint8_t b;
    uint8_t g;
    uint8_t r;

    uint8_t instance_id;
 };

}
