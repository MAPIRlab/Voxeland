#pragma once 
#include "Semantics.hpp"

namespace voxeland
{
    struct RGBSemantics : public Semantics
    {
        using PointCloudType = pcl::PointCloud<pcl::PointXYZRGBSemantics>;
        Color rgb;
        std::vector<double> probabilities;

        RGBSemantics()
            : rgb()
        {}

        void update(const pcl::PointXYZRGBSemantics& pcl)
        {
            UpdateProbabilities(pcl.instance_id);

            rgb.r = pcl.r;
            rgb.g = pcl.g;
            rgb.b = pcl.b;
        }

        Color toColor() override
        {
            SemanticMap& semantics = SemanticMap::get_instance();

            std::vector<double>::iterator it = std::max_element(probabilities.begin(), probabilities.end());
            uint8_t mainObjectCategory = std::distance(probabilities.begin(), it);
            
            if (mainObjectCategory == (semantics.default_categories.size() - 1))
                return rgb;
            
            uint32_t hexColor = semantics.indexToHexColor(mainObjectCategory);
            return Color::FromHex(hexColor);
        }
    };
}