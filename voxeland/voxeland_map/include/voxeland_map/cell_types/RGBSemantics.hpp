#pragma once 
#include "Semantics.hpp"

namespace voxeland
{
    struct RGBSemantics : public Semantics
    {
        using PointCloudType = pcl::PointCloud<pcl::PointXYZRGBSemantics>;
        Color rgb;

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
            CategoryManager& catManager = CategoryManager::getInstance();

            // Find category with maximum probability
            CategoryManager::CategoryIndex mainObjectCategory = CategoryManager::UNKNOWN_CATEGORY;
            double maxProbability = 0.0;
            
            for (const auto& [categoryIndex, probability] : alphasDirichlet)
            {
                if (probability > maxProbability)
                {
                    maxProbability = probability;
                    mainObjectCategory = categoryIndex;
                }
            }
            
            if (mainObjectCategory == CategoryManager::UNKNOWN_CATEGORY)
                return rgb;
            
            uint32_t hexColor = semantics.indexToHexColor(mainObjectCategory);
            return Color::FromHex(hexColor);
        }
    };
}