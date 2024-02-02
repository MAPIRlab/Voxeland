#pragma once 
#include "probabilistic_map.hpp"
#include <pcl/io/pcd_io.h>
#include <bonxai_map/pcl_utils.hpp>
#include <bonxai_map/semantics.hpp>

namespace Bonxai
{

    /**
     * @brief The ProbabilisticMap class is meant to behave as much as possible as
     * octomap::Octree, given the same voxel size.
     *
     * Insert a point cloud to update the current probability
     */

    template <typename DataT>
    struct ProbabilisticCell
    {
        // variable used to check if a cell was already updated in this loop
        int32_t update_id : 4;
        // the probability of the cell to be occupied
        int32_t probability_log : 28;
        // Arbitrary data (RGB, Semantics, etc.)
        DataT data;

        ProbabilisticCell()
            : update_id(0)
            , probability_log(ProbabilisticMap::UnknownProbability){};
    };

    struct Color
    {
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

        Color(const pcl::PointXYZRGB& pcl)
        {
            r = pcl.r;
            g = pcl.g;
            b = pcl.b;
        }
        
        Color toColor()
        {
            return Color(*this);
        }
    };

    struct Empty
    {
        Empty() = default;
        Empty(const pcl::PointXYZ& pcl)
        {}

        Color toColor()
        {
            return Color(255, 255, 255);
        }
    };

    struct Semantics
    {
        std::vector<double> probabilities;
        uint8_t instanceID;

        Semantics() = default;

        Semantics(const pcl::PointXYZSemantics& pcl)
        {
            SemanticMap& semantics = SemanticMap::get_instance();
            instanceID = semantics.localToGlobalInstance(pcl.instance_id);
            probabilities = semantics.globalSemanticMap[instanceID].probabilities;
        }

        Color toColor()
        {
            SemanticMap& semantics = SemanticMap::get_instance();

            std::vector<double>::iterator it = std::max_element(probabilities.begin(), probabilities.end());

            uint32_t hexColor = semantics.indexToHexColor(instanceID);

            if(std::distance(probabilities.begin(), it) == (semantics.default_categories.size() - 1)){
                hexColor = 0xbcbcbc;
            }
                     
            return Color((hexColor >> 16) & 0xFF, (hexColor >> 8) & 0xFF, hexColor & 0xFF);
        }
    };

    struct RGBSemantics
    {
        Color rgb;
        Semantics semantics;

        RGBSemantics() {}

        RGBSemantics(const Color& _rgb, const Semantics& _semantics)
        : rgb(_rgb)
        , semantics(_semantics)
        {}

        RGBSemantics(const pcl::PointXYZRGBSemantics& pcl)
        {
            rgb.r = pcl.r;
            rgb.g = pcl.g;
            rgb.b = pcl.b;
        }

        Color toColor()
        {
            return rgb;
        }
    };
}