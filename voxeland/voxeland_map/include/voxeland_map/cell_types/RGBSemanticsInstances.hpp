#pragma once
#include "SemanticsInstances.hpp"

namespace voxeland
{
    struct RGBSemanticsInstances : public SemanticsInstances
    {
        using PointCloudType = pcl::PointCloud<pcl::PointXYZRGBSemantics>;
        Color rgb;

        RGBSemanticsInstances(){};

        void update(const pcl::PointXYZRGBSemantics& pcl)
        {
            SemanticMap& semantics = SemanticMap::get_instance();
            InstanceID_t thisGlobalID = semantics.localToGlobalInstance(pcl.instance_id);
            AddVote(thisGlobalID);

            rgb.r = pcl.r;
            rgb.g = pcl.g;
            rgb.b = pcl.b;
        }

        Color toColor() override
        {
            updateCandidatesAndVotes();

            InstanceID_t bestInstance = getMostRepresentativeInstance();
            if (bestInstance == 0)
                return rgb;

            // Set a unique color for the most probable instance
            uint32_t hexColor = SemanticMap::get_instance().indexToHexColor(bestInstance);
            return Color::FromHex(hexColor);
        }
    };
}  // namespace Bonxai