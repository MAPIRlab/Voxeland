#pragma once
#include "Color.hpp"

namespace voxeland
{
    struct Semantics
    {
        using PointCloudType = pcl::PointCloud<pcl::PointXYZSemantics>;
        std::vector<double> probabilities;

        Semantics(){};

        void update(const pcl::PointXYZSemantics& pcl) { UpdateProbabilities(pcl.instance_id); }

        virtual Color toColor()
        {
            SemanticMap& semantics = SemanticMap::get_instance();

            std::vector<double>::iterator it = std::max_element(probabilities.begin(), probabilities.end());
            uint8_t mainObjectCategory = std::distance(probabilities.begin(), it);
            uint32_t hexColor = semantics.indexToHexColor(mainObjectCategory);

            if (mainObjectCategory == (semantics.default_categories.size() - 1))
                hexColor = 0xbcbcbc;

            return Color::FromHex(hexColor);
        }

        std::string toPLY(const Bonxai::Point3D& point)
        {
            double uncertainty_categories = expected_shannon_entropy<double>(probabilities);
            return fmt::format("{} {} {}\n", XYZtoPLY(point), RGBtoPLY(toColor()), uncertainty_categories);
        }

        static std::string getHeaderPLY()
        {
            return fmt::format(
                "{}\n"
                "{}\n"
                "property float uncertainty_categories",
                getXYZheader(),
                getRGBheader());
        }

        std::vector<double> GetClassProbabilities() { return probabilities; }

    protected:
        void UpdateProbabilities(InstanceID_t id)
        {
            SemanticMap& semantics = SemanticMap::get_instance();
            if (!probabilities.empty())
            {
                for (InstanceID_t i = 0; i < probabilities.size(); i++)
                {
                    probabilities[i] += semantics.lastLocalSemanticMap[id].probabilities[i];
                }
            }
            else
            {
                probabilities = semantics.lastLocalSemanticMap[id].probabilities;
            }
        }
    };
}  // namespace Bonxai