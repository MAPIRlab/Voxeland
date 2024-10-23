#pragma once
#include <voxeland_map/Utils/Math.hpp>

#include "Color.hpp"

namespace voxeland
{
    struct Semantics
    {
        using PointCloudType = pcl::PointCloud<pcl::PointXYZSemantics>;
        std::vector<double> alphasDirichlet;

        Semantics() {}

        void update(const pcl::PointXYZSemantics& pcl)
        {
            UpdateProbabilities(pcl.instance_id);
        }

        virtual Color toColor()
        {
            SemanticMap& semantics = SemanticMap::get_instance();

            std::vector<double>::iterator it = std::max_element(alphasDirichlet.begin(), alphasDirichlet.end());
            uint8_t mainObjectCategory = std::distance(alphasDirichlet.begin(), it);
            uint32_t hexColor = semantics.indexToHexColor(mainObjectCategory);

            // the last one is the background category, which always gets this grey color
            if (mainObjectCategory == (semantics.default_categories.size() - 1))
                hexColor = 0xbcbcbc;

            return Color::FromHex(hexColor);
        }

        std::string toPLY(const Bonxai::Point3D& point)
        {
            double uncertainty_categories = expected_shannon_entropy<double>(alphasDirichlet);
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

        std::vector<double> GetClassProbabilities()
        {
            // give a small weight to every class to avoid 0 probabilities
            std::vector<double> alphasDirichlet_local = alphasDirichlet;
            for (size_t i = 0; i < alphasDirichlet.size(); i++)
                alphasDirichlet_local[i] = std::max(alphasDirichlet_local[i], 1.);

            double sum = std::accumulate(alphasDirichlet_local.begin(), alphasDirichlet_local.end(), 0.);
            std::vector<double> probabilities(alphasDirichlet_local.size());

            for (size_t i = 0; i < alphasDirichlet_local.size(); i++)
                probabilities[i] = alphasDirichlet_local[i] / sum;
            VXL_ASSERT_MSG(probabilities.size() == 0 || Utils::approx(std::accumulate(probabilities.begin(), probabilities.end(), 0.), 1),
                           "Class probabilities are not normalized!");

            return probabilities;
        }

    protected:
        void UpdateProbabilities(InstanceID_t id)
        {
            SemanticMap& semantics = SemanticMap::get_instance();
            if (!alphasDirichlet.empty())
            {
                for (InstanceID_t i = 0; i < alphasDirichlet.size(); i++)
                    alphasDirichlet[i] += semantics.lastLocalSemanticMap[id].alphaParamsCategories[i];
            }
            else
                alphasDirichlet = semantics.lastLocalSemanticMap[id].alphaParamsCategories;
        }
    };
}  // namespace voxeland