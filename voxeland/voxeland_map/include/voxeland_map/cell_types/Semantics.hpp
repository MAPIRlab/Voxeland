#pragma once
#include <unordered_map>
#include <voxeland_map/Utils/Math.hpp>
#include <voxeland_map/category_manager.hpp>

#include "Color.hpp"

namespace voxeland
{
    struct Semantics
    {
        using PointCloudType = pcl::PointCloud<pcl::PointXYZSemantics>;

        // Dynamic storage for Dirichlet parameters - grows as needed
        std::unordered_map<CategoryManager::CategoryIndex, double> alphasDirichlet;

        Semantics() {}

        void update(const pcl::PointXYZSemantics& pcl)
        {
            UpdateProbabilities(pcl.instance_id);
        }

        virtual Color toColor()
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

            uint32_t hexColor = semantics.indexToHexColor(mainObjectCategory);

            // The background category gets this grey color
            if (mainObjectCategory == CategoryManager::UNKNOWN_CATEGORY)
                hexColor = 0xbcbcbc;

            return Color::FromHex(hexColor);
        }

        std::string toPLY(const Bonxai::Point3D& point)
        {
            // Convert to vector for entropy calculation
            std::vector<double> alphaVector = getAlphasDirichletVector();
            double uncertainty_categories = expected_shannon_entropy<double>(alphaVector);
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

        // Helper method to get alphas as vector for compatibility
        std::vector<double> getAlphasDirichletVector() const
        {
            CategoryManager& catManager = CategoryManager::getInstance();
            size_t numCategories = catManager.getNumCategories();
            std::vector<double> alphaVector(numCategories, 0.0);

            for (const auto& [categoryIndex, probability] : alphasDirichlet)
            {
                if (categoryIndex < numCategories)
                {
                    alphaVector[categoryIndex] = probability;
                }
            }

            return alphaVector;
        }

        std::vector<double> GetClassProbabilities()
        {
            // Convert to vector format and give small weight to every class to avoid 0 probabilities
            std::vector<double> alphasDirichlet_local = getAlphasDirichletVector();
            for (size_t i = 0; i < alphasDirichlet_local.size(); i++)
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

            const SemanticObject& semanticObject = semantics.lastLocalSemanticMap.at(id);

            // Merge with existing alphas or set new ones
            if (alphasDirichlet.empty())
                alphasDirichlet = semanticObject.alphaParamsCategories;
            else
            {
                for (const auto& [categoryIndex, probability] : semanticObject.alphaParamsCategories)
                    alphasDirichlet[categoryIndex] += probability;
            }
        }
    };
}  // namespace voxeland