#pragma once
#include "Color.hpp"

namespace voxeland
{
    struct SemanticsInstances
    {
        using PointCloudType = pcl::PointCloud<pcl::PointXYZSemantics>;
        std::vector<InstanceID_t> instances_candidates;
        std::vector<uint32_t> instances_votes;

        SemanticsInstances() {};

        void update(const pcl::PointXYZSemantics& pcl)
        {
            SemanticMap& semantics = SemanticMap::get_instance();
            InstanceID_t thisGlobalID = semantics.localToGlobalInstance(pcl.instance_id);
            AddVote(thisGlobalID);
        }

        virtual Color toColor()
        {
            updateCandidatesAndVotes();

            // Set a unique color for the most probable instance
            InstanceID_t bestInstance = getMostRepresentativeInstance();
            uint32_t hexColor = SemanticMap::get_instance().indexToHexColor(bestInstance);

            return Color::FromHex(hexColor);
        }

        std::string toPLY(const Bonxai::Point3D& point)
        {
            updateCandidatesAndVotes();
            std::vector<double> total_probability = GetClassProbabilities();
            InstanceID_t instanceid = getMostRepresentativeInstance();
            double uncertainty_instances = expected_shannon_entropy<uint32_t>(instances_votes);
            double uncertainty_categories = expected_shannon_entropy<double>(total_probability);
            return fmt::format("{} {} {} {} {}\n", XYZtoPLY(point), RGBtoPLY(toColor()), instanceid, uncertainty_instances, uncertainty_categories);
        }

        static std::string getHeaderPLY()
        {
            return fmt::format(
                "{}\n"
                "{}\n"
                "property int instanceid\n"
                "property float uncertainty_instances\n"
                "property float uncertainty_categories",
                getXYZheader(),
                getRGBheader());
        }

        std::vector<double> GetClassProbabilities()
        {
            if (instances_candidates.size() == 0)
                return {};
            SemanticMap& semantics = SemanticMap::get_instance();
            std::vector<double> alphasDirichlet(semantics.default_categories.size(), 0.01); //arbitrary amount of weight to all classes to avoid 0 probability

            for (InstanceID_t localInstanceID = 0; localInstanceID < instances_candidates.size(); localInstanceID++)
            {
                const SemanticObject* globalInstance = &semantics.globalSemanticMap[instances_candidates[localInstanceID]];
                
                // if the instance has been fused with others, find the new instance that represents the fusion
                while (globalInstance->pointsTo != -1)
                    globalInstance = &semantics.globalSemanticMap[globalInstance->pointsTo];

                float votesInstance = instances_votes[localInstanceID];
                for (size_t category = 0; category < semantics.default_categories.size(); category++)
                {
                    alphasDirichlet.at(category) += votesInstance * globalInstance->alphaParamsCategories.at(category);
                }
            }
            double sum = std::accumulate(alphasDirichlet.begin(), alphasDirichlet.end(), 0.);
            std::vector<double> probabilities(alphasDirichlet.size());

            for (size_t i = 0; i < alphasDirichlet.size(); i++)
                probabilities[i] = alphasDirichlet[i] / sum;

            return probabilities;
        }

    protected:
        // if the instance with the most votes is background, but there is a real instance very close behind, returns the second one
        InstanceID_t getMostRepresentativeInstance()
        {
            InstanceID_t idxMaxVotes1 = 0;

            if (instances_votes.size() > 1)
            {
                InstanceID_t idxMaxVotes2 = 0;

                uint32_t max1 = instances_votes[0];
                uint32_t max2 = std::numeric_limits<uint32_t>::min();

                for (InstanceID_t i = 1; i < instances_votes.size(); ++i)
                {
                    if (instances_votes[i] > max1)
                    {
                        // Update the second largest before updating the largest
                        max2 = max1;
                        idxMaxVotes2 = idxMaxVotes1;
                        max1 = instances_votes[i];
                        idxMaxVotes1 = i;
                    }
                    else if (instances_votes[i] > max2)
                    {
                        max2 = instances_votes[i];
                        idxMaxVotes2 = i;
                    }
                }

                if ((instances_candidates[idxMaxVotes1] == 0) && (max1 * 0.2 < max2))
                    idxMaxVotes1 = idxMaxVotes2;
            }

            return instances_candidates[idxMaxVotes1];
        }

        void AddVote(InstanceID_t thisGlobalID)
        {
            auto it = std::find(instances_candidates.begin(), instances_candidates.end(), thisGlobalID);
            if (it != instances_candidates.end())
                instances_votes[std::distance(instances_candidates.begin(), it)] += 1;
            else
            {
                instances_candidates.push_back(thisGlobalID);
                instances_votes.push_back(1);
            }
        }

        // account for instances having been fused since they were last observed
        void updateCandidatesAndVotes()
        {
            SemanticMap& semantics = SemanticMap::get_instance();

            std::vector<InstanceID_t> candidates_temp;
            candidates_temp.reserve(instances_candidates.size());
            std::map<InstanceID_t, uint32_t> combining_instances;

            for (InstanceID_t i = 0; i < instances_candidates.size(); i++)
            {
                if (semantics.globalSemanticMap[instances_candidates[i]].pointsTo == -1)
                    candidates_temp.push_back(instances_candidates[i]);
                else
                    candidates_temp.push_back(semantics.globalSemanticMap[instances_candidates[i]].pointsTo);
            }

            for (InstanceID_t i = 0; i < candidates_temp.size(); i++)
            {
                combining_instances[candidates_temp[i]] += instances_votes[i];
            }

            instances_candidates.clear();
            instances_votes.clear();

            for (const std::pair<InstanceID_t, uint32_t>& instance : combining_instances)
            {
                instances_candidates.push_back(instance.first);
                instances_votes.push_back(instance.second);
            }
        }
    };
}  // namespace voxeland