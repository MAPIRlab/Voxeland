#pragma once
#include <sstream>

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
            std::vector<double> alphasDirichlet(semantics.getNumCategories(), 0.01);  // arbitrary amount of weight to all classes to avoid 0 probability

            for (InstanceID_t localInstanceID = 0; localInstanceID < instances_candidates.size(); localInstanceID++)
            {
                const SemanticObject* globalInstance = &semantics.globalSemanticMap[instances_candidates[localInstanceID]];

                // if the instance has been fused with others, find the new instance that represents the fusion
                while (!globalInstance->isStillValid())
                    globalInstance = &semantics.globalSemanticMap[globalInstance->pointsTo];

                float votesInstance = instances_votes[localInstanceID];
                for (size_t category = 0; category < semantics.getNumCategories(); category++)
                {
                    if (globalInstance->alphaParamsCategories.contains(category))
                        alphasDirichlet.at(category) += votesInstance * globalInstance->alphaParamsCategories.at(category);
                }
            }
            double sum = std::accumulate(alphasDirichlet.begin(), alphasDirichlet.end(), 0.);
            std::vector<double> probabilities(alphasDirichlet.size());

            for (size_t i = 0; i < alphasDirichlet.size(); i++)
                probabilities[i] = alphasDirichlet[i] / sum;

            return probabilities;
        }

        // if the instance with the most votes is background, but there is a real instance very close behind, returns the second one
        InstanceID_t getMostRepresentativeInstance()
        {
            if (instances_votes.size() == 0)
                return 0;
            updateCandidatesAndVotes();

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

            SemanticMap& semantics = SemanticMap::get_instance();
            const SemanticObject* globalInstance = &semantics.globalSemanticMap[instances_candidates[idxMaxVotes1]];

            return globalInstance->instanceID;
        }

        double GetProbabilityOfInstance(InstanceID_t id)
        {
            updateCandidatesAndVotes();
            size_t idx = -1;
            double sum = 0;
            for (size_t i = 0; i < instances_candidates.size(); i++)
            {
                sum = instances_votes.at(i);
                if (instances_candidates.at(i) == id)
                    idx = i;
            }

            if (idx == -1)
                return 0;
            return instances_votes.at(idx) / sum;
        }

        void ReplaceInstanceVotes(InstanceID_t out, InstanceID_t in)
        {
            updateCandidatesAndVotes();

            size_t idxOut = std::distance(instances_candidates.begin(), std::find(instances_candidates.begin(), instances_candidates.end(), out));
            size_t idxIn = std::distance(instances_candidates.begin(), std::find(instances_candidates.begin(), instances_candidates.end(), in));

            if (idxOut < instances_candidates.size())
            {
                if (idxIn < instances_candidates.size())
                {
                    // if the "in" instance was already in this voxel, combine both lots of votes and delete the "out" instance as a candidate
                    instances_candidates.at(idxIn) += instances_candidates.at(idxOut);
                    instances_candidates.erase(instances_candidates.begin() + idxOut);
                    instances_votes.erase(instances_votes.begin() + idxOut);
                }
                else
                    instances_candidates.at(idxOut) = in;
            }
        }

        // we implement this one as a class member because we really would like to call updateCandidatesAndVotes() before presenting any info to the user
        std::string GetDebugInfo()
        {
            updateCandidatesAndVotes();
            std::stringstream ss;
            for (size_t i = 0; i < instances_candidates.size(); i++)
                ss << "ojb" << instances_candidates.at(i) << ": " << instances_votes.at(i) << " votes\n";

            return ss.str();
        }

    protected:
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
            // TODO (pepe) since this adds a vote for every *pixel* that falls inside the voxel, we could end up running into numerical precission issues if left running for a while
            // might be a good idea to, at some point, reduce the votes to all the instances by a set amount to avoid that
        }

        // account for instances having been fused since they were last observed
        void updateCandidatesAndVotes()
        {
            SemanticMap& semantics = SemanticMap::get_instance();

            // possible early out
            {
                bool needed = false;
                for (size_t i = 0; i < instances_candidates.size(); i++)
                    if (!semantics.globalSemanticMap.at(instances_candidates.at(i)).isStillValid())
                        needed = true;

                if (!needed)
                    return;
            }

            std::vector<InstanceID_t> candidates_temp;
            candidates_temp.reserve(instances_candidates.size());
            std::map<InstanceID_t, uint32_t> combining_instances;

            for (InstanceID_t i = 0; i < instances_candidates.size(); i++)
            {
                if (semantics.globalSemanticMap[instances_candidates[i]].isStillValid())
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

    template <>
    inline std::string GetVoxelDescription(SemanticsInstances& voxel)
    {
        return voxel.GetDebugInfo();
    }
}  // namespace voxeland