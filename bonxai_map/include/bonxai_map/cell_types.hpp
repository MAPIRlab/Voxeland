#pragma once 
#include "probabilistic_map.hpp"

namespace Bonxai
{

    /**
     * @brief The ProbabilisticMap class is meant to behave as much as possible as
     * octomap::Octree, given the same voxel size.
     *
     * Insert a point cloud to update the current probability
     */

    struct ProbabilisticCell
    {
    // variable used to check if a cell was already updated in this loop
    int32_t update_id : 4;
    // the probability of the cell to be occupied
    int32_t probability_log : 28;

    ProbabilisticCell()
        : update_id(0)
        , probability_log(ProbabilisticMap::UnknownProbability){};
    };
}