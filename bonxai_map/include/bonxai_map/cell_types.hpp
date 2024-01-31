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

    struct Empty
    {};

    struct Color
    {
    uint8_t r;
    uint8_t g;
    uint8_t b;

    Color(uint8_t _r, uint8_t _g, uint8_t _b)
    : r(_r)
    , g(_g)
    , b(_b)
    {}

    };

    struct Semantics
    {
    uint8_t r;
    uint8_t g;
    uint8_t b;

    Semantics(uint8_t _r, uint8_t _g, uint8_t _b)
    : r(_r)
    , g(_g)
    , b(_b)
    {}

    };

    struct RGBSemantics
    {
    Color rgb;
    Semantics semantics;

    RGBSemantics(const Color& _rgb, const Semantics& _semantics)
    : rgb(_rgb)
    , semantics(_semantics)
    {}

    };
}