#pragma once

namespace voxeland
{

    enum class DataMode
    {
        Uninitialized = 1,
        Empty = 0,
        RGB = 1 << 1,
        Semantics = 1 << 2,
        SemanticsInstances = Semantics | (1 << 3),
        RGBSemantics = RGB | Semantics,
        RGBSemanticsInstances = RGBSemantics | SemanticsInstances
    };

    inline DataMode operator|(DataMode a, DataMode b)
    {
        return static_cast<DataMode>(static_cast<int>(a) | static_cast<int>(b));
    }

    inline DataMode operator&(DataMode a, DataMode b)
    {
        return static_cast<DataMode>(static_cast<int>(a) & static_cast<int>(b));
    }
};  // namespace 