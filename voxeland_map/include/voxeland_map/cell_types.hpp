#pragma once

#include <voxeland_map/data_modes.hpp>

#include "cell_types/Color.hpp"
#include "cell_types/Empty.hpp"
#include "cell_types/RGBSemantics.hpp"
#include "cell_types/RGBSemanticsInstances.hpp"
#include "cell_types/Semantics.hpp"
#include "cell_types/SemanticsInstances.hpp"

namespace voxeland
{
    //this takes a function template that expects a cell type as the template argument and a DataMode enum
    //the fun thing is that, since we are doing things with a switch, the enum can be a runtime value
    template <typename F>
    inline auto TemplateFromEnum(DataMode t, F&& f)
    {
        switch (t)
        {
            case DataMode::Empty:
                return f.template operator()<Empty>();
            case DataMode::RGB:
                return f.template operator()<Color>();
            case DataMode::Semantics:
                return f.template operator()<Semantics>();
            case DataMode::RGBSemantics:
                return f.template operator()<RGBSemantics>();
            case DataMode::SemanticsInstances:
                return f.template operator()<SemanticsInstances>();
            case DataMode::RGBSemanticsInstances:
                return f.template operator()<RGBSemanticsInstances>();
            case DataMode::Uninitialized:
                VXL_ERROR("Tried to auto-template a function with an uninitialized enum!");
        };
    }

    template <typename F>
    inline auto TemplateFromEnumInstanceOnly(DataMode t, F&& f)
    {
        switch (t)
        {
            case DataMode::SemanticsInstances:
                return f.template operator()<SemanticsInstances>();
            case DataMode::RGBSemanticsInstances:
                return f.template operator()<RGBSemanticsInstances>();
            case DataMode::Uninitialized:
                VXL_ERROR("Tried to auto-template a function with an uninitialized enum!");
        };
    }

    template <typename F>
    inline auto TemplateFromEnumSemanticsOnly(DataMode t, F&& f)
    {
        switch (t)
        {
            case DataMode::Semantics:
                return f.template operator()<Semantics>();
            case DataMode::RGBSemantics:
                return f.template operator()<RGBSemantics>();
            case DataMode::SemanticsInstances:
                return f.template operator()<SemanticsInstances>();
            case DataMode::RGBSemanticsInstances:
                return f.template operator()<RGBSemanticsInstances>();
            case DataMode::Uninitialized:
                VXL_ERROR("Tried to auto-template a function with an uninitialized enum!");
        };
    }

    #define AUTO_TEMPLATE(mode, ...) TemplateFromEnum(mode, [&]<typename DataT>(){__VA_ARGS__;})
    #define AUTO_TEMPLATE_INSTANCES_ONLY(mode, ...) TemplateFromEnumInstanceOnly(mode, [&]<typename DataT>(){__VA_ARGS__;})
    #define AUTO_TEMPLATE_SEMANTICS_ONLY(mode, ...) TemplateFromEnumSemanticsOnly(mode, [&]<typename DataT>(){__VA_ARGS__;})
}  // namespace 