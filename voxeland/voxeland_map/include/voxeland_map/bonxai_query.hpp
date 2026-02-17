#pragma once
#include "voxeland_map/probabilistic_map_templated.hpp"

namespace VoxelandMap
{
    inline Bonxai::ProbabilisticMap* g_bonxai = nullptr;
}

// Provides a global way to access the bonxai object
template <typename DataT>
class BonxaiQuery
{
public:
    static void createAccessor(Bonxai::ProbabilisticMapT<DataT>* _bonxai);

    // Note that, before calling this function, the accessor object has to be created beforehand.
    static typename Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>::Accessor& getAccessor();

    static Bonxai::ProbabilisticMap* getBonxai() { return bonxai; }
    static Bonxai::ProbabilisticMapT<DataT>* getBonxaiT() { return bonxai; }

private:
    inline static Bonxai::ProbabilisticMapT<DataT>* bonxai;
    inline static thread_local std::optional<typename Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>::Accessor> accessor;
};

template <typename DataT>
inline Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>::Accessor& BonxaiQuery<DataT>::getAccessor()
{
    // accessor was created for another thread, we need a new one
    if (!accessor && bonxai)
        createAccessor(bonxai);

    return *accessor;
}

template <typename DataT>
inline void BonxaiQuery<DataT>::createAccessor(Bonxai::ProbabilisticMapT<DataT>* _bonxai)
{
    bonxai = _bonxai;
    VoxelandMap::g_bonxai = bonxai;
    accessor.emplace(_bonxai->grid()->createAccessor());
}
