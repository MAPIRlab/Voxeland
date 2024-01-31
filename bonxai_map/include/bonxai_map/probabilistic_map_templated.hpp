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
class ProbabilisticMapT : public ProbabilisticMap
{
public:
  using Vector3D = Eigen::Vector3d;
  ProbabilisticMapT(double resolution)
    : _grid(resolution)
    , _accessor(_grid.createAccessor())
  {}

  [[nodiscard]] VoxelGrid<ProbabilisticCell<DataT>>& grid()
  {
    return _grid;
  }

  [[nodiscard]] const VoxelGrid<ProbabilisticCell<DataT>>& grid() const
  {
    return _grid;
  }


  [[nodiscard]] bool isOccupied(const Bonxai::CoordT& coord) const
  {
    if(auto* cell = _accessor.value(coord, false))
    {
      return cell->probability_log > _options.occupancy_threshold_log;
    }
    return false;
  }

  [[nodiscard]] bool isUnknown(const Bonxai::CoordT& coord) const
  {
    if(auto* cell = _accessor.value(coord, false))
    {
      return cell->probability_log == _options.occupancy_threshold_log;
    }
    return true;
  }

  [[nodiscard]] bool isFree(const Bonxai::CoordT& coord) const
  {
    if(auto* cell = _accessor.value(coord, false))
    {
      return cell->probability_log < _options.occupancy_threshold_log;
    }
    return false;
  }

  void getFreeVoxels(std::vector<Bonxai::CoordT>& coords)
  {
    coords.clear();
    auto visitor = [&](ProbabilisticCell<DataT>& cell, const CoordT& coord) {
      if (cell.probability_log < _options.occupancy_threshold_log)
      {
        coords.push_back(coord);
      }
    };
    _grid.forEachCell(visitor);
  }

private:
  VoxelGrid<ProbabilisticCell<DataT>> _grid;
  uint8_t _update_count = 1;

  std::vector<CoordT> _miss_coords;
  std::vector<CoordT> _hit_coords;

  mutable typename Bonxai::VoxelGrid<ProbabilisticCell<DataT>>::Accessor _accessor;

  void updateFreeCells(const Vector3D& origin) override
  {
    auto accessor = _grid.createAccessor();

    // same as addMissPoint, but using lambda will force inlining
    auto clearPoint = [this, &accessor](const CoordT& coord)
    {
      ProbabilisticCell<DataT>* cell = accessor.value(coord, true);
      if (cell->update_id != _update_count)
      {
        cell->probability_log = std::max(
            cell->probability_log + _options.prob_miss_log, _options.clamp_min_log);
        cell->update_id = _update_count;
      }
      return true;
    };

    const auto coord_origin = _grid.posToCoord(origin);

    for (const auto& coord_end : _hit_coords)
    {
      RayIterator(coord_origin, coord_end, clearPoint);
    }
    _hit_coords.clear();

    for (const auto& coord_end : _miss_coords)
    {
      RayIterator(coord_origin, coord_end, clearPoint);
    }
    _miss_coords.clear();

    if (++_update_count == 4)
    {
      _update_count = 1;
    }
  }

  Point3D coordToPos(CoordT coord) override
  {
    return _grid.coordToPos(coord);
  }
};

//--------------------------------------------------

}  // namespace Bonxai
