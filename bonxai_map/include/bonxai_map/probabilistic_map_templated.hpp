#pragma once

#include "probabilistic_map.hpp"
#include <bonxai_map/cell_types.hpp>

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

  /**
   * @brief insertPointCloud will update the probability map
   * with a new set of detections.
   * The template function can accept points of different types,
   * such as pcl:Point, Eigen::Vector or Bonxai::Point3d
   *
   * Both origin and points must be in world coordinates
   *
   * @param points   a vector of points which represent detected obstacles
   * @param origin   origin of the point cloud
   * @param max_range  max range of the ray, if exceeded, we will use that
   * to compute a free space
   */
  template <typename PointT, typename allocatorT>
  void insertPointCloud(const std::vector<PointT, allocatorT>& points,
                        const PointT& origin,
                        double max_range)
  {
    const auto from = ConvertPoint<Vector3D>(origin);
    const double max_range_sqr = max_range * max_range;
    for (const auto& point : points)
    {
      const auto to = ConvertPoint<Vector3D>(point);
      Vector3D vect(to - from);
      const double squared_norm = vect.squaredNorm();
      // points that exceed the max_range will create a cleaning ray
      if (squared_norm >= max_range_sqr)
      {
        // The new point will have distance == max_range from origin
        const Vector3D new_point =
            from + ((vect / std::sqrt(squared_norm)) * max_range);
        addMissPoint(new_point);
      }
      else
      {
        addHitPoint(to, DataT(point));
      }
    }
    updateFreeCells(from);
  }

  // This function is usually called by insertPointCloud
  // We expose it here to add more control to the user.
  // Once finished adding points, you must call updateFreeCells()
  void addHitPoint(const Vector3D& point, const DataT& data)
  {
    const auto coord = _grid.posToCoord(point);
    ProbabilisticCell<DataT>* cell = _accessor.value(coord, true);

    //TODO updating the data here should call a function in DataT that specifies how the information is to be fused, rather than just overwriting with the latest
    cell->data = data;

    if (cell->update_id != _update_count)
    {
      cell->probability_log = std::min(cell->probability_log + _options.prob_hit_log,
                                      _options.clamp_max_log);

      cell->update_id = _update_count;
      _hit_coords.push_back(coord);
    }
  }

  // This function is usually called by insertPointCloud
  // We expose it here to add more control to the user.
  // Once finished adding points, you must call updateFreeCells()
  void addMissPoint(const Vector3D& point)
  {
    const auto coord = _grid.posToCoord(point);
    ProbabilisticCell<DataT>* cell = _accessor.value(coord, true);

    if (cell->update_id != _update_count)
    {
      cell->probability_log = std::max(
          cell->probability_log + _options.prob_miss_log, _options.clamp_min_log);

      cell->update_id = _update_count;
      _miss_coords.push_back(coord);
    }
  }

  template <typename PointT>
  void getOccupiedVoxels(std::vector<PointT>& cells_points, std::vector<DataT>& cells_data)
  {
    std::vector<Bonxai::CoordT> coords;
    coords.clear();
    getOccupiedVoxels(coords, cells_data);
    for (const auto& coord : coords)
    {
      const auto p = coordToPos(coord);
      cells_points.emplace_back(p.x, p.y, p.z);
    }
  }

  void getOccupiedVoxels(std::vector<Bonxai::CoordT>& coords, std::vector<DataT>& cells_data)
  {
    coords.clear();
    auto visitor = [&](ProbabilisticCell<DataT>& cell, const CoordT& coord) {
      if (cell.probability_log > _options.occupancy_threshold_log)
      {
        coords.push_back(coord);
        cells_data.push_back(cell.data);
      }
    };
    _grid.forEachCell(visitor);
  }

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
