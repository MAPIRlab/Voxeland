#pragma once

#include "bonxai/bonxai.hpp"
#include <eigen3/Eigen/Geometry>
#include <unordered_set>

namespace Bonxai
{


template <class Functor>
void RayIterator(const CoordT& key_origin,
                 const CoordT& key_end,
                 const Functor& func);

inline void ComputeRay(const CoordT& key_origin,
                       const CoordT& key_end,
                       std::vector<CoordT>& ray)
{
  ray.clear();
  RayIterator(key_origin, key_end, [&ray](const CoordT& coord) {
    ray.push_back(coord);
    return true;
  });
}


/// Compute the logds, but return the result as an integer,
/// The real number is represented as a fixed precision
/// integer (6 decimals after the comma)
[[nodiscard]] static constexpr int32_t logodds(float prob)
{
return int32_t(1e6 * std::log(prob / (1.0 - prob)));
}

/// Expect the fixed comma value returned by logodds()
[[nodiscard]] static constexpr float prob(int32_t logodds_fixed)
{
float logodds = float(logodds_fixed) * 1e-6;
return (1.0 - 1.0 / (1.0 + std::exp(logodds)));
}

class ProbabilisticMap
{
public:
  using Vector3D = Eigen::Vector3d;


  static constexpr int32_t UnknownProbability = logodds(0.5f);
  /// These default values are the same as OctoMap
  struct Options
  {
    int32_t prob_miss_log = logodds(0.4f);
    int32_t prob_hit_log = logodds(0.7f);

    int32_t clamp_min_log = logodds(0.12f);
    int32_t clamp_max_log = logodds(0.97f);

    int32_t occupancy_threshold_log = logodds(0.5);
  };

  [[nodiscard]] const Options& options() const
  {
    return _options;
  }

  void setOptions(const Options& options) 
  {
    _options = options;
  }


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
  template <typename PointT, typename DataT>
  void insertPointCloud(const std::vector<PointT>& points,
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
  template <typename DataT>
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
  template <typename DataT>
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

  template <typename DataT>
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
  
  template <typename PointT, typename DataT>
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

protected:
  virtual void addMissPoint(const Vector3D& point) = 0;
  virtual void addHitPoint(const Vector3D& point) = 0;
  virtual void updateFreeCells(const Vector3D& origin) = 0;
  virtual Point3D coordToPos(CoordT coord) = 0;

  Options _options;
};

//--------------------------------------------------

template <class Functor>
inline void RayIterator(const CoordT& key_origin,
                        const CoordT& key_end,
                        const Functor& func)
{
  if (key_origin == key_end)
  {
    return;
  }
  if (!func(key_origin))
  {
    return;
  }

  CoordT error = { 0, 0, 0 };
  CoordT coord = key_origin;
  CoordT delta = (key_end - coord);
  const CoordT step = { delta.x < 0 ? -1 : 1,
                        delta.y < 0 ? -1 : 1,
                        delta.z < 0 ? -1 : 1 };

  delta = { delta.x < 0 ? -delta.x : delta.x,
            delta.y < 0 ? -delta.y : delta.y,
            delta.z < 0 ? -delta.z : delta.z };

  const int max = std::max(std::max(delta.x, delta.y), delta.z);

  // maximum change of any coordinate
  for (int i = 0; i < max - 1; ++i)
  {
    // update errors
    error = error + delta;
    // manual loop unrolling
    if ((error.x << 1) >= max)
    {
      coord.x += step.x;
      error.x -= max;
    }
    if ((error.y << 1) >= max)
    {
      coord.y += step.y;
      error.y -= max;
    }
    if ((error.z << 1) >= max)
    {
      coord.z += step.z;
      error.z -= max;
    }
    if (!func(coord))
    {
      return;
    }
  }
}

}  // namespace Bonxai
