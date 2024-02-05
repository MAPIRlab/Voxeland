#pragma once

#include "bonxai/bonxai.hpp"
#include <eigen3/Eigen/Geometry>
#include <unordered_set>

namespace Bonxai
{

template <typename DataT>
class ProbabilisticMapT;

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

  template <typename DataT>
  ProbabilisticMapT<DataT>* With()
  {
    return dynamic_cast<ProbabilisticMapT<DataT>*>(this);
  }
protected:
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
