//
// Created by qiayuan on 2022/8/9.
//

#pragma once

#include "cbf_interface/factory_access_helper.h"

#include <cbf_geometry/convex_region_2d.h>

namespace cbf
{
class DualityObstacles
{
public:
  DualityObstacles(const DualityInfo& info) : info_(info)
  {
    for (size_t i = 0; i < info_.num_obstacles; ++i)
      vertex_.push_back(Vertex2d<scalar_t>(info_.num_points, vector_t()));
  }

  void setPointsArray(const vector_array_t& points_array)
  {
    vertex_.clear();
    for (const auto& i : points_array)
      vertex_.push_back(Vertex2d<scalar_t>(info_.num_points, i));
    for (size_t i = points_array.size(); i < info_.num_obstacles; ++i)
      vertex_.push_back(Vertex2d<scalar_t>(info_.num_points, vector_t()));
  }

  virtual size_t getParametersSize() const
  {
    return 3 * info_.num_obstacles * info_.num_points;
  }

  virtual vector_t getParameters() const
  {
    vector_t ret = vector_t::Zero(getParametersSize());
    for (size_t o = 0; o < info_.num_obstacles; ++o)
    {
      for (int p = 0; p < info_.num_points; ++p)
        ret.segment<2>(o * 3 * info_.num_points + p * 2) = vertex_[o].getA().row(p);
      ret.segment(o * 3 * info_.num_points + info_.num_points * 2, info_.num_points) = vertex_[o].getB();
    }

    return ret;
  }

  Vertex2d<scalar_t> getRegion(size_t i) const
  {
    return vertex_[i];
  }

  DualityInfo getInfo() const
  {
    return info_;
  }

protected:
  const DualityInfo& info_;
  std::vector<Vertex2d<scalar_t>> vertex_;
};

class CbfObstacles : public DualityObstacles
{
public:
  CbfObstacles(const DualityInfo& info) : DualityObstacles(info)
  {
    time_ = 0.;
    dists_ = vector_t::Zero(info_.num_obstacles);
  }

  void setDists(scalar_t time, const vector_t& dists)
  {
    time_ = time;
    dists_ = dists;
  }

  size_t getParametersSize() const override
  {
    return DualityObstacles::getParametersSize() + 1 + info_.num_obstacles;
  }

  vector_t getParameters() const override
  {
    vector_t ret = vector_t::Zero(3 * info_.num_obstacles * info_.num_points + 1 + info_.num_obstacles);
    ret.head(3 * info_.num_obstacles * info_.num_points) = DualityObstacles::getParameters();
    ret(3 * info_.num_obstacles * info_.num_points) = time_;
    ret.tail(info_.num_obstacles) = dists_;
    return ret;
  }

private:
  scalar_t time_;
  vector_t dists_;
};

}  // namespace cbf
