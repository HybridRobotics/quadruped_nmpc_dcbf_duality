//
// Created by qiayuan on 2022/8/9.
//

#pragma once

#include "cbf_interface/FactoryAccessHelper.h"

#include <cbf_geometry/ConvexRegion2d.h>

namespace cbf {
class DualityObstacles {
 public:
  DualityObstacles(const DualityInfo& info) : info_(info) {
    for (size_t i = 0; i < info_.numObstacles; ++i) vertex_.push_back(Vertex2d<scalar_t>(info_.numPoints, vector_t()));
  }

  void setPointsArray(const vector_array_t& points_array) {
    vertex_.clear();
    for (const auto& i : points_array) vertex_.push_back(Vertex2d<scalar_t>(info_.numPoints, i));
    for (size_t i = points_array.size(); i < info_.numObstacles; ++i) vertex_.push_back(Vertex2d<scalar_t>(info_.numPoints, vector_t()));
  }

  virtual size_t getParametersSize() const { return 3 * info_.numObstacles * info_.numPoints; }

  virtual vector_t getParameters() const {
    vector_t ret = vector_t::Zero(getParametersSize());
    for (size_t o = 0; o < info_.numObstacles; ++o) {
      for (int p = 0; p < info_.numPoints; ++p) ret.segment<2>(o * 3 * info_.numPoints + p * 2) = vertex_[o].getA().row(p);
      ret.segment(o * 3 * info_.numPoints + info_.numPoints * 2, info_.numPoints) = vertex_[o].getB();
    }

    return ret;
  }

  Vertex2d<scalar_t> getRegion(size_t i) const { return vertex_[i]; }

  DualityInfo getInfo() const { return info_; }

 protected:
  const DualityInfo& info_;
  std::vector<Vertex2d<scalar_t>> vertex_;
};

class CbfObstacles : public DualityObstacles {
 public:
  CbfObstacles(const DualityInfo& info) : DualityObstacles(info) {
    time_ = 0.;
    dists_ = vector_t::Zero(info_.numObstacles);
  }

  void setDists(scalar_t time, const vector_t& dists) {
    time_ = time;
    dists_ = dists;
  }

  size_t getParametersSize() const override { return DualityObstacles::getParametersSize() + 1 + info_.numObstacles; }

  vector_t getParameters() const override {
    vector_t ret = vector_t::Zero(3 * info_.numObstacles * info_.numPoints + 1 + info_.numObstacles);
    ret.head(3 * info_.numObstacles * info_.numPoints) = DualityObstacles::getParameters();
    ret(3 * info_.numObstacles * info_.numPoints) = time_;
    ret.tail(info_.numObstacles) = dists_;
    return ret;
  }

 private:
  scalar_t time_;
  vector_t dists_;
};

}  // namespace cbf
