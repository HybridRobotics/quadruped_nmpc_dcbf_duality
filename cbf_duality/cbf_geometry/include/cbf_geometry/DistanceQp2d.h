//
// Created by qiayuan on 2022/8/8.
//
#pragma once
#include <memory>

#include <ocs2_core/Types.h>
#include <qpOASES.hpp>

#include "cbf_geometry/ConvexRegion2d.h"

namespace cbf {
class DistanceQp2d {
 public:
  DistanceQp2d(const ConvexRegion2d<scalar_t>& region0, ConvexRegion2d<scalar_t>& region1);
  scalar_t getDistance();
  vector_t getSolution();

 private:
  Eigen::Matrix<scalar_t, 4, 4, Eigen::RowMajor> h_;
  Eigen::Matrix<scalar_t, Eigen::Dynamic, 4, Eigen::RowMajor> c_;
  vector_t g_, ub_;

  vector_t sol_;
};

class Duality2d {
 public:
  Duality2d(const ConvexRegion2d<scalar_t>& region0, ConvexRegion2d<scalar_t>& region1);
  scalar_t getDistance();
  vector_t getSolution();

 private:
  void setupCost(const ConvexRegion2d<scalar_t>& region0, ConvexRegion2d<scalar_t>& region1);
  void setupConstraints(const ConvexRegion2d<scalar_t>& region0, ConvexRegion2d<scalar_t>& region1);

  std::shared_ptr<qpOASES::QProblem> qpProblem_;

  size_t numPointsRegion0_, numPointsRegion1_, numPoints_;

  Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> h_;
  Eigen::Matrix<scalar_t, 4, Eigen::Dynamic, Eigen::RowMajor> c_;
  vector_t g_, ub_, lb_, l_;
};

}  // namespace cbf
