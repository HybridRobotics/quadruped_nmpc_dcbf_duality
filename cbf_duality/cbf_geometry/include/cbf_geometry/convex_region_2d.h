//
// Created by qiayuan on 2022/7/25.
//

#pragma once
#include <ocs2_core/Types.h>

namespace cbf
{
using namespace ocs2;

template <typename SCALAR>
using vector_temp = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;

template <typename SCALAR>
using matrix_temp = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

template <typename SCALAR>
class ConvexRegion2d
{
public:
  explicit ConvexRegion2d(size_t num_points)
  {
    this->a_ = matrix_temp<SCALAR>::Zero(num_points, 2);
    this->b_ = vector_temp<SCALAR>::Zero(num_points);
  }
  matrix_temp<SCALAR> getA() const
  {
    return a_;
  }
  vector_temp<SCALAR> getB() const
  {
    return b_;
  }

protected:
  matrix_temp<SCALAR> a_;
  vector_temp<SCALAR> b_;
};

template <typename SCALAR>
class Rectangle2d : public ConvexRegion2d<SCALAR>
{
public:
  Rectangle2d(const vector_temp<SCALAR>& pose, const vector_temp<SCALAR>& size);
};

template <typename SCALAR>
class Vertex2d : public ConvexRegion2d<SCALAR>
{
public:
  Vertex2d(size_t num_points, const vector_temp<SCALAR>& points);
};

}  // namespace cbf
