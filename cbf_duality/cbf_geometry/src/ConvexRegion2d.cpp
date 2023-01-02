//
// Created by qiayuan on 2022/7/25.
//

#include "cbf_geometry/ConvexRegion2d.h"

#include <ocs2_core/automatic_differentiation/Types.h>
#include <qpOASES.hpp>

namespace cbf {
using namespace ocs2;

template <typename SCALAR>
Rectangle2d<SCALAR>::Rectangle2d(const vector_temp<SCALAR>& pose, const vector_temp<SCALAR>& size) : ConvexRegion2d<SCALAR>(4) {
  matrix_temp<SCALAR> rot(2, 2);
  rot << cos(-pose(2)), -sin(-pose(2)), sin(-pose(2)), cos(-pose(2));
  this->a_ << SCALAR(1), SCALAR(0), SCALAR(-1), SCALAR(0), SCALAR(0), SCALAR(1), SCALAR(0), SCALAR(-1);
  this->a_ = this->a_ * rot;
  vector_temp<SCALAR> pos = rot * pose.head(2);
  this->b_ << pos(0) + size(0) / 2, -(pos(0) - size(0) / 2), pos(1) + size(1) / 2, -(pos(1) - size(1) / 2);
}

template <typename SCALAR>
Vertex2d<SCALAR>::Vertex2d(size_t num_points, const vector_temp<SCALAR>& points) : ConvexRegion2d<SCALAR>(num_points) {
  size_t size_points = points.size() / 2;  // The numPoints actually greater or equal the size of points/2
  for (size_t i = 0; i < size_points; i++) {
    size_t j = i + 1;
    if (j == size_points) j = 0;
    size_t k = j + 1;
    if (k == size_points) k = 0;
    vector_temp<SCALAR> point_a = points.segment(i * 2, 2);
    vector_temp<SCALAR> point_b = points.segment(j * 2, 2);
    vector_temp<SCALAR> point_c = points.segment(k * 2, 2);

    this->a_.row(i) << point_b.y() - point_a.y(), point_a.x() - point_b.x();
    this->b_(i) = point_a.x() * point_b.y() - point_a.y() * point_b.x();

    if (typeid(SCALAR) == typeid(ad_scalar_t)) {
      SCALAR scalar;
      scalar = CppAD::CondExpGt((this->a_.row(i) * point_c)(0), this->b_(i), SCALAR(-1), SCALAR(1));
      this->a_.row(i) *= scalar;
      this->b_(i) *= scalar;
    } else {
      if ((this->a_.row(i) * point_c)(0) > this->b_(i)) {
        this->a_.row(i) *= SCALAR(-1);
        this->b_(i) *= SCALAR(-1);
      }
    }
  }
}

// explicit template instantiation
template class ConvexRegion2d<scalar_t>;
template class ConvexRegion2d<ad_scalar_t>;
template class Rectangle2d<scalar_t>;
template class Rectangle2d<ad_scalar_t>;
template class Vertex2d<scalar_t>;
template class Vertex2d<ad_scalar_t>;

}  // namespace cbf
