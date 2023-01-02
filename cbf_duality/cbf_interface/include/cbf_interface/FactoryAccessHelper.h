//
// Created by qiayuan on 2022/7/22.
//

#pragma once

#include <cbf_geometry/ConvexRegion2d.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace cbf {
using namespace ocs2;

struct DualityInfo {
  CentroidalModelInfo centroidalInfo;
  size_t numObstacles{};
  size_t numPoints{};
};

DualityInfo createDualityInfo(const CentroidalModelInfo& info, size_t numObstacles, size_t numPoints);

template <typename Derived>
Eigen::Block<Derived, -1, 1> getLambda2D(Eigen::MatrixBase<Derived>& input, const DualityInfo& info) {
  const size_t startRow =
      info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts + 6 * info.centroidalInfo.numSixDofContacts;
  return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, info.numObstacles * 4 + info.numObstacles * info.numPoints, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getLambda2D(const Eigen::MatrixBase<Derived>& input, const DualityInfo& info) {
  const size_t startRow =
      info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts + 6 * info.centroidalInfo.numSixDofContacts;
  return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, info.numObstacles * 4 + info.numObstacles * info.numPoints, 1);
}

template <typename Derived>
Eigen::Block<Derived, -1, 1> getObstacleLambda(Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o) {
  const size_t startRow = info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts +
                          6 * info.centroidalInfo.numSixDofContacts + o * (4 + info.numPoints) + 4;
  return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, info.numPoints, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getObstacleLambda(const Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o) {
  const size_t startRow = info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts +
                          6 * info.centroidalInfo.numSixDofContacts + o * (4 + info.numPoints) + 4;
  return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, info.numPoints, 1);
}

template <typename Derived>
Eigen::Block<Derived, -1, 1> getRobotLambda(Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o) {
  const size_t startRow = info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts +
                          6 * info.centroidalInfo.numSixDofContacts + o * (4 + info.numPoints);
  return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, 4, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getRobotLambda(const Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o) {
  //  const size_t startRow = o * (4 + info.numPoints);
  const size_t startRow = info.centroidalInfo.actuatedDofNum + 3 * info.centroidalInfo.numThreeDofContacts +
                          6 * info.centroidalInfo.numSixDofContacts + o * (4 + info.numPoints);
  return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, 4, 1);
}

template <typename SCALAR>
vector_temp<SCALAR> pointsRobot2World(const vector_temp<SCALAR>& state, const CentroidalModelInfo& info, vector_temp<SCALAR> points) {
  vector_temp<SCALAR> basePose = centroidal_model::getBasePose(state, info);
  matrix_temp<SCALAR> rot(2, 2);
  rot << cos(basePose(3)), -sin(basePose(3)), sin(basePose(3)), cos(basePose(3));
  vector_temp<SCALAR> ret(points.size());
  for (size_t i = 0; i < points.size() / 2; ++i) {
    ret.segment(i * 2, 2) = rot * points.segment(i * 2, 2) + basePose.head(2);
  }
  return ret;
}

template <typename SCALAR>
Vertex2d<SCALAR> createRobotRegion(const vector_temp<SCALAR>& state, const CentroidalModelInfo& info) {
  vector_temp<SCALAR> points(8);
  points << SCALAR(0.24), SCALAR(0.16), SCALAR(0.24), SCALAR(-0.16), SCALAR(-0.35), SCALAR(-0.16), SCALAR(-0.35), SCALAR(0.16);
  matrix_temp<SCALAR> rot(2, 2);
  return Vertex2d<SCALAR>(4, pointsRobot2World(state, info, points));
}

}  // namespace cbf
