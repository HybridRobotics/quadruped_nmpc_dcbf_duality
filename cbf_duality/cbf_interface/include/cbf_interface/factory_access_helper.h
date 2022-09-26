//
// Created by qiayuan on 2022/7/22.
//

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <cbf_geometry/convex_region_2d.h>

namespace cbf
{
using namespace ocs2;

struct DualityInfo
{
  CentroidalModelInfo centroidal_info;
  size_t num_obstacles;
  size_t num_points;
};

DualityInfo createDualityInfo(const CentroidalModelInfo& info, size_t num_obstacles, size_t num_points);

template <typename Derived>
Eigen::Block<Derived, -1, 1> getLambda2D(Eigen::MatrixBase<Derived>& input, const DualityInfo& info)
{
  const size_t start_row = info.centroidal_info.actuatedDofNum + 3 * info.centroidal_info.numThreeDofContacts +
                           6 * info.centroidal_info.numSixDofContacts;
  return Eigen::Block<Derived, -1, 1>(input.derived(), start_row, 0,
                                      info.num_obstacles * 4 + info.num_obstacles * info.num_points, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getLambda2D(const Eigen::MatrixBase<Derived>& input, const DualityInfo& info)
{
  const size_t start_row = info.centroidal_info.actuatedDofNum + 3 * info.centroidal_info.numThreeDofContacts +
                           6 * info.centroidal_info.numSixDofContacts;
  return Eigen::Block<const Derived, -1, 1>(input.derived(), start_row, 0,
                                            info.num_obstacles * 4 + info.num_obstacles * info.num_points, 1);
}

template <typename Derived>
Eigen::Block<Derived, -1, 1> getObstacleLambda(Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o)
{
  const size_t start_row = info.centroidal_info.actuatedDofNum + 3 * info.centroidal_info.numThreeDofContacts +
                           6 * info.centroidal_info.numSixDofContacts + o * (4 + info.num_points) + 4;
  return Eigen::Block<Derived, -1, 1>(input.derived(), start_row, 0, info.num_points, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getObstacleLambda(const Eigen::MatrixBase<Derived>& input,
                                                           const DualityInfo& info, size_t o)
{
  const size_t start_row = info.centroidal_info.actuatedDofNum + 3 * info.centroidal_info.numThreeDofContacts +
                           6 * info.centroidal_info.numSixDofContacts + o * (4 + info.num_points) + 4;
  return Eigen::Block<const Derived, -1, 1>(input.derived(), start_row, 0, info.num_points, 1);
}

template <typename Derived>
Eigen::Block<Derived, -1, 1> getRobotLambda(Eigen::MatrixBase<Derived>& input, const DualityInfo& info, size_t o)
{
  const size_t start_row = info.centroidal_info.actuatedDofNum + 3 * info.centroidal_info.numThreeDofContacts +
                           6 * info.centroidal_info.numSixDofContacts + o * (4 + info.num_points);
  return Eigen::Block<Derived, -1, 1>(input.derived(), start_row, 0, 4, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getRobotLambda(const Eigen::MatrixBase<Derived>& input,
                                                        const DualityInfo& info, size_t o)
{
  //  const size_t start_row = o * (4 + info.num_points);
  const size_t start_row = info.centroidal_info.actuatedDofNum + 3 * info.centroidal_info.numThreeDofContacts +
                           6 * info.centroidal_info.numSixDofContacts + o * (4 + info.num_points);
  return Eigen::Block<const Derived, -1, 1>(input.derived(), start_row, 0, 4, 1);
}

template <typename SCALAR>
vector_temp<SCALAR> pointsRobot2World(const vector_temp<SCALAR>& state, const CentroidalModelInfo& info,
                                      vector_temp<SCALAR> points)
{
  vector_temp<SCALAR> base_pose = centroidal_model::getBasePose(state, info);
  matrix_temp<SCALAR> rot(2, 2);
  rot << cos(base_pose(3)), -sin(base_pose(3)), sin(base_pose(3)), cos(base_pose(3));
  vector_temp<SCALAR> ret(points.size());
  for (size_t i = 0; i < points.size() / 2; ++i)
    ret.segment(i * 2, 2) = rot * points.segment(i * 2, 2) + base_pose.head(2);
  return ret;
}

template <typename SCALAR>
Vertex2d<SCALAR> createRobotRegion(const vector_temp<SCALAR>& state, const CentroidalModelInfo& info)
{
  vector_temp<SCALAR> points(8);
  points << SCALAR(0.24), SCALAR(0.16), SCALAR(0.24), SCALAR(-0.16), SCALAR(-0.35), SCALAR(-0.16), SCALAR(-0.35),
      SCALAR(0.16);
  matrix_temp<SCALAR> rot(2, 2);
  return Vertex2d<SCALAR>(4, pointsRobot2World(state, info, points));
}

}  // namespace cbf
