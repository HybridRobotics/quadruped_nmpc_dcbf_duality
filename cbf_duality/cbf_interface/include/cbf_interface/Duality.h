//
// Created by qiayuan on 2022/7/27.
//
#pragma once
#include "cbf_geometry/ConvexRegion2d.h"
#include "cbf_interface/FactoryAccessHelper.h"
#include "cbf_interface/Obstacle.h"

#include <cbf_geometry/ConvexRegion2d.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/constraint/StateInputConstraintCppAd.h>

namespace cbf {
using namespace ocs2;

class DualityBaseAd : public StateInputConstraintCppAd {
 public:
  DualityBaseAd(const DualityObstacles& obstacles, const DualityInfo& info)
      : StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear), obstacles_(obstacles), info_(info) {}

  vector_t getParameters(scalar_t time, const PreComputation& /* preComputation */) const override { return obstacles_.getParameters(); };

 protected:
  ad_matrix_t getObstacleA(const ad_vector_t& parameters, size_t o) const {
    ad_matrix_t ret(info_.numPoints, 2);
    for (int p = 0; p < info_.numPoints; ++p) ret.row(p) = parameters.segment<2>(o * 3 * info_.numPoints + p * 2);
    return ret;
  }

  ad_vector_t getObstacleB(const ad_vector_t& parameters, size_t o) const {
    return parameters.segment(o * 3 * info_.numPoints + info_.numPoints * 2, info_.numPoints);
  }

  DualityBaseAd(const DualityObstacles& obstacles, const DualityBaseAd& rhs)
      : StateInputConstraintCppAd(rhs), obstacles_(obstacles), info_(rhs.info_) {}

  const DualityObstacles& obstacles_;
  const DualityInfo info_;
};

class DualityLagrangianAd : public DualityBaseAd {
 public:
  DualityLagrangianAd(const DualityObstacles& obstacles, const DualityInfo& info) : DualityBaseAd(obstacles, info) {
    initialize(info.centroidalInfo.stateDim, info.centroidalInfo.inputDim, obstacles.getParametersSize(), "DualityLagrangian", "/tmp/ocs2",
               true, false);
  }
  size_t getNumConstraints(ocs2::scalar_t time) const override { return info_.numObstacles; }
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override {
    auto robot = createRobotRegion(state, info_.centroidalInfo);

    ad_vector_t constraint(info_.numObstacles);
    //    constraint(0) = (-ad_scalar_t(0.25) * lambda.tail(numPoints).transpose() * getObstacleA(parameters) *
    //                         getObstacleA(parameters).transpose() * lambda.tail(numPoints) -
    //                     lambda.head(4).transpose() * robot.getB() -
    //                     lambda.tail(numPoints).transpose() * getObstacleB(parameters))(0) -
    //                    0.05;

    for (size_t o = 0; o < info_.numObstacles; ++o)
      constraint(o) = (-getRobotLambda(input, info_, o).transpose() * robot.getB() -
                       getObstacleLambda(input, info_, o).transpose() * getObstacleB(parameters, o))(0) -
                      0.03;

    return constraint;
  }
  DualityLagrangianAd* clone() const override { return new DualityLagrangianAd(*this); }

 private:
  DualityLagrangianAd(const DualityObstacles& obstacles, const DualityLagrangianAd& rhs) : DualityBaseAd(rhs) {}
};

class DualityVectorAd : public DualityBaseAd {
 public:
  DualityVectorAd(const DualityObstacles& obstacles, const DualityInfo& info) : DualityBaseAd(obstacles, info) {
    initialize(info.centroidalInfo.stateDim, info.centroidalInfo.inputDim, obstacles.getParametersSize(), "DualityVector", "/tmp/ocs2",
               true, false);
  }
  size_t getNumConstraints(ocs2::scalar_t time) const override { return 2 * info_.numObstacles; }
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override {
    auto robot = createRobotRegion(state, info_.centroidalInfo);

    ad_vector_t constraint(2 * info_.numObstacles);

    for (size_t o = 0; o < info_.numObstacles; ++o)
      constraint.segment(o * 2, 2) = robot.getA().transpose() * getRobotLambda(input, info_, o) +
                                     getObstacleA(parameters, o).transpose() * getObstacleLambda(input, info_, o);

    return constraint;
  }
  DualityVectorAd* clone() const override { return new DualityVectorAd(*this); }

 private:
  DualityVectorAd(const DualityVectorAd& rhs) : DualityBaseAd(rhs) {}
};

class DualityPositiveAd : public DualityBaseAd {
 public:
  DualityPositiveAd(const DualityObstacles& obstacles, const DualityInfo& info) : DualityBaseAd(obstacles, info) {
    initialize(info.centroidalInfo.stateDim, info.centroidalInfo.inputDim, obstacles.getParametersSize(), "DualityPositive", "/tmp/ocs2",
               true, false);
  }
  size_t getNumConstraints(ocs2::scalar_t time) const override { return info_.numObstacles * 4 + info_.numObstacles * info_.numPoints; }
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override {
    return getLambda2D(input, info_);
  }
  DualityPositiveAd* clone() const override { return new DualityPositiveAd(*this); }

 private:
  DualityPositiveAd(const DualityPositiveAd& rhs) : DualityBaseAd(rhs) {}
};

class DualityNormAd : public DualityBaseAd {
 public:
  DualityNormAd(const DualityObstacles& obstacles, const DualityInfo& info) : DualityBaseAd(obstacles, info) {
    initialize(info.centroidalInfo.stateDim, info.centroidalInfo.inputDim, obstacles.getParametersSize(), "DualityNorm", "/tmp/ocs2", true,
               false);
  }
  size_t getNumConstraints(ocs2::scalar_t time) const override { return info_.numObstacles; }
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override {
    auto robot = createRobotRegion(state, info_.centroidalInfo);
    ad_vector_t constraint(info_.numObstacles);
    ad_vector_t lambda = getLambda2D(input, info_);
    for (size_t o = 0; o < info_.numObstacles; ++o)
      constraint(o) = -(getObstacleA(parameters, o).transpose() * getObstacleLambda(input, info_, o)).norm() + 1.;

    return constraint;
  }
  DualityNormAd* clone() const override { return new DualityNormAd(*this); }

 private:
  DualityNormAd(const DualityNormAd& rhs) : DualityBaseAd(rhs) {}
};

}  // namespace cbf
