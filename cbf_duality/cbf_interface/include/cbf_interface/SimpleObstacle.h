//
// Created by qiayuan on 2022/7/16.
//
#pragma once

#include "cbf_interface/FactoryAccessHelper.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/constraint/StateInputConstraintCppAd.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

namespace cbf {
using namespace ocs2;

class ObstacleSimple : public SolverSynchronizedModule {
 public:
  ObstacleSimple(const CentroidalModelInfo& info, const vector_t& pos, scalar_t radius) : info_(info) {
    params_ = vector_t(6);
    params_.head(2) = pos;
    params_(2) = radius;
  };
  vector_t getParameters() const { return params_; }
  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override {
    params_(3) = initTime;

    vector_t points(4);
    points << 0.12, 0., -0.2, 0.;
    points = pointsRobot2World(currentState, info_, points);
    for (size_t i = 0; i < 2; ++i) params_(i + 4) = (params_.segment<2>(0) - points.segment<2>(i * 2)).norm() - params_(2) - 0.35 / 2.;
  }
  void postSolverRun(const PrimalSolution& primalSolution) override{};

 private:
  vector_t params_;
  const CentroidalModelInfo info_;
};

class DCbfSimpleAd final : public StateInputConstraintCppAd {
 public:
  DCbfSimpleAd(const ObstacleSimple& distance, const CentroidalModelInfo& info)
      : StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear), distance_(distance), info_(info) {
    initialize(info.stateDim, info.inputDim, 6, "DCbfObstacle", "/tmp/ocs2", true, false);
  }
  vector_t getParameters(scalar_t time, const PreComputation& /* preComputation */) const override { return distance_.getParameters(); }
  DCbfSimpleAd* clone() const override { return new DCbfSimpleAd(*this); }
  size_t getNumConstraints(ocs2::scalar_t time) const override { return 2; }
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override {
    scalar_t decay_rate = 1.0;
    ad_vector_t constraint(2);

    ad_vector_t points(4);
    points << ad_scalar_t(0.12), ad_scalar_t(0.), ad_scalar_t(-0.2), ad_scalar_t(0.);
    points = pointsRobot2World(state, info_, points);
    for (size_t i = 0; i < 2; ++i)
      constraint(i) = (parameters.head(2) - points.segment(i * 2, 2)).norm() - parameters(2) - 0.35 / 2. -
                      exp(-decay_rate * (time - parameters(3))) * parameters(i + 4);

    return constraint;
  }

 private:
  DCbfSimpleAd(const DCbfSimpleAd& rhs) : StateInputConstraintCppAd(rhs), distance_(rhs.distance_), info_(rhs.info_) {}

  const CentroidalModelInfo info_;
  const ObstacleSimple& distance_;
};

}  // namespace cbf
