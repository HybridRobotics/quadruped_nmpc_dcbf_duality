//
// Created by qiayuan on 2022/8/20.
//
#pragma once
#include <utility>

#include "cbf_interface/Duality.h"

namespace cbf {
using namespace ocs2;

class DCbfLagrangianAd : public DualityBaseAd {
 public:
  DCbfLagrangianAd(const DualityObstacles& obstacles, const DualityInfo& info) : DualityBaseAd(obstacles, info) {
    initialize(info.centroidalInfo.stateDim, info.centroidalInfo.inputDim, obstacles.getParametersSize(), "DCbfLagrangian", "/tmp/ocs2",
               true, false);
  }
  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override { return info_.numObstacles; }
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override {
    scalar_t decay_rate = 1.0;

    auto robot = createRobotRegion(state, info_.centroidalInfo);
    ad_vector_t constraint(info_.numObstacles);

    for (size_t o = 0; o < info_.numObstacles; ++o) {
      constraint(o) = (-getRobotLambda(input, info_, o).transpose() * robot.getB() -
                       getObstacleLambda(input, info_, o).transpose() * getObstacleB(parameters, o))(0) -
                      (0.05 + exp(-decay_rate * (time - getObstacleTime(parameters))) * getObstacleDist(parameters, o));
    }

    return constraint;
  }
  DCbfLagrangianAd* clone() const override { return new DCbfLagrangianAd(*this); }

 protected:
  ad_scalar_t getObstacleTime(const ad_vector_t& parameters) const { return parameters(3 * info_.numObstacles * info_.numPoints); }

  ad_scalar_t getObstacleDist(const ad_vector_t& parameters, size_t o) const {
    return parameters(3 * info_.numObstacles * info_.numPoints + 1 + o);
  }

 private:
  DCbfLagrangianAd(const DualityObstacles& obstacles, DCbfLagrangianAd rhs) : DCbfLagrangianAd(std::move(rhs)) {}
};

}  // namespace cbf
