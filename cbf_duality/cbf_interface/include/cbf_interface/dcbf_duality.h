//
// Created by qiayuan on 2022/8/20.
//
#pragma once
#include "cbf_interface/duality.h"

namespace cbf
{
using namespace ocs2;

class DCbfLagrangianAd : public DualityBaseAd
{
public:
  DCbfLagrangianAd(const DualityObstacles& obstacles, const DualityInfo& info) : DualityBaseAd(obstacles, info)
  {
    initialize(info.centroidal_info.stateDim, info.centroidal_info.inputDim, obstacles.getParametersSize(),
               "DCbfLagrangian", "/tmp/ocs2", true, false);
  }
  size_t getNumConstraints(ocs2::scalar_t time) const override
  {
    return info_.num_obstacles;
  }
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override
  {
    scalar_t decay_rate = 1.0;

    auto robot = createRobotRegion(state, info_.centroidal_info);
    ad_vector_t constraint(info_.num_obstacles);

    for (size_t o = 0; o < info_.num_obstacles; ++o)
      constraint(o) = (-getRobotLambda(input, info_, o).transpose() * robot.getB() -
                       getObstacleLambda(input, info_, o).transpose() * getObstacleB(parameters, o))(0) -
                      (0.05 + exp(-decay_rate * (time - getObstacleTime(parameters))) * getObstacleDist(parameters, o));

    return constraint;
  }
  DCbfLagrangianAd* clone() const override
  {
    return new DCbfLagrangianAd(*this);
  }

protected:
  ad_scalar_t getObstacleTime(const ad_vector_t& parameters) const
  {
    return parameters(3 * info_.num_obstacles * info_.num_points);
  }

  ad_scalar_t getObstacleDist(const ad_vector_t& parameters, size_t o) const
  {
    return parameters(3 * info_.num_obstacles * info_.num_points + 1 + o);
  }

private:
  DCbfLagrangianAd(const DualityObstacles& obstacles, const DCbfLagrangianAd& rhs) : DCbfLagrangianAd(rhs)
  {
  }
};

}  // namespace cbf
