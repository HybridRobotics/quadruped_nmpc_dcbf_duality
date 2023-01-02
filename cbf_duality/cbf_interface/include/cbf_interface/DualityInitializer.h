//
// Created by qiayuan on 2022/8/10.
//

#pragma once

#include <ocs2_legged_robot/common/utils.h>

#include <cbf_geometry/DistanceQp2d.h>

namespace cbf {
using namespace ocs2;
using namespace legged_robot;

class DualityInitializer : public Initializer {
 public:
  DualityInitializer(DualityInfo info, const SwitchedModelReferenceManager& referenceManager, const DualityObstacles& obstacles);
  ~DualityInitializer() override = default;
  DualityInitializer* clone() const override;

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 private:
  DualityInitializer(const DualityInitializer& other) = default;
  const DualityInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const DualityObstacles& obstacles_;

  scalar_t lastTime_{};
  vector_t lastState_;
  std::vector<std::shared_ptr<Duality2d> > qp_;
};

DualityInitializer::DualityInitializer(DualityInfo info, const SwitchedModelReferenceManager& referenceManager,
                                       const DualityObstacles& obstacles)
    : info_(std::move(info)), referenceManagerPtr_(&referenceManager), obstacles_(obstacles), qp_(info_.numObstacles) {
  lastState_ = vector_t::Zero(info_.centroidalInfo.stateDim);
}

DualityInitializer* DualityInitializer::clone() const {
  return new DualityInitializer(*this);
}

void DualityInitializer::compute(scalar_t time, const vector_t& state, scalar_t /*nextTime*/, vector_t& input, vector_t& nextState) {
  nextState = state;

  const auto contact_flags = referenceManagerPtr_->getContactFlags(time);
  input = weightCompensatingInput(info_.centroidalInfo, contact_flags);

  auto robot = createRobotRegion(state, info_.centroidalInfo);

  bool need_update = false;
  for (size_t o = 0; o < info_.numObstacles; o++) {
    if (qp_[o] == nullptr) {
      need_update = true;
    }
  }
  if (lastTime_ != time && !lastState_.isApprox(state)) {
    need_update = true;
  }
  if (need_update) {
    for (size_t o = 0; o < info_.numObstacles; o++) {
      auto obstacle = obstacles_.getRegion(o);
      qp_[o] = std::make_shared<Duality2d>(robot, obstacle);
    }
  }

  getLambda2D(input, info_).setZero();
  for (size_t o = 0; o < info_.numObstacles; o++) {
    double dist = qp_[o]->getDistance();
    if (!isnan(dist) && dist > 1e-3) {
      vector_t lambda = (0.5 / dist) * qp_[o]->getSolution();
      getRobotLambda(input, info_, o) = lambda.head(4);
      getObstacleLambda(input, info_, o) = lambda.tail(info_.numPoints);
    }
  }
}

}  // namespace cbf
