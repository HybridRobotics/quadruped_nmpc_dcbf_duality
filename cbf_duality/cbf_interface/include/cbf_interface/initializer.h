//
// Created by qiayuan on 2022/8/10.
//

#pragma once

#include <ocs2_legged_robot/common/utils.h>

#include <cbf_geometry/distance_qp_2d.h>

namespace cbf
{
using namespace ocs2;
using namespace legged_robot;

class DualityInitializer : public Initializer
{
public:
  DualityInitializer(DualityInfo info, const SwitchedModelReferenceManager& reference_manager,
                     const DualityObstacles& obstacles);
  ~DualityInitializer() override = default;
  DualityInitializer* clone() const override;

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

private:
  DualityInitializer(const DualityInitializer& other) = default;
  const DualityInfo info_;
  const SwitchedModelReferenceManager* reference_manager_ptr_;
  const DualityObstacles& obstacles_;

  scalar_t last_time_;
  vector_t last_state_;
  std::vector<std::shared_ptr<Duality2d> > qp_;
};

DualityInitializer::DualityInitializer(DualityInfo info, const SwitchedModelReferenceManager& reference_manager,
                                       const DualityObstacles& obstacles)
  : info_(std::move(info)), reference_manager_ptr_(&reference_manager), obstacles_(obstacles), qp_(info_.num_obstacles)
{
  last_state_ = vector_t::Zero(info_.centroidal_info.stateDim);
}

DualityInitializer* DualityInitializer::clone() const
{
  return new DualityInitializer(*this);
}

void DualityInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input,
                                 vector_t& nextState)
{
  nextState = state;

  const auto contact_flags = reference_manager_ptr_->getContactFlags(time);
  input = weightCompensatingInput(info_.centroidal_info, contact_flags);

  auto robot = createRobotRegion(state, info_.centroidal_info);

  bool need_update = false;
  for (size_t o = 0; o < info_.num_obstacles; o++)
    if (qp_[o] == nullptr)
      need_update = true;
  if (last_time_ != time && !last_state_.isApprox(state))
    need_update = true;
  if (need_update)
    for (size_t o = 0; o < info_.num_obstacles; o++)
    {
      auto obstacle = obstacles_.getRegion(o);
      qp_[o] = std::make_shared<Duality2d>(robot, obstacle);
    }

  getLambda2D(input, info_).setZero();
  for (size_t o = 0; o < info_.num_obstacles; o++)
  {
    double dist = qp_[o]->getDistance();
    if (!isnan(dist) && dist > 1e-3)
    {
      vector_t lambda = (0.5 / dist) * qp_[o]->getSolution();
      getRobotLambda(input, info_, o) = lambda.head(4);
      getObstacleLambda(input, info_, o) = lambda.tail(info_.num_points);
    }
  }
}

}  // namespace cbf
