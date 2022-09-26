//
// Created by qiayuan on 2022/7/16.
//

#include "cbf_interface/legged_interface.h"
#include "cbf_interface/factory_access_helper.h"
#include "cbf_interface/duality.h"
#include "cbf_interface/initializer.h"

#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>

namespace cbf
{
void DCbfSimpleLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                           const std::string& referenceFile, bool verbose)
{
  LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  vector_t obstacle_pos(2);
  obstacle_pos << 0.6, 0.8;
  obstacles_.push_back(std::make_shared<ObstacleSimple>(centroidalModelInfo_, obstacle_pos, 0.353));

  for (int i = 0; i < obstacles_.size(); ++i)
  {
    std::unique_ptr<DCbfSimpleAd> cbf_ptr(new DCbfSimpleAd(*obstacles_[i], centroidalModelInfo_));
    std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(0.1, 1e-3)));
    problemPtr_->softConstraintPtr->add(
        "obstacle_" + std::to_string(i),
        std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(cbf_ptr), std::move(penalty))));
  }
}

void DCbfSimpleLeggedInterface::setupModel(const std::string& taskFile, const std::string& urdfFile,
                                           const std::string& referenceFile, bool verbose)
{
  LeggedInterface::setupModel(taskFile, urdfFile, referenceFile, verbose);
}

void DualityLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                        const std::string& referenceFile, bool verbose)
{
  LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  setupObstacles();
  //  std::unique_ptr<PenaltyBase> penalty_lag(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(5.0, 1e-2)));
  std::unique_ptr<PenaltyBase> penalty_lag(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(0.01, 1e-3)));
  std::unique_ptr<PenaltyBase> penalty_positive(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(0.1, 1e-3)));

  std::unique_ptr<DualityLagrangianAd> lagrangian(new DualityLagrangianAd(*obstacles_, duality_info_));
  problemPtr_->softConstraintPtr->add(
      "duality_lagrangian",
      std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(lagrangian), std::move(penalty_lag))));

  std::unique_ptr<DualityPositiveAd> positive(new DualityPositiveAd(*obstacles_, duality_info_));
  problemPtr_->softConstraintPtr->add("duality_positive", std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(
                                                              std::move(positive), std::move(penalty_positive))));

  std::unique_ptr<DualityVectorAd> equality(new DualityVectorAd(*obstacles_, duality_info_));
  problemPtr_->equalityConstraintPtr->add("duality_vector", std::move(equality));

  std::unique_ptr<DualityNormAd> norm(new DualityNormAd(*obstacles_, duality_info_));
  problemPtr_->equalityConstraintPtr->add("duality_norm", std::move(norm));

  initializerPtr_.reset(new DualityInitializer(duality_info_, *referenceManagerPtr_, *obstacles_));
}

void DualityLeggedInterface::setupModel(const std::string& taskFile, const std::string& urdfFile,
                                        const std::string& referenceFile, bool verbose)
{
  LeggedInterface::setupModel(taskFile, urdfFile, referenceFile, verbose);

  duality_info_ = createDualityInfo(centroidalModelInfo_, 4, 15);
  centroidalModelInfo_ = duality_info_.centroidal_info;
}

void DCbfDualityLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                            const std::string& referenceFile, bool verbose)
{
  DualityLeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
  problemPtr_->softConstraintPtr->erase("duality_lagrangian");

  std::unique_ptr<PenaltyBase> penalty_lag(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(0.01, 1e-3)));
  std::unique_ptr<DCbfLagrangianAd> lagrangian(new DCbfLagrangianAd(*obstacles_, duality_info_));
  problemPtr_->softConstraintPtr->add("dcbf_lagrangian", std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(
                                                             std::move(lagrangian), std::move(penalty_lag))));
}

}  // namespace cbf
