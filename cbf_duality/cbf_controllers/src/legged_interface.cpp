//
// Created by qiayuan on 2022/7/16.
//

#include "cbf_controllers/cbf_controllers.h"

#include "cbf_interface/legged_interface.h"
#include "cbf_interface/obstacle_receiver.h"

#include <legged_controllers/legged_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace cbf
{
void DCbfSimpleController::setupMpc()
{
  LeggedController::setupMpc();
  auto obstacles = static_cast<DCbfSimpleLeggedInterface&>(*legged_interface_).getObstacles();
  for (auto& obstacle : obstacles)
    mpc_->getSolverPtr()->addSynchronizedModule(obstacle);
}

void DCbfSimpleController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                                const std::string& reference_file, bool verbose)
{
  legged_interface_ = std::make_shared<DCbfSimpleLeggedInterface>(task_file, urdf_file, reference_file, verbose);
  legged_interface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
}

void DualityController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                             const std::string& reference_file, bool verbose)
{
  legged_interface_ = std::make_shared<DualityLeggedInterface>(task_file, urdf_file, reference_file, verbose);
  legged_interface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
}

void DualityController::setupMpc()
{
  LeggedController::setupMpc();

  ros::NodeHandle nh;
  auto obstacle_receiver = std::make_shared<ObstacleReceiver>(
      nh, static_cast<DualityLeggedInterface&>(*legged_interface_).getObstacles(), mpc_->getSolverPtr());
  mpc_->getSolverPtr()->addSynchronizedModule(obstacle_receiver);
}

void DCbfDualityController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                                 const std::string& reference_file, bool verbose)
{
  legged_interface_ = std::make_shared<DCbfDualityLeggedInterface>(task_file, urdf_file, reference_file, verbose);
  legged_interface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
}

void DCbfDualityController::setupMpc()
{
  LeggedController::setupMpc();

  ros::NodeHandle nh;
  auto obstacle_receiver = std::make_shared<CbfObstaclesReceiver>(
      nh, static_cast<DCbfDualityLeggedInterface&>(*legged_interface_).getObstacles(), mpc_->getSolverPtr());
  mpc_->getSolverPtr()->addSynchronizedModule(obstacle_receiver);
}

}  // namespace cbf

PLUGINLIB_EXPORT_CLASS(cbf::DCbfSimpleController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cbf::DualityController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cbf::DCbfDualityController, controller_interface::ControllerBase)
