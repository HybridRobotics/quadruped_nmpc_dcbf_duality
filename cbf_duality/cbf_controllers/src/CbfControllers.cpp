//
// Created by qiayuan on 2022/7/16.
//

#include "cbf_controllers/CbfControllers.h"

#include "cbf_interface/DCbfLeggedInterface.h"
#include "cbf_interface/ObstacleReceiver.h"

#include <pluginlib/class_list_macros.hpp>

namespace cbf {
void DCbfSimpleController::setupMpc() {
  LeggedController::setupMpc();
  auto obstacles = dynamic_cast<DCbfSimpleLeggedInterface&>(*leggedInterface_).getObstacles();
  for (auto& obstacle : obstacles) {
    mpc_->getSolverPtr()->addSynchronizedModule(obstacle);
  }
}

void DCbfSimpleController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                                const std::string& reference_file, bool verbose) {
  leggedInterface_ = std::make_shared<DCbfSimpleLeggedInterface>(task_file, urdf_file, reference_file, verbose);
  leggedInterface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
}

void DualityController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file, const std::string& reference_file,
                                             bool verbose) {
  leggedInterface_ = std::make_shared<DualityLeggedInterface>(task_file, urdf_file, reference_file, verbose);
  leggedInterface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
}

void DualityController::setupMpc() {
  LeggedController::setupMpc();

  ros::NodeHandle nh;
  auto obstacle_receiver =
      std::make_shared<ObstacleReceiver>(nh, dynamic_cast<DualityLeggedInterface&>(*leggedInterface_).getObstacles(), mpc_->getSolverPtr());
  mpc_->getSolverPtr()->addSynchronizedModule(obstacle_receiver);
}

void DCbfDualityController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                                 const std::string& reference_file, bool verbose) {
  leggedInterface_ = std::make_shared<DCbfDualityLeggedInterface>(task_file, urdf_file, reference_file, verbose);
  leggedInterface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
}

void DCbfDualityController::setupMpc() {
  LeggedController::setupMpc();

  ros::NodeHandle nh;
  auto obstacle_receiver = std::make_shared<CbfObstaclesReceiver>(
      nh, dynamic_cast<DCbfDualityLeggedInterface&>(*leggedInterface_).getObstacles(), mpc_->getSolverPtr());
  mpc_->getSolverPtr()->addSynchronizedModule(obstacle_receiver);
}

}  // namespace cbf

PLUGINLIB_EXPORT_CLASS(cbf::DCbfSimpleController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cbf::DualityController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cbf::DCbfDualityController, controller_interface::ControllerBase)
