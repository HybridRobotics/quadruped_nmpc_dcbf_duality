//
// Created by qiayuan on 2022/8/9.
//

#pragma once

#include <mutex>

#include <ros/ros.h>
#include <cbf_msgs/Obstacle.h>

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_mpc/MPC_BASE.h>

#include "cbf_interface/obstacle.h"

namespace cbf
{
using namespace ocs2;

class ObstacleReceiver : public SolverSynchronizedModule
{
public:
  ObstacleReceiver(ros::NodeHandle nh, std::shared_ptr<DualityObstacles> obstacle, SolverBase* solver);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

protected:
  std::shared_ptr<DualityObstacles> obstacle_ptr_;
  vector_t dists_;

private:
  void pointsCallback(const cbf_msgs::ObstacleConstPtr& msg);

  SolverBase* solver_;

  ros::Subscriber subscriber_;
  ros::Publisher pub_;

  std::mutex mutex;
  std::atomic_bool points_updated_;
  vector_array_t points_array_;
  size_t msgs_size_;
};

class CbfObstaclesReceiver : public ObstacleReceiver
{
public:
  using ObstacleReceiver::ObstacleReceiver;

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;
};

}  // namespace cbf
