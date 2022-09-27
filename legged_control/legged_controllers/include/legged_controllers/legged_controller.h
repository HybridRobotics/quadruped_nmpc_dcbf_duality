//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <legged_common/hardware_interface/contact_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>

#include <legged_interface/legged_interface.h>
#include <legged_estimation/state_estimate_base.h>
#include <legged_wbc/wbc.h>

#include "legged_controllers/safety_checker.h"

namespace legged
{
using namespace ocs2;
using namespace legged_robot;

class LeggedController
  : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                          ContactSensorInterface>
{
public:
  LeggedController() = default;
  ~LeggedController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override
  {
    mpc_running_ = false;
  }

protected:
  virtual void setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                    const std::string& reference_file, bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(LeggedInterface& legged_interface,
                                  const std::vector<HybridJointHandle>& hybrid_joint_handles,
                                  const std::vector<ContactSensorHandle>& contact_sensor_handles,
                                  const hardware_interface::ImuSensorHandle& imu_sensor_handle);

  std::shared_ptr<LeggedInterface> legged_interface_;
  std::shared_ptr<Wbc> wbc_;
  std::shared_ptr<SafetyChecker> safety_checker_;

  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpc_mrt_interface_;
  std::shared_ptr<CentroidalModelRbdConversions> rbd_conversions_;
  std::shared_ptr<StateEstimateBase> state_estimate_;

  std::shared_ptr<LeggedRobotVisualizer> visualizer_;
  ros::Publisher observation_publisher_;

  SystemObservation current_observation_;
  std::vector<HybridJointHandle> hybrid_joint_handles_;

private:
  std::thread mpc_thread_;
  std::atomic_bool controller_running_, mpc_running_{};
};

class LeggedCheaterController : public LeggedController
{
protected:
  void setupStateEstimate(LeggedInterface& legged_interface, const std::vector<HybridJointHandle>& hybrid_joint_handles,
                          const std::vector<ContactSensorHandle>& contact_sensor_handles,
                          const hardware_interface::ImuSensorHandle& imu_sensor_handle) override;
};

}  // namespace legged
