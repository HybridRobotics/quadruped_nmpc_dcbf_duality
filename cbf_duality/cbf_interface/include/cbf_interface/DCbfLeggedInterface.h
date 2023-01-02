//
// Created by qiayuan on 2022/7/16.
//

#pragma once
#include "cbf_interface/DcbfDuality.h"
#include "cbf_interface/Duality.h"
#include "cbf_interface/SimpleObstacle.h"

#include <legged_interface/LeggedInterface.h>

namespace cbf {
using namespace ocs2;
using namespace legged_robot;

class DCbfSimpleLeggedInterface : public legged::LeggedInterface {
 public:
  using LeggedInterface::LeggedInterface;

  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                  bool verbose) override;

  std::vector<std::shared_ptr<ObstacleSimple>> getObstacles() const { return obstacles_; }

 protected:
  void setupModel(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose) override;
  std::vector<std::shared_ptr<ObstacleSimple>> obstacles_;
};

class DualityLeggedInterface : public legged::LeggedInterface {
 public:
  using LeggedInterface::LeggedInterface;

  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                  bool verbose) override;

  virtual void setupObstacles() { obstacles_ = std::make_shared<DualityObstacles>(dualityInfo_); }

  std::shared_ptr<DualityObstacles> getObstacles() const { return obstacles_; }

 protected:
  void setupModel(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose) override;

  DualityInfo dualityInfo_;
  std::shared_ptr<DualityObstacles> obstacles_;
};

class DCbfDualityLeggedInterface : public DualityLeggedInterface {
 public:
  using DualityLeggedInterface::DualityLeggedInterface;

  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                  bool verbose) override;

  void setupObstacles() override { obstacles_ = std::make_shared<CbfObstacles>(dualityInfo_); }
};

}  // namespace cbf
