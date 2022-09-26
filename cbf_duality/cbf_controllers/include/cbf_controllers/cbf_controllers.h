//
// Created by qiayuan on 2022/7/16.
//

#pragma once

#include <legged_controllers/legged_controller.h>

namespace cbf
{
using namespace ocs2;
using namespace legged_robot;

class DCbfSimpleController : public legged::LeggedController
{
protected:
  void setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                            const std::string& reference_file, bool verbose) override;
  void setupMpc() override;
};

class DualityController : public legged::LeggedController
{
protected:
  void setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                            const std::string& reference_file, bool verbose) override;
  void setupMpc() override;
};

class DCbfDualityController : public legged::LeggedController
{
protected:
  void setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                            const std::string& reference_file, bool verbose) override;
  void setupMpc() override;
};

}  // namespace cbf
