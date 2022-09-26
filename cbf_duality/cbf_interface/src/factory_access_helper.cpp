//
// Created by qiayuan on 2022/7/22.
//

#include "cbf_interface/factory_access_helper.h"

namespace cbf
{
CentroidalModelInfo createCbfModelInfo(const CentroidalModelInfo& info)
{
  CentroidalModelInfo cbf_info = info;
  cbf_info.inputDim += 1;
  return cbf_info;
}

DualityInfo createDualityInfo(const CentroidalModelInfo& info, size_t num_obstacles, size_t num_points)
{
  DualityInfo poly_info;
  poly_info.centroidal_info = info;
  poly_info.num_obstacles = num_obstacles;
  poly_info.num_points = num_points;
  poly_info.centroidal_info.inputDim += num_obstacles * 4 + num_obstacles * num_points;

  return poly_info;
}

}  // namespace cbf
