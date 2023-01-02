//
// Created by qiayuan on 2022/7/22.
//

#include "cbf_interface/FactoryAccessHelper.h"

namespace cbf {
CentroidalModelInfo createCbfModelInfo(const CentroidalModelInfo& info) {
  CentroidalModelInfo cbfInfo = info;
  cbfInfo.inputDim += 1;
  return cbfInfo;
}

DualityInfo createDualityInfo(const CentroidalModelInfo& info, size_t numObstacles, size_t numPoints) {
  DualityInfo polyInfo;
  polyInfo.centroidalInfo = info;
  polyInfo.numObstacles = numObstacles;
  polyInfo.numPoints = numPoints;
  polyInfo.centroidalInfo.inputDim += numObstacles * 4 + numObstacles * numPoints;

  return polyInfo;
}

}  // namespace cbf
