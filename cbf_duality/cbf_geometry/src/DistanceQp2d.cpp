//
// Created by qiayuan on 2022/8/8.
//
#include "cbf_geometry/DistanceQp2d.h"

namespace cbf {
using namespace ocs2;

DistanceQp2d::DistanceQp2d(const ConvexRegion2d<scalar_t>& region0, ConvexRegion2d<scalar_t>& region1) {
  matrix_t i = matrix_t::Identity(2, 2);
  h_ << i, -i, -i, i;
  g_ = vector_t::Zero(4);

  vector_t b0 = region0.getB();
  vector_t b1 = region1.getB();

  c_ = matrix_t::Zero(b0.size() + b1.size(), 4);
  c_.block(0, 0, b0.size(), 2) = region0.getA();
  c_.block(b0.size(), 2, b1.size(), 2) = region1.getA();
  ub_ = vector_t::Zero(b0.size() + b1.size());
  ub_ << b0, b1;

  auto qpProblem = qpOASES::QProblem(4, ub_.size());
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  qpProblem.setOptions(options);
  int n_wsr = 20;

  qpProblem.init(h_.data(), g_.data(), c_.data(), nullptr, nullptr, nullptr, ub_.data(), n_wsr);
  sol_ = vector_t(4);
  qpProblem.getPrimalSolution(sol_.data());
}

scalar_t DistanceQp2d::getDistance() {
  return sqrt(sol_.transpose() * h_ * sol_);
}

vector_t DistanceQp2d::getSolution() {
  return sol_;
}

Duality2d::Duality2d(const ConvexRegion2d<scalar_t>& region0, ConvexRegion2d<scalar_t>& region1)
    : numPointsRegion0_(region0.getB().size()),
      numPointsRegion1_(region1.getB().size()),
      numPoints_(numPointsRegion0_ + numPointsRegion1_) {
  setupCost(region0, region1);
  setupConstraints(region0, region1);

  qpProblem_ = std::make_shared<qpOASES::QProblem>(numPoints_, c_.rows());
  qpOASES::Options options;
  options.setToReliable();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  qpProblem_->setOptions(options);
  int n_wsr = 50;
  qpProblem_->init(h_.data(), g_.data(), c_.data(), l_.data(), nullptr, lb_.data(), ub_.data(), n_wsr);
}

scalar_t Duality2d::getDistance() {
  return sqrt(-qpProblem_->getObjVal());
}

vector_t Duality2d::getSolution() {
  vector_t sol = vector_t(h_.rows());
  qpProblem_->getPrimalSolution(sol.data());
  return sol;
}

void Duality2d::setupCost(const ConvexRegion2d<scalar_t>& region0, ConvexRegion2d<scalar_t>& region1) {
  h_ = matrix_t(numPoints_, numPoints_);
  h_ << region0.getA() * region0.getA().transpose(), matrix_t::Zero(numPointsRegion0_, numPointsRegion1_),
      matrix_t::Zero(numPointsRegion1_, numPoints_);
  h_ *= 0.5;
  g_ = vector_t(numPoints_);
  g_ << region0.getB(), region1.getB();
}

void Duality2d::setupConstraints(const ConvexRegion2d<scalar_t>& region0, ConvexRegion2d<scalar_t>& region1) {
  c_ = matrix_t(4, numPoints_);
  c_ << region0.getA().transpose(), region1.getA().transpose();
  c_.block(2, 0, 2, numPoints_) = -c_.block(0, 0, 2, numPoints_);

  l_ = vector_t::Zero(numPoints_);
  ub_ = vector_t::Zero(4);
  lb_ = vector_t::Zero(4);
}

}  // namespace cbf
