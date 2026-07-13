#include <ocs2_switched_model_interface/cost/SelfCollisionAvoidanceCost.h>

#include <ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h>
#include <ocs2_switched_model_interface/cost/LinearStateInequalitySoftconstraint.h>

namespace switched_model {

SelfCollisionAvoidanceCost::SelfCollisionAvoidanceCost(ocs2::ThresholdRelaxedBarrierPenalty::Config settings)
    : penalty_(std::make_unique<ocs2::ThresholdRelaxedBarrierPenalty>(settings)) {}

SelfCollisionAvoidanceCost::SelfCollisionAvoidanceCost(const SelfCollisionAvoidanceCost& rhs) : penalty_(rhs.penalty_->clone()) {}

SelfCollisionAvoidanceCost* SelfCollisionAvoidanceCost::clone() const {
  return new SelfCollisionAvoidanceCost(*this);
}

scalar_t SelfCollisionAvoidanceCost::getValue(scalar_t /*time*/, const vector_t& /*state*/,
                                              const ocs2::TargetTrajectories& /*targetTrajectories*/,
                                              const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);
  const auto& spheres = switchedModelPreComp.selfCollisionSpheresInOriginFrame();

  scalar_t cost = 0.0;
  for (const auto& pair : switchedModelPreComp.selfCollisionPairs()) {
    const auto& first = spheres[pair.first];
    const auto& second = spheres[pair.second];
    const scalar_t separation = (first.position - second.position).norm() - first.radius - second.radius - pair.minimumDistance;
    cost += penalty_->getValue(0.0, separation);
  }
  return cost;
}

ScalarFunctionQuadraticApproximation SelfCollisionAvoidanceCost::getQuadraticApproximation(
    scalar_t /*time*/, const vector_t& /*state*/, const ocs2::TargetTrajectories& /*targetTrajectories*/,
    const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);
  const auto& spheres = switchedModelPreComp.selfCollisionSpheresInOriginFrame();
  const auto& sphereDerivatives = switchedModelPreComp.selfCollisionSpheresInOriginFrameStateDerivative();

  ScalarFunctionQuadraticApproximation cost;
  cost.f = 0.0;
  cost.dfdx = vector_t::Zero(STATE_DIM);
  cost.dfdxx = matrix_t::Zero(STATE_DIM, STATE_DIM);

  for (const auto& pair : switchedModelPreComp.selfCollisionPairs()) {
    const auto& first = spheres[pair.first];
    const auto& second = spheres[pair.second];
    const vector3_t displacement = first.position - second.position;
    const scalar_t centerDistance = displacement.norm();
    if (centerDistance < 1.0e-9) {
      continue;  // The distance direction is undefined only for exactly coincident sphere centres.
    }

    SingleLinearStateInequalitySoftConstraint constraint;
    constraint.A = (displacement / centerDistance).transpose();
    constraint.h = centerDistance - first.radius - second.radius - pair.minimumDistance;
    constraint.penalty = penalty_.get();

    const matrix_t displacementDerivative = sphereDerivatives[pair.first] - sphereDerivatives[pair.second];
    const auto pairCost = switched_model::getQuadraticApproximation(constraint, displacement, displacementDerivative);
    cost.f += pairCost.f;
    cost.dfdx += pairCost.dfdx;
    cost.dfdxx += pairCost.dfdxx;
  }

  return cost;
}

}  // namespace switched_model
