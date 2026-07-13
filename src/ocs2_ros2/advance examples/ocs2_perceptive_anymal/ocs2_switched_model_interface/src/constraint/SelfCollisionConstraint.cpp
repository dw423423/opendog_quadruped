#include <ocs2_switched_model_interface/constraint/SelfCollisionConstraint.h>

#include <iostream>
#include <stdexcept>

#include <ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h>

namespace switched_model {
namespace {
constexpr scalar_t kDirectionEpsilon = 1.0e-9;

vector3_t separationDirection(const vector3_t& displacement, size_t pairIndex, scalar_t time) {
  const scalar_t distance = displacement.norm();
  if (distance >= kDirectionEpsilon) return displacement / distance;
  // A deterministic, finite fallback preserves the row when centres coincide.
  // The controller must move along this direction or report infeasibility; it
  // is never acceptable to silently remove the colliding pair.
  std::cerr << "[SelfCollisionConstraint] degenerate pair=" << pairIndex << " distance=" << distance
            << " time=" << time << ", using +X fallback direction." << std::endl;
  return vector3_t::UnitX();
}
}  // namespace

SelfCollisionConstraint::SelfCollisionConstraint(size_t numPairs)
    : StateConstraint(ocs2::ConstraintOrder::Linear), numPairs_(numPairs) {}

SelfCollisionConstraint* SelfCollisionConstraint::clone() const { return new SelfCollisionConstraint(*this); }

vector_t SelfCollisionConstraint::getValue(scalar_t /*time*/, const vector_t& /*state*/,
                                           const ocs2::PreComputation& preComp) const {
  const auto& pc = ocs2::cast<SwitchedModelPreComputation>(preComp);
  const auto& spheres = pc.selfCollisionSpheresInOriginFrame();
  const auto& pairs = pc.selfCollisionPairs();
  if (pairs.size() != numPairs_) throw std::runtime_error("Self-collision pair count changed after construction.");
  vector_t h(numPairs_);
  for (size_t i = 0; i < numPairs_; ++i) {
    const auto& pair = pairs[i];
    const auto& first = spheres.at(pair.first);
    const auto& second = spheres.at(pair.second);
    h(i) = (first.position - second.position).norm() - first.radius - second.radius - pair.minimumDistance;
  }
  return h;
}

VectorFunctionLinearApproximation SelfCollisionConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const ocs2::PreComputation& preComp) const {
  const auto& pc = ocs2::cast<SwitchedModelPreComputation>(preComp);
  const auto& spheres = pc.selfCollisionSpheresInOriginFrame();
  const auto& derivatives = pc.selfCollisionSpheresInOriginFrameStateDerivative();
  const auto& pairs = pc.selfCollisionPairs();
  VectorFunctionLinearApproximation approximation;
  approximation.f = getValue(time, state, preComp);
  approximation.dfdx.resize(numPairs_, STATE_DIM);
  for (size_t i = 0; i < numPairs_; ++i) {
    const auto& pair = pairs.at(i);
    const vector3_t displacement = spheres.at(pair.first).position - spheres.at(pair.second).position;
    approximation.dfdx.row(i) = separationDirection(displacement, i, time).transpose() *
                                  (derivatives.at(pair.first) - derivatives.at(pair.second));
  }
  return approximation;
}

}  // namespace switched_model
