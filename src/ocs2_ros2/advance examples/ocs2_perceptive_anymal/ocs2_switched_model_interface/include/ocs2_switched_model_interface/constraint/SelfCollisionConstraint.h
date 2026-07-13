#pragma once

#include <ocs2_core/constraint/StateConstraint.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace switched_model {

/** Sphere-pair clearance constraint h(x) >= 0 for every declared pair. */
class SelfCollisionConstraint final : public ocs2::StateConstraint {
 public:
  explicit SelfCollisionConstraint(size_t numPairs);

  SelfCollisionConstraint* clone() const override;
  size_t getNumConstraints(scalar_t /*time*/) const override { return numPairs_; }
  vector_t getValue(scalar_t time, const vector_t& state, const ocs2::PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                            const ocs2::PreComputation& preComp) const override;

 private:
  SelfCollisionConstraint(const SelfCollisionConstraint&) = default;
  const size_t numPairs_;
};

}  // namespace switched_model
