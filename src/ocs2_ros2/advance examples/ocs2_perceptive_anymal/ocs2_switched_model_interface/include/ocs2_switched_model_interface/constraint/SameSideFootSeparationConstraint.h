#pragma once

#include <ocs2_core/constraint/StateConstraint.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace switched_model {

/**
 * Enforces a minimum world-x separation between the front and hind foot on
 * each side of the robot: x_front - x_hind >= minimumSeparation.
 */
class SameSideFootSeparationConstraint final : public ocs2::StateConstraint {
 public:
  explicit SameSideFootSeparationConstraint(scalar_t minimumSeparation);

  SameSideFootSeparationConstraint* clone() const override;

  size_t getNumConstraints(scalar_t time) const override { return 2; }

  vector_t getValue(scalar_t time, const vector_t& state,
                    const ocs2::PreComputation& preComp) const override;

  VectorFunctionLinearApproximation getLinearApproximation(
      scalar_t time, const vector_t& state,
      const ocs2::PreComputation& preComp) const override;

 private:
  SameSideFootSeparationConstraint(
      const SameSideFootSeparationConstraint& rhs) = default;

  const scalar_t minimumSeparation_;
};

}  // namespace switched_model
