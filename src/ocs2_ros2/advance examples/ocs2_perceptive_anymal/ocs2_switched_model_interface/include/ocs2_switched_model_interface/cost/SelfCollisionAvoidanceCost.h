//
// Smooth self-collision avoidance for declared pairs of collision spheres.
//

#pragma once

#include <memory>

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/penalties/ThresholdRelaxedBarrierPenalty.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace switched_model {

class SelfCollisionAvoidanceCost final : public ocs2::StateCost {
 public:
  explicit SelfCollisionAvoidanceCost(ocs2::ThresholdRelaxedBarrierPenalty::Config settings);
  ~SelfCollisionAvoidanceCost() override = default;

  SelfCollisionAvoidanceCost* clone() const override;

  scalar_t getValue(scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories,
                    const ocs2::PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories,
      const ocs2::PreComputation& preComp) const override;

 private:
  SelfCollisionAvoidanceCost(const SelfCollisionAvoidanceCost& rhs);

  std::unique_ptr<ocs2::ThresholdRelaxedBarrierPenalty> penalty_;
};

}  // namespace switched_model
