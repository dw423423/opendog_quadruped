#include "ocs2_switched_model_interface/constraint/SameSideFootSeparationConstraint.h"

#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"

namespace switched_model {

namespace {

constexpr size_t kLeftFront = static_cast<size_t>(FeetEnum::LF);
constexpr size_t kRightFront = static_cast<size_t>(FeetEnum::RF);
constexpr size_t kLeftHind = static_cast<size_t>(FeetEnum::LH);
constexpr size_t kRightHind = static_cast<size_t>(FeetEnum::RH);

}  // namespace

SameSideFootSeparationConstraint::SameSideFootSeparationConstraint(
    scalar_t minimumSeparation)
    : StateConstraint(ocs2::ConstraintOrder::Linear),
      minimumSeparation_(minimumSeparation) {}

SameSideFootSeparationConstraint*
SameSideFootSeparationConstraint::clone() const {
  return new SameSideFootSeparationConstraint(*this);
}

vector_t SameSideFootSeparationConstraint::getValue(
    scalar_t /*time*/, const vector_t& /*state*/,
    const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp =
      ocs2::cast<SwitchedModelPreComputation>(preComp);
  const auto& leftFront =
      switchedModelPreComp.footPositionInOriginFrame(kLeftFront);
  const auto& leftHind =
      switchedModelPreComp.footPositionInOriginFrame(kLeftHind);
  const auto& rightFront =
      switchedModelPreComp.footPositionInOriginFrame(kRightFront);
  const auto& rightHind =
      switchedModelPreComp.footPositionInOriginFrame(kRightHind);

  vector_t h(2);
  h << leftFront.x() - leftHind.x() - minimumSeparation_,
      rightFront.x() - rightHind.x() - minimumSeparation_;
  return h;
}

VectorFunctionLinearApproximation
SameSideFootSeparationConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state,
    const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp =
      ocs2::cast<SwitchedModelPreComputation>(preComp);

  VectorFunctionLinearApproximation approximation;
  approximation.f = getValue(time, state, preComp);
  approximation.dfdx.resize(2, STATE_DIM);
  approximation.dfdx.row(0) =
      switchedModelPreComp.footPositionInOriginFrameStateDerivative(kLeftFront)
          .row(0) -
      switchedModelPreComp.footPositionInOriginFrameStateDerivative(kLeftHind)
          .row(0);
  approximation.dfdx.row(1) =
      switchedModelPreComp.footPositionInOriginFrameStateDerivative(kRightFront)
          .row(0) -
      switchedModelPreComp.footPositionInOriginFrameStateDerivative(kRightHind)
          .row(0);
  return approximation;
}

}  // namespace switched_model
