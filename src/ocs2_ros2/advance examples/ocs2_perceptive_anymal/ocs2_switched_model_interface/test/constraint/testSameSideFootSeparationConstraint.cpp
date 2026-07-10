#include <gtest/gtest.h>

#include <ocs2_core/penalties/penalties/SquaredHingePenalty.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_switched_model_interface/constraint/SameSideFootSeparationConstraint.h>
#include <ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h>

namespace switched_model {
namespace {

constexpr size_t kLeftFront = static_cast<size_t>(FeetEnum::LF);
constexpr size_t kRightFront = static_cast<size_t>(FeetEnum::RF);
constexpr size_t kLeftHind = static_cast<size_t>(FeetEnum::LH);
constexpr size_t kRightHind = static_cast<size_t>(FeetEnum::RH);

TEST(SameSideFootSeparationConstraintTest, ValueAndLinearApproximation) {
  SwitchedModelPreComputationMockup preComputation;
  preComputation.feetPositionInOriginFrame(kLeftFront) = {1.0, 0.0, 0.0};
  preComputation.feetPositionInOriginFrame(kLeftHind) = {0.8, 0.0, 0.0};
  preComputation.feetPositionInOriginFrame(kRightFront) = {1.1, 0.0, 0.0};
  preComputation.feetPositionInOriginFrame(kRightHind) = {0.7, 0.0, 0.0};

  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    preComputation.feetPositionInOriginFrameStateDerivative(leg) =
        matrix_t::Zero(3, STATE_DIM);
  }
  preComputation.feetPositionInOriginFrameStateDerivative(kLeftFront)(0, 0) =
      1.0;
  preComputation.feetPositionInOriginFrameStateDerivative(kLeftHind)(0, 0) =
      0.25;
  preComputation.feetPositionInOriginFrameStateDerivative(kRightFront)(0, 1) =
      0.5;
  preComputation.feetPositionInOriginFrameStateDerivative(kRightHind)(0, 1) =
      -0.25;

  SameSideFootSeparationConstraint constraint(0.2);
  const vector_t state = vector_t::Zero(STATE_DIM);

  const auto value = constraint.getValue(0.0, state, preComputation);
  EXPECT_NEAR(value(0), 0.0, 1e-12);
  EXPECT_NEAR(value(1), 0.2, 1e-12);

  const auto approximation =
      constraint.getLinearApproximation(0.0, state, preComputation);
  EXPECT_TRUE(approximation.f.isApprox(value));
  EXPECT_NEAR(approximation.dfdx(0, 0), 0.75, 1e-12);
  EXPECT_NEAR(approximation.dfdx(1, 1), 0.75, 1e-12);
  EXPECT_EQ(approximation.dfdx.rows(), 2);
  EXPECT_EQ(approximation.dfdx.cols(), STATE_DIM);
}

TEST(SameSideFootSeparationConstraintTest, SquaredHingePenaltyActivatesBelowGuard) {
  constexpr scalar_t minimumSeparation = 0.20;
  constexpr scalar_t guard = 0.03;

  SwitchedModelPreComputationMockup preComputation;
  preComputation.feetPositionInOriginFrame(kLeftFront) = {0.43, 0.0, 0.0};
  preComputation.feetPositionInOriginFrame(kLeftHind) = {0.20, 0.0, 0.0};
  preComputation.feetPositionInOriginFrame(kRightFront) = {0.43, 0.0, 0.0};
  preComputation.feetPositionInOriginFrame(kRightHind) = {0.20, 0.0, 0.0};

  ocs2::StateSoftConstraint penalty(
      std::make_unique<SameSideFootSeparationConstraint>(minimumSeparation + guard),
      std::make_unique<ocs2::SquaredHingePenalty>(
          ocs2::SquaredHingePenalty::Config{5.0e3, 0.0}));

  const vector_t state = vector_t::Zero(STATE_DIM);
  const ocs2::TargetTrajectories targetTrajectories;
  EXPECT_NEAR(penalty.getValue(0.0, state, targetTrajectories, preComputation), 0.0, 1e-20);

  preComputation.feetPositionInOriginFrame(kLeftFront).x() = 0.20;
  EXPECT_GT(penalty.getValue(0.0, state, targetTrajectories, preComputation), 0.0);
}

}  // namespace
}  // namespace switched_model
