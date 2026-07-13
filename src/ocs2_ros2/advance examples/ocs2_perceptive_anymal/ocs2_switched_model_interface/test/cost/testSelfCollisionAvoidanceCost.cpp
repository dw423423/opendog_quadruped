#include <gtest/gtest.h>

#include <ocs2_core/reference/TargetTrajectories.h>

#include <ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h>
#include <ocs2_switched_model_interface/cost/SelfCollisionAvoidanceCost.h>

namespace switched_model {
namespace {

ocs2::TargetTrajectories makeTargetTrajectories() {
  return {{0.0}, {vector_t::Zero(STATE_DIM)}, {vector_t::Zero(INPUT_DIM)}};
}

TEST(SelfCollisionAvoidanceCostTest, ActivatesOnlyInsideConfiguredClearance) {
  SwitchedModelPreComputationMockup preComputation;
  auto& spheres = preComputation.selfCollisionSpheresInOriginFrame();
  spheres.resize(2);
  spheres[0] = {vector3_t::Zero(), 0.10};
  spheres[1] = {vector3_t{0.40, 0.0, 0.0}, 0.10};
  preComputation.selfCollisionPairs().push_back({0, 1, 0.03});

  SelfCollisionAvoidanceCost cost({0.1, 0.005, 0.05});
  const auto targetTrajectories = makeTargetTrajectories();

  EXPECT_DOUBLE_EQ(cost.getValue(0.0, vector_t::Zero(STATE_DIM), targetTrajectories, preComputation), 0.0);

  spheres[1].position.x() = 0.22;  // Surface clearance is -0.01 m after the requested 3 cm margin.
  EXPECT_GT(cost.getValue(0.0, vector_t::Zero(STATE_DIM), targetTrajectories, preComputation), 0.0);
}

TEST(SelfCollisionAvoidanceCostTest, GradientSeparatesTheSpheres) {
  SwitchedModelPreComputationMockup preComputation;
  auto& spheres = preComputation.selfCollisionSpheresInOriginFrame();
  spheres.resize(2);
  spheres[0] = {vector3_t::Zero(), 0.10};
  spheres[1] = {vector3_t{0.22, 0.0, 0.0}, 0.10};
  preComputation.selfCollisionPairs().push_back({0, 1, 0.03});

  auto& derivatives = preComputation.selfCollisionSpheresDerivative();
  derivatives.resize(2, matrix_t::Zero(3, STATE_DIM));
  derivatives[0](0, 0) = 1.0;

  SelfCollisionAvoidanceCost cost({0.1, 0.005, 0.05});
  const auto targetTrajectories = makeTargetTrajectories();
  const auto approximation = cost.getQuadraticApproximation(0.0, vector_t::Zero(STATE_DIM), targetTrajectories, preComputation);

  // Increasing the first sphere's x position moves it toward the second sphere. A positive gradient makes gradient descent move it away.
  EXPECT_GT(approximation.dfdx(0), 0.0);
  EXPECT_GT(approximation.dfdxx(0, 0), 0.0);
}

}  // namespace
}  // namespace switched_model
