#include <gtest/gtest.h>

#include <ocs2_switched_model_interface/constraint/SelfCollisionConstraint.h>
#include <ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h>

namespace switched_model {
TEST(SelfCollisionConstraintTest, ValueGradientAndDegeneratePairRemainFinite) {
  SwitchedModelPreComputationMockup pc;
  auto& spheres = pc.selfCollisionSpheresInOriginFrame();
  spheres = {{vector3_t::Zero(), 0.10}, {vector3_t{0.35, 0.0, 0.0}, 0.15}};
  pc.selfCollisionPairs().push_back({0, 1, 0.05});
  auto& derivatives = pc.selfCollisionSpheresDerivative();
  derivatives.resize(2, matrix_t::Zero(3, STATE_DIM));
  derivatives[0](0, 0) = 1.0;

  SelfCollisionConstraint constraint(1);
  const auto value = constraint.getValue(0.0, vector_t::Zero(STATE_DIM), pc);
  EXPECT_NEAR(value(0), 0.05, 1e-12);
  const auto approximation = constraint.getLinearApproximation(0.0, vector_t::Zero(STATE_DIM), pc);
  EXPECT_NEAR(approximation.dfdx(0, 0), -1.0, 1e-12);

  spheres[1].position.setZero();
  const auto degenerate = constraint.getLinearApproximation(0.25, vector_t::Zero(STATE_DIM), pc);
  EXPECT_EQ(degenerate.f.size(), 1);
  EXPECT_TRUE(degenerate.f.allFinite());
  EXPECT_TRUE(degenerate.dfdx.allFinite());
}
}  // namespace switched_model
