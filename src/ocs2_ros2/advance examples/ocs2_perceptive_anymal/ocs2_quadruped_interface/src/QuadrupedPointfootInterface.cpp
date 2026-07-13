//
// Created by rgrandia on 19.03.20.
//

#include "ocs2_quadruped_interface/QuadrupedPointfootInterface.h"

#include <ocs2_ddp/ContinuousTimeLqr.h>
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>
#include <ocs2_core/penalties/penalties/SquaredHingePenalty.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_switched_model_interface/cost/SelfCollisionAvoidanceCost.h>
#include <ocs2_switched_model_interface/constraint/SameSideFootSeparationConstraint.h>
#include <ocs2_switched_model_interface/core/TorqueApproximation.h>

namespace switched_model {

namespace {

// SQP enforces costs directly, whereas state-only inequality constraints are
// currently only evaluated for metrics. Target a small buffer above the
// requested separation so the closed-loop rollout remains above the limit.
constexpr scalar_t kSameSideFootSeparationGuard = 0.03;
constexpr scalar_t kSameSideFootSeparationPenalty = 5.0e3;

}  // namespace

QuadrupedPointfootInterface::QuadrupedPointfootInterface(const kinematic_model_t& kinematicModel,
                                                         const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                                                         const ad_com_model_t& adComModel,
                                                         const InverseKinematicsModelBase* inverseKinematics, Settings settings,
                                                         std::vector<std::string> jointNames, std::string baseName)
    : QuadrupedInterface(kinematicModel, adKinematicModel, comModel, adComModel, inverseKinematics, std::move(settings),
                         std::move(jointNames), std::move(baseName)) {
  // nominal values
  const auto stanceFlags = switched_model::constantFeetArray(true);
  const auto uSystemForWeightCompensation = weightCompensatingInputs(getComModel(), stanceFlags, switched_model::vector3_t::Zero());
  const auto jointTorquesForWeightCompensation = torqueApproximation(
      getJointPositions(getInitialState()), toArray<scalar_t>(uSystemForWeightCompensation.head<3 * NUM_CONTACT_POINTS>()), kinematicModel);

  problemPtr_->preComputationPtr = createPrecomputation();

  // Cost terms
  problemPtr_->costPtr->add("MotionTrackingCost", createMotionTrackingCost());
  problemPtr_->stateCostPtr->add("FootPlacementCost", createFootPlacementCost());
  problemPtr_->stateCostPtr->add("CollisionAvoidanceCost", createCollisionAvoidanceCost());
  if (modelSettings().enableSelfCollisionAvoidance_) {
    problemPtr_->stateCostPtr->add(
        "SelfCollisionAvoidanceCost",
        std::make_unique<SelfCollisionAvoidanceCost>(ocs2::ThresholdRelaxedBarrierPenalty::Config{
            modelSettings().muSelfCollision_, modelSettings().deltaSelfCollision_,
            modelSettings().selfCollisionActivationDistance_}));
  }
  if (modelSettings().minimumSameSideFootSeparation_ > 0.0) {
    const auto minimumSeparation =
        modelSettings().minimumSameSideFootSeparation_;

    // Keep the explicit inequality for diagnostics and solvers that support
    // state-only inequalities natively.
    problemPtr_->stateInequalityConstraintPtr->add(
        "SameSideFootSeparation",
        std::make_unique<SameSideFootSeparationConstraint>(
            minimumSeparation));

    // The SQP MPC backend optimizes state costs, but does not pass generic
    // state-only inequalities to its QP. This high-weight hinge penalty is
    // consequently the active enforcement mechanism for this demo.
    problemPtr_->stateCostPtr->add(
        "SameSideFootSeparationCost",
        std::make_unique<ocs2::StateSoftConstraint>(
            std::make_unique<SameSideFootSeparationConstraint>(
                minimumSeparation + kSameSideFootSeparationGuard),
            std::make_unique<ocs2::SquaredHingePenalty>(
                ocs2::SquaredHingePenalty::Config{
                    kSameSideFootSeparationPenalty, 0.0})));
  }
  problemPtr_->costPtr->add("JointLimitCost", createJointLimitsSoftConstraint());
  problemPtr_->costPtr->add("TorqueLimitCost", createTorqueLimitsSoftConstraint(jointTorquesForWeightCompensation));
  problemPtr_->costPtr->add("FrictionCones", createFrictionConeCost());

  // Dynamics
  problemPtr_->dynamicsPtr = createDynamics();

  // Per leg terms
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    const auto& footName = feetNames[i];
    problemPtr_->equalityConstraintPtr->add(footName + "_ZeroForce", createZeroForceConstraint(i));
    problemPtr_->equalityConstraintPtr->add(footName + "_EENormal", createFootNormalConstraint(i));
    problemPtr_->equalityConstraintPtr->add(footName + "_EEVel", createEndEffectorVelocityConstraint(i));
  }

  // Initialize cost to be able to query it
  ocs2::TargetTrajectories targetTrajectories({0.0}, {getInitialState()}, {uSystemForWeightCompensation});
  problemPtr_->targetTrajectoriesPtr = &targetTrajectories;

  getSwitchedModelModeScheduleManagerPtr()->setTargetTrajectories(targetTrajectories);
  getSwitchedModelModeScheduleManagerPtr()->preSolverRun(0.0, 1.0, getInitialState());
  auto lqrSolution = ocs2::continuous_time_lqr::solve(*problemPtr_, 0.0, getInitialState(), uSystemForWeightCompensation);
  lqrSolution.valueFunction *= 10.0;
  problemPtr_->finalCostPtr->add("lqr_terminal_cost", createMotionTrackingTerminalCost(lqrSolution.valueFunction));

  // Store cost approximation at nominal state input
  auto& preComputation = *problemPtr_->preComputationPtr;
  constexpr auto request = ocs2::Request::Cost + ocs2::Request::SoftConstraint + ocs2::Request::Approximation;
  preComputation.request(request, 0.0, getInitialState(), uSystemForWeightCompensation);
  nominalCostApproximation_ = ocs2::approximateCost(*problemPtr_, 0.0, getInitialState(), uSystemForWeightCompensation);

  // Reset, the target trajectories pointed to are local
  problemPtr_->targetTrajectoriesPtr = nullptr;

  initializerPtr_.reset(new ComKinoInitializer(getComModel(), *getSwitchedModelModeScheduleManagerPtr()));
  timeTriggeredRolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings()));
}

}  // namespace switched_model
