//
// Created by biao on 3/21/25.
//
#include "ocs2_quadruped_controller/perceptive/constraint/FootCollisionConstraint.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace ocs2::legged_robot
{
    namespace
    {
        template <typename VectorLike>
        const auto& selectLegKinematics(const VectorLike& values, size_t leg)
        {
            return values.size() == 1 ? values.front() : values.at(leg);
        }
    }

    FootCollisionConstraint::FootCollisionConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                     const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                     std::shared_ptr<grid_map::SignedDistanceField> sdfPtr,
                                                     size_t contactPointIndex,
                                                     scalar_t clearance)
        : StateConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(&referenceManager),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          sdfPtr_(std::move(sdfPtr)),
          contactPointIndex_(contactPointIndex),
          clearance_(clearance)
    {
    }

    FootCollisionConstraint::FootCollisionConstraint(const FootCollisionConstraint& rhs)
        : StateConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(rhs.referenceManagerPtr_),
          endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
          sdfPtr_(rhs.sdfPtr_),
          contactPointIndex_(rhs.contactPointIndex_),
          clearance_(rhs.clearance_)
    {
    }

    bool FootCollisionConstraint::isActive(scalar_t time) const
    {
        scalar_t offset = 0.05;
        return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_] &&
            !referenceManagerPtr_->getContactFlags(time + 0.5 * offset)[contactPointIndex_] &&
            !referenceManagerPtr_->getContactFlags(time - offset)[contactPointIndex_];
    }

    vector_t FootCollisionConstraint::getValue(scalar_t /*time*/, const vector_t& state,
                                               const PreComputation& preComp) const
    {
        if (auto* pinocchioKinematics =
            dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get()))
        {
            pinocchioKinematics->setPinocchioInterface(
                cast<PerceptiveLeggedPrecomputation>(preComp).getPinocchioInterface());
        }
        vector_t value(1);
        const auto positions = endEffectorKinematicsPtr_->getPosition(state);
        value(0) = sdfPtr_->getDistanceAt(
                grid_map::Position3(selectLegKinematics(positions, contactPointIndex_))) -
            clearance_;
        return value;
    }

    VectorFunctionLinearApproximation FootCollisionConstraint::getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& preComp) const
    {
        VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(1, state.size(), 0);
        approx.f = getValue(time, state, preComp);
        const auto positions = endEffectorKinematicsPtr_->getPosition(state);
        const auto positionApproximations = endEffectorKinematicsPtr_->getPositionLinearApproximation(state);
        approx.dfdx = sdfPtr_->getDistanceGradientAt(
                           grid_map::Position3(selectLegKinematics(positions, contactPointIndex_))).
                       transpose() *
            selectLegKinematics(positionApproximations, contactPointIndex_).dfdx;
        return approx;
    }
} // namespace legged
