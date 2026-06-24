//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/constraint/FootPlacementConstraint.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <array>
#include <iomanip>
#include <iostream>

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

    FootPlacementConstraint::FootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                     const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                     size_t contactPointIndex,
                                                     size_t numVertices)
        : StateConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(&referenceManager),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          contactPointIndex_(contactPointIndex),
          numVertices_(numVertices)
    {
    }

    FootPlacementConstraint::FootPlacementConstraint(const FootPlacementConstraint& rhs)
        : StateConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(rhs.referenceManagerPtr_),
          endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
          contactPointIndex_(rhs.contactPointIndex_),
          numVertices_(rhs.numVertices_)
    {
    }

    bool FootPlacementConstraint::isActive(scalar_t time) const
    {
        const bool active = dynamic_cast<const PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_).getFootPlacementFlags(time)[
            contactPointIndex_];
        static std::array<bool, 4> initialized{};
        static std::array<bool, 4> lastActive{};
        static std::array<scalar_t, 4> lastLogTime = {-1.0, -1.0, -1.0, -1.0};
        if (contactPointIndex_ < initialized.size() &&
            (!initialized[contactPointIndex_] || active != lastActive[contactPointIndex_] ||
                time - lastLogTime[contactPointIndex_] > 0.25))
        {
            initialized[contactPointIndex_] = true;
            lastActive[contactPointIndex_] = active;
            lastLogTime[contactPointIndex_] = time;
            std::cerr << std::fixed << std::setprecision(3)
                << "[FootPlacementConstraint] leg=" << contactPointIndex_
                << " time=" << time
                << " active=" << static_cast<int>(active) << std::endl;
        }
        return active;
    }

    vector_t FootPlacementConstraint::getValue(scalar_t /*time*/, const vector_t& state,
                                               const PreComputation& preComp) const
    {
        static std::array<bool, 4> checkedPrecomputationType{};
        if (contactPointIndex_ < checkedPrecomputationType.size() && !checkedPrecomputationType[contactPointIndex_])
        {
            checkedPrecomputationType[contactPointIndex_] = true;
            const bool isPerceptive =
                dynamic_cast<const PerceptiveLeggedPrecomputation*>(&preComp) != nullptr;
            std::cerr << "[FootPlacementConstraint] preComputation cast check leg=" << contactPointIndex_
                << " is PerceptiveLeggedPrecomputation="
                << static_cast<int>(isPerceptive) << std::endl;
        }
        const auto& perceptivePreComp = cast<PerceptiveLeggedPrecomputation>(preComp);
        if (auto* pinocchioKinematics =
            dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get()))
        {
            pinocchioKinematics->setPinocchioInterface(perceptivePreComp.getPinocchioInterface());
        }
        const auto param = perceptivePreComp.getFootPlacementConParameters()[contactPointIndex_];
        const auto positions = endEffectorKinematicsPtr_->getPosition(state);
        return param.a * selectLegKinematics(positions, contactPointIndex_) + param.b;
    }

    VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(
        scalar_t /*time*/, const vector_t& state,
        const PreComputation& preComp) const
    {
        VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(
            numVertices_, state.size(), 0);
        const auto& perceptivePreComp = cast<PerceptiveLeggedPrecomputation>(preComp);
        if (auto* pinocchioKinematics =
            dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get()))
        {
            pinocchioKinematics->setPinocchioInterface(perceptivePreComp.getPinocchioInterface());
        }
        const auto param = perceptivePreComp.getFootPlacementConParameters()[contactPointIndex_];

        const auto positionApproximations = endEffectorKinematicsPtr_->getPositionLinearApproximation(state);
        const auto positionApprox = selectLegKinematics(positionApproximations, contactPointIndex_);
        approx.f = param.a * positionApprox.f + param.b;
        approx.dfdx = param.a * positionApprox.dfdx;
        return approx;
    }
} // namespace legged
