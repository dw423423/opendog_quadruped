//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/constraint/FootPlacementConstraint.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

#include <iomanip>
#include <iostream>

namespace ocs2::legged_robot
{
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
        if (contactPointIndex_ == 0)
        {
            static bool initialized = false;
            static bool lastActive = false;
            static scalar_t lastLogTime = -1.0;
            if (!initialized || active != lastActive || time - lastLogTime > 0.25)
            {
                initialized = true;
                lastActive = active;
                lastLogTime = time;
                std::cerr << std::fixed << std::setprecision(3)
                    << "[FootPlacementConstraint] leg=0 time=" << time
                    << " active=" << static_cast<int>(active) << std::endl;
            }
        }
        return active;
    }

    vector_t FootPlacementConstraint::getValue(scalar_t /*time*/, const vector_t& state,
                                               const PreComputation& preComp) const
    {
        if (contactPointIndex_ == 0)
        {
            static bool checkedPrecomputationType = false;
            if (!checkedPrecomputationType)
            {
                checkedPrecomputationType = true;
                const bool isPerceptive =
                    dynamic_cast<const PerceptiveLeggedPrecomputation*>(&preComp) != nullptr;
                std::cerr << "[FootPlacementConstraint] preComputation cast check leg=0 "
                    << "is PerceptiveLeggedPrecomputation="
                    << static_cast<int>(isPerceptive) << std::endl;
            }
        }
        const auto param = cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[
            contactPointIndex_];
        return param.a * endEffectorKinematicsPtr_->getPosition(state).front() + param.b;
    }

    VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(
        scalar_t /*time*/, const vector_t& state,
        const PreComputation& preComp) const
    {
        VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(
            numVertices_, state.size(), 0);
        const auto param = cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[
            contactPointIndex_];

        const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
        approx.f = param.a * positionApprox.f + param.b;
        approx.dfdx = param.a * positionApprox.dfdx;
        return approx;
    }
} // namespace legged
