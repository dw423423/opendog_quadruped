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
#include <sstream>

namespace ocs2::legged_robot
{
    namespace
    {
        template <typename VectorLike>
        const auto& selectLegKinematics(const VectorLike& values, size_t leg)
        {
            return values.size() == 1 ? values.front() : values.at(leg);
        }

        const char* legName(size_t leg)
        {
            constexpr std::array<const char*, 4> names = {"FL", "FR", "RL", "RR"};
            return leg < names.size() ? names[leg] : "UNKNOWN";
        }

        std::string vectorToString(const vector_t& values)
        {
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(6) << "[";
            for (Eigen::Index i = 0; i < values.size(); ++i)
            {
                if (i > 0)
                {
                    stream << ", ";
                }
                stream << values(i);
            }
            stream << "]";
            return stream.str();
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

    vector_t FootPlacementConstraint::getValue(scalar_t time, const vector_t& state,
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
        const auto footPosition = selectLegKinematics(positions, contactPointIndex_);
        const vector_t values = param.a * footPosition + param.b;
        const scalar_t minMargin = values.size() > 0 ? values.minCoeff() : 0.0;
        const bool insideByOcs2Convention = values.size() > 0 && minMargin >= -1e-5;

        static std::array<scalar_t, 4> lastObservationLogTime = {-1.0, -1.0, -1.0, -1.0};
        static std::array<scalar_t, 4> lastViolationLogTime = {-1.0, -1.0, -1.0, -1.0};
        if (contactPointIndex_ < lastObservationLogTime.size() &&
            (lastObservationLogTime[contactPointIndex_] < 0.0 ||
                time - lastObservationLogTime[contactPointIndex_] > 0.25))
        {
            lastObservationLogTime[contactPointIndex_] = time;
            std::cerr << std::fixed << std::setprecision(3)
                << "[FootPlacementConstraint] leg=" << legName(contactPointIndex_)
                << " leg_index=" << contactPointIndex_
                << " time=" << time
                << " ocs2_soft_constraint_convention=h>=0"
                << " foot_position=(" << footPosition.x() << ","
                << footPosition.y() << "," << footPosition.z() << ")"
                << " constraint_values=" << vectorToString(values)
                << " minimum_constraint_margin=" << minMargin
                << " inside=" << (insideByOcs2Convention ? "true" : "false")
                << std::endl;
        }
        if (!insideByOcs2Convention && contactPointIndex_ < lastViolationLogTime.size() &&
            (lastViolationLogTime[contactPointIndex_] < 0.0 ||
                time - lastViolationLogTime[contactPointIndex_] > 0.25))
        {
            lastViolationLogTime[contactPointIndex_] = time;
            std::cerr << std::fixed << std::setprecision(3)
                << "[FootPlacementConstraint][WARN] leg=" << legName(contactPointIndex_)
                << " leg_index=" << contactPointIndex_
                << " time=" << time
                << " violation=true"
                << " ocs2_soft_constraint_convention=h>=0"
                << " foot_position=(" << footPosition.x() << ","
                << footPosition.y() << "," << footPosition.z() << ")"
                << " constraint_values=" << vectorToString(values)
                << " minimum_constraint_margin=" << minMargin
                << " violation_amount=" << -minMargin
                << std::endl;
        }

        return values;
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
