//
// Created by tlab-uav on 25-2-27.
//

#include "ocs2_quadruped_controller/FSM/StateOCS2.h"

#include <algorithm>
#include <array>
#include <angles/angles.h>
#include <cmath>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_quadruped_controller/perceptive/interface/FixedFootholdRegions.h>
#include <ocs2_sqp/SqpMpc.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <sstream>

namespace ocs2::legged_robot
{
    namespace
    {
        bool findNextTouchdown(const ModeSchedule& modeSchedule, scalar_t now, size_t leg,
                               scalar_t& swingStartTime, scalar_t& touchdownTime)
        {
            const auto& eventTimes = modeSchedule.eventTimes;
            const auto& modeSequence = modeSchedule.modeSequence;
            if (eventTimes.size() < 2 || modeSequence.size() < 2)
            {
                return false;
            }
            for (size_t phase = 1; phase < modeSequence.size(); ++phase)
            {
                const scalar_t eventTime = eventTimes[phase];
                if (eventTime <= now)
                {
                    continue;
                }
                const bool wasSwing = !modeNumber2StanceLeg(modeSequence[phase - 1])[leg];
                const bool becomesStance = modeNumber2StanceLeg(modeSequence[phase])[leg];
                if (wasSwing && becomesStance)
                {
                    swingStartTime = eventTimes[phase - 1];
                    touchdownTime = eventTime;
                    return true;
                }
            }
            return false;
        }

        void updatePinocchioFrames(PinocchioInterface& pinocchioInterface,
                                   const CentroidalModelInfo& info,
                                   const vector_t& state)
        {
            CentroidalModelPinocchioMapping mapping(info);
            mapping.setPinocchioInterface(pinocchioInterface);
            const vector_t q = mapping.getPinocchioJointPosition(state);
            const auto& model = pinocchioInterface.getModel();
            auto& data = pinocchioInterface.getData();
            pinocchio::forwardKinematics(model, data, q);
            pinocchio::updateFramePlacements(model, data);
        }

        ConvexRegionSelector* getConvexRegionSelector(CtrlComponent& ctrlComponent)
        {
            const auto referenceManagerPtr = ctrlComponent.legged_interface_->getReferenceManagerPtr();
            auto* perceptiveReferenceManager =
                dynamic_cast<PerceptiveLeggedReferenceManager*>(referenceManagerPtr.get());
            return perceptiveReferenceManager == nullptr
                       ? nullptr
                       : perceptiveReferenceManager->getConvexRegionSelectorPtr().get();
        }
    }

    StateOCS2::StateOCS2(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<CtrlComponent>& ctrl_component)
        : FSMState(FSMStateName::OCS2, "OCS2 State", ctrl_interfaces),
          ctrl_component_(ctrl_component),
          node_(ctrl_component->node_)
    {
        if (!node_->has_parameter("default_kp"))
        {
            node_->declare_parameter("default_kp", default_kp_);
        }
        if (!node_->has_parameter("default_kd"))
        {
            node_->declare_parameter("default_kd", default_kd_);
        }
        default_kp_ = node_->get_parameter("default_kp").as_double();
        default_kd_ = node_->get_parameter("default_kd").as_double();

        // selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
        //                                                                        leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

        // Whole body control
        wbc_ = std::make_shared<WeightedWbc>(ctrl_component_->legged_interface_->getPinocchioInterface(),
                                             ctrl_component_->legged_interface_->getCentroidalModelInfo(),
                                             *ctrl_component_->ee_kinematics_);
        wbc_->loadTasksSetting(ctrl_component_->task_file_, ctrl_component_->verbose_);

        // Safety Checker
        safety_checker_ = std::make_shared<SafetyChecker>(ctrl_component_->legged_interface_->getCentroidalModelInfo());
    }

    void StateOCS2::enter()
    {
        startup_log_count_ = 0;
        actual_touchdown_contacts_initialized_ = false;

        const std::array<double, 12> stance_joint_targets = {
            0.0, 0.72, -1.44,
            0.0, 0.72, -1.44,
            0.0, 0.72, -1.44,
            0.0, 0.72, -1.44
        };
        double max_joint_error = 0.0;
        for (size_t i = 0; i < std::min(ctrl_interfaces_.joint_position_state_interface_.size(),
                                        stance_joint_targets.size()); ++i)
        {
            const double position = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
            max_joint_error = std::max(max_joint_error, std::abs(position - stance_joint_targets[i]));
        }
        if (max_joint_error > 0.15)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[OCS2] Entering OCS2 with large fixed-stand joint error: %.3f rad",
                        max_joint_error);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "[OCS2] Entering OCS2. Max fixed-stand joint error: %.3f rad",
                        max_joint_error);
        }

        ctrl_component_->init();
    }

    void StateOCS2::run(const rclcpp::Time& /**time**/,
                        const rclcpp::Duration& period)
    {
        if (ctrl_component_->mpc_running_ == false)
        {
            return;
        }

        // Load the latest MPC policy
        ctrl_component_->mpc_mrt_interface_->updatePolicy();

        // Evaluate the current policy
        size_t planned_mode = 0; // The mode that is active at the time the policy is evaluated at.
        ctrl_component_->mpc_mrt_interface_->evaluatePolicy(ctrl_component_->observation_.time,
                                                            ctrl_component_->observation_.state,
                                                            optimized_state_,
                                                            optimized_input_, planned_mode);

        static scalar_t lastPredictedTouchdownLogTime = -1.0;
        static scalar_t lastWaitingForAdvanceLogTime = -1.0;
        const auto& policy = ctrl_component_->mpc_mrt_interface_->getPolicy();
        auto* convexRegionSelector = getConvexRegionSelector(*ctrl_component_);
        if (ctrl_component_->consumeFootholdSequenceAdvanceRequest())
        {
            if (convexRegionSelector == nullptr || !convexRegionSelector->fixedFootholdSequenceEnabled())
            {
                RCLCPP_INFO(node_->get_logger(),
                            "[FootholdSequence] ignore_advance_key reason=sequence_disabled");
            }
            else
            {
                auto& sequenceManager = convexRegionSelector->getFixedFootholdSequenceManager();
                RCLCPP_INFO(node_->get_logger(),
                            "[FootholdSequence] advance_key_received key=%s state=%s",
                            sequenceManager.getConfig().advanceKey.c_str(),
                            sequenceManager.getStateName());
                if (sequenceManager.isFinalDone())
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "[FootholdSequence] ignore_advance_key reason=final_done active_set=%zu",
                                sequenceManager.getActiveSetIndex());
                }
                else if (!sequenceManager.isWaitingForAdvance())
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "[FootholdSequence] ignore_advance_key reason=current_set_not_completed "
                                "active_set=%zu reached=%s",
                                sequenceManager.getActiveSetIndex(),
                                sequenceManager.reachedToString().c_str());
                }
                else
                {
                    const size_t activeSetBefore = sequenceManager.getActiveSetIndex();
                    if (sequenceManager.advanceToNextSetByUser())
                    {
                        RCLCPP_INFO(node_->get_logger(),
                                    "[FootholdSequence] advance_set_by_key from=%zu to=%zu name=%s "
                                    "reset_reached=%s",
                                    activeSetBefore,
                                    sequenceManager.getActiveSetIndex(),
                                    sequenceManager.getActiveSetName().c_str(),
                                    sequenceManager.reachedToString().c_str());
                        RCLCPP_INFO(node_->get_logger(),
                                    "[FootholdSequence] request_gait %s",
                                    sequenceManager.getConfig().resumeGaitAfterAdvance.c_str());
                        ctrl_component_->requestGaitMode(sequenceManager.getConfig().resumeGaitAfterAdvance);
                    }
                }
            }
        }
        if (convexRegionSelector != nullptr && convexRegionSelector->fixedFootholdSequenceEnabled())
        {
            const auto& sequenceManager = convexRegionSelector->getFixedFootholdSequenceManager();
            if (sequenceManager.isWaitingForAdvance() &&
                (lastWaitingForAdvanceLogTime < 0.0 ||
                    ctrl_component_->observation_.time - lastWaitingForAdvanceLogTime > 1.0))
            {
                RCLCPP_INFO(node_->get_logger(),
                            "[FootholdSequence] state=WAITING_FOR_ADVANCE active_set=%zu name=%s",
                            sequenceManager.getActiveSetIndex(),
                            sequenceManager.getActiveSetName().c_str());
                lastWaitingForAdvanceLogTime = ctrl_component_->observation_.time;
            }
        }
        if ((lastPredictedTouchdownLogTime < 0.0 ||
                ctrl_component_->observation_.time - lastPredictedTouchdownLogTime > 0.25) &&
            !policy.timeTrajectory_.empty() &&
            convexRegionSelector != nullptr &&
            convexRegionSelector->fixedFootholdRegionsEnabled())
        {
            const auto& fixedRegionSettings = convexRegionSelector->getFixedFootholdRegionSettings();
            for (size_t leg = 0; leg < fixedRegionSettings.regions.size(); ++leg)
            {
                scalar_t swingStartTime = 0.0;
                scalar_t touchdownTime = 0.0;
                if (!findNextTouchdown(policy.modeSchedule_, ctrl_component_->observation_.time, leg,
                                       swingStartTime, touchdownTime) ||
                    touchdownTime > policy.timeTrajectory_.back())
                {
                    continue;
                }

                vector_t touchdownState;
                vector_t touchdownInput;
                size_t touchdownMode = 0;
                try
                {
                    ctrl_component_->mpc_mrt_interface_->evaluatePolicy(touchdownTime,
                                                                        ctrl_component_->observation_.state,
                                                                        touchdownState,
                                                                        touchdownInput,
                                                                        touchdownMode);
                    updatePinocchioFrames(ctrl_component_->legged_interface_->getPinocchioInterface(),
                                          ctrl_component_->legged_interface_->getCentroidalModelInfo(),
                                          touchdownState);
                    const vector3_t footPosition =
                        ctrl_component_->ee_kinematics_->getPosition(touchdownState)[leg];
                    RCLCPP_INFO(node_->get_logger(),
                                "[PREDICTED_TOUCHDOWN] leg=%zu name=%s selected_region=%s "
                                "swing_start=%.3f touchdown=%.3f mode=%zu "
                                "predicted_foot=(%.3f, %.3f, %.3f) inside_xy=%s",
                                leg,
                                convexRegionSelector->getFixedFootholdRegion(leg).name,
                                convexRegionSelector->fixedFootholdRegionToString(leg).c_str(),
                                swingStartTime,
                                touchdownTime,
                                touchdownMode,
                                footPosition.x(),
                                footPosition.y(),
                                footPosition.z(),
                                convexRegionSelector->isInsideFixedFootholdRegionXY(leg, footPosition)
                                    ? "true"
                                    : "false");
                }
                catch (const std::exception& e)
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "[PREDICTED_TOUCHDOWN] leg=%zu name=%s failed to evaluate touchdown policy "
                                "at %.3f: %s",
                                leg, convexRegionSelector->getFixedFootholdRegion(leg).name, touchdownTime, e.what());
                }
            }
            lastPredictedTouchdownLogTime = ctrl_component_->observation_.time;
        }

        const auto actualContacts = modeNumber2StanceLeg(ctrl_component_->observation_.mode);
        if (!actual_touchdown_contacts_initialized_)
        {
            last_actual_touchdown_contacts_ = actualContacts;
            actual_touchdown_contacts_initialized_ = true;
        }
        else
        {
            if (convexRegionSelector != nullptr && convexRegionSelector->fixedFootholdRegionsEnabled())
            {
                updatePinocchioFrames(ctrl_component_->legged_interface_->getPinocchioInterface(),
                                      ctrl_component_->legged_interface_->getCentroidalModelInfo(),
                                      ctrl_component_->observation_.state);
                const auto actualFootPositions = ctrl_component_->ee_kinematics_->getPosition(
                    ctrl_component_->observation_.state);
                const auto& fixedRegionSettings = convexRegionSelector->getFixedFootholdRegionSettings();
                for (size_t leg = 0; leg < fixedRegionSettings.regions.size(); ++leg)
                {
                    if (!last_actual_touchdown_contacts_[leg] && actualContacts[leg])
                    {
                        const auto& footPosition = actualFootPositions[leg];
                        const bool inside = convexRegionSelector->isInsideFixedFootholdRegionXY(leg, footPosition);
                        RCLCPP_INFO(node_->get_logger(),
                                    "[ACTUAL_TOUCHDOWN] leg=%zu name=%s selected_region=%s "
                                    "time=%.3f actual_foot=(%.3f, %.3f, %.3f) inside_xy=%s",
                                    leg,
                                    convexRegionSelector->getFixedFootholdRegion(leg).name,
                                    convexRegionSelector->fixedFootholdRegionToString(leg).c_str(),
                                    ctrl_component_->observation_.time,
                                    footPosition.x(),
                                    footPosition.y(),
                                    footPosition.z(),
                                    inside ? "true" : "false");

                        if (convexRegionSelector->fixedFootholdSequenceEnabled())
                        {
                            auto& sequenceManager = convexRegionSelector->getFixedFootholdSequenceManager();
                            const size_t activeSetBefore = sequenceManager.getActiveSetIndex();
                            const std::string activeSetNameBefore = sequenceManager.getActiveSetName();
                            const bool insideSequenceRegion = sequenceManager.markLegTouchdown(leg, footPosition);
                            RCLCPP_INFO(node_->get_logger(),
                                        "[FootholdSequence] touchdown leg=%zu name=%s active_set=%zu "
                                        "set_name=%s actual_foot=(%.3f,%.3f,%.3f) inside_xy=%s reached=%s",
                                        leg,
                                        sequenceManager.getActiveRegion(leg).name,
                                        activeSetBefore,
                                        activeSetNameBefore.c_str(),
                                        footPosition.x(),
                                        footPosition.y(),
                                        footPosition.z(),
                                        insideSequenceRegion ? "true" : "false",
                                        sequenceManager.reachedToString().c_str());

                            if (sequenceManager.isCurrentSetCompleted())
                            {
                                const std::string reachedBeforeAdvance = sequenceManager.reachedToString();
                                if (sequenceManager.isFinalSet())
                                {
                                    if (sequenceManager.enterFinalDoneIfCompleted() &&
                                        sequenceManager.consumeFinalCompletionAnnouncement())
                                    {
                                        RCLCPP_INFO(node_->get_logger(),
                                                    "[FootholdSequence] final_set_completed index=%zu name=%s "
                                                    "reached=%s",
                                                    sequenceManager.getActiveSetIndex(),
                                                    sequenceManager.getActiveSetName().c_str(),
                                                    sequenceManager.reachedToString().c_str());
                                        if (sequenceManager.consumeFinalStanceRequest())
                                        {
                                            RCLCPP_INFO(node_->get_logger(),
                                                        "[FootholdSequence] request_stance "
                                                        "auto_stance_on_final=true");
                                            ctrl_component_->requestStanceMode();
                                        }
                                    }
                                }
                                else
                                {
                                    if (sequenceManager.tryAdvanceSetAfterTouchdown())
                                    {
                                        RCLCPP_INFO(node_->get_logger(),
                                                    "[FootholdSequence] set_completed index=%zu name=%s reached=%s",
                                                    activeSetBefore,
                                                    activeSetNameBefore.c_str(),
                                                    reachedBeforeAdvance.c_str());
                                        RCLCPP_INFO(node_->get_logger(),
                                                    "[FootholdSequence] advance_set from=%zu to=%zu name=%s",
                                                    activeSetBefore,
                                                    sequenceManager.getActiveSetIndex(),
                                                    sequenceManager.getActiveSetName().c_str());
                                    }
                                    else if (sequenceManager.enterWaitingForAdvanceIfCompleted())
                                    {
                                        RCLCPP_INFO(node_->get_logger(),
                                                    "[FootholdSequence] set_completed index=%zu name=%s reached=%s",
                                                    activeSetBefore,
                                                    activeSetNameBefore.c_str(),
                                                    reachedBeforeAdvance.c_str());
                                        RCLCPP_INFO(node_->get_logger(),
                                                    "[FootholdSequence] waiting_for_advance key=%s "
                                                    "request_stance=%s",
                                                    sequenceManager.getConfig().advanceKey.c_str(),
                                                    sequenceManager.shouldRequestStanceOnSetCompleted()
                                                        ? "true"
                                                        : "false");
                                        if (sequenceManager.consumeSetCompletedStanceRequest())
                                        {
                                            ctrl_component_->requestStanceMode();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            last_actual_touchdown_contacts_ = actualContacts;
        }

        // Whole body control
        ctrl_component_->observation_.input = optimized_input_;

        wbc_timer_.startTimer();
        vector_t x = wbc_->update(optimized_state_, optimized_input_, ctrl_component_->measured_rbd_state_,
                                  planned_mode,
                                  period.seconds());
        wbc_timer_.endTimer();

        vector_t torque = x.tail(12);

        if (startup_log_count_ < 5)
        {
            const auto contact_flags = modeNumber2StanceLeg(ctrl_component_->observation_.mode);
            std::ostringstream foot_forces;
            for (size_t i = 0; i < ctrl_interfaces_.foot_force_state_interface_.size(); ++i)
            {
                if (i > 0)
                {
                    foot_forces << ", ";
                }
                foot_forces << ctrl_interfaces_.foot_force_state_interface_[i].get().get_value();
            }

            RCLCPP_INFO(node_->get_logger(),
                        "[OCS2] startup[%d] obs_mode=%zu planned_mode=%zu contacts=[%d,%d,%d,%d] "
                        "base_rpy=[%.3f, %.3f, %.3f] base_z=%.3f foot_forces=[%s] "
                        "tau_FL=[%.2f, %.2f, %.2f] tau_FR=[%.2f, %.2f, %.2f] "
                        "tau_RL=[%.2f, %.2f, %.2f] tau_RR=[%.2f, %.2f, %.2f]",
                        startup_log_count_,
                        ctrl_component_->observation_.mode,
                        planned_mode,
                        static_cast<int>(contact_flags[0]),
                        static_cast<int>(contact_flags[1]),
                        static_cast<int>(contact_flags[2]),
                        static_cast<int>(contact_flags[3]),
                        ctrl_component_->observation_.state(11),
                        ctrl_component_->observation_.state(10),
                        ctrl_component_->observation_.state(9),
                        ctrl_component_->observation_.state(8),
                        foot_forces.str().c_str(),
                        torque(0), torque(1), torque(2),
                        torque(3), torque(4), torque(5),
                        torque(6), torque(7), torque(8),
                        torque(9), torque(10), torque(11));
            ++startup_log_count_;
        }

        vector_t pos_des = centroidal_model::getJointAngles(optimized_state_,
                                                            ctrl_component_->legged_interface_->
                                                                             getCentroidalModelInfo());
        vector_t vel_des = centroidal_model::getJointVelocities(optimized_input_,
                                                                ctrl_component_->legged_interface_->
                                                                getCentroidalModelInfo());

        for (int i = 0; i < 12; i++)
        {
            ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque(i));
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(pos_des(i));
            ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(vel_des(i));
            ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(default_kp_);
            ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(default_kd_);
        }

        // Visualization
        ctrl_component_->visualizer_->update(ctrl_component_->mpc_mrt_interface_->getPolicy(),
                                             ctrl_component_->mpc_mrt_interface_->getCommand());
    }

    void StateOCS2::exit()
    {
    }

    FSMStateName StateOCS2::checkChange()
    {
        // Safety check, if failed, stop the controller
        if (!safety_checker_->check(ctrl_component_->observation_, optimized_state_, optimized_input_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[Legged Controller] Safety check failed, stopping the controller.");
            return FSMStateName::PASSIVE;
        }
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1:
            return FSMStateName::PASSIVE;
        default:
            return FSMStateName::OCS2;
        }
    }

}
