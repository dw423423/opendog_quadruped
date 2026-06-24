//
// Created by biao on 3/15/25.
//

#include "ocs2_quadruped_controller/control/CtrlComponent.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <angles/angles.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_quadruped_controller/estimator/FromOdomTopic.h>
#include <ocs2_quadruped_controller/estimator/GroundTruth.h>
#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_quadruped_controller/control/GaitManager.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedInterface.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h>
#include <ocs2_quadruped_controller/perceptive/synchronize/PlanarTerrainReceiver.h>
#include <ocs2_sqp/SqpMpc.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <algorithm>
#include <string>
#include <vector>

namespace ocs2::legged_robot
{
    namespace
    {
        const std::array<const char*, 4> kLegNames = {"FL", "FR", "RL", "RR"};
        constexpr int kFootholdSequenceAdvanceCommand = 11;

        const char* boolString(bool value)
        {
            return value ? "true" : "false";
        }
    }

    CtrlComponent::CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                                 CtrlInterfaces& ctrl_interfaces) : node_(node), ctrl_interfaces_(ctrl_interfaces)
    {
        robot_pkg_ = node_->get_parameter("robot_pkg").as_string();
        joint_names_ = node_->get_parameter("joints").as_string_array();
        feet_names_ = node_->get_parameter("feet").as_string_array();

        if (!node_->has_parameter("enable_perceptive")) {
            node_->declare_parameter("enable_perceptive", enable_perceptive_);
        }
        enable_perceptive_ = node_->get_parameter("enable_perceptive").as_bool();

        const std::string package_share_directory = ament_index_cpp::get_package_share_directory(robot_pkg_);
        fixed_foothold_region_settings_ = loadFixedFootholdRegionSettings();
        fixed_foothold_sequence_config_ =
            loadFixedFootholdSequenceConfig(package_share_directory + "/config/gazebo.yaml");
        foothold_sequence_advance_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
            "/foothold_sequence/advance", 10,
            [this](const std_msgs::msg::Empty::SharedPtr /*msg*/)
            {
                foothold_sequence_advance_requested_.store(true);
            });

        urdf_file_ = package_share_directory + "/urdf/robot.urdf";
        task_file_ = package_share_directory + "/config/ocs2/task.info";
        reference_file_ = package_share_directory + "/config/ocs2/reference.info";
        gait_file_ = package_share_directory + "/config/ocs2/gait.info";

        loadData::loadCppDataType(task_file_, "legged_robot_interface.verbose", verbose_);

        setupLeggedInterface();
        setupMpc();
        setupMrt();

        CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
        ee_kinematics_ = std::make_unique<PinocchioEndEffectorKinematics>(
            legged_interface_->getPinocchioInterface(), pinocchio_mapping,
            legged_interface_->modelSettings().contactNames3DoF);
        ee_kinematics_->setPinocchioInterface(legged_interface_->getPinocchioInterface());

        rbd_conversions_ = std::make_unique<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                           legged_interface_->getCentroidalModelInfo());

        // Init visualizer
        visualizer_ = std::make_unique<LeggedRobotVisualizer>(
            legged_interface_->getPinocchioInterface(),
            legged_interface_->getCentroidalModelInfo(),
            *ee_kinematics_,
            node_);

        // Init observation
        observation_.state.setZero(static_cast<long>(legged_interface_->getCentroidalModelInfo().stateDim));
        observation_.input.setZero(
            static_cast<long>(legged_interface_->getCentroidalModelInfo().inputDim));
        observation_.mode = STANCE;
    }

    void CtrlComponent::setupStateEstimate(const std::string& estimator_type)
    {
        if (estimator_type == "ground_truth")
        {
            estimator_ = std::make_unique<GroundTruth>(legged_interface_->getCentroidalModelInfo(),
                                                       ctrl_interfaces_,
                                                       node_);
            RCLCPP_INFO(node_->get_logger(), "Using Ground Truth Estimator");
        }
        else if (estimator_type == "linear_kalman")
        {
            estimator_ = std::make_unique<KalmanFilterEstimate>(
                legged_interface_->getPinocchioInterface(),
                legged_interface_->getCentroidalModelInfo(),
                *ee_kinematics_, ctrl_interfaces_,
                node_);
            dynamic_cast<KalmanFilterEstimate&>(*estimator_).loadSettings(task_file_, verbose_);
            RCLCPP_INFO(node_->get_logger(), "Using Kalman Filter Estimator");
        }
        else
        {
            estimator_ = std::make_unique<FromOdomTopic>(
                legged_interface_->getCentroidalModelInfo(), ctrl_interfaces_, node_);
            RCLCPP_INFO(node_->get_logger(), "Using Odom Topic Based Estimator");
        }
        observation_.time = 0;
    }

    void CtrlComponent::updateState(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // Update State Estimation
        measured_rbd_state_ = estimator_->update(time, period);
        observation_.time += period.seconds();
        const scalar_t yaw_last = observation_.state(9);
        observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state_);
        observation_.state(9) = yaw_last + angles::shortest_angular_distance(
            yaw_last, observation_.state(9));
        observation_.mode = estimator_->getMode();

        visualizer_->update(observation_);
        if (enable_perceptive_)
        {
            if (footPlacementVisualizationPtr_ != nullptr)
            {
                footPlacementVisualizationPtr_->update(observation_);
            }
            if (sphereVisualizationPtr_ != nullptr)
            {
                sphereVisualizationPtr_->update(observation_);
            }
        }

        // Compute target trajectory
        target_manager_->update(observation_);
        // Update the current state of the system
        mpc_mrt_interface_->setCurrentObservation(observation_);
    }

    void CtrlComponent::init()
    {
        if (mpc_running_ == false)
        {
            const TargetTrajectories target_trajectories({observation_.time},
                                                         {observation_.state},
                                                         {observation_.input});

            // Set the first observation and command and wait for optimization to finish
            mpc_mrt_interface_->setCurrentObservation(observation_);
            mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);
            RCLCPP_INFO(node_->get_logger(), "Waiting for the initial policy ...");
            while (!mpc_mrt_interface_->initialPolicyReceived())
            {
                mpc_mrt_interface_->advanceMpc();
                rclcpp::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
            }
            RCLCPP_INFO(node_->get_logger(), "Initial policy has been received.");

            mpc_running_ = true;
        }
    }

    void CtrlComponent::requestStanceMode()
    {
        ctrl_interfaces_.control_inputs_.lx = 0.0;
        ctrl_interfaces_.control_inputs_.ly = 0.0;
        ctrl_interfaces_.control_inputs_.rx = 0.0;
        ctrl_interfaces_.control_inputs_.ry = 0.0;
        ctrl_interfaces_.control_inputs_.command = 2;
        if (target_manager_ != nullptr)
        {
            target_manager_->clearVelocityCommand();
        }
    }

    bool CtrlComponent::requestGaitMode(const std::string& gait_name)
    {
        std::vector<std::string> gaitNames;
        loadData::loadStdVector(gait_file_, "list", gaitNames, false);
        const auto it = std::find(gaitNames.begin(), gaitNames.end(), gait_name);
        if (it == gaitNames.end())
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[FootholdSequence] request_gait failed; gait=%s not found in %s",
                         gait_name.c_str(), gait_file_.c_str());
            return false;
        }

        ctrl_interfaces_.control_inputs_.lx = 0.0;
        ctrl_interfaces_.control_inputs_.ly = 0.0;
        ctrl_interfaces_.control_inputs_.rx = 0.0;
        ctrl_interfaces_.control_inputs_.ry = 0.0;
        ctrl_interfaces_.control_inputs_.command = static_cast<int>(std::distance(gaitNames.begin(), it)) + 2;
        if (target_manager_ != nullptr)
        {
            target_manager_->clearVelocityCommand();
        }
        return true;
    }

    bool CtrlComponent::consumeFootholdSequenceAdvanceRequest()
    {
        if (foothold_sequence_advance_requested_.exchange(false))
        {
            return true;
        }
        if (ctrl_interfaces_.control_inputs_.command != kFootholdSequenceAdvanceCommand)
        {
            return false;
        }

        ctrl_interfaces_.control_inputs_.lx = 0.0;
        ctrl_interfaces_.control_inputs_.ly = 0.0;
        ctrl_interfaces_.control_inputs_.rx = 0.0;
        ctrl_interfaces_.control_inputs_.ry = 0.0;
        ctrl_interfaces_.control_inputs_.command = 0;
        return true;
    }

    void CtrlComponent::setupLeggedInterface()
    {
        if (enable_perceptive_)
        {
            legged_interface_ = std::make_unique<PerceptiveLeggedInterface>(
                task_file_, urdf_file_, reference_file_, fixed_foothold_region_settings_,
                fixed_foothold_sequence_config_);
            RCLCPP_INFO(node_->get_logger(),
                        "[CtrlComponent] enable_perceptive=true, created PerceptiveLeggedInterface");
        }
        else
        {
            legged_interface_ = std::make_unique<LeggedInterface>(task_file_, urdf_file_, reference_file_);
            RCLCPP_INFO(node_->get_logger(),
                        "[CtrlComponent] enable_perceptive=false, created LeggedInterface");
        }

        legged_interface_->setupJointNames(joint_names_, feet_names_);
        legged_interface_->setupOptimalControlProblem(task_file_, urdf_file_, reference_file_, verbose_);

        if (enable_perceptive_)
        {
            footPlacementVisualizationPtr_ = std::make_unique<FootPlacementVisualization>(
                *dynamic_cast<PerceptiveLeggedReferenceManager&>(*legged_interface_->getReferenceManagerPtr()).
                getConvexRegionSelectorPtr(),
                legged_interface_->getCentroidalModelInfo().numThreeDofContacts, node_);

            const auto sphereInterfacePtr =
                dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getPinocchioSphereInterfacePtr();
            if (sphereInterfacePtr != nullptr)
            {
                sphereVisualizationPtr_ = std::make_unique<SphereVisualization>(
                    legged_interface_->getPinocchioInterface(), legged_interface_->getCentroidalModelInfo(),
                    *sphereInterfacePtr, node_);
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[CtrlComponent] PinocchioSphereInterface unavailable, skipping sphere visualization");
            }
        }
    }

    FixedFootholdRegionSettings CtrlComponent::loadFixedFootholdRegionSettings()
    {
        FixedFootholdRegionSettings settings = defaultFixedFootholdRegionSettings();

        auto getBoolParam = [this](const std::string& name, bool defaultValue)
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, defaultValue);
            }
            return node_->get_parameter(name).as_bool();
        };

        auto getStringParam = [this](const std::string& name, const std::string& defaultValue)
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, defaultValue);
            }
            return node_->get_parameter(name).as_string();
        };

        auto getDoubleParam = [this](const std::string& name, double defaultValue)
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, defaultValue);
            }
            return node_->get_parameter(name).as_double();
        };

        settings.enable = getBoolParam("fixed_foothold_regions.enable", settings.enable);
        settings.frame = getStringParam("fixed_foothold_regions.frame", settings.frame);

        for (auto& region : settings.regions)
        {
            const std::string prefix = "fixed_foothold_regions." + std::string(region.name) + ".";
            region.xMin = getDoubleParam(prefix + "x_min", region.xMin);
            region.xMax = getDoubleParam(prefix + "x_max", region.xMax);
            region.yMin = getDoubleParam(prefix + "y_min", region.yMin);
            region.yMax = getDoubleParam(prefix + "y_max", region.yMax);
            region.z = getDoubleParam(prefix + "z", region.z);
        }

        RCLCPP_INFO(node_->get_logger(),
                    "[FixedFootholdRegions] enable=%d frame=%s",
                    static_cast<int>(settings.enable), settings.frame.c_str());
        if (settings.enable && settings.frame != "world")
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[FixedFootholdRegions] frame=%s is configured, but only world-frame fixed regions "
                        "are currently implemented.",
                        settings.frame.c_str());
        }
        for (size_t leg = 0; leg < settings.regions.size(); ++leg)
        {
            const auto& region = settings.regions[leg];
            RCLCPP_INFO(node_->get_logger(),
                        "[FixedFootholdRegions] leg=%zu name=%s x[%.3f,%.3f] y[%.3f,%.3f] z=%.3f",
                        leg, region.name, region.xMin, region.xMax, region.yMin, region.yMax, region.z);
        }

        return settings;
    }

    FixedFootholdSequenceConfig CtrlComponent::loadFixedFootholdSequenceConfig(const std::string& config_file)
    {
        FixedFootholdSequenceConfig config = defaultFixedFootholdSequenceConfig();

        auto printSummary = [&]()
        {
            RCLCPP_INFO(node_->get_logger(),
                        "[FootholdSequence] enable=%s frame=%s sets=%zu active_set=%zu "
                        "manual_advance=%s advance_key=%s pause_gait_on_set_completed=%s "
                        "resume_gait_after_advance=%s auto_stance_on_final=%s "
                        "require_z_for_reached=%s z_tolerance=%.3f stable_hold_time=%.3f",
                        boolString(config.enable), config.frame.c_str(), config.sets.size(), config.activeSet,
                        boolString(config.manualAdvance), config.advanceKey.c_str(),
                        boolString(config.pauseGaitOnSetCompleted),
                        config.resumeGaitAfterAdvance.c_str(),
                        boolString(config.autoStanceOnFinal),
                        boolString(config.requireZForReached), config.zTolerance,
                        config.stableHoldTime);
            for (size_t setIndex = 0; setIndex < config.sets.size(); ++setIndex)
            {
                const auto& set = config.sets[setIndex];
                for (size_t leg = 0; leg < set.regions.size(); ++leg)
                {
                    const auto& region = set.regions[leg];
                    RCLCPP_INFO(node_->get_logger(),
                                "[FootholdSequence] set=%zu name=%s leg=%zu name=%s "
                                "x[%.3f,%.3f] y[%.3f,%.3f] z=%.3f",
                                setIndex, set.name.c_str(), leg, region.name,
                                region.xMin, region.xMax, region.yMin, region.yMax, region.z);
                }
            }
        };

        auto getBoolParam = [this](const std::string& name, bool defaultValue)
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, defaultValue);
            }
            return node_->get_parameter(name).as_bool();
        };

        auto getStringParam = [this](const std::string& name, const std::string& defaultValue)
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, defaultValue);
            }
            return node_->get_parameter(name).as_string();
        };

        auto getDoubleParam = [this](const std::string& name, double defaultValue)
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, defaultValue);
            }
            return node_->get_parameter(name).as_double();
        };

        auto getSizeParam = [this](const std::string& name, size_t defaultValue)
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, static_cast<int64_t>(defaultValue));
            }
            return static_cast<size_t>(node_->get_parameter(name).as_int());
        };

        std::vector<std::string> setNames;
        if (!node_->has_parameter("fixed_foothold_sequence.set_names"))
        {
            node_->declare_parameter("fixed_foothold_sequence.set_names", setNames);
        }
        setNames = node_->get_parameter("fixed_foothold_sequence.set_names").as_string_array();

        if (!setNames.empty())
        {
            config.enable = getBoolParam("fixed_foothold_sequence.enable", config.enable);
            config.frame = getStringParam("fixed_foothold_sequence.frame", config.frame);
            config.manualAdvance = getBoolParam("fixed_foothold_sequence.manual_advance",
                                                config.manualAdvance);
            config.advanceKey = getStringParam("fixed_foothold_sequence.advance_key", config.advanceKey);
            config.pauseGaitOnSetCompleted =
                getBoolParam("fixed_foothold_sequence.pause_gait_on_set_completed",
                             config.pauseGaitOnSetCompleted);
            config.resumeGaitAfterAdvance =
                getStringParam("fixed_foothold_sequence.resume_gait_after_advance",
                               config.resumeGaitAfterAdvance);
            config.autoStanceOnFinal = getBoolParam("fixed_foothold_sequence.auto_stance_on_final",
                                                    config.autoStanceOnFinal);
            config.requireZForReached = getBoolParam("fixed_foothold_sequence.require_z_for_reached",
                                                     config.requireZForReached);
            config.zTolerance = getDoubleParam("fixed_foothold_sequence.z_tolerance",
                                               config.zTolerance);
            config.stableHoldTime = getDoubleParam("fixed_foothold_sequence.stable_hold_time",
                                                   config.stableHoldTime);
            config.activeSet = getSizeParam("fixed_foothold_sequence.active_set", config.activeSet);

            bool valid = true;
            for (size_t setIndex = 0; setIndex < setNames.size(); ++setIndex)
            {
                FixedFootholdRegionSet set = defaultFixedFootholdRegionSet(setNames[setIndex]);
                for (size_t leg = 0; leg < kLegNames.size(); ++leg)
                {
                    auto& region = set.regions[leg];
                    region.name = kLegNames[leg];
                    const std::string prefix = "fixed_foothold_sequence." + setNames[setIndex] + "." +
                        std::string(kLegNames[leg]) + ".";
                    const std::array<const char*, 5> fields = {"x_min", "x_max", "y_min", "y_max", "z"};
                    bool legValid = true;
                    for (const auto* field : fields)
                    {
                        if (!node_->has_parameter(prefix + field))
                        {
                            RCLCPP_ERROR(node_->get_logger(),
                                         "[FootholdSequence] set=%zu name=%s leg=%s missing field=%s; "
                                         "disabling sequence",
                                         setIndex, set.name.c_str(), kLegNames[leg], field);
                            valid = false;
                            legValid = false;
                        }
                    }
                    if (!legValid)
                    {
                        continue;
                    }
                    region.xMin = getDoubleParam(prefix + "x_min", region.xMin);
                    region.xMax = getDoubleParam(prefix + "x_max", region.xMax);
                    region.yMin = getDoubleParam(prefix + "y_min", region.yMin);
                    region.yMax = getDoubleParam(prefix + "y_max", region.yMax);
                    region.z = getDoubleParam(prefix + "z", region.z);
                    if (region.xMin >= region.xMax || region.yMin >= region.yMax)
                    {
                        RCLCPP_ERROR(node_->get_logger(),
                                     "[FootholdSequence] set=%zu name=%s leg=%zu name=%s invalid region "
                                     "x[%.3f,%.3f] y[%.3f,%.3f]; disabling sequence",
                                     setIndex, set.name.c_str(), leg, region.name,
                                     region.xMin, region.xMax, region.yMin, region.yMax);
                        valid = false;
                    }
                }
                config.sets.push_back(set);
            }

            if (config.enable && config.sets.empty())
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "[FootholdSequence] enable=true but sets is empty; disabling sequence");
                valid = false;
            }
            if (config.activeSet >= config.sets.size() && !config.sets.empty())
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "[FootholdSequence] active_set=%zu is outside sets size=%zu; falling back to 0",
                             config.activeSet, config.sets.size());
                config.activeSet = 0;
            }
            if (config.enable && config.frame != "world")
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[FootholdSequence] frame=%s is configured, but only world-frame fixed regions "
                            "are currently implemented; treating regions as world-frame",
                            config.frame.c_str());
            }
            if (!valid)
            {
                config.enable = false;
            }

            printSummary();
            if (config.enable)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "[FootholdSequence] sequence mode enabled; single fixed_foothold_regions will be ignored");
            }
            return config;
        }

        try
        {
            const YAML::Node root = YAML::LoadFile(config_file);
            YAML::Node params = root["ocs2_quadruped_controller"] &&
                                root["ocs2_quadruped_controller"]["ros__parameters"]
                                    ? root["ocs2_quadruped_controller"]["ros__parameters"]
                                    : root;
            const YAML::Node sequence = params["fixed_foothold_sequence"];
            if (!sequence)
            {
                printSummary();
                return config;
            }

            if (sequence["enable"])
            {
                config.enable = sequence["enable"].as<bool>();
            }
            if (sequence["frame"])
            {
                config.frame = sequence["frame"].as<std::string>();
            }
            if (sequence["manual_advance"])
            {
                config.manualAdvance = sequence["manual_advance"].as<bool>();
            }
            if (sequence["advance_key"])
            {
                config.advanceKey = sequence["advance_key"].as<std::string>();
            }
            if (sequence["pause_gait_on_set_completed"])
            {
                config.pauseGaitOnSetCompleted = sequence["pause_gait_on_set_completed"].as<bool>();
            }
            if (sequence["resume_gait_after_advance"])
            {
                config.resumeGaitAfterAdvance = sequence["resume_gait_after_advance"].as<std::string>();
            }
            if (sequence["auto_stance_on_final"])
            {
                config.autoStanceOnFinal = sequence["auto_stance_on_final"].as<bool>();
            }
            if (sequence["require_z_for_reached"])
            {
                config.requireZForReached = sequence["require_z_for_reached"].as<bool>();
            }
            if (sequence["z_tolerance"])
            {
                config.zTolerance = sequence["z_tolerance"].as<double>();
            }
            if (sequence["stable_hold_time"])
            {
                config.stableHoldTime = sequence["stable_hold_time"].as<double>();
            }
            if (sequence["active_set"])
            {
                config.activeSet = sequence["active_set"].as<size_t>();
            }

            auto parseSet = [&](const YAML::Node& setNode, const std::string& fallbackName,
                                size_t setIndex, bool& valid) -> FixedFootholdRegionSet
            {
                FixedFootholdRegionSet set = defaultFixedFootholdRegionSet(fallbackName);
                if (setNode["name"])
                {
                    set.name = setNode["name"].as<std::string>();
                }
                for (size_t leg = 0; leg < kLegNames.size(); ++leg)
                {
                    const std::string legName = kLegNames[leg];
                    const YAML::Node legNode = setNode[legName];
                    if (!legNode)
                    {
                        RCLCPP_ERROR(node_->get_logger(),
                                     "[FootholdSequence] set=%zu name=%s missing leg=%s; disabling sequence",
                                     setIndex, set.name.c_str(), legName.c_str());
                        valid = false;
                        continue;
                    }

                    const std::array<const char*, 5> fields = {"x_min", "x_max", "y_min", "y_max", "z"};
                    for (const auto* field : fields)
                    {
                        if (!legNode[field])
                        {
                            RCLCPP_ERROR(node_->get_logger(),
                                         "[FootholdSequence] set=%zu name=%s leg=%s missing field=%s; "
                                         "disabling sequence",
                                         setIndex, set.name.c_str(), legName.c_str(), field);
                            valid = false;
                        }
                    }
                    if (!valid)
                    {
                        continue;
                    }

                    auto& region = set.regions[leg];
                    region.name = kLegNames[leg];
                    region.xMin = legNode["x_min"].as<double>();
                    region.xMax = legNode["x_max"].as<double>();
                    region.yMin = legNode["y_min"].as<double>();
                    region.yMax = legNode["y_max"].as<double>();
                    region.z = legNode["z"].as<double>();
                    if (region.xMin >= region.xMax || region.yMin >= region.yMax)
                    {
                        RCLCPP_ERROR(node_->get_logger(),
                                     "[FootholdSequence] set=%zu name=%s leg=%zu name=%s invalid region "
                                     "x[%.3f,%.3f] y[%.3f,%.3f]; disabling sequence",
                                     setIndex, set.name.c_str(), leg, region.name,
                                     region.xMin, region.xMax, region.yMin, region.yMax);
                        valid = false;
                    }
                }
                return set;
            };

            bool valid = true;
            if (sequence["sets"] && sequence["sets"].IsSequence())
            {
                for (size_t setIndex = 0; setIndex < sequence["sets"].size(); ++setIndex)
                {
                    config.sets.push_back(parseSet(
                        sequence["sets"][setIndex],
                        "set" + std::to_string(setIndex),
                        setIndex,
                        valid));
                }
            }
            else if (sequence["set_names"] && sequence["set_names"].IsSequence())
            {
                for (size_t setIndex = 0; setIndex < sequence["set_names"].size(); ++setIndex)
                {
                    const std::string setName = sequence["set_names"][setIndex].as<std::string>();
                    const YAML::Node setNode = sequence[setName];
                    if (!setNode)
                    {
                        RCLCPP_ERROR(node_->get_logger(),
                                     "[FootholdSequence] set_names[%zu]=%s has no matching set block; "
                                     "disabling sequence",
                                     setIndex, setName.c_str());
                        valid = false;
                        continue;
                    }
                    config.sets.push_back(parseSet(setNode, setName, setIndex, valid));
                    config.sets.back().name = setName;
                }
            }

            if (config.enable && config.sets.empty())
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "[FootholdSequence] enable=true but sets is empty; disabling sequence");
                valid = false;
            }
            if (config.activeSet >= config.sets.size() && !config.sets.empty())
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "[FootholdSequence] active_set=%zu is outside sets size=%zu; falling back to 0",
                             config.activeSet, config.sets.size());
                config.activeSet = 0;
            }
            if (config.enable && config.frame != "world")
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[FootholdSequence] frame=%s is configured, but only world-frame fixed regions "
                            "are currently implemented; treating regions as world-frame",
                            config.frame.c_str());
            }
            if (!valid)
            {
                config.enable = false;
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[FootholdSequence] failed to read %s: %s; disabling sequence",
                         config_file.c_str(), e.what());
            config.enable = false;
            config.sets.clear();
        }

        printSummary();
        if (config.enable)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "[FootholdSequence] sequence mode enabled; single fixed_foothold_regions will be ignored");
        }
        return config;
    }

    /**
     * Set up the SQP MPC, Gait Manager and Reference Manager
     */
    void CtrlComponent::setupMpc()
    {
        mpc_ = std::make_shared<SqpMpc>(legged_interface_->mpcSettings(),
                                        legged_interface_->sqpSettings(),
                                        legged_interface_->getOptimalControlProblem(),
                                        legged_interface_->getInitializer());

        // Initialize the reference manager
        const auto gait_manager_ptr = std::make_shared<GaitManager>(
            ctrl_interfaces_,
            legged_interface_->getSwitchedModelReferenceManagerPtr()->
                               getGaitSchedule());
        gait_manager_ptr->init(gait_file_);
        mpc_->getSolverPtr()->addSynchronizedModule(gait_manager_ptr);
        mpc_->getSolverPtr()->setReferenceManager(legged_interface_->getReferenceManagerPtr());

        target_manager_ = std::make_unique<TargetManager>(ctrl_interfaces_,
                                                          node_,
                                                          legged_interface_->getReferenceManagerPtr(),
                                                          task_file_,
                                                          reference_file_);

        if (enable_perceptive_)
        {
            const auto planarTerrainReceiver =
                std::make_shared<PlanarTerrainReceiver>(
                    node_, dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getPlanarTerrainPtr(),
                    dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getSignedDistanceFieldPtr(),
                    "/convex_plane_decomposition_ros/planar_terrain", "elevation");
            mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
        }
    }

    void CtrlComponent::setupMrt()
    {
        mpc_mrt_interface_ = std::make_unique<MPC_MRT_Interface>(*mpc_);
        mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());
        mpc_timer_.reset();

        controller_running_ = true;
        mpc_thread_ = std::thread([&]
        {
            while (controller_running_)
            {
                try
                {
                    executeAndSleep(
                        [&]
                        {
                            if (mpc_running_)
                            {
                                mpc_timer_.startTimer();
                                mpc_mrt_interface_->advanceMpc();
                                mpc_timer_.endTimer();
                            }
                        },
                        legged_interface_->mpcSettings().mpcDesiredFrequency_);
                }
                catch (const std::exception& e)
                {
                    controller_running_ = false;
                    RCLCPP_WARN(node_->get_logger(), "[Ocs2 MPC thread] Error : %s", e.what());
                }
            }
        });
        setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpc_thread_);
        RCLCPP_INFO(node_->get_logger(), "MRT initialized. MPC thread started.");
    }
}
