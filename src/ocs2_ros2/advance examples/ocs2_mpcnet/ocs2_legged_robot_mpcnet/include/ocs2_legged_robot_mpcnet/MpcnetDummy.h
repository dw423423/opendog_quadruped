//
// Created by tlab-uav on 8/27/24.
//

#ifndef MPCNETDUMMY_H
#define MPCNETDUMMY_H

#include <rclcpp/rclcpp.hpp>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#ifdef OCS2_LEGGED_ROBOT_MPCNET_HAS_RAISIM
#include <grid_map_msgs/msg/grid_map.hpp>
#include <ocs2_legged_robot_raisim/RaiSimConversions.h>
#include <ocs2_legged_robot_raisim/RaiSimVisualizer.h>
#include <ocs2_raisim_core/RaisimRollout.h>
#endif

#include <ocs2_mpcnet_core/dummy/MpcnetDummyObserverRos.h>
#include <ocs2_mpcnet_core/control/MpcnetOnnxController.h>

namespace ocs2::legged_robot {
    class MpcNetDummy final : public rclcpp::Node {
#ifdef OCS2_LEGGED_ROBOT_MPCNET_HAS_RAISIM
        void publishGridMap(const std::string &frameId) const;
#endif

        std::string robotName_;
        std::string taskFile_;
        std::string urdfFile_;
        std::string referenceFile_;
        std::string raisimFile_;
        std::string resourcePath_;
        std::string policyFile_;
        bool useRaisim;

#ifdef OCS2_LEGGED_ROBOT_MPCNET_HAS_RAISIM
        std::shared_ptr<LeggedRobotRaisimConversions> conversions_;
        std::shared_ptr<raisim::HeightMap> terrainPtr_;

        rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmapPublisher_;
#endif

    public:
        explicit MpcNetDummy(const std::string &robot_name);

        ~MpcNetDummy() override = default;

        void init();

        std::shared_ptr<LeggedRobotInterface> interface_;
        std::unique_ptr<RolloutBase> rollout_;
        std::shared_ptr<LeggedRobotVisualizer> visualizer_;

        std::shared_ptr<GaitReceiver> gaitReceiver_;
        std::unique_ptr<mpcnet::MpcnetOnnxController> controller_;
        std::shared_ptr<mpcnet::MpcnetDummyObserverRos> dummyObserverRos_;
        std::shared_ptr<RosReferenceManager> rosReferenceManager_;
    };
}


#endif //MPCNETDUMMY_H
