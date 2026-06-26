//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/visualize/FootPlacementVisualization.h"
#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include <ocs2_quadruped_controller/perceptive/interface/FixedFootholdRegions.h>

#include <array>
#include <limits>
#include <string>
#include <utility>

namespace ocs2::legged_robot
{
    namespace
    {
        visualization_msgs::msg::Marker getFixedTargetRegionMarker(const std_msgs::msg::Header& header,
                                                                    const FixedFootholdRegion& region,
                                                                    size_t leg, Color color)
        {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "Fixed Target Regions";
            marker.id = 10000 + static_cast<int>(leg);
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.012;
            marker.color = getColor(color, 1.0);
            marker.color.a = 1.0;

            const std::array<std::pair<double, double>, 5> corners = {
                std::pair<double, double>{region.xMin, region.yMin},
                std::pair<double, double>{region.xMax, region.yMin},
                std::pair<double, double>{region.xMax, region.yMax},
                std::pair<double, double>{region.xMin, region.yMax},
                std::pair<double, double>{region.xMin, region.yMin}
            };
            for (const auto& [x, y] : corners)
            {
                geometry_msgs::msg::Point point;
                point.x = x;
                point.y = y;
                point.z = region.z;
                marker.points.push_back(point);
            }
            return marker;
        }

        visualization_msgs::msg::Marker getNoStepRegionMarker(const std_msgs::msg::Header& header,
                                                              const convex_plane_decomposition::CgalPolygon2d& hole,
                                                              const Eigen::Isometry3d& transformPlaneToWorld,
                                                              size_t regionIndex)
        {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "No-Step Regions";
            marker.id = 20000 + static_cast<int>(regionIndex);
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.02;
            marker.color = getColor(Color::red, 0.9);

            if (!hole.is_empty())
            {
                marker.points.reserve(hole.size() + 1);
                for (const auto& point : hole)
                {
                    const auto pointInWorld = convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                        point, transformPlaneToWorld);
                    geometry_msgs::msg::Point pointRos;
                    pointRos.x = pointInWorld.x();
                    pointRos.y = pointInWorld.y();
                    pointRos.z = pointInWorld.z() + 0.01;
                    marker.points.push_back(pointRos);
                }
                const auto firstPointInWorld =
                    convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                        hole.vertex(0), transformPlaneToWorld);
                geometry_msgs::msg::Point firstPointRos;
                firstPointRos.x = firstPointInWorld.x();
                firstPointRos.y = firstPointInWorld.y();
                firstPointRos.z = firstPointInWorld.z() + 0.01;
                marker.points.push_back(firstPointRos);
            }

            return marker;
        }

        visualization_msgs::msg::Marker getNoStepRegionLabelMarker(const std_msgs::msg::Header& header,
                                                                   const convex_plane_decomposition::CgalPolygon2d& hole,
                                                                   const Eigen::Isometry3d& transformPlaneToWorld,
                                                                   size_t regionIndex)
        {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "No-Step Labels";
            marker.id = 21000 + static_cast<int>(regionIndex);
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.z = 0.12;
            marker.color = getColor(Color::red, 1.0);
            constexpr std::array<const char*, 3> labels = {"B", "C", "D"};
            marker.text = regionIndex < labels.size()
                              ? labels[regionIndex]
                              : std::string("No-Step ") + std::to_string(regionIndex);

            if (!hole.is_empty())
            {
                vector3_t center = vector3_t::Zero();
                for (const auto& point : hole)
                {
                    center += convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                        point, transformPlaneToWorld);
                }
                center /= static_cast<scalar_t>(hole.size());
                marker.pose.position.x = center.x();
                marker.pose.position.y = center.y();
                marker.pose.position.z = center.z() + 0.05;
            }

            return marker;
        }
    }

    FootPlacementVisualization::FootPlacementVisualization(const ConvexRegionSelector& convexRegionSelector,
                                                           size_t numFoot,
                                                           const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                                           scalar_t maxUpdateFrequency)
        : convex_region_selector_(convexRegionSelector),
          num_foot_(numFoot),
          last_time_(std::numeric_limits<scalar_t>::lowest()),
          min_publish_time_difference_(1.0 / maxUpdateFrequency)
    {
        marker_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/foot_placement", 1);
    }

    void FootPlacementVisualization::update(const SystemObservation& observation)
    {
        if (observation.time - last_time_ > min_publish_time_difference_)
        {
            last_time_ = observation.time;

            std_msgs::msg::Header header;
            //    header.stamp.fromNSec(planarTerrainPtr->gridMap.getTimestamp());
            header.frame_id = "odom";

            visualization_msgs::msg::MarkerArray makerArray;
            if (const auto planarTerrain = convex_region_selector_.getPlanarTerrainPtr())
            {
                size_t noStepRegionIndex = 0;
                for (const auto& planarRegion : planarTerrain->planarRegions)
                {
                    for (const auto& hole : planarRegion.boundaryWithInset.boundary.holes())
                    {
                        makerArray.markers.push_back(getNoStepRegionMarker(
                            header, hole, planarRegion.transformPlaneToWorld, noStepRegionIndex));
                        makerArray.markers.push_back(getNoStepRegionLabelMarker(
                            header, hole, planarRegion.transformPlaneToWorld, noStepRegionIndex));
                        ++noStepRegionIndex;
                    }
                }
            }

            if (convex_region_selector_.fixedFootholdRegionsEnabled())
            {
                const auto& settings = convex_region_selector_.getFixedFootholdRegionSettings();
                for (size_t leg = 0; leg < settings.regions.size(); ++leg)
                {
                    makerArray.markers.push_back(getFixedTargetRegionMarker(
                        header, convex_region_selector_.getFixedFootholdRegion(leg), leg, feet_color_map_[leg]));
                }
            }

            size_t i = 0;
            for (int leg = 0; leg < num_foot_; ++leg)
            {
                auto middleTimes = convex_region_selector_.getMiddleTimes(leg);

                int kStart = 0;
                for (int k = 0; k < middleTimes.size(); ++k)
                {
                    const auto projection = convex_region_selector_.getProjection(leg, middleTimes[k]);
                    if (projection.regionPtr == nullptr)
                    {
                        continue;
                    }
                    if (middleTimes[k] < observation.time)
                    {
                        kStart = k + 1;
                        continue;
                    }
                    auto color = feet_color_map_[leg];
                    float alpha = 1 - static_cast<float>(k - kStart) / static_cast<float>(middleTimes.size() - kStart);
                    // Projections
                    auto projectionMaker = getArrowAtPointMsg(
                        projection.regionPtr->transformPlaneToWorld.linear() * vector3_t(0, 0, 0.1),
                        projection.positionInWorld, color);
                    projectionMaker.header = header;
                    projectionMaker.ns = "Projections";
                    projectionMaker.id = i;
                    projectionMaker.color.a = alpha;
                    makerArray.markers.push_back(projectionMaker);

                    // Convex Region
                    const auto convexRegion = convex_region_selector_.getConvexPolygon(leg, middleTimes[k]);
                    auto convexRegionMsg =
                        convex_plane_decomposition::to3dRosPolygon(convexRegion,
                                                                   projection.regionPtr->transformPlaneToWorld, header);
                    makerArray.markers.push_back(to3dRosMarker(convexRegion,
                                                               projection.regionPtr->transformPlaneToWorld, header,
                                                               color, alpha, i));

                    // Nominal Footholds
                    const auto nominal = convex_region_selector_.getNominalFootholds(leg, middleTimes[k]);
                    auto nominalMarker = getFootMarker(nominal, true, color, foot_marker_diameter_, 1.);
                    nominalMarker.header = header;
                    nominalMarker.ns = "Nominal Footholds";
                    nominalMarker.id = i;
                    nominalMarker.color.a = alpha;
                    makerArray.markers.push_back(nominalMarker);

                    i++;
                }
            }

            marker_publisher_->publish(makerArray);
        }
    }

    visualization_msgs::msg::Marker FootPlacementVisualization::to3dRosMarker(
        const convex_plane_decomposition::CgalPolygon2d& polygon,
        const Eigen::Isometry3d& transformPlaneToWorld,
        const std_msgs::msg::Header& header, Color color, float alpha, size_t i) const
    {
        visualization_msgs::msg::Marker marker;
        marker.ns = "Convex Regions";
        marker.id = i;
        marker.header = header;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.scale.x = line_width_;
        marker.color = getColor(color, alpha);
        if (!polygon.is_empty())
        {
            marker.points.reserve(polygon.size() + 1);
            for (const auto& point : polygon)
            {
                const auto pointInWorld = convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                    point, transformPlaneToWorld);
                geometry_msgs::msg::Point point_ros;
                point_ros.x = pointInWorld.x();
                point_ros.y = pointInWorld.y();
                point_ros.z = pointInWorld.z();
                marker.points.push_back(point_ros);
            }
            // repeat the first point to close to polygon
            const auto pointInWorld =
                convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                    polygon.vertex(0), transformPlaneToWorld);
            geometry_msgs::msg::Point point_ros;
            point_ros.x = pointInWorld.x();
            point_ros.y = pointInWorld.y();
            point_ros.z = pointInWorld.z();
            marker.points.push_back(point_ros);
        }
        marker.pose.orientation.w = 1.0;
        return marker;
    }
} // namespace legged
