//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/visualize/FootPlacementVisualization.h"
#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include <ocs2_quadruped_controller/perceptive/interface/FixedFootholdRegions.h>

#include <array>
#include <cmath>
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

        bool almostEqual(double lhs, double rhs)
        {
            return std::abs(lhs - rhs) < 1e-6;
        }

        bool isDefaultNoStepHole(const convex_plane_decomposition::CgalPolygon2d& hole)
        {
            if (hole.is_empty())
            {
                return false;
            }

            struct Bounds
            {
                double xMin;
                double xMax;
                double yMin;
                double yMax;
            };

            constexpr std::array<Bounds, 3> noStepBounds = {
                Bounds{-1.80, -1.60, -1.30, 1.30},
                Bounds{0.85, 1.60, 0.70, 1.45},
                Bounds{1.20, 1.50, -1.85, -0.40},
            };
            const auto bbox = hole.bbox();
            for (const auto& bounds : noStepBounds)
            {
                if (almostEqual(bbox.xmin(), bounds.xMin) &&
                    almostEqual(bbox.xmax(), bounds.xMax) &&
                    almostEqual(bbox.ymin(), bounds.yMin) &&
                    almostEqual(bbox.ymax(), bounds.yMax))
                {
                    return true;
                }
            }
            return false;
        }

        bool isGazeboStairTopRegion(const convex_plane_decomposition::PlanarRegion& planarRegion)
        {
            struct StairTop
            {
                double xMin;
                double xMax;
                double yMin;
                double yMax;
                double z;
            };

            constexpr std::array<StairTop, 2> stairTops = {
                StairTop{0.40, 0.75, -0.40, 0.40, 0.15},
                StairTop{0.75, 1.10, -0.40, 0.40, 0.30},
            };
            const auto& boundary = planarRegion.boundaryWithInset.boundary.outer_boundary();
            if (boundary.is_empty())
            {
                return false;
            }

            const auto bbox = boundary.bbox();
            const auto z = planarRegion.transformPlaneToWorld.translation().z();
            for (const auto& stairTop : stairTops)
            {
                if (almostEqual(bbox.xmin(), stairTop.xMin) &&
                    almostEqual(bbox.xmax(), stairTop.xMax) &&
                    almostEqual(bbox.ymin(), stairTop.yMin) &&
                    almostEqual(bbox.ymax(), stairTop.yMax) &&
                    almostEqual(z, stairTop.z))
                {
                    return true;
                }
            }
            return false;
        }

        geometry_msgs::msg::Point toRosPoint(const convex_plane_decomposition::CgalPoint2d& point,
                                             const Eigen::Isometry3d& transformPlaneToWorld,
                                             scalar_t zOffset = 0.0)
        {
            const auto pointInWorld = convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                point, transformPlaneToWorld);
            geometry_msgs::msg::Point pointRos;
            pointRos.x = pointInWorld.x();
            pointRos.y = pointInWorld.y();
            pointRos.z = pointInWorld.z() + zOffset;
            return pointRos;
        }

        visualization_msgs::msg::Marker getStairSteppableSurfaceMarker(
            const std_msgs::msg::Header& header,
            const convex_plane_decomposition::CgalPolygon2d& polygon,
            const Eigen::Isometry3d& transformPlaneToWorld,
            size_t stairIndex)
        {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "Stair Steppable Surfaces";
            marker.id = 22000 + static_cast<int>(stairIndex);
            marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.color = getColor(Color::green, 0.22);

            if (polygon.size() >= 3)
            {
                for (size_t i = 1; i + 1 < polygon.size(); ++i)
                {
                    marker.points.push_back(toRosPoint(polygon.vertex(0), transformPlaneToWorld, 0.012));
                    marker.points.push_back(toRosPoint(polygon.vertex(static_cast<int>(i)), transformPlaneToWorld, 0.012));
                    marker.points.push_back(toRosPoint(polygon.vertex(static_cast<int>(i + 1)), transformPlaneToWorld, 0.012));
                }
            }
            return marker;
        }

        visualization_msgs::msg::Marker getStairSteppableOutlineMarker(
            const std_msgs::msg::Header& header,
            const convex_plane_decomposition::CgalPolygon2d& polygon,
            const Eigen::Isometry3d& transformPlaneToWorld,
            size_t stairIndex)
        {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "Stair Steppable Regions";
            marker.id = 23000 + static_cast<int>(stairIndex);
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.018;
            marker.color = getColor(Color::green, 1.0);

            if (!polygon.is_empty())
            {
                marker.points.reserve(polygon.size() + 1);
                for (const auto& point : polygon)
                {
                    marker.points.push_back(toRosPoint(point, transformPlaneToWorld, 0.018));
                }
                marker.points.push_back(toRosPoint(polygon.vertex(0), transformPlaneToWorld, 0.018));
            }
            return marker;
        }

        visualization_msgs::msg::Marker getStairSteppableLabelMarker(
            const std_msgs::msg::Header& header,
            const convex_plane_decomposition::CgalPolygon2d& polygon,
            const Eigen::Isometry3d& transformPlaneToWorld,
            size_t stairIndex)
        {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "Stair Steppable Labels";
            marker.id = 24000 + static_cast<int>(stairIndex);
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.z = 0.10;
            marker.color = getColor(Color::green, 1.0);
            marker.text = std::string("Step ") + std::to_string(stairIndex + 1);

            if (!polygon.is_empty())
            {
                vector3_t center = vector3_t::Zero();
                for (const auto& point : polygon)
                {
                    const auto pointInWorld = convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                        point, transformPlaneToWorld);
                    center += pointInWorld;
                }
                center /= static_cast<scalar_t>(polygon.size());
                marker.pose.position.x = center.x();
                marker.pose.position.y = center.y();
                marker.pose.position.z = center.z() + 0.08;
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
                size_t stairRegionIndex = 0;
                for (const auto& planarRegion : planarTerrain->planarRegions)
                {
                    if (isGazeboStairTopRegion(planarRegion))
                    {
                        const auto& stairTop = planarRegion.boundaryWithInset.boundary.outer_boundary();
                        makerArray.markers.push_back(getStairSteppableSurfaceMarker(
                            header, stairTop, planarRegion.transformPlaneToWorld, stairRegionIndex));
                        makerArray.markers.push_back(getStairSteppableOutlineMarker(
                            header, stairTop, planarRegion.transformPlaneToWorld, stairRegionIndex));
                        makerArray.markers.push_back(getStairSteppableLabelMarker(
                            header, stairTop, planarRegion.transformPlaneToWorld, stairRegionIndex));
                        ++stairRegionIndex;
                    }

                    for (const auto& hole : planarRegion.boundaryWithInset.boundary.holes())
                    {
                        if (!isDefaultNoStepHole(hole))
                        {
                            continue;
                        }
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
