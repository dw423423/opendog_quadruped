//
// Created by biao on 3/21/25.
//

#include <ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_quadruped_controller/perceptive/interface/FixedFootholdRegions.h>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition/GeometryUtils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>

namespace ocs2::legged_robot
{
    namespace
    {
        convex_plane_decomposition::CgalPolygon2d makeFixedTargetRegion(const FixedFootholdRegion& region,
                                                                        size_t numVertices)
        {
            convex_plane_decomposition::CgalPolygon2d polygon;
            numVertices = std::max<size_t>(4, numVertices);
            for (size_t i = 0; i < numVertices; ++i)
            {
                const scalar_t phase = 4.0 * static_cast<scalar_t>(i) / static_cast<scalar_t>(numVertices);
                scalar_t x = region.xMax;
                scalar_t y = region.yMax;
                if (phase < 1.0)
                {
                    x = region.xMax + phase * (region.xMin - region.xMax);
                }
                else if (phase < 2.0)
                {
                    x = region.xMin;
                    y = region.yMax + (phase - 1.0) * (region.yMin - region.yMax);
                }
                else if (phase < 3.0)
                {
                    x = region.xMin + (phase - 2.0) * (region.xMax - region.xMin);
                    y = region.yMin;
                }
                else
                {
                    y = region.yMin + (phase - 3.0) * (region.yMax - region.yMin);
                }
                polygon.push_back(convex_plane_decomposition::CgalPoint2d(x, y));
            }
            return polygon;
        }

        std::string polygonToString(const convex_plane_decomposition::CgalPolygon2d& polygon)
        {
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(3) << "[";
            for (size_t i = 0; i < polygon.size(); ++i)
            {
                if (i > 0)
                {
                    stream << ", ";
                }
                const auto vertex = polygon.vertex(static_cast<int>(i));
                stream << "(" << vertex.x() << "," << vertex.y() << ")";
            }
            stream << "]";
            return stream.str();
        }

        scalar_t signedPolygonArea(const convex_plane_decomposition::CgalPolygon2d& polygon)
        {
            if (polygon.size() < 3)
            {
                return 0.0;
            }

            scalar_t doubleArea = 0.0;
            for (size_t i = 0; i < polygon.size(); ++i)
            {
                const auto a = polygon.vertex(static_cast<int>(i));
                const auto b = polygon.vertex(static_cast<int>((i + 1) % polygon.size()));
                doubleArea += a.x() * b.y() - b.x() * a.y();
            }
            return 0.5 * doubleArea;
        }

        bool polygonHasFiniteVertices(const convex_plane_decomposition::CgalPolygon2d& polygon)
        {
            for (size_t i = 0; i < polygon.size(); ++i)
            {
                const auto vertex = polygon.vertex(static_cast<int>(i));
                if (!std::isfinite(vertex.x()) || !std::isfinite(vertex.y()))
                {
                    return false;
                }
            }
            return true;
        }

        scalar_t minPolygonEdgeLength(const convex_plane_decomposition::CgalPolygon2d& polygon)
        {
            if (polygon.size() < 2)
            {
                return 0.0;
            }

            scalar_t minLength = std::numeric_limits<scalar_t>::infinity();
            for (size_t i = 0; i < polygon.size(); ++i)
            {
                const auto a = polygon.vertex(static_cast<int>(i));
                const auto b = polygon.vertex(static_cast<int>((i + 1) % polygon.size()));
                const scalar_t dx = b.x() - a.x();
                const scalar_t dy = b.y() - a.y();
                minLength = std::min(minLength, std::sqrt(dx * dx + dy * dy));
            }
            return minLength;
        }

        bool polygonHasConsistentOrientation(const convex_plane_decomposition::CgalPolygon2d& polygon)
        {
            if (polygon.size() < 3)
            {
                return false;
            }

            int sign = 0;
            constexpr scalar_t kCrossTolerance = 1e-10;
            for (size_t i = 0; i < polygon.size(); ++i)
            {
                const auto a = polygon.vertex(static_cast<int>(i));
                const auto b = polygon.vertex(static_cast<int>((i + 1) % polygon.size()));
                const auto c = polygon.vertex(static_cast<int>((i + 2) % polygon.size()));
                const scalar_t abx = b.x() - a.x();
                const scalar_t aby = b.y() - a.y();
                const scalar_t bcx = c.x() - b.x();
                const scalar_t bcy = c.y() - b.y();
                const scalar_t cross = abx * bcy - aby * bcx;
                if (std::abs(cross) <= kCrossTolerance)
                {
                    continue;
                }

                const int currentSign = cross > 0.0 ? 1 : -1;
                if (sign == 0)
                {
                    sign = currentSign;
                }
                else if (sign != currentSign)
                {
                    return false;
                }
            }
            return sign != 0;
        }

        bool isPointInsideConvexPolygon(const convex_plane_decomposition::CgalPoint2d& point,
                                        const convex_plane_decomposition::CgalPolygon2d& polygon)
        {
            if (polygon.size() < 3)
            {
                return false;
            }

            int sign = 0;
            constexpr scalar_t kInsideTolerance = 1e-9;
            for (size_t i = 0; i < polygon.size(); ++i)
            {
                const auto a = polygon.vertex(static_cast<int>(i));
                const auto b = polygon.vertex(static_cast<int>((i + 1) % polygon.size()));
                const scalar_t cross =
                    (b.x() - a.x()) * (point.y() - a.y()) - (b.y() - a.y()) * (point.x() - a.x());
                if (std::abs(cross) <= kInsideTolerance)
                {
                    continue;
                }

                const int currentSign = cross > 0.0 ? 1 : -1;
                if (sign == 0)
                {
                    sign = currentSign;
                }
                else if (sign != currentSign)
                {
                    return false;
                }
            }
            return true;
        }

        struct PolygonValidation
        {
            bool valid = false;
            bool finite = false;
            bool orientationConsistent = false;
            scalar_t signedArea = 0.0;
            scalar_t area = 0.0;
            scalar_t minEdgeLength = 0.0;
        };

        PolygonValidation validateConvexPolygon(const convex_plane_decomposition::CgalPolygon2d& polygon)
        {
            constexpr scalar_t kMinPolygonArea = 1e-4;
            constexpr scalar_t kMinEdgeLength = 1e-4;
            PolygonValidation validation;
            validation.signedArea = signedPolygonArea(polygon);
            validation.area = std::abs(validation.signedArea);
            validation.finite = polygonHasFiniteVertices(polygon);
            validation.minEdgeLength = minPolygonEdgeLength(polygon);
            validation.orientationConsistent = polygonHasConsistentOrientation(polygon);
            validation.valid = polygon.size() >= 3 && validation.area > kMinPolygonArea &&
                validation.finite && validation.minEdgeLength > kMinEdgeLength &&
                validation.orientationConsistent;
            return validation;
        }

        std::string bboxToString(const convex_plane_decomposition::CgalBbox2d& bbox)
        {
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(3)
                << "x[" << bbox.xmin() << "," << bbox.xmax() << "]"
                << ",y[" << bbox.ymin() << "," << bbox.ymax() << "]";
            return stream.str();
        }

        std::string stairRawBoundsToString(const StairFootholdRegionSettings& settings, size_t stepIndex, size_t leg)
        {
            const scalar_t xMin = settings.stepXStart + static_cast<scalar_t>(stepIndex) * settings.stepDepth;
            const scalar_t xMax = xMin + settings.stepDepth;
            const scalar_t halfWidth = 0.5 * settings.stepWidth;
            const bool leftLeg = leg == 0 || leg == 2;
            const scalar_t laneYMin = leftLeg ? settings.stairYCenter : settings.stairYCenter - halfWidth;
            const scalar_t laneYMax = leftLeg ? settings.stairYCenter + halfWidth : settings.stairYCenter;
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(3)
                << "x[" << xMin << "," << xMax << "]"
                << ",y[" << laneYMin << "," << laneYMax << "]"
                << ",z=" << static_cast<scalar_t>(stepIndex + 1) * settings.stepHeight;
            return stream.str();
        }

        bool shouldLogFootholdRegion(scalar_t initTime)
        {
            static scalar_t lastLogTime = -1.0;
            if (lastLogTime >= 0.0 && initTime - lastLogTime < 1.0)
            {
                return false;
            }
            lastLogTime = initTime;
            return true;
        }

        const char* legName(size_t leg)
        {
            constexpr std::array<const char*, 4> names = {"FL", "FR", "RL", "RR"};
            return leg < names.size() ? names[leg] : "UNKNOWN";
        }

        bool isLeftLeg(size_t leg)
        {
            return leg == 0 || leg == 2;
        }

        convex_plane_decomposition::CgalPolygon2d makeRectangle(double xMin, double xMax,
                                                                double yMin, double yMax)
        {
            convex_plane_decomposition::CgalPolygon2d rectangle;
            rectangle.push_back(convex_plane_decomposition::CgalPoint2d(xMax, yMax));
            rectangle.push_back(convex_plane_decomposition::CgalPoint2d(xMin, yMax));
            rectangle.push_back(convex_plane_decomposition::CgalPoint2d(xMin, yMin));
            rectangle.push_back(convex_plane_decomposition::CgalPoint2d(xMax, yMin));
            return rectangle;
        }

        std::pair<scalar_t, scalar_t> legLaneYBounds(const StairFootholdRegionSettings& settings, size_t leg)
        {
            const scalar_t halfWidth = 0.5 * settings.stepWidth;
            if (isLeftLeg(leg))
            {
                return {settings.stairYCenter, settings.stairYCenter + halfWidth};
            }
            return {settings.stairYCenter - halfWidth, settings.stairYCenter};
        }

        std::optional<size_t> findStairStepByXY(const StairFootholdRegionSettings& settings,
                                                size_t leg, scalar_t x, scalar_t y)
        {
            const auto [laneYMin, laneYMax] = legLaneYBounds(settings, leg);
            if (y < laneYMin || y > laneYMax)
            {
                return std::nullopt;
            }

            for (size_t reverseIndex = 0; reverseIndex < settings.numSteps; ++reverseIndex)
            {
                const size_t stepIndex = settings.numSteps - 1 - reverseIndex;
                const scalar_t xMin = settings.stepXStart + static_cast<scalar_t>(stepIndex) * settings.stepDepth;
                const scalar_t xMax = xMin + settings.stepDepth;
                if (x >= xMin && x <= xMax)
                {
                    return stepIndex;
                }
            }
            return std::nullopt;
        }

        scalar_t stairStepHeight(const StairFootholdRegionSettings& settings, size_t stepIndex)
        {
            return static_cast<scalar_t>(stepIndex + 1) * settings.stepHeight;
        }

        bool makeStairTopRegion(const StairFootholdRegionSettings& settings, size_t stepIndex, size_t leg,
                                scalar_t edgeMarginX, scalar_t edgeMarginY,
                                convex_plane_decomposition::PlanarRegion& region)
        {
            const scalar_t xMinRaw = settings.stepXStart + static_cast<scalar_t>(stepIndex) * settings.stepDepth;
            const scalar_t xMaxRaw = xMinRaw + settings.stepDepth;
            const auto [laneYMinRaw, laneYMaxRaw] = legLaneYBounds(settings, leg);

            const scalar_t xMin = xMinRaw + edgeMarginX;
            const scalar_t xMax = xMaxRaw - edgeMarginX;
            const scalar_t yMin = laneYMinRaw + edgeMarginY;
            const scalar_t yMax = laneYMaxRaw - edgeMarginY;
            if (xMin >= xMax || yMin >= yMax)
            {
                return false;
            }

            region.transformPlaneToWorld.setIdentity();
            region.transformPlaneToWorld.translation().z() = stairStepHeight(settings, stepIndex);
            region.bbox2d = convex_plane_decomposition::CgalBbox2d(xMin, yMin, xMax, yMax);

            convex_plane_decomposition::CgalPolygonWithHoles2d boundary;
            boundary.outer_boundary() = makeRectangle(xMin, xMax, yMin, yMax);
            region.boundaryWithInset.boundary = boundary;
            region.boundaryWithInset.insets.clear();
            region.boundaryWithInset.insets.push_back(boundary);
            return true;
        }

        convex_plane_decomposition::PlanarTerrainProjection projectToStairTopRegion(
            const vector3_t& nominalFoothold, const convex_plane_decomposition::PlanarRegion& region)
        {
            convex_plane_decomposition::PlanarTerrainProjection projection;
            const scalar_t x = std::clamp(nominalFoothold.x(), region.bbox2d.xmin(), region.bbox2d.xmax());
            const scalar_t y = std::clamp(nominalFoothold.y(), region.bbox2d.ymin(), region.bbox2d.ymax());
            projection.regionPtr = &region;
            projection.positionInTerrainFrame = convex_plane_decomposition::CgalPoint2d(x, y);
            projection.positionInWorld =
                convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                    projection.positionInTerrainFrame, region.transformPlaneToWorld);
            projection.cost = (nominalFoothold - projection.positionInWorld).squaredNorm();
            return projection;
        }

        struct KinematicPenaltyResult
        {
            scalar_t penalty = 0.0;
            scalar_t reach = 0.0;
            bool crossesBody = false;
            bool hardReachExceeded = false;
            vector3_t candidateInBase = vector3_t::Zero();
        };

        struct FootholdProjectionCandidate
        {
            bool valid = false;
            std::string invalidReason;
            std::string regionType;
            int regionIndex = -1;
            int stairStepIndex = -1;
            std::string stairLane = "none";
            vector3_t nominalWorld = vector3_t::Zero();
            convex_plane_decomposition::PlanarTerrainProjection projection;
            convex_plane_decomposition::PlanarRegion stairRegion;
            scalar_t distanceCost = 0.0;
            scalar_t kinematicPenalty = 0.0;
            scalar_t totalCost = std::numeric_limits<scalar_t>::infinity();
            scalar_t reach = 0.0;
            bool crossesBody = false;
            bool hardReachExceeded = false;
            vector3_t candidateInBase = vector3_t::Zero();
        };

        vector3_t transformWorldToBaseYawOnly(const vector3_t& pointWorld, const vector_t& state,
                                              const CentroidalModelInfo& info)
        {
            const auto basePose = centroidal_model::getBasePose(state, info);
            const vector3_t basePosition = basePose.head(3);
            const scalar_t yaw = basePose(3);
            const scalar_t c = std::cos(yaw);
            const scalar_t s = std::sin(yaw);
            const vector3_t delta = pointWorld - basePosition;
            return (vector3_t() << c * delta.x() + s * delta.y(),
                    -s * delta.x() + c * delta.y(),
                    delta.z()).finished();
        }

        KinematicPenaltyResult computeKinematicPenalty(size_t leg, const vector3_t& candidateWorld,
                                                       const vector_t& state, const CentroidalModelInfo& info,
                                                       const PerceptiveFootholdSettings& settings)
        {
            KinematicPenaltyResult result;
            result.candidateInBase = transformWorldToBaseYawOnly(candidateWorld, state, info);
            result.reach = result.candidateInBase.norm();
            if (result.reach > settings.maxReachSoft)
            {
                const scalar_t excess = result.reach - settings.maxReachSoft;
                result.penalty += settings.reachPenaltyWeight * excess * excess;
            }
            result.hardReachExceeded = result.reach > settings.maxReachHard;

            if (isLeftLeg(leg))
            {
                if (result.candidateInBase.y() < settings.minLeftFootY)
                {
                    result.crossesBody = true;
                    const scalar_t crossingDistance = settings.minLeftFootY - result.candidateInBase.y();
                    result.penalty += settings.crossBodyPenaltyWeight * crossingDistance * crossingDistance;
                }
            }
            else
            {
                if (result.candidateInBase.y() > settings.maxRightFootY)
                {
                    result.crossesBody = true;
                    const scalar_t crossingDistance = result.candidateInBase.y() - settings.maxRightFootY;
                    result.penalty += settings.crossBodyPenaltyWeight * crossingDistance * crossingDistance;
                }
            }

            return result;
        }

        scalar_t weightedDistanceCost(const vector3_t& nominalWorld, const vector3_t& projectionWorld,
                                      const PerceptiveFootholdSettings& settings)
        {
            const vector3_t delta = projectionWorld - nominalWorld;
            return settings.distanceWeightX * delta.x() * delta.x() +
                settings.distanceWeightY * delta.y() * delta.y() +
                settings.distanceWeightZ * delta.z() * delta.z();
        }

        FootholdProjectionCandidate makePlanarCandidate(
            const vector3_t& nominalWorld, size_t leg, int regionIndex,
            const convex_plane_decomposition::PlanarRegion& region,
            const vector_t& initState, const CentroidalModelInfo& info,
            const PerceptiveFootholdSettings& settings)
        {
            FootholdProjectionCandidate candidate;
            candidate.regionType = "perceptive";
            candidate.regionIndex = regionIndex;
            candidate.nominalWorld = nominalWorld;
            const vector3_t nominalInTerrain = region.transformPlaneToWorld.inverse() * nominalWorld;
            candidate.projection.regionPtr = &region;
            candidate.projection.positionInTerrainFrame =
                convex_plane_decomposition::projectToPlanarRegion(
                    convex_plane_decomposition::CgalPoint2d(nominalInTerrain.x(), nominalInTerrain.y()), region);
            candidate.projection.positionInWorld =
                convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                    candidate.projection.positionInTerrainFrame, region.transformPlaneToWorld);
            candidate.distanceCost = weightedDistanceCost(nominalWorld, candidate.projection.positionInWorld, settings);
            const auto kinematic = computeKinematicPenalty(leg, candidate.projection.positionInWorld,
                                                           initState, info, settings);
            candidate.kinematicPenalty = kinematic.penalty;
            candidate.totalCost = candidate.distanceCost + candidate.kinematicPenalty;
            candidate.reach = kinematic.reach;
            candidate.crossesBody = kinematic.crossesBody;
            candidate.hardReachExceeded = kinematic.hardReachExceeded;
            candidate.candidateInBase = kinematic.candidateInBase;
            candidate.valid = !candidate.hardReachExceeded;
            candidate.invalidReason = candidate.hardReachExceeded ? "max_reach_hard_exceeded" : "";
            candidate.projection.cost = candidate.totalCost;
            return candidate;
        }

        FootholdProjectionCandidate makeStairCandidate(
            const vector3_t& nominalWorld, size_t leg, size_t stepIndex,
            const StairFootholdRegionSettings& stairSettings,
            const vector_t& initState, const CentroidalModelInfo& info,
            const PerceptiveFootholdSettings& settings)
        {
            FootholdProjectionCandidate candidate;
            candidate.regionType = "stair_top";
            candidate.regionIndex = static_cast<int>(stepIndex);
            candidate.stairStepIndex = static_cast<int>(stepIndex);
            candidate.stairLane = isLeftLeg(leg) ? "left" : "right";
            candidate.nominalWorld = nominalWorld;
            if (!makeStairTopRegion(stairSettings, stepIndex, leg,
                                    stairSettings.edgeMarginX, stairSettings.edgeMarginY,
                                    candidate.stairRegion))
            {
                candidate.valid = false;
                candidate.invalidReason = "stair_margin_region_invalid";
                return candidate;
            }

            candidate.projection = projectToStairTopRegion(nominalWorld, candidate.stairRegion);
            candidate.distanceCost = weightedDistanceCost(nominalWorld, candidate.projection.positionInWorld, settings);
            const auto kinematic = computeKinematicPenalty(leg, candidate.projection.positionInWorld,
                                                           initState, info, settings);
            candidate.kinematicPenalty = kinematic.penalty;
            candidate.totalCost = candidate.distanceCost + candidate.kinematicPenalty;
            candidate.reach = kinematic.reach;
            candidate.crossesBody = kinematic.crossesBody;
            candidate.hardReachExceeded = kinematic.hardReachExceeded;
            candidate.candidateInBase = kinematic.candidateInBase;
            candidate.valid = !candidate.hardReachExceeded;
            candidate.invalidReason = candidate.hardReachExceeded ? "max_reach_hard_exceeded" : "";
            candidate.projection.cost = candidate.totalCost;
            return candidate;
        }

        std::string candidateToString(const FootholdProjectionCandidate& candidate)
        {
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(3)
                << "valid=" << (candidate.valid ? "true" : "false")
                << " type=" << candidate.regionType
                << " region_index=" << candidate.regionIndex
                << " stair_step=" << (candidate.stairStepIndex >= 0
                                           ? std::to_string(candidate.stairStepIndex + 1)
                                           : std::string("none"))
                << " stair_lane=" << candidate.stairLane
                << " projection_world=(" << candidate.projection.positionInWorld.x()
                << "," << candidate.projection.positionInWorld.y()
                << "," << candidate.projection.positionInWorld.z() << ")"
                << " projection_terrain=(" << candidate.projection.positionInTerrainFrame.x()
                << "," << candidate.projection.positionInTerrainFrame.y() << ")"
                << " distance_cost=" << candidate.distanceCost
                << " kinematic_penalty=" << candidate.kinematicPenalty
                << " total_cost=" << candidate.totalCost
                << " reach=" << candidate.reach
                << " candidate_base=(" << candidate.candidateInBase.x()
                << "," << candidate.candidateInBase.y()
                << "," << candidate.candidateInBase.z() << ")"
                << " crosses_body=" << (candidate.crossesBody ? "true" : "false")
                << " hard_reach_exceeded=" << (candidate.hardReachExceeded ? "true" : "false");
            if (!candidate.invalidReason.empty())
            {
                stream << " invalid_reason=" << candidate.invalidReason;
            }
            return stream.str();
        }

        convex_plane_decomposition::CgalPolygon2d growConvexRegion(
            const convex_plane_decomposition::PlanarTerrainProjection& projection, size_t numVertices)
        {
            constexpr scalar_t growthFactor = 1.05;
            return convex_plane_decomposition::growConvexPolygonInsideShape(
                projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame,
                numVertices, growthFactor);
        }

        void nudgeProjectionTowardInterior(const vector3_t& nominalFoothold,
                                           convex_plane_decomposition::PlanarTerrainProjection& projection)
        {
            if (projection.regionPtr == nullptr)
            {
                return;
            }

            const vector3_t nominalInTerrain =
                projection.regionPtr->transformPlaneToWorld.inverse() * nominalFoothold;
            Eigen::Matrix<scalar_t, 2, 1> nominal2d(nominalInTerrain.x(), nominalInTerrain.y());
            Eigen::Matrix<scalar_t, 2, 1> projected2d(
                projection.positionInTerrainFrame.x(), projection.positionInTerrainFrame.y());
            Eigen::Matrix<scalar_t, 2, 1> direction = projected2d - nominal2d;
            const scalar_t distance = direction.norm();
            if (distance < 1e-6)
            {
                return;
            }

            constexpr scalar_t nudgeDistance = 0.01;
            direction /= distance;
            const convex_plane_decomposition::CgalPoint2d nudgedPoint(
                projection.positionInTerrainFrame.x() + nudgeDistance * direction.x(),
                projection.positionInTerrainFrame.y() + nudgeDistance * direction.y());
            if (!convex_plane_decomposition::isInside(
                    nudgedPoint, projection.regionPtr->boundaryWithInset.boundary))
            {
                return;
            }

            projection.positionInTerrainFrame = nudgedPoint;
            projection.positionInWorld =
                convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                    nudgedPoint, projection.regionPtr->transformPlaneToWorld);
        }
    }

    ConvexRegionSelector::ConvexRegionSelector(CentroidalModelInfo info,
                                               std::shared_ptr<convex_plane_decomposition::PlanarTerrain>
                                               planarTerrainPtr,
                                               const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                               size_t numVertices,
                                               FixedFootholdRegionSettings fixedFootholdRegionSettings,
                                               FixedFootholdSequenceConfig fixedFootholdSequenceConfig,
                                               StairFootholdRegionSettings stairFootholdRegionSettings,
                                               PerceptiveFootholdSettings perceptiveFootholdSettings)
        : info_(std::move(info)),
          numVertices_(numVertices),
          planarTerrainPtr_(std::move(planarTerrainPtr)),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          fixedFootholdRegionSettings_(std::move(fixedFootholdRegionSettings)),
          fixedFootholdSequenceManager_(std::move(fixedFootholdSequenceConfig)),
          stairFootholdRegionSettings_(std::move(stairFootholdRegionSettings)),
          perceptiveFootholdSettings_(std::move(perceptiveFootholdSettings))
    {
    }

    convex_plane_decomposition::PlanarTerrainProjection ConvexRegionSelector::getProjection(
        size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
        return feetProjections_[leg][index];
    }

    convex_plane_decomposition::CgalPolygon2d ConvexRegionSelector::getConvexPolygon(size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
        return convexPolygons_[leg][index];
    }

    vector3_t ConvexRegionSelector::getNominalFootholds(size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
        return nominalFootholds_[leg][index];
    }

    void ConvexRegionSelector::update(const ModeSchedule& modeSchedule, scalar_t initTime, const vector_t& initState,
                                      TargetTrajectories& targetTrajectories)
    {
        planarTerrain_ = *planarTerrainPtr_;
        // Need copy storage it since PlanarTerrainProjection.regionPtr is a pointer
        const auto& modeSequence = modeSchedule.modeSequence;
        const auto& eventTimes = modeSchedule.eventTimes;
        const auto contactFlagStocks = extractContactFlags(modeSequence);
        const size_t numPhases = modeSequence.size();
        const bool logFootholdRegion = shouldLogFootholdRegion(initTime);

        // Find start and final index of time for legs
        feet_array_t<std::vector<int>> startIndices;
        feet_array_t<std::vector<int>> finalIndices;
        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            startIndices[leg] = std::vector<int>(numPhases, 0);
            finalIndices[leg] = std::vector<int>(numPhases, 0);
            const bool hasSwingPhase = std::any_of(contactFlagStocks[leg].begin(), contactFlagStocks[leg].end(),
                                                   [](bool contact) { return !contact; });
            if (!hasSwingPhase)
            {
                continue;
            }
            // find the startTime and finalTime indices for swing feet
            for (size_t i = 0; i < numPhases; i++)
            {
                // skip if it is a stance leg
                if (contactFlagStocks[leg][i])
                {
                    std::tie(startIndices[leg][i], finalIndices[leg][i]) = findIndex(i, contactFlagStocks[leg]);
                }
            }
        }

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            feetProjections_[leg].clear();
            convexPolygons_[leg].clear();
            nominalFootholds_[leg].clear();
            stairTopRegions_[leg].clear();
            feetProjections_[leg].resize(numPhases);
            convexPolygons_[leg].resize(numPhases);
            nominalFootholds_[leg].resize(numPhases);
            stairTopRegions_[leg].resize(numPhases);
            middleTimes_[leg].clear();
            initStandFinalTime_[leg] = 0;
            const bool hasSwingPhase = std::any_of(contactFlagStocks[leg].begin(), contactFlagStocks[leg].end(),
                                                   [](bool contact) { return !contact; });
            if (!hasSwingPhase)
            {
                initStandFinalTime_[leg] = std::numeric_limits<scalar_t>::infinity();
                if (logFootholdRegion)
                {
                    std::cerr << "[FootholdRegion] leg=" << leg
                        << " has no swing phase; fixed foothold constraint disabled for this schedule"
                        << std::endl;
                }
                continue;
            }

            scalar_t lastStandMiddleTime = NAN;
            // Stand leg foot
            for (size_t i = 0; i < numPhases; ++i)
            {
                if (contactFlagStocks[leg][i])
                {
                    const int standStartIndex = startIndices[leg][i];
                    const int standFinalIndex = finalIndices[leg][i];
                    const scalar_t standStartTime = eventTimes[standStartIndex];
                    const scalar_t standFinalTime = eventTimes[standFinalIndex];
                    const scalar_t standMiddleTime = standStartTime + (standFinalTime - standStartTime) / 2;

                    if (!numerics::almost_eq(standMiddleTime, lastStandMiddleTime))
                    {
                        vector3_t footPos = getNominalFoothold(leg, standMiddleTime, initState, targetTrajectories);
                        auto penaltyFunction = [](const vector3_t& /*projectedPoint*/) { return 0.0; };
                        const bool useFixedRegion = fixedFootholdRegionsEnabled();
                        const auto& fixedRegion = getFixedFootholdRegion(leg);
                        std::string selectedRegion = "perceptive";
                        std::optional<size_t> candidateStep;
                        std::optional<size_t> nominalStepFootprint;
                        bool stairTopFallback = false;
                        bool selectedStairTop = false;
                        size_t candidateCount = 0;
                        std::string selectedCandidateSummary;
                        convex_plane_decomposition::PlanarTerrainProjection projection;
                        convex_plane_decomposition::PlanarTerrainProjection originalProjection;
                        convex_plane_decomposition::CgalPolygon2d convexRegion;

                        if (useFixedRegion)
                        {
                            const vector3_t targetCenter(
                                0.5 * (fixedRegion.xMin + fixedRegion.xMax),
                                0.5 * (fixedRegion.yMin + fixedRegion.yMax),
                                fixedRegion.z);
                            projection = getBestPlanarRegionAtPositionInWorld(
                                targetCenter, planarTerrain_.planarRegions, penaltyFunction);
                            if (projection.regionPtr == nullptr)
                            {
                                std::cerr << std::fixed << std::setprecision(3)
                                    << "[FootholdRegion][WARN] leg=" << legName(leg)
                                    << " leg_index=" << leg
                                    << " phase_index=" << i
                                    << " phase_start=" << standStartTime
                                    << " phase_end=" << standFinalTime
                                    << " fixed_region=true"
                                    << " target_center=(" << targetCenter.x() << ","
                                    << targetCenter.y() << "," << targetCenter.z() << ")"
                                    << " has no planar region projection; skipping fixed region constraint"
                                    << std::endl;
                                continue;
                            }
                            const vector3_t targetCenterInTerrain =
                                projection.regionPtr->transformPlaneToWorld.inverse() * targetCenter;
                            projection.positionInTerrainFrame = convex_plane_decomposition::CgalPoint2d(
                                targetCenterInTerrain.x(), targetCenterInTerrain.y());
                            projection.positionInWorld = targetCenter;
                            convexRegion = makeFixedTargetRegion(fixedRegion, numVertices_);
                            selectedRegion = fixedFootholdRegionToString(leg);
                            if (logFootholdRegion && fixedFootholdSequenceManager_.isEnabled())
                            {
                                std::cerr << std::fixed << std::setprecision(3)
                                    << "[FootholdSequence] selector active_set="
                                    << fixedFootholdSequenceManager_.getActiveSetIndex()
                                    << " set_name=" << fixedFootholdSequenceManager_.getActiveSetName()
                                    << " leg=" << leg
                                    << " name=" << fixedRegion.name
                                    << " region=x[" << fixedRegion.xMin << "," << fixedRegion.xMax << "]"
                                    << ",y[" << fixedRegion.yMin << "," << fixedRegion.yMax << "]"
                                    << ",z=" << fixedRegion.z
                                    << std::endl;
                            }
                        }
                        else
                        {
                            if (perceptiveFootholdSettings_.projectionCandidateSearchEnable)
                            {
                                if (stairFootholdRegionSettings_.enable)
                                {
                                    nominalStepFootprint = findStairStepByXY(
                                        stairFootholdRegionSettings_, leg, footPos.x(), footPos.y());
                                }
                                std::vector<FootholdProjectionCandidate> candidates;
                                candidates.reserve(planarTerrain_.planarRegions.size() +
                                    (stairFootholdRegionSettings_.enable ? stairFootholdRegionSettings_.numSteps : 0));
                                for (size_t regionIndex = 0; regionIndex < planarTerrain_.planarRegions.size();
                                     ++regionIndex)
                                {
                                    candidates.push_back(makePlanarCandidate(
                                        footPos, leg, static_cast<int>(regionIndex),
                                        planarTerrain_.planarRegions[regionIndex],
                                        initState, info_, perceptiveFootholdSettings_));
                                }
                                if (stairFootholdRegionSettings_.enable)
                                {
                                    for (size_t stepIndex = 0; stepIndex < stairFootholdRegionSettings_.numSteps;
                                         ++stepIndex)
                                    {
                                        candidates.push_back(makeStairCandidate(
                                            footPos, leg, stepIndex, stairFootholdRegionSettings_,
                                            initState, info_, perceptiveFootholdSettings_));
                                    }
                                }

                                candidateCount = candidates.size();
                                auto bestCandidateIt = candidates.end();
                                for (auto it = candidates.begin(); it != candidates.end(); ++it)
                                {
                                    if (!it->valid)
                                    {
                                        continue;
                                    }
                                    if (bestCandidateIt == candidates.end() ||
                                        it->totalCost < bestCandidateIt->totalCost)
                                    {
                                        bestCandidateIt = it;
                                    }
                                }

                                if (logFootholdRegion && perceptiveFootholdSettings_.printCandidateDebug)
                                {
                                    for (size_t candidateIndex = 0; candidateIndex < candidates.size();
                                         ++candidateIndex)
                                    {
                                        std::cerr << std::fixed << std::setprecision(3)
                                            << "[FootholdCandidate] leg=" << legName(leg)
                                            << " leg_index=" << leg
                                            << " phase_index=" << i
                                            << " candidate_index=" << candidateIndex
                                            << " " << candidateToString(candidates[candidateIndex])
                                            << std::endl;
                                    }
                                }

                                if (bestCandidateIt == candidates.end())
                                {
                                    std::cerr << std::fixed << std::setprecision(3)
                                        << "[FootholdCandidateSearch][WARN] leg=" << legName(leg)
                                        << " leg_index=" << leg
                                        << " phase_index=" << i
                                        << " phase_start=" << standStartTime
                                        << " phase_end=" << standFinalTime
                                        << " nominal=(" << footPos.x() << "," << footPos.y() << ","
                                        << footPos.z() << ")"
                                        << " candidate_count=" << candidateCount
                                        << " no_valid_candidate=true"
                                        << " note=no_last_valid_fallback_no_polygon_replacement"
                                        << std::endl;
                                    if (logFootholdRegion)
                                    {
                                        for (size_t candidateIndex = 0; candidateIndex < candidates.size();
                                             ++candidateIndex)
                                        {
                                            std::cerr << std::fixed << std::setprecision(3)
                                                << "[FootholdCandidateSearch][INVALID] leg=" << legName(leg)
                                                << " leg_index=" << leg
                                                << " phase_index=" << i
                                                << " candidate_index=" << candidateIndex
                                                << " " << candidateToString(candidates[candidateIndex])
                                                << std::endl;
                                        }
                                    }
                                    continue;
                                }

                                selectedCandidateSummary = candidateToString(*bestCandidateIt);
                                selectedRegion = bestCandidateIt->regionType;
                                selectedStairTop = selectedRegion == "stair_top";
                                if (selectedStairTop)
                                {
                                    candidateStep = static_cast<size_t>(bestCandidateIt->stairStepIndex);
                                    stairTopRegions_[leg][i] = bestCandidateIt->stairRegion;
                                    projection = bestCandidateIt->projection;
                                    projection.regionPtr = &stairTopRegions_[leg][i];
                                }
                                else
                                {
                                    projection = bestCandidateIt->projection;
                                }
                                convexRegion = growConvexRegion(projection, numVertices_);
                            }
                            else
                            {
                                originalProjection = getBestPlanarRegionAtPositionInWorld(
                                    footPos, planarTerrain_.planarRegions, penaltyFunction);
                                if (originalProjection.regionPtr == nullptr)
                                {
                                    std::cerr << std::fixed << std::setprecision(3)
                                        << "[FootholdRegion] leg=" << leg
                                        << " phase_start=" << standStartTime
                                        << " phase_end=" << standFinalTime
                                        << " nominal=(" << footPos.x() << "," << footPos.y() << ","
                                        << footPos.z() << ")"
                                        << " has no planar region projection; skipping region selection"
                                        << std::endl;
                                    continue;
                                }
                                nudgeProjectionTowardInterior(footPos, originalProjection);
                                projection = originalProjection;
                                if (stairFootholdRegionSettings_.enable)
                                {
                                    nominalStepFootprint = findStairStepByXY(
                                        stairFootholdRegionSettings_, leg, footPos.x(), footPos.y());
                                }
                                convexRegion = growConvexRegion(projection, numVertices_);
                            }
                        }

                        const auto polygonValidation = validateConvexPolygon(convexRegion);
                        const bool projectionInsidePolygon =
                            projection.regionPtr != nullptr &&
                            isPointInsideConvexPolygon(projection.positionInTerrainFrame, convexRegion);
                        if ((!polygonValidation.valid || !projectionInsidePolygon) && logFootholdRegion)
                        {
                            std::cerr << std::fixed << std::setprecision(3)
                                << "[FootholdRegionValidate][WARN] leg=" << legName(leg)
                                << " leg_index=" << leg
                                << " phase_index=" << i
                                << " phase_start=" << standStartTime
                                << " phase_end=" << standFinalTime
                                << " phase_mid=" << standMiddleTime
                                << " selected_region=" << selectedRegion
                                << " polygon_valid=" << (polygonValidation.valid ? "true" : "false")
                                << " vertex_count=" << convexRegion.size()
                                << " area=" << polygonValidation.area
                                << " signed_area=" << polygonValidation.signedArea
                                << " finite=" << (polygonValidation.finite ? "true" : "false")
                                << " min_edge_length=" << polygonValidation.minEdgeLength
                                << " orientation_consistent="
                                << (polygonValidation.orientationConsistent ? "true" : "false")
                                << " projection_inside_polygon="
                                << (projectionInsidePolygon ? "true" : "false")
                                << " projection_terrain=(" << projection.positionInTerrainFrame.x()
                                << "," << projection.positionInTerrainFrame.y() << ")"
                                << " projection_world=(" << projection.positionInWorld.x()
                                << "," << projection.positionInWorld.y()
                                << "," << projection.positionInWorld.z() << ")"
                                << " vertices=" << polygonToString(convexRegion)
                                << " note=no_fallback_no_polygon_replacement"
                                << std::endl;
                        }

                        feetProjections_[leg][i] = projection;
                        convexPolygons_[leg][i] = convexRegion;
                        nominalFootholds_[leg][i] = footPos;
                        middleTimes_[leg].push_back(standMiddleTime);
                        if (logFootholdRegion)
                        {
                            std::cerr << std::fixed << std::setprecision(3)
                                << "[FootholdRegionSelect] leg=" << legName(leg)
                                << " leg_index=" << leg
                                << " phase_index=" << i
                                << " phase_start=" << standStartTime
                                << " phase_end=" << standFinalTime
                                << " phase_mid=" << standMiddleTime
                                << " nominal=(" << footPos.x() << "," << footPos.y() << "," << footPos.z() << ")"
                                << " candidate_search_enable="
                                << (perceptiveFootholdSettings_.projectionCandidateSearchEnable ? "true" : "false")
                                << " candidate_count=" << candidateCount
                                << " inside_step_footprint=" << (nominalStepFootprint.has_value() ? "true" : "false")
                                << " candidate_step="
                                << (nominalStepFootprint.has_value()
                                        ? std::to_string(nominalStepFootprint.value() + 1)
                                        : std::string("none"))
                                << " selected_region=" << selectedRegion
                                << " selected_step="
                                << (selectedStairTop && candidateStep.has_value()
                                        ? std::to_string(candidateStep.value() + 1)
                                        : std::string("none"))
                                << " projection=(" << projection.positionInWorld.x() << ","
                                << projection.positionInWorld.y() << ","
                                << projection.positionInWorld.z() << ")"
                                << " projection_terrain=(" << projection.positionInTerrainFrame.x() << ","
                                << projection.positionInTerrainFrame.y() << ")"
                                << " polygon_vertices=" << convexRegion.size()
                                << " polygon_area=" << polygonValidation.area
                                << " projection_inside_polygon="
                                << (projectionInsidePolygon ? "true" : "false")
                                << " region_z="
                                << (projection.regionPtr != nullptr
                                        ? projection.regionPtr->transformPlaneToWorld.translation().z()
                                        : 0.0)
                                << " fallback=" << (stairTopFallback ? "true" : "false");
                            if (!selectedCandidateSummary.empty())
                            {
                                std::cerr << " selected_candidate={" << selectedCandidateSummary << "}";
                            }
                            if (candidateStep.has_value())
                            {
                                std::cerr
                                    << " stair_raw_region="
                                    << stairRawBoundsToString(stairFootholdRegionSettings_, candidateStep.value(), leg);
                            }
                            if (selectedStairTop)
                            {
                                std::cerr
                                    << " stair_edge_margin_x=" << stairFootholdRegionSettings_.edgeMarginX
                                    << " stair_edge_margin_y=" << stairFootholdRegionSettings_.edgeMarginY
                                    << " stair_margin_region=" << bboxToString(stairTopRegions_[leg][i].bbox2d)
                                    << " stair_margin_note=region_construction_not_grow_shrink";
                            }
                            if (useFixedRegion)
                            {
                                std::cerr
                                    << " fixed_frame="
                                    << (fixedFootholdSequenceManager_.isEnabled()
                                            ? fixedFootholdSequenceManager_.getConfig().frame
                                            : fixedFootholdRegionSettings_.frame)
                                    << " fixed_vertices_world=" << polygonToString(convexRegion)
                                    << " fixed_note=direct_polygon_source_no_ordinary_or_stair_selection";
                            }
                            std::cerr << std::endl;
                            std::cerr << std::fixed << std::setprecision(3)
                                << "[FootholdRegion] leg=" << leg
                                << " phase_index=" << i
                                << " fixed_enabled=" << static_cast<int>(fixedFootholdRegionsEnabled())
                                << " sequence_enabled="
                                << static_cast<int>(fixedFootholdSequenceManager_.isEnabled())
                                << " frame=" << (fixedFootholdSequenceManager_.isEnabled()
                                                     ? fixedFootholdSequenceManager_.getConfig().frame
                                                     : fixedFootholdRegionSettings_.frame)
                                << " selected_region=" << selectedRegion
                                << " phase_start=" << standStartTime
                                << " phase_end=" << standFinalTime
                                << " phase_mid=" << standMiddleTime
                                << " nominal=(" << footPos.x() << "," << footPos.y() << "," << footPos.z() << ")"
                                << " projection=(" << projection.positionInWorld.x() << ","
                                << projection.positionInWorld.y() << ","
                                << projection.positionInWorld.z() << ")"
                                << " projection_terrain=(" << projection.positionInTerrainFrame.x() << ","
                                << projection.positionInTerrainFrame.y() << ")"
                                << " polygon_area=" << polygonValidation.area
                                << " vertices=" << polygonToString(convexRegion);
                            if (fixedFootholdRegionsEnabled())
                            {
                                std::cerr << " inside_nominal_xy="
                                    << (isInsideFixedFootholdRegionXY(leg, footPos) ? "true" : "false");
                            }
                            std::cerr << std::endl;
                        }
                    }
                    else
                    {
                        feetProjections_[leg][i] = feetProjections_[leg][i - 1];
                        convexPolygons_[leg][i] = convexPolygons_[leg][i - 1];
                        nominalFootholds_[leg][i] = nominalFootholds_[leg][i - 1];
                    }

                    if (standStartTime < initTime && initTime < standFinalTime)
                    {
                        initStandFinalTime_[leg] = standFinalTime;
                    }
                }
            }
        }

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            timeEvents_[leg] = eventTimes;
        }
    }

    feet_array_t<std::vector<bool>> ConvexRegionSelector::extractContactFlags(
        const std::vector<size_t>& phaseIDsStock) const
    {
        const size_t numPhases = phaseIDsStock.size();

        feet_array_t<std::vector<bool>> contactFlagStock;
        std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

        for (size_t i = 0; i < numPhases; i++)
        {
            const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
            for (size_t j = 0; j < info_.numThreeDofContacts; j++)
            {
                contactFlagStock[j][i] = contactFlag[j];
            }
        }
        return contactFlagStock;
    }

    std::pair<int, int> ConvexRegionSelector::findIndex(size_t index, const std::vector<bool>& contactFlagStock)
    {
        const size_t numPhases = contactFlagStock.size();

        if (numPhases < 2 || !contactFlagStock[index])
        {
            return {0, 0};
        }

        // find the starting time
        int startTimesIndex = 0;
        for (int ip = index - 1; ip >= 0; ip--)
        {
            if (!contactFlagStock[ip])
            {
                startTimesIndex = ip;
                break;
            }
        }
        // find the final time
        int finalTimesIndex = numPhases - 2;
        for (size_t ip = index + 1; ip < numPhases; ip++)
        {
            if (!contactFlagStock[ip])
            {
                finalTimesIndex = ip - 1;
                break;
            }
        }
        return {startTimesIndex, finalTimesIndex};
    }

    vector3_t ConvexRegionSelector::getNominalFoothold(size_t leg, scalar_t time, const vector_t& initState,
                                                       TargetTrajectories& targetTrajectories)
    {
        scalar_t height = 0.4;

        vector_t desiredState = targetTrajectories.getDesiredState(time);
        vector3_t desiredVel = centroidal_model::getNormalizedMomentum(desiredState, info_).head(3);
        vector3_t measuredVel = centroidal_model::getNormalizedMomentum(initState, info_).head(3);

        auto feedback = (vector3_t() << (std::sqrt(height / 9.81) * (measuredVel - desiredVel)).head(2), 0).finished();
        vector_t zyx = centroidal_model::getBasePose(desiredState, info_).tail(3);
        scalar_t offset = tan(-zyx(1)) * height;
        vector3_t offsetVector(offset * cos(-zyx(1)), 0, offset * sin(-zyx(1)));
        matrix3_t R;
        scalar_t z = zyx(0);
        R << cos(z), -sin(z), 0, // clang-format off
             sin(z), cos(z), 0,
             0, 0, 1;  // clang-format on
        //  return endEffectorKinematicsPtr_->getPosition(targetTrajectories.getDesiredState(time))[leg] + feedback;
        return endEffectorKinematicsPtr_->getPosition(targetTrajectories.getDesiredState(time))[leg] - R.transpose() *
            offsetVector;
    }
} // namespace legged
