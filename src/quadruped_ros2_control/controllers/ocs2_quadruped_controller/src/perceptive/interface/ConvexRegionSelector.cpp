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
                                               StairFootholdRegionSettings stairFootholdRegionSettings)
        : info_(std::move(info)),
          numVertices_(numVertices),
          planarTerrainPtr_(std::move(planarTerrainPtr)),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          fixedFootholdRegionSettings_(std::move(fixedFootholdRegionSettings)),
          fixedFootholdSequenceManager_(std::move(fixedFootholdSequenceConfig)),
          stairFootholdRegionSettings_(std::move(stairFootholdRegionSettings))
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
                        auto originalProjection = getBestPlanarRegionAtPositionInWorld(
                            footPos, planarTerrain_.planarRegions, penaltyFunction);
                        if (originalProjection.regionPtr == nullptr)
                        {
                            std::cerr << std::fixed << std::setprecision(3)
                                << "[FootholdRegion] leg=" << leg
                                << " phase_start=" << standStartTime
                                << " phase_end=" << standFinalTime
                                << " nominal=(" << footPos.x() << "," << footPos.y() << "," << footPos.z() << ")"
                                << " has no planar region projection; skipping region selection"
                                << std::endl;
                            continue;
                        }
                        const bool useFixedRegion = fixedFootholdRegionsEnabled();
                        nudgeProjectionTowardInterior(footPos, originalProjection);
                        auto projection = originalProjection;
                        const auto& fixedRegion = getFixedFootholdRegion(leg);
                        std::string selectedRegion = "perceptive";
                        std::optional<size_t> candidateStep;
                        bool stairTopFallback = false;
                        bool selectedStairTop = false;
                        if (!useFixedRegion && stairFootholdRegionSettings_.enable &&
                            stairFootholdRegionSettings_.preferStairTopWhenInsideFootprint)
                        {
                            candidateStep = findStairStepByXY(stairFootholdRegionSettings_, leg,
                                                              footPos.x(), footPos.y());
                            if (candidateStep.has_value())
                            {
                                auto& stairTopRegion = stairTopRegions_[leg][i];
                                if (makeStairTopRegion(stairFootholdRegionSettings_, candidateStep.value(), leg,
                                                       stairFootholdRegionSettings_.edgeMarginX,
                                                       stairFootholdRegionSettings_.edgeMarginY,
                                                       stairTopRegion))
                                {
                                    projection = projectToStairTopRegion(footPos, stairTopRegion);
                                    selectedRegion = "stair_top";
                                    selectedStairTop = true;
                                }
                                else
                                {
                                    stairTopFallback = true;
                                    if (logFootholdRegion)
                                    {
                                        std::cerr << std::fixed << std::setprecision(3)
                                            << "[FootholdRegionSelect][WARN] leg=" << legName(leg)
                                            << " candidate_step=" << (candidateStep.value() + 1)
                                            << " stair_top_failed=true fallback=original"
                                            << " reason=shrunken_region_invalid"
                                            << std::endl;
                                    }
                                }
                            }
                        }

                        auto convexRegion = growConvexRegion(projection, numVertices_);
                        if (selectedStairTop && convexRegion.size() < 3)
                        {
                            auto& stairTopRegion = stairTopRegions_[leg][i];
                            if (makeStairTopRegion(stairFootholdRegionSettings_, candidateStep.value(), leg,
                                                   0.5 * stairFootholdRegionSettings_.edgeMarginX,
                                                   0.5 * stairFootholdRegionSettings_.edgeMarginY,
                                                   stairTopRegion))
                            {
                                projection = projectToStairTopRegion(footPos, stairTopRegion);
                                convexRegion = growConvexRegion(projection, numVertices_);
                            }
                            if (convexRegion.size() < 3)
                            {
                                projection = originalProjection;
                                convexRegion = growConvexRegion(projection, numVertices_);
                                selectedRegion = "perceptive";
                                selectedStairTop = false;
                                stairTopFallback = true;
                                if (logFootholdRegion)
                                {
                                    std::cerr << std::fixed << std::setprecision(3)
                                        << "[FootholdRegionSelect][WARN] leg=" << legName(leg)
                                        << " candidate_step=" << (candidateStep.value() + 1)
                                        << " stair_top_failed=true fallback=original"
                                        << " reason=empty_active_region"
                                        << std::endl;
                                }
                            }
                        }
                        {
                            if (useFixedRegion)
                            {
                                const vector3_t targetCenter(
                                    0.5 * (fixedRegion.xMin + fixedRegion.xMax),
                                    0.5 * (fixedRegion.yMin + fixedRegion.yMax),
                                    fixedRegion.z);
                                projection = getBestPlanarRegionAtPositionInWorld(
                                    targetCenter, planarTerrain_.planarRegions, penaltyFunction);
                                if (projection.regionPtr != nullptr)
                                {
                                    const vector3_t targetCenterInTerrain =
                                        projection.regionPtr->transformPlaneToWorld.inverse() * targetCenter;
                                    projection.positionInTerrainFrame = convex_plane_decomposition::CgalPoint2d(
                                        targetCenterInTerrain.x(), targetCenterInTerrain.y());
                                    projection.positionInWorld = targetCenter;
                                }
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
                        }

                        feetProjections_[leg][i] = projection;
                        convexPolygons_[leg][i] = convexRegion;
                        nominalFootholds_[leg][i] = footPos;
                        middleTimes_[leg].push_back(standMiddleTime);
                        if (logFootholdRegion)
                        {
                            std::cerr << std::fixed << std::setprecision(3)
                                << "[FootholdRegionSelect] leg=" << legName(leg)
                                << " nominal=(" << footPos.x() << "," << footPos.y() << "," << footPos.z() << ")"
                                << " inside_step_footprint=" << (candidateStep.has_value() ? "true" : "false")
                                << " candidate_step="
                                << (candidateStep.has_value()
                                        ? std::to_string(candidateStep.value() + 1)
                                        : std::string("none"))
                                << " selected_region=" << selectedRegion
                                << " selected_step="
                                << (selectedStairTop && candidateStep.has_value()
                                        ? std::to_string(candidateStep.value() + 1)
                                        : std::string("none"))
                                << " projection=(" << projection.positionInWorld.x() << ","
                                << projection.positionInWorld.y() << ","
                                << projection.positionInWorld.z() << ")"
                                << " region_z="
                                << (projection.regionPtr != nullptr
                                        ? projection.regionPtr->transformPlaneToWorld.translation().z()
                                        : 0.0)
                                << " fallback=" << (stairTopFallback ? "true" : "false")
                                << std::endl;
                            std::cerr << std::fixed << std::setprecision(3)
                                << "[FootholdRegion] leg=" << leg
                                << " fixed_enabled=" << static_cast<int>(fixedFootholdRegionsEnabled())
                                << " sequence_enabled="
                                << static_cast<int>(fixedFootholdSequenceManager_.isEnabled())
                                << " frame=" << (fixedFootholdSequenceManager_.isEnabled()
                                                     ? fixedFootholdSequenceManager_.getConfig().frame
                                                     : fixedFootholdRegionSettings_.frame)
                                << " selected_region=" << selectedRegion
                                << " phase_start=" << standStartTime
                                << " phase_end=" << standFinalTime
                                << " nominal=(" << footPos.x() << "," << footPos.y() << "," << footPos.z() << ")"
                                << " projection=(" << projection.positionInWorld.x() << ","
                                << projection.positionInWorld.y() << ","
                                << projection.positionInWorld.z() << ")"
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
