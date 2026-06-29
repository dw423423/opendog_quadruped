//
// Created by biao on 3/21/25.
//

#include <ocs2_core/misc/LoadData.h>
#include "ocs2_quadruped_controller/perceptive/constraint/FootCollisionConstraint.h"
#include "ocs2_quadruped_controller/perceptive/constraint/SphereSdfConstraint.h"

#include "ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedInterface.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <grid_map_core/iterators/GridMapIterator.hpp>

#include <algorithm>
#include <array>
#include <exception>
#include <memory>
#include <string>
#include <typeinfo>
#include <vector>

namespace ocs2::legged_robot
{
    namespace
    {
        struct NoStepRegion
        {
            const char* name;
            double xMin;
            double xMax;
            double yMin;
            double yMax;
        };

        struct StairStepRegion
        {
            const char* name;
            double xMin;
            double xMax;
            double yMin;
            double yMax;
            double topHeight;
        };

        constexpr std::array<NoStepRegion, 3> kDefaultNoStepRegions = {
            NoStepRegion{"B", -1.75, -1.65, -1.25, 1.25},
            NoStepRegion{"C", 0.90, 1.55, 0.75, 1.40},
            NoStepRegion{"D", 1.25, 1.45, -1.80, -0.45},
        };
        constexpr std::array<StairStepRegion, 2> kGazeboStairSteps = {
            StairStepRegion{"fixed_step_01", 0.40, 0.75, -0.40, 0.40, 0.15},
            StairStepRegion{"fixed_step_02", 0.75, 1.10, -0.40, 0.40, 0.30},
        };
        constexpr double kNoStepSafetyMargin = 0.05;
        constexpr double kTerrainInset = 0.01;

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

        double clampRectangleErosionMargin(double xMin, double xMax, double yMin, double yMax, double margin)
        {
            const double halfX = 0.5 * (xMax - xMin);
            const double halfY = 0.5 * (yMax - yMin);
            return std::max(0.0, std::min({margin, 0.95 * halfX, 0.95 * halfY}));
        }

        convex_plane_decomposition::CgalPolygon2d makeErodedRectangle(double xMin, double xMax,
                                                                      double yMin, double yMax,
                                                                      double margin)
        {
            const double safeMargin = clampRectangleErosionMargin(xMin, xMax, yMin, yMax, margin);
            return makeRectangle(xMin + safeMargin, xMax - safeMargin,
                                 yMin + safeMargin, yMax - safeMargin);
        }

        void addDefaultNoStepRegions(convex_plane_decomposition::CgalPolygonWithHoles2d& polygon,
                                     double groundErosionMargin)
        {
            for (const auto& region : kDefaultNoStepRegions)
            {
                polygon.holes().push_back(makeRectangle(region.xMin - groundErosionMargin,
                                                        region.xMax + groundErosionMargin,
                                                        region.yMin - groundErosionMargin,
                                                        region.yMax + groundErosionMargin));
            }
        }

        void addStairFootprintHoles(convex_plane_decomposition::CgalPolygonWithHoles2d& polygon,
                                    double groundErosionMargin)
        {
            if (kGazeboStairSteps.empty())
            {
                return;
            }

            double xMin = kGazeboStairSteps.front().xMin;
            double xMax = kGazeboStairSteps.front().xMax;
            double yMin = kGazeboStairSteps.front().yMin;
            double yMax = kGazeboStairSteps.front().yMax;
            for (const auto& step : kGazeboStairSteps)
            {
                xMin = std::min(xMin, step.xMin);
                xMax = std::max(xMax, step.xMax);
                yMin = std::min(yMin, step.yMin);
                yMax = std::max(yMax, step.yMax);
            }
            polygon.holes().push_back(makeRectangle(xMin - groundErosionMargin,
                                                    xMax + groundErosionMargin,
                                                    yMin - groundErosionMargin,
                                                    yMax + groundErosionMargin));
        }

        convex_plane_decomposition::PlanarRegion makeHorizontalPlanarRegion(double xMin, double xMax,
                                                                            double yMin, double yMax,
                                                                            double z)
        {
            convex_plane_decomposition::PlanarRegion region;
            region.transformPlaneToWorld.setIdentity();
            region.transformPlaneToWorld.translation().z() = z;
            region.bbox2d = convex_plane_decomposition::CgalBbox2d(xMin, yMin, xMax, yMax);

            convex_plane_decomposition::CgalPolygonWithHoles2d boundary;
            boundary.outer_boundary() = makeRectangle(xMin, xMax, yMin, yMax);
            region.boundaryWithInset.boundary = boundary;

            convex_plane_decomposition::CgalPolygonWithHoles2d inset;
            inset.outer_boundary() = makeRectangle(xMin + kTerrainInset, xMax - kTerrainInset,
                                                   yMin + kTerrainInset, yMax - kTerrainInset);
            region.boundaryWithInset.insets.push_back(inset);
            return region;
        }

        void addGazeboStairsToGridMap(grid_map::GridMap& gridMap, const std::string& elevationLayer,
                                      const std::string& smoothLayer)
        {
            for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator)
            {
                grid_map::Position position;
                if (!gridMap.getPosition(*iterator, position))
                {
                    continue;
                }

                double height = 0.0;
                for (const auto& step : kGazeboStairSteps)
                {
                    if (position.x() >= step.xMin && position.x() <= step.xMax &&
                        position.y() >= step.yMin && position.y() <= step.yMax)
                    {
                        height = step.topHeight;
                    }
                }

                gridMap.at(elevationLayer, *iterator) = height;
                gridMap.at(smoothLayer, *iterator) = height;
            }
        }

        std::vector<std::string> getCalfCollisionLinks(const std::vector<std::string>& footNames)
        {
            std::vector<std::string> collisionLinks;
            collisionLinks.reserve(footNames.size());
            for (auto footName : footNames)
            {
                const std::string suffix = "_foot";
                if (footName.size() >= suffix.size() &&
                    footName.compare(footName.size() - suffix.size(), suffix.size(), suffix) == 0)
                {
                    footName.replace(footName.size() - suffix.size(), suffix.size(), "_calf");
                }
                collisionLinks.push_back(std::move(footName));
            }
            return collisionLinks;
        }
    }

    PerceptiveLeggedInterface::PerceptiveLeggedInterface(
        const std::string& taskFile, const std::string& urdfFile,
        const std::string& referenceFile, FixedFootholdRegionSettings fixedFootholdRegionSettings,
        FixedFootholdSequenceConfig fixedFootholdSequenceConfig,
        StairFootholdRegionSettings stairFootholdRegionSettings,
        scalar_t groundSteppableErosionMargin,
        bool useHardFrictionConeConstraint)
        : LeggedInterface(taskFile, urdfFile, referenceFile, useHardFrictionConeConstraint),
          fixedFootholdRegionSettings_(std::move(fixedFootholdRegionSettings)),
          fixedFootholdSequenceConfig_(std::move(fixedFootholdSequenceConfig)),
          stairFootholdRegionSettings_(std::move(stairFootholdRegionSettings)),
          groundSteppableErosionMargin_(std::max<scalar_t>(0.0, groundSteppableErosionMargin))
    {
    }

    void PerceptiveLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                               const std::string& referenceFile, bool verbose)
    {
        planarTerrainPtr_ = std::make_shared<convex_plane_decomposition::PlanarTerrain>();

        double width{10.0}, height{10.0};
        const double groundErosionMargin = clampRectangleErosionMargin(
            -height / 2, +height / 2, -width / 2, +width / 2, groundSteppableErosionMargin_);
        convex_plane_decomposition::PlanarRegion plannerRegion;
        plannerRegion.transformPlaneToWorld.setIdentity();
        plannerRegion.bbox2d = convex_plane_decomposition::CgalBbox2d(
            -height / 2 + groundErosionMargin,
            -width / 2 + groundErosionMargin,
            +height / 2 - groundErosionMargin,
            +width / 2 - groundErosionMargin);
        convex_plane_decomposition::CgalPolygonWithHoles2d boundary;
        boundary.outer_boundary() = makeErodedRectangle(
            -height / 2, +height / 2, -width / 2, +width / 2, groundErosionMargin);
        addDefaultNoStepRegions(boundary, groundErosionMargin);
        addStairFootprintHoles(boundary, groundErosionMargin);
        plannerRegion.boundaryWithInset.boundary = boundary;
        convex_plane_decomposition::CgalPolygonWithHoles2d insets;
        insets.outer_boundary() = makeErodedRectangle(
            -height / 2, +height / 2, -width / 2, +width / 2,
            groundErosionMargin + kTerrainInset);
        addDefaultNoStepRegions(insets, groundErosionMargin + kTerrainInset);
        addStairFootprintHoles(insets, groundErosionMargin + kTerrainInset);
        plannerRegion.boundaryWithInset.insets.push_back(insets);
        planarTerrainPtr_->planarRegions.push_back(plannerRegion);
        for (const auto& step : kGazeboStairSteps)
        {
            planarTerrainPtr_->planarRegions.push_back(makeHorizontalPlanarRegion(
                step.xMin, step.xMax, step.yMin, step.yMax, step.topHeight));
        }

        std::string layer = "elevation_before_postprocess";
        planarTerrainPtr_->gridMap.setGeometry(grid_map::Length(10.0, 10.0), 0.03);
        planarTerrainPtr_->gridMap.add(layer, 0);
        planarTerrainPtr_->gridMap.add("smooth_planar", 0);
        addGazeboStairsToGridMap(planarTerrainPtr_->gridMap, layer, "smooth_planar");
        signedDistanceFieldPtr_ = std::make_shared<grid_map::SignedDistanceField>();
        signedDistanceFieldPtr_->calculateSignedDistanceField(planarTerrainPtr_->gridMap, layer, 0.1);

        LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
        setupPreComputation(taskFile, urdfFile, referenceFile, verbose);
        const bool usesPerceptivePrecomputation =
            dynamic_cast<PerceptiveLeggedPrecomputation*>(problem_ptr_->preComputationPtr.get()) != nullptr;
        std::cerr << "[PerceptiveLeggedInterface] preComputation type: "
            << typeid(*problem_ptr_->preComputationPtr).name()
            << ", is PerceptiveLeggedPrecomputation="
            << static_cast<int>(usesPerceptivePrecomputation) << std::endl;

        for (size_t i = 0; i < centroidal_model_info_.numThreeDofContacts; i++)
        {
            const std::string& footName = modelSettings().contactNames3DoF[i];
            std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);

            std::unique_ptr<PenaltyBase> placementPenalty(
                new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-4)));
            std::unique_ptr<PenaltyBase> collisionPenalty(
                new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-3)));

            // For foot placement
            std::unique_ptr<FootPlacementConstraint> footPlacementConstraint(
                new FootPlacementConstraint(*reference_manager_ptr_, *eeKinematicsPtr, i, numVertices_));
            problem_ptr_->stateSoftConstraintPtr->add(
                footName + "_footPlacement",
                std::make_unique<StateSoftConstraint>(std::move(footPlacementConstraint), std::move(placementPenalty)));

            // For foot Collision
            std::unique_ptr<FootCollisionConstraint> footCollisionConstraint(
                new FootCollisionConstraint(*reference_manager_ptr_, *eeKinematicsPtr, signedDistanceFieldPtr_, i,
                                            0.03));
            problem_ptr_->stateSoftConstraintPtr->add(
                footName + "_footCollision",
                std::make_unique<StateSoftConstraint>(std::move(footCollisionConstraint), std::move(collisionPenalty)));
        }

        // For collision avoidance
        scalar_t thighExcess = 0.025;
        scalar_t calfExcess = 0.02;

        std::vector<std::string> collisionLinks = getCalfCollisionLinks(modelSettings().contactNames3DoF);
        std::vector<scalar_t> maxExcesses(collisionLinks.size(), calfExcess);
        std::cerr << "[PerceptiveLeggedInterface] collision links:";
        for (const auto& link : collisionLinks)
        {
            std::cerr << " " << link;
        }
        std::cerr << std::endl;

        try
        {
            pinocchioSphereInterfacePtr_ = std::make_shared<PinocchioSphereInterface>(
                *pinocchio_interface_ptr_, collisionLinks, maxExcesses, 0.6);
        }
        catch (const std::exception& e)
        {
            std::cerr << "[PerceptiveLeggedInterface] failed to create PinocchioSphereInterface: "
                << e.what() << ". Sphere visualization/SDF collision spheres will be disabled." << std::endl;
            pinocchioSphereInterfacePtr_.reset();
        }

        CentroidalModelPinocchioMapping pinocchioMapping(centroidal_model_info_);
        if (pinocchioSphereInterfacePtr_ != nullptr)
        {
            auto sphereKinematicsPtr = std::make_unique<PinocchioSphereKinematics>(
                *pinocchioSphereInterfacePtr_, pinocchioMapping);

            std::unique_ptr<SphereSdfConstraint> sphereSdfConstraint(
                new SphereSdfConstraint(*sphereKinematicsPtr, signedDistanceFieldPtr_));
        }

        //  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-3, 1e-3)));
        //  problem_ptr_->stateSoftConstraintPtr->add(
        //      "sdfConstraint", std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(sphereSdfConstraint), std::move(penalty))));
    }

    void PerceptiveLeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& /*urdfFile*/,
                                                          const std::string& referenceFile, bool verbose)
    {
        auto swingTrajectoryPlanner =
            std::make_unique<SwingTrajectoryPlanner>(
                loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);

        std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr(
            {model_settings_.contactNames3DoF}, "ALL_FOOT");
        auto convexRegionSelector =
            std::make_unique<ConvexRegionSelector>(centroidal_model_info_, planarTerrainPtr_, *eeKinematicsPtr,
                                                   numVertices_, fixedFootholdRegionSettings_,
                                                   fixedFootholdSequenceConfig_,
                                                   stairFootholdRegionSettings_);

        scalar_t comHeight = 0;
        loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
        reference_manager_ptr_.reset(new PerceptiveLeggedReferenceManager(
            centroidal_model_info_, loadGaitSchedule(referenceFile, verbose),
            std::move(swingTrajectoryPlanner), std::move(convexRegionSelector),
            *eeKinematicsPtr, comHeight));
    }

    void PerceptiveLeggedInterface::setupPreComputation(const std::string& /*taskFile*/,
                                                        const std::string& /*urdfFile*/,
                                                        const std::string& /*referenceFile*/, bool /*verbose*/)
    {
        problem_ptr_->preComputationPtr = std::make_unique<PerceptiveLeggedPrecomputation>(
            *pinocchio_interface_ptr_, centroidal_model_info_, *reference_manager_ptr_->getSwingTrajectoryPlanner(),
            model_settings_,
            *dynamic_cast<PerceptiveLeggedReferenceManager&>(*reference_manager_ptr_).getConvexRegionSelectorPtr());
    }
} // namespace legged
