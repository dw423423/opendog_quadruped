//
// Created by biao on 3/21/25.
//

#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>
#include <ocs2_quadruped_controller/interface/LeggedInterface.h>
#include <ocs2_quadruped_controller/perceptive/interface/FixedFootholdRegions.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>
#include <grid_map_sdf/SignedDistanceField.hpp>

namespace ocs2::legged_robot
{
    class PerceptiveLeggedInterface final : public LeggedInterface
    {
    public:
        PerceptiveLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                  const std::string& referenceFile,
                                  FixedFootholdRegionSettings fixedFootholdRegionSettings =
                                  defaultFixedFootholdRegionSettings(),
                                  bool useHardFrictionConeConstraint = false);

        void setupOptimalControlProblem(const std::string& taskFile,
                                        const std::string& urdfFile,
                                        const std::string& referenceFile,
                                        bool verbose) override;

        void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile,
                                   const std::string& referenceFile,
                                   bool verbose) override;

        void setupPreComputation(const std::string& taskFile, const std::string& urdfFile,
                                 const std::string& referenceFile,
                                 bool verbose);

        std::shared_ptr<grid_map::SignedDistanceField> getSignedDistanceFieldPtr() const
        {
            return signedDistanceFieldPtr_;
        }

        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() const
        {
            return planarTerrainPtr_;
        }

        std::shared_ptr<PinocchioSphereInterface> getPinocchioSphereInterfacePtr() const
        {
            return pinocchioSphereInterfacePtr_;
        }

        const FixedFootholdRegionSettings& getFixedFootholdRegionSettings() const
        {
            return fixedFootholdRegionSettings_;
        }

        size_t getNumVertices() const { return numVertices_; }

    protected:
        size_t numVertices_ = 16;

        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
        std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr_;
        std::shared_ptr<PinocchioSphereInterface> pinocchioSphereInterfacePtr_;
        FixedFootholdRegionSettings fixedFootholdRegionSettings_;
    };
} // namespace ocs2::legged_robot
