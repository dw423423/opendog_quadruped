//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"
#include "ocs2_quadruped_controller/perceptive/interface/FixedFootholdRegions.h"

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace ocs2::legged_robot
{
    namespace
    {
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

        convex_plane_decomposition::CgalPoint2d polygonAveragePoint(
            const convex_plane_decomposition::CgalPolygon2d& polygon)
        {
            scalar_t x = 0.0;
            scalar_t y = 0.0;
            for (size_t i = 0; i < polygon.size(); ++i)
            {
                const auto vertex = polygon.vertex(static_cast<int>(i));
                x += vertex.x();
                y += vertex.y();
            }
            const scalar_t scale = polygon.size() == 0 ? 1.0 : 1.0 / static_cast<scalar_t>(polygon.size());
            return convex_plane_decomposition::CgalPoint2d(scale * x, scale * y);
        }
    }

    PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(PinocchioInterface pinocchioInterface,
                                                                   const CentroidalModelInfo& info,
                                                                   const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                                                   ModelSettings settings,
                                                                   const ConvexRegionSelector& convexRegionSelector)
        : LeggedRobotPreComputation(std::move(pinocchioInterface), info, swingTrajectoryPlanner, std::move(settings)),
          convexRegionSelectorPtr_(&convexRegionSelector)
    {
        footPlacementConParameters_.resize(info.numThreeDofContacts);
    }

    PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(const PerceptiveLeggedPrecomputation& rhs)
        : LeggedRobotPreComputation(rhs), convexRegionSelectorPtr_(rhs.convexRegionSelectorPtr_)
    {
        footPlacementConParameters_.resize(rhs.footPlacementConParameters_.size());
    }

    void PerceptiveLeggedPrecomputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u)
    {
        if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint))
        {
            return;
        }
        LeggedRobotPreComputation::request(request, t, x, u);

        if (request.contains(Request::Constraint))
        {
            for (size_t i = 0; i < footPlacementConParameters_.size(); i++)
            {
                FootPlacementConstraint::Parameter params;

                auto projection = convexRegionSelectorPtr_->getProjection(i, t);
                if (projection.regionPtr == nullptr)
                {
                    // Swing leg
                    continue;
                }

                const auto polygon = convexRegionSelectorPtr_->getConvexPolygon(i, t);
                matrix_t polytopeA;
                vector_t polytopeB;
                std::tie(polytopeA, polytopeB) = getPolygonConstraint(polygon);
                matrix_t p = (matrix_t(2, 3) << // clang-format off
                        1, 0, 0,
                        0, 1, 0).finished();  // clang-format on
                params.a = polytopeA * p * projection.regionPtr->transformPlaneToWorld.inverse().linear();
                params.b = polytopeB + polytopeA * projection.regionPtr->transformPlaneToWorld.inverse().translation().
                                                              head(2);

                footPlacementConParameters_[i] = params;
                static std::array<scalar_t, 4> lastLogTime = {-1.0, -1.0, -1.0, -1.0};
                if (i < lastLogTime.size() && (lastLogTime[i] < 0.0 || t - lastLogTime[i] > 0.25))
                {
                    lastLogTime[i] = t;
                    const auto centerInTerrain = polygonAveragePoint(polygon);
                    const vector_t center2d = (vector_t(2) << centerInTerrain.x(), centerInTerrain.y()).finished();
                    const vector_t projection2d =
                        (vector_t(2) << projection.positionInTerrainFrame.x(),
                         projection.positionInTerrainFrame.y()).finished();
                    const vector_t centerTerrainValues = polytopeA * center2d + polytopeB;
                    const vector_t projectionTerrainValues = polytopeA * projection2d + polytopeB;
                    const vector3_t centerWorld =
                        convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                            centerInTerrain, projection.regionPtr->transformPlaneToWorld);
                    const vector_t centerWorldValues = params.a * centerWorld + params.b;
                    const vector_t projectionWorldValues = params.a * projection.positionInWorld + params.b;
                    const bool centerInsideByOcs2Convention =
                        centerWorldValues.size() > 0 && centerWorldValues.minCoeff() >= -1e-8;
                    const bool projectionInsideByOcs2Convention =
                        projectionWorldValues.size() > 0 && projectionWorldValues.minCoeff() >= -1e-8;
                    std::cerr << std::fixed << std::setprecision(3)
                        << "[PerceptivePrecomputation] leg=" << legName(i)
                        << " leg_index=" << i
                        << " fixed_enabled="
                        << static_cast<int>(convexRegionSelectorPtr_->fixedFootholdRegionsEnabled());
                    if (convexRegionSelectorPtr_->fixedFootholdRegionsEnabled())
                    {
                        std::cerr << " selected_region=" << convexRegionSelectorPtr_->fixedFootholdRegionToString(i);
                    }
                    else
                    {
                        std::cerr << " selected_region=perceptive";
                    }
                    std::cerr
                        << " time=" << t
                        << " ocs2_soft_constraint_convention=h>=0"
                        << " center=(" << centerWorld.x() << "," << centerWorld.y() << "," << centerWorld.z() << ")"
                        << " projection=(" << projection.positionInWorld.x() << ","
                        << projection.positionInWorld.y() << "," << projection.positionInWorld.z() << ")"
                        << " center_values=" << vectorToString(centerWorldValues)
                        << " projection_values=" << vectorToString(projectionWorldValues)
                        << " center_inside=" << (centerInsideByOcs2Convention ? "true" : "false")
                        << " projection_inside=" << (projectionInsideByOcs2Convention ? "true" : "false")
                        << " center_terrain_values=" << vectorToString(centerTerrainValues)
                        << " projection_terrain_values=" << vectorToString(projectionTerrainValues)
                        << "\nparam.a=\n" << params.a
                        << "\nparam.b=\n" << params.b.transpose()
                        << std::endl;
                    if (!centerInsideByOcs2Convention || !projectionInsideByOcs2Convention)
                    {
                        std::cerr << std::fixed << std::setprecision(3)
                            << "[PerceptivePrecomputation][WARN] leg=" << legName(i)
                            << " leg_index=" << i
                            << " time=" << t
                            << " half_space_self_check_failed=true"
                            << " ocs2_soft_constraint_convention=h>=0"
                            << " polygon_vertices=" << polygonToString(polygon)
                            << " center_values=" << vectorToString(centerWorldValues)
                            << " projection_values=" << vectorToString(projectionWorldValues)
                            << " center_terrain_values=" << vectorToString(centerTerrainValues)
                            << " projection_terrain_values=" << vectorToString(projectionTerrainValues)
                            << "\nparam.a=\n" << params.a
                            << "\nparam.b=\n" << params.b.transpose()
                            << std::endl;
                    }
                }
            }
        }
    }

    std::pair<matrix_t, vector_t> PerceptiveLeggedPrecomputation::getPolygonConstraint(
        const convex_plane_decomposition::CgalPolygon2d& polygon) const
    {
        size_t numVertices = polygon.size();
        matrix_t polytopeA = matrix_t::Zero(numVertices, 2);
        vector_t polytopeB = vector_t::Zero(numVertices);

        for (size_t i = 0; i < numVertices; i++)
        {
            size_t j = i + 1;
            if (j == numVertices)
            {
                j = 0;
            }
            size_t k = j + 1;
            if (k == numVertices)
            {
                k = 0;
            }
            const auto point_a = polygon.vertex(i);
            const auto point_b = polygon.vertex(j);
            const auto point_c = polygon.vertex(k);

            polytopeA.row(i) << point_b.y() - point_a.y(), point_a.x() - point_b.x();
            polytopeB(i) = point_a.y() * point_b.x() - point_a.x() * point_b.y();
            if (polytopeA.row(i) * (vector_t(2) << point_c.x(), point_c.y()).finished() + polytopeB(i) < 0)
            {
                polytopeA.row(i) *= -1;
                polytopeB(i) *= -1;
            }
        }

        return {polytopeA, polytopeB};
    }
} // namespace legged
