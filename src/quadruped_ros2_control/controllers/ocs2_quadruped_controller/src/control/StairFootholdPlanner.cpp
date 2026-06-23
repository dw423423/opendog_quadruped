#include "ocs2_quadruped_controller/control/StairFootholdPlanner.h"

#include <algorithm>
#include <cmath>

namespace ocs2::legged_robot
{
    StairFootholdPlanner::StairFootholdPlanner(StairFootholdPlannerSettings settings)
        : settings_(settings)
    {
    }

    void StairFootholdPlanner::setSettings(const StairFootholdPlannerSettings& settings)
    {
        settings_ = settings;
    }

    const StairFootholdPlannerSettings& StairFootholdPlanner::settings() const
    {
        return settings_;
    }

    double StairFootholdPlanner::targetX() const
    {
        return settings_.stair_x_start + 0.5 * settings_.step_depth;
    }

    StairFoothold StairFootholdPlanner::targetForLeg(const std::string& leg) const
    {
        const double target_y = leg == "LF" ? settings_.lf_target_y : settings_.rf_target_y;
        return StairFoothold{
            leg,
            targetX(),
            target_y,
            settings_.step_height
        };
    }

    std::vector<StairFoothold> StairFootholdPlanner::targetsForMode(const std::string& mode) const
    {
        if (mode == "lf_step_up")
        {
            return {targetForLeg("LF")};
        }
        if (mode == "rf_step_up")
        {
            return {targetForLeg("RF")};
        }
        if (mode == "front_legs_step_up")
        {
            return {targetForLeg("LF"), targetForLeg("RF")};
        }
        return {};
    }

    bool StairFootholdPlanner::isFootholdSafe(const StairFoothold& foothold) const
    {
        const double step_x_min = settings_.stair_x_start;
        const double step_x_max = settings_.stair_x_start + settings_.step_depth;
        const double min_x_margin = std::min(0.05, 0.25 * settings_.step_depth);
        const bool x_is_centered = foothold.x > step_x_min + min_x_margin &&
                                   foothold.x < step_x_max - min_x_margin;

        const double step_y_min = settings_.stair_y_center - 0.5 * settings_.step_width;
        const double step_y_max = settings_.stair_y_center + 0.5 * settings_.step_width;
        const bool y_is_inside = foothold.y >= step_y_min && foothold.y <= step_y_max;

        constexpr double kHeightTolerance = 1e-9;
        const bool z_matches_step = std::abs(foothold.z - settings_.step_height) < kHeightTolerance;

        return x_is_centered && y_is_inside && z_matches_step;
    }
}
