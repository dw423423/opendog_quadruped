#ifndef STAIRFOOTHOLDPLANNER_H
#define STAIRFOOTHOLDPLANNER_H

#include <string>
#include <vector>

namespace ocs2::legged_robot
{
    struct StairFoothold
    {
        std::string leg;
        double x{};
        double y{};
        double z{};
    };

    struct StairFootholdPlannerSettings
    {
        double stair_x_start = 1.0;
        double step_height = 0.03;
        double step_depth = 0.35;
        double step_width = 1.0;
        double stair_y_center = 0.0;
        double lf_target_y = 0.12;
        double rf_target_y = -0.12;
    };

    class StairFootholdPlanner
    {
    public:
        explicit StairFootholdPlanner(StairFootholdPlannerSettings settings = {});

        void setSettings(const StairFootholdPlannerSettings& settings);

        const StairFootholdPlannerSettings& settings() const;

        double targetX() const;

        StairFoothold targetForLeg(const std::string& leg) const;

        std::vector<StairFoothold> targetsForMode(const std::string& mode) const;

        bool isFootholdSafe(const StairFoothold& foothold) const;

    private:
        StairFootholdPlannerSettings settings_;
    };
}

#endif // STAIRFOOTHOLDPLANNER_H
