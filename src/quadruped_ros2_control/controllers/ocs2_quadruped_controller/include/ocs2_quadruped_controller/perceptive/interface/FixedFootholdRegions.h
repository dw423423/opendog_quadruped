//
// Fixed world-frame foothold regions for static-walk perceptive constraint tests.
//

#pragma once

#include <array>
#include <sstream>
#include <string>

#include <ocs2_legged_robot/common/Types.h>

namespace ocs2::legged_robot
{
    struct FixedFootholdRegion
    {
        const char* name;
        scalar_t xMin;
        scalar_t xMax;
        scalar_t yMin;
        scalar_t yMax;
        scalar_t z;
    };

    struct FixedFootholdRegionSettings
    {
        bool enable = true;
        std::string frame = "world";
        std::array<FixedFootholdRegion, 4> regions;
    };

    inline constexpr std::array<FixedFootholdRegion, 4> kFixedFootholdRegions = {
        FixedFootholdRegion{"FL", 0.25, 0.35, 0.10, 0.18, 0.0},
        FixedFootholdRegion{"FR", 0.25, 0.35, -0.18, -0.10, 0.0},
        FixedFootholdRegion{"RL", -0.05, 0.05, 0.10, 0.18, 0.0},
        FixedFootholdRegion{"RR", -0.05, 0.05, -0.18, -0.10, 0.0},
    };

    inline FixedFootholdRegionSettings defaultFixedFootholdRegionSettings()
    {
        FixedFootholdRegionSettings settings;
        settings.regions = kFixedFootholdRegions;
        return settings;
    }

    inline const FixedFootholdRegion& getFixedFootholdRegion(const FixedFootholdRegionSettings& settings, size_t leg)
    {
        return settings.regions.at(leg);
    }

    inline bool isInsideFixedFootholdRegionXY(const FixedFootholdRegionSettings& settings, size_t leg,
                                              const vector3_t& position)
    {
        const auto& region = getFixedFootholdRegion(settings, leg);
        return position.x() >= region.xMin && position.x() <= region.xMax &&
            position.y() >= region.yMin && position.y() <= region.yMax;
    }

    inline std::string fixedFootholdRegionToString(const FixedFootholdRegion& region)
    {
        std::ostringstream stream;
        stream << region.name
            << ":x[" << region.xMin << "," << region.xMax << "]"
            << ",y[" << region.yMin << "," << region.yMax << "]"
            << ",z=" << region.z;
        return stream.str();
    }

    inline std::string fixedFootholdRegionToString(const FixedFootholdRegionSettings& settings, size_t leg)
    {
        return fixedFootholdRegionToString(getFixedFootholdRegion(settings, leg));
    }
} // namespace ocs2::legged_robot
