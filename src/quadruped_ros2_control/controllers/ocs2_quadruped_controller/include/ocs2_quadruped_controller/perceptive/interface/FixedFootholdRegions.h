//
// Fixed world-frame foothold regions for static-walk perceptive constraint tests.
//

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

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

    struct FixedFootholdRegionSet
    {
        std::string name;
        std::array<FixedFootholdRegion, 4> regions;
    };

    struct FixedFootholdSequenceConfig
    {
        bool enable = false;
        std::string frame = "world";
        bool manualAdvance = true;
        std::string advanceKey = "g";
        bool pauseGaitOnSetCompleted = true;
        std::string resumeGaitAfterAdvance = "static_walk";
        bool autoStanceOnFinal = true;
        bool requireZForReached = false;
        scalar_t zTolerance = 0.04;
        scalar_t stableHoldTime = 0.3;
        size_t activeSet = 0;
        std::vector<FixedFootholdRegionSet> sets;
    };

    struct StairFootholdRegionSettings
    {
        bool enable = true;
        bool preferStairTopWhenInsideFootprint = true;
        bool lockRegionDuringSwing = true;
        size_t numSteps = 2;
        scalar_t stepXStart = 0.40;
        scalar_t stepDepth = 0.35;
        scalar_t stepHeight = 0.15;
        scalar_t stepWidth = 0.80;
        scalar_t stairYCenter = 0.0;
        scalar_t edgeMarginX = 0.04;
        scalar_t edgeMarginY = 0.03;
        scalar_t activeRegionHalfLengthX = 0.10;
        scalar_t activeRegionHalfLengthY = 0.06;
    };

    struct FixedFootholdRegionCheck
    {
        bool insideXY = false;
        scalar_t zError = 0.0;
        bool insideZ = false;
        bool insideXYZ = false;
    };

    enum class FootholdSequenceState
    {
        RUNNING_SET,
        WAITING_FOR_ADVANCE,
        FINAL_DONE,
    };

    inline constexpr std::array<FixedFootholdRegion, 4> kFixedFootholdRegions = {
        FixedFootholdRegion{"FL", 0.25, 0.35, 0.10, 0.18, 0.0},
        FixedFootholdRegion{"FR", 0.25, 0.35, -0.18, -0.10, 0.0},
        FixedFootholdRegion{"RL", -0.05, 0.05, 0.10, 0.18, 0.0},
        FixedFootholdRegion{"RR", -0.05, 0.05, -0.18, -0.10, 0.0},
    };

    inline FixedFootholdRegionSet defaultFixedFootholdRegionSet(const std::string& name = "set0")
    {
        FixedFootholdRegionSet set;
        set.name = name;
        set.regions = kFixedFootholdRegions;
        return set;
    }

    inline FixedFootholdRegionSettings defaultFixedFootholdRegionSettings()
    {
        FixedFootholdRegionSettings settings;
        settings.regions = kFixedFootholdRegions;
        return settings;
    }

    inline FixedFootholdSequenceConfig defaultFixedFootholdSequenceConfig()
    {
        return {};
    }

    inline StairFootholdRegionSettings defaultStairFootholdRegionSettings()
    {
        return {};
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

    inline FixedFootholdRegionCheck checkFixedFootholdRegion(const FixedFootholdRegion& region,
                                                             const vector3_t& position,
                                                             scalar_t zTolerance)
    {
        FixedFootholdRegionCheck check;
        check.insideXY = position.x() >= region.xMin && position.x() <= region.xMax &&
            position.y() >= region.yMin && position.y() <= region.yMax;
        check.zError = position.z() - region.z;
        check.insideZ = std::abs(check.zError) <= zTolerance;
        check.insideXYZ = check.insideXY && check.insideZ;
        return check;
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

    class FixedFootholdSequenceManager
    {
    public:
        explicit FixedFootholdSequenceManager(FixedFootholdSequenceConfig config = defaultFixedFootholdSequenceConfig())
            : config_(std::move(config))
        {
            resetCurrentSetReached();
        }

        bool isEnabled() const
        {
            return config_.enable && !config_.sets.empty() && config_.activeSet < config_.sets.size();
        }

        const FixedFootholdSequenceConfig& getConfig() const { return config_; }

        FootholdSequenceState getState() const { return state_; }

        const char* getStateName() const
        {
            switch (state_)
            {
            case FootholdSequenceState::RUNNING_SET:
                return "RUNNING_SET";
            case FootholdSequenceState::WAITING_FOR_ADVANCE:
                return "WAITING_FOR_ADVANCE";
            case FootholdSequenceState::FINAL_DONE:
                return "FINAL_DONE";
            }
            return "UNKNOWN";
        }

        size_t getActiveSetIndex() const { return config_.activeSet; }

        const std::string& getActiveSetName() const
        {
            return config_.sets.at(config_.activeSet).name;
        }

        const FixedFootholdRegion& getActiveRegion(size_t leg) const
        {
            return config_.sets.at(config_.activeSet).regions.at(leg);
        }

        void resetCurrentSetReached()
        {
            reached_.fill(false);
            currentSetCompleted_ = false;
            setCompletionAnnounced_ = false;
            stanceRequestSentForCurrentSet_ = false;
        }

        FixedFootholdRegionCheck markLegTouchdown(size_t leg, const vector3_t& actualFootPosition)
        {
            const auto check = checkActiveRegion(leg, actualFootPosition);
            const bool reached = config_.requireZForReached ? check.insideXYZ : check.insideXY;
            if (reached)
            {
                reached_.at(leg) = true;
                currentSetCompleted_ = reached_[0] && reached_[1] && reached_[2] && reached_[3];
            }
            return check;
        }

        bool isLegReached(size_t leg) const { return reached_.at(leg); }

        bool isCurrentSetCompleted() const { return currentSetCompleted_; }

        bool isFinalSet() const
        {
            return isEnabled() && config_.activeSet + 1 >= config_.sets.size();
        }

        bool isFinalCompleted() const
        {
            return isFinalSet() && currentSetCompleted_;
        }

        bool isWaitingForAdvance() const
        {
            return state_ == FootholdSequenceState::WAITING_FOR_ADVANCE;
        }

        bool isFinalDone() const
        {
            return state_ == FootholdSequenceState::FINAL_DONE;
        }

        bool enterWaitingForAdvanceIfCompleted()
        {
            if (!isEnabled() || !currentSetCompleted_ || isFinalSet() || !config_.manualAdvance ||
                state_ != FootholdSequenceState::RUNNING_SET || setCompletionAnnounced_)
            {
                return false;
            }
            state_ = FootholdSequenceState::WAITING_FOR_ADVANCE;
            setCompletionAnnounced_ = true;
            return true;
        }

        bool enterFinalDoneIfCompleted()
        {
            if (!isFinalCompleted() || state_ == FootholdSequenceState::FINAL_DONE)
            {
                return false;
            }
            state_ = FootholdSequenceState::FINAL_DONE;
            return true;
        }

        bool shouldRequestStanceOnSetCompleted() const
        {
            return config_.pauseGaitOnSetCompleted;
        }

        bool consumeSetCompletedStanceRequest()
        {
            if (!isWaitingForAdvance() || !config_.pauseGaitOnSetCompleted ||
                stanceRequestSentForCurrentSet_)
            {
                return false;
            }
            stanceRequestSentForCurrentSet_ = true;
            return true;
        }

        bool tryAdvanceSetAfterTouchdown()
        {
            if (!isEnabled() || !currentSetCompleted_ || isFinalSet() || config_.manualAdvance ||
                state_ != FootholdSequenceState::RUNNING_SET)
            {
                return false;
            }
            ++config_.activeSet;
            resetCurrentSetReached();
            return true;
        }

        bool advanceToNextSetByUser()
        {
            if (!isWaitingForAdvance() || isFinalSet())
            {
                return false;
            }
            ++config_.activeSet;
            resetCurrentSetReached();
            state_ = FootholdSequenceState::RUNNING_SET;
            return true;
        }

        bool isInsideActiveRegionXY(size_t leg, const vector3_t& position) const
        {
            const auto& region = getActiveRegion(leg);
            return position.x() >= region.xMin && position.x() <= region.xMax &&
                position.y() >= region.yMin && position.y() <= region.yMax;
        }

        FixedFootholdRegionCheck checkActiveRegion(size_t leg, const vector3_t& position) const
        {
            return checkFixedFootholdRegion(getActiveRegion(leg), position, config_.zTolerance);
        }

        std::string activeRegionToString(size_t leg) const
        {
            std::ostringstream stream;
            stream << "set=" << getActiveSetIndex()
                << ",name=" << getActiveSetName()
                << "," << fixedFootholdRegionToString(getActiveRegion(leg));
            return stream.str();
        }

        std::string reachedToString() const
        {
            std::ostringstream stream;
            stream << "[" << static_cast<int>(reached_[0])
                << "," << static_cast<int>(reached_[1])
                << "," << static_cast<int>(reached_[2])
                << "," << static_cast<int>(reached_[3]) << "]";
            return stream.str();
        }

        bool consumeFinalStanceRequest()
        {
            if (!isFinalDone() || finalStanceRequested_)
            {
                return false;
            }
            finalStanceRequested_ = true;
            return config_.autoStanceOnFinal;
        }

        bool consumeFinalCompletionAnnouncement()
        {
            if (!isFinalDone() || finalCompletionAnnounced_)
            {
                return false;
            }
            finalCompletionAnnounced_ = true;
            return true;
        }

    private:
        FixedFootholdSequenceConfig config_;
        std::array<bool, 4> reached_{};
        FootholdSequenceState state_ = FootholdSequenceState::RUNNING_SET;
        bool currentSetCompleted_ = false;
        bool setCompletionAnnounced_ = false;
        bool stanceRequestSentForCurrentSet_ = false;
        bool finalStanceRequested_ = false;
        bool finalCompletionAnnounced_ = false;
    };
} // namespace ocs2::legged_robot
