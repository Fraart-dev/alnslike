#pragma once
#include <alnslike/core/types.hpp>

namespace alnslike::builtin::selectors {

struct RewardWeights {
    static constexpr double kDefaultRejected = 0.0;
    static constexpr double kDefaultAccepted = 1.0;
    static constexpr double kDefaultImprovedObjective = 2.0;
    static constexpr double kDefaultImprovedFeasibility = 3.0;
    static constexpr double kDefaultNewBest = 6.0;

    double rejected{kDefaultRejected};
    double accepted{kDefaultAccepted};
    double improved_objective{kDefaultImprovedObjective};
    double improved_feasibility{kDefaultImprovedFeasibility};
    double new_best{kDefaultNewBest};

    [[nodiscard]] constexpr double Determine(const core::OperatorReward& reward) const noexcept {
        if (!reward.is_accepted) {
            return rejected;
        }

        if (reward.is_new_best) {
            return new_best;
        }

        if (reward.feasibility_violation_delta < 0.0) {
            return improved_feasibility;
        }

        if (reward.new_cost < reward.old_cost && reward.feasibility_violation_delta <= 0.0) {
            return improved_objective;
        }

        return accepted;
    }
};

}  // namespace alnslike::builtin::selectors