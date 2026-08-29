#pragma once

#include "assist_recovery.hpp"
#include "vtol_assist.hpp"

namespace fwcpp::vtol_assist {

enum class RecoveryPath : std::uint8_t {
    kFwRecoveryAllowed,
    kBlockedFwForceDisabled,
    kBlockedTailsitter,
    kBlockedQacro,
};

[[nodiscard]] inline RecoveryPath classify_recovery_path(const VtolAssist& assist,
                                                         const AssistRecoveryInputs& in) {
    if (assist.option_is_set(AssistOption::kFwForceDisabled)) {
        return RecoveryPath::kBlockedFwForceDisabled;
    }
    if (in.tailsitter_enabled) {
        return RecoveryPath::kBlockedTailsitter;
    }
    if (in.in_qacro_mode) {
        return RecoveryPath::kBlockedQacro;
    }
    return RecoveryPath::kFwRecoveryAllowed;
}

[[nodiscard]] inline bool spin_recovery_option_allows(const VtolAssist& assist) {
    return !assist.option_is_set(AssistOption::kSpinDisabled);
}

}  // namespace fwcpp::vtol_assist
