#pragma once

#include <cstdint>

namespace fwcpp::q_loiter {

struct QLoiterQlandOptionalEffects {
    bool cut_ic_engine{false};
};

struct QLoiterQlandOptionalInputs {
    bool active_control_is_qland{false};
    bool land_final_transition{false};
    bool icengine_enabled{false};
    std::int8_t land_icengine_cut{0};
};

/// Optional QLAND IC-engine cut on LAND_FINAL (landing gear stays on QLAND `_enter`).
/// Default off: `icengine_enabled` false or `land_icengine_cut == 0`.
[[nodiscard]] inline QLoiterQlandOptionalEffects qloiter_qland_optional_effects(
    const QLoiterQlandOptionalInputs& in) {
    QLoiterQlandOptionalEffects out{};
    if (in.active_control_is_qland && in.land_final_transition && in.icengine_enabled &&
        in.land_icengine_cut != 0) {
        out.cut_ic_engine = true;
    }
    return out;
}

}  // namespace fwcpp::q_loiter
