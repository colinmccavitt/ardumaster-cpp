#pragma once

#include <cstdint>

namespace fwcpp::qautotune {

struct QAutotuneInitInputs {
    bool quadplane_available{false};
    bool previous_mode_was_qloiter{false};
};

struct QAutotuneInitResult {
    bool ok{false};
    bool position_hold{false};
};

[[nodiscard]] inline QAutotuneInitResult resolve_qautotune_init(const QAutotuneInitInputs& in) {
    QAutotuneInitResult out{};
    if (!in.quadplane_available) {
        return out;
    }
    out.position_hold = in.previous_mode_was_qloiter;
    out.ok = true;
    return out;
}

}  // namespace fwcpp::qautotune
