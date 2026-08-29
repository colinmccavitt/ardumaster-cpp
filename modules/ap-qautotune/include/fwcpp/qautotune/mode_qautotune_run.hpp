#pragma once

#include <fwcpp/q_modes/q_run_common.hpp>

#include <cstdint>

namespace fwcpp::qautotune {

enum class QAutotuneRunPhase : std::uint8_t {
    kFwTransitionControllers = 0,
    kAutotuneBody = 1,
};

struct QAutotuneRunInputs {
    bool tailsitter_in_vtol_transition{false};
    bool qautotune_enabled{true};
};

struct QAutotuneRunResult {
    QAutotuneRunPhase phase{QAutotuneRunPhase::kAutotuneBody};
    bool delegate_mode_run{false};
    bool run_qautotune{false};
    fwcpp::q_modes::QRunFwSurfaceFollowup fw_followup{};
};

[[nodiscard]] inline QAutotuneRunResult qautotune_run(const QAutotuneRunInputs& in) {
    QAutotuneRunResult out{};
    if (fwcpp::q_modes::run_delegates_to_fw_controllers(in.tailsitter_in_vtol_transition)) {
        out.phase = QAutotuneRunPhase::kFwTransitionControllers;
        out.delegate_mode_run = true;
        return out;
    }
    out.phase = QAutotuneRunPhase::kAutotuneBody;
    if (in.qautotune_enabled) {
        out.run_qautotune = true;
    }
    out.fw_followup = fwcpp::q_modes::q_run_fw_surface_followup();
    return out;
}

}  // namespace fwcpp::qautotune
