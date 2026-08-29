#pragma once

#include <fwcpp/q_loiter/mode_qloiter_run.hpp>

namespace fwcpp::q_loiter {

struct QLandRunResult {
    bool delegates_qloiter_run{true};
    QLoiterRunResult qloiter{};
};

/// Port of ModeQLand::run — delegates to ModeQLoiter::run with QLAND vertical branch.
[[nodiscard]] inline QLandRunResult qland_run(QLoiterRunInputs in, PosControlState& pc) {
    QLandRunResult out{};
    in.active_control_is_qland = true;
    out.qloiter = qloiter_run(in, pc);
    return out;
}

[[nodiscard]] inline QLandRunResult qland_run(QLoiterRunInputs in) {
    PosControlState pc{};
    pc.state = in.poscontrol_state;
    return qland_run(in, pc);
}

}  // namespace fwcpp::q_loiter
