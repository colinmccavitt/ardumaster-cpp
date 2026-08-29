#pragma once

#include <fwcpp/q_loiter/mode_qloiter_run.hpp>

namespace fwcpp::q_loiter {

struct QLandRunResult {
    bool delegates_qloiter_run{true};
    QLoiterRunResult qloiter{};
};

[[nodiscard]] inline QLandRunResult qland_run(QLoiterRunInputs in) {
    QLandRunResult out{};
    in.active_control_is_qland = true;
    out.qloiter = qloiter_run(in);
    return out;
}

}  // namespace fwcpp::q_loiter
