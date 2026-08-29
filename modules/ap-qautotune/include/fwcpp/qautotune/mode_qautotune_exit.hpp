#pragma once

namespace fwcpp::qautotune {

struct QAutotuneExitActions {
    bool stop_qautotune{false};
};

[[nodiscard]] inline QAutotuneExitActions qautotune_exit_actions(bool qautotune_enabled) {
    QAutotuneExitActions out{};
    if (qautotune_enabled) {
        out.stop_qautotune = true;
    }
    return out;
}

}  // namespace fwcpp::qautotune
