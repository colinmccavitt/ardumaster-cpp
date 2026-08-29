#pragma once

namespace fwcpp::qautotune {

struct QAutotuneUpdateActions {
    bool delegate_qstabilize_update{true};
};

[[nodiscard]] inline QAutotuneUpdateActions qautotune_update_actions() {
    QAutotuneUpdateActions out{};
    out.delegate_qstabilize_update = true;
    return out;
}

}  // namespace fwcpp::qautotune
