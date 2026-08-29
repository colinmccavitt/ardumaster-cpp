#pragma once

namespace fwcpp::qrtl {

struct QrtlUpdateActions {
    bool delegate_qstabilize_update{true};
};

/// Port of ModeQRTL::update() — always delegates to ModeQStabilize::update().
[[nodiscard]] inline QrtlUpdateActions qrtl_update_actions() {
    QrtlUpdateActions out{};
    out.delegate_qstabilize_update = true;
    return out;
}

}  // namespace fwcpp::qrtl
