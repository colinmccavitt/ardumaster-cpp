#pragma once

#include <fwcpp/qautotune/qautotune_init.hpp>

namespace fwcpp::qautotune {

struct QAutotuneEnterInputs {
    QAutotuneInitInputs init{};
    bool qautotune_enabled{true};
};

struct QAutotuneEnterResult {
    bool entered{false};
    QAutotuneInitResult init{};
};

[[nodiscard]] inline QAutotuneEnterResult qautotune_enter(const QAutotuneEnterInputs& in) {
    QAutotuneEnterResult out{};
    if (!in.qautotune_enabled) {
        return out;
    }
    out.init = resolve_qautotune_init(in.init);
    out.entered = out.init.ok;
    return out;
}

}  // namespace fwcpp::qautotune
