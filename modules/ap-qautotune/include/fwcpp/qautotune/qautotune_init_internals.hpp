#pragma once

#include <fwcpp/qautotune/qautotune_init.hpp>

namespace fwcpp::qautotune {

// Upstream QAutoTune::init() (qautotune.cpp:9-22) after the availability
// gate: return init_internals(position_hold, attitude_control, pos_control,
// ahrs_view). This ports that four-argument wiring only. The
// AC_AutoTune_Multi body stays out of scope.

struct QAutotuneInitInternalsArgs {
    bool position_hold{false};
    void* attitude_control{nullptr};
    void* pos_control{nullptr};
    void* ahrs_view{nullptr};
};

struct QAutotuneInitInternalsResult {
    bool ok{false};
    QAutotuneInitInternalsArgs args{};
};

[[nodiscard]] inline QAutotuneInitInternalsResult resolve_qautotune_init_internals(
    const QAutotuneInitInputs& init, const QAutotuneInitInternalsArgs& handles) {
    QAutotuneInitInternalsResult out{};
    const auto base = resolve_qautotune_init(init);
    if (!base.ok) {
        return out;
    }
    out.ok = true;
    out.args.position_hold = base.position_hold;
    out.args.attitude_control = handles.attitude_control;
    out.args.pos_control = handles.pos_control;
    out.args.ahrs_view = handles.ahrs_view;
    return out;
}

}  // namespace fwcpp::qautotune
