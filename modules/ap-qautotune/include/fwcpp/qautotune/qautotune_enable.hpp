#pragma once

#include <fwcpp/qautotune/qautotune_defaults.hpp>
#include <fwcpp/qautotune/qautotune_types.hpp>

namespace fwcpp::qautotune {

[[nodiscard]] inline constexpr bool qautotune_compile_enabled(const QAutotuneEnableInputs& in) {
    return in.hal_quadplane_enabled &&
           static_cast<std::uint8_t>(in.board) == kHalBoardSitl;
}

class QAutotuneGate {
public:
    [[nodiscard]] static QAutotuneGate from_inputs(const QAutotuneEnableInputs& in) {
        QAutotuneGate gate{};
        gate.enabled_ = qautotune_compile_enabled(in);
        return gate;
    }

    [[nodiscard]] bool enabled() const { return enabled_; }

private:
    bool enabled_{false};
};

}  // namespace fwcpp::qautotune
