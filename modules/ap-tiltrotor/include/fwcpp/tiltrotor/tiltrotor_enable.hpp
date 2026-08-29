#pragma once

#include <cstdint>

#include <fwcpp/tiltrotor/tiltrotor_setup.hpp>

namespace fwcpp::tiltrotor {

class TiltrotorGate {
public:
    [[nodiscard]] static TiltrotorGate from_setup(const TiltrotorSetupResult& setup) {
        TiltrotorGate gate{};
        gate.enable_ = setup.enable;
        gate.setup_complete_ = setup.setup_complete;
        return gate;
    }

    [[nodiscard]] std::int8_t enable() const { return enable_; }
    [[nodiscard]] bool setup_complete() const { return setup_complete_; }

    [[nodiscard]] bool enabled() const { return enable_ > 0 && setup_complete_; }

private:
    std::int8_t enable_{0};
    bool setup_complete_{false};
};

}  // namespace fwcpp::tiltrotor
