#pragma once

// Copter::run_nav_updates leftover. Upstream ArduCopter/navigation.cpp
// ~6-9. The entire body is update_super_simple_bearing(false).
// SUPER_SIMPLE radius / 5deg bearing filter stay on the
// update_super_simple_bearing remaining row (Copter.cpp ~891-914).

namespace fwcpp::copter {

struct RunNavUpdatesEffects {
    bool update_super_simple_bearing{false};
    bool force_update{true};  // recorded argument; leftover always passes false
};

[[nodiscard]] inline RunNavUpdatesEffects run_nav_updates() {
    RunNavUpdatesEffects fx{};
    fx.update_super_simple_bearing = true;
    fx.force_update = false;
    return fx;
}

}  // namespace fwcpp::copter
