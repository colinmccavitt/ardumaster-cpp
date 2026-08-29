#pragma once

// Leftover Tailsitter::setup() after the enable heuristic (Plane-4.7.0
// tailsitter.cpp 213-244). Caller injects configured/assigned flags; this
// port does not persist params or reach into QuadPlane members.

#include <cstdint>

#include <fwcpp/quadplane/quadplane_air_mode.hpp>
#include <fwcpp/quadplane/quadplane_options.hpp>
#include <fwcpp/quadplane_transition/transition_timing.hpp>
#include <fwcpp/tailsitter/tailsitter_defaults.hpp>
#include <fwcpp/tailsitter/tailsitter_input_type.hpp>
#include <fwcpp/tailsitter/tailsitter_setup.hpp>
#include <fwcpp/tailsitter/tailsitter_types.hpp>
#include <fwcpp/vtol_assist/vtol_assist.hpp>

namespace fwcpp::tailsitter {

struct SrvAssigned {
    bool tilt_left{false};
    bool tilt_right{false};
    bool elevator{false};
    bool aileron{false};
    bool rudder{false};
    bool elevon_left{false};
    bool elevon_right{false};
    bool vtail_left{false};
    bool vtail_right{false};
};

struct TailsitterSetupFlagInputs {
    std::uint8_t frame_class{0};
    float vectored_hover_gain{kVectoredHoverGainDefault};
    SrvAssigned srv{};
    bool transition_rate_fw_configured{false};
    float transition_rate_fw{kTransitionRateFwDefault};
    std::int8_t transition_angle_fw{kTransitionAngleFwDefault};
    std::int16_t transition_time_ms{fwcpp::quadplane_transition::kQTransitionMsDefault};
    fwcpp::vtol_assist::AssistState assist_state{fwcpp::vtol_assist::AssistState::kAssistEnabled};
    fwcpp::quadplane::AirMode air_mode{fwcpp::quadplane::AirMode::kOff};
    std::int32_t options{fwcpp::quadplane::kQOptionsDefault};
};

struct TailsitterEnable2Effects {
    bool applied{false};
    fwcpp::vtol_assist::AssistState assist_state{fwcpp::vtol_assist::AssistState::kAssistEnabled};
    fwcpp::quadplane::AirMode air_mode{fwcpp::quadplane::AirMode::kOff};
    std::int32_t options{fwcpp::quadplane::kQOptionsDefault};
};

struct TailsitterSetupFlags {
    float transition_rate_fw{kTransitionRateFwDefault};
    bool transition_rate_fw_auto_set{false};
    bool is_vectored{false};
    SurfaceAssign surfaces{};
    TailsitterEnable2Effects enable2{};
};

struct TailsitterSetupWithFlags {
    TailsitterSetupResult setup{};
    TailsitterSetupFlags flags{};
};

[[nodiscard]] inline constexpr float resolve_transition_rate_fw(bool configured, float current_rate,
                                                                std::int8_t angle_fw,
                                                                std::int16_t transition_time_ms) {
    if (configured) {
        return current_rate;
    }
    return static_cast<float>(angle_fw) / (static_cast<float>(transition_time_ms) / 2000.0f);
}

[[nodiscard]] inline bool setup_is_vectored(std::uint8_t frame_class, float vectored_hover_gain,
                                            bool tilt_left, bool tilt_right) {
    TailsitterInputContext ctx{};
    ctx.frame_class = frame_class;
    ctx.vectored_hover_gain = vectored_hover_gain;
    ctx.tilt_motor_left = tilt_left;
    ctx.tilt_motor_right = tilt_right;
    return is_vectored(ctx);
}

[[nodiscard]] inline constexpr SurfaceAssign resolve_surface_flags(const SrvAssigned& srv) {
    return SurfaceAssign{
        .elevator = srv.elevator,
        .aileron = srv.aileron,
        .rudder = srv.rudder,
        .elevon = srv.elevon_left || srv.elevon_right,
        .v_tail = srv.vtail_left || srv.vtail_right,
    };
}

[[nodiscard]] inline constexpr TailsitterEnable2Effects resolve_enable2(
    std::int8_t enable, fwcpp::vtol_assist::AssistState assist_state,
    fwcpp::quadplane::AirMode air_mode, std::int32_t options) {
    TailsitterEnable2Effects out{};
    out.assist_state = assist_state;
    out.air_mode = air_mode;
    out.options = options;
    if (enable != 2) {
        return out;
    }
    out.applied = true;
    out.assist_state = fwcpp::vtol_assist::AssistState::kForceEnabled;
    out.air_mode = fwcpp::quadplane::AirMode::kAssistedFlightOnly;
    out.options = options | static_cast<std::int32_t>(fwcpp::quadplane::QOption::kOnlyArmInQmodeOrAuto);
    return out;
}

inline void apply_enable2_effects(const TailsitterEnable2Effects& effects,
                                  fwcpp::vtol_assist::AssistState& assist_state,
                                  fwcpp::quadplane::AirMode& air_mode, std::int32_t& options) {
    if (!effects.applied) {
        return;
    }
    assist_state = effects.assist_state;
    air_mode = effects.air_mode;
    options = effects.options;
}

[[nodiscard]] inline TailsitterSetupFlags resolve_setup_flags(const TailsitterSetupResult& setup,
                                                             const TailsitterSetupFlagInputs& in) {
    TailsitterSetupFlags out{};
    out.transition_rate_fw = in.transition_rate_fw;
    out.enable2.assist_state = in.assist_state;
    out.enable2.air_mode = in.air_mode;
    out.enable2.options = in.options;
    if (setup.enable <= 0) {
        return out;
    }
    out.transition_rate_fw = resolve_transition_rate_fw(
        in.transition_rate_fw_configured, in.transition_rate_fw, in.transition_angle_fw,
        in.transition_time_ms);
    out.transition_rate_fw_auto_set = !in.transition_rate_fw_configured;
    out.is_vectored =
        setup_is_vectored(in.frame_class, in.vectored_hover_gain, in.srv.tilt_left, in.srv.tilt_right);
    out.surfaces = resolve_surface_flags(in.srv);
    out.enable2 = resolve_enable2(setup.enable, in.assist_state, in.air_mode, in.options);
    return out;
}

[[nodiscard]] inline TailsitterSetupWithFlags resolve_setup_with_flags(
    const TailsitterSetupInputs& setup_in, const TailsitterSetupFlagInputs& flag_in) {
    TailsitterSetupWithFlags out{};
    out.setup = resolve_setup(setup_in);
    out.flags = resolve_setup_flags(out.setup, flag_in);
    return out;
}

}  // namespace fwcpp::tailsitter
