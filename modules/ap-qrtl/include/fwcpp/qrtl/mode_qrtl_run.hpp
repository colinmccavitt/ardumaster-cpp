#pragma once

#include <fwcpp/q_modes/q_run_common.hpp>
#include <fwcpp/qrtl/mode_qrtl_enter.hpp>
#include <fwcpp/qrtl/mode_qrtl_land_handoff.hpp>
#include <fwcpp/qrtl/qrtl_defaults.hpp>
#include <fwcpp/qrtl/qrtl_geometry.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

#include <cstdint>

namespace fwcpp::qrtl {

using fwcpp::quadplane::PosControlState;

enum class QrtlRunAction : std::uint8_t {
    kFwControllers = 0,
    kClimb = 1,
    kClimbThenReturn = 2,
    kReturn = 3,
};

struct QrtlHeightAbove {
    bool valid{false};
    float meters{0.0F};
};

struct QrtlRunView {
    bool tailsitter_in_vtol_transition{false};
    QrtlSubMode submode{QrtlSubMode::kClimb};
    float home_dist_m{200.0F};
    float rally_dist_m{0.0F};
    bool has_rally{false};
    bool rally_incl_home{true};
    float loiter_radius_m{kWpLoiterRadDefaultM};
    float rtl_radius_m{kRtlRadiusDefaultM};
    float qrtl_alt_m{kQRtlAltDefaultM};
    std::int32_t current_alt_abs_cm{1000};
    std::int32_t home_alt_abs_cm{0};
    std::int32_t next_wp_alt_abs_cm{1500};
    QrtlHeightAbove stopping_height_above_next_wp{};
    QrtlHeightAbove current_height_above_next_wp{};
    bool vtol_roll_pitch_limited{false};
    float wp_speed_up_ms{kQWpSpdUpDefaultMs};
    fwcpp::quadplane::PositionControlState poscontrol_state{
        fwcpp::quadplane::PositionControlState::kNone};
};

struct QrtlRunEffects {
    bool set_position1{false};
    bool do_rtl{false};
    bool poscontrol_init_approach{false};
    bool slow_descent{false};
};

struct QrtlRunResult {
    QrtlRunAction action{QrtlRunAction::kReturn};
    QrtlSubMode submode{QrtlSubMode::kRtl};
    QrtlDestination dest{QrtlDestination::kHome};
    float dist_m{0.0F};
    float radius_m{0.0F};
    float climb_rate_ms{0.0F};
    bool xy_hold{false};
    bool tilt_assigned{false};
    bool ne_externally_limited{false};
    bool weathervane{false};
    bool z_controller{false};
    bool position1{false};
    bool do_rtl{false};
    bool poscontrol_init_approach{false};
    bool slow_descent{false};
    std::int32_t rtl_alt_abs_cm{0};
    bool vtol_position_controller{false};
    bool fw_stabilize{false};
    bool copy_home_alt{false};
    bool verify_vtol_land{false};
    bool stick_mixing_fbw{false};
    bool delegate_mode_run{false};
};

[[nodiscard]] inline bool qrtl_climb_finished(const QrtlHeightAbove& stopping_height_above_next_wp) {
    if (!stopping_height_above_next_wp.valid) {
        return true;
    }
    return stopping_height_above_next_wp.meters > 0.0F;
}

[[nodiscard]] inline QrtlRunView qrtl_run_view_climbing() {
    QrtlRunView view{};
    view.stopping_height_above_next_wp.valid = true;
    view.stopping_height_above_next_wp.meters = -5.0F;
    return view;
}

[[nodiscard]] inline QrtlRunView qrtl_run_view_climb_done_far() {
    QrtlRunView view = qrtl_run_view_climbing();
    view.current_alt_abs_cm = 1500;
    view.stopping_height_above_next_wp.meters = 1.0F;
    view.current_height_above_next_wp.valid = true;
    view.current_height_above_next_wp.meters = 0.0F;
    return view;
}

[[nodiscard]] inline QrtlRunView qrtl_run_view_climb_done_close() {
    QrtlRunView view = qrtl_run_view_climb_done_far();
    view.home_dist_m = 50.0F;
    view.next_wp_alt_abs_cm = 1200;
    view.current_alt_abs_cm = 1200;
    return view;
}

[[nodiscard]] inline QrtlRunView qrtl_run_view_returning() {
    QrtlRunView view = qrtl_run_view_climbing();
    view.submode = QrtlSubMode::kRtl;
    view.stopping_height_above_next_wp = {};
    return view;
}

[[nodiscard]] inline QrtlRunView qrtl_run_view_tailsitter_fw_transition() {
    QrtlRunView view = qrtl_run_view_climbing();
    view.tailsitter_in_vtol_transition = true;
    return view;
}

namespace detail {

[[nodiscard]] inline QrtlRunResult qrtl_run_climb_tick(const QrtlRunView& view,
                                                       PosControlState& pc,
                                                       QrtlRunEffects& effects) {
    const float radius_m = qrtl_vtol_return_radius_m(view.loiter_radius_m, view.rtl_radius_m);
    const std::int32_t rtl_alt_abs_cm =
        view.home_alt_abs_cm + static_cast<std::int32_t>(view.qrtl_alt_m * 100.0F);

    if (!qrtl_climb_finished(view.stopping_height_above_next_wp)) {
        QrtlRunResult out{};
        out.action = QrtlRunAction::kClimb;
        out.submode = QrtlSubMode::kClimb;
        out.dest = QrtlDestination::kHome;
        out.dist_m = view.home_dist_m;
        out.radius_m = radius_m;
        out.climb_rate_ms = view.wp_speed_up_ms;
        out.xy_hold = true;
        out.tilt_assigned = true;
        out.ne_externally_limited = view.vtol_roll_pitch_limited;
        out.weathervane = true;
        out.z_controller = true;
        out.rtl_alt_abs_cm = rtl_alt_abs_cm;
        out.fw_stabilize = true;
        return out;
    }

    const QrtlDestination dest =
        calc_best_rally_or_home(view.home_dist_m, view.rally_dist_m, view.rally_incl_home,
                              view.has_rally);
    const float dist_m = qrtl_destination_dist_m(dest, view.home_dist_m, view.rally_dist_m);
    std::int32_t rtl_alt_out_cm = rtl_alt_abs_cm;
    bool position1 = false;
    if (dist_m < radius_m) {
        if (view.next_wp_alt_abs_cm < rtl_alt_out_cm) {
            rtl_alt_out_cm = view.next_wp_alt_abs_cm;
        }
        pc.state = fwcpp::quadplane::PositionControlState::kPosition1;
        effects.set_position1 = true;
        position1 = true;
    }

    bool slow_descent = false;
    if (view.current_height_above_next_wp.valid) {
        slow_descent = is_positive(view.current_height_above_next_wp.meters);
    } else {
        slow_descent = view.current_alt_abs_cm > rtl_alt_out_cm;
    }
    effects.do_rtl = true;
    effects.poscontrol_init_approach = true;
    effects.slow_descent = slow_descent;
    pc.slow_descent = slow_descent;

    QrtlRunResult out{};
    out.action = QrtlRunAction::kClimbThenReturn;
    out.submode = QrtlSubMode::kRtl;
    out.dest = dest;
    out.dist_m = dist_m;
    out.radius_m = radius_m;
    out.climb_rate_ms = view.wp_speed_up_ms;
    out.xy_hold = true;
    out.tilt_assigned = true;
    out.ne_externally_limited = view.vtol_roll_pitch_limited;
    out.weathervane = true;
    out.z_controller = true;
    out.position1 = position1;
    out.do_rtl = true;
    out.poscontrol_init_approach = true;
    out.slow_descent = slow_descent;
    out.rtl_alt_abs_cm = rtl_alt_out_cm;
    out.fw_stabilize = true;
    return out;
}

[[nodiscard]] inline QrtlRunResult qrtl_run_return_tick(const QrtlRunView& view) {
    const QrtlDestination dest =
        calc_best_rally_or_home(view.home_dist_m, view.rally_dist_m, view.rally_incl_home,
                              view.has_rally);
    const float dist_m = qrtl_destination_dist_m(dest, view.home_dist_m, view.rally_dist_m);
    const QrtlLandHandoff handoff = qrtl_land_handoff(view.poscontrol_state);
    const std::int32_t rtl_alt_abs_cm =
        view.home_alt_abs_cm + static_cast<std::int32_t>(view.qrtl_alt_m * 100.0F);

    QrtlRunResult out{};
    out.action = QrtlRunAction::kReturn;
    out.submode = QrtlSubMode::kRtl;
    out.dest = dest;
    out.dist_m = dist_m;
    out.radius_m = qrtl_vtol_return_radius_m(view.loiter_radius_m, view.rtl_radius_m);
    out.rtl_alt_abs_cm = rtl_alt_abs_cm;
    out.vtol_position_controller = true;
    out.fw_stabilize = true;
    out.copy_home_alt = handoff.copy_home_alt;
    out.verify_vtol_land = handoff.verify_vtol_land;
    out.stick_mixing_fbw = handoff.stick_mixing_fbw;
    return out;
}

}  // namespace detail

/// Port of ModeQRTL::run control flow (tailsitter FW pull-up, climb, RTL + land handoff).
[[nodiscard]] inline QrtlRunResult qrtl_run(const QrtlRunView& view, PosControlState& pc,
                                            QrtlRunEffects& effects) {
    effects = QrtlRunEffects{};
    if (fwcpp::q_modes::run_delegates_to_fw_controllers(view.tailsitter_in_vtol_transition)) {
        QrtlRunResult out{};
        out.action = QrtlRunAction::kFwControllers;
        out.submode = QrtlSubMode::kClimb;
        out.delegate_mode_run = true;
        return out;
    }

    switch (view.submode) {
        case QrtlSubMode::kClimb:
            return detail::qrtl_run_climb_tick(view, pc, effects);
        case QrtlSubMode::kRtl:
            return detail::qrtl_run_return_tick(view);
    }
    return detail::qrtl_run_return_tick(view);
}

}  // namespace fwcpp::qrtl
