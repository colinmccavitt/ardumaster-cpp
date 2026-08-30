#pragma once

// Copter Mode base + Stabilize/AltHold/ModeAuto/ModeRTL stubs + set_mode checks.
// Upstream ArduCopter/mode.h Number ~77-109, Mode virtuals ~119-143,
// ModeStabilize ~1723, ModeAltHold ~498, ModeAuto ~531-545; mode.cpp
// mode_from_mode_num ~32, set_mode ~313-474 (AUTO_RTL ~345-350,
// gcs_mode_enabled ~184-215 / AP_Vehicle::block_GCS_mode_change ~1210-1225),
// Write_Mode ~438-439, notify_flight_mode ~472 / ~549-554),
// exit_mode ~511-524 (non-heli: I transfer then takeoff_stop + exit);
// mode_auto.cpp return_path_or_jump_to_landing_sequence_auto_RTL ~286-301,
// enter_auto_rtl ~303-329 (second Write_Mode after auto_RTL).
//
// Mode is not a heap singleton. The caller owns FlightModeTable;
// FlightModeContext holds a non-owning Mode* into that table.
// ADR-0012: header-only, C++20, no exceptions, no AP::, no flight-path alloc.
// ModeStabilize/Acro/AltHold run bodies are CCP-039. ModeAuto::init leftover
// is auto_init (on main). ModeAuto::exit leftover is on main. ModeAuto::run
// waiting_to_start leftover is on main. Else-path leftover (change detector
// restart + mission.update) is on main. SubMode switch leftover (dispatch
// flags only) is on main. auto_RTL landing-sequence leftover is on
// main. ModeAuto::takeoff_run leftover is on main. ModeAuto::wp_run leftover
// is on main. ModeAuto::land_run leftover is on main. ModeAuto::rtl_run
// leftover is on main. ModeAuto::loiter_run leftover is on main.
// ModeAuto::circle_run leftover is on main. ModeAuto::loiter_to_alt_run
// leftover (ground-handling / reached_xy leftover_wp_run / loiter_start /
// alt_error / land_run_horizontal / climb flags) is on main.
// leftover_surface_tracking_update stays false (AP_RANGEFINDER remaining).
// ModeAuto::nav_guided_run leftover (ModeGuided::run flag) is on main.
// ModeAuto::nav_attitude_time_run leftover (ground-handling +
// constrain/avoidance + leftover leftover_nav_att_lean + leftover leftover_d_set +
// pos_D_update leftover flags) is on main.
// ModeRTL::init leftover (home_is_set gate + leftover wp_and_spline/STARTING
// flags) is on main. leftover leftover_precland_statemachine remaining.
// ModeRTL::run leftover (armed gate + STARTING leftover leftover_build_path /
// leftover leftover_climb_start flags) is this slice. leftover leftover_return_start
// / leftover leftover_climb_return_run remaining. ModeLand, ModeGuided::run body,
// land_run_normal_or_precland body, land_run_horizontal_control body, and
// auto_takeoff.run body stay later.
// update_flight_mode is CCP-035.

#include <fwcpp/copter/mode_reason.hpp>
#include <fwcpp/copter/pilot_input.hpp>

#include <cmath>
#include <cstdint>

namespace fwcpp::copter {

class Mode {
public:
    enum class Number : std::uint8_t {
        STABILIZE = 0,
        ACRO = 1,
        ALT_HOLD = 2,
        AUTO = 3,
        GUIDED = 4,
        LOITER = 5,
        RTL = 6,
        CIRCLE = 7,
        LAND = 9,
        DRIFT = 11,
        SPORT = 13,
        FLIP = 14,
        AUTOTUNE = 15,
        POSHOLD = 16,
        BRAKE = 17,
        THROW = 18,
        AVOID_ADSB = 19,
        GUIDED_NOGPS = 20,
        SMART_RTL = 21,
        FLOWHOLD = 22,
        FOLLOW = 23,
        ZIGZAG = 24,
        SYSTEMID = 25,
        AUTOROTATE = 26,
        AUTO_RTL = 27,
        TURTLE = 28,
    };

    Mode() = default;
    Mode(const Mode&) = delete;
    Mode& operator=(const Mode&) = delete;

    [[nodiscard]] virtual Number mode_number() const = 0;
    [[nodiscard]] virtual bool init(bool /*ignore_checks*/) { return true; }
    virtual void exit() {}
    virtual void run() = 0;
    [[nodiscard]] virtual bool requires_position() const = 0;
    [[nodiscard]] virtual bool has_manual_throttle() const = 0;
    [[nodiscard]] virtual bool allows_entry_in_rc_failsafe() const { return true; }

    // Upstream Mode::takeoff_stop is static and calls takeoff.stop().
    // This slice: no-op stub so exit_mode can call it.
    void takeoff_stop() {}
};

class ModeStabilize : public Mode {
public:
    ModeStabilize() = default;

    [[nodiscard]] Number mode_number() const override { return Number::STABILIZE; }
    [[nodiscard]] bool init(bool /*ignore_checks*/) override { return true; }
    // Body is stabilize_run() in mode_stabilize.hpp (injected context).
    void run() override {}
    [[nodiscard]] bool requires_position() const override { return false; }
    [[nodiscard]] bool has_manual_throttle() const override { return true; }
    [[nodiscard]] bool allows_entry_in_rc_failsafe() const override { return false; }
};

class ModeAltHold : public Mode {
public:
    ModeAltHold() = default;

    [[nodiscard]] Number mode_number() const override { return Number::ALT_HOLD; }
    [[nodiscard]] bool init(bool /*ignore_checks*/) override { return true; }
    void run() override {}
    [[nodiscard]] bool requires_position() const override { return false; }
    [[nodiscard]] bool has_manual_throttle() const override { return false; }
};

// Stub: mode_number AUTO_RTL if auto_RTL else AUTO. requires_position is
// true this slice (upstream NAV_ATTITUDE_TIME exception is leftover).
// init leftover is auto_init (mode_auto.cpp ~23-68). exit leftover is
// ModeAuto::exit (mode_auto.cpp ~71-81). run leftover is waiting_to_start
// + origin (mode_auto.cpp ~85-98, on main), else-path change detector
// + mission.update (mode_auto.cpp ~99-113, on main), SubMode switch
// dispatch flags (mode_auto.cpp ~116-164, on main), auto_RTL
// landing-sequence leftover (mode_auto.cpp ~166-174, on main),
// takeoff_run leftover (mode_auto.cpp ~1075-1083, on main),
// wp_run leftover (mode_auto.cpp ~1087-1107, on main),
// land_run leftover (mode_auto.cpp ~1111-1125, on main),
// rtl_run leftover (mode_auto.cpp ~1129-1133, on main),
// loiter_run leftover (mode_auto.cpp ~1162-1180, on main),
// circle_run leftover (mode_auto.cpp ~1135-1148, on main), and
// loiter_to_alt_run leftover (mode_auto.cpp ~1184-1245, on main).
// Other *_run bodies and set_submode stay later.
class ModeAuto : public Mode {
public:
    // Copter-4.7.0 mode.h ~563-578. NAV_PAYLOAD_PLACE omitted this slice.
    enum class SubMode : std::uint8_t {
        TAKEOFF,
        WP,
        LAND,
        RTL,
        CIRCLE_MOVE_TO_EDGE,
        CIRCLE,
        NAVGUIDED,
        LOITER,
        LOITER_TO_ALT,
        NAV_SCRIPT_TIME,
        NAV_ATTITUDE_TIME,
    };

    bool auto_RTL{false};
    bool waiting_to_start{false};
    bool submode_loiter{false};
    // Injected _mode. Default LOITER (auto_init records submode_loiter).
    SubMode submode{SubMode::LOITER};
    bool auto_yaw_roi_to_hold{false};
    bool wp_spline_init{false};
    bool speed_override_cleared{false};
    bool guided_limit_clear{false};
    bool land_repo_active_cleared{false};
    // Injected mission.state() == MISSION_RUNNING (no AP_Mission).
    bool mission_running{false};
    // Leftover mission.stop() when leaving AUTO while running.
    bool mission_stop{false};
    // HAL_MOUNT_ENABLED camera_mount.set_mode_to_default remaining.
    bool camera_mount_default{false};
    // Injected ahrs.get_origin (no AHRS object).
    bool has_origin{false};
    // Leftover mission.start_or_resume() when waiting_to_start && origin.
    bool start_or_resume{false};
    // Leftover IGNORE_RETURN(mis_change_detector.check_for_mission_change()).
    bool mis_change_check_init{false};
    // Injected mis_change_detector.check_for_mission_change() (else-path).
    bool mission_changed{false};
    // Injected _mode == SubMode::WP for the else-path WP check. Kept this
    // slice; do not replace with `submode == SubMode::WP`.
    bool submode_is_wp{false};
    // Injected mission.restart_current_nav_cmd() result.
    bool restart_nav_ok{false};
    // Leftover restart_current_nav_cmd call when changed && running && WP.
    bool restart_nav_cmd{false};
    // Leftover GCS "restarted command" (no GCS object).
    bool gcs_mission_changed_restarted{false};
    // Leftover GCS "failed to restart command" (no GCS object).
    bool gcs_mission_changed_failed{false};
    // Leftover mission.update() on the else path (always, even if no change).
    bool mission_update{false};
    // AC_NAV_GUIDED / AP_SCRIPTING_ENABLED stand-in. Default closed.
    bool nav_guided_or_scripting{false};
    // SubMode switch leftover flags (no controller objects / *_run bodies).
    bool takeoff_run{false};
    bool wp_run{false};
    bool land_run{false};
    bool rtl_run{false};
    bool circle_run{false};
    bool nav_guided_run{false};
    bool loiter_run{false};
    bool loiter_to_alt_run{false};
    bool nav_attitude_time_run{false};
    // Injected mission.get_in_landing_sequence_flag() (no AP_Mission).
    bool in_landing_sequence{false};
    // Injected mission.get_in_return_path_flag().
    bool in_return_path{false};
    // Injected mission.state() == MISSION_COMPLETE. Do not reuse
    // mission_running (that is MISSION_RUNNING).
    bool mission_complete{false};
    // Leftover logger.Write_Mode after auto_RTL clear (no logger / ctx).
    bool write_mode_auto_rtl_exit{false};
    Number written_mode_number{Number::AUTO};
    ModeReason written_reason{ModeReason::UNKNOWN};
    // Injected Option::AllowTakeOffWithoutRaisingThrottle. No Option
    // enum / g.auto_options bitmask this slice.
    bool allow_takeoff_without_raising_throttle{false};
    // Leftover copter.set_auto_armed(true) when the option is enabled.
    bool set_auto_armed{false};
    // Leftover auto_takeoff.run() call (body stays later).
    bool auto_takeoff_run{false};
    // Injected is_disarmed_or_landed() (no motors / land-complete object).
    bool disarmed_or_landed{false};
    // Leftover make_safe_ground_handling() when disarmed or landed.
    bool make_safe_ground_handling{false};
    // Leftover motors->set_desired_spool_state(THROTTLE_UNLIMITED).
    bool desired_spool_unlimited{false};
    // Leftover wp_nav->update_wpnav() (no wp_nav object).
    bool update_wpnav{false};
    // Leftover copter.failsafe_terrain_set_status (no Copter).
    bool terrain_failsafe_status{false};
    // Leftover pos_control->D_update_controller() (no pos_control).
    bool pos_D_update{false};
    // Leftover attitude_control->input_thrust_vector_heading (no attitude).
    bool input_thrust_vector_heading{false};
    // Leftover land_run_normal_or_precland() (body stays later).
    bool land_run_normal_or_precland{false};
    // Leftover copter.mode_rtl.run(false) from ModeAuto::rtl_run.
    // Does not call ModeRTL::init or run. disarm_on_land is the
    // argument (always false), not a ModeRTL state machine.
    bool leftover_mode_rtl_run{false};
    bool leftover_mode_rtl_disarm_on_land{false};
    // Leftover copter.mode_guided.run() from ModeAuto::nav_guided_run.
    // No ModeGuided object / run body this slice.
    bool leftover_mode_guided_run{false};
    // Leftover circle_nav->update_ms() from ModeAuto::circle_run. No
    // circle_nav object. Distinct from leftover update_wpnav.
    bool leftover_circle_nav_update{false};
    // Injected motors->get_interlock(). Default true so the flying path
    // works without a motors object.
    bool motors_interlock{true};
    // Injected loiter_to_alt.reached_destination_xy (no loiter_to_alt
    // struct / pos_control).
    bool leftover_loiter_to_alt_reached_xy{false};
    // Leftover wp_nav->reached_wp_destination_NE() query when checking xy.
    bool leftover_reached_wp_destination_ne{false};
    // True when leftover_loiter_to_alt_run entered the rest leftover
    // (loiter_start / alt_error / land_run_horizontal flags).
    bool leftover_loiter_to_alt_rest{false};
    // Injected loiter_to_alt.loiter_start_done. Becomes true after the
    // first rest-path loiter_start leftover.
    bool leftover_loiter_start_done{false};
    // Injected pos_control->NE_is_active().
    bool leftover_ne_is_active{false};
    // Leftover pos_control->NE_set_max_speed_accel_m.
    bool leftover_ne_set_max_speed_accel{false};
    // Leftover pos_control->NE_set_correction_speed_accel_m.
    bool leftover_ne_set_correction_speed_accel{false};
    // Leftover pos_control->NE_init_controller() when !NE_is_active.
    bool leftover_ne_init_controller{false};
    // Injected copter.current_loc.alt (cm).
    std::int32_t current_loc_alt_cm{0};
    // Injected loiter_to_alt.alt_m.
    float leftover_loiter_to_alt_alt_m{0.0f};
    // Injected previous loiter_to_alt.alt_error_m (sign-change check).
    float leftover_prev_alt_error_m{0.0f};
    // Computed alt_error_m this tick.
    float leftover_alt_error_m{0.0f};
    // Leftover loiter_to_alt.reached_alt.
    bool leftover_reached_alt{false};
    // Leftover land_run_horizontal_control() call (body remaining).
    bool leftover_land_run_horizontal_control{false};
    // True when leftover_loiter_to_alt_run entered the climb leftover
    // (sqrt_controller / constrain / avoidance / D_set / D_update flags).
    bool leftover_loiter_to_alt_climb{false};
    // leftover leftover_sqrt_controller (flag only; no sqrt_controller body).
    bool leftover_sqrt_controller{false};
    // leftover leftover_constrain_climb (constrain_float on climb rate).
    bool leftover_constrain_climb{false};
    // leftover leftover_avoidance_climbrate (flag only; no AC_Avoid).
    bool leftover_avoidance_climbrate{false};
    // leftover leftover_surface_tracking_update stays false
    // (AP_RANGEFINDER remaining).
    bool leftover_surface_tracking_update{false};
    // leftover leftover_d_set_pos_target_from_climb (no pos_control).
    bool leftover_d_set_pos_target_from_climb{false};
    // leftover leftover_nav_att_lean (lean-angle MAX/MIN/limit_length +
    // leftover leftover_input_euler_angle_roll_pitch_yaw_rad; no bodies).
    bool leftover_nav_att_lean{false};

    ModeAuto() = default;

    [[nodiscard]] Number mode_number() const override {
        return auto_RTL ? Number::AUTO_RTL : Number::AUTO;
    }
    // enter_mode calls auto_init for AUTO / AUTO_RTL; this stub stays unused.
    [[nodiscard]] bool init(bool /*ignore_checks*/) override { return true; }
    // enter_mode already calls current->exit() after takeoff_stop.
    void exit() override {
        mission_stop = mission_running;
        auto_RTL = false;
    }
    // Leftover ModeAuto::takeoff_run (mode_auto.cpp ~1075-1083). No Copter
    // / AutoTakeoff / Option enum. Switch still records takeoff_run as the
    // "would call takeoff_run" leftover, then this helper.
    void leftover_takeoff_run() {
        if (allow_takeoff_without_raising_throttle) {
            set_auto_armed = true;
        }
        auto_takeoff_run = true;
    }
    // Leftover ModeAuto::wp_run (mode_auto.cpp ~1087-1107). No motors /
    // wp_nav / pos_control / attitude objects. Switch still records
    // wp_run as the "would call wp_run" leftover, then this helper.
    void leftover_wp_run() {
        if (disarmed_or_landed) {
            make_safe_ground_handling = true;
            return;
        }
        desired_spool_unlimited = true;
        update_wpnav = true;
        terrain_failsafe_status = true;
        pos_D_update = true;
        input_thrust_vector_heading = true;
    }
    // Leftover ModeAuto::land_run (mode_auto.cpp ~1111-1125). Reuses
    // disarmed_or_landed / make_safe_ground_handling / desired_spool_unlimited.
    // Switch still records land_run as the "would call land_run" leftover,
    // then this helper. land_run_normal_or_precland body stays later.
    void leftover_land_run() {
        if (disarmed_or_landed) {
            make_safe_ground_handling = true;
            return;
        }
        desired_spool_unlimited = true;
        land_run_normal_or_precland = true;
    }
    // Leftover ModeAuto::rtl_run (mode_auto.cpp ~1129-1133). Records
    // ModeRTL::run(false) as flags only. Does not call ModeRTL::init
    // or ModeRTL::run. Switch still records rtl_run as the "would call
    // rtl_run" leftover, then this helper.
    void leftover_rtl_run() {
        leftover_mode_rtl_run = true;
        leftover_mode_rtl_disarm_on_land = false;
    }
    // Leftover ModeAuto::loiter_run (mode_auto.cpp ~1162-1180). Same flags
    // as leftover_wp_run (upstream bodies match). No motors / wp_nav /
    // pos_control / attitude objects. Switch still records loiter_run as
    // the "would call loiter_run" leftover, then this helper.
    // LOITER_TO_ALT does not call this.
    void leftover_loiter_run() {
        leftover_wp_run();
    }
    // Leftover ModeAuto::circle_run (mode_auto.cpp ~1135-1148). Unlike
    // wp_run/loiter_run there is no is_disarmed_or_landed check and no
    // spool. leftover_circle_nav_update stands in for circle_nav->update_ms;
    // do not set update_wpnav. No circle_nav object. Switch still records
    // circle_run as the "would call circle_run" leftover, then this helper.
    // CIRCLE_MOVE_TO_EDGE stays leftover_wp_run.
    void leftover_circle_run() {
        leftover_circle_nav_update = true;
        terrain_failsafe_status = true;
        pos_D_update = true;
        input_thrust_vector_heading = true;
    }
    // Leftover ModeAuto::loiter_to_alt_run (mode_auto.cpp ~1184-1245).
    // Ground-handling + reached_xy leftover_wp_run reuse + loiter_start /
    // alt_error / land_run_horizontal / climb flags. No motors / wp_nav /
    // pos_control. Switch still records loiter_to_alt_run as the
    // "would call loiter_to_alt_run" leftover, then this helper.
    // sqrt_controller / AC_Avoid / surface_tracking / land_run_horizontal
    // body remaining. Do not call leftover_loiter_run.
    void leftover_loiter_to_alt_run() {
        if (disarmed_or_landed || !motors_interlock) {
            make_safe_ground_handling = true;
            return;
        }
        if (!leftover_loiter_to_alt_reached_xy) {
            leftover_reached_wp_destination_ne = true;
            leftover_wp_run();
            return;
        }
        leftover_loiter_to_alt_rest = true;
        if (!leftover_loiter_start_done) {
            leftover_ne_set_max_speed_accel = true;
            leftover_ne_set_correction_speed_accel = true;
            if (!leftover_ne_is_active) {
                leftover_ne_init_controller = true;
            }
            leftover_loiter_start_done = true;
        }
        leftover_alt_error_m = static_cast<float>(current_loc_alt_cm) * 0.01f -
                               leftover_loiter_to_alt_alt_m;
        if (fabsf(leftover_alt_error_m) < 0.05f) {
            leftover_reached_alt = true;
        } else if (leftover_alt_error_m * leftover_prev_alt_error_m < 0.0f) {
            leftover_reached_alt = true;
        }
        leftover_prev_alt_error_m = leftover_alt_error_m;
        leftover_land_run_horizontal_control = true;
        leftover_loiter_to_alt_climb = true;
        leftover_sqrt_controller = true;
        leftover_constrain_climb = true;
        leftover_avoidance_climbrate = true;
        leftover_d_set_pos_target_from_climb = true;
        pos_D_update = true;
    }
    // Leftover ModeAuto::nav_guided_run (mode_auto.cpp ~1150-1158). Records
    // ModeGuided::run as a flag only. No ModeGuided object / run body.
    // Switch still records nav_guided_run as the "would call nav_guided_run"
    // leftover when nav_guided_or_scripting, then this helper.
    void leftover_nav_guided_run() {
        leftover_mode_guided_run = true;
    }
    // Leftover ModeAuto::nav_attitude_time_run (mode_auto.cpp ~1249-1275).
    // Ground-handling + leftover leftover_constrain_climb /
    // leftover leftover_avoidance_climbrate + leftover leftover_nav_att_lean
    // (lean-angle + leftover leftover_input_euler) + leftover leftover_d_set +
    // pos_D_update leftover flags. leftover leftover_sqrt_controller /
    // leftover leftover_surface_tracking_update stay false. Reuses
    // leftover leftover_d_set_pos_target_from_climb / pos_D_update. Reuses
    // disarmed_or_landed / motors_interlock / make_safe_ground_handling.
    // Switch still records nav_attitude_time_run as the
    // "would call nav_attitude_time_run" leftover, then this helper.
    void leftover_nav_attitude_time_run() {
        if (disarmed_or_landed || !motors_interlock) {
            make_safe_ground_handling = true;
            return;
        }
        leftover_constrain_climb = true;
        leftover_avoidance_climbrate = true;
        leftover_nav_att_lean = true;
        leftover_d_set_pos_target_from_climb = true;
        pos_D_update = true;
    }
    // Leftover ModeAuto::run waiting_to_start + origin (mode_auto.cpp ~85-98),
    // else-path change detector + mission.update (~99-113), SubMode switch
    // leftover flags (~116-164), auto_RTL landing-sequence leftover
    // (~166-174), takeoff_run leftover (~1075-1083), wp_run leftover
    // (~1087-1107), land_run leftover (~1111-1125), rtl_run leftover
    // (~1129-1133), loiter_run leftover (~1162-1180), circle_run leftover
    // (~1135-1148), loiter_to_alt_run leftover (~1184-1245),
    // nav_guided_run leftover (~1150-1158), and nav_attitude_time_run
    // leftover (~1249-1275). Switch always runs, including
    // while still waiting_to_start. No AP_Mission / detector / GCS / logger
    // / ModeRTL / ModeGuided / circle_nav / *_run bodies. run has no ctx.
    void run() override {
        if (waiting_to_start) {
            if (has_origin) {
                start_or_resume = true;
                waiting_to_start = false;
                mis_change_check_init = true;
            }
        } else {
            if (mission_changed && mission_running && submode_is_wp) {
                restart_nav_cmd = true;
                if (restart_nav_ok) {
                    gcs_mission_changed_restarted = true;
                } else {
                    gcs_mission_changed_failed = true;
                }
            }
            mission_update = true;
        }

        takeoff_run = false;
        wp_run = false;
        land_run = false;
        rtl_run = false;
        circle_run = false;
        nav_guided_run = false;
        loiter_run = false;
        loiter_to_alt_run = false;
        nav_attitude_time_run = false;
        set_auto_armed = false;
        auto_takeoff_run = false;
        make_safe_ground_handling = false;
        desired_spool_unlimited = false;
        update_wpnav = false;
        terrain_failsafe_status = false;
        pos_D_update = false;
        input_thrust_vector_heading = false;
        land_run_normal_or_precland = false;
        leftover_mode_rtl_run = false;
        leftover_mode_rtl_disarm_on_land = false;
        leftover_mode_guided_run = false;
        leftover_circle_nav_update = false;
        leftover_reached_wp_destination_ne = false;
        leftover_loiter_to_alt_rest = false;
        leftover_ne_set_max_speed_accel = false;
        leftover_ne_set_correction_speed_accel = false;
        leftover_ne_init_controller = false;
        leftover_reached_alt = false;
        leftover_land_run_horizontal_control = false;
        leftover_loiter_to_alt_climb = false;
        leftover_sqrt_controller = false;
        leftover_constrain_climb = false;
        leftover_avoidance_climbrate = false;
        leftover_surface_tracking_update = false;
        leftover_d_set_pos_target_from_climb = false;
        leftover_nav_att_lean = false;

        switch (submode) {
        case SubMode::TAKEOFF:
            takeoff_run = true;
            leftover_takeoff_run();
            break;
        case SubMode::WP:
        case SubMode::CIRCLE_MOVE_TO_EDGE:
            wp_run = true;
            leftover_wp_run();
            break;
        case SubMode::LAND:
            land_run = true;
            leftover_land_run();
            break;
        case SubMode::RTL:
            rtl_run = true;
            leftover_rtl_run();
            break;
        case SubMode::CIRCLE:
            circle_run = true;
            leftover_circle_run();
            break;
        case SubMode::NAVGUIDED:
        case SubMode::NAV_SCRIPT_TIME:
            if (nav_guided_or_scripting) {
                nav_guided_run = true;
                leftover_nav_guided_run();
            }
            break;
        case SubMode::LOITER:
            loiter_run = true;
            leftover_loiter_run();
            break;
        case SubMode::LOITER_TO_ALT:
            loiter_to_alt_run = true;
            leftover_loiter_to_alt_run();
            break;
        case SubMode::NAV_ATTITUDE_TIME:
            nav_attitude_time_run = true;
            leftover_nav_attitude_time_run();
            break;
        }

        // only pretend to be in auto RTL so long as mission still thinks
        // its in a landing sequence or the mission has completed
        const bool auto_rtl_active =
            in_landing_sequence || in_return_path || mission_complete;
        if (auto_RTL && !auto_rtl_active) {
            auto_RTL = false;
            write_mode_auto_rtl_exit = true;
            // Upstream logs flightmode->mode_number() after auto_RTL=false,
            // so AUTO not AUTO_RTL. No FlightModeContext / logger here.
            written_mode_number = mode_number();
            written_reason = ModeReason::AUTO_RTL_EXIT;
        }
    }
    [[nodiscard]] bool requires_position() const override { return true; }
    [[nodiscard]] bool has_manual_throttle() const override { return false; }
};

// Stub: ModeRTL (mode.h ~1495-1570). init leftover is ModeRTL::init
// (mode_rtl.cpp ~80-105). home_is_set / failsafe.terrain injected.
// leftover leftover_precland_statemachine remaining. leftover leftover_run
// is ModeRTL::run (mode.h run() { return run(true); } + mode_rtl.cpp
// ~132-145 armed gate + STARTING leftover leftover_build_path /
// leftover leftover_climb_start flags). Do not dump climb_start /
// return_start / climb_return_run / LAND. ModeAuto leftover leftover_rtl_run
// does not call leftover leftover_run.
class ModeRTL : public Mode {
public:
    enum class SubMode : std::uint8_t {
        STARTING,
        INITIAL_CLIMB,
        RETURN_HOME,
        LOITER_AT_HOME,
        FINAL_DESCENT,
        LAND,
    };

    // Injected AP::ahrs().home_is_set(). Default true so armed+position_ok
    // set_mode(RTL) works without a SetModeInputs field.
    bool home_is_set{true};
    // Injected copter.failsafe.terrain.
    bool failsafe_terrain{false};
    // leftover wp_nav->wp_and_spline_init_m (no wp_nav).
    bool leftover_wp_and_spline_init{false};
    // leftover terrain_following_allowed = !failsafe.terrain.
    bool leftover_terrain_following_allowed{false};
    // leftover copter.ap.land_repo_active / prec_land_active.
    bool leftover_land_repo_active{false};
    bool leftover_prec_land_active{false};
    // leftover leftover_precland_statemachine_init stays false (remaining).
    bool leftover_precland_statemachine_init{false};
    // Injected motors->armed(). Default true so leftover leftover_run
    // proceeds without a motors object.
    bool motors_armed{true};
    // leftover leftover_build_path / leftover leftover_climb_start (flags
    // only; do not change _state to INITIAL_CLIMB; climb_start body remaining).
    bool leftover_build_path{false};
    bool leftover_climb_start{false};
    // leftover leftover_return_start / leftover leftover_climb_return_run
    // stay false this slice.
    bool leftover_return_start{false};
    bool leftover_climb_return_run{false};
    // leftover leftover_rtl_run_disarm_on_land records leftover leftover_run
    // argument (true from run()).
    bool leftover_rtl_run_disarm_on_land{false};
    SubMode _state{SubMode::STARTING};
    bool _state_complete{false};

    ModeRTL() = default;

    [[nodiscard]] Number mode_number() const override { return Number::RTL; }
    [[nodiscard]] bool init(bool ignore_checks) override {
        if (!ignore_checks && !home_is_set) {
            return false;
        }
        leftover_wp_and_spline_init = true;
        _state = SubMode::STARTING;
        _state_complete = true;
        leftover_terrain_following_allowed = !failsafe_terrain;
        leftover_land_repo_active = false;
        leftover_prec_land_active = false;
        return true;
    }
    // Leftover ModeRTL::run(bool) (mode_rtl.cpp ~132-145). Resets leftover
    // leftover_build_path / leftover leftover_climb_start at entry so a later
    // !armed tick does not leave stale true flags. Records leftover
    // leftover_rtl_run_disarm_on_land. Armed gate then STARTING leftover
    // leftover leftover_build_path / leftover leftover_climb_start only.
    // Do not handle other SubModes. Do not run the second switch.
    void leftover_run(bool disarm_on_land) {
        leftover_build_path = false;
        leftover_climb_start = false;
        leftover_rtl_run_disarm_on_land = disarm_on_land;
        if (!motors_armed) {
            return;
        }
        if (_state_complete && _state == SubMode::STARTING) {
            leftover_build_path = true;
            leftover_climb_start = true;
        }
    }
    // upstream mode.h: run() { return run(true); }
    void run() override { leftover_run(true); }
    [[nodiscard]] bool requires_position() const override { return true; }
    [[nodiscard]] bool has_manual_throttle() const override { return false; }

    [[nodiscard]] SubMode state() const { return _state; }
    [[nodiscard]] bool state_complete() const { return _state_complete; }
};

// Caller-owned table. AUTO_RTL is not a true mode (nullptr from
// mode_from_mode_num); set_mode(AUTO_RTL) uses table.mode_auto.
// LAND stays nullptr. RTL is table.mode_rtl.
struct FlightModeTable {
    ModeStabilize stabilize;
    ModeAltHold althold;
    ModeAuto mode_auto;
    ModeRTL mode_rtl;
};

struct FlightModeContext {
    Mode* current{nullptr};
    ModeReason reason{ModeReason::UNKNOWN};
    // Leftover mission.set_force_resume; no AP_Mission this slice.
    bool force_resume{false};
    // exit_mode ~515-518: recorded when manual-to-auto I transfer runs.
    bool accel_throttle_I_set{false};
    float accel_throttle_I{0.0f};
    // set_mode ~438-439 logger.Write_Mode leftover (no AP_Logger object).
    bool write_mode{false};
    Mode::Number written_mode_number{Mode::Number::STABILIZE};
    ModeReason written_reason{ModeReason::UNKNOWN};
    // notify_flight_mode ~549-554 leftover (no AP_Notify object).
    bool notify_flight_mode{false};
    Mode::Number notify_flight_mode_number{Mode::Number::STABILIZE};
    // Upstream AP_Notify::flags.autopilot_mode = flightmode->is_autopilot().
    // Mode has no is_autopilot this slice; true only for ModeAuto.
    bool notify_autopilot_mode{false};
    // gcs().send_message(MSG_HEARTBEAT) remaining; no GCS this slice.
    bool gcs_heartbeat{false};
    // fence.manual_recovery_start leftover (no AC_Fence this slice).
    bool fence_manual_recovery_start{false};
};

// Injected Copter / motors / EKF / RC / mission-jump state. Upstream
// reads these via AP:: / copter / mission members. FLTMODE_GCSBLOCK is
// injected as fltmode_gcsblock (default 0 = nothing blocked). The
// gcs_mode_enabled bool remains the already-resolved gate (default open).
// Jump flags default false so `{}` fails AUTO_RTL.
// is_drift injects MODE_DRIFT_ENABLED user_throttle leftover.
struct SetModeInputs {
    bool gcs_mode_enabled{true};
    // FLTMODE_GCSBLOCK leftover: bit i blocks mode_list[i] from GCS.
    std::uint32_t fltmode_gcsblock{0};
    bool armed{false};
    bool land_complete{true};
    float pilot_desired_throttle{0.0f};
    float throttle_hover{0.5f};
    float non_takeoff_throttle{0.0f};
    bool position_ok{true};
    bool ekf_alt_ok{true};
    bool rc_failsafe{false};
    bool jump_to_closest_mission_leg{false};
    bool jump_to_landing_sequence{false};
    // MODE_DRIFT_ENABLED leftover: treat next as user_throttle for the
    // throttle-too-high gate. Default false; no ModeDrift this slice.
    bool is_drift{false};
    // Fence leftovers (no AC_Fence). Defaults keep existing tests open
    // and do not record manual_recovery_start.
    bool fence_enabled{false};
    bool fence_disable_mode_change{false};
    bool fence_breaches{false};
    // AP_FENCE_ENABLED stand-in for the post-switch leftover.
    bool fence_present{false};
    bool fence_action_report_only{true};
    // ModeAuto::init leftovers (no AP_Mission / AutoYaw objects).
    // mission_present defaults true so existing armed AUTO tests still pass.
    bool mission_present{true};
    bool starts_with_takeoff{false};
    bool yaw_mode_is_roi{false};
};

// Leftover ModeAuto::init (mode_auto.cpp ~23-68). No mission / wp_nav /
// guided / precland objects. precland_statemachine.init remaining.
[[nodiscard]] inline bool auto_init(ModeAuto& mode, bool ignore_checks, const SetModeInputs& in) {
    mode.auto_RTL = false;
    if (in.mission_present || ignore_checks) {
        if (in.armed && in.land_complete && !in.starts_with_takeoff) {
            return false;
        }
        mode.submode_loiter = true;
        if (in.yaw_mode_is_roi) {
            mode.auto_yaw_roi_to_hold = true;
        }
        mode.wp_spline_init = true;
        mode.speed_override_cleared = true;
        mode.waiting_to_start = true;
        mode.guided_limit_clear = true;
        mode.land_repo_active_cleared = true;
        return true;
    }
    return false;
}

// Copter::gcs_mode_enabled ~184-215 + AP_Vehicle::block_GCS_mode_change ~1210-1225.
// mode_list index is the FLTMODE_GCSBLOCK bit. Default mask 0 allows all.
// Modes not in the list (LAND=9, RTL=6) are never blocked.
[[nodiscard]] inline constexpr bool gcs_mode_enabled(Mode::Number mode,
                                                     std::uint32_t fltmode_gcsblock) {
    constexpr Mode::Number kModeList[] = {
        Mode::Number::STABILIZE,
        Mode::Number::ACRO,
        Mode::Number::ALT_HOLD,
        Mode::Number::AUTO,
        Mode::Number::GUIDED,
        Mode::Number::LOITER,
        Mode::Number::CIRCLE,
        Mode::Number::DRIFT,
        Mode::Number::SPORT,
        Mode::Number::FLIP,
        Mode::Number::AUTOTUNE,
        Mode::Number::POSHOLD,
        Mode::Number::BRAKE,
        Mode::Number::THROW,
        Mode::Number::AVOID_ADSB,
        Mode::Number::GUIDED_NOGPS,
        Mode::Number::SMART_RTL,
        Mode::Number::FLOWHOLD,
        Mode::Number::FOLLOW,
        Mode::Number::ZIGZAG,
        Mode::Number::SYSTEMID,
        Mode::Number::AUTOROTATE,
        Mode::Number::AUTO_RTL,
        Mode::Number::TURTLE,
    };
    constexpr auto kCount =
        static_cast<std::uint8_t>(sizeof(kModeList) / sizeof(kModeList[0]));
    static_assert(kCount == 24);
    const auto mode_num = static_cast<std::uint8_t>(mode);
    for (std::uint8_t i = 0; i < kCount; ++i) {
        if (static_cast<std::uint8_t>(kModeList[i]) == mode_num) {
            return (fltmode_gcsblock & (1U << i)) == 0U;
        }
    }
    return true;
}

[[nodiscard]] inline Mode* mode_from_mode_num(Mode::Number mode, FlightModeTable& table) {
    switch (mode) {
        case Mode::Number::STABILIZE:
            return &table.stabilize;
        case Mode::Number::ALT_HOLD:
            return &table.althold;
        case Mode::Number::AUTO:
            return &table.mode_auto;
        case Mode::Number::RTL:
            return &table.mode_rtl;
        default:
            return nullptr;
    }
}

// Leftover logger.Write_Mode((uint8_t)mode_number, reason). No AP_Logger.
inline void record_write_mode(FlightModeContext& ctx, Mode::Number mode, ModeReason reason) {
    ctx.write_mode = true;
    ctx.written_mode_number = mode;
    ctx.written_reason = reason;
}

// Leftover notify_flight_mode. No AP_Notify / name4 virtual this slice.
// autopilot_mode is true only for ModeAuto (AUTO or AUTO_RTL).
inline void record_notify_flight_mode(FlightModeContext& ctx, const Mode& next) {
    ctx.notify_flight_mode = true;
    ctx.notify_flight_mode_number = next.mode_number();
    const Mode::Number n = next.mode_number();
    ctx.notify_autopilot_mode = (n == Mode::Number::AUTO || n == Mode::Number::AUTO_RTL);
}

// Post-lookup checks + exit + switch. Upstream set_mode ~359-472
// (ignore_checks through Write_Mode + notify_flight_mode leftovers).
// Drift leftover: MODE_DRIFT_ENABLED forces user_throttle true
// (mode.cpp ~375-380); injected is_drift, no ModeDrift class.
// Fence leftovers: DISABLE_MODE_CHANGE gate (~408-419) +
// manual_recovery_start (~447-453); no AC_Fence.
// Skips HELI runup, GCS heartbeat, ADSB/camera/rate_tc, AP_Notify sounds.
[[nodiscard]] inline bool enter_mode(FlightModeContext& ctx, Mode& next, ModeReason reason,
                                     const SetModeInputs& in) {
    const bool ignore_checks = !in.armed;

    // Upstream: user_throttle = next.has_manual_throttle(); then Drift
    // pointer-compare forces true. Injected is_drift stands in for
    // new_flightmode == &mode_drift.
    bool user_throttle = next.has_manual_throttle();
    if (in.is_drift) {
        user_throttle = true;
    }
    if (!ignore_checks && in.land_complete && user_throttle && ctx.current != nullptr &&
        !ctx.current->has_manual_throttle() &&
        in.pilot_desired_throttle > in.non_takeoff_throttle) {
        return false;
    }

    if (!ignore_checks && next.requires_position() && !in.position_ok) {
        return false;
    }

    if (!ignore_checks && !in.ekf_alt_ok && ctx.current != nullptr &&
        ctx.current->has_manual_throttle() && !next.has_manual_throttle()) {
        return false;
    }

    // Upstream get_control_mode_reason() is the current reason, not incoming.
    if (!ignore_checks && in.fence_enabled && in.fence_disable_mode_change &&
        in.fence_breaches && in.armed && ctx.reason == ModeReason::FENCE_BREACHED &&
        !in.land_complete) {
        return false;
    }

    // Upstream does not gate rc_failsafe on ignore_checks.
    if (in.rc_failsafe && !next.allows_entry_in_rc_failsafe()) {
        return false;
    }

    const Mode::Number next_num = next.mode_number();
    if (next_num == Mode::Number::AUTO || next_num == Mode::Number::AUTO_RTL) {
        if (!auto_init(static_cast<ModeAuto&>(next), ignore_checks, in)) {
            return false;
        }
    } else if (!next.init(ignore_checks)) {
        return false;
    }

    if (ctx.current != nullptr) {
        // Upstream Copter::exit_mode ~511-524: I transfer, then takeoff_stop, then exit.
        ctx.accel_throttle_I_set = false;
        ctx.accel_throttle_I = 0.0f;
        if (ctx.current->has_manual_throttle() && !next.has_manual_throttle() && in.armed &&
            !in.land_complete) {
            // leftover injects pilot_desired_throttle (upstream get_throttle_in).
            ctx.accel_throttle_I = set_accel_throttle_I_from_pilot_throttle(
                in.pilot_desired_throttle, in.throttle_hover);
            ctx.accel_throttle_I_set = true;
        }
        ctx.current->takeoff_stop();
        ctx.current->exit();
    }

    ctx.current = &next;
    ctx.reason = reason;
    record_write_mode(ctx, next.mode_number(), reason);
    record_notify_flight_mode(ctx, next);
    if (in.fence_present && !in.fence_action_report_only) {
        ctx.fence_manual_recovery_start = true;
    }
    return true;
}

// set_mode by already-constructed Mode (test TestPosMode; also used after lookup).
[[nodiscard]] inline bool set_mode(FlightModeContext& ctx, Mode& next, ModeReason reason,
                                   const SetModeInputs& in) {
    if (ctx.current != nullptr && ctx.current->mode_number() == next.mode_number()) {
        ctx.reason = reason;
        return true;
    }
    if (reason == ModeReason::GCS_COMMAND &&
        (!in.gcs_mode_enabled || !gcs_mode_enabled(next.mode_number(), in.fltmode_gcsblock))) {
        return false;
    }
    if (next.mode_number() == Mode::Number::AUTO_RTL) {
        return false;
    }
    return enter_mode(ctx, next, reason, in);
}

[[nodiscard]] inline bool set_mode(FlightModeContext& ctx, FlightModeTable& table, Mode::Number mode,
                                   ModeReason reason, const SetModeInputs& in);

// Upstream ModeAuto::enter_auto_rtl ~303-329. Skips LOGGER_WRITE_ERROR,
// GCS send_text, AP_Notify sounds. Write_Mode leftover after auto_RTL
// so the logged number is AUTO_RTL, not AUTO.
[[nodiscard]] inline bool enter_auto_rtl(FlightModeContext& ctx, FlightModeTable& table,
                                         ModeReason reason, const SetModeInputs& in) {
    ctx.force_resume = true;
    if (ctx.current == &table.mode_auto ||
        set_mode(ctx, table, Mode::Number::AUTO, reason, in)) {
        table.mode_auto.auto_RTL = true;
        record_write_mode(ctx, table.mode_auto.mode_number(), reason);
        return true;
    }
    ctx.force_resume = false;
    table.mode_auto.auto_RTL = false;
    return false;
}

// Upstream ModeAuto::return_path_or_jump_to_landing_sequence_auto_RTL
// ~286-301. Injected jumps; first success short-circuits the second.
[[nodiscard]] inline bool return_path_or_jump_to_landing_sequence_auto_RTL(
    FlightModeContext& ctx, FlightModeTable& table, ModeReason reason, const SetModeInputs& in) {
    if (!in.jump_to_closest_mission_leg && !in.jump_to_landing_sequence) {
        return false;
    }
    return enter_auto_rtl(ctx, table, reason, in);
}

// Copter::set_mode(Mode::Number, ModeReason). Upstream ~313-430.
// GCS_COMMAND gate runs before the AUTO_RTL special case.
[[nodiscard]] inline bool set_mode(FlightModeContext& ctx, FlightModeTable& table, Mode::Number mode,
                                   ModeReason reason, const SetModeInputs& in) {
    if (ctx.current != nullptr && ctx.current->mode_number() == mode) {
        ctx.reason = reason;
        return true;
    }
    if (reason == ModeReason::GCS_COMMAND &&
        (!in.gcs_mode_enabled || !gcs_mode_enabled(mode, in.fltmode_gcsblock))) {
        return false;
    }
    if (mode == Mode::Number::AUTO_RTL) {
        return return_path_or_jump_to_landing_sequence_auto_RTL(ctx, table, reason, in);
    }
    Mode* next = mode_from_mode_num(mode, table);
    if (next == nullptr) {
        return false;
    }
    return enter_mode(ctx, *next, reason, in);
}

}  // namespace fwcpp::copter
