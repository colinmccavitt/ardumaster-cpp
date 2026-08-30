#pragma once

// Copter Mode base + Stabilize/AltHold/ModeAuto stubs + set_mode checks.
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
// is auto_init (this slice). ModeAuto::run/exit and RTL/LAND stay later.
// update_flight_mode is CCP-035 leftover.

#include <fwcpp/copter/mode_reason.hpp>
#include <fwcpp/copter/pilot_input.hpp>

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
// init leftover is auto_init (mode_auto.cpp ~23-68). run/exit, SubMode,
// and the separate jump_to_landing / return_path_start AUTO_RTL APIs stay later.
class ModeAuto : public Mode {
public:
    bool auto_RTL{false};
    bool waiting_to_start{false};
    bool submode_loiter{false};
    bool auto_yaw_roi_to_hold{false};
    bool wp_spline_init{false};
    bool speed_override_cleared{false};
    bool guided_limit_clear{false};
    bool land_repo_active_cleared{false};

    ModeAuto() = default;

    [[nodiscard]] Number mode_number() const override {
        return auto_RTL ? Number::AUTO_RTL : Number::AUTO;
    }
    // enter_mode calls auto_init for AUTO / AUTO_RTL; this stub stays unused.
    [[nodiscard]] bool init(bool /*ignore_checks*/) override { return true; }
    void run() override {}
    [[nodiscard]] bool requires_position() const override { return true; }
    [[nodiscard]] bool has_manual_throttle() const override { return false; }
};

// Caller-owned table. AUTO_RTL is not a true mode (nullptr from
// mode_from_mode_num); set_mode(AUTO_RTL) uses table.mode_auto.
struct FlightModeTable {
    ModeStabilize stabilize;
    ModeAltHold althold;
    ModeAuto mode_auto;
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
