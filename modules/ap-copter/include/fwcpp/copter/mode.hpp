#pragma once

// Copter Mode base + Stabilize/AltHold stubs + set_mode checks.
// Upstream ArduCopter/mode.h Number ~77-109, Mode virtuals ~119-143,
// ModeStabilize ~1723, ModeAltHold ~498; mode.cpp mode_from_mode_num ~32,
// set_mode ~313-430, exit_mode ~511-524 (non-heli takeoff_stop + exit).
//
// Mode is not a heap singleton. The caller owns FlightModeTable;
// FlightModeContext holds a non-owning Mode* into that table.
// ADR-0012: header-only, C++20, no exceptions, no AP::, no flight-path alloc.
// ACRO/AUTO/RTL bodies stay CCP-039. update_flight_mode is CCP-035 leftover.

#include <fwcpp/copter/mode_reason.hpp>

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

// Caller-owned table. Stabilize and AltHold only this slice; other
// Number values stay unknown (nullptr from mode_from_mode_num).
struct FlightModeTable {
    ModeStabilize stabilize;
    ModeAltHold althold;
};

struct FlightModeContext {
    Mode* current{nullptr};
    ModeReason reason{ModeReason::UNKNOWN};
};

// Injected Copter / motors / EKF / RC state. Upstream reads these via
// AP:: / copter members. FLTMODE_GCSBLOCK param lookup is remaining;
// gcs_mode_enabled is the already-resolved gate (default open).
struct SetModeInputs {
    bool gcs_mode_enabled{true};
    bool armed{false};
    bool land_complete{true};
    float pilot_desired_throttle{0.0f};
    float non_takeoff_throttle{0.0f};
    bool position_ok{true};
    bool ekf_alt_ok{true};
    bool rc_failsafe{false};
};

[[nodiscard]] inline Mode* mode_from_mode_num(Mode::Number mode, FlightModeTable& table) {
    switch (mode) {
        case Mode::Number::STABILIZE:
            return &table.stabilize;
        case Mode::Number::ALT_HOLD:
            return &table.althold;
        default:
            return nullptr;
    }
}

// Post-lookup checks + exit + switch. Upstream set_mode ~359-437
// (ignore_checks through flightmode = new / control_mode_reason = reason).
// Skips HELI runup, MODE_DRIFT_ENABLED throttle special, fence recovery,
// logger.Write_Mode, notify, set_accel_throttle_I.
[[nodiscard]] inline bool enter_mode(FlightModeContext& ctx, Mode& next, ModeReason reason,
                                     const SetModeInputs& in) {
    const bool ignore_checks = !in.armed;

    const bool user_throttle = next.has_manual_throttle();
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

    // Upstream does not gate rc_failsafe on ignore_checks.
    if (in.rc_failsafe && !next.allows_entry_in_rc_failsafe()) {
        return false;
    }

    if (!next.init(ignore_checks)) {
        return false;
    }

    if (ctx.current != nullptr) {
        ctx.current->takeoff_stop();
        ctx.current->exit();
    }

    ctx.current = &next;
    ctx.reason = reason;
    return true;
}

// set_mode by already-constructed Mode (test TestPosMode; also used after lookup).
[[nodiscard]] inline bool set_mode(FlightModeContext& ctx, Mode& next, ModeReason reason,
                                   const SetModeInputs& in) {
    if (ctx.current != nullptr && ctx.current->mode_number() == next.mode_number()) {
        ctx.reason = reason;
        return true;
    }
    if (reason == ModeReason::GCS_COMMAND && !in.gcs_mode_enabled) {
        return false;
    }
    if (next.mode_number() == Mode::Number::AUTO_RTL) {
        return false;
    }
    return enter_mode(ctx, next, reason, in);
}

// Copter::set_mode(Mode::Number, ModeReason). Upstream ~313-430.
[[nodiscard]] inline bool set_mode(FlightModeContext& ctx, FlightModeTable& table, Mode::Number mode,
                                   ModeReason reason, const SetModeInputs& in) {
    if (ctx.current != nullptr && ctx.current->mode_number() == mode) {
        ctx.reason = reason;
        return true;
    }
    if (reason == ModeReason::GCS_COMMAND && !in.gcs_mode_enabled) {
        return false;
    }
    if (mode == Mode::Number::AUTO_RTL) {
        return false;
    }
    Mode* next = mode_from_mode_num(mode, table);
    if (next == nullptr) {
        return false;
    }
    return enter_mode(ctx, *next, reason, in);
}

}  // namespace fwcpp::copter
