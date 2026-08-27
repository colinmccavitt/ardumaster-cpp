#pragma once

// Port of APM_Control/AP_PitchController.h + AP_PitchController.cpp.
// CPP-032. Initial code by Jon Challinger, modified by Paul Riseborough.
// Upstream: libraries/APM_Control/AP_PitchController.{h,cpp}
// (Plane-4.7.0, 28 + 336 lines) - read directly from the pinned upstream
// worktree in full before writing a line of this file, not from
// training-data memory.
//
// See fw_controller.hpp's file banner for the module-wide judgment calls
// this file relies on (composition over inheritance, RateLoopInputs, the
// AP_AutoTune exclusion).
//
// PITCH HAS REAL ADDITIONAL COMPLEXITY ROLL DOESN'T (per the task's own
// warning - confirmed by reading both files in full): get_servo_out()
// adds two whole mechanisms roll's equivalent has no counterpart for:
//   1) _get_coordination_rate_offset() (ported below as
//      get_coordination_rate_offset(), private): a gravity/kinematic
//      correction term - the pitch rate needed, purely from bank angle
//      and airspeed, to hold height in a coordinated turn (a banked
//      aircraft's nose must rise at a rate related to
//      g*tan(bank)*sin(bank)/V to avoid descending). Added to the angle-
//      error-derived desired_rate before the rmax clamp, unless inverted
//      (see get_servo_out() below - inverted flips this from an addend
//      rate offset to preserved sign while desired_rate itself flips).
//   2) The roll-limit pitch-authority blend at the bottom of
//      get_servo_out(): once bank angle exceeds the configured roll
//      limit (plus an 8500cd-capped margin), pitch authority is linearly
//      reduced to zero at 90 degrees of bank - upstream's own comment:
//      "Using elevator for pitch control at large roll angles is
//      ineffective, and can be counter productive as it induces earth-
//      frame yaw which can reduce the ability to roll."
//
// var_info[] DEFAULTS (AP_PitchController::var_info,
// AP_PitchController.cpp) transcribed into Gains below:
//   PTCH2SRV_TCONST   -> tau        (0.5f)
//   PTCH2SRV_RMAX_UP  -> rmax_pos   (0.0f, disabled)
//   PTCH2SRV_RMAX_DN  -> rmax_neg   (0.0f, disabled)
//   PTCH2SRV_RLL      -> roll_ff    (1.0f) - upstream's `_roll_ff`, the
//                         turn-coordination gain multiplier
//   _RATE_P/_I/_D/_FF/_IMAX/_FLTT/_FLTE/_FLTD/_SMAX (AP_SUBGROUPINFO,
//   index 11) -> Gains::rate_pid, defaults from AP_PitchController's
//   constructor's AC_PID::Defaults initializer (NOT var_info, which only
//   documents ranges/units - identical values to roll's):
//     p=0.04, i=0.15, d=0.0, ff=0.345, imax=0.666, filt_T_hz=3.0,
//     filt_E_hz=0.0, filt_D_hz=12.0, srmax=150.0, srtau=1.0
//   (note: p differs from roll's 0.04 vs 0.08 - transcribed exactly as
//   each controller's own constructor initializes it, not assumed equal)
//
// convert_pid() NOT PORTED - same EEPROM/AP_Param-migration reasoning as
// roll_controller.hpp.
//
// LITERAL SAFETY: no bare ambiguous double literals - every upstream bare
// double literal (radians(90), 0.01, 7000/70deg-equivalent, etc.) is
// explicitly float-suffixed here with no value change.
//
// === CPP-045 ADDENDUM: real top-level AP_Param Info[] table for
// PitchController::Gains, load/save round-trip ===
//
// Phase 2b of the AP_Param vehicle-integration effort CPP-043 began
// (phase 1: Plane::aparm - a FLAT top-level table, since upstream's own
// aparm object turned out not to be a real GROUP at all, CPP-043's own
// finding #1). This ticket covers PitchController::Gains, which is a
// DIFFERENT shape: upstream's real AP_PitchController genuinely IS
// registered as a nested GROUP. Verified by reading both real upstream
// files in full, not assumed (this ticket's own scope text guessed the
// GOBJECT prefix was "PTCH2SRV_" - reading ArduPlane/Parameters.cpp
// directly shows that guess is WRONG):
//
//   ArduPlane/Parameters.cpp: `GOBJECT(pitchController, "PTCH",
//   AP_PitchController)` - the real GOBJECT prefix string is the bare 4
//   characters "PTCH", not "PTCH2SRV_". Every var_info[] entry name
//   below is concatenated onto "PTCH" by AP_Param's own group-name
//   matching at runtime (find_group, name_lookup.hpp) to form the real
//   full names a GCS actually shows - e.g. "PTCH" + "2SRV_TCONST" =
//   "PTCH2SRV_TCONST". The "PTCH2SRV_" this file's own pre-existing
//   Gains field comments (above) and this ticket's scope text both use
//   is the real FULL name's common prefix, coincidentally readable as
//   its own string, but it is NOT the GOBJECT prefix argument itself.
//
//   libraries/APM_Control/AP_PitchController.cpp's real var_info[]
//   (read in full):
//     AP_GROUPINFO("2SRV_TCONST",  0, AP_PitchController, gains.tau,       0.5f)
//     // idx 1-3 reserved for old PID values (upstream's own comment) -
//     // no live field, upstream or here, corresponds to them.
//     AP_GROUPINFO("2SRV_RMAX_UP", 4, AP_PitchController, gains.rmax_pos, 0.0f)
//     AP_GROUPINFO("2SRV_RMAX_DN", 5, AP_PitchController, gains.rmax_neg, 0.0f)
//     AP_GROUPINFO("2SRV_RLL",     6, AP_PitchController, _roll_ff,       1.0f)
//     // idx 7-8 reserved for old IMAX/FF (upstream's own comment) - same
//     // "no live field" situation as idx 1-3.
//     AP_SUBGROUPINFO(rate_pid, "_RATE_", 11, AP_PitchController, AC_PID)
//
//   So of Gains' 5 fields, 4 have a genuine, DIRECT upstream AP_Param
//   backing: tau, rmax_pos, rmax_neg, and roll_ff (upstream's own
//   `_roll_ff` maps 1:1 onto this port's `Gains::roll_ff` - confirmed by
//   reading AP_PitchController.h's own member declaration alongside the
//   var_info line above, exactly the mapping this file's pre-existing
//   banner already recorded). The 5th, `rate_pid`, IS a real upstream
//   AP_Param too (AP_SUBGROUPINFO index 11 -> real full names like
//   "PTCH_RATE_P") but is DELIBERATELY EXCLUDED from the table below -
//   see "DEFERRED" below for why.
//
// NO GROUP `Info` WRAPPER BUILT HERE - a DIFFERENT reason than CPP-043's
// aparm (whose real upstream object simply isn't a GROUP at all).
// PitchController::Gains genuinely IS a real upstream GROUP, but this
// ticket is expressly scoped to touch ONLY this file (plus its own test
// file and tests/CMakeLists.txt) - not plane.hpp/mode.hpp, the only
// place a Plane-wide top-level table exists to nest a GROUP `Info` entry
// inside. This is deliberate: CPP-044/046/047/048/049 each need to add
// their OWN analogous entry to that same vehicle-wide table concurrently,
// in six independent, isolated clones - touching plane.hpp here would
// either be dead code (no real table to point a GROUP entry into from
// this module) or, if plane.hpp WERE edited, a guaranteed merge conflict
// with every sibling ticket editing the identical file at the same time.
// `pitch_param_info()` below is instead a SELF-CONTAINED, FLAT top-level
// table using each field's REAL FULL name (GOBJECT prefix "PTCH" +
// var_info name, pre-concatenated by hand, e.g. "PTCH2SRV_TCONST").
// `top_level::find()` (CPP-043, fwcpp/param/top_level.hpp, reused here
// completely unchanged) works identically against a flat table whether
// or not a future integration ticket later re-wires it as a nested GROUP
// under Plane's own table - find()'s GROUP-dispatch branch only matters
// to whichever caller holds the OUTER table, not to this module's own
// table or its own find()-by-name test.
//
// KEY ALLOCATION: `PitchParamKey` below is this port's OWN top-level key
// space for these 4 fields - informed by, but independent of, upstream's
// real Parameters.h `k_param_*` enum (an EEPROM-migration/ordering detail
// out of this port's scope, same reasoning as CPP-043's own
// `AparmParamKey`) and ALSO independent of `AparmParamKey`'s own key
// space (plane.hpp, CPP-043) - these are two separate, self-contained
// top-level tables for now, each numbering from 1. A future integration
// ticket that merges every phase-2 table under one real Plane-wide
// top-level table will need to allocate a single non-overlapping key
// space at that point (matching CPP-043's own "no full vehicle-wide key
// space exists yet" note) - not attempted here.
//
// WHY native_value.hpp, NOT CPP-022 slice 6/7's set_value/cast_to_float:
// every one of Gains' 4 real-backed fields (tau, rmax_pos, rmax_neg,
// roll_ff) is declared as a plain C++ `float`, not this port's own
// `param::ParamFloat` wrapper class (param.hpp) - reinterpreting a plain
// `float` object as a `ParamFloat` object to call the latter's member
// functions would be exactly the unsafe reinterpretation ADR-0012
// forbids (same reasoning CPP-043's own file banner already gives for
// aparm's fields, which are equally plain `float`/`bool`).
// `fwcpp/param/native_value.hpp`'s memcpy-based `set_native_value`/
// `native_cast_to_float` (CPP-043, reused here completely unchanged) are
// the honest fit. `find_group`/`get_base`/`load_raw`/`save_raw`/`scan`/
// `should_skip_save`/`type_size` (CPP-022, persistence.hpp/
// group_info.hpp) are likewise reused completely unchanged - none of
// them touch the pointee's static type at all.
//
// DEFERRED, NAMED EXPLICITLY (not silently skipped):
//   - `Gains::rate_pid` (`pid::AcPid::Gains`) - upstream's real
//     `AP_SUBGROUPINFO(rate_pid, "_RATE_", 11, ...)` nested GROUP
//     (P/I/D/FF/IMAX/FLTT/FLTE/FLTD/SMAX/PDMX/D_FF/NTF/NEF sub-fields).
//     `AC_PID` (modules/ap-pid/include/fwcpp/pid/ac_pid.hpp) is SHARED
//     infrastructure reused, unmodified, by RollController,
//     PitchController, YawController, and SteerController alike - the
//     roll/yaw/steer siblings of this very ticket (CPP-044/047/046 in
//     this project's own numbering) each embed the IDENTICAL
//     `pid::AcPid::Gains` type. Porting AC_PID's own var_info would
//     require editing ac_pid.hpp - a file this ticket is NOT scoped to
//     touch, and one every sibling rate-controller ticket would ALSO
//     need to touch concurrently in its own isolated clone, guaranteeing
//     a merge conflict across multiple of the 6 parallel tickets. A
//     single follow-up ticket, once the rate-controller phase-2 tickets
//     have landed, is the correct owner of AC_PID's own Info table -
//     ported exactly once, shared by every caller, instead of being
//     duplicated (and racing) across several.
//   - A GROUP `Info` wrapper nesting `pitch_param_info`'s table under a
//     vehicle-level table - explicitly deferred to a future integration
//     ticket per this ticket's own scope text (plane.hpp/mode.hpp
//     untouched here).
//   - `find_var_info` by-pointer-identity self-discovery - same
//     exclusion CPP-043 already established (no real caller without a
//     GCS/parameter-enumeration consumer this port doesn't have).
//   - CPP-023's conversion/upgrade machinery for older-format storage.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <fwcpp/fw_control/fw_controller.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/param/group_info.hpp> // param::Info
#include <fwcpp/param/native_value.hpp> // param::set_native_value/native_cast_to_float
#include <fwcpp/param/param.hpp> // param::VarType, param::ParamHeader, param::set_key
#include <fwcpp/param/persistence.hpp> // param::load_raw/save_raw/scan/should_skip_save/type_size
#include <fwcpp/param/storage.hpp> // storage::StorageAccess

namespace fwcpp::fw_control {

// RateLoopInputs plus the two additional attitude readings
// _get_coordination_rate_offset()/get_servo_out()'s roll-limit blend need
// - both are upstream AP::ahrs() reads (get_roll_rad(), get_pitch_rad(),
// and the pitch_sensor/roll_sensor integer-centidegree twins of the same
// two angles - this port works in float radians/degrees throughout
// rather than also carrying redundant centidegree integer copies, since
// float precision loss at the 0.01-degree centidegree quantum is not
// meaningful to any comparison performed against these two values).
struct PitchInputs : RateLoopInputs {
    float bank_angle_rad = 0.0f; // upstream: AP::ahrs().get_roll_rad(), read twice (coordination + roll-limit blend)
    float pitch_rad = 0.0f;      // upstream: AP::ahrs().get_pitch_rad() / .pitch_sensor
};

class PitchController {
public:
    struct Gains {
        float tau = 0.5f;      // PTCH2SRV_TCONST, seconds
        float rmax_pos = 0.0f; // PTCH2SRV_RMAX_UP, deg/s. 0 = disabled
        float rmax_neg = 0.0f; // PTCH2SRV_RMAX_DN, deg/s. 0 = disabled
        float roll_ff = 1.0f;  // PTCH2SRV_RLL, turn-coordination gain
        pid::AcPid::Gains rate_pid{
            .p = 0.04f, .i = 0.15f, .d = 0.0f, .ff = 0.345f, .imax = 0.666f, .filt_t_hz = 3.0f,
            .filt_e_hz = 0.0f,      .filt_d_hz = 12.0f, .srmax = 150.0f, .srtau = 1.0f,
        };
    };

    PitchController(const Gains& gains, const FwAparm& aparm) : base_(gains.rate_pid), gains_(gains), aparm_(aparm) {}

    PitchController(const PitchController&) = delete;
    PitchController& operator=(const PitchController&) = delete;

    // upstream: AP_PitchController::get_servo_out(angle_err, scaler,
    // disable_integrator, ground_mode).
    //
    // Function returns an equivalent elevator deflection in centi-degrees
    // in the range from -4500 to 4500. A positive demand is up.
    float get_servo_out(std::int32_t angle_err_cd, float scaler, bool disable_integrator, bool ground_mode,
                         const PitchInputs& in) {
        if (gains_.tau < 0.05f) {
            gains_.tau = 0.05f;
        }

        const float aspeed = in.airspeed;

        bool inverted = false;
        const float rate_offset = get_coordination_rate_offset(aspeed, in.eas2tas, in.bank_angle_rad, in.pitch_rad, inverted);

        // Calculate the desired pitch rate (deg/sec) from the angle error
        const float angle_err_deg = static_cast<float>(angle_err_cd) * 0.01f;
        float desired_rate = angle_err_deg / gains_.tau;

        // limit the maximum pitch rate demand. Don't apply when inverted
        // as the rates will be tuned when upright, and it is common that
        // much higher rates are needed inverted
        if (!inverted) {
            desired_rate += rate_offset;
            if (gains_.rmax_neg != 0.0f && desired_rate < -gains_.rmax_neg) {
                desired_rate = -gains_.rmax_neg;
            } else if (gains_.rmax_pos != 0.0f && desired_rate > gains_.rmax_pos) {
                desired_rate = gains_.rmax_pos;
            }
        } else {
            // Make sure not to invert the turn coordination offset
            desired_rate = -desired_rate + rate_offset;
        }

        // when we are past the configured roll limit for the aircraft our
        // priority should be to bring the aircraft back within the roll
        // limit - see file banner. Linearly reduce demanded pitch rate
        // when beyond the configured roll limit, reducing to zero at 90
        // degrees.
        float roll_deg = std::fabs(math::degrees(in.bank_angle_rad));
        if (roll_deg > 90.0f) {
            roll_deg = 180.0f - roll_deg;
        }
        const float roll_limit_margin_deg = std::min(aparm_.roll_limit_deg + 5.0f, 85.0f);
        if (roll_deg > roll_limit_margin_deg && std::fabs(math::degrees(in.pitch_rad)) < 70.0f) {
            const float roll_prop = (roll_deg - roll_limit_margin_deg) / (90.0f - roll_limit_margin_deg);
            desired_rate *= (1.0f - roll_prop);
        }

        const bool underspeed = aspeed <= 0.5f * aparm_.airspeed_min;
        return base_.get_rate_out_full(desired_rate, scaler, disable_integrator, underspeed, ground_mode, in);
    }

    // Forwarded from the composed FwController base - see
    // fw_controller.hpp's file banner.
    float get_rate_out(float desired_rate, float scaler, const RateLoopInputs& in) {
        const bool underspeed = in.airspeed <= 0.5f * aparm_.airspeed_min;
        return base_.get_rate_out(desired_rate, scaler, underspeed, in);
    }
    void reset_i() { base_.reset_i(); }
    void decay_i() { base_.decay_i(); }
    void set_ff_scale(float ff_scale) { base_.set_ff_scale(ff_scale); }
    [[nodiscard]] const pid::PidInfo& get_pid_info() const { return base_.get_pid_info(); }
    [[nodiscard]] pid::AcPid& rate_pid() { return base_.rate_pid(); }

private:
    // upstream: AP_PitchController::_get_coordination_rate_offset().
    //
    // Get the rate offset in degrees/second needed for pitch in body
    // frame to maintain height in a coordinated turn. Also returns the
    // inverted flag via the output parameter, matching upstream's
    // `bool &inverted` out-parameter shape.
    float get_coordination_rate_offset(float aspeed, float eas2tas, float bank_angle_in, float pitch_rad,
                                        bool& inverted) const {
        float bank_angle = bank_angle_in;

        // limit bank angle between +- 80 deg if right way up
        if (std::fabs(bank_angle) < math::radians(90.0f)) {
            bank_angle = math::constrain_value(bank_angle, -math::radians(80.0f), math::radians(80.0f));
            inverted = false;
        } else {
            inverted = true;
            if (bank_angle > 0.0f) {
                bank_angle = math::constrain_value(bank_angle, math::radians(100.0f), math::radians(180.0f));
            } else {
                bank_angle = math::constrain_value(bank_angle, -math::radians(180.0f), -math::radians(100.0f));
            }
        }

        float rate_offset;
        if (std::fabs(pitch_rad) > math::radians(70.0f)) {
            // don't do turn coordination handling when at very high pitch
            // angles (upstream: abs(_ahrs.pitch_sensor) > 7000)
            rate_offset = 0.0f;
        } else {
            rate_offset = std::cos(pitch_rad) *
                          std::fabs(math::degrees((kGravityMss / std::max(aspeed * eas2tas, std::max(aparm_.airspeed_min, 1.0f))) *
                                                   std::tan(bank_angle) * std::sin(bank_angle))) *
                          gains_.roll_ff;
        }
        if (inverted) {
            rate_offset = -rate_offset;
        }
        return rate_offset;
    }

    FwController base_;
    Gains gains_;
    FwAparm aparm_;
};

// This port's own top-level key allocation for the 4 real `Gains` fields
// below - see this file's "CPP-045 ADDENDUM" banner (above the class)
// for the full rationale.
enum class PitchParamKey : std::uint16_t {
    kTau = 1,
    kRmaxPos = 2,
    kRmaxNeg = 3,
    kRollFf = 4,
};

// Builds a fresh top-level param::Info[] table (4 real scalar entries +
// a VarType::None sentinel, matching every other table in this port's
// AP_Param module) addressing `gains`'s fields DIRECTLY (info.ptr =
// &gains.field). Built per-call, not a shared `static` table - same
// reasoning as CPP-043's `aparm_param_info`: this port allows more than
// one live `PitchController::Gains` (the round-trip test below
// constructs two), so there is no single fixed address to bake in at
// compile time.
//
// Names are the REAL full upstream names (GOBJECT prefix "PTCH" +
// var_info name, see this file's own "CPP-045 ADDENDUM" banner for the
// exact upstream lines each is transcribed from); every def_value is
// upstream's real var_info[] literal (2SRV_TCONST=0.5, 2SRV_RMAX_UP=0.0,
// 2SRV_RMAX_DN=0.0, 2SRV_RLL=1.0 - AP_PitchController.cpp, read
// directly).
[[nodiscard]] inline std::array<param::Info, 5> pitch_param_info(PitchController::Gains& gains) {
    using param::Info;
    using param::VarType;
    auto entry = [](const char* name, const void* ptr, float def_value, PitchParamKey key, VarType type) {
        Info info{};
        info.name = name;
        info.ptr = ptr;
        info.def_value = def_value;
        info.flags = 0;
        info.key = static_cast<std::uint16_t>(key);
        info.type = static_cast<std::uint8_t>(type);
        return info;
    };
    return {{
        entry("PTCH2SRV_TCONST", &gains.tau, 0.5f, PitchParamKey::kTau, VarType::Float),
        entry("PTCH2SRV_RMAX_UP", &gains.rmax_pos, 0.0f, PitchParamKey::kRmaxPos, VarType::Float),
        entry("PTCH2SRV_RMAX_DN", &gains.rmax_neg, 0.0f, PitchParamKey::kRmaxNeg, VarType::Float),
        entry("PTCH2SRV_RLL", &gains.roll_ff, 1.0f, PitchParamKey::kRollFf, VarType::Float),
        Info{}, // sentinel: type == VarType::None (0) via zero-init, matching every other table in this module
    }};
}

// Applies every entry's own AP_Param-table default directly into
// `gains`'s live fields - an explicit, opt-in function, not called from
// PitchController's constructor (matches CPP-043's own "explicit, not
// implicit" bias - PitchController::Gains' own C++ in-class default
// member initializers already give every existing test the same values,
// independently re-verified against AP_PitchController.cpp's real
// var_info[] literals for this ticket).
inline void apply_pitch_defaults(PitchController::Gains& gains) {
    const std::array<param::Info, 5> table = pitch_param_info(gains);
    for (const param::Info& info : table) {
        if (info.type == static_cast<std::uint8_t>(param::VarType::None)) {
            break;
        }
        param::set_native_value(static_cast<param::VarType>(info.type), const_cast<void*>(info.ptr), info.def_value);
    }
}

// Port of AP_Param::load() (AP_Param.cpp ~line 1310, read in full),
// specialized to a flat top-level table exactly as CPP-043's
// `load_aparm_parameters` is (each of these 4 fields is treated as its
// own top-level entry here - see this file's "CPP-045 ADDENDUM" banner
// for why no GROUP nesting is built in this ticket - so group_element is
// always 0), and to NOT use find_var_info's by-pointer-identity
// self-discovery (out of scope - the caller already knows which table it
// is loading). If a stored value is found, its bytes are read straight
// into the live field (load_raw, CPP-022 slice 5, unchanged); if not
// found, the table's own default is applied (set_native_value, CPP-043)
// - both behaviors matching real upstream load() exactly.
inline void load_pitch_parameters(const storage::StorageAccess& storage, PitchController::Gains& gains) {
    const std::array<param::Info, 5> table = pitch_param_info(gains);
    for (const param::Info& info : table) {
        if (info.type == static_cast<std::uint8_t>(param::VarType::None)) {
            break;
        }
        const auto type = static_cast<param::VarType>(info.type);
        param::ParamHeader phdr{};
        phdr.type = info.type;
        param::set_key(phdr, info.key);
        phdr.group_element = 0;
        void* field_ptr = const_cast<void*>(info.ptr);
        if (!param::load_raw(storage, phdr, field_ptr, param::type_size(type))) {
            param::set_native_value(type, field_ptr, info.def_value);
        }
    }
}

// Port of AP_Param::save_sync's default-skip-then-write path
// (AP_Param.cpp ~line 1138, read in full), specialized the same way
// load_pitch_parameters is above (flat top-level table, no find_var_info
// self-discovery). Reuses should_skip_save (CPP-022 slice 7,
// persistence.hpp) COMPLETELY UNCHANGED - pure float arithmetic, no
// pointer casting. `force_save` matches upstream's own
// save_sync(force_save, ...) parameter.
inline void save_pitch_parameters(storage::StorageAccess& storage, PitchController::Gains& gains, bool force_save = false) {
    const std::array<param::Info, 5> table = pitch_param_info(gains);
    for (const param::Info& info : table) {
        if (info.type == static_cast<std::uint8_t>(param::VarType::None)) {
            break;
        }
        const auto type = static_cast<param::VarType>(info.type);
        const void* field_ptr = info.ptr;
        const float current = param::native_cast_to_float(type, field_ptr);
        if (param::should_skip_save(type, current, info.def_value, force_save)) {
            continue;
        }
        param::ParamHeader phdr{};
        phdr.type = info.type;
        param::set_key(phdr, info.key);
        phdr.group_element = 0;
        (void)param::save_raw(storage, phdr, field_ptr, param::type_size(type));
    }
}

} // namespace fwcpp::fw_control
