#pragma once

// Port of APM_Control/AP_RollController.h + AP_RollController.cpp.
// CPP-032. Written by Jon Challinger, modified by Paul Riseborough.
// Upstream: libraries/APM_Control/AP_RollController.{h,cpp}
// (Plane-4.7.0, 33 + 268 lines) - read directly from the pinned upstream
// worktree in full before writing a line of this file, not from
// training-data memory.
//
// See fw_controller.hpp's file banner for the module-wide judgment calls
// this file relies on: composition over inheritance, RateLoopInputs
// replacing the is_underspeed()/get_airspeed()/get_measured_rate()
// virtual hooks, and the full AP_AutoTune exclusion.
//
// var_info[] DEFAULTS (AP_RollController::var_info, AP_RollController.cpp)
// transcribed into Gains below:
//   2SRV_TCONST  -> tau       (0.5f)
//   2SRV_RMAX    -> rmax_pos  (0, meaning "disabled" - upstream's own
//                        `if (gains.rmax_pos && ...)` treats 0 as "no
//                        limit", reproduced verbatim below)
//   _RATE_P/_I/_D/_FF/_IMAX/_FLTT/_FLTE/_FLTD/_SMAX (AP_SUBGROUPINFO into
//   the embedded AC_PID, index 9) -> Gains::rate_pid, defaults taken from
//   AP_RollController's constructor's AC_PID::Defaults initializer, NOT
//   var_info (var_info only documents ranges/units for these - the real
//   defaults are the constructor's aggregate-init literals):
//     p=0.08, i=0.15, d=0.0, ff=0.345, imax=0.666, filt_T_hz=3.0,
//     filt_E_hz=0.0, filt_D_hz=12.0, srmax=150.0, srtau=1.0
//   _RATE_NTF/_RATE_NEF (notch filter indices) and _RATE_PDMX/_RATE_D_FF
//   are AC_PID sub-fields with no non-zero/non-default value set by
//   AP_RollController's constructor - left at AcPid::Gains's own defaults
//   (matches upstream: an AC_PID::Defaults aggregate-init that doesn't
//   mention a field leaves that field at AC_PID's own class-default).
//
// NOTE (CPP-044, see this file's own ADDENDUM below the class for the
// full AP_Param treatment): the field names above are the literal
// `var_info` entry names, NOT the full user-visible parameter names.
// Upstream's real GOBJECT registration (ArduPlane/Parameters.cpp) is
// `GOBJECT(rollController, "RLL", AP_RollController)` - prefix "RLL",
// NOT "RLL2SRV_"/"ROLL2SRV_" as this banner's own pre-CPP-044 text (and
// this ticket's own scope text) assumed without checking. The real
// full names are therefore "RLL" + entry name, e.g. "RLL2SRV_TCONST",
// "RLL2SRV_RMAX", "RLL_RATE_P" (the "_RATE_" comes from the
// AP_SUBGROUPINFO(rate_pid, "_RATE_", 9, ...) prefix, concatenated the
// same way).
//
// in_recovery / set_in_recovery() NOT PORTED: upstream's own comment
// states its exact purpose - "set the in_recovery flag, which is used
// during a VTOL upset recovery" - and its only effect is to skip the
// rmax_pos rate-limit clamp for one loop. No VTOL/quadplane vehicle
// exists in this port (task-mandated exclusion), so in_recovery's value
// is permanently false, its only reachable state here - the rmax_pos
// clamp below is applied unconditionally, exactly matching that
// permanent-false behavior with no flag needed to express it.
//
// convert_pid() NOT PORTED: EEPROM/AP_Param old-to-new-gain migration
// helper, meaningless without AP_Param backing (see fw_controller.hpp's
// banner).
//
// LITERAL SAFETY: no bare ambiguous double literals - 0.01f, 0.05f, 160.0f,
// 180.0f, 2.0f all explicitly float-suffixed, matching upstream's own
// (upstream writes some of these, e.g. `angle_err * 0.01`, as bare double
// literals against a float LHS - this port makes every one explicitly
// float per the port's own literal-safety convention, with no change in
// value).

#include <array>
#include <cmath>
#include <cstdint>

#include <fwcpp/fw_control/fw_controller.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/param/group_info.hpp> // param::Info (CPP-044)
#include <fwcpp/param/native_value.hpp> // param::set_native_value/native_cast_to_float (CPP-044)
#include <fwcpp/param/param.hpp> // param::VarType/ParamHeader/set_key (CPP-044)
#include <fwcpp/param/persistence.hpp> // param::load_raw/save_raw/scan/should_skip_save/type_size (CPP-044)
#include <fwcpp/param/storage.hpp> // param::storage::StorageAccess (CPP-044)

namespace fwcpp::fw_control {

class RollController {
public:
    struct Gains {
        float tau = 0.5f;      // real name "RLL2SRV_TCONST", seconds
        float rmax_pos = 0.0f; // real name "RLL2SRV_RMAX", deg/s. 0 = disabled
        pid::AcPid::Gains rate_pid{
            .p = 0.08f, .i = 0.15f, .d = 0.0f, .ff = 0.345f, .imax = 0.666f, .filt_t_hz = 3.0f,
            .filt_e_hz = 0.0f,      .filt_d_hz = 12.0f, .srmax = 150.0f, .srtau = 1.0f,
        };
    };

    RollController(const Gains& gains, const FwAparm& aparm) : base_(gains.rate_pid), gains_(gains), aparm_(aparm) {}

    RollController(const RollController&) = delete;
    RollController& operator=(const RollController&) = delete;

    // upstream: AP_RollController::get_servo_out(angle_err, scaler,
    // disable_integrator, ground_mode).
    //
    // Function returns an equivalent aileron deflection in centi-degrees
    // in the range from -4500 to 4500. A positive demand is up.
    // Inputs are:
    // 1) demanded bank angle in centi-degrees
    // 2) control gain scaler = scaling_speed / aspeed
    // 3) boolean which is true when the integrator should be disabled
    //    (e.g. stabilise mode)
    // 4) boolean which is true when the aircraft is on the ground
    float get_servo_out(std::int32_t angle_err_cd, float scaler, bool disable_integrator, bool ground_mode,
                         const RateLoopInputs& in) {
        if (gains_.tau < 0.05f) {
            gains_.tau = 0.05f;
        }

        // Calculate the desired roll rate (deg/sec) from the angle error
        const float angle_err_deg = static_cast<float>(angle_err_cd) * 0.01f;
        float desired_rate = angle_err_deg / gains_.tau;

        // prevent indecision in the roll controller when target roll is
        // close to 180 degrees from the current roll
        constexpr float indecision_threshold_deg = 160.0f;
        const float last_desired_rate = base_.get_pid_info().target;
        const float abs_angle_err_deg = std::fabs(angle_err_deg);
        if (abs_angle_err_deg > indecision_threshold_deg && angle_err_deg <= 180.0f) {
            if (desired_rate * last_desired_rate < 0.0f) {
                desired_rate = -desired_rate;
                // increase the desired rate in proportion to the extra
                // angle we are requesting
                const float new_angle_err_deg = abs_angle_err_deg + (180.0f - abs_angle_err_deg) * 2.0f;
                desired_rate *= new_angle_err_deg / abs_angle_err_deg;
            }
        }

        // Limit the demanded roll rate. See file banner: in_recovery's
        // "skip this limit for one loop" is not ported (no VTOL/quadplane
        // in this port), so the limit is applied unconditionally.
        if (gains_.rmax_pos != 0.0f && desired_rate < -gains_.rmax_pos) {
            desired_rate = -gains_.rmax_pos;
        } else if (gains_.rmax_pos != 0.0f && desired_rate > gains_.rmax_pos) {
            desired_rate = gains_.rmax_pos;
        }

        const bool underspeed = in.airspeed <= aparm_.airspeed_min;
        return base_.get_rate_out_full(desired_rate, scaler, disable_integrator, underspeed, ground_mode, in);
    }

    // Forwarded from the composed FwController base - see
    // fw_controller.hpp's file banner for why these exist as forwards
    // rather than through inheritance.
    float get_rate_out(float desired_rate, float scaler, const RateLoopInputs& in) {
        const bool underspeed = in.airspeed <= aparm_.airspeed_min;
        return base_.get_rate_out(desired_rate, scaler, underspeed, in);
    }
    void reset_i() { base_.reset_i(); }
    void decay_i() { base_.decay_i(); }
    void set_ff_scale(float ff_scale) { base_.set_ff_scale(ff_scale); }
    [[nodiscard]] const pid::PidInfo& get_pid_info() const { return base_.get_pid_info(); }
    [[nodiscard]] pid::AcPid& rate_pid() { return base_.rate_pid(); }

private:
    FwController base_;
    Gains gains_;
    FwAparm aparm_;
};

// === CPP-044 ADDENDUM: real top-level AP_Param Info[] table for
// RollController::Gains, load/save round-trip ===
//
// Phase 2a of the AP_Param vehicle-integration effort CPP-043 started
// (phase 1: Plane::aparm, fwcpp/vehicle/plane.hpp's own "CPP-043
// ADDENDUM" - read that first, this follows its pattern exactly).
// Entirely self-contained to this file, per this ticket's own
// parallel-work constraint (CPP-044 through CPP-049 each own one
// controller's module and must not touch plane.hpp/mode.hpp or any
// sibling module).
//
// FINDING #1 - the ticket's own prefix claim ("RLL2SRV_") is WRONG,
// verified by reading ArduPlane/Parameters.cpp directly (grepped for
// "rollController"/"GOBJECT"): the real registration is
// `GOBJECT(rollController, "RLL", AP_RollController)` - prefix is just
// "RLL". The "2SRV_"/"_RATE_" text that makes names like "RLL2SRV_TCONST"
// and "RLL_RATE_P" look like they contain a "2SRV_"/"RLL2SRV" prefix
// actually comes from the var_info ENTRY names themselves ("2SRV_TCONST",
// "2SRV_RMAX") and the embedded AC_PID subgroup's own prefix
// ("_RATE_", AP_SUBGROUPINFO(rate_pid, "_RATE_", 9, AP_RollController,
// AC_PID), AP_RollController.cpp:142) - concatenated with the real "RLL"
// group prefix, not a "RLL2SRV_" prefix on its own. This file's own
// pre-existing banner (above) repeated the same wrong assumption
// ("ROLL2SRV_TCONST"/"ROLL2SRV_RMAX") before this ticket - corrected
// above too.
//
// FINDING #2 - of this port's RollController::Gains fields, only 12 are
// genuinely backed by a real, reachable upstream AP_Param, verified by
// reading libraries/APM_Control/AP_RollController.cpp's real var_info[]
// AND libraries/AC_PID/AC_PID.cpp's real var_info[] (the table
// AP_SUBGROUPINFO(rate_pid, "_RATE_", 9, ...) actually dispatches into)
// in full - NOT assumed to be full 1:1 coverage, matching CPP-043's own
// finding that only a fraction of its target struct turned out to be
// real AP_Param-backed:
//   tau          -> "RLL2SRV_TCONST" (AP_RollController.cpp:36, key 0)
//   rmax_pos     -> "RLL2SRV_RMAX"   (AP_RollController.cpp:47, key 4)
//   rate_pid.p         -> "RLL_RATE_P"    (AC_PID.cpp "P",    key 0)
//   rate_pid.i         -> "RLL_RATE_I"    (AC_PID.cpp "I",    key 1)
//   rate_pid.d         -> "RLL_RATE_D"    (AC_PID.cpp "D",    key 2)
//   rate_pid.ff        -> "RLL_RATE_FF"   (AC_PID.cpp "FF",   key 4)
//   rate_pid.imax      -> "RLL_RATE_IMAX" (AC_PID.cpp "IMAX", key 5)
//   rate_pid.filt_t_hz -> "RLL_RATE_FLTT" (AC_PID.cpp "FLTT", key 9)
//   rate_pid.filt_e_hz -> "RLL_RATE_FLTE" (AC_PID.cpp "FLTE", key 10)
//   rate_pid.filt_d_hz -> "RLL_RATE_FLTD" (AC_PID.cpp "FLTD", key 11)
//   rate_pid.srmax     -> "RLL_RATE_SMAX" (AC_PID.cpp "SMAX", key 12)
//   rate_pid.dff       -> "RLL_RATE_D_FF" (AC_PID.cpp "D_FF", key 14)
//
// EXCLUDED FIELDS, each named explicitly with why (not silently
// skipped), per this ticket's own acceptance criteria:
//   - rate_pid.srtau (AcPid::Gains::srtau): AC_PID.h:182 declares the
//     backing member `_slew_rate_tau` as AP_Float, but its own comment
//     reads verbatim "Not exposed in this class by default, but defined
//     as an AP_Float so parent classes can make it configurable via
//     param table" - and AC_PID::var_info[] (AC_PID.cpp, read in full)
//     has NO entry for it at all (no "SRTAU" AP_GROUPINFO line; key 3 is
//     commented "was for uint16 IMAX", nothing claims srtau's slot).
//     AP_RollController's AP_SUBGROUPINFO uses AC_PID's var_info
//     UNCHANGED (no subclass override adding an "RLL_RATE_SRTAU" entry
//     the way AC_PID.h's own comment says a "parent class" COULD) - so
//     for RollController specifically, srtau is genuinely NOT
//     AP_Param-backed. It is set only via the constructor's
//     AC_PID::Defaults{..., .srtau = 1.0} aggregate-init literal,
//     reproduced as AcPid::Gains's own srtau default (1.0f, already
//     present in this port before this ticket) with no Info entry here.
//   - AC_PID's "PDMX"/_kpdmax (key 13), "NTF"/_notch_T_filter (key 15),
//     "NEF"/_notch_E_filter (key 16): real upstream AP_Params, reachable
//     under "RLL_RATE_PDMX"/"RLL_RATE_NTF"/"RLL_RATE_NEF" - but this
//     port's `pid::AcPid::Gains` (modules/ap-pid, a DIFFERENT module,
//     out of this ticket's scope to modify) has no corresponding field
//     at all to point an Info entry at. Wiring these up would require
//     adding fields to ap-pid's own Gains struct, which belongs to a
//     sibling module this ticket must not touch (see this file's own
//     parallel-work constraint above).
//   - Upstream's `AP_AutoTune::ATGains::rmax_neg` (AP_AutoTune.h:16,
//     AP_Int16): a real struct field sitting right next to rmax_pos in
//     the same ATGains aggregate AP_RollController::gains is typed as -
//     but AP_RollController.cpp's var_info[] (read in full) has NO
//     AP_GROUPINFO entry for it at all (only rmax_pos, key 4, is
//     registered - rmax_neg is set only programmatically, e.g. by
//     AP_AutoTune, never exposed to the user). This port's own Gains
//     struct never had an rmax_neg field to begin with (established
//     before this ticket, and AP_AutoTune itself is a full, separately-
//     registered exclusion - see this file's own top banner) - excluded
//     because there is no real var_info entry to port regardless of
//     this port's own field coverage.
//   - ATGains's AutoTune-only scratch fields (FF, P, I, D, IMAX, flt_T,
//     flt_E, flt_D): plain floats with no AP_GROUPINFO entries anywhere,
//     used only internally by AP_AutoTune - out of scope (AP_AutoTune
//     fully excluded, this file's own top banner).
//
// FINDING #3, A REGISTERED DIVERGENCE (ADR-0007): upstream's real
// `AP_AutoTune::ATGains::rmax_pos` (AP_AutoTune.h:15) is `AP_Int16` (2
// bytes), but this port's `RollController::Gains::rmax_pos` (established
// at CPP-032, long before this ticket) is a plain C++ `float` (4 bytes) -
// the same shape of divergence CPP-043 registered for seven of
// FixedWingTunables' fields. Retrofitting rmax_pos's C++ type was
// rejected here for the identical reason: it is read as a plain float
// throughout get_servo_out() above, and changing its width would ripple
// through arithmetic this ticket has no behavioral reason to touch.
// `roll_param_info` below uses VarType::Float (this port's own live
// width) for rmax_pos - meaning this port's on-storage encoding for
// "RLL2SRV_RMAX" does not match a byte blob a real upstream vehicle
// would produce for the same name. Every other real field above (tau,
// and all ten rate_pid fields) is genuinely AP_Float upstream too (AC_PID.h
// declares _kp/_ki/_kd/_kff/_kimax/_filt_T_hz/_filt_E_hz/_filt_D_hz/
// _slew_rate_max/_kdff all as AP_Float, read directly) - no divergence
// for those.
//
// FINDING #4 - why native_value.hpp (CPP-043), not CPP-022 slice 6/7's
// ParamValue<T>-based set_value/cast_to_float, is reused here: every
// field above (Gains::tau/rmax_pos, AcPid::Gains's members) is a plain
// C++ `float` (ap-pid's own file banner explains why: "AP_Float REPLACED
// WITH PLAIN float... AP_Param does not exist in this port yet" - true
// when CPP-016 was written, still true of the field's C++ type today),
// not this port's `param::ParamFloat` wrapper class. Reinterpreting a
// `float` object as a `ParamFloat` to call its member functions would be
// exactly the unsafe reinterpretation ADR-0012 forbids, even though the
// two share layout on every compiler this port targets - the same
// reasoning CPP-043 already applied to Plane::aparm. `set_native_value`/
// `native_cast_to_float`'s memcpy-based read/write is the honest
// alternative, reused here COMPLETELY UNCHANGED, along with
// find_group/get_base (group_info.hpp) and load_raw/save_raw/scan/
// should_skip_save/type_size (persistence.hpp) - none of those are
// type-specific, exactly as CPP-043 found.
//
// SHAPE DECISION - flat top-level table, not a nested GROUP `Info`
// entry: upstream's real registration genuinely IS a two-level nested
// GROUP (Plane's own var_info has a GROUP entry for "RLL" ->
// AP_RollController::var_info, which itself has a nested GROUP entry
// for "_RATE_" -> AC_PID::var_info) - unlike CPP-043's aparm, which
// turned out to have no GROUP wrapper at all. Building the full nested
// GroupInfo scaffolding for that here would require either (a) a
// GroupInfo table for AC_PID's own var_info, which belongs to a
// DIFFERENT module (ap-pid) this ticket must not touch, or (b)
// duplicating AC_PID's shape locally, which would misrepresent AC_PID as
// something RollController-specific. This file's `roll_param_info`
// instead reproduces the REAL, fully-resolved flat NAMES ("RLL2SRV_TCONST",
// "RLL_RATE_P", etc. - i.e. what `top_level::find("RLL_RATE_P", ...)`
// would ultimately resolve to after walking both real nesting levels) as
// independent top-level `param::Info` scalar entries pointing directly
// at `Gains`'s own fields, matching this ticket's own requested
// `roll_param_info(RollController::Gains&) -> std::array<param::Info, N+1>`
// shape exactly. `param::find` (top_level.hpp, CPP-043) still works
// unmodified against this flat table (its own SCALAR branch, case-
// insensitive strcasecmp) - exercised by this file's own test.
//
// KEY ALLOCATION: RollParamKey below is a LOCAL, per-module allocation
// (matching CPP-043's own AparmParamKey precedent) starting at 1 - it
// has no relationship to upstream's real k_param_* values (an
// EEPROM-migration/ordering detail this port has no reason to reproduce,
// same as CPP-043) and is NOT coordinated with any sibling ticket's own
// key allocation (CPP-045 through CPP-049 each independently number
// their own module's fields starting at 1 too, per this ticket's own
// parallel-work isolation) - a future integration ticket wiring multiple
// controllers' Info tables into one shared vehicle-level table will need
// to assign each module a distinct, non-overlapping key range before any
// of them can safely share one real StorageAccess region; this ticket's
// own round-trip test (below) only ever exercises RollController::Gains
// in isolation against a fresh backing store, so the collision risk is
// real but not yet exercised.
//
// EXPLICIT, NOT IMPLICIT (matching CPP-043's own bias): apply_roll_
// defaults/load_roll_parameters/save_roll_parameters are ordinary free
// functions a caller invokes explicitly - nothing calls them
// automatically. RollController::Gains keeps getting its defaults from
// its own C++ in-class default member initializers exactly as before
// this ticket (independently re-verified against real upstream values
// above) - apply_roll_defaults exists as a separate, explicit path to
// the SAME values sourced from the AP_Param table itself, for a caller
// (e.g. this ticket's own test) that wants that guarantee independent of
// whether the two have been kept in sync by hand.
//
// DEFERRED, NAMED EXPLICITLY: Plane-level wiring/convenience methods (a
// separate future integration ticket, once CPP-044 through CPP-049 all
// land - explicitly out of this ticket's own scope); find_var_info
// by-pointer-identity self-discovery (no real caller, same exclusion
// CPP-043 already established); CPP-023's conversion/upgrade machinery;
// the nested-GROUP `Info`/`GroupInfo` scaffolding this "SHAPE DECISION"
// note above explains was not built (a future phase that also ports
// ap-pid's own AC_PID GroupInfo table for real could revisit this).

// This port's own top-level key allocation for the 12 real
// RollController::Gains fields below - see this addendum's "KEY
// ALLOCATION" note above for why these values are local to this module,
// not shared/coordinated vehicle-wide key space.
enum class RollParamKey : std::uint16_t {
    kTau = 1,
    kRmaxPos = 2,
    kRateP = 3,
    kRateI = 4,
    kRateD = 5,
    kRateFf = 6,
    kRateImax = 7,
    kRateFltT = 8,
    kRateFltE = 9,
    kRateFltD = 10,
    kRateSmax = 11,
    kRateDff = 12,
};

// Builds a fresh top-level param::Info[] table (12 real scalar entries +
// a VarType::None sentinel, matching every other table in this port's
// AP_Param module) addressing `gains`'s fields DIRECTLY. Built per-call,
// not a shared `static` table - see plane.hpp's own aparm_param_info
// banner for why (more than one live Gains object can exist; there is no
// single fixed address to bake in at compile time).
//
// Names/keys/order are transcribed directly from the real upstream
// var_info tables (AP_RollController.cpp + AC_PID.cpp, both read in
// full - see FINDING #2 above); every def_value is the real upstream
// constructor-supplied default (this addendum's FINDING #2/AP_RollController's
// own AC_PID::Defaults aggregate-init, re-verified for this ticket).
[[nodiscard]] inline std::array<param::Info, 13> roll_param_info(RollController::Gains& gains) {
    using param::Info;
    using param::VarType;
    auto entry = [](const char* name, const void* ptr, float def_value, RollParamKey key, VarType type) {
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
        entry("RLL2SRV_TCONST", &gains.tau, 0.5f, RollParamKey::kTau, VarType::Float),
        entry("RLL2SRV_RMAX", &gains.rmax_pos, 0.0f, RollParamKey::kRmaxPos, VarType::Float),
        entry("RLL_RATE_P", &gains.rate_pid.p, 0.08f, RollParamKey::kRateP, VarType::Float),
        entry("RLL_RATE_I", &gains.rate_pid.i, 0.15f, RollParamKey::kRateI, VarType::Float),
        entry("RLL_RATE_D", &gains.rate_pid.d, 0.0f, RollParamKey::kRateD, VarType::Float),
        entry("RLL_RATE_FF", &gains.rate_pid.ff, 0.345f, RollParamKey::kRateFf, VarType::Float),
        entry("RLL_RATE_IMAX", &gains.rate_pid.imax, 0.666f, RollParamKey::kRateImax, VarType::Float),
        entry("RLL_RATE_FLTT", &gains.rate_pid.filt_t_hz, 3.0f, RollParamKey::kRateFltT, VarType::Float),
        entry("RLL_RATE_FLTE", &gains.rate_pid.filt_e_hz, 0.0f, RollParamKey::kRateFltE, VarType::Float),
        entry("RLL_RATE_FLTD", &gains.rate_pid.filt_d_hz, 12.0f, RollParamKey::kRateFltD, VarType::Float),
        entry("RLL_RATE_SMAX", &gains.rate_pid.srmax, 150.0f, RollParamKey::kRateSmax, VarType::Float),
        entry("RLL_RATE_D_FF", &gains.rate_pid.dff, 0.0f, RollParamKey::kRateDff, VarType::Float),
        Info{}, // sentinel: type == VarType::None (0) via zero-init, matching every other table in this module
    }};
}

// Applies every entry's own AP_Param-table default directly into
// `gains`'s live fields - see this addendum's "EXPLICIT, NOT IMPLICIT"
// note above for why this is a separate, opt-in function.
inline void apply_roll_defaults(RollController::Gains& gains) {
    const std::array<param::Info, 13> table = roll_param_info(gains);
    for (const param::Info& info : table) {
        if (info.type == static_cast<std::uint8_t>(param::VarType::None)) {
            break;
        }
        param::set_native_value(static_cast<param::VarType>(info.type), const_cast<void*>(info.ptr), info.def_value);
    }
}

// Port of AP_Param::load() (AP_Param.cpp ~line 1310, read in full),
// specialized to a flat top-level table (this addendum's own "SHAPE
// DECISION" note above: every real field is addressed by its fully-
// resolved name as an independent top-level entry, so group_element is
// always 0) and to NOT use find_var_info's by-pointer-identity
// self-discovery (out of scope - the caller already knows which object/
// table it's loading) - matches CPP-043's load_aparm_parameters exactly.
inline void load_roll_parameters(const storage::StorageAccess& storage, RollController::Gains& gains) {
    const std::array<param::Info, 13> table = roll_param_info(gains);
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
// load_roll_parameters is above. Reuses should_skip_save (CPP-022 slice
// 7, persistence.hpp) COMPLETELY UNCHANGED - pure float arithmetic, no
// pointer casting - matches CPP-043's save_aparm_parameters exactly.
// `force_save` matches upstream's own save_sync(force_save, ...)
// parameter, wired through for a future caller even though this
// ticket's own test relies on the default-skip path.
inline void save_roll_parameters(storage::StorageAccess& storage, RollController::Gains& gains, bool force_save = false) {
    const std::array<param::Info, 13> table = roll_param_info(gains);
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
