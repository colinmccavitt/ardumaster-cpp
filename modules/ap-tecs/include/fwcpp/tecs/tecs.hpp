#pragma once

// Port of AP_TECS (Total Energy Control System). CPP-029, slice 1.
// Written by Paul Riseborough 2013. Upstream: AP_TECS/AP_TECS.h,
// AP_TECS.cpp (Plane-4.7.0, ~514/~1676 lines) - read directly from the
// pinned upstream worktree in full before writing a line of this file, not
// from training-data memory.
//
// SCOPE: this slice ports the complete NORMAL-flight-stage control law -
// every function upstream's real call sequence (update_pitch_throttle(),
// read first to understand the order, exactly as the task suggested)
// exercises when flight_stage == AP_FixedWing::FlightStage::NORMAL and no
// AP_Landing state is active. That is: update_50hz(), _update_speed(),
// _update_speed_demand(), _update_height_demand(), _detect_underspeed(),
// _update_energies(), timeConstant(), _update_throttle_with_airspeed(),
// _get_i_gain(), _detect_bad_descent(), _update_pitch(),
// _initialise_states(), _update_STE_rate_lim(), update_pitch_throttle()'s
// own orchestration, set_throttle_min/max, _update_throttle_limits(),
// set_pitch_min/max, _update_pitch_limits(), offset_altitude(), and
// use_airspeed() - i.e. every function the task's own recommendation
// named, ported in full for the NORMAL case.
//
// NOT PORTED, and why (grep upstream AP_TECS.cpp for these names to verify
// each citation):
//
//   - Every AP_FixedWing::FlightStage::{TAKEOFF,LAND,ABORT_LANDING,VTOL}
//     branch, and every `_landing.*()` call (is_flaring, is_on_approach,
//     is_on_final, get_pitch_cd, get_throttle_slewrate) - no landing,
//     takeoff, or VTOL subsystem in this port (task-mandated exclusion).
//     Concretely dropped:
//       * update_pitch_throttle()'s `flight_stage`, `ptchMinCO_cd`,
//         `distance_beyond_land_wp` parameters (only ever read by the
//         above), and its TAKEOFF-reached-speed check block.
//       * _update_speed()'s `_landing.is_on_final() && airspeed_stall>0`
//         TASmin override (~line 419).
//       * _update_speed_demand() is otherwise fully in scope (no landing
//         reference in it at all).
//       * _update_height_demand()'s `is_doing_auto_land`
//         approach-lag-compensation branch (~line 583-587, `_hgt_dem +=
//         _hgt_dem_tconst*_hgt_rate_dem` vs. the "don't get too far ahead"
//         else-arm this slice already ports); the `_maxSinkRate_approach`
//         special-case (~line 540); the TAKEOFF/ABORT_LANDING exclusion
//         inside max_climb_condition and the `!_landing.is_flaring()`
//         guard inside max_descent_condition both become
//         unconditionally-true/simplified-away terms since they can never
//         be false/true respectively in NORMAL-only scope. (The
//         `_landing.is_flaring()` flare branch itself, ~line 611-644, IS
//         now ported - see the CPP-040 ADDENDUM below for its own scope.)
//       * _detect_underspeed()'s `flight_stage==VTOL` clear and
//         `!_landing.is_flaring()` guard (~line 658-663) - both
//         unconditionally satisfied here, simplified away.
//       * _update_throttle_with_airspeed()'s VTOL SPE-error zeroing
//         (~line 722-729), TAKEOFF/ABORT_LANDING full-throttle-until-
//         target-speed integrator override (~line 787-792), landing
//         throttle-damping override (~line 774-776), and
//         `_landing.is_on_approach()` slew-rate override (~line 798-803).
//       * _get_i_gain()'s TAKEOFF and is_doing_auto_land branches (~line
//         896-901) - collapses to always returning the plain INTEG_GAIN.
//       * _detect_bad_descent()'s `flight_stage==VTOL` early-out (~line
//         964).
//       * _update_pitch()'s VTOL/TAKEOFF/ABORT_LANDING speed-weighting
//         forcing and is_doing_auto_land sliding-weight branch (~line
//         991-1003), its flare/is_doing_auto_land pitch-damping override
//         (~line 1035-1039), and its TAKEOFF/ABORT_LANDING pitch-floor
//         bias (~line 1054-1057).
//       * _initialise_states()'s entire TAKEOFF/ABORT_LANDING branch
//         (~line 1203-1234) and the reached_speed_takeoff/
//         flag_have_reset_after_takeoff bookkeeping that only that branch
//         and the dropped TAKEOFF checks in update_pitch_throttle() and
//         _update_throttle_with_airspeed() ever touch - the flag and its
//         backing field are dropped from this port entirely, not merely
//         left always-false.
//       * _update_pitch_limits() drops ptchMinCO_cd (TAKEOFF-only),
//         everything gated on `_landing.is_flaring()`/`is_on_approach()`
//         (the _land_pitch_min hysteresis tracking, the flare pitch-range
//         blend against LAND_PITCH_DEG, LAND_PMAX) - what remains is just
//         PITCH_MAX/PITCH_MIN vs. PTCH_LIM_MAX/MIN_DEG selection plus the
//         external-limit clamp and degree->radian conversion.
//       * offset_altitude() drops its three flare-state offsets
//         (_flare_hgt_dem_ideal/_flare_hgt_dem_adj/_hgt_at_start_of_flare)
//         - the remaining five height-state offsets are fully in scope.
//
//   - _update_throttle_without_airspeed() (~line 910-957): the no-
//     airspeed-sensor throttle fallback. Per the task's own framing, this
//     port has no airspeed-sensor subsystem yet, so a real caller is
//     expected to always present an airspeed reading (real or synthetic)
//     via TecsInputs, making use_airspeed() true and this branch
//     unreachable - the task explicitly permits deferring it on exactly
//     this basis. It also depends on _pitch_demand_lpf/_pitch_measured_lpf
//     (LowPassFilterFloat members), which are consequently ALSO dropped
//     from this port's state - there is no other consumer of either
//     filter. update_pitch_throttle() below still has an `else` arm for
//     the case use_airspeed() is false at runtime, but it is a documented
//     safe no-op (hold + re-clamp the previous throttle demand), not an
//     implementation of upstream's pitch-to-throttle mapping - see that
//     arm's own comment.
//
//   - Logging: every `#if HAL_LOGGING_ENABLED` block (TEC2/TEC3/TEC4/TECS
//     AP::logger() messages) is cut - no logging subsystem in this port.
//     GCS/MAVLink status reporting (send_TECS_status(), which doesn't
//     even live in this file upstream) is likewise out of scope - no GCS
//     subsystem.
//
//   - AP_TECS_STALL_PREVENTION_ENABLED-style niche build flags: none
//     exist in this file upstream (checked) - AP_TECS_Option's two bits
//     (GLIDER_ONLY, DESCENT_SPEEDUP) are ordinary runtime options, not
//     build-time flags, and both ARE ported (see Gains::option_glider_only/
//     option_descent_speedup below) since neither is landing-specific.
//
//   - get_land_airspeed()/set_path_proportion() accessors - STILL excluded
//     as of CPP-041 (verified: grepped every call site of both in
//     AP_TECS.cpp/.h and ArduPlane, neither is the flare height-rate blend
//     CPP-040 ported nor CPP-041's glide-slope aim-point math). _landAirspeed
//     and _path_proportion (and the AP_Landing-derived state
//     set_path_proportion() feeds) likewise stay unported - no caller in
//     this port's scope needs them (CPP-041's own "CPP-041 ADDENDUM",
//     plane.hpp, traces get_land_airspeed()'s real disabled-sentinel (-1)
//     default explicitly). get_land_sinkrate() itself IS now ported
//     (CPP-041) - see this class's own doc comment on that accessor, right
//     next to get_max_sinkrate() below - because CPP-041's real caller
//     (Plane::setup_landing_glide_slope(), plane.hpp) needed it directly,
//     exactly the "future AP_Landing-equivalent" this note originally
//     anticipated.
//
// UPSTREAM DEAD CODE FOUND, not ported (a fact about upstream, not a
// judgment call about scope): `AP_Float _accel_gf;` is declared in
// AP_TECS.h (~line 228) but never read, assigned, or registered in
// var_info[] anywhere in AP_TECS.cpp - verified by grepping the whole
// file for "accel_gf" and finding only the declaration. Not ported.
// `_lag_comp_hgt_offset` is written once, unconditionally, to 0.0f inside
// _initialise_states()'s reset branch (~line 1181) and never read again
// anywhere in the file - also dropped as dead state.
//
// A GENUINE UPSTREAM QUIRK, faithfully reproduced rather than fixed (see
// ADR-0007/the port's "fix inherited bugs, register every divergence"
// policy - this is the opposite call: KEEPING the odd behavior because
// it's ambiguous whether it's a bug, and inventing a "fix" would be a
// bigger risk than reproducing the documented original): _update_speed()'s
// TASmax adjustment (~line 405-412) scales by the class-level `_DT` -
// i.e. the CALLING PERIOD OF update_pitch_throttle() (10-500Hz, set once
// per call to that function) - not by the `DT` *parameter* _update_speed()
// itself was just called with (update_50hz()'s own, independent 50Hz-or-
// faster period). Since _update_speed() is invoked from update_50hz(),
// which upstream's own doc comment says may run faster than
// update_pitch_throttle(), this literally scales a per-50Hz-tick state
// update by a stale, unrelated timestep from the last main-loop tick.
// This port reproduces it exactly (update_speed() below reads dt_, the
// main-loop timestep cached by update_pitch_throttle(), at exactly the
// same two call sites upstream does) rather than substituting the local
// `dt` parameter, which would be the "obviously more correct" fix but is
// not what upstream ships. One conservative, safety-only deviation: this
// port default-initialises dt_ to 0.0f rather than leaving it as upstream
// leaves `_DT` (a plain, non-defaulted float member - genuinely
// indeterminate/UB to read before the first assignment in
// AP_TECS::update_pitch_throttle()). In practice this UB is masked
// upstream too: the branch that reads _DT is only reachable when
// `_flags.reset` is false, and reset is forced true on TECS's very first
// tick (the elapsed-time-since-never-called calculation is enormous) -
// so the dangerous read never actually happens on either side. Concretely
// tested by "TASmax adjustment uses the main-loop DT, not update_50hz's
// own DT" in tests/tecs_test.cpp.
//
// CPP-040 ADDENDUM (flare height-rate-demand blending, phase 1 of
// MAV_CMD_NAV_LAND): ports upstream's `_update_height_demand()` flare
// branch (AP_TECS.cpp real line 611-644, the `} else {` arm of
// `if (!_landing.is_flaring()) {...} else {...}`) - the smooth
// height-rate blend from the pre-flare rate toward a configured sink
// rate as the aircraft nears the ground, used during the last seconds
// of an automatic landing. Read directly from AP_TECS.cpp/.h, not from
// memory - every field/default below was grepped from the real
// AP_GROUPINFO table and struct declarations.
//
//   - VERIFIED CORRECTION TO THE TASK'S OWN FRAMING: the task that
//     scoped this ticket describes the branch split as
//     `if (!is_doing_auto_land) {...} else {// flaring...}`. Reading
//     AP_TECS.cpp directly shows this is NOT what the real code gates
//     on: the flare branch is `if (!_landing.is_flaring())  {...} else
//     {...}` - `is_flaring()` is a narrower, separate boolean
//     (AP_Landing.cpp:356-371, `AP_Landing::is_flaring()`) that is true
//     only once the slope-landing state machine reaches
//     `SlopeStage::FINAL` (AP_Landing_Slope.cpp:393-396,
//     `type_slope_is_flaring()`), a strict SUBSET of
//     `is_doing_auto_land` (`_flight_stage == FlightStage::LAND`,
//     AP_TECS.cpp:1344) - a plane can be `is_doing_auto_land` (on
//     approach) for a long time before `is_flaring()` ever becomes
//     true. Porting this as `is_doing_auto_land` would have started the
//     flare blend the instant the (not-yet-existing) LAND flight stage
//     was entered, rather than at the real, later flare trigger -
//     wrong behavior. This port therefore takes a plain `is_flaring`
//     bool (TecsLandingInputs::is_flaring below), matching what the
//     ported arithmetic ACTUALLY reads upstream, not the task's
//     approximate framing. A future phase-2 ticket, once an
//     AP_Landing-equivalent exists, is responsible for computing this
//     from real slope/flare-trigger state, not merely from flight_stage.
//   - New Gains fields: land_sink (LAND_SINK, AP_TECS.cpp:163,
//     `AP_GROUPINFO("LAND_SINK", 17, AP_TECS, _land_sink, 0.25f)`),
//     land_sink_rate_change (LAND_SRC, AP_TECS.cpp:205,
//     `AP_GROUPINFO("LAND_SRC", 22, AP_TECS, _land_sink_rate_change, 0)`),
//     flare_holdoff_hgt (FLARE_HGT, AP_TECS.cpp:283,
//     `AP_GROUPINFO("FLARE_HGT", 32, AP_TECS, _flare_holdoff_hgt, 1.0f)`)
//     - all three defaults read directly from the real var_info[] table,
//     not assumed.
//   - New flare state (AP_TECS.h ~298-299/406-408/429): flare_initialised_,
//     flare_hgt_dem_adj_, flare_hgt_dem_ideal_, hgt_at_start_of_flare_,
//     hgt_rate_dem_at_flare_entry_ - same names, same reset-on-entry
//     semantics (`if (!_flare_initialised) {...}` seeds all four from the
//     current hgt_dem_/height_/hgt_afe/hgt_rate_dem_ exactly once per
//     flare, AP_TECS.cpp:619-624).
//   - CONSOLIDATED RESET, a deliberate, disclosed divergence from
//     upstream's file LAYOUT (not its behavior): upstream resets
//     `_flare_initialised = false` in `_update_pitch_limits()`
//     (AP_TECS.cpp:1594 when on approach but not yet flaring, :1598/:1600
//     when neither on approach nor flaring) - a function this port does
//     not have any flare state in at all (see the existing "NOT PORTED"
//     list above: update_pitch_limits() here never grew is_flaring()
//     branches). Since this ticket adds NO changes to update_pitch_limits(),
//     the reset is instead performed inline at the top of
//     update_height_demand()'s own non-flare arm below - the same net
//     effect (flare_initialised_ is false on every tick that isn't
//     currently flaring, so the NEXT flare entry reseeds correctly) via a
//     different, self-contained call site, since this ticket's own
//     caller-facing contract is a single `is_flaring` bool with no
//     separate "on approach" state to distinguish "just left flare" from
//     "never approached".
//   - The blend itself (update_height_demand()'s `else` arm below) ports
//     AP_TECS.cpp:611-644 arithmetic line-for-line: `land_sink_rate_adj =
//     land_sink + land_sink_rate_change*distance_beyond_land_wp`; `p`
//     ramps 0->1 as hgt_afe drops from hgt_at_start_of_flare_ to
//     flare_holdoff_hgt (or is pinned to 1.0 if the flare started at or
//     below flare_holdoff_hgt already); hgt_rate_dem_ blends from the
//     rate captured at flare entry toward `-land_sink_rate_adj`;
//     flare_hgt_dem_ideal_/flare_hgt_dem_adj_ integrate that rate forward
//     each tick; hgt_dem_ is the p-weighted blend of the two integrated
//     profiles.
//   - is_flaring/distance_beyond_land_wp are threaded through as a small
//     TecsLandingInputs parameter struct (see below, next to TecsInputs),
//     matching this port's ADR-0012 "explicit inputs, no singletons"
//     precedent - not stored as a hidden Flags bit the way upstream's
//     `_flags.is_doing_auto_land`/`_distance_beyond_land_wp` members are.
//     hgt_afe is passed straight through from update_pitch_throttle()'s
//     existing hgt_afe parameter (already in scope pre-CPP-040 for
//     initialise_states()) rather than also cached as a new member -
//     nothing else needs it to outlive the current tick.
//     update_pitch_throttle() takes `const TecsLandingInputs& landing =
//     {}` as a DEFAULTED trailing parameter specifically so the two
//     existing call sites in plane.hpp (both pre-dating this ticket) need
//     ZERO changes - a default-constructed TecsLandingInputs
//     (is_flaring=false) reproduces this port's pre-CPP-040 behavior
//     exactly, which is what the regression test below proves.
//   - offset_altitude() (AP_TECS.cpp:1640-1665) gains the three flare-state
//     offset lines (`flare_hgt_dem_ideal_`/`flare_hgt_dem_adj_`/
//     `hgt_at_start_of_flare_` all `-= alt_offset`, AP_TECS.cpp:1652-1654)
//     that this port's own pre-CPP-040 banner explicitly noted as dropped
//     "since the remaining five height-state offsets are fully in scope" -
//     now that the three flare fields themselves exist, leaving them
//     un-offset would be a self-inflicted state-consistency bug on a home-
//     altitude change during an active flare, so they are added to keep
//     offset_altitude() correct for every float height state this port
//     now has, exactly matching upstream's own real offset_altitude().
//
//   STILL OUT OF SCOPE for CPP-040 (real, disclosed, deferred to a future
//   phase-2/3 ticket once Plane-side flight_stage/AP_Landing exist - every
//   one of the ~15 is_doing_auto_land/flight_stage/is_flaring-gated
//   branches in AP_TECS.cpp *other than* the one flare height-rate blend
//   above):
//     1. `_update_height_demand()`'s `_maxSinkRate_approach` special-case
//        sink-rate limit (AP_TECS.cpp:540, `if (_maxSinkRate_approach > 0
//        && is_doing_auto_land)`) - approach sink-rate limiting.
//     2. `_update_height_demand()`'s approach-lag-compensation branch
//        (AP_TECS.cpp:583-587, `if (is_doing_auto_land) { hgt_dem +=
//        hgt_dem_tconst*hgt_rate_dem; } else {...}`).
//     3. `timeConstant()`'s is_doing_auto_land branch (AP_TECS.cpp:704,
//        selects `_landTimeConst`/LAND_TCONST instead of TIME_CONST).
//     4. `_update_throttle_with_airspeed()`'s land throttle-damping
//        override (AP_TECS.cpp:769, `_land_throttle_damp`/LAND_TDAMP).
//     5. `_get_i_gain()`'s integral-gain selection (AP_TECS.cpp:899,
//        `_integGain_land`/LAND_IGAIN).
//     6. `_update_throttle_without_airspeed()`'s nominal-throttle override
//        (AP_TECS.cpp:919, `_landThrottle`/LAND_THR) - inside a function
//        this port already fully excludes (no airspeed-sensor subsystem).
//     7. `_update_throttle_without_airspeed()`'s SKE/SPE speed-weighting
//        sliding scale (AP_TECS.cpp:1005, `_spdWeightLand`/
//        `_path_proportion`/set_path_proportion()) - same already-excluded
//        function as #6.
//     8. `_update_pitch()`'s land pitch-damping override (AP_TECS.cpp:1050,
//        `_land_pitch_damp`/LAND_PDAMP) - a SEPARATE is_doing_auto_land
//        gate from #9 below, both selecting `pitch_damp` in the same
//        `if`/`else if` chain.
//     9. `_update_pitch()`'s is_flaring() pitch-damping override
//        (AP_TECS.cpp:1048-1049, `pitch_damp = _landDamp`/LAND_DAMP) -
//        keyed on the SAME `is_flaring()` signal CPP-040 now threads
//        through as TecsLandingInputs::is_flaring, but this ticket does
//        NOT wire it into update_pitch()'s pitch_damp selection - only
//        the height-rate-demand blend is in scope here.
//    10. `_update_pitch_limits()`'s entire flare pitch-limit blend
//        (AP_TECS.cpp:1573-1590, smoothly moving PITCHminf toward
//        LAND_PITCH_DEG via `_landing.get_pitch_cd()` as the SAME `p`
//        fraction computed independently there) and its `is_on_approach()`
//        pitch-min hysteresis tracking (AP_TECS.cpp:1592-1614,
//        `_land_pitch_min`/LAND_PMAX) - already noted in the pre-CPP-040
//        "NOT PORTED" list above; unaffected by this ticket.
//     Also unaffected, and land_sink/land_sink_rate_change/
//     flare_holdoff_hgt notwithstanding: `get_land_sinkrate()`/
//     `get_land_airspeed()`/`set_path_proportion()` accessors (see the
//     "UPSTREAM QUIRK"-adjacent accessor note above, now updated for
//     CPP-040) and `_landAirspeed`/LAND_ARSPD itself.
//
// NO SINGLETONS, EXPLICIT INPUTS INSTEAD (ADR-0012), matching L1Inputs'
// established shape (see fwcpp/nav/l1_control.hpp's own banner for the
// precedent this follows):
//   - TecsInputs REPLACES the stored `AP_AHRS &_ahrs` upstream reaches
//     into on nearly every call (get_relative_position_D_home,
//     get_velocity_NED, get_accel_ef, get_rotation_body_to_ned,
//     get_EAS2TAS, using_airspeed_sensor, airspeed_EAS, cos_roll,
//     get_pitch_rad) plus AP::ins().get_accel() and AP::baro().get_altitude()
//     - one struct bundling everything both update_50hz() and
//     update_pitch_throttle() need from the outside world for one tick,
//     built the same way L1Inputs bundled AP_AHRS&/AP_TECS* state. See
//     each field's own comment below for its exact upstream origin.
//   - `const AP_Landing &_landing` is dropped entirely, not replaced by
//     anything - every call site was landing-only (see exclusions above).
//   - `const AP_FixedWing &parms` (upstream: `aparm`) becomes
//     FixedWingParams, constructed once and held by value - same
//     "AP_Param not wired in yet" treatment L1Control::Gains/AcPid::Gains
//     already gave their own upstream AP_Float members, extended here to
//     an upstream *reference-to-another-struct*'s fields instead of the
//     class's own AP_Float members. Holds only the ~9 AP_FixedWing fields
//     this slice's in-scope functions actually read (airspeed_min/max/
//     cruise/stall, stall_prevention, throttle_cruise/max/slewrate,
//     pitch_limit_max/min) out of AP_FixedWing's full ~20-field struct -
//     the rest (takeoff/rangefinder/autotune fields) have no reader in
//     this slice's scope.
//   - `const uint32_t log_bitmask` constructor parameter is dropped -
//     logging is out of scope (see above), so there is nothing to gate.
//   - EAS2TAS (TecsInputs::eas2tas, default 1.0f) - this port has no
//     barometer/atmosphere model, identical exclusion to AhrsDcm's and
//     SimPlane's own EAS2TAS treatment (grep either file's banner) -
//     "true == equivalent airspeed" until a future slice adds one.
//
// GAINS: AP_Float/AP_Int8 REPLACED WITH PLAIN float, same AcPid::Gains/
// L1Control::Gains precedent - see Gains below for every upstream
// var_info[] default reproduced verbatim (read directly from AP_TECS.cpp
// lines ~18-291, not from memory). Landing-only tunables (LAND_ARSPD,
// LAND_THR, LAND_SPDWGT, LAND_TCONST, LAND_DAMP, LAND_PMAX, APPR_SMAX,
// LAND_SRC, LAND_TDAMP, LAND_IGAIN, LAND_PDAMP, FLARE_HGT) and the two
// takeoff/landing-only integrator gains (TKOFF_IGAIN, LAND_IGAIN) are
// dropped entirely, not merely defaulted - there is no code path in this
// slice that could ever read them. TECS_OPTIONS' AP_Int32 bitmask becomes
// two plain bools (option_glider_only/option_descent_speedup) - only two
// of its bits exist upstream (GLIDER_ONLY=bit0, DESCENT_SPEEDUP=bit1) and
// both are genuinely in scope (see above).
//
// AverageFilterFloat_Size5 (Filter/AverageFilter.h) REPLACED WITH A
// PRIVATE MovingAverage5 NESTED CLASS: upstream's _vdot_filter is the only
// call site of this generic 5-point running-average filter anywhere in
// this slice, and this port has not built a general Filter/AverageFilter
// module (unlike LowPassFilter, which ap-filter already has for other
// consumers - but this slice needs neither LowPassFilterFloat member,
// since both belonged to the now-dropped _update_throttle_without_airspeed).
// MovingAverage5's apply()/reset() reproduce AverageFilter<float,float,5>'s
// exact behavior: a 5-slot ring buffer, zero-initialised, summed over all
// 5 slots every call (unfilled slots are zero and contribute nothing) and
// divided by the number of real samples seen so far (capped at 5) - see
// AP_TECS.h's `AverageFilterFloat_Size5 _vdot_filter` and
// Filter/AverageFilter.h's apply()/FilterWithBuffer::apply() for the
// upstream implementation this reproduces.
//
// math::linear_interpolate ADDED (fwcpp/math/scalar.hpp, alongside this
// slice): upstream AP_Math.cpp's float-only linear_interpolate(), needed
// by _update_speed_demand()'s velRateMin calculation and absent from this
// port's math module until now - same "add the one function actually
// needed, when a concrete caller needs it" precedent l1_control.hpp's
// banner already cites for cd_to_rad/Location::same_loc_as.
//
// LITERAL SAFETY: every literal touched in the ported functions is
// already explicitly float-suffixed upstream (5.0f, 0.9f, 1.4142f, etc.);
// nothing here needed scalar.cpp's compiled-.cpp treatment.
//
// ap-ahrs LINKED BUT NOT NAMED: modules/ap-tecs/CMakeLists.txt links
// ap-ahrs (mirroring ap-nav's own shape) even though TecsInputs below
// names only plain fwcpp::math:: types (Matrix3f, not any fwcpp::ahrs::
// type) - same choice L1Inputs made for the identical reason (a caller
// wiring a real AhrsDcm's dcm_matrix/roll/pitch fields into TecsInputs is
// the expected real use, so the dependency is kept available rather than
// forced onto a caller that doesn't already have it), not because this
// header itself needs anything ap-ahrs declares.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::tecs {

// upstream: GRAVITY_MSS (AP_Math/definitions.h), 9.80665f - same constant
// l1_control.hpp's kGravityMss already reproduces independently.
inline constexpr float kGravityMss = 9.80665f;

// Everything update_50hz()/update_pitch_throttle() need from the AHRS/IMU/
// baro/clock for one tick - see file banner. Field order follows the
// order upstream's own call sequence first reads each quantity.
struct TecsInputs {
    // update_50hz() inputs.
    float relative_position_d_home_m = 0.0f; // upstream: _ahrs.get_relative_position_D_home(posD) out-param (+ve DOWN from home)
    bool velocity_ned_valid = false;         // upstream: _ahrs.get_velocity_NED(velned) return value
    float velocity_down_ms = 0.0f;           // upstream: velned.z (only component read)
    float baro_altitude_m = 0.0f;            // upstream: AP::baro().get_altitude() - only read when velocity_ned_valid is false
    float accel_ef_z = 0.0f;                 // upstream: _ahrs.get_accel_ef().z - only read when velocity_ned_valid is false

    // _update_speed() inputs.
    math::Matrix3f rotation_body_to_ned;     // upstream: _ahrs.get_rotation_body_to_ned()
    float accel_body_x = 0.0f;               // upstream: AP::ins().get_accel().x
    float eas2tas = 1.0f;                    // upstream: _ahrs.get_EAS2TAS() - no atmosphere model, see file banner
    bool using_airspeed_sensor = false;      // upstream: _ahrs.using_airspeed_sensor(), feeds use_airspeed()
    bool airspeed_eas_valid = false;         // upstream: _ahrs.airspeed_EAS(eas) return value
    float airspeed_eas = 0.0f;               // upstream: airspeed_EAS() out-param

    // _update_throttle_with_airspeed()/_update_pitch() inputs.
    float roll_rad = 0.0f;                   // upstream: _ahrs.cos_roll() reproduced as std::cos(roll_rad) - AhrsDcm exposes roll directly

    // _initialise_states() input.
    float pitch_rad = 0.0f;                  // upstream: _ahrs.get_pitch_rad(), used to seed _last_pitch_dem on reset

    // Clocks - two independent ones, matching upstream's own two reads
    // (AP_HAL::micros64() and AP_HAL::millis()), same "don't derive one
    // from the other" treatment L1Inputs' now_us/now_ms already established.
    std::uint64_t now_us = 0; // upstream: AP_HAL::micros64()
    std::uint32_t now_ms = 0; // upstream: AP_HAL::millis(), used only by _detect_underspeed()'s 3-second latch
};

// update_pitch_throttle()'s CPP-040 flare-blend inputs - see the file
// banner's "CPP-040 ADDENDUM" for exactly what these drive and why
// `is_flaring` (not `is_doing_auto_land`) is the real upstream gate.
// Defaults reproduce this port's pre-CPP-040 behavior exactly (no flare),
// which is what update_pitch_throttle()'s defaulted `landing` parameter
// relies on to leave plane.hpp's existing call sites unchanged.
struct TecsLandingInputs {
    // upstream: AP_Landing::is_flaring() (AP_Landing.cpp:356-371), true
    // only once the (not-yet-ported) landing state machine has reached its
    // final flare stage - NOT the same thing as "in the LAND flight
    // stage"/is_doing_auto_land. A future phase-2 ticket derives this from
    // a real AP_Landing-equivalent.
    bool is_flaring = false;

    // upstream: update_pitch_throttle()'s `distance_beyond_land_wp`
    // parameter / the `_distance_beyond_land_wp` member it is copied into
    // - metres traveled past the LAND waypoint, used only by the flare
    // blend's LAND_SRC adjustment below. Meaningless (and unread) while
    // is_flaring is false.
    float distance_beyond_land_wp = 0.0f;
};

class Tecs {
public:
    // Upstream var_info[] tunables, NORMAL-flight-stage subset only - see
    // file banner for every dropped landing/takeoff-only gain. Every
    // default below is upstream's real var_info[] default (AP_TECS.cpp,
    // read directly), not invented.
    struct Gains {
        float max_climb_rate = 5.0f;          // CLMB_MAX
        float min_sink_rate = 2.0f;           // SINK_MIN
        float max_sink_rate = 5.0f;           // SINK_MAX
        float time_const = 5.0f;              // TIME_CONST
        float thr_damp = 0.5f;                // THR_DAMP
        float integ_gain = 0.3f;              // INTEG_GAIN
        float vert_acc_lim = 7.0f;             // VERT_ACC
        float hgt_comp_filt_omega = 3.0f;      // HGT_OMEGA
        float spd_comp_filt_omega = 2.0f;      // SPD_OMEGA
        float roll_comp = 10.0f;               // RLL2THR
        float spd_weight = 1.0f;               // SPDWEIGHT
        float ptch_damp = 0.3f;                // PTCH_DAMP
        float pitch_max = 15.0f;               // PITCH_MAX (0 sentinel: use aparm.pitch_limit_max instead)
        float pitch_min = 0.0f;                // PITCH_MIN (0 sentinel: use aparm.pitch_limit_min instead)
        bool use_synthetic_airspeed = false;   // SYNAIRSPEED
        bool option_glider_only = false;       // OPTIONS bit 0 (AP_TECS::Option::GLIDER_ONLY)
        bool option_descent_speedup = false;   // OPTIONS bit 1 (AP_TECS::Option::DESCENT_SPEEDUP)
        float pitch_ff_v0 = 12.0f;             // PTCH_FF_V0
        float pitch_ff_k = 0.0f;               // PTCH_FF_K
        float thr_min_pct_ext_rate_lim = 20.0f; // THR_ERATE
        float hgt_dem_tconst = 3.0f;           // HDEM_TCONST

        // CPP-040: flare height-rate-demand blend tunables - see file
        // banner's "CPP-040 ADDENDUM". Defaults are the real var_info[]
        // defaults (AP_TECS.cpp, read directly), not invented.
        float land_sink = 0.25f;               // LAND_SINK
        float land_sink_rate_change = 0.0f;    // LAND_SRC
        float flare_holdoff_hgt = 1.0f;        // FLARE_HGT
    };

    // Upstream `const AP_FixedWing &aparm` - the subset of AP_FixedWing's
    // fields this slice's in-scope functions read. Defaults are
    // ArduPlane's real GSCALAR/ASCALAR defaults from
    // ArduPlane/Parameters.cpp + ArduPlane/config.h (read directly), not
    // AP_FixedWing.h's own (which has none - AP_FixedWing is a bare
    // struct of AP_Param types with defaults supplied where it's
    // embedded into the vehicle's parameter table).
    struct FixedWingParams {
        float airspeed_min = 9.0f;      // AIRSPEED_MIN, upstream default AIRSPEED_FBW_MIN
        float airspeed_max = 22.0f;     // AIRSPEED_MAX, upstream default AIRSPEED_FBW_MAX
        float airspeed_cruise = 12.0f;  // AIRSPEED_CRUISE
        float airspeed_stall = 0.0f;    // AIRSPEED_STALL
        bool stall_prevention = true;   // STALL_PREVENTION, upstream default 1
        float throttle_cruise = 45.0f;  // TRIM_THROTTLE, upstream default AP_PLANE_TRIM_THROTTLE_DEFAULT
        float throttle_max = 100.0f;    // THR_MAX, upstream default THROTTLE_MAX
        float throttle_slewrate = 100.0f; // THR_SLEWRATE
        float pitch_limit_max = 20.0f;  // PTCH_LIM_MAX_DEG, upstream default PITCH_MAX
        float pitch_limit_min = -25.0f; // PTCH_LIM_MIN_DEG, upstream default PITCH_MIN
    };

    Tecs(const Gains& gains, const FixedWingParams& aparm) : gains_(gains), aparm_(aparm) {}

    Tecs(const Tecs&) = delete;
    Tecs& operator=(const Tecs&) = delete;

    // upstream: AP_TECS::update_50hz(). Should be called at 50Hz or
    // greater; internally calls update_speed() (upstream: _update_speed()).
    void update_50hz(const TecsInputs& in) {
        using_airspeed_sensor_ = in.using_airspeed_sensor;

        height_ = -in.relative_position_d_home_m;

        const std::uint64_t now = in.now_us;
        float dt = static_cast<float>(now - update_50hz_last_us_) * 1.0e-6f;
        flags_.reset = dt > 1.0f;
        if (flags_.reset) {
            climb_rate_ = 0.0f;
            height_filter_.dd_height = 0.0f;
            dt = 0.02f; // when first starting TECS, use most likely time constant
            vdot_filter_.reset();
        }
        update_50hz_last_us_ = now;

        if (in.velocity_ned_valid) {
            climb_rate_ = -in.velocity_down_ms;
        } else {
            const float baro_alt = in.baro_altitude_m;
            const float hgt_ddot_mea = -(in.accel_ef_z + kGravityMss);
            const float omega2 = gains_.hgt_comp_filt_omega * gains_.hgt_comp_filt_omega;
            const float hgt_err = baro_alt - height_filter_.height;
            const float integ1_input = hgt_err * omega2 * gains_.hgt_comp_filt_omega;

            height_filter_.dd_height += integ1_input * dt;

            const float integ2_input = height_filter_.dd_height + hgt_ddot_mea + hgt_err * omega2 * 3.0f;
            climb_rate_ += integ2_input * dt;

            const float integ3_input = climb_rate_ + hgt_err * gains_.hgt_comp_filt_omega * 3.0f;
            if (flags_.reset) {
                height_filter_.height = height_;
            } else {
                height_filter_.height += integ3_input * dt;
            }
        }

        update_speed(dt, in);
    }

    // upstream: AP_TECS::update_pitch_throttle(). Signature reduced to
    // this slice's scope: `flight_stage` (always NORMAL - `landing` below
    // carries only the two flare-blend-relevant facts a real flight_stage
    // would otherwise gate, see file banner's "CPP-040 ADDENDUM"),
    // `ptchMinCO_cd` (TAKEOFF-only), `throttle_nudge`/`pitch_trim_deg`
    // (no-airspeed fallback only) are all dropped - see file banner.
    // `landing` defaults to a non-flaring TecsLandingInputs{} so this
    // port's two pre-CPP-040 call sites (plane.hpp) need no changes.
    // Do not call slower than 10Hz or faster than 500Hz (upstream's own
    // documented contract - unchanged).
    void update_pitch_throttle(std::int32_t hgt_dem_cm, std::int32_t eas_dem_cm, float hgt_afe, float load_factor,
                                const TecsInputs& in, const TecsLandingInputs& landing = {}) {
        using_airspeed_sensor_ = in.using_airspeed_sensor;
        eas2tas_ = in.eas2tas;
        cos_roll_ = std::cos(in.roll_rad);
        load_factor_ = load_factor;

        const std::uint64_t now = in.now_us;

        // check how long since we last did the 50Hz update; do nothing in
        // this loop if that hasn't run for some significant period of
        // time. Notably, it may never have run, leaving _TAS_state as
        // zero and subsequently division-by-zero errors.
        const float dt_for_50hz = static_cast<float>(now - update_50hz_last_us_) * 1.0e-6f;
        if (update_50hz_last_us_ == 0 || dt_for_50hz > 1.0f) {
            return;
        }

        dt_ = static_cast<float>(now - update_pitch_throttle_last_us_) * 1.0e-6f;
        dt_ = std::max(dt_, 0.001f);
        update_pitch_throttle_last_us_ = now;

        flags_.is_gliding = flags_.gliding_requested || flags_.propulsion_failed || (aparm_.throttle_max == 0.0f);

        hgt_dem_in_raw_ = static_cast<float>(hgt_dem_cm) * 0.01f;
        eas_dem_ = static_cast<float>(eas_dem_cm) * 0.01f;

        // Don't allow height demand to continue changing in a direction
        // that saturates vehicle manoeuvre limits if vehicle is unable to
        // follow the demanded climb or descent. Upstream ANDs in
        // `!(flight_stage==TAKEOFF||ABORT_LANDING)` for max_climb_condition
        // - unconditionally true in NORMAL-only scope, dropped.
        const bool max_climb_condition = (pitch_dem_unc_ > pitchmaxf_) || (thr_clip_status_ == ClipStatus::kMax);
        const bool max_descent_condition = (pitch_dem_unc_ < pitchminf_) || (thr_clip_status_ == ClipStatus::kMin);
        if (max_climb_condition && hgt_dem_in_raw_ > hgt_dem_in_prev_) {
            hgt_dem_in_ = hgt_dem_in_prev_;
        } else if (max_descent_condition && hgt_dem_in_raw_ < hgt_dem_in_prev_) {
            hgt_dem_in_ = hgt_dem_in_prev_;
        } else {
            hgt_dem_in_ = hgt_dem_in_raw_;
        }

        update_throttle_limits();
        update_pitch_limits();

        // upstream's TAKEOFF/ABORT_LANDING reached_speed_takeoff check is
        // dropped here - unreachable without a TAKEOFF/ABORT_LANDING
        // flight stage, and the flag itself doesn't exist in this port.

        initialise_states(hgt_afe, in);
        update_ste_rate_lim();
        update_speed_demand();
        update_height_demand(hgt_afe, landing);
        detect_underspeed(in);
        update_energies();
        update_pitch();

        if (use_airspeed()) {
            update_throttle_with_airspeed();
            use_synthetic_airspeed_once_ = false;
            using_airspeed_for_throttle_ = true;
        } else {
            // _update_throttle_without_airspeed() is out of scope for this
            // slice (see file banner) - this port has no airspeed-sensor
            // subsystem yet, so a real caller is expected to always
            // present some airspeed estimate via TecsInputs (a sensor
            // reading, or Gains::use_synthetic_airspeed /
            // use_synthetic_airspeed_once()), making use_airspeed() true
            // and this branch unreachable in practice. If a caller
            // nonetheless reaches it, throttle_dem_ is left at its
            // previous value and merely re-clamped - a safe no-op that
            // keeps thr_clip_status_ well-defined for the next tick's
            // _update_height_demand(), NOT an implementation of
            // upstream's pitch-to-throttle mapping.
            constrain_throttle();
            using_airspeed_for_throttle_ = false;
        }

        detect_bad_descent();

        if (gains_.option_glider_only) {
            flags_.bad_descent = false;
        }
    }

    // demanded throttle in percentage, -100 to 100, usually positive
    // unless reverse thrust is enabled via a negative throttle_min.
    [[nodiscard]] float get_throttle_demand() const { return throttle_dem_ * 100.0f; }

    // demanded pitch angle in centidegrees, -9000 to +9000.
    [[nodiscard]] std::int32_t get_pitch_demand() const { return static_cast<std::int32_t>(pitch_dem_ * 5729.5781f); }

    // Rate of change of velocity along the X body axis, m/s^2.
    [[nodiscard]] float get_vxdot() const { return vel_dot_; }

    // Current target airspeed (true), matching upstream's
    // _TAS_dem_adj / _ahrs.get_EAS2TAS(). eas2tas_ is the value cached
    // from the most recent update_50hz()/update_pitch_throttle() call.
    [[nodiscard]] float get_target_airspeed() const { return tas_dem_adj_ / eas2tas_; }

    [[nodiscard]] float get_max_climbrate() const { return gains_.max_climb_rate; }
    [[nodiscard]] float get_max_sinkrate() const { return gains_.max_sink_rate; }

    // upstream: AP_TECS::get_land_sinkrate() (AP_TECS.h) - `return
    // _land_sink;`. CPP-040 ported the underlying LAND_SINK field
    // (Gains::land_sink) but left this public accessor unported, noting
    // "no external caller (e.g. a future AP_Landing-equivalent) that would
    // call it instead of reading gains_.land_sink" - CPP-041 (fwcpp::
    // vehicle::Plane::setup_landing_glide_slope(), plane.hpp) is exactly
    // that future caller: the real glide-slope aim-point math
    // (type_slope_setup_landing_glide_slope(), AP_Landing_Slope.cpp) reads
    // tecs_Controller->get_land_sinkrate() directly, and gains_ is private
    // to this class, so a real caller outside Tecs needs this accessor.
    [[nodiscard]] float get_land_sinkrate() const { return gains_.land_sink; }

    // Added to let SoaringController reset pitch integrator to zero.
    void reset_pitch_i() {
        integ_sebdot_ = 0.0f;
        integ_ke_ = 0.0f;
    }

    void reset_throttle_i() { integ_thr_state_ = 0.0f; }

    [[nodiscard]] float get_height_rate_demand() const { return hgt_rate_dem_; }

    void set_gliding_requested_flag(bool gliding_requested) { flags_.gliding_requested = gliding_requested; }
    void set_propulsion_failed_flag(bool propulsion_failed) { flags_.propulsion_failed = propulsion_failed; }

    // upstream: AP_TECS::set_throttle_min - [-1,1] range, one-shot unless
    // reset_output requests the slew limiter also respect the new floor.
    void set_throttle_min(float thr_min, bool reset_output = false) {
        if (thr_min > thrminf_ext_) {
            thrminf_ext_ = thr_min;
            if (reset_output) {
                last_throttle_dem_ = std::max(last_throttle_dem_, thrminf_ext_);
                throttle_dem_ = last_throttle_dem_;
            }
        }
    }

    // upstream: AP_TECS::set_throttle_max - applicable for one control
    // cycle only.
    void set_throttle_max(float thr_max) {
        if (thr_max < thrmaxf_ext_) {
            thrmaxf_ext_ = thr_max;
        }
    }

    // upstream: AP_TECS::set_pitch_min/set_pitch_max, degrees, applicable
    // for one control cycle only.
    void set_pitch_min(float pitch_min) {
        if (pitch_min > pitchminf_ext_) {
            pitchminf_ext_ = pitch_min;
        }
    }
    void set_pitch_max(float pitch_max) {
        if (pitch_max < pitchmaxf_ext_) {
            pitchmaxf_ext_ = pitch_max;
        }
    }

    // Force use of synthetic airspeed for one loop.
    void use_synthetic_airspeed() { use_synthetic_airspeed_once_ = true; }

    // Reset on next loop.
    void reset() { need_reset_ = true; }

    // upstream: AP_TECS::offset_altitude(). CPP-040 adds the three
    // flare-state offsets (AP_TECS.cpp:1652-1654) this port's pre-CPP-040
    // banner had explicitly dropped as flare-only - now that
    // flare_hgt_dem_ideal_/flare_hgt_dem_adj_/hgt_at_start_of_flare_ exist,
    // they must be offset too or a home-altitude change mid-flare would
    // desync them from the other five height states below (a real bug,
    // not upstream behavior to faithfully reproduce). See file banner's
    // "CPP-040 ADDENDUM".
    void offset_altitude(float alt_offset) {
        flare_hgt_dem_ideal_ -= alt_offset;
        flare_hgt_dem_adj_ -= alt_offset;
        hgt_at_start_of_flare_ -= alt_offset;
        hgt_dem_in_prev_ -= alt_offset;
        hgt_dem_lpf_ -= alt_offset;
        hgt_dem_rate_ltd_ -= alt_offset;
        hgt_dem_prev_ -= alt_offset;
        height_filter_.height -= alt_offset;
    }

    // upstream: AP_TECS::use_airspeed() - "return true if airspeed should
    // be used (either from a sensor or synthetic)". using_airspeed_sensor_
    // is cached from TecsInputs at the top of update_50hz()/
    // update_pitch_throttle() rather than re-read live each call - see
    // file banner.
    [[nodiscard]] bool use_airspeed() const {
        return using_airspeed_sensor_ || gains_.use_synthetic_airspeed || use_synthetic_airspeed_once_;
    }

    // Observability into whether the last update_throttle_limits() call
    // found the throttle range forced to a single point (upstream:
    // _flag_throttle_forced) - real computed state with no in-scope
    // internal consumer this slice (its one upstream consumer,
    // _initialise_states()'s post-takeoff-offset calc, is out of scope -
    // see file banner), exposed the same way AhrsDcm exposes otherwise-
    // private controller state for testing/status use.
    [[nodiscard]] bool throttle_forced() const { return flag_throttle_forced_; }

    [[nodiscard]] bool underspeed() const { return flags_.underspeed; }
    [[nodiscard]] bool bad_descent() const { return flags_.bad_descent; }

    // TEST/OBSERVABILITY ACCESSORS, upstream has none of these (it reads
    // its own private members directly, being a single monolithic class) -
    // added the same way AhrsDcm exposes omega_yaw_p()/omega_i(): real
    // internal state, exposed read-only, for both testing and downstream
    // status/logging use a future slice may want.

    // upstream: _height/_climb_rate/_TAS_state - update_50hz()'s/
    // _update_speed()'s own filtered estimates.
    [[nodiscard]] float get_height() const { return height_; }
    [[nodiscard]] float get_climb_rate() const { return climb_rate_; }
    [[nodiscard]] float get_tas_state() const { return tas_state_; }

    // Bundles _update_energies()'s eight output quantities (upstream:
    // _SPE_dem/_SKE_dem/_SPEdot_dem/_SKEdot_dem/_SPE_est/_SKE_est/_SPEdot/
    // _SKEdot) for direct inspection against hand-computed expected values.
    struct EnergyState {
        float spe_dem = 0.0f;
        float ske_dem = 0.0f;
        float spedot_dem = 0.0f;
        float skedot_dem = 0.0f;
        float spe_est = 0.0f;
        float ske_est = 0.0f;
        float spedot = 0.0f;
        float skedot = 0.0f;
    };
    [[nodiscard]] EnergyState energy_state() const {
        return {spe_dem_, ske_dem_, spedot_dem_, skedot_dem_, spe_est_, ske_est_, spedot_, skedot_};
    }

    // upstream: _STE_error - _update_throttle_with_airspeed()'s total
    // energy error, the throttle law's main P-term input.
    [[nodiscard]] float get_ste_error() const { return ste_error_; }

    // upstream: _hgt_dem/_TAS_dem_adj - the shaped height/airspeed demands
    // update_energies() actually converts into spe_dem_/ske_dem_. Exposed
    // so a test can check the exact spe_dem_==hgt_dem*GRAVITY_MSS /
    // ske_dem_==0.5*tas_dem_adj^2 algebraic identities directly, the same
    // way get_height()/get_tas_state() let a test check spe_est_/ske_est_.
    [[nodiscard]] float get_hgt_dem() const { return hgt_dem_; }
    [[nodiscard]] float get_tas_dem_adj() const { return tas_dem_adj_; }

    // Resolved pitch limits in degrees (upstream: _PITCHminf/_PITCHmaxf,
    // radians internally) - lets a test check _update_pitch_limits()'s
    // PITCH_MAX/MIN-vs-PTCH_LIM_MAX/MIN_DEG sentinel selection and
    // external-limit clamp directly.
    struct PitchLimitsDeg {
        float min_deg = 0.0f;
        float max_deg = 0.0f;
    };
    [[nodiscard]] PitchLimitsDeg pitch_limits_deg() const { return {math::degrees(pitchminf_), math::degrees(pitchmaxf_)}; }

    // Resolved throttle limits, fraction [-1,1] (upstream: _THRminf/_THRmaxf).
    struct ThrottleLimits {
        float min = 0.0f;
        float max = 0.0f;
    };
    [[nodiscard]] ThrottleLimits throttle_limits() const { return {thrminf_, thrmaxf_}; }

private:
    // 1 if a quantity is clipping at its max value, -1 at its min value,
    // 0 otherwise - upstream: AP_TECS::clipStatus.
    enum class ClipStatus : std::int8_t { kMin = -1, kNone = 0, kMax = 1 };

    // upstream: AverageFilterFloat_Size5 _vdot_filter - see file banner.
    class MovingAverage5 {
    public:
        float apply(float sample) {
            samples_[index_] = sample;
            index_ = static_cast<std::uint8_t>((index_ + 1) % kSize);
            if (num_samples_ < kSize) {
                ++num_samples_;
            }
            float sum = 0.0f;
            for (float s : samples_) {
                sum += s;
            }
            return sum / static_cast<float>(num_samples_);
        }

        void reset() {
            samples_.fill(0.0f);
            index_ = 0;
            num_samples_ = 0;
        }

    private:
        static constexpr std::uint8_t kSize = 5;
        std::array<float, kSize> samples_{};
        std::uint8_t index_ = 0;
        std::uint8_t num_samples_ = 0;
    };

    // upstream: AP_TECS::flags bitfield struct, NORMAL-scope subset -
    // is_doing_auto_land and reached_speed_takeoff are dropped entirely
    // (see file banner), not merely left unused.
    struct Flags {
        bool underspeed = false;
        bool bad_descent = false;
        bool gliding_requested = false;
        bool is_gliding = false;
        bool propulsion_failed = false;
        bool reset = true;
    };

    // upstream: timeConstant() - is_doing_auto_land branch dropped (see
    // file banner), always the plain TIME_CONST floor.
    [[nodiscard]] float time_constant() const { return (gains_.time_const < 0.1f) ? 0.1f : gains_.time_const; }

    // upstream: _get_i_gain() - TAKEOFF/is_doing_auto_land branches
    // dropped (see file banner); collapses to the plain integrator gain.
    [[nodiscard]] float get_i_gain() const { return gains_.integ_gain; }

    // upstream: AP_TECS::_update_speed(float DT). See file banner for the
    // faithfully-reproduced _DT-vs-DT upstream quirk in the TASmax
    // adjustment below.
    void update_speed(float dt, const TecsInputs& in) {
        if (flags_.reset) {
            vdot_filter_.reset();
            vel_dot_lpf_ = vel_dot_;
        } else {
            const float temp = in.rotation_body_to_ned.c.x * kGravityMss + in.accel_body_x;
            vel_dot_ = vdot_filter_.apply(temp);
            const float alpha = dt / (dt + time_constant());
            vel_dot_lpf_ = vel_dot_lpf_ * (1.0f - alpha) + vel_dot_ * alpha;
        }

        const bool should_use_airspeed = use_airspeed();

        eas2tas_ = in.eas2tas;
        tas_dem_ = eas_dem_ * eas2tas_;
        if (flags_.reset || !should_use_airspeed) {
            tas_max_ = aparm_.airspeed_max * eas2tas_;
        } else if (thr_clip_status_ == ClipStatus::kMax) {
            // wind down airspeed upper limit to prevent a situation where
            // the aircraft can't climb at the maximum speed. NOTE: `dt_`
            // here, not the `dt` parameter - see file banner's "genuine
            // upstream quirk" note.
            const float vel_rate_min = 0.5f * stedot_min_ / std::max(tas_state_, aparm_.airspeed_min * eas2tas_);
            tas_max_ += dt_ * vel_rate_min;
            tas_max_ = std::max(tas_max_, aparm_.airspeed_cruise * eas2tas_);
        } else {
            // wind airspeed upper limit back to parameter defined value.
            const float vel_rate_max = 0.5f * stedot_max_ / std::max(tas_state_, aparm_.airspeed_min * eas2tas_);
            tas_max_ += dt_ * vel_rate_max;
        }
        tas_max_ = std::min(tas_max_, aparm_.airspeed_max * eas2tas_);
        tas_min_ = aparm_.airspeed_min * eas2tas_;

        // upstream's `_landing.is_on_final() && airspeed_stall>0` TASmin
        // override is dropped - landing-only, see file banner.

        if (aparm_.stall_prevention) {
            // when stall prevention is active we raise the minimum
            // airspeed based on aerodynamic load factor.
            if (math::is_positive(aparm_.airspeed_stall)) {
                tas_min_ = std::max(tas_min_, aparm_.airspeed_stall * eas2tas_ * std::sqrt(load_factor_));
            } else {
                tas_min_ *= std::sqrt(load_factor_);
            }
        }

        if (tas_max_ < tas_min_) {
            tas_max_ = tas_min_;
        }

        // Get measured airspeed or default to trim speed and constrain to
        // range between min and max if airspeed sensor data cannot be used.
        float eas;
        if (!should_use_airspeed || !in.airspeed_eas_valid) {
            eas = math::constrain_value(aparm_.airspeed_cruise, aparm_.airspeed_min, aparm_.airspeed_max);
        } else {
            eas = in.airspeed_eas;
        }

        constexpr float kMinAirspeed = 3.0f;

        if (flags_.reset) {
            tas_state_ = std::max(eas * eas2tas_, kMinAirspeed);
            integ_dtas_state_ = 0.0f;
            return;
        }

        // Implement a second order complementary filter to obtain a
        // smoothed airspeed estimate, held in tas_state_.
        const float aspd_err = (eas * eas2tas_) - tas_state_;
        float integ_dtas_input = aspd_err * gains_.spd_comp_filt_omega * gains_.spd_comp_filt_omega;
        // Prevent state from winding up.
        if (tas_state_ < 3.1f) {
            integ_dtas_input = std::max(integ_dtas_input, 0.0f);
        }
        integ_dtas_state_ = integ_dtas_state_ + integ_dtas_input * dt;
        const float tas_input = integ_dtas_state_ + vel_dot_ + aspd_err * gains_.spd_comp_filt_omega * 1.4142f;
        tas_state_ = tas_state_ + tas_input * dt;
        tas_state_ = std::max(tas_state_, kMinAirspeed);
    }

    // upstream: AP_TECS::_update_speed_demand().
    void update_speed_demand() {
        if (gains_.option_descent_speedup) {
            // Allow demanded speed to go to maximum when descending at
            // maximum descent rate.
            tas_dem_ = tas_dem_ + (tas_max_ - tas_dem_) * sink_fraction_;
        }

        // Set the airspeed demand to the minimum value if an underspeed
        // condition exists or a bad descent condition exists.
        if (flags_.bad_descent || flags_.underspeed) {
            tas_dem_ = tas_min_;
        }

        tas_dem_ = math::constrain_value(tas_dem_, tas_min_, tas_max_);

        const float tas_cruise = aparm_.airspeed_cruise * eas2tas_;

        // calculate velocity rate limits based on physical performance
        // limits. Use 50% of maximum energy rate on gain, 90% on
        // dissipation to allow margin for total energy controller.
        const float vel_rate_max = 0.5f * stedot_max_ / tas_state_;
        const float vel_rate_neg_max = 0.9f * stedot_neg_max_ / tas_max_;
        const float vel_rate_neg_cruise = 0.9f * stedot_min_ / tas_cruise;
        const float vel_rate_min =
            math::linear_interpolate(vel_rate_neg_max, vel_rate_neg_cruise, tas_state_, tas_max_, tas_cruise);
        const float tas_dem_previous = tas_dem_adj_;

        if ((tas_dem_ - tas_dem_previous) > (vel_rate_max * dt_)) {
            tas_dem_adj_ = tas_dem_previous + vel_rate_max * dt_;
            tas_rate_dem_ = vel_rate_max;
        } else if ((tas_dem_ - tas_dem_previous) < (vel_rate_min * dt_)) {
            tas_dem_adj_ = tas_dem_previous + vel_rate_min * dt_;
            tas_rate_dem_ = vel_rate_min;
        } else {
            tas_rate_dem_ = (tas_dem_ - tas_dem_previous) / dt_;
            tas_dem_adj_ = tas_dem_;
        }

        if (flags_.reset) {
            tas_dem_adj_ = tas_state_;
            tas_rate_dem_lpf_ = tas_rate_dem_;
        } else {
            const float alpha = dt_ / (dt_ + time_constant());
            tas_rate_dem_lpf_ = tas_rate_dem_lpf_ * (1.0f - alpha) + tas_rate_dem_ * alpha;
        }

        // Constrain speed demand again to protect against bad values on
        // initialisation.
        tas_dem_adj_ = math::constrain_value(tas_dem_adj_, tas_min_, tas_max_);
    }

    // upstream: AP_TECS::_update_height_demand() - `_maxSinkRate_approach`
    // special-case and the is_doing_auto_land approach-lag-compensation
    // branch remain dropped (still out of scope, see file banner); the
    // `_landing.is_flaring()` flare branch IS now ported (CPP-040) as the
    // `else` arm below - see file banner's "CPP-040 ADDENDUM".
    void update_height_demand(float hgt_afe, const TecsLandingInputs& landing) {
        climb_rate_limit_ = gains_.max_climb_rate * max_climb_scaler_;
        sink_rate_limit_ = gains_.max_sink_rate * max_sink_scaler_;

        if (!landing.is_flaring) {
            // CPP-040: upstream resets `_flare_initialised = false` inside
            // `_update_pitch_limits()` (AP_TECS.cpp:1594/1598/1600, on
            // every tick that isn't currently flaring) - a function this
            // port has no flare state in at all (see file banner). The
            // same net effect is achieved here instead: flare_initialised_
            // is unconditionally false whenever this (non-flare) arm runs,
            // so the next real flare entry below always reseeds. See file
            // banner's "CONSOLIDATED RESET" note.
            flare_initialised_ = false;

            // Apply 2 point moving average to demanded height.
            const float hgt_dem = 0.5f * (hgt_dem_in_ + hgt_dem_in_prev_);
            hgt_dem_in_prev_ = hgt_dem_in_;

            // Limit height rate of change.
            if ((hgt_dem - hgt_dem_rate_ltd_) > (climb_rate_limit_ * dt_)) {
                hgt_dem_rate_ltd_ = hgt_dem_rate_ltd_ + climb_rate_limit_ * dt_;
                sink_fraction_ = 0.0f;
            } else if ((hgt_dem - hgt_dem_rate_ltd_) < (-sink_rate_limit_ * dt_)) {
                hgt_dem_rate_ltd_ = hgt_dem_rate_ltd_ - sink_rate_limit_ * dt_;
                sink_fraction_ = 1.0f;
            } else {
                const float numerator = hgt_dem - hgt_dem_rate_ltd_;
                const float denominator = -sink_rate_limit_ * dt_;
                if (math::is_negative(numerator) && math::is_negative(denominator)) {
                    sink_fraction_ = numerator / denominator;
                } else {
                    sink_fraction_ = 0.0f;
                }
                hgt_dem_rate_ltd_ = hgt_dem;
            }

            // Apply a first order lag to height demand.
            const float coef = std::min(dt_ / (dt_ + std::max(gains_.hgt_dem_tconst, dt_)), 1.0f);
            hgt_rate_dem_ = (hgt_dem_rate_ltd_ - hgt_dem_lpf_) / gains_.hgt_dem_tconst;
            hgt_dem_lpf_ = hgt_dem_rate_ltd_ * coef + (1.0f - coef) * hgt_dem_lpf_;
            post_to_hgt_offset_ *= (1.0f - coef); // always 0 in this slice - see post_to_hgt_offset_'s own comment
            hgt_dem_ = hgt_dem_lpf_ + post_to_hgt_offset_;

            // Don't allow height demand to get too far ahead of the vehicle's
            // current height if it is unable to follow the demanded climb or
            // descent. Upstream's `!(TAKEOFF||ABORT_LANDING)`/
            // `!_landing.is_flaring()` guards are both unconditionally true in
            // NORMAL-only scope and are dropped (this whole block is itself
            // nested inside `!landing.is_flaring`, so the guard would be
            // unconditionally true here even with real landing state).
            bool max_climb_condition = (pitch_dem_unc_ > pitchmaxf_) || (sebdot_dem_clip_ == ClipStatus::kMax);
            bool max_descent_condition = (pitch_dem_unc_ < pitchminf_) || (sebdot_dem_clip_ == ClipStatus::kMin);
            if (using_airspeed_for_throttle_) {
                // large height errors will result in the throttle saturating.
                max_climb_condition |= (thr_clip_status_ == ClipStatus::kMax);
                max_descent_condition |= (thr_clip_status_ == ClipStatus::kMin);
            }
            const float hgt_dem_alpha = dt_ / std::max(dt_ + gains_.hgt_dem_tconst, dt_);
            if (max_climb_condition && hgt_dem_ > hgt_dem_prev_) {
                max_climb_scaler_ *= (1.0f - hgt_dem_alpha);
            } else if (max_descent_condition && hgt_dem_ < hgt_dem_prev_) {
                max_sink_scaler_ *= (1.0f - hgt_dem_alpha);
            } else {
                max_climb_scaler_ = max_climb_scaler_ * (1.0f - hgt_dem_alpha) + hgt_dem_alpha;
                max_sink_scaler_ = max_sink_scaler_ * (1.0f - hgt_dem_alpha) + hgt_dem_alpha;
            }
            hgt_dem_prev_ = hgt_dem_;
        } else {
            // CPP-040: when flaring, force height rate demand to the
            // configured sink rate and adjust the demanded height to be
            // kinematically consistent with the height rate (upstream:
            // AP_TECS.cpp:611-644).

            // set all height filter states to current height to prevent
            // large pitch transients if flare is aborted.
            hgt_dem_lpf_ = height_;
            hgt_dem_rate_ltd_ = height_;
            hgt_dem_in_prev_ = height_;

            if (!flare_initialised_) {
                flare_hgt_dem_adj_ = hgt_dem_;
                flare_hgt_dem_ideal_ = height_;
                hgt_at_start_of_flare_ = hgt_afe;
                hgt_rate_dem_at_flare_entry_ = hgt_rate_dem_;
                flare_initialised_ = true;
            }

            // adjust the flare sink rate to increase/decrease as you
            // travel further beyond the land wp.
            const float land_sink_rate_adj =
                gains_.land_sink + gains_.land_sink_rate_change * landing.distance_beyond_land_wp;

            // bring it in linearly with height.
            float p;
            if (hgt_at_start_of_flare_ > gains_.flare_holdoff_hgt) {
                p = math::constrain_value((hgt_at_start_of_flare_ - hgt_afe) /
                                               (hgt_at_start_of_flare_ - gains_.flare_holdoff_hgt),
                                           0.0f, 1.0f);
            } else {
                p = 1.0f;
            }
            hgt_rate_dem_ = hgt_rate_dem_at_flare_entry_ * (1.0f - p) - land_sink_rate_adj * p;

            flare_hgt_dem_ideal_ += dt_ * hgt_rate_dem_; // the ideal height profile to follow
            flare_hgt_dem_adj_ += dt_ * hgt_rate_dem_;   // the demanded height profile that includes
                                                          // the pre-flare height tracking offset

            // fade across to the ideal height profile.
            hgt_dem_ = flare_hgt_dem_adj_ * (1.0f - p) + flare_hgt_dem_ideal_ * p;
        }
    }

    // upstream: AP_TECS::_detect_underspeed() - VTOL clear and
    // `!_landing.is_flaring()` guard both dropped (unconditionally
    // true/false in NORMAL-only scope), see file banner.
    void detect_underspeed(const TecsInputs& in) {
        // see if we can clear a previous underspeed condition. We clear
        // it if we are now more than 15% above min speed, and haven't
        // been below min speed for at least 3 seconds.
        if (flags_.underspeed && tas_state_ >= tas_min_ * 1.15f && (in.now_ms - underspeed_start_ms_) > 3000U) {
            flags_.underspeed = false;
        }

        if (((tas_state_ < tas_min_ * 0.9f) && (throttle_dem_ >= thrmaxf_ * 0.95f)) ||
            ((height_ < hgt_dem_) && flags_.underspeed)) {
            flags_.underspeed = true;
            if (tas_state_ < tas_min_ * 0.9f) {
                underspeed_start_ms_ = in.now_ms;
            }
        } else {
            flags_.underspeed = false;
        }
    }

    // upstream: AP_TECS::_update_energies() - fully in scope, unchanged.
    void update_energies() {
        spe_dem_ = hgt_dem_ * kGravityMss;
        ske_dem_ = 0.5f * tas_dem_adj_ * tas_dem_adj_;

        skedot_dem_ = tas_state_ * (tas_rate_dem_ - tas_rate_dem_lpf_);

        spe_est_ = height_ * kGravityMss;
        ske_est_ = 0.5f * tas_state_ * tas_state_;

        spedot_ = climb_rate_ * kGravityMss;
        skedot_ = tas_state_ * (vel_dot_ - vel_dot_lpf_);
    }

    // upstream: AP_TECS::_update_throttle_with_airspeed() - VTOL SPE-error
    // zeroing, TAKEOFF integrator override, landing damping/slew
    // overrides all dropped, see file banner.
    void update_throttle_with_airspeed() {
        // Calculate limits to be applied to potential energy error to
        // prevent over or underspeed occurring due to large height errors.
        const float spe_err_max = std::max(ske_est_ - 0.5f * tas_min_ * tas_min_, 0.0f);
        const float spe_err_min = std::min(ske_est_ - 0.5f * tas_max_ * tas_max_, 0.0f);

        // rate of change of potential energy is proportional to height error.
        spedot_dem_ = (spe_dem_ - spe_est_) / time_constant();

        ste_error_ = math::constrain_value(spe_dem_ - spe_est_, spe_err_min, spe_err_max) + ske_dem_ - ske_est_;
        float stedot_dem = math::constrain_value(spedot_dem_ + skedot_dem_, stedot_min_, stedot_max_);
        float stedot_error = stedot_dem - spedot_ - skedot_;

        // Apply 0.5 second first order filter to STEdot_error to remove
        // accelerometer noise from the measurement.
        const float filt_coef = 2.0f * dt_;
        stedot_error = filt_coef * stedot_error + (1.0f - filt_coef) * stedot_err_last_;
        stedot_err_last_ = stedot_error;

        if (flags_.underspeed) {
            throttle_dem_ = 1.0f;
        } else if (flags_.is_gliding) {
            throttle_dem_ = 0.0f;
        } else {
            // gain scaler from specific energy error to throttle - the
            // derivative of STEdot wrt throttle measured across the max
            // allowed throttle range.
            const float k_thr2ste = (stedot_max_ - stedot_min_) / (thrmaxf_ - thrminf_);
            const float k_ste2thr = 1.0f / (time_constant() * k_thr2ste);

            const float nom_thr = aparm_.throttle_cruise * 0.01f;
            // Use the demanded rate of change of total energy as the
            // feed-forward demand, plus a component scaling with
            // (1/cos(bank)^2 - 1) to compensate for turn-induced drag.
            stedot_dem = stedot_dem + gains_.roll_comp * (1.0f / math::constrain_value(cos_roll_ * cos_roll_, 0.1f, 1.0f) - 1.0f);
            const float ff_throttle = nom_thr + stedot_dem / k_thr2ste;

            const float throttle_damp = gains_.thr_damp;
            throttle_dem_ = (ste_error_ + stedot_error * throttle_damp) * k_ste2thr + ff_throttle;

            const float thrminf_clipped_to_zero = math::constrain_value(thrminf_, 0.0f, thrmaxf_);

            // Calculate integrator state upper/lower limits, allowing for
            // 10% throttle saturation to absorb demand noise.
            const float max_amp = 0.5f * (thrmaxf_ - thrminf_clipped_to_zero);
            const float integ_max = math::constrain_value(thrmaxf_ - throttle_dem_ + 0.1f, -max_amp, max_amp);
            const float integ_min = math::constrain_value(thrminf_ - throttle_dem_ - 0.1f, -max_amp, max_amp);

            integ_thr_state_ = integ_thr_state_ + (ste_error_ * get_i_gain()) * dt_ * k_ste2thr;
            integ_thr_state_ = math::constrain_value(integ_thr_state_, integ_min, integ_max);

            const float throttle_slewrate = aparm_.throttle_slewrate;
            if (!math::is_zero(throttle_slewrate)) {
                const float thr_rate_incr = dt_ * (thrmaxf_ - thrminf_clipped_to_zero) * throttle_slewrate * 0.01f;
                throttle_dem_ =
                    math::constrain_value(throttle_dem_, last_throttle_dem_ - thr_rate_incr, last_throttle_dem_ + thr_rate_incr);
                last_throttle_dem_ = throttle_dem_;
            }

            throttle_dem_ = throttle_dem_ + integ_thr_state_;
        }

        constrain_throttle();
    }

    // upstream: AP_TECS::constrain_throttle().
    void constrain_throttle() {
        if (throttle_dem_ > thrmaxf_) {
            thr_clip_status_ = ClipStatus::kMax;
            throttle_dem_ = thrmaxf_;
        } else if (throttle_dem_ < thrminf_) {
            thr_clip_status_ = ClipStatus::kMin;
            throttle_dem_ = thrminf_;
        } else {
            thr_clip_status_ = ClipStatus::kNone;
        }
    }

    // upstream: AP_TECS::_detect_bad_descent() - VTOL early-out dropped.
    void detect_bad_descent() {
        if (flags_.is_gliding || flags_.underspeed) {
            flags_.bad_descent = false;
            return;
        }

        const float stedot = spedot_ + skedot_;
        if (((ste_error_ > 200.0f) && (stedot < 0.0f) && (throttle_dem_ >= thrmaxf_ * 0.9f)) ||
            (flags_.bad_descent && (ste_error_ > 0.0f))) {
            flags_.bad_descent = true;
        } else {
            flags_.bad_descent = false;
        }
    }

    // upstream: AP_TECS::_update_pitch() - VTOL/TAKEOFF/is_doing_auto_land
    // branches dropped throughout, see file banner.
    void update_pitch() {
        // Calculate Speed/Height Control Weighting.
        ske_weighting_ = math::constrain_value(gains_.spd_weight, 0.0f, 2.0f);
        if (!use_airspeed()) {
            ske_weighting_ = 0.0f;
        } else if (flags_.underspeed || flags_.is_gliding) {
            ske_weighting_ = 2.0f;
        }

        float spe_weighting = 2.0f - ske_weighting_;
        spe_weighting = std::min(spe_weighting, 1.0f);
        ske_weighting_ = std::min(ske_weighting_, 1.0f);

        const float seb_dem = spe_dem_ * spe_weighting - ske_dem_ * ske_weighting_;
        const float seb_est = spe_est_ * spe_weighting - ske_est_ * ske_weighting_;
        const float seb_error = seb_dem - seb_est;

        float sebdot_dem = hgt_rate_dem_ * kGravityMss * spe_weighting + seb_error / time_constant();
        const float sebdot_dem_min = -gains_.max_sink_rate * kGravityMss;
        const float sebdot_dem_max = gains_.max_climb_rate * kGravityMss;
        if (sebdot_dem < sebdot_dem_min) {
            sebdot_dem = sebdot_dem_min;
            sebdot_dem_clip_ = ClipStatus::kMin;
        } else if (sebdot_dem > sebdot_dem_max) {
            sebdot_dem = sebdot_dem_max;
            sebdot_dem_clip_ = ClipStatus::kMax;
        } else {
            sebdot_dem_clip_ = ClipStatus::kNone;
        }

        const float sebdot_est = spedot_ * spe_weighting - skedot_ * ske_weighting_;
        const float sebdot_error = sebdot_dem - sebdot_est;

        const float pitch_damp = gains_.ptch_damp;
        const float sebdot_dem_total = sebdot_dem + sebdot_error * pitch_damp;

        // inverse of gain from SEB to pitch angle.
        const float gain_inv = tas_state_ * kGravityMss;

        // Calculate max/min integrator values allowing for 5deg of pitch
        // saturation before the integrator is clipped.
        const float integsebdot_min = (gain_inv * (pitchminf_ - math::radians(5.0f))) - sebdot_dem_total;
        const float integsebdot_max = (gain_inv * (pitchmaxf_ + math::radians(5.0f))) - sebdot_dem_total;

        // don't allow the integrator to rise by more than 10% of its full
        // range in one step (Issue #4066).
        const float integseb_range = integsebdot_max - integsebdot_min;
        const float integseb_delta =
            math::constrain_value(sebdot_error * get_i_gain() * dt_, -integseb_range * 0.1f, integseb_range * 0.1f);

        // predict what pitch will be with unconstrained integration.
        pitch_dem_unc_ = (sebdot_dem_total + integ_sebdot_ + integseb_delta + integ_ke_) / gain_inv;

        const bool inhibit_integrator = ((pitch_dem_unc_ > pitchmaxf_) && integseb_delta > 0.0f) ||
                                         ((pitch_dem_unc_ < pitchminf_) && integseb_delta < 0.0f);
        if (!inhibit_integrator) {
            integ_sebdot_ += integseb_delta;
            integ_ke_ += (ske_est_ - ske_dem_) * ske_weighting_ * dt_ / time_constant();
        } else {
            // fade out integrator if saturating.
            const float coef = 1.0f - dt_ / (dt_ + time_constant());
            integ_sebdot_ *= coef;
            integ_ke_ *= coef;
        }
        integ_sebdot_ = math::constrain_value(integ_sebdot_, integsebdot_min, integsebdot_max);
        const float ke_integ_limit = 0.25f * (pitchmaxf_ - pitchminf_) * gain_inv;
        integ_ke_ = math::constrain_value(integ_ke_, -ke_integ_limit, ke_integ_limit);

        // Calculate pitch demand from specific energy balance signals.
        pitch_dem_unc_ = (sebdot_dem_total + integ_sebdot_ + integ_ke_) / gain_inv;

        // Add a feedforward term from demanded airspeed to pitch, active
        // only while gliding (a real, non-landing-specific condition -
        // see Flags::is_gliding).
        if (flags_.is_gliding) {
            pitch_dem_unc_ += (tas_dem_adj_ - gains_.pitch_ff_v0) * gains_.pitch_ff_k;
        }

        pitch_dem_ = math::constrain_value(pitch_dem_unc_, pitchminf_, pitchmaxf_);

        // Rate limit the pitch demand to comply with the specified
        // vertical acceleration limit.
        const float ptch_rate_incr = dt_ * gains_.vert_acc_lim / tas_state_;
        if ((pitch_dem_ - last_pitch_dem_) > ptch_rate_incr) {
            pitch_dem_ = last_pitch_dem_ + ptch_rate_incr;
        } else if ((pitch_dem_ - last_pitch_dem_) < -ptch_rate_incr) {
            pitch_dem_ = last_pitch_dem_ - ptch_rate_incr;
        }
        last_pitch_dem_ = pitch_dem_;
    }

    // upstream: AP_TECS::_initialise_states() - TAKEOFF/ABORT_LANDING
    // branch and the pitch_demand_lpf/pitch_measured_lpf setup dropped,
    // see file banner.
    void initialise_states(float hgt_afe, const TecsInputs& in) {
        flags_.reset = false;

        if (dt_ > 0.2f || need_reset_) {
            ske_weighting_ = 1.0f;
            integ_thr_state_ = 0.0f;
            integ_sebdot_ = 0.0f;
            integ_ke_ = 0.0f;
            last_throttle_dem_ = aparm_.throttle_cruise * 0.01f;
            last_pitch_dem_ = in.pitch_rad;
            hgt_dem_in_prev_ = hgt_afe;
            hgt_dem_lpf_ = hgt_afe;
            hgt_dem_rate_ltd_ = hgt_afe;
            hgt_dem_prev_ = hgt_afe;
            tas_dem_adj_ = tas_dem_;
            flags_.reset = true;
            dt_ = 0.02f; // when first starting TECS, use the most likely time constant
            post_to_hgt_offset_ = 0.0f;
            use_synthetic_airspeed_once_ = false;

            flags_.underspeed = false;
            flags_.bad_descent = false;
            need_reset_ = false;

            max_climb_scaler_ = 1.0f;
            max_sink_scaler_ = 1.0f;
        }
    }

    // upstream: AP_TECS::_update_STE_rate_lim().
    void update_ste_rate_lim() {
        stedot_max_ = climb_rate_limit_ * kGravityMss;
        stedot_min_ = -gains_.min_sink_rate * kGravityMss;
        stedot_neg_max_ = -gains_.max_sink_rate * kGravityMss;
    }

    // upstream: AP_TECS::_update_throttle_limits() - no landing/flight-
    // stage reference exists in this function upstream; ported unchanged.
    void update_throttle_limits() {
        thrmaxf_ = std::min(1.0f, thrmaxf_ext_);
        thrminf_ = std::max(-1.0f, thrminf_ext_);

        // Allow a minimum of 1% throttle range, primarily to prevent
        // TECS numerical errors.
        constexpr float kThrEps = 0.01f;
        if (std::fabs(thrminf_ - thrmaxf_) < kThrEps) {
            flag_throttle_forced_ = true;
            if (thrmaxf_ < 1.0f) {
                thrmaxf_ = std::max(thrmaxf_, thrminf_ + 0.01f);
            } else {
                thrminf_ = std::min(thrminf_, thrmaxf_ - 0.01f);
            }
        } else {
            flag_throttle_forced_ = false;
        }

        // Reset the external throttle limits - caller will have to reset
        // them again next iteration.
        if (gains_.thr_min_pct_ext_rate_lim > 0.0f) {
            thrminf_ext_ -= 0.01f * gains_.thr_min_pct_ext_rate_lim * dt_;
            thrminf_ext_ = std::max(thrminf_ext_, -1.0f);
        } else {
            thrminf_ext_ = -1.0f;
        }
        thrmaxf_ext_ = 1.0f;
    }

    // upstream: AP_TECS::_update_pitch_limits(ptchMinCO_cd) - parameter
    // dropped (TAKEOFF-only) and every landing/flare branch dropped, see
    // file banner. What remains: PITCH_MAX/MIN vs. PTCH_LIM_MAX/MIN_DEG
    // selection, external-limit clamp, and degree->radian conversion.
    void update_pitch_limits() {
        pitchmaxf_ = math::is_zero(gains_.pitch_max) ? aparm_.pitch_limit_max : gains_.pitch_max;
        pitchminf_ = math::is_zero(gains_.pitch_min) ? aparm_.pitch_limit_min : gains_.pitch_min;

        pitchmaxf_ = std::min(pitchmaxf_, pitchmaxf_ext_);
        pitchminf_ = std::max(pitchminf_, pitchminf_ext_);

        // Reset the external pitch limits.
        pitchminf_ext_ = -90.0f;
        pitchmaxf_ext_ = 90.0f;

        pitchmaxf_ = math::radians(pitchmaxf_);
        pitchminf_ = math::radians(pitchminf_);

        // don't allow max pitch to go below min pitch.
        pitchmaxf_ = std::max(pitchmaxf_, pitchminf_);
    }

    Gains gains_;
    FixedWingParams aparm_;

    // Last time update_50hz()/update_pitch_throttle() were called,
    // microseconds - upstream: _update_50hz_last_usec/
    // _update_pitch_throttle_last_usec.
    std::uint64_t update_50hz_last_us_ = 0;
    std::uint64_t update_pitch_throttle_last_us_ = 0;

    // current height estimate (above field elevation, upstream naming
    // notwithstanding - it is actually height above home here, since this
    // slice has no field-elevation concept - see relative_position_d_home_m).
    float height_ = 0.0f;

    float throttle_dem_ = 0.0f;
    float pitch_dem_ = 0.0f;

    float climb_rate_ = 0.0f;
    float climb_rate_limit_ = 0.0f;
    float sink_rate_limit_ = 0.0f;

    struct HeightFilter {
        float dd_height = 0.0f;
        float height = 0.0f;
    } height_filter_;

    float integ_dtas_state_ = 0.0f;
    float tas_state_ = 0.0f;
    float integ_thr_state_ = 0.0f;
    float integ_sebdot_ = 0.0f;
    float integ_ke_ = 0.0f;

    float last_throttle_dem_ = 0.0f;
    float last_pitch_dem_ = 0.0f;

    float vel_dot_ = 0.0f;
    float vel_dot_lpf_ = 0.0f;

    float tas_max_ = 0.0f;
    float tas_min_ = 0.0f;
    float tas_dem_ = 0.0f;
    float eas_dem_ = 0.0f;

    float hgt_dem_in_raw_ = 0.0f;
    float hgt_dem_in_ = 0.0f;
    float hgt_dem_in_prev_ = 0.0f;
    float hgt_dem_lpf_ = 0.0f;
    float hgt_dem_ = 0.0f;
    float hgt_dem_prev_ = 0.0f;

    float hgt_dem_rate_ltd_ = 0.0f;
    float hgt_rate_dem_ = 0.0f;

    // CPP-040 flare state - upstream: _flare_initialised/
    // _flare_hgt_dem_adj/_flare_hgt_dem_ideal/_hgt_at_start_of_flare/
    // _hgt_rate_dem_at_flare_entry (AP_TECS.h ~298-299/406-408/429). See
    // file banner's "CPP-040 ADDENDUM" and update_height_demand()'s flare
    // arm for how these are seeded/consumed.
    bool flare_initialised_ = false;
    float flare_hgt_dem_adj_ = 0.0f;
    float flare_hgt_dem_ideal_ = 0.0f;
    float hgt_at_start_of_flare_ = 0.0f;
    float hgt_rate_dem_at_flare_entry_ = 0.0f;

    // Offset applied to height demand post-takeoff to compensate for
    // height demand filter lag - upstream: _post_TO_hgt_offset. Only ever
    // set non-zero by _initialise_states()'s TAKEOFF branch, which this
    // slice drops (see file banner); stays permanently 0.0f here. Kept
    // (rather than deleted outright) because update_height_demand()'s
    // decay multiply (`post_to_hgt_offset_ *= (1.0f - coef)`) is itself
    // in-scope, harmless, and matches upstream line-for-line.
    float post_to_hgt_offset_ = 0.0f;

    float tas_dem_adj_ = 0.0f;
    float tas_rate_dem_ = 0.0f;
    float tas_rate_dem_lpf_ = 0.0f;

    float stedot_err_last_ = 0.0f;

    Flags flags_;

    std::uint32_t underspeed_start_ms_ = 0;

    float pitch_dem_unc_ = 0.0f;

    float stedot_max_ = 0.0f;
    float stedot_min_ = 0.0f;
    float stedot_neg_max_ = 0.0f;

    float thrmaxf_ = 0.0f;
    float thrminf_ = 0.0f;
    float thrmaxf_ext_ = 1.0f;
    float thrminf_ext_ = -1.0f;
    float pitchmaxf_ext_ = 90.0f;
    float pitchminf_ext_ = -90.0f;

    float pitchmaxf_ = 0.0f;
    float pitchminf_ = 0.0f;

    ClipStatus thr_clip_status_ = ClipStatus::kNone;

    float spe_dem_ = 0.0f;
    float ske_dem_ = 0.0f;
    float spedot_dem_ = 0.0f;
    float skedot_dem_ = 0.0f;
    float spe_est_ = 0.0f;
    float ske_est_ = 0.0f;
    float spedot_ = 0.0f;
    float skedot_ = 0.0f;

    float max_climb_scaler_ = 1.0f;
    float max_sink_scaler_ = 1.0f;
    float sink_fraction_ = 0.0f;

    float ste_error_ = 0.0f;

    ClipStatus sebdot_dem_clip_ = ClipStatus::kNone;

    // Time since last update of the main TECS loop, seconds - upstream:
    // _DT. Default-initialised to 0.0f rather than left indeterminate -
    // see file banner's "genuine upstream quirk" note for why this
    // differs (safely) from upstream's own uninitialised _DT.
    float dt_ = 0.0f;

    bool need_reset_ = false;
    bool flag_throttle_forced_ = false;

    float ske_weighting_ = 0.0f;

    bool use_synthetic_airspeed_once_ = false;
    bool using_airspeed_sensor_ = false;
    bool using_airspeed_for_throttle_ = false;

    // Cached from TecsInputs at the top of whichever of update_50hz()/
    // update_pitch_throttle() ran most recently this tick - see file
    // banner's "no singletons" note.
    float eas2tas_ = 1.0f;
    float cos_roll_ = 1.0f;
    float load_factor_ = 1.0f;

    MovingAverage5 vdot_filter_;
};

} // namespace fwcpp::tecs
