#pragma once

// Port of AP_NavEKF3's state vector, strapdown INS mechanization, and
// covariance time-propagation. CPP-052, PHASE 1 of a large multi-phase
// epic - see this file's own "OUT OF SCOPE" section below before assuming
// anything not listed here exists.
//
// Upstream: AP_NavEKF3/AP_NavEKF3_core.h (state_elements struct, ~line
// 566; Matrix24/Vector24 typedefs, ~line 531-558) and
// AP_NavEKF3_core.cpp (NavEKF3_core::UpdateStrapdownEquationsNED(), ~line
// 743; NavEKF3_core::CovariancePrediction(), ~line 1008;
// NavEKF3_core::CovarianceInit(), ~line 573; NavEKF3_core::ConstrainStates(),
// ~line 2016; NavEKF3_core::ConstrainVariances(), ~line 1877;
// NavEKF3_core::calcEarthRateNED(), ~line 2066; NavEKF3_core::
// InitialGyroBiasUncertainty(), AP_NavEKF3_GyroBias.cpp ~line 20) - all
// read directly from the pinned Plane-4.7.0 upstream worktree.
//
// ============================================================================
// SCALE WARNING (reproduced from ticket CPP-052 - read before extending
// this file): upstream's real AP_NavEKF3 is ~17,000 lines: a genuine
// 24-state extended Kalman filter with multi-IMU/multi-lane management,
// GPS/baro/compass/optical-flow/rangefinder fusion, innovation-consistency
// gating, and a Gaussian Sum Filter yaw estimator. This file is
// deliberately NOT that. It is the first of a multi-phase epic.
// ============================================================================
//
// WHAT THIS PHASE BUILDS:
//   - The real 24-element state vector (state_elements, upstream
//     AP_NavEKF3_core.h:566-575), all 24 conceptual slots present.
//   - The real 24x24 covariance matrix P: upstream's traced (not invented)
//     initial values (CovarianceInit()) and its real time-propagation
//     (CovariancePrediction() - the process-noise Jacobian for the
//     quaternion/velocity/position/gyro-bias/accel-bias block is
//     transcribed verbatim from upstream's auto-generated symbolic
//     expressions, see ekf_core.cpp's own banner).
//   - UpdateStrapdownEquationsNED()'s real mechanization: quaternion
//     update from bias-corrected delta-angle (with earth-rotation-rate
//     compensation), trapezoidal velocity/position integration from
//     bias-corrected, gravity-compensated delta-velocity.
//
// WHAT THIS PHASE DOES NOT BUILD (named per-file, matching the ticket's
// acceptance criterion - this is the real roadmap for phase 2+, not a
// commitment to any particular next order):
//   - AP_NavEKF3_PosVelFusion.cpp - GPS/baro/external-nav position and
//     velocity fusion (~2073 lines).
//   - AP_NavEKF3_MagFusion.cpp - 3-axis and yaw-only magnetometer fusion,
//     declination fusion (~1544 lines).
//   - AP_NavEKF3_OptFlowFusion.cpp - optical flow fusion.
//   - AP_NavEKF3_RngBcnFusion.cpp - range beacon fusion.
//   - AP_NavEKF3_Measurements.cpp - multi-IMU/multi-sensor sample
//     buffering, the delayed fusion time horizon, and inactiveBias[]
//     per-IMU bias bookkeeping (~1555 lines). This port has one IMU and no
//     delay buffer - see SIMPLIFICATION notes below.
//   - AP_NavEKF3.cpp - multi-core/multi-lane management, lane switching,
//     core arbitration (~2173 lines).
//   - AP_NavEKF3_Control.cpp - health/mode logic, inhibitMagStates/
//     inhibitWindStates/inhibitDelAngBiasStates/inhibitDelVelBiasStates
//     control-flow, badIMUdata vibration detection, onGround/dvelBiasAxis
//     ground-alignment gating. Phase 1 hardcodes the permanent-no-fusion
//     equivalents of these flags - see SIMPLIFICATION notes below.
//   - The GSF (Gaussian Sum Filter) yaw estimator.
//   - AP_NavEKF3_Outputs.cpp - output-buffer/complementary-filter
//     blending between the fusion time horizon and "now".
//   - Innovation gating / consistency checks (no fusion exists to gate).
//   - Wiring this estimator into Plane as an alternative to AhrsDcm - a
//     separate, later integration decision once fusion exists.
//
// SIMPLIFICATIONS MADE IN THIS PHASE (each is a real, documented
// divergence from upstream behavior, not a bug - "port fixes bugs, not
// upstream" / "register every divergence"):
//
//   1. inhibitMagStates and inhibitWindStates are permanently TRUE (no
//      compass/airspeed fusion exists in this phase to ever clear them -
//      upstream sets these in AP_NavEKF3_Control.cpp, out of scope). Per
//      upstream's OWN logic for this condition (ConstrainVariances(),
//      ~line 1970-1994), this means P[16..21] (mag) and P[22..23] (wind)
//      are zeroed every predict cycle - covariance_init() sets their real
//      upstream initial values once, and the very first covariance_
//      prediction() call zeros them again, exactly reproducing upstream's
//      real behavior for a configuration with no magnetometer/airspeed
//      fusion enabled. This is NOT a shortcut - computing their would-be
//      correlation growth with the 0-9 block would be wasted work upstream
//      itself throws away in this condition.
//
//   2. inhibitDelAngBiasStates and inhibitDelVelBiasStates are permanently
//      FALSE (gyro/accel bias states keep accumulating real process-noise
//      uncertainty every cycle, matching normal in-flight operation) - but
//      the per-axis ground-alignment observability gate
//      (dvelBiasAxisInhibit[], tied to onGround/prevTnb, CovariancePrediction
//      ~line 158-173) is NOT reproduced (it depends on an onGround flag
//      this port's EkfCore has no source for in phase 1). Accel bias
//      variance can therefore grow slightly more freely on the ground than
//      upstream's real ground-alignment gate would allow - harmless for
//      the pure free-integrating-INS behavior this phase targets, and
//      exactly the kind of thing phase 2 (which will need onGround/mode
//      context anyway for real fusion gating) should revisit.
//
//   3. badIMUdata (vibration detection, AP_NavEKF3_Control.cpp) is
//      permanently FALSE - no vibration-detection subsystem in this port.
//      Accel process noise always uses the normal acc_noise parameter,
//      never the elevated BAD_IMU_DATA_ACC_P_NSE fallback.
//
//   4. Single-IMU bias correction: upstream's correctDeltaAngle()/
//      correctDeltaVelocity() (AP_NavEKF3_core.cpp ~line 726) subtract
//      inactiveBias[gyro_index]/[accel_index] - a per-IMU bias bookkeeping
//      array that exists to let multiple simultaneous IMU instances share
//      one set of EKF bias *states* while independently tracking their own
//      raw sensor offsets (AP_NavEKF3_Measurements.cpp, out of scope: no
//      multi-IMU affinity/blending in this port). With one IMU, upstream's
//      inactiveBias[active_index] IS this filter's own stateStruct.
//      gyro_bias/accel_bias - so update_strapdown_equations_ned()
//      subtracts state.gyro_bias/state.accel_bias directly. Bit-identical
//      to upstream's real single-IMU behavior, not an approximation of it.
//
//   5. No fusion time-horizon delay buffer (AP_NavEKF3_Measurements.cpp's
//      storedIMU FIFO, ~150ms typical). Upstream runs strapdown mechanics
//      on a delayed IMU sample so it lines up with delayed GPS/baro/mag
//      measurements at fusion time; with no fusion in this phase there is
//      no other measurement stream to align with, so "the current sample"
//      IS "the delayed sample" trivially. Phase 2 (any real fusion type)
//      will need to reintroduce this buffer.
//
//   6. dtEkfAvg (upstream: a running low-pass-filtered average IMU
//      timestep, itself computed in AP_NavEKF3_Measurements.cpp from the
//      downsampled IMU rate, out of scope) is an explicit caller-supplied
//      input here (ADR-0012: no singleton AP::ins().get_loop_rate_hz()
//      read). This mirrors AhrsDcm's own explicit-input pattern (this
//      class does no polling/averaging of its own; the caller - whoever
//      owns the real IMU driver - is expected to maintain it, e.g. as a
//      simple exponential filter of consecutive dt samples, matching
//      upstream's own `dtEkfAvg = 0.98f * dtEkfAvg + 0.02f * dtNow`).
//
//   7. Earth-rotation-rate compensation (calcEarthRateNED(), upstream
//      home-latitude-dependent) is an explicit `earth_rate_ned` field the
//      caller sets via calc_earth_rate_ned(latitude_rad) once a home
//      location is known (ADR-0012: no AP::ahrs().get_home() singleton
//      read). Defaults to zero (no compensation) until the caller sets
//      it - matching upstream behavior before a home location exists.
//
//   8. MagTableConstrain() (World Magnetic Model table lookup,
//      ConstrainStates()'s alternate earth-magfield clamp path) is not
//      ported - this phase always takes upstream's simpler unconditional
//      +-1 Gauss clamp branch (have_table_earth_field is always false
//      here, matching a fresh port with no WMM table wired in).
//
//   9. Terrain state / rangefinder-ground clamp (ConstrainStates()'s final
//      `if (!inhibitGndState) terrainState = MAX(...)` line) is excluded
//      entirely - terrainState is a separate scalar outside the 24-element
//      state_elements vector and belongs with AP_NavEKF3_RngBcnFusion.cpp/
//      terrain-following, out of scope here.
//
// NO SINGLETONS, EXPLICIT INPUTS INSTEAD (ADR-0012), matching AhrsDcm's
// GyroSample/AccelSample pattern exactly (see ap-ahrs/ahrs_dcm.hpp):
//   - GyroSample/AccelSample REPLACE AP::ins() reads.
//   - earth_rate_ned REPLACES AP::ahrs().get_home()-derived latitude.
//   - dt_ekf_avg is a per-call parameter, not a polled loop rate.
//   - Noise/limit parameters (gyr_noise, acc_noise, gps_horiz_vel_noise,
//     etc.) are plain public fields defaulted to the real upstream
//     Plane-4.7.0 parameter defaults (AP_NavEKF3.cpp's APM_BUILD_ArduPlane
//     block) - same "AP_Param not wired in yet" treatment as AhrsDcm's
//     kp_yaw.
//
// ftype: mirrors upstream's AP_Math/ftype.h HAL_WITH_EKF_DOUBLE switch via
// this port's own FWCPP_EKF_DOUBLE CMake option (top-level CMakeLists.txt,
// already declared - this ticket is its first consumer). Kept LOCAL to
// this module (fwcpp::ekf::ftype), not hoisted into ap-math alongside
// postype_t, deliberately: five parallel agents are editing disjoint
// modules this round, and ap-math is not part of this ticket's scope.
// Phase 2+ may want to hoist it into ap-math as a shared primitive
// (upstream's own ftype.h is shared AP_Math, after all) once that can be
// coordinated outside a multi-agent parallel session.
//
// ============================================================================
// CPP-056, PHASE 2 (this ticket): the first real measurement update.
// Everything above this point is phase 1 (CPP-052), unmodified. Read
// FuseVelPosNED() in full (AP_NavEKF3_PosVelFusion.cpp ~line 694-1181)
// before extending anything below - it was read in full for this phase,
// not skimmed from the ticket's own summary of it.
// ============================================================================
//
// WHAT THIS PHASE BUILDS:
//   - fuse_direct_state_observation(): the reusable "fuse one direct
//     observation of a single state element" primitive, transcribed
//     line-by-line from FuseVelPosNED()'s obsIndex loop body (~line
//     1024-1163) for the case that matters for GPS vel/pos: real GPS-
//     derived aiding, i.e. upstream's PV_AidingMode == AID_ABSOLUTE (see
//     "NOT PORTED FROM FuseVelPosNED" below for exactly what that
//     excludes and why it's the right cut).
//   - fuse_gps_velocity() / fuse_gps_position(): the GPS-specific
//     obsIndex 0-2 / 3-4 callers, including the real R_OBS "no reported
//     accuracy" noise-variance formula.
//   - GpsSample: a new, EKF-specific explicit input struct (see below for
//     why ahrs::GpsSample, used by AhrsDcm, is NOT reused as-is).
//   - Four real inhibitDelAngBiasStates/inhibitDelVelBiasStates/
//     inhibitMagStates/inhibitWindStates bool fields (see "CORRECTION TO
//     THE CPP-056 TICKET'S OWN PREMISE" below).
//
// CORRECTION TO THE CPP-056 TICKET'S OWN PREMISE: the ticket asserts
// phase 1 "already has real inhibitDelAngBiasStates/
// inhibitDelVelBiasStates/inhibitMagStates/inhibitWindStates fields".
// Verified directly against the phase-1 source (ekf_core.cpp) before
// writing any phase-2 code: this is FALSE. Phase 1 has no such fields -
// the equivalent permanently-true/permanently-false behavior is baked
// directly into constrain_variances()'s and covariance_prediction()'s
// control flow (unconditional branches, described only in prose
// comments). Since fuse_direct_state_observation() genuinely needs these
// as real, gateable fields (upstream's own Kfusion computation branches
// on them per-call, and a future phase may want to flip inhibitMagStates
// once mag fusion exists), this phase adds them as real public bool
// fields below, defaulted to reproduce phase 1's existing hardcoded
// behavior exactly (inhibit_del_ang_bias_states=false,
// inhibit_del_vel_bias_states=false, inhibit_mag_states=true,
// inhibit_wind_states=true) - a behavior-preserving correction, not a
// new simplification. constrain_variances()/covariance_prediction()
// themselves are NOT modified to read these fields dynamically (that
// would mean touching the delicate, independently-verified transcribed
// Jacobian block in ekf_core.cpp for no behavioral gain at today's fixed
// defaults) - a named gap: a future phase that actually wants to flip
// inhibit_mag_states at runtime must also wire it into those two
// functions's currently-hardcoded branches.
//
// CPP-065 UPDATE (phase 11): this gap is now CLOSED - covariance_prediction()
// and constrain_variances() are wired to read inhibit_mag_states/
// inhibit_wind_states at runtime (see those two functions' own "CPP-065
// phase 11" banners in ekf_core.cpp). inhibit_del_ang_bias_states/
// inhibit_del_vel_bias_states remain NOT wired into covariance_prediction()'s
// column-10..15 handling (that loop still runs unconditionally, matching
// phase 1's original hardcoded behavior) - a real, narrower remaining gap,
// out of CPP-065's scope, harmless today since neither flag is ever set true
// anywhere in this port.
//
// NOT PORTED FROM FuseVelPosNED() (named per the ticket's own acceptance
// criterion - each is a real upstream mechanism, verified present in the
// ~694-1181 read, deliberately left out of fuse_direct_state_observation
// because it belongs to a mode/feature this port doesn't have yet):
//   - The `PV_AidingMode == AID_NONE` "poorObservability" gyro-bias-
//     Kalman-gain gate and "horizInhibit" delta-velocity-bias gate
//     (~line 1051-1084). Both are guarded by `PV_AidingMode == AID_NONE`
//     (upstream's fake-position/fake-velocity "hold attitude without
//     real aiding" mode) and are UNCONDITIONALLY SKIPPED whenever
//     PV_AidingMode == AID_ABSOLUTE, which is the real mode real GPS
//     fusion runs under - so omitting them here is not an approximation,
//     it's the literal AID_ABSOLUTE-mode behavior of upstream's own
//     code, verified by reading the guard conditions directly.
//   - `dvelBiasAxisInhibit[]` (per-axis delta-velocity-bias inhibit tied
//     to onGround/ground-alignment, AP_NavEKF3_Control.cpp) - already a
//     named phase-1 gap (see simplification 2 above); this phase's
//     inhibit_del_vel_bias_states gate is all-or-nothing across x/y/z,
//     same as phase 1.
//   - `badIMUdata` gating of the delta-velocity-bias Kalman gain - always
//     false, per phase-1 simplification 3.
//   - `treatWindStatesAsTruth` narrowing the wind-state Kalman gain gate
//     (`!inhibitWindStates && !treatWindStatesAsTruth`) - no such field
//     exists in this port (no optical-flow/const-position-hold subsystem
//     that would ever set it); moot in this phase anyway since
//     inhibit_wind_states is permanently true.
//   - `gpsNoiseScaler` (satellite-count-based R_OBS scaling, applied
//     inside the obsIndex loop at ~line 1027/1032) - named in the ticket
//     as part of SelectVelPosFusion()'s orchestration, out of scope here.
//   - The innovation-consistency test-ratio gates (velTestRatio/
//     posTestRatio, ~line 811-928) that decide fuseVelData/fusePosData
//     in the first place, and faultStatus.bad_* bookkeeping - only the
//     healthyFusion negative-variance guard is in scope (see ticket).
//     fuse_gps_velocity()/fuse_gps_position() therefore have NO way to
//     reject a bad GPS fix beyond that one guard - a REAL, NAMED GAP:
//     upstream's real innovation gating is what lets the filter refuse a
//     glitching GPS fix rather than being corrupted by it, and this
//     phase cannot do that yet.
//   - The GPS-reported-accuracy branch (`gpsSpdAccuracy > 0.0f` /
//     `gpsPosAccuracy > 0.0f`, ~line 736-756) and the ExtNav branches -
//     only the "no reported accuracy" R_OBS formula is ported (see
//     GpsSample's own comment for why the struct has no accuracy
//     fields).
//   - The delay-buffer / observation-time-horizon machinery
//     (AP_NavEKF3_Measurements.cpp's storedGPS/imuDataDelayed ring
//     buffers) that lets upstream fuse a GPS sample against the state as
//     of the time that sample was actually valid. fuse_gps_velocity()/
//     fuse_gps_position() fuse a GpsSample against the CURRENT state,
//     assuming it is already time-aligned - a real, disclosed
//     simplification, not upstream's real behavior. Any future use of
//     this against genuinely asynchronous sensor timing needs this
//     buffer reintroduced first.
//   - Height/baro fusion (obsIndex==5, selectHeightForFusion()) - no
//     baro sensor model exists in this port.
//   - ResetVelocity()/ResetPosition()/initial alignment, and
//     SelectVelPosFusion()'s full orchestration (timeout detection,
//     glitch-radius handling, the GPS-vertical-velocity-vs-baro aliasing
//     cross-check) - this phase calls the fusion functions directly with
//     an already-known-good GpsSample; it does not decide WHEN to call
//     them or what to do on a stale/bad fix.
//
// GpsSample vs. ahrs::GpsSample (ADR-0012 explicit-input check, per the
// ticket): read ap-ahrs/ahrs_dcm.hpp's GpsSample directly before adding a
// new struct. It carries ground_speed_ms/ground_course_deg/velocity_ned/
// num_sats/has_fix/has_3d_fix - built for AhrsDcm's drift-correction use,
// which never fuses absolute position. It has NO position field at all
// (DCM has no position state to correct). This phase's GPS position
// fusion needs a local-NE position observation, which ahrs::GpsSample
// cannot represent - so it is not "reusable as-is", and not a subset/
// superset relationship either (ahrs::GpsSample's ground_speed_ms/
// ground_course_deg polar form vs. this phase's NED velocity vector
// don't correspond 1:1). This phase therefore defines its own
// fwcpp::ekf::GpsSample below rather than extending or wrapping
// ahrs::GpsSample - a genuinely different explicit input, not a
// duplicate of an existing one.
//
// ============================================================================
// CPP-057, PHASE 3 (this ticket): innovation-consistency gating for GPS
// velocity/position fusion. Everything above this point is phase 1
// (CPP-052) plus phase 2 (CPP-056), unmodified. Read FuseVelPosNED()'s
// innovation-gating block in full (AP_NavEKF3_PosVelFusion.cpp ~line
// 792-932) before extending anything below - it was read in full for
// this phase, not skimmed from the ticket's own summary of it.
// ============================================================================
//
// THE GAP THIS PHASE CLOSES: phase 2's own banner above named it
// explicitly - fuse_gps_velocity()/fuse_gps_position() fuse every GPS
// sample unconditionally, with no way to reject a bad/glitching fix
// beyond fuse_direct_state_observation()'s negative-variance guard (which
// only catches a numerically-corrupting update, not a merely-wrong one).
// This phase adds the real upstream normalized-innovation ("test ratio")
// check that decides upstream's own fusePosData/fuseVelData booleans, and
// wires it in as a pre-check that skips the ENTIRE velocity or position
// vector (all axes) for one cycle when it fails - upstream's own
// `else { fusePosData = false; }` / `else { fuseVelData = false; }`
// branches (~line 859, ~line 928).
//
// WHAT THIS PHASE BUILDS:
//   - gps_vel_test_ratio() / gps_pos_test_ratio(): the real velTestRatio/
//     posTestRatio formulas (~line 875-901 velocity, ~line 806-816
//     position), public (like the existing gps_*_obs_variance()
//     functions) so tests can verify the exact formula independently.
//   - Wiring into fuse_gps_velocity()/fuse_gps_position(): each now
//     computes its test ratio ONCE, using the state/P at function entry
//     (matching upstream: the test block runs once, before the separate
//     per-axis sequential fusion loop it gates), and returns 0 (no axes
//     fused, state/P completely untouched) if the ratio is >= 1.0,
//     otherwise proceeds exactly as phase 2 already did.
//   - gps_vel_innov_gate_pct / gps_pos_innov_gate_pct: new public fields,
//     defaulted to the real upstream VEL_I_GATE_DEFAULT/POS_I_GATE_DEFAULT
//     = 500 (AP_NavEKF3.cpp, verified byte-identical across every
//     APM_BUILD_TYPE #elif block including ArduPlane's own, ~line 34-39,
//     60-65, 86-91, 112-117) - same "AP_Param not wired in yet" treatment
//     as this file's other noise/limit parameters.
//
// CORRECTION TO NOTHING - THE CPP-057 TICKET'S OWN FORMULAS CHECKED OUT:
// unlike CPP-056 (which found the ticket wrong about phase 1 already
// having inhibit* fields), this ticket's posTestRatio/velTestRatio
// formulas and its VEL_I_GATE_DEFAULT/POS_I_GATE_DEFAULT/
// GLITCH_RADIUS_DEFAULT = 500/500/25 were verified line-by-line against
// the real ~line 792-932 block and AP_NavEKF3.cpp's real parameter
// defaults and matched exactly - no corrections needed this round. One
// worthwhile independent finding beyond what the ticket already stated:
// R_OBS_DATA_CHECKS[0..2] (the variance used for velTestRatio) is NOT
// always textually identical to R_OBS[0..2] (the variance used for the
// Kalman gain) in upstream's general code - upstream deliberately
// recomputes R_OBS_DATA_CHECKS[0..2] from the "no reported accuracy"
// formula unconditionally (~line 762, its own comment: "we don't want
// the acceptance radius to increase with reported GPS accuracy"), even
// on the branch where R_OBS[0..2] itself used the `gpsSpdAccuracy > 0.0f`
// reported-accuracy formula instead. That distinction is moot for THIS
// port specifically, because phase 2 never implemented the
// `gpsSpdAccuracy > 0.0f` branch at all (a named phase-2 exclusion, see
// above) - this port's R_OBS[0..2] is ALWAYS the "no reported accuracy"
// formula, so it is legitimately, exactly equal to R_OBS_DATA_CHECKS[0..2]
// here, not merely approximated as such. gps_vel_test_ratio() therefore
// reuses gps_horiz_vel_obs_variance()/gps_vert_vel_obs_variance() directly
// rather than duplicating the formula under a second name. Position's
// R_OBS_DATA_CHECKS[3]/[4] are unconditionally copied from R_OBS[3]/[4]
// with no such distinction (~line 774: `for (i=3;i<=5;i++)
// R_OBS_DATA_CHECKS[i] = R_OBS[i];`), so gps_pos_test_ratio() reusing
// gps_horiz_pos_obs_variance() is exact for the same reason without even
// needing the "no accuracy branch" argument.
//
// NOT MODELED / EXPLICITLY EXCLUDED FROM THIS PHASE (each is a real
// upstream mechanism, verified present in the ~792-932 read, named here
// with its real upstream trigger per the ticket's own acceptance
// criterion):
//   - The `_gpsGlitchRadiusMax <= 0` "soft accept" branch (~line 819-826
//     position, ~line 907-914 velocity): inflate varInnov by the failing
//     test ratio itself so the test ratio becomes exactly 1.0 and the
//     sample is fused anyway with widened variance, instead of being
//     rejected outright. Verified dead code for the real default
//     configuration: GLITCH_RADIUS_DEFAULT = 25 (positive) across every
//     APM_BUILD_TYPE block, and this port has no EK3_GLITCH_RAD parameter
//     to ever set it non-positive. A future parameterization phase that
//     wires up real AP_Param storage for this field would need to add
//     this branch to stay faithful once the parameter becomes
//     user-settable.
//   - `PV_AidingMode == AID_NONE`'s unconditional posCheckPassed = true
//     (~line 816: `if (posTestRatio < 1.0f || (PV_AidingMode ==
//     AID_NONE))`) - this port has no aiding-mode state machine
//     (AID_NONE/AID_RELATIVE/AID_ABSOLUTE, AP_NavEKF3_Control.cpp) at
//     all; phase 2's fusion already runs unconditionally in the real
//     AID_ABSOLUTE-equivalent regime (actual GPS being fused), so this
//     bypass simply does not apply. Named here again as the real gap a
//     future health/mode phase must resolve.
//   - `posTimeout`/`velTimeout`/`badIMUdata`-driven force-fuse-anyway
//     (`if (posCheckPassed || posTimeout || badIMUdata)`, ~line 836; `if
//     (velCheckPassed || velTimeout || badIMUdata)`, ~line 917) and the
//     `ResetVelocity()`/`ResetPosition()` reset-on-timeout sequence that
//     follows (including the covariance zeroRows/zeroCols/re-seed-to
//     -glitch-radius steps, ~line 838-857). None of `lastVelPassTime_ms`/
//     `lastGpsPosPassTime_ms` wall-clock bookkeeping or a real
//     ResetVelocity()/ResetPosition() re-initialization path exist in
//     this port. Real, named consequence: a sustained GPS outage in this
//     port, after this ticket, simply stops updating state/covariance
//     from GPS entirely (safe - the filter degrades to pure INS dead
//     reckoning - but not self-recovering the way upstream's real
//     reset-after-timeout behavior is). This is the real next gap for a
//     future phase, not silently absorbed into today's behavior.
//   - `gpsNoiseScaler`/satellite-count-based R_OBS scaling and
//     `sources.useVelXYSource`/`getPosXYSource` source-selection branches
//     - already a named phase-2 exclusion (no satellite-count/source-
//     selection modeling in this port); carried forward unchanged since
//     the gate formulas consume the same R_OBS this port already
//     computes.
//   - Height/baro's `hgtTestRatio` gating (`obsIndex==5`, ~line 934-957)
//     - no baro fusion exists yet, a named exclusion carried over from
//     CPP-056.
//   - The GPS-vertical-velocity-vs-baro aliasing cross-check that sets
//     `badIMUdata` (~line 777-799) - depends on independent baro height
//     fusion, out of scope for the same reason as height gating above.
//
// ============================================================================
// CPP-058, PHASE 4 (this ticket): GPS fusion timeout detection and
// reset-on-timeout recovery. Everything above this point is phase 1
// (CPP-052) plus phase 2 (CPP-056) plus phase 3 (CPP-057), unmodified.
// Read NavEKF3_core::ResetVelocity()/ResetPosition() in full
// (AP_NavEKF3_PosVelFusion.cpp lines 14-89 / 94-186) and the real
// timeout-DETECTION logic (AP_NavEKF3_Control.cpp ~line 361-406,
// posAidLossCritical/attAidLossCritical/PV_AidingMode) before extending
// anything below - both were read in full for this phase, not skimmed
// from the ticket's own summary of them.
// ============================================================================
//
// THE GAP THIS PHASE CLOSES: phase 3's own banner above named it
// explicitly - after a sustained real GPS outage, fuse_gps_velocity()/
// fuse_gps_position() just permanently stop updating state/covariance
// from GPS (every sample keeps failing gps_vel_test_ratio()/
// gps_pos_test_ratio() against an increasingly stale, drifted state),
// with no way to ever recover. This phase adds the missing half:
// elapsed-time tracking since the last successful fusion, a fixed
// timeout check, and a reduced-scope reset that unconditionally
// re-anchors state+covariance to the next available (still-failing-the-
// gate) GPS sample once the timeout fires.
//
// WHY THIS PHASE IS SCOPED NARROWER THAN "PORT ResetVelocity()/
// ResetPosition()" SOUNDS: investigation before writing any code (as the
// ticket itself directed) found both halves of upstream's real mechanism
// more entangled with already-excluded machinery than a naive read of
// AP_NavEKF3_PosVelFusion.cpp's function names alone would suggest -
// upstream's real ResetVelocity()/ResetPosition() touch output-buffer/
// complementary-filter blending state (storedOutput[]/outputDataNew/
// outputDataDelayed) this port has never built, and ResetPosition()
// separately projects a raw lat/lng Location through
// EKF_origin.get_distance_NE_ftype() that this port's GpsSample already
// supplies pre-projected (CPP-056's own convention). And upstream's real
// timeout-DETECTION logic lives inside the full PV_AidingMode state
// machine (AP_NavEKF3_Control.cpp), already named excluded since CPP-052
// phase 1 and still not built - so this phase does NOT port
// posAidLossCritical/attAidLossCritical or any AID_NONE/AID_RELATIVE/
// AID_ABSOLUTE transition logic; it substitutes the one real,
// always-applicable fixed threshold (posRetryTimeUseVel_ms) directly.
// See "NOT PORTED" below for the complete, itemized cut.
//
// WHAT THIS PHASE BUILDS:
//   - last_vel_pass_time_s / last_pos_pass_time_s: elapsed-time
//     bookkeeping (upstream: lastVelPassTime_ms/lastGpsPosPassTime_ms,
//     AP_NavEKF3_core.h:1137-1138), in seconds per this port's own
//     dt_ekf_avg-style caller-supplied-time convention (ADR-0012) - see
//     fuse_gps_velocity()/fuse_gps_position()'s new `now_s` parameter
//     below, never a real-time-clock read.
//   - A fixed 10.0s timeout (upstream: posRetryTimeUseVel_ms,
//     AP_NavEKF3.h:493, verified directly: `const uint16_t
//     posRetryTimeUseVel_ms = 10000;` - a real, hardcoded upstream
//     constant, NOT an AP_Param, same "not user-tunable" treatment as
//     e.g. kGpsNeVelVarAccScale in ekf_core.cpp, hence a file-local
//     anonymous-namespace constant there rather than a public field),
//     applied identically to BOTH position and velocity timeout - see
//     "SIMPLIFICATION NAMED EXPLICITLY" below for why one threshold
//     covers both here.
//   - reset_velocity()/reset_position(): reduced-scope ports of
//     ResetVelocity()/ResetPosition() covering only the state+covariance
//     work - see "NOT PORTED" below for exactly what's cut and why.
//     Public (like fuse_direct_state_observation()) so tests can verify
//     the reset behavior directly.
//   - Wiring into fuse_gps_velocity()/fuse_gps_position(): a gate
//     failure (gps_vel_test_ratio()/gps_pos_test_ratio() >= 1.0) now
//     checks the elapsed-time timeout BEFORE returning 0 - if timed out,
//     calls the new reset function instead (which does NOT count as a
//     fusion, matching upstream's own "Don't fuse the same data we have
//     used to reset states" comment, ~line 852) and the reset function
//     itself re-stamps the pass-time so the timeout clock restarts from
//     the reset; if not yet timed out, behavior is byte-for-byte
//     identical to phase 3 (state/P untouched, returns 0).
//   - `now_s` added as a new, DEFAULTED (=0) trailing parameter to both
//     fuse_gps_velocity() and fuse_gps_position(), so every phase 2/3
//     test and call site keeps compiling and behaving identically
//     without modification (last_*_pass_time_s also defaults to 0, so
//     elapsed-since-last-pass stays permanently 0 unless a caller opts
//     in by passing real monotonic time) - the timeout genuinely never
//     fires for a caller that hasn't started threading time through yet,
//     which is the correct, safe default for a caller that doesn't know
//     this feature exists.
//
// CORRECTIONS/FINDINGS VERIFIED THIS ROUND (the ticket's own text asked
// to "say so clearly" if its premise turned out wrong anywhere - two
// things found beyond what the ticket already flagged):
//   1. The ticket states reset_position()'s variance re-seed should be
//      sq(gps_horiz_pos_noise), matching ResetPosition()'s OWN
//      P[7][7]/P[8][8] formula (~line 146: `P[7][7] = P[8][8] =
//      sq(MAX(gpsPosAccuracy,frontend->_gpsHorizPosNoise));`, MAX
//      degenerating exactly as the ticket describes once gpsPosAccuracy
//      is excluded - THIS PART IS CORRECT, verified directly). But
//      reading the real TIMEOUT-TRIGGERED CALL SITE (FuseVelPosNED(),
//      AP_NavEKF3_PosVelFusion.cpp ~line 844-856, NOT ResetPosition()
//      itself) shows upstream immediately RE-OVERRIDES that value right
//      after calling ResetPosition(): `P[7][7] =
//      sq(ftype(0.5f*frontend->_gpsGlitchRadiusMax)); P[8][8] =
//      P[7][7];` (~line 855-856) - with GLITCH_RADIUS_DEFAULT=25
//      (verified, CPP-057 banner above), that's sq(12.5)=156.25, ~625x
//      looser than sq(gps_horiz_pos_noise)=sq(0.5)=0.25. This override
//      does NOT exist on the symmetric velocity-reset call site (~line
//      920-924 has no P override after ResetVelocity()) - it is a
//      position-only real behavior. This port deliberately follows the
//      ticket's literal instruction and ResetPosition()'s OWN formula
//      (sq(gps_horiz_pos_noise)), NOT the caller-side override, because
//      that override belongs to the full "reset-on-timeout sequence"
//      orchestration CPP-057's own banner already named as excluded (see
//      finding 2 below) - this ticket's own scope pointer is
//      specifically "lines ~14-183" (ResetVelocity()/ResetPosition() in
//      isolation), not the ~700-960 FuseVelPosNED() orchestration around
//      them. Named here as a real, disclosed divergence, not silently
//      absorbed: this port's post-reset position variance is TIGHTER
//      than upstream's real post-reset value, meaning this port's very
//      next position-gate check after a reset is measurably stricter
//      than upstream's would be.
//   2. Discovered while reading the same call site: upstream's real
//      timeout-triggered position reset is additionally gated by
//      `(!velAiding || gpsGoodToAlign)` (~line 845) - since this port
//      always assumes velAiding==true (see "SIMPLIFICATION NAMED
//      EXPLICITLY" below), that reduces to requiring `gpsGoodToAlign` (a
//      GPS-quality-sufficient-to-align check, computed elsewhere and not
//      modeled in this port at all) before upstream would even perform
//      the reset. There is also a SECOND, independent reset trigger this
//      port does not model: `posVarianceIsTooLarge` (~line 844,
//      `(P[8][8] + P[7][7]) > sq(_gpsGlitchRadiusMax)` - a
//      growing-covariance trigger, distinct from elapsed-time). This
//      port's reset_position() is wired to fire unconditionally once its
//      own elapsed-time timeout is reached, with no gpsGoodToAlign
//      quality gate and no posVarianceIsTooLarge alternate trigger -
//      both real upstream mechanisms, both out of this ticket's stated
//      scope (which asked only for "a single fixed elapsed-time timeout
//      check").
//
// SIMPLIFICATION NAMED EXPLICITLY (per the ticket's own instruction):
// this port always assumes GPS velocity aiding is available (velAiding
// == true in upstream's own terms, since fuse_gps_velocity() always
// exists and is always called), matching upstream's own
// posRetryTimeUseVel_ms branch selection (AP_NavEKF3_Control.cpp ~line
// 376-380: `if (!velAiding) { maxLossTime_ms = posRetryTimeNoVel_ms; }
// else { maxLossTime_ms = posRetryTimeUseVel_ms; }`).
// posRetryTimeNoVel_ms (7000ms, the no-velocity-aiding case) is
// therefore never the applicable branch here and is not ported;
// tiltDriftTimeMax_ms (15000ms, attAidLossCritical's threshold, a
// different failure mode - loss of ALL attitude aiding sources, not
// specifically GPS position/velocity) is likewise not ported. If a
// future phase ever models a GPS-velocity-unavailable case (e.g. a
// non-GPS velocity source, or a GPS dropout that also takes out velocity
// aiding specifically), this assumption - and the single-threshold
// simplification above - would need revisiting.
//
// NOT PORTED FROM ResetVelocity()/ResetPosition() (named per the
// ticket's own acceptance criterion - each is a real upstream mechanism,
// verified present in the ~14-186 read, deliberately left out because it
// belongs to a mode/buffer/feature this port doesn't have):
//   - Output-buffer/complementary-filter blending updates (`storedOutput
//     [i].velocity`/`.position` for i in 0..imu_buffer_length,
//     `outputDataNew`/`outputDataDelayed` - ~line 76-83 velocity, ~line
//     168-175 position) - this port has no such buffers at all (named
//     excluded since CPP-052 phase 1's "AP_NavEKF3_Outputs.cpp" bullet).
//     Nothing to update since nothing exists to update.
//   - `EKF_origin.get_distance_NE_ftype(gpsloc)` raw-lat/lng-to-local-NE
//     projection (~line 139-140) plus its `gps_corrected.vel.xy()*
//     0.001*tdiff` time-alignment correction (~line 143) - GpsSample
//     already supplies an already-projected local-NE position (CPP-056's
//     own established convention, see this file's phase-2 banner).
//   - `velResetNE`/`posResetNE` reset-delta bookkeeping (~line 35-36,
//     84-86 velocity; ~line 109-110, 177-179 position) - write-only
//     diagnostic fields (upstream logs/reports the reset jump magnitude;
//     nothing in this port consumes an equivalent value).
//   - `resetDataSource` selection (`frontend->sources.getVelXYSource()`/
//     `getPosXYSource()`, RNGBCN/EXTNAV branches, ~line 17-31/96-110) -
//     this port models exactly one position/velocity source (GPS); the
//     RNGBCN (range beacon) and EXTNAV (external nav) branches assume
//     sensors this port has never modeled (already-named exclusions,
//     RNGBCN since CPP-052, EXTNAV since CPP-056).
//   - `CorrectGPSForAntennaOffset(gps_corrected)` (~line 53, 138) - GPS
//     antenna lever-arm correction; GpsSample is taken as
//     already-corrected, matching CPP-056's own convention for
//     GpsSample's velocity/position fields.
//   - The `PV_AidingMode != AID_ABSOLUTE` branches entirely
//     (constant-position/zero-velocity fallback when not real-GPS-
//     aiding, ~line 44-48 velocity, ~line 125-131 position) - this port
//     has no aiding-mode state machine (already-named phase-1
//     exclusion); real GPS aiding is this port's only mode, matching the
//     real AID_ABSOLUTE branch, which is the one this phase ports.
//   - The EXTNAV/RNGBCN preference-order `else if` branches inside the
//     AID_ABSOLUTE case itself (~line 58-68 velocity, ~line 147-163
//     position) - same reasoning as the resetDataSource bullet above.
//   - `posTimeout`/`velTimeout`/`badIMUdata`-driven force-fuse-anyway
//     booleans, `posVarianceIsTooLarge`, `gpsGoodToAlign`, and the
//     caller-side P-override at the real timeout call site (~line
//     820-928) - see "CORRECTIONS/FINDINGS" #1/#2 above; this is the
//     full "reset-on-timeout SEQUENCE" CPP-057's banner already named as
//     excluded, distinct from the ResetVelocity()/ResetPosition()
//     functions this ticket actually scopes.
//
// STILL NOT MODELED / CARRIED FORWARD FROM EARLIER PHASES (unaffected by
// this ticket, restated for completeness since this phase touches the
// same file area):
//   - `PV_AidingMode`/`AID_NONE`/`AID_RELATIVE` state machine and
//     `attAidLossCritical` handling (AP_NavEKF3_Control.cpp ~line
//     361-370) - already-named phase-1 exclusion, unaffected by this
//     ticket.
//   - `gpsSpdAccuracy`/`gpsPosAccuracy` GPS-reported-accuracy terms in
//     the reset variance formulas - already-established phase-2
//     exclusion (GpsSample carries no reported-accuracy field), carried
//     forward unchanged into reset_velocity()/reset_position().
//
// A REAL, NAMED DIVERGENCE IN THIS PHASE'S OWN WIRING: upstream sets
// lastVelPassTime_ms/lastGpsPosPassTime_ms as soon as the test-ratio
// gate itself passes (velCheckPassed/posCheckPassed, ~line 822/901),
// BEFORE and INDEPENDENTLY of whether the deeper per-axis healthyFusion
// negative-variance guard (this port's fuse_direct_state_observation()
// return value) actually succeeds for any axis. This port instead
// stamps last_vel_pass_time_s/last_pos_pass_time_s only when at least
// one axis's fuse_direct_state_observation() call actually returns true
// (n_fused > 0) - per this ticket's own explicit instruction ("gate
// passed AND healthyFusion passed"). This is stricter than upstream (a
// pathological case where the gate passes but every axis's healthyFusion
// guard fails would restart upstream's timeout clock but NOT this
// port's) - a deliberate, disclosed, conservative choice: never claim
// "GPS fusion is current" when literally zero state was actually
// updated. Worth revisiting only if a future phase's
// fuse_direct_state_observation() behavior changes enough to make that
// edge case not vanishingly rare.
//
// ============================================================================
// CPP-059, PHASE 5 (this ticket): 3-axis magnetometer fusion. Everything
// above this point is phase 1 (CPP-052) through phase 4 (CPP-058),
// unmodified. Read NavEKF3_core::FuseMagnetometer() in full
// (AP_NavEKF3_MagFusion.cpp ~line 473-843, ~370 lines) before extending
// anything below - it was read in full for this phase, not skimmed from
// the ticket's own summary of it.
// ============================================================================
//
// WHAT THIS PHASE IS: a genuinely different, independent capability from
// phases 2-4's GPS work - NOT built on fuse_direct_state_observation().
// GPS fusion observes a single state element directly (H has exactly one
// nonzero entry). Magnetometer fusion observes a nonlinear function of the
// attitude quaternion (states 0-3) AND both magnetic-field state blocks
// (earth_magfield 16-18, body_magfield 19-21) simultaneously - H_MAG below
// has up to 8 nonzero entries per axis, and its own dense, auto-generated
// varInnovMag/SK_MX/SK_MY/SK_MZ coefficient algebra. This was flagged
// during CPP-056's own investigation and independently re-confirmed this
// round by reading the real function - verbatim transcription (see below)
// is the only responsible way to port it, exactly as CPP-052 phase 1 did
// for CovariancePrediction()'s PS0..PS222 block.
//
// WHAT THIS PHASE BUILDS:
//   - MagSample: new explicit input (see struct comment above).
//   - fuse_magnetometer(mag, gyro, dt_ekf_avg): ports FuseMagnetometer()'s
//     real body - predicted-field DCM rotation, innovMag, R_MAG, the
//     SH_MAG common-subexpression array, per-axis varInnovMag/H_MAG/
//     SK_MX/SK_MY/SK_MZ, the kalman_mask-gated Kalman gain, the KHP
//     covariance update, and state correction - X, Y, Z axes fused
//     sequentially (obsIndex 0/1/2), matching upstream's own per-axis loop
//     structure directly.
//   - innov_mag / var_innov_mag: public Vector3F members (upstream:
//     innovMag/varInnovMag) so tests can verify the dense per-axis
//     formulas independently, per the ticket's own verification standard
//     (same treatment as gps_vel_test_ratio()/gps_pos_test_ratio() above).
//   - GyroSample (phase 1's existing struct) reused, not duplicated, as
//     fuse_magnetometer()'s second parameter: R_MAG's real
//     `imuDataDelayed.delAng.length()/delAngDT` term needs exactly the two
//     fields GyroSample already carries - no new type needed for this
//     input.
//
// THE REAL, DISTINCTIVE FAILURE-BEHAVIOR THIS PHASE PORTS (verified
// directly, matches the ticket): unlike GPS fusion's fuse_direct_state_
// observation(), which just returns false and leaves P/state untouched
// when its healthyFusion guard fails, FuseMagnetometer() calls
// CovarianceInit() - a FULL covariance re-initialization, this port's
// existing covariance_init() - and unconditionally aborts the ENTIRE
// 3-axis fusion call on EITHER of two failure modes:
//   1. `varInnovMag[i] < R_MAG` ("badly conditioned") on any single axis -
//      checked BEFORE that axis's Kalman gain is even computed. Verified:
//      the three varInnovMag checks for X/Y/Z all happen up front,
//      sequentially, each with its own early return - not one combined
//      check across all three.
//   2. The same per-axis healthyFusion `KHP[i][i] > P[i][i]` guard GPS
//      fusion also has - but here, failing it triggers CovarianceInit() +
//      abort instead of GPS's simple "skip this axis, keep going".
// fuse_magnetometer() returns a fresh bool (neither fuse_gps_*()'s
// axes-fused int count nor fuse_direct_state_observation()'s single-axis
// bool fits a 3-state all-or-nothing collapse cleanly): true only if all 3
// axes completed without either abort path firing. A false return means P
// has ALREADY been reset via covariance_init() by the time control returns
// - callers should treat it as "the filter recovered by resetting P", not
// "an update was merely skipped".
//
// CORRECTION / CLARIFICATION vs. THE TICKET'S OWN FRAMING: the ticket
// describes the excluded magInnovGate test-ratio check (see below) as
// analogous to "a separate outer gate" the way CPP-057's GPS gating wraps
// around fuse_direct_state_observation() from outside. Verified directly
// this round: that framing is not quite accurate. Upstream's real
// magTestRatio/magHealth check (~line 606-616: `magTestRatio[i] =
// sq(innovMag[i]) / (...); magHealth = (...); if (!magHealth) return;`) is
// NOT a separate function at all - it lives INSIDE FuseMagnetometer()
// itself, textually between the three varInnovMag checks and the per-axis
// H_MAG/Kalman-gain loop. This ticket's own scope instruction to exclude
// "innovation-consistency gating analogous to CPP-057" is followed
// literally here regardless of that framing difference - the
// magTestRatio/magHealth block is skipped entirely below, not reproduced -
// but it means this phase omits a check that is textually inside the real
// FuseMagnetometer() body, not a separate outer wrapper the ticket's
// phrasing implied. Named here as a real, disclosed correction to the
// ticket's own premise, per the ticket's own instruction to say so clearly
// when something doesn't check out. Practical, real consequence: this
// port's fuse_magnetometer() will proceed to fuse an axis with a
// genuinely large normalized innovation - one that upstream's real
// MAG_I_GATE_DEFAULT=300 would have rejected (verified: AP_NavEKF3.cpp
// #define MAG_I_GATE_DEFAULT 300, all four APM_BUILD_TYPE blocks) - as
// long as it clears the much coarser bad-conditioning/healthyFusion
// checks. A real, named gap for a future phase, same spirit as CPP-057
// already closed for GPS.
//
// EXPLICITLY OUT OF SCOPE (each named with its real upstream trigger, per
// the ticket's own acceptance criterion):
//   - `dvelBiasAxisInhibit[]` per-axis accel-bias-state narrowing inside
//     kalman_mask (real trigger: `if (!dvelBiasAxisInhibit[index])
//     kalman_mask |= (1<<stateIndex);`, appears identically x3, once per
//     axis) - already a named phase-1/2 gap (this port has one
//     inhibit_del_vel_bias_states bool covering all 3 accel-bias states,
//     not a per-axis array). The kalman_mask construction below sets bits
//     13-15 together, gated only by that single existing flag.
//   - `MagTableConstrain()`/`have_table_earth_field`/`_mag_ef_limit` (real
//     trigger: `if (have_table_earth_field && frontend->_mag_ef_limit >
//     0) MagTableConstrain();`, run once per successfully-fused axis) -
//     World Magnetic Model table lookup; no such table exists in this
//     port (already a named phase-1 exclusion for ConstrainStates()'s
//     alternate earth-magfield clamp branch - this is that same
//     mechanism's second real upstream call site).
//   - `magFusePerformed` (real trigger: set true once per axis, inside
//     each of the three obsIndex branches) - a cross-process flag that
//     exists solely to coordinate with `fuseEulerYaw()` (a separate
//     yaw-only fusion path) so the two don't double-fuse yaw information
//     in the same frame; moot with no fuseEulerYaw() in this port.
//   - `controlMagYawReset()`/`realignYawGPS()`/`alignYawAngle()` (the
//     whole yaw-reset/realignment state machine), `fuseEulerYaw()`,
//     `FuseDeclination()`, and the EKFGSF_* (Gaussian Sum Filter yaw
//     estimator) functions - all real, separate mechanisms elsewhere in
//     AP_NavEKF3_MagFusion.cpp (verified: none of them are called from
//     inside FuseMagnetometer() itself), none ported. This ticket is
//     FuseMagnetometer() ONLY, matching its own stated scope.
//   - The `magTestRatio`/`magHealth` innovation-consistency gate (real
//     trigger: `frontend->_magInnovGate`, MAG_I_GATE_DEFAULT=300,
//     AP_NavEKF3.cpp) - see the correction note above for exactly what/
//     where this is and why it's excluded per the ticket's own
//     instruction; a real, named gap, not silently absorbed.
//   - `learnMagBiasFromGPS()` - a separate, unrelated mag-bias learning
//     mechanism (verified: a distinct function, not called from within
//     FuseMagnetometer()), not part of this ticket.
//   - `faultStatus.bad_xmag`/`bad_ymag`/`bad_zmag` bookkeeping (real
//     trigger: set on each of the abort paths and the per-axis
//     healthyFusion-false branch) - write-only diagnostic flags; no
//     faultStatus struct exists in this port (already the established
//     GPS-fusion precedent - see fuse_gps_velocity()'s own doc comment
//     above: "no faultStatus.bad_nvel/bad_evel/bad_dvel bookkeeping").
//     fuse_magnetometer()'s bool return value is this port's only fault
//     signal, same treatment.
//   - `stateIndexLim` bounding of the Kfusion-computation loop itself
//     (real trigger: upstream's own `for (auto i=0; i<24; i++)`, ~line
//     668/728/788 - unlike the KHP/healthyFusion/P-update loops right
//     after it, which upstream DOES bound at stateIndexLim, this
//     particular loop is NOT bounded upstream; it instead relies on
//     kalman_mask's bits being unset for any index the inhibit flags
//     would otherwise exclude). Verified algebraically equal to bounding
//     at state_index_lim() for every inhibit-flag combination this port's
//     state_index_lim() and this phase's kalman_mask construction can
//     produce - both are built from the same four inhibit flags in the
//     same nesting order. Reproduced literally as upstream's own
//     unbounded 0..23 loop with the mask check inside, NOT "corrected" to
//     bound at lim - matching upstream exactly, not a divergence from it.
//
// A REAL, NOTABLE CONSEQUENCE OF inhibit_mag_states DEFAULTING TO true
// (unchanged since phase 2 - see that banner's own "CORRECTION" section
// above): H_MAG[16..18] (earth-field Jacobian entries) are still computed
// and used inside the KHP covariance-coupling sum on every call, but
// kalman_mask never sets bits 16-21 while inhibit_mag_states is true, so
// Kfusion[16..21] is exactly 0 in that configuration - meaning
// earth_magfield/body_magfield themselves are never actually updated by
// fuse_magnetometer() at today's default settings, EXACTLY matching GPS
// fusion's own already-documented mag/wind-permanently-inhibited behavior
// (phase 1 banner, simplification 1). Attitude/velocity/position
// (H_MAG[0..3], bits 0-9, unconditionally unmasked) and gyro/accel-bias
// (bits 10-12/13-15, unmasked by default too) ARE corrected regardless -
// so mag fusion has real, useful effect on attitude even with the
// mag-field states still inhibited, verified by this phase's own tests.
// ============================================================================

// ============================================================================
// CPP-060, PHASE 6 (this ticket): magnetometer innovation-consistency
// gating. Everything above this point is phase 1 (CPP-052) through phase
// 5 (CPP-059), unmodified. Read AP_NavEKF3_MagFusion.cpp lines ~571-582
// directly before extending anything below - it was read in full for
// this phase (short, ~12 lines, unlike phase 1/5's dense algebra, but
// verified directly anyway per the ticket's own instruction).
// ============================================================================
//
// THE GAP THIS PHASE CLOSES: phase 5's own banner above named it
// explicitly, in its own "CORRECTION / CLARIFICATION" section -
// fuse_magnetometer() fuses every reading that clears the coarser
// per-axis bad-conditioning/healthyFusion checks, with no way to reject
// a reading that is merely INCONSISTENT (e.g. local magnetic
// interference, a miscalibrated compass) but not extreme enough to trip
// either of those. This mirrors the exact gap CPP-057 already closed for
// GPS fusion.
//
// THE REAL, VERIFIED FORMULA AND LOCATION (AP_NavEKF3_MagFusion.cpp
// lines 571-582, read directly this round):
//   for (i = 0..2):
//     magTestRatio[i] = sq(innovMag[i]) /
//                        (sq(MAX(0.01*_magInnovGate, 1.0)) * varInnovMag[i]);
//   magHealth = (magTestRatio[0] < 1.0 && magTestRatio[1] < 1.0 &&
//                magTestRatio[2] < 1.0);
//   if (!magHealth) { return; }
// Verified directly: this is ALL THREE axes individually passing < 1.0,
// NOT one combined sum-of-squares ratio the way gps_vel_test_ratio()/
// gps_pos_test_ratio() work (those sum sq(innovation) across N/E/D or
// N/E into ONE ratio before comparing to 1.0 - see this file's "CPP-057,
// PHASE 3" banner). A single badly-wrong axis here fails the whole gate
// on its own, with no averaging-out across the other two axes the way a
// combined ratio would allow - this port's own test exploits exactly
// that per-axis structure.
//
// Real default verified this round: MAG_I_GATE_DEFAULT = 300
// (AP_NavEKF3.cpp #define, identical across all four APM_BUILD_TYPE
// #elif blocks including ArduPlane's own, ~line 37/63/89/115 - same
// pattern as CPP-057's VEL_I_GATE_DEFAULT/POS_I_GATE_DEFAULT).
// `_magInnovGate` is an AP_Int16 "Percentage number of standard
// deviations", hence this port's mag_innov_gate_pct field name (same
// _pct convention as gps_vel_innov_gate_pct/gps_pos_innov_gate_pct).
//
// WHAT THIS PHASE BUILDS:
//   - mag_test_ratio(): the real per-axis magTestRatio formula, public
//     (like gps_vel_test_ratio()/gps_pos_test_ratio()) so tests can
//     verify it independently. Reads this object's own stored
//     innov_mag/var_innov_mag - unlike the GPS gates (which take a fresh
//     GpsSample and recompute innovations themselves from state), this
//     phase's innovations are already populated as members by
//     fuse_magnetometer() itself before this gate needs them (they are
//     computed unconditionally near the top of that function, phase 5),
//     so there is nothing left for mag_test_ratio() to recompute from
//     scratch - it just applies the gate formula to what is already
//     there.
//   - mag_innov_gate_pct: new public field, defaulted to the real
//     MAG_I_GATE_DEFAULT = 300 (see above) - same "AP_Param not wired in
//     yet" treatment as this file's other noise/limit/gate parameters.
//   - Wiring into fuse_magnetometer(): the gate is evaluated ONCE, using
//     var_innov_mag/innov_mag as populated by THIS call (all three axes'
//     bad-conditioning checks have already run and passed by this
//     point - matching upstream's own real textual ordering exactly,
//     verified directly: the magTestRatio/magHealth block sits between
//     the three varInnovMag checks, ~line 532-569, and the per-axis
//     H_MAG/Kalman-gain obsIndex loop, ~line 584 onward). On failure,
//     fuse_magnetometer() returns false immediately - see below for why
//     this is NOT the same false as the two existing failure modes.
//
// WHERE THIS PLUGS IN, exactly (verified directly, not assumed from the
// ticket's own summary): AFTER the three per-axis bad-conditioning
// checks (unchanged from CPP-059 - each can still independently abort
// with its own full covariance_init() reset, before the gate ever runs),
// BUT BEFORE the per-axis H_MAG/Kalman-gain obsIndex loop. A reading that
// fails a bad-conditioning check never reaches this gate at all (it has
// already returned); a reading that passes all three bad-conditioning
// checks is then subjected to this new gate before any Kalman gain is
// even computed for any axis.
//
// THE CRITICAL, DISTINCT THIRD OUTCOME (get this right - CPP-059's own
// banner already flagged this as the real, disclosed gap this phase
// closes): unlike a badly-conditioned axis or a failed healthyFusion
// guard - BOTH of which call covariance_init() (upstream:
// CovarianceInit()) and reset the ENTIRE covariance matrix before
// aborting - a failed mag_test_ratio()/magHealth gate is upstream's own
// bare `return;` (~line 580-582) with NO CovarianceInit() call anywhere
// near it. This port's fuse_magnetometer() therefore returns false on
// THREE genuinely different real upstream conditions now, only two of
// which reset P:
//   1. Badly-conditioned axis            -> covariance_init() then false.
//   2. Failed healthyFusion guard        -> covariance_init() then false.
//   3. (THIS PHASE) Failed mag gate      -> false, P/state UNTOUCHED.
// A false return alone no longer distinguishes "the filter recovered by
// resetting P" from "this one reading was simply skipped, state intact"
// - callers/tests that need to tell these apart must inspect state/P
// directly (byte-for-byte, as this phase's own tests do), exactly the
// same caveat CPP-059's own doc comment already carried for
// distinguishing failure modes 1 and 2 from each other, now extended to
// a third.
//
// CORRECTION TO NOTHING - THE TICKET'S OWN PREMISE CHECKED OUT: this
// ticket's own text already carried forward CPP-059's real, disclosed
// correction (the gate lives textually INSIDE FuseMagnetometer() itself,
// between the bad-conditioning checks and the H_MAG loop - NOT a
// separate outer wrapper the way CPP-057's GPS gating was originally,
// mistakenly framed as analogous to). Re-verified directly this round
// against the live upstream source at lines 571-582: the formula, the
// per-axis (not combined) magHealth structure, the MAG_I_GATE_DEFAULT=300
// constant, and the exact textual ordering all matched the ticket's own
// text exactly. No further correction needed this round.
//
// EXPLICITLY OUT OF SCOPE (each named with its real upstream trigger, per
// the ticket's own acceptance criterion):
//   - Any timeout/reset-on-timeout recovery mechanism for magnetometer,
//     analogous to CPP-058's GPS work - verified directly: no such
//     mechanism exists inside FuseMagnetometer() itself. The real
//     yaw-realignment/reset machinery for a persistently bad compass
//     lives in the separate, unrelated, still-excluded
//     controlMagYawReset()/realignYawGPS()/alignYawAngle() state
//     machine - already a named CPP-059 exclusion, confirmed again this
//     round to be genuinely distinct from (and not called by)
//     FuseMagnetometer(). A gate failure here simply means this one
//     reading is skipped for this one cycle; there is no wall-clock
//     "how long has mag fusion been failing" bookkeeping analogous to
//     last_vel_pass_time_s/last_pos_pass_time_s (CPP-058), and none is
//     added by this phase.
//   - Any change to the existing bad-conditioning/healthyFusion failure
//     paths (both unchanged from CPP-059 - this phase only adds the new,
//     additional gate that runs between them and the fusion loop).
//   - `faultStatus.bad_xmag`/`bad_ymag`/`bad_zmag` bookkeeping remains
//     out of scope for the same reason CPP-059 already excluded it for
//     the other two failure modes (write-only diagnostic flags, no
//     faultStatus struct in this port) - upstream sets no additional
//     faultStatus flag on a magHealth failure either way (verified:
//     lines 571-582 touch no faultStatus field at all), so this phase
//     introduces no new instance of that already-named gap.
// ============================================================================

// ============================================================================
// CPP-062, PHASE 8 (this ticket): baro height fusion. Everything above this
// point is phase 1 (CPP-052) through phase 6 (CPP-060), unmodified (phase 7,
// CPP-061, was a validation-only ticket that added zero EkfCore production
// code - see ekf_closed_loop_test.cpp's own banner). Read
// AP_NavEKF3_PosVelFusion.cpp lines ~927-980 (the height innovation-
// consistency gate) and ~1181-1382 (selectHeightForFusion(), read in full
// this round, not just the baro branch, to confirm every other branch is
// genuinely inapplicable) and NavEKF3_core::ResetHeight() (lines 287-355)
// directly before extending anything below - all three were read in full
// for this phase, not skimmed from the ticket's own summary of them.
// ============================================================================
//
// THE GAP THIS PHASE CLOSES: CPP-061's own closed-loop validation named it
// explicitly (see ekf_closed_loop_test.cpp's "REAL, DISCLOSED GAPS" section,
// now stale as of this phase) - state.position.z (altitude) was never
// directly observed, only indirectly disciplined via fuse_gps_velocity()'s
// real vertical-velocity fusion integrated forward with no independent
// position anchor. This phase adds the missing direct observation: real
// baro height fusion, reusing fuse_direct_state_observation() (CPP-056) at
// state_index=9, an innovation-consistency gate mirroring CPP-057/CPP-060's
// structure, and a timeout/reset mirroring CPP-058's reset_position()-style
// pattern - composition of three already-verified primitives at a new state
// index, not new algorithmic machinery, exactly as the ticket predicted.
//
// SIGN CONVENTION - VERIFIED DIRECTLY, NOT ASSUMED: upstream's baro branch
// (selectHeightForFusion(), ~line 1367-1382) sets `hgtMea = baroDataDelayed.
// hgt - baroHgtOffset` then `velPosObs[5] = -hgtMea`. `baroDataDelayed.hgt`
// traces to `dal.baro().get_altitude()` (AP_NavEKF3_Measurements.cpp
// readBaroData(), ~line 784) - a positive-UP altitude reading (AP_Baro's own
// convention), the same positive-up convention already established for GPS
// height in this same function's GPS branch (`hgtMea = gpsDataDelayed.hgt;
// velPosObs[5] = -hgtMea;`, ~line 1349-1350). The height gate (~line 929)
// then computes `innovVelPos[5] = stateStruct.position.z - velPosObs[5]`,
// i.e. `position.z - (-hgtMea) = position.z + hgtMea`. This port's
// fuse_baro_height() takes `baro_altitude_m` as that same positive-up
// reading (matching TECS's own `baro_altitude_m` convention exactly, see
// "BaroSample vs. a bare scalar" below) and computes the innovation as
// `state.position.z + baro_altitude_m` directly - algebraically identical
// to upstream's `position.z - velPosObs[5]` via `velPosObs[5] =
// -baro_altitude_m`, not a new formula.
//
// baroHgtOffset IS ALWAYS ZERO IN A BARO-ONLY PORT - A VERIFIED CONSEQUENCE,
// NOT A SEPARATE EXCLUSION: `baroHgtOffset` (AP_NavEKF3_core.h:1345) is only
// ever written by `calcFiltBaroOffset()`, and that call is itself gated by
// `if (activeHgtSource != AP_NavEKF_Source::SourceZ::BARO)` (~line 1291) -
// i.e. it exists solely to avoid a jump when SWITCHING TO baro from some
// other active height source. This port has no other height source and no
// source-selection state machine at all (see exclusions below) - baro is
// unconditionally the only source, so that condition is never true and
// baroHgtOffset is never written, remaining at its zero-initialized default
// forever. `hgtMea = baroDataDelayed.hgt - baroHgtOffset` therefore
// degenerates exactly to `baroDataDelayed.hgt` for this port - not an
// approximation, a verified consequence of the baro-only exclusion already
// named below, so no separate baro_hgt_offset field is added.
//
// BaroSample vs. a bare scalar (the ticket's own open question, per its
// "New input needed" section): checked both established precedents directly
// before deciding. GpsSample/MagSample (this file's own phase 2/5 structs)
// exist because each bundles MULTIPLE distinct fields (velocity_ned +
// position_ne; a single mag vector needing its own struct only for
// consistency with GpsSample's precedent) AND, in GpsSample's case, made a
// real documented convention decision (already-projected local-NE position)
// worth anchoring a named type to. A single barometric altitude reading is
// exactly one ftype with no such bundling or convention decision to anchor -
// TECS's own precedent (`TecsInputs::baro_altitude_m`, modules/ap-tecs/
// include/fwcpp/tecs/tecs.hpp:417, a bare float field reading directly from
// upstream's own `AP::baro().get_altitude()`) is the closer match: a single
// scalar sensor reading passed as a bare parameter, no wrapper type. This
// phase follows that precedent - fuse_baro_height() takes `ftype
// baro_altitude_m` directly, matching TECS's own field name for exactly the
// same physical quantity, deliberately NOT adding a one-field BaroSample
// struct that would carry no information a bare parameter doesn't already
// convey.
//
// WHAT THIS PHASE BUILDS:
//   - hgt_test_ratio(baro_altitude_m): the real hgtTestRatio formula (~line
//     934, `sq(innovVelPos[5]) / (sq(MAX(0.01*_hgtInnovGate,1.0)) *
//     varInnovVelPos[5])`), public (like gps_vel_test_ratio()/gps_pos_test_
//     ratio()/mag_test_ratio()) so tests can verify it independently. Unlike
//     mag_test_ratio() (which reads already-populated members) but LIKE
//     gps_vel_test_ratio()/gps_pos_test_ratio() (which recompute fresh),
//     this recomputes the innovation from the given `baro_altitude_m`
//     itself - height fusion has exactly one obsIndex, the same
//     single-fresh-sample shape as GPS's per-call recomputation, not mag's
//     multi-axis stored-member shape.
//   - baro_hgt_obs_variance(): the real `posDownObsNoise = sq(constrain_
//     ftype(frontend->_baroAltNoise, 0.1f, 100.0f))` formula (~line
//     1376-1377), reusing the ALREADY-EXISTING `baro_alt_noise` field
//     (added in phase 1, defaulted to the real ALT_M_NSE_DEFAULT=3.0 for
//     ArduPlane specifically, verified again this round directly against
//     AP_NavEKF3.cpp's APM_BUILD_ArduPlane block, ~line 78) rather than
//     adding a second noise field - phase 1's own field was already exactly
//     what this phase needs, unused until now. Public, like the GPS/mag
//     obs_variance() functions, so tests can verify it independently.
//   - hgt_innov_gate_pct: new public field, defaulted to the real
//     HGT_I_GATE_DEFAULT = 500 (AP_NavEKF3.cpp, verified identical across
//     every APM_BUILD_TYPE #elif block including ArduPlane's own,
//     ~line 39/65/91/117) - matching VEL_I_GATE_DEFAULT/POS_I_GATE_DEFAULT's
//     own 500 and DIFFERENT from MAG_I_GATE_DEFAULT's 300, exactly as the
//     ticket predicted. Same "AP_Param not wired in yet" treatment as this
//     file's other gate parameters.
//   - last_hgt_pass_time_s: elapsed-time bookkeeping (upstream:
//     lastHgtPassTime_ms), same caller-supplied-time convention as
//     last_vel_pass_time_s/last_pos_pass_time_s (CPP-058).
//   - A new, SEPARATE `kBaroFusionTimeoutS = 10.0` file-local constant in
//     ekf_core.cpp (upstream: `hgtRetryTimeMode0_ms = 10000`, AP_NavEKF3.h:
//     495) - see "A REAL CONSTANT-IDENTITY CHECK" below for why this is
//     deliberately NOT a reuse of the existing kGpsFusionTimeoutS constant
//     despite sharing its numeric value.
//   - reset_height(baro_altitude_m, now_s): a reduced-scope port of
//     ResetHeight() covering ONLY state.position.z + P[9][9] - see "A REAL
//     DIVERGENCE FOUND IN ResetHeight()'S OWN BODY" below for what upstream
//     ADDITIONALLY does that this deliberately does not reproduce.
//   - fuse_baro_height(baro_altitude_m, dt_ekf_avg, now_s=0): the obsIndex==5
//     caller, wired exactly like fuse_gps_velocity()/fuse_gps_position() -
//     gate check first (state/P completely untouched on failure), elapsed-
//     time timeout check on a gate failure (reset_height() instead of a
//     plain skip once timed out), fuse_direct_state_observation(9, ...) on a
//     gate pass, last_hgt_pass_time_s stamped only when the fusion actually
//     applied (same "gate passed AND healthyFusion passed" stricter
//     convention CPP-058's own banner already established and named as a
//     real, disclosed divergence from upstream's own gate-pass-only
//     condition).
//
// RETURN TYPE - bool, NOT fuse_gps_*()'s int axes-fused count, AND NOT
// fuse_magnetometer()'s "three distinct failure modes" bool: height fusion
// has exactly ONE obsIndex (unlike GPS's 2-3 sequentially-fused axes) and
// exactly ONE failure mode worth distinguishing from success (the gate,
// gated on hgt_test_ratio() - there is no per-axis "some axes fused, some
// didn't" partial-success state to count, and no CovarianceInit()-triggering
// bad-conditioning path the way mag fusion has). A plain bool - true iff
// fuse_direct_state_observation() actually applied the correction - is
// therefore the honest return type for this specific primitive, not a
// borrowed shape from either sibling.
//
// A REAL CONSTANT-IDENTITY CHECK (per the ticket's own instruction to verify
// directly, not assume): `posRetryTimeUseVel_ms = 10000` (AP_NavEKF3.h:493,
// already this port's kGpsFusionTimeoutS) and `hgtRetryTimeMode0_ms = 10000`
// (AP_NavEKF3.h:495) are verified to be two TEXTUALLY SEPARATE upstream
// `const uint16_t` declarations that simply happen to share the same
// numeric value - not one shared constant used for two purposes. Confirmed
// further by reading the real height-timeout-selection call site
// (selectHeightForFusion()'s own end, ~line 1404-1411): `hgtRetryTime_ms =
// ((useGpsVertVel || useExtNavVel) && !velTimeout) ? hgtRetryTimeMode0_ms :
// hgtRetryTimeMode12_ms;` - Mode0 (10000ms, "WITH vertical velocity
// measurement") is the applicable branch here because this port always
// assumes GPS velocity aiding is available (`useGpsVertVel` always true in
// upstream's own terms - the same always-has-GPS-velocity-aiding assumption
// CPP-058's own banner already established and named for its own
// posRetryTimeUseVel_ms/posRetryTimeNoVel_ms branch selection). The sibling
// `hgtRetryTimeMode12_ms = 5000` (AP_NavEKF3.h:496, the without-vertical-
// velocity case) is therefore never the applicable branch here and is not
// ported, same reasoning as CPP-058's posRetryTimeNoVel_ms exclusion. This
// phase adds its OWN `kBaroFusionTimeoutS` constant rather than reusing
// kGpsFusionTimeoutS, preserving the real upstream fact that these are two
// independently-declared constants that could in principle diverge in a
// future upstream version, even though they are numerically identical
// today.
//
// A REAL DIVERGENCE FOUND IN ResetHeight()'S OWN BODY - EXACTLY THE KIND OF
// THING THE TICKET ASKED TO CHECK FOR (per CPP-058's own "ResetPosition()'s
// caller-side glitch-radius override" precedent for what shape of surprise
// to expect): ResetHeight() (AP_NavEKF3_PosVelFusion.cpp lines 287-355) is
// NOT simply "ResetPosition()'s shape applied to one axis". After the
// position.z/P[9][9] work this phase DOES reproduce, it ALSO unconditionally
// zeroes P[6][6] (vertical velocity covariance, `zeroRows(P,6,6);
// zeroCols(P,6,6);`, ~line 344-345) and re-seeds it (`P[6][6] =
// sq(frontend->_gpsVertVelNoise);` in the non-ExtNav branch, ~line 353-355),
// AND conditionally overwrites state.velocity.z itself (~line 328-338):
// to GPS's reported vertical velocity if `inFlight && (gpsIsInUse ||
// badIMUdata) && useVelZSource(GPS) && gpsDataNew.have_vz &&` a recency
// check; to exactly 0 if `onGround`; left UNCHANGED in the remaining case
// (in flight, no fresh GPS vz available). This is a real, verified
// divergence, not assumed from the ticket's own framing (which only
// predicted "it may have its own real divergences," not what shape) -
// upstream's real height reset touches a SECOND, DIFFERENT state block
// (vertical velocity) that this ticket's own axis (position.z) does not
// obviously imply.
//
// DELIBERATELY NOT REPRODUCED, AND WHY: this port's reset_height() touches
// ONLY state.position.z and P[9][9] - the velocity.z/P[6][6] touch above is
// excluded, for three compounding reasons: (1) the ticket's own instruction
// is explicit - "a reduced-scope reset_height() (state+covariance only)" -
// read at the same granularity reset_position()/reset_velocity() (CPP-058)
// were each scoped to touch only THEIR OWN axis's state+covariance, never
// spilling into the other's; reset_velocity() does not touch position.x/y
// on a velocity reset, so reset_height() touching velocity.z on a HEIGHT
// reset would be the one asymmetric exception, not the established pattern;
// (2) upstream's own condition for what to assign is built entirely from
// state this port does not model (`inFlight`, `gpsIsInUse`, `onGround`,
// `badIMUdata` - already-named exclusions since phase 1/3/4) with no
// principled substitute available; (3) unlike ResetPosition()'s caller-side
// P-override (CPP-058's own precedent - a real override this port
// deliberately did NOT follow because it belonged to still-excluded
// orchestration code outside ResetPosition() itself), this P[6][6] touch is
// genuinely PART OF ResetHeight()'s own body, at the same "which reduced
// scope does the ticket's own instruction draw" line CPP-058 already had to
// draw for reset_position()/reset_velocity() individually. REAL, NAMED
// CONSEQUENCE: after a height-timeout-triggered reset in this port,
// P[6][6] (vertical velocity uncertainty) is NOT widened back to a fresh
// sq(gps_vert_vel_noise) floor the way upstream's real ResetHeight() always
// does - a real, disclosed gap for a future phase that wants closer
// fidelity to revisit, not silently absorbed into today's behavior.
//
// EXPLICITLY OUT OF SCOPE (each named with its real upstream trigger, per
// the ticket's own acceptance criterion):
//   - selectHeightForFusion()'s full multi-source branching - this ticket
//     is BARO-ONLY, this port's real intended primary height source for a
//     fixed-wing vehicle. Verified directly this round (grepping this
//     port's own module tree): no rangefinder or external-nav model exists
//     anywhere in this port, confirming the ticket's own expectation.
//     Named exclusions: the rangefinder branch (`activeHgtSource ==
//     RANGEFINDER`, terrain-relative height via `terrainState`,
//     `_terrGradMax`/`_rngNoise`/`rngOnGnd`, ~line 1338-1360); the
//     GPS-as-height-source branch (`activeHgtSource == GPS`,
//     `gpsHgtAccuracy`, ~line 1360-1367); and the `SourceZ::NONE` "fuse a
//     constant height of 0 at 14Hz" branch (~line 1383-1394) - no
//     source-selection state machine (`AP_NavEKF_Source`/`sources.
//     getPosZSource()`) exists in this port at all, baro is unconditionally
//     this ticket's only height source.
//   - Ground-effect baro scaling (`dal.get_takeoff_expected()`/
//     `get_touchdown_expected()`, `frontend->gndEffectBaroScaler`, ~line
//     1378-1380) - no takeoff/touchdown-expected state modeled in this
//     port.
//   - `EKF_origin`/`ekfGpsRefHgt`/`_originHgtMode` height-datum-matching
//     offset adjustments (`hgtMea += (float)(ekfGpsRefHgt - 0.01 *
//     (double)EKF_origin.alt)`, ~line 1373-1375, and the whole
//     `correctEkfOriginHeight()` call, ~line 1300-1305) - already an
//     established exclusion pattern (same reasoning as GpsSample's
//     already-projected-position convention, CPP-056's banner).
//   - `baroHgtOffset`/`calcFiltBaroOffset()` - see "baroHgtOffset IS ALWAYS
//     ZERO" above: a verified consequence of the baro-only exclusion, not a
//     separately-triggered exclusion of its own.
//   - `hgtInnovFiltState`'s on-ground pre-flight-health-check filtering
//     (~line 963-969) - a diagnostic-only value with no consumer in this
//     port (same "write-only diagnostic" treatment as `faultStatus.bad_*`
//     fields, CPP-059's own precedent).
//   - The `onGroundNotNavigating`/`AID_NONE`-driven `maxTestRatio=3.0`
//     relaxation (~line 940-941: `const bool onGroundNotNavigating =
//     (PV_AidingMode == AID_NONE) && onGround; const float maxTestRatio =
//     onGroundNotNavigating ? 3.0f : 1.0f;`) - depends on both `onGround`
//     and the aiding-mode state machine, neither modeled in this port
//     (same reasoning CPP-057/CPP-058 already used to exclude AID_NONE's
//     bypass/reset paths). hgt_test_ratio()'s pass condition is
//     unconditionally `< 1.0`, matching the non-onGround branch always.
//   - The glitch-radius soft-accept branch inside the height gate (~line
//     945-951: `else if ((frontend->_gpsGlitchRadiusMax <= 0) &&
//     !onGroundNotNavigating && (activeHgtSource ==
//     AP_NavEKF_Source::SourceZ::GPS))`) - already-established dead code
//     since CPP-057 (GLITCH_RADIUS_DEFAULT=25, positive, across every
//     APM_BUILD_TYPE block), doubly so here since this branch additionally
//     requires `activeHgtSource == GPS`, never true in a baro-only port.
//   - "Detect changes in source and reset height" (~line 1398-1402: `if
//     ((activeHgtSource != prevHgtSource) && fuseHgtData) { prevHgtSource =
//     activeHgtSource; ResetPositionD(-hgtMea); }`) - dead code for a
//     fixed, permanently-BARO source (`activeHgtSource` never changes, so
//     this condition is never true). Note this calls `ResetPositionD()`
//     (lines 263-286), a THIRD, textually distinct reset function from
//     `ResetHeight()` (lines 287-355) - verified directly to avoid
//     conflating the two; moot either way since this branch never fires
//     for this port's fixed single-source configuration.
//   - `ResetHeight()`'s velocity.z/P[6][6] touch - see "A REAL DIVERGENCE
//     FOUND" and "DELIBERATELY NOT REPRODUCED" above.
//   - `ResetHeight()`'s terrain-state update (~line 297-302) - already an
//     established phase-1 exclusion (simplification 9, no terrain model).
//   - `ResetHeight()`'s output-buffer/complementary-filter touches
//     (`outputDataNew`/`outputDataDelayed`/`storedOutput[]`/
//     `vertCompFiltState`, ~line 293-296, 303-306, 340, 350) - already an
//     established exclusion since phase 1/4 (no output buffer in this
//     port).
//   - `ResetHeight()`'s `posResetD`/`lastPosResetD_ms` diagnostic
//     bookkeeping (~line 289, 309-312) - write-only diagnostic fields,
//     already-established exclusion pattern (velResetNE/posResetNE,
//     CPP-058's own precedent).
// ============================================================================

// ============================================================================
// CPP-063, PHASE 9 (this ticket): true airspeed / wind velocity fusion.
// Everything above this point is phase 1 (CPP-052) through phase 8 (CPP-062),
// unmodified. Read NavEKF3_core::FuseAirspeed() in full (AP_NavEKF3_
// AirDataFusion.cpp lines ~20-156, ~136 lines) directly before extending
// anything below - it was read in full for this phase, not skimmed from the
// ticket's own summary of it.
// ============================================================================
//
// THE GAP THIS PHASE CLOSES: the 24-state vector's LAST two states, wind_vel
// (states 22-23), have existed since phase 1 (CPP-052) but have never been
// fused - inhibit_wind_states defaults true (phase 2, CPP-056) and nothing
// has ever cleared it. This phase adds the one real upstream mechanism that
// actually estimates wind from a direct sensor: true-airspeed fusion. Unlike
// every prior fusion phase (GPS: a single direct state observation per axis;
// mag/baro: reuse or extend existing primitives), this is a genuinely
// independent, self-contained function with its own dense Jacobian spanning
// FIVE state indices at once (velocity 4-6 AND wind 22-23) from a single
// SCALAR observation (not 3-axis like mag, not "one axis per obsIndex" like
// GPS/baro) - closer in spirit to phase 5's mag fusion (dense, auto-generated
// algebra, verbatim transcription sanctioned) than to any of the
// fuse_direct_state_observation()-based GPS/baro work.
//
// VERIFIED STRUCTURE (read directly from AP_NavEKF3_AirDataFusion.cpp lines
// 20-156, not assumed from the ticket's own summary):
//   - Predicted airspeed: `VtasPred = norm((ve-vwe), (vn-vwn), vd)` (~line
//     33) - the ticket's own component-order summary is confirmed exact
//     (E-component first, then N, then D; norm() is order-independent
//     anyway, matching this port's Vector3F::length()).
//   - The ENTIRE function body (Jacobian, Kalman gain, gate, state/
//     covariance update, AND the trailing ForceSymmetry()/ConstrainVariances()
//     calls) is wrapped in a single `if (VtasPred > 1.0f) { ... }` (~line 34,
//     closing brace ~line 155) - confirmed by reading the real indentation/
//     brace structure directly, not assumed. Below 1 m/s predicted airspeed
//     (SH_TAS[0] = 1/VtasPred would be singular/ill-conditioned at VtasPred=0)
//     NOTHING happens at all: no CovarianceInit(), no ForceSymmetry(), no
//     ConstrainVariances() - a real, THIRD bail-out shape distinct from both
//     of the ticket's named two failure outcomes (see "THE REAL, THREE-WAY
//     OUTCOME SHAPE" below).
//   - `innovVtas = VtasPred - tasDataDelayed.tas` (~line 37) - a scalar
//     innovation, state-minus-observation, matching this file's established
//     sign convention (innov_mag's own `mag_pred - mag.mag`).
//   - The badly-conditioned check (~line 54-60): `temp >= tasDataDelayed.
//     tasVariance` passes (SK_TAS[0] = 1/temp), else CovarianceInit() +
//     early return - mirrors phase 5's mag-fusion badly-conditioned-axis
//     failure mode exactly (same "reset the whole covariance and abort"
//     shape), reused directly, not re-derived.
//   - THE REAL, DISTINCTIVE ABSENCE VERIFIED THIS ROUND: unlike phase 5's
//     mag fusion, FuseAirspeed() has NO second "healthyFusion"
//     (KHP[i][i] > P[i][i]) guard anywhere in its body - verified directly
//     by reading the full ~136 lines line-by-line. After the badly-
//     conditioned check passes, the Kalman gain is computed and used
//     unconditionally (subject only to the gate below). This port's
//     fuse_airspeed() therefore has exactly ONE covariance-reset abort path
//     (badly-conditioned), not phase 5/6's two.
//   - The innovation-consistency gate (~line 108-110): `tasTestRatio =
//     sq(innovVtas) / (sq(MAX(0.01f*_tasInnovGate,1.0f)) * varInnovVtas)`,
//     `isConsistent = (tasTestRatio < 1.0f) || badIMUdata`. `varInnovVtas =
//     1.0f/SK_TAS[0]` (~line 105) - i.e. EXACTLY `temp` from the badly-
//     conditioned check above, NOT a separable R_OBS-style noise formula
//     the way GPS's/baro's gates are (see "tas_test_ratio() IS MAG-SHAPED,
//     NOT GPS/BARO-SHAPED" below).
//   - `badIMUdata` - already-established permanently-false exclusion (phase
//     1 simplification 3) - `isConsistent` reduces to the bare
//     `tasTestRatio < 1.0f` check.
//   - VERIFIED DIRECTLY, PER THIS TICKET'S OWN INSTRUCTION: NO Reset*() call
//     anywhere in FuseAirspeed()'s ~136 lines - confirmed by grepping the
//     real function body. A gate failure has no reset consequence at all
//     (see "THE REAL, THREE-WAY OUTCOME SHAPE" below for exactly what does
//     happen instead).
//
// tas_test_ratio() IS MAG-SHAPED, NOT GPS/BARO-SHAPED - A REAL DESIGN
// DECISION, NOT AN ARBITRARY CHOICE: gps_vel_test_ratio()/gps_pos_test_ratio()/
// hgt_test_ratio() all RECOMPUTE their innovation/variance fresh from a given
// sample, because their R_OBS noise formulas are separable from the Kalman
// gain computation (GPS/baro observe a single state element directly, H has
// exactly one nonzero entry, so varInnovVelPos = P[stateIndex][stateIndex] +
// R_OBS needs no Jacobian at all). mag_test_ratio() instead reads
// ALREADY-POPULATED innov_mag/var_innov_mag members, because varInnovMag is
// itself the output of the same dense, P-coupled Jacobian algebra the Kalman
// gain needs - recomputing it independently would mean duplicating that
// algebra under a second name. Verified directly this round: airspeed's
// varInnovVtas has the EXACT SAME shape - `varInnovVtas = 1.0f/SK_TAS[0]`
// IS `temp`, the same dense P[4..6][4..6,22,23]-coupled expression the
// badly-conditioned check and the Kalman gain both consume. This phase
// therefore follows mag fusion's stored-member convention (innov_vtas/
// var_innov_vtas populated by fuse_airspeed() itself; tas_test_ratio() just
// applies the gate formula to what is already there), NOT GPS/baro's
// recompute-fresh convention - a deliberate, disclosed choice matching the
// real upstream algebraic shape, not an arbitrary pick between two
// established precedents.
//
// THE REAL, THREE-WAY OUTCOME SHAPE (get this right - the ticket's own text
// warned against inventing a nonexistent third "timeout" outcome; the real
// third outcome is different from that, and distinct from phase 5/6's
// three-outcome mag shape too):
//   1. VtasPred <= 1.0 (predicted airspeed below the singular-Jacobian
//      threshold) -> returns false immediately. NOTHING happens: not even
//      ForceSymmetry()/ConstrainVariances() run (they are INSIDE the
//      `if (VtasPred > 1.0f)` block upstream, confirmed by direct reading -
//      see "VERIFIED STRUCTURE" above). State and P are byte-for-byte
//      untouched.
//   2. Badly-conditioned (`temp < tasDataDelayed.tasVariance`) -> covariance_
//      init() then returns false, mirroring phase 5's mag-fusion abort
//      exactly (P has ALREADY been reset by the time this returns).
//   3. Gate failure (tasTestRatio >= 1.0) -> returns false, but - A REAL,
//      VERIFIED DIVERGENCE FROM GPS/BARO'S "COMPLETELY UNTOUCHED" GATE
//      FAILURE, AND FROM PHASE 6'S MAG-GATE "BARE RETURN, NOTHING TOUCHED"
//      SHAPE TOO - ForceSymmetry()/ConstrainVariances() are NOT inside the
//      `isConsistent` if-block upstream (~line 117-149); they are the
//      trailing two lines of the OUTER `if (VtasPred > 1.0f)` block
//      (~line 153-154), so they run UNCONDITIONALLY whenever VtasPred > 1.0,
//      REGARDLESS of whether the gate passed or failed. Confirmed directly
//      by reading the real brace nesting, not assumed from the ticket's own
//      "a gate failure just skips fusion for that cycle, full stop" framing
//      (which is correct about StateVector and about there being no
//      Reset*() call, but does not by itself rule out P being touched by
//      something other than a reset). This port's fuse_airspeed() therefore
//      calls force_symmetry(lim) + constrain_variances(dt_ekf_avg)
//      unconditionally on a gate failure too, matching upstream exactly.
//      PRACTICAL CONSEQUENCE, VERIFIED EMPIRICALLY BY THIS PHASE'S OWN
//      TESTS - AND NOT AS BENIGN AS "NO-OP" ACROSS THE WHOLE MATRIX:
//      force_symmetry() is a no-op on an already-symmetric P, and
//      constrain_variances()'s clamps over P[0..15] are no-ops on a P
//      already produced by a prior covariance_prediction()/covariance_init()
//      call under normal operating conditions - so StateVector and P[0..15]
//      are exactly byte-for-byte untouched on a gate failure in that
//      realistic case (this phase's own gate-failure test constructs its
//      fixture this way and confirms it directly, rather than asserting the
//      weaker "no reset happened"). BUT constrain_variances()'s ALREADY-
//      ESTABLISHED unconditional mag/wind zeroing (this file's own phase-2
//      banner: "P[16..21]... are zeroed every predict cycle") is NOT a
//      no-op here whenever covariance_init() last seeded a nonzero
//      P[16..21] diagonal (sq(mag_noise)) - it fires on EVERY call that
//      reaches the trailing force_symmetry()/constrain_variances() pair,
//      gate failure included, verified directly by this phase's own
//      gate-failure test. So the honest claim is: StateVector and P[0..15]
//      are untouched on a gate failure in the realistic case; P[16..23]
//      are NOT - they get zeroed regardless of whether the gate passed,
//      failed, or - the one case that differs - never got a chance to run
//      at all (VtasPred <= 1.0, outcome 1 above, where NEITHER function
//      runs). This is still a real, disclosed divergence from GPS/baro's
//      structurally-guaranteed untouched-on-failure shape, not merely a
//      restatement of it - a test or caller that engineers P[0..15] to
//      violate a constrain_variances() clamp right before a gate-failing
//      fuse_airspeed() call would see that violation silently corrected
//      despite no fusion having occurred, something GPS's/baro's gate
//      failure paths cannot do (they never call constrain_variances() at
//      all on failure) - and P[16..23] specifically are unconditionally
//      zeroed on a gate failure regardless of what they held at entry.
//
// airDataFusionWindOnly IS PROVABLY ALWAYS FALSE IN THIS PORT - VERIFIED
// DIRECTLY, NOT ASSUMED FROM THE TICKET'S OWN TEXT: `airDataFusionWindOnly`
// (a bool member, AP_NavEKF3_core.h:1219) gates every kalman_mask bit in the
// real FuseAirspeed() EXCEPT the wind bits 22-23 (which are instead gated by
// `!treatWindStatesAsTruth`, see below) - real upstream lines 67/71/75/85:
// `if (tasDataDelayed.allowFusion && !airDataFusionWindOnly) {...}` etc.
// Traced EVERY assignment of airDataFusionWindOnly in the real upstream
// source, not just inside FuseAirspeed() itself (grepped the whole
// AP_NavEKF3_AirDataFusion.cpp and AP_NavEKF3_core.cpp):
//   - ONE-TIME INIT: `airDataFusionWindOnly = false;` (AP_NavEKF3_core.cpp:
//     327, inside InitialiseVariables() or equivalent).
//   - THE ONLY TWO ASSIGNMENTS ANYWHERE ELSE: both inside
//     `NavEKF3_core::SelectBetaDragFusion()` (AP_NavEKF3_AirDataFusion.cpp
//     lines 214/217) - a COMPLETELY DIFFERENT function from FuseAirspeed(),
//     driving `FuseSideslip()` (synthetic zero-sideslip fusion) and (behind
//     `#if EK3_FEATURE_DRAG_FUSION`) `FuseDragForces()` (body-frame drag
//     fusion) - two separate, unrelated wind-estimation mechanisms this
//     ticket explicitly does not port (see "EXPLICITLY OUT OF SCOPE" below).
//   - CONFIRMED: this ticket does not call SelectBetaDragFusion()/
//     FuseSideslip()/FuseDragForces() anywhere, and this port has never built
//     any part of them (grepped this port's own module tree - no trace).
//     With the only two real assignment sites unreachable and the one-time
//     init being `false`, airDataFusionWindOnly CAN NEVER become true in
//     this port - PROVABLY always false, the same shape as CPP-062's own
//     "baroHgtOffset is provably always zero" finding. THE TICKET'S OWN
//     FINDING HOLDS, verified directly this round, not merely trusted. Every
//     `!airDataFusionWindOnly` gate in the real kalman_mask construction
//     therefore simplifies away (always true) - no field/parameter is added
//     for something that can never vary in this port.
//
// treatWindStatesAsTruth - NOT A NEW FINDING, RE-APPLYING AN
// ALREADY-ESTABLISHED PHASE-2 DECISION: the wind bits (22-23) in
// FuseAirspeed()'s kalman_mask are gated by `!inhibitWindStates &&
// !treatWindStatesAsTruth` (~line 89), NOT by `!airDataFusionWindOnly` the
// way every other bit is - a real, verified distinction from the ticket's
// own summary ("gates every mask bit EXCEPT the wind-state bits behind
// !airDataFusionWindOnly" - true only in the sense that the wind bits are
// the one exception; they are gated by a DIFFERENT flag, not left
// ungated). `treatWindStatesAsTruth` already has NO equivalent in this port
// - this was decided in phase 2 (CPP-056)'s own banner above ("NOT PORTED
// FROM FuseVelPosNED()": "no such field exists in this port (no
// optical-flow/const-position-hold subsystem that would ever set it)") and
// applied again identically in this file's fuse_direct_state_observation()
// (ekf_core.cpp's own comment: "treatWindStatesAsTruth has no equivalent in
// this port... moot since inhibit_wind_states is permanently true in this
// phase regardless"). This phase re-verifies that reasoning still holds
// (traced treatWindStatesAsTruth's real assignments: AP_NavEKF3_core.cpp:274
// one-time init to false, :1100/:1104 set inside CovariancePrediction()'s
// own `if (!inhibitWindStates) {...}` block from `isDragFusionDeadReckoning`/
// `windStateIsObservable` - both real upstream concepts this port's own
// covariance_prediction() does not model, already an unaffected area since
// this ticket does not touch that function) and drops the term the same way
// phase 2 already did - `!inhibit_wind_states` alone gates the wind bits
// here, consistent with the rest of this file, not a fresh, independent cut.
//
// A NOTABLE STRUCTURAL CONSEQUENCE, VERIFIED THIS ROUND: after applying the
// three already-established exclusions above (tasDataDelayed.allowFusion -
// see "EXPLICITLY OUT OF SCOPE" below - always true; airDataFusionWindOnly
// provably always false; treatWindStatesAsTruth not modeled/moot) PLUS the
// already-established dvelBiasAxisInhibit[] collapse (single
// inhibit_del_vel_bias_states flag, phases 1/2/5's own precedent),
// FuseAirspeed()'s real kalman_mask construction becomes ALGEBRAICALLY
// IDENTICAL IN STRUCTURE to fuse_magnetometer()'s own kalman_mask block
// already in this file (ekf_core.cpp): bits 0-9 unconditional, bits 10-12
// gated by inhibit_del_ang_bias_states, bits 13-15 by
// inhibit_del_vel_bias_states, bits 16-21 by inhibit_mag_states, bits 22-23
// by inhibit_wind_states - the same four inhibit flags, same nesting order.
// This is not a coincidence to re-derive from scratch: both real upstream
// functions share this exact four-flag structure, and phase 5/6's own
// banner already established (and this phase's own kalman_mask reuses
// without re-proving) that this mask shape is algebraically equal to
// bounding the Kfusion-computation loop at state_index_lim() for every
// combination these four flags can produce - the Kfusion loop below is
// therefore reproduced literally as upstream's own unbounded 0..23 loop
// with the mask check inside, matching phase 5/6's own precedent exactly,
// not "corrected" to bound at lim.
//
// WHAT THIS PHASE BUILDS:
//   - fuse_airspeed(true_airspeed_m_s, dt_ekf_avg): the real FuseAirspeed()
//     body - VtasPred precondition, innovVtas, the dense SH_TAS/H_TAS/temp/
//     SK_TAS Jacobian/Kalman-gain algebra (verbatim transcription, same
//     disclosed exception as phase 1's CovariancePrediction() block and
//     phase 5's FuseMagnetometer() block - dense, auto-generated,
//     error-prone-to-hand-rederive algebra), the simplified kalman_mask
//     (see above), the gate, and the conditional state/unconditional
//     covariance-symmetrize-and-constrain tail. Reuses apply_state_correction()
//     (this file's existing helper) for the state update - no new state-
//     correction code needed, same as fuse_baro_height()'s own reuse.
//   - tas_test_ratio(): the real tasTestRatio formula, public (like every
//     other gate formula in this file) so tests can verify it independently
//     - see "tas_test_ratio() IS MAG-SHAPED" above for why this reads
//     stored innov_vtas/var_innov_vtas members rather than recomputing
//     fresh.
//   - innov_vtas / var_innov_vtas: public ftype members (upstream: innovVtas/
//     varInnovVtas), same treatment as innov_mag/var_innov_mag - scalars
//     here (one obsIndex), not Vector3F (three).
//   - eas_noise: new public field, upstream `_easNoise` (AP_GROUPINFO(
//     "EAS_M_NSE", 16, NavEKF3, _easNoise, 1.4f), AP_NavEKF3.cpp ~line 274) -
//     EAS_M_NSE_DEFAULT=1.4, verified directly. Same "AP_Param not wired in
//     yet" treatment as this file's other noise parameters.
//   - tas_innov_gate_pct: new public field, upstream `_tasInnovGate`
//     (AP_GROUPINFO("EAS_I_GATE", 17, NavEKF3, _tasInnovGate, 400),
//     AP_NavEKF3.cpp ~line 282) - EAS_I_GATE_DEFAULT=400.
//
// A REAL, VERIFIED CONSTANT-DEFINITION-STYLE DISTINCTION (per the ticket's
// own instruction to verify directly, not assume): every other gate default
// in this file (VEL_I_GATE_DEFAULT/POS_I_GATE_DEFAULT/MAG_I_GATE_DEFAULT/
// HGT_I_GATE_DEFAULT) is a `#define` MACRO, repeated identically across
// every APM_BUILD_TYPE #elif block in AP_NavEKF3.cpp (verified in phases
// 3/6/8's own banners). `EAS_I_GATE`/`EAS_M_NSE` are NOT macros at all -
// grepped AP_NavEKF3.cpp and AP_NavEKF3.h for both names: the ONLY two
// occurrences are the `@Param` doc comment and the single `AP_GROUPINFO(
// "EAS_I_GATE", 17, NavEKF3, _tasInnovGate, 400)` call itself (~line 282) -
// a LITERAL default argument in ONE AP_GROUPINFO call, not gated by
// APM_BUILD_TYPE at all (this whole parameter table entry is vehicle-
// independent, unlike the #elif-gated blocks the other four gate defaults
// live in). Same for EAS_M_NSE/_easNoise (~line 274, literal `1.4f`). A
// real, verified distinction from this file's other four gate parameters,
// not a stylistic detail - there is no second value to cross-check across
// vehicle types the way phases 3/6/8 each did for their own gate constant.
//
// EAS2TAS - NOT MODELED, FOLLOWING AN ALREADY-ESTABLISHED PORT-WIDE
// PRECEDENT: the real `tasDataNew.tasVariance = sq(MAX(frontend->_easNoise *
// EAS2TAS, 0.5f))` formula (AP_NavEKF3_Measurements.cpp's readAirSpdData(),
// already out of scope - see below) scales equivalent airspeed noise by
// `dal.get_EAS2TAS()`, an air-density-derived equivalent-to-true-airspeed
// conversion factor - a full atmospheric/density model this port does not
// have anywhere. This is NOT a fresh gap this phase introduces: modules/
// ap-tecs/include/fwcpp/tecs/tecs.hpp's own file banner already established
// the identical simplification for TECS's own airspeed handling verbatim
// ("EAS2TAS (TecsInputs::eas2tas, default 1.0f) - this port has no
// atmosphere model... 'true == equivalent airspeed' until a future slice
// adds one"). This phase follows that exact precedent: EAS2TAS is treated
// as exactly 1.0 (sea-level, no compressibility/density correction), so
// `eas_noise` is used directly as true-airspeed noise in m/s
// (`tas_variance = sq(max(eas_noise, 0.5))`), and the caller's own
// `true_airspeed_m_s` parameter is taken as already being TRUE airspeed
// (matching upstream's own `tasDataDelayed.tas` naming, which is TAS, not
// EAS) - not a new port-specific choice, a direct reuse of TECS's own
// already-disclosed one.
//
// tas_reading: BARE ftype, NOT A NEW STRUCT - SAME REASONING CPP-062 USED
// FOR baro_altitude_m: checked this file's own two established precedents
// before deciding (per the ticket's own "New input needed" instruction).
// GpsSample/MagSample exist because each bundles MULTIPLE distinct fields
// (GpsSample: velocity_ned + position_ne) or anchors a real, documented
// convention decision worth a named type (MagSample, for consistency with
// GpsSample's own precedent). A single true-airspeed reading is exactly one
// ftype with no bundling or convention decision to anchor - CPP-062's own
// `baro_altitude_m` (a bare parameter reading directly from a single sensor,
// deliberately NOT wrapped in a one-field BaroSample struct) is the closer,
// directly-applicable precedent, not GpsSample/MagSample. This phase follows
// it: fuse_airspeed() takes `ftype true_airspeed_m_s` directly, matching
// baro_altitude_m's own "bare scalar, unit-suffixed name" convention for
// exactly the same reason - a wrapper struct here would carry no information
// a bare parameter doesn't already convey.
//
// EXPLICITLY OUT OF SCOPE (each named with its real upstream trigger, per
// the ticket's own acceptance criterion):
//   - `FuseSideslip()`/`SelectBetaDragFusion()`/`FuseDragForces()` (the other
//     three functions in AP_NavEKF3_AirDataFusion.cpp, verified: this ticket
//     is FuseAirspeed() only) - separate, unrelated synthetic-zero-sideslip
//     and body-frame-drag wind-estimation mechanisms, not the direct
//     TAS-sensor fusion this ticket ports. This is also WHY
//     airDataFusionWindOnly is provably always false in this port - see
//     above.
//   - The `tasTimeout && posTimeout` forced-fusion override (`if
//     (tasDataDelayed.allowFusion && (isConsistent || (tasTimeout &&
//     posTimeout)))`, ~line 117) - a real cross-fusion-type coupling with
//     GPS position's own `posTimeout` bool, a different, wall-clock-driven
//     concept from this port's last_pos_pass_time_s elapsed-time field
//     (CPP-058) that this port does not attempt to wire together (matching
//     the ticket's own explicit instruction). fuse_airspeed() uses the
//     simpler `isConsistent` condition alone - a gate failure is never
//     force-fused, unlike upstream's real behavior during a simultaneous
//     airspeed-and-GPS-position outage.
//   - `badIMUdata`-driven forced-fusion bypass (`|| badIMUdata` in
//     `isConsistent`'s real definition) - already-established permanently-
//     false exclusion (phase 1 simplification 3), same treatment as every
//     prior phase.
//   - `dvelBiasAxisInhibit[]` per-axis accel-bias-state narrowing inside
//     kalman_mask - already a named phase-1/2/5 exclusion (single
//     inhibit_del_vel_bias_states flag covering all 3 axes together); see
//     "A NOTABLE STRUCTURAL CONSEQUENCE" above.
//   - `SelectTasFusion()`'s own orchestration (~line 158-176 of the same
//     upstream file) - the magFusePerformed-driven `airSpdFusionDelayed`
//     one-tick-slip logic, `readAirSpdData()` (see below), and the
//     `tasDataToFuse && statesInitialised && !inhibitWindStates` call
//     condition - this ticket builds FuseAirspeed() itself, callable
//     directly, matching this port's established "caller decides when to
//     call" convention (same as every other fusion function built so far:
//     fuse_gps_velocity()/fuse_gps_position()/fuse_magnetometer()/
//     fuse_baro_height()).
//   - `readAirSpdData()`/`tasDataDelayed`/`storedTAS` (AP_NavEKF3_
//     Measurements.cpp, already an established phase-1 exclusion - "no
//     fusion time-horizon delay buffer... multi-sensor sample buffering")
//     - `tasDataDelayed.allowFusion` (real trigger: `airspeed->healthy(...)
//     && airspeed->use(...)`, the real physical-sensor branch, ~line 869)
//     is therefore always treated as true here: the caller is expected to
//     have already validated sensor health before calling
//     fuse_airspeed(), exactly matching GpsSample/MagSample/baro_altitude_m's
//     own established "caller supplies an already-known-good reading"
//     convention. The synthetic-airspeed fallback branches
//     (`assume_zero_sideslip()`'s `defaultAirSpeed`/`lastAspdEstIsValid`
//     paths, ~line 884-911, for vehicles/configurations with no physical
//     airspeed sensor at all) are also out of scope for the same reason -
//     this port's caller always supplies a real reading when it calls this
//     function, there is no "no sensor, fall back to a model estimate"
//     mode here.
//   - `EAS2TAS` (air-density-derived EAS-to-TAS conversion) - see "EAS2TAS -
//     NOT MODELED" above.
//   - `faultStatus.bad_airspeed` bookkeeping - write-only diagnostic flag,
//     already-established exclusion pattern (same as every faultStatus.bad_*
//     field since phase 5's own precedent) - fuse_airspeed()'s bool return
//     value is this port's only fault signal, same treatment.
// ============================================================================

// ============================================================================
// CPP-065, PHASE 11 (this ticket): real mag/wind covariance growth and
// runtime-aware constrain_variances(). Fixes, together (fixing only one is
// provably insufficient, per CPP-064's own analysis), the root cause CPP-064
// (phase 10) diagnosed for "state.wind_vel never moves" and CPP-059 (phase
// 5) diagnosed for "earth-field learning capped at one call": (1)
// constrain_variances() unconditionally zeroed the mag/wind covariance
// block (states 16-23) every call regardless of the runtime
// inhibit_mag_states/inhibit_wind_states flags, and (2)
// covariance_prediction() never populated those states' own cross-terms or
// gave them any process noise in the first place.
//
// WHAT THIS PHASE BUILDS:
//   - covariance_prediction()'s dense Jacobian block extended to columns
//     16-23, VERBATIM-TRANSCRIBED from upstream (see that function's own
//     banner for the verification account) and gated by state_index_lim()
//     exactly as upstream gates by stateIndexLim - CPP-056's existing
//     function, reused directly, not reinvented.
//   - The general process-noise loop's bound changed from a hardcoded 15
//     to state_index_lim(), and the real magEarthVar/magBodyVar/windVelVar
//     formulas (see covariance_prediction()'s own banner) now populate
//     process_noise_variance[6..13] when the corresponding inhibit flag is
//     clear.
//   - A real, persistent hgt_rate member (see its own field comment) -
//     upstream's real hgtRate is a 10-second-time-constant EMA filter of
//     down-velocity, NOT a raw state.velocity.z alias; verified directly
//     before deciding to port it as a real filtered member rather than
//     approximating it away.
//   - constrain_variances()'s mag (16-21) and wind (22-23) blocks replaced
//     with upstream's real per-state-group if/else structure (clamp when
//     active, zero when inhibited) instead of the old phase-1 unconditional
//     zeroing.
//   - Four new noise-parameter fields (mag_earth_process_noise,
//     mag_body_process_noise, wind_vel_process_noise,
//     wind_var_hgt_rate_scale) and one new constant (kWindVelVarianceMax) -
//     see their own comments for the real upstream defaults/provenance.
//
// VERIFICATION STANDARD FOR THIS PHASE: given this touches
// covariance_prediction()'s dense PS0..PS222 Jacobian block - deliberately
// left untouched by every phase since CPP-052 (phase 1) specifically
// because it is numerically-sensitive, machine-derived algebra - this
// phase's own commit message documents substantially more spot-checks
// than this port's usual 2-4, including a full whitespace-normalized
// programmatic diff of the extended block against the real upstream text
// (not just eyeballing a handful of lines) before it was ever inserted.
//
// TICKET PREMISE CHECK: every real formula/constant/line-range this
// ticket's own text asserted (the PS-coefficient reuse for columns 16-23,
// the stateIndexLim>15/>21 nesting, MAGE_P_NSE_DEFAULT=1.0E-03,
// MAGB_P_NSE_DEFAULT=1.0E-04, WIND_P_NSE_DEFAULT=0.1,
// _wndVarHgtRateScale default 1.0f, WIND_VEL_VARIANCE_MAX=400.0, the real
// ConstrainVariances() if/else structure) was verified directly against
// the pinned Plane-4.7.0 source and confirmed accurate - no correction to
// the ticket's own premise was needed this round, unlike some prior
// phases. The one genuinely open question the ticket posed (hgtRate's
// real source) was investigated directly and resolved: it is a real,
// separate, persistent filtered member, not an alias - see hgt_rate's own
// field comment.
//
// EXPLICITLY OUT OF SCOPE (each with its real upstream trigger, matching
// this port's established disclosure convention):
//   - `needMagBodyVarReset`/`needEarthBodyVarReset`-triggered P-block
//     resets and the `FuseDeclination(radians(20.0f))` call the latter
//     makes - ties to yaw-realignment machinery, already excluded since
//     phases 5/6 (see covariance_prediction()'s own banner for exactly
//     where this was skipped).
//   - `treatWindStatesAsTruth`/`isDragFusionDeadReckoning`/
//     `windStateIsObservable` - already-established exclusion (phase 2,
//     reconfirmed phases 9/10); this port always takes upstream's real
//     "normal" (non-treat-as-truth) branch, both in the process-noise
//     computation and in constrain_variances()'s wind clamp.
//   - `tasDataDelayed.allowFusion`-gated 10x wind-noise scaling for a
//     failed airspeed sensor - no airspeed-sensor-health state modeled
//     (already-established phase-9 exclusion pattern).
//   - `dvelBiasAxisInhibit[]`-gated covariance reset inside
//     CovariancePrediction() itself (distinct from ConstrainVariances()'s
//     own accel-bias handling) - already a named phase-1/2 gap
//     (this port's inhibit_del_vel_bias_states is all-or-nothing across
//     x/y/z, with no per-axis ground-alignment inhibiting to ever trigger
//     the real upstream branch) - freshly named at its exact insertion
//     point in covariance_prediction() since this is the first phase to
//     touch that region of the function.
//   - inhibit_del_ang_bias_states/inhibit_del_vel_bias_states are still
//     NOT wired into covariance_prediction()'s column-10..15 handling
//     (that loop runs unconditionally, matching phase 1's original
//     behavior) - a real, narrower remaining gap, out of this ticket's
//     scope, harmless today since neither flag is ever set true anywhere
//     in this port (see the "CPP-056, PHASE 2" banner's own updated note).
//
// REAL RESULT (see this ticket's commit message for the full numbers):
// with inhibit_mag_states/inhibit_wind_states cleared, P[16][16]/P[22][22]
// now genuinely grow tick-over-tick instead of staying frozen, and
// CPP-059's/CPP-064's own closed-loop tests were re-run and their new,
// honestly-reported numbers are in the commit message - whatever was
// actually found, not assumed.
// ============================================================================

#include <array>
#include <cmath>
#include <cstdint>

#include <fwcpp/ekf/ekf_buffer.hpp>
#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::ekf {

#if FWCPP_EKF_DOUBLE
using ftype = double;
#else
using ftype = float;
#endif

using Vector2F = fwcpp::math::Vector2<ftype>;
using Vector3F = fwcpp::math::Vector3<ftype>;
using Matrix3F = fwcpp::math::Matrix3<ftype>;
using QuaternionF = fwcpp::math::QuaternionT<ftype>;

// upstream: sq(T), AP_Math/AP_Math.h - trivial, not worth a shared header
// dependency for one line.
template <typename T>
[[nodiscard]] constexpr T sq(T x) { return x * x; }

// upstream: NavEKF3_core::calcEarthRateNED(), AP_NavEKF3_core.cpp ~line
// 2066. earthRate constant: AP_NavEKF3_core.h:49, `#define earthRate
// 0.000072921f` - an explicitly float-precision literal even in a
// double-ftype build (upstream's own choice, reproduced exactly via the
// float() narrowing below rather than widened to full double precision).
[[nodiscard]] inline Vector3F calc_earth_rate_ned(ftype latitude_rad) {
    constexpr ftype kEarthRate = static_cast<ftype>(0.000072921f);
    return Vector3F(kEarthRate * std::cos(latitude_rad), ftype(0),
                     -kEarthRate * std::sin(latitude_rad));
}

// upstream: imu_elements' delAng/delAngDT fields (AP_NavEKF3_core.h ~589-
// 591), reshaped into AhrsDcm's GyroSample/AccelSample split rather than
// upstream's single combined imu_elements struct - see file banner's
// "NO SINGLETONS" section.
struct GyroSample {
    Vector3F delta_angle;      // upstream: imuDataDelayed.delAng, rad
    ftype delta_angle_dt = 0;  // upstream: imuDataDelayed.delAngDT, s
};

struct AccelSample {
    Vector3F delta_velocity;      // upstream: imuDataDelayed.delVel, m/s
    ftype delta_velocity_dt = 0;  // upstream: imuDataDelayed.delVelDT, s
};

// ============================================================================
// CPP-067, PHASE 13 (this ticket): time-correct GPS sample recall via
// ObsBuffer. Phase 12 (CPP-066) built fwcpp::ekf::ObsBuffer<T,N>/
// ImuBuffer<T,N> (ekf_buffer.hpp) as standalone, UNWIRED infrastructure -
// this is its FIRST real consumer. Read directly before writing any code
// (per the ticket's own instruction): AP_NavEKF3_Measurements.cpp
// readGpsData() (~line 560-735, storedGPS.push(gpsDataNew) at the very end
// of the `if (validOrigin && !waitingForGpsChecks)` block, ~line 733) and
// AP_NavEKF3_PosVelFusion.cpp SelectVelPosFusion()'s `gpsDataToFuse =
// storedGPS.recall(gpsDataDelayed, imuDataDelayed.time_ms) &&
// !waitingForGpsChecks;` (~line 534) and its surrounding context
// (~line 500-560). Both verified to match the ticket's own line-number
// summary closely enough that no correction to those citations is needed.
//
// THE SCOPING DECISION ALREADY MADE (not revisited here): upstream's FULL
// architecture - EkfCore's own state representing "now minus a fusion
// delay", plus the output-state complementary-filter blending
// (AP_NavEKF3_Outputs.cpp) that extrapolates the delayed state back to
// present time for consumption - is a much larger, multi-phase
// undertaking, deliberately deferred. This phase does something
// narrower: use ObsBuffer to make GPS fusion recall the CORRECT sample by
// timestamp relative to the CALLER'S OWN CURRENT TIME (`now_s`, this
// port's existing CPP-058 convention), not upstream's real delayed-IMU-
// time-horizon (`imuDataDelayed.time_ms`, which does not exist here). This
// is a real, disclosed simplification relative to upstream's true
// delayed-horizon recall - but a genuine, meaningful improvement over what
// existed before this phase (fuse_gps_velocity()/fuse_gps_position()
// could only fuse a GpsSample the caller handed them synchronously, on
// the exact tick they wanted it fused, with zero tolerance for GPS
// arriving asynchronously relative to the IMU tick).
//
// WHAT THIS PHASE BUILDS:
//   - GpsSample now derives from ObsElement and gains time_s()/
//     set_time_s() (see GpsSample's own comment below for the real,
//     disclosed ms-vs-seconds tension this resolves).
//   - EkfCore::gps_buffer, an ObsBuffer<GpsSample, kGpsBufferCapacity>
//     member (see EkfCore's own comment next to it for the buffer-size
//     reasoning: a realistic 5-10Hz GPS rate, this port's own
//     kGpsPeriodTicks closed-loop-test precedent, times a buffering
//     window sized to tolerate realistic push/recall cadence mismatch).
//   - EkfCore::push_gps_sample(const GpsSample&): pushes a
//     caller-timestamped sample into gps_buffer, decoupled from the
//     EKF's own per-tick fusion cadence - the new thing callers do AS
//     SAMPLES ARRIVE.
//   - EkfCore::recall_gps_sample(GpsSample&, ftype now_s): recalls the
//     time-correct sample from gps_buffer using now_s. See its own
//     comment for why this is ONE combined recall primitive rather than
//     two independently-recalling "fuse_gps_velocity_buffered()"/
//     "fuse_gps_position_buffered()" methods (recall() is destructive;
//     upstream itself recalls once per tick and feeds the one result to
//     both the velocity and position fusion paths - two independent
//     recalls against the same shared queue would be a real correctness
//     bug, not a style choice).
//
// THE KEEP-VS-REPLACE DECISION FOR fuse_gps_velocity()/fuse_gps_position():
// KEPT UNCHANGED, verbatim, both signature and behavior. ~20 existing call
// sites across ekf_fusion_test.cpp/ekf_closed_loop_test.cpp call these
// directly with a hand-built GpsSample - every one of them keeps compiling
// and passing byte-for-byte, unmodified, with zero churn. The new
// capability is delivered entirely through the two ADDITIONS above
// (push_gps_sample()/recall_gps_sample()): a caller who wants buffered,
// asynchronous-arrival-tolerant GPS fusion calls recall_gps_sample() once
// per tick and, on success, feeds the recalled GpsSample to the SAME
// unchanged fuse_gps_velocity()/fuse_gps_position() every other caller
// already uses. This is the option the ticket itself asks to weigh
// ("pick whichever delivers the real capability with the least
// unnecessary churn") - replacing the two functions' signatures instead
// would have forced every existing test to route through a buffer it
// doesn't need, for no behavioral benefit.
//
// EXPLICITLY OUT OF SCOPE (each with its real upstream trigger, per the
// ticket's own acceptance criterion):
//   - Redefining EkfCore's own state/covariance as representing a delayed
//     time horizon rather than "now" (upstream: the whole
//     imuDataDelayed/output-complementary-filter mechanism) - the full
//     architectural redesign, deliberately deferred to a future phase.
//   - IMU sample buffering/downsampling/multi-instance switching
//     (upstream: imuDataDownSampledNew, multiple IMU instances/lanes) -
//     already established as moot for this port by ADR-0012's
//     explicit-input convention (callers already supply one clean,
//     single-instance, pre-downsampled sample per tick) - see
//     ekf_buffer.hpp's own "CORRECTION TO THIS TICKET'S OWN PREMISE" for
//     the full discussion of why upstream even needs this at all.
//   - The output-state complementary-filter blending
//     (AP_NavEKF3_Outputs.cpp) a true delayed-state architecture would
//     need to produce a "present time" estimate for consumption - not
//     needed here since this phase does not introduce a delayed state.
//   - Applying this same buffered-recall pattern to magnetometer/baro/
//     airspeed (upstream: storedMag/storedBaro/storedTAS, the same
//     EKF_obs_buffer_t<T> machinery) - GPS only, in this phase, to keep
//     it bounded. A likely, disclosed NEXT ticket, not this one.
//     CPP-068 UPDATE (phase 14): the magnetometer portion of this
//     exclusion is now LIFTED - see this file's "CPP-068, PHASE 14"
//     banner (above EkfCore::mag_buffer) for push_mag_sample()/
//     recall_mag_sample(), the identical pattern applied to
//     magnetometer fusion. baro/airspeed remain out of scope, unchanged.
//   - The `waitingForGpsChecks`-gated startup logic (upstream:
//     SelectVelPosFusion()'s own `&& !waitingForGpsChecks` conjunct on
//     the recall-result line, verified directly, ~line 534) - ties to
//     initial-alignment/pre-arm checking machinery this port does not
//     model at all, an already-established exclusion pattern (same
//     treatment as badIMUdata/EK3_FEATURE_EXTERNAL_NAV elsewhere in this
//     file).
// ============================================================================

// CPP-056 phase 2. upstream: gps_elements (AP_NavEKF3_core.h), the subset
// FuseVelPosNED()'s obsIndex 0-4 path actually reads (gpsDataDelayed.vel,
// and the local-NE position AP_NavEKF3_PosVelFusion.cpp ~line 574-580
// computes as `EKF_origin.get_distance_NE_ftype(gpsloc)` from the raw
// lat/lng fix). See this file's banner ("GpsSample vs. ahrs::GpsSample")
// for why this is a new struct, not a reuse of ap-ahrs's GpsSample.
//
// position_ne is ALREADY the local-NE projection, in metres relative to
// whatever local origin the caller's EKF instance uses - the raw-fix-to-
// local-NE projection itself (upstream: Location::get_distance_NE_ftype(),
// tied to the EKF_origin/home-location singleton) is explicitly out of
// scope for this phase (see banner) - the caller must have already done
// that conversion, matching this port's general explicit-input
// convention (ADR-0012).
// CPP-067 phase 13 addendum: GpsSample now also satisfies fwcpp::ekf::
// ObsElement (phase 12/CPP-066 ekf_buffer.hpp's ObsBuffer<T,N> element
// contract - see that file's own static_assert) so it can be stored in
// the EkfCore::gps_buffer member added below, letting GPS fusion RECALL
// the correct sample by timestamp instead of requiring the caller to
// hand-feed exactly the right sample synchronously on every fusion call
// - see this file's "CPP-067, PHASE 13" banner and ticket CPP-067 for
// the full scope/reasoning.
//
// A REAL TENSION, RESOLVED EXPLICITLY: the ticket asks for "a real
// timestamp field... seconds as ftype, matching now_s/dt_ekf_avg... not
// upstream's raw uint32_t milliseconds" AND "satisfy phase 12's
// ObsElement contract". Those two asks are in genuine tension:
// ObsElement's contract (ekf_buffer.hpp, matching upstream's real
// EKF_obs_element_t) IS a `std::uint32_t time_ms` field, and
// ObsBuffer<T,N>::recall()'s algorithm (a hardcoded 100ms acceptance
// window, uint32_t-subtract-then-reinterpret-as-int32_t dt arithmetic)
// is written in terms of that exact field, not a generic template
// parameter - it is what upstream's real ring buffer does. Rather than
// give GpsSample two independently-settable timestamp fields (a real
// duplication/drift hazard - nothing would stop a caller from setting
// one and not the other), GpsSample derives from ObsElement (inheriting
// the one, literal `time_ms` field ObsBuffer's push()/recall() read and
// write) and exposes time_s()/set_time_s() as the ONLY caller-facing
// accessors, in this port's own now_s-style seconds convention
// (ADR-0012/CPP-058 precedent). There is exactly one stored timestamp,
// never two that could disagree; time_s()/set_time_s() are a millisecond-
// quantized view over it (matching ObsBuffer/upstream's own ms
// resolution - not sub-millisecond-precise, a disclosed quantization,
// not a bug).
struct GpsSample : public ObsElement {
    Vector3F velocity_ned;  // upstream: gpsDataDelayed.vel, NED m/s
    Vector2F position_ne;   // upstream: velPosObs[3]/[4] source value, local NE metres

    // Caller-facing timestamp accessors - see banner above for why these
    // exist instead of reading/writing the inherited `time_ms` directly.
    // Negative `t` is clamped to 0 before conversion (rather than letting
    // a cast-to-uint32_t of a negative value be implementation-defined/
    // UB-adjacent) - `now_s` is never legitimately negative in this
    // port's own convention (elapsed simulated time from an arbitrary
    // zero, ADR-0012), so this is a defensive clamp, not a real case this
    // port expects to hit.
    void set_time_s(ftype t) {
        const ftype clamped_s = t > ftype(0) ? t : ftype(0);
        time_ms = static_cast<std::uint32_t>(clamped_s * ftype(1000));
    }
    [[nodiscard]] ftype time_s() const { return static_cast<ftype>(time_ms) / ftype(1000); }
};

// CPP-059 phase 5. upstream: mag_elements' `mag` field (AP_NavEKF3_core.h),
// the one field FuseMagnetometer() actually reads via magDataDelayed.mag -
// see this file's "CPP-059, PHASE 5" banner below for the full discussion.
// Checked directly, per the ticket's own instruction, before adding this:
// ap-compass's Compass class (modules/ap-compass/include/fwcpp/compass/
// compass.hpp) takes a raw body-frame Vector3f via update(), with no
// dedicated sample struct of its own to reuse - so this is a genuinely new
// explicit input (ADR-0012), not a duplicate of an existing one, same
// reasoning CPP-056's own GpsSample-vs-ahrs::GpsSample discussion above
// already established for GPS.
//
// CPP-068 phase 14 addendum: MagSample now also derives from ObsElement
// and gains set_time_s()/time_s(), the IDENTICAL pattern CPP-067 (phase
// 13) established for GpsSample above - reused directly, not reinvented.
// The same real tension GpsSample's own comment resolves applies here
// unchanged (ObsElement's contract IS a uint32_t time_ms field;
// ObsBuffer<T,N>::recall()'s algorithm is written in terms of that exact
// field) and is resolved the identical way: one single stored timestamp
// (the inherited time_ms), time_s()/set_time_s() as the only caller-facing
// accessors, negative input clamped to 0 before the uint32_t cast. See
// this file's "CPP-068, PHASE 14" banner (below EkfCore::mag_buffer) for
// the full scope/reasoning writeup.
struct MagSample : public ObsElement {
    Vector3F mag;  // upstream: magDataDelayed.mag, body-frame gauss

    // See GpsSample::set_time_s()/time_s() above - identical convention,
    // reused verbatim for MagSample.
    void set_time_s(ftype t) {
        const ftype clamped_s = t > ftype(0) ? t : ftype(0);
        time_ms = static_cast<std::uint32_t>(clamped_s * ftype(1000));
    }
    [[nodiscard]] ftype time_s() const { return static_cast<ftype>(time_ms) / ftype(1000); }
};

// upstream: state_elements, AP_NavEKF3_core.h:566-575. Field order and
// element-count comments match upstream's own byte-for-byte (quat 0..3,
// velocity 4..6, position 7..9, gyro_bias 10..12, accel_bias 13..15,
// earth_magfield 16..18, body_magfield 19..21, wind_vel 22..23 = 24
// total) - read directly from upstream, not assumed from the ticket's own
// summary of it.
struct StateVector {
    QuaternionF quat;               // 0..3   - local NED earth frame -> body frame
    Vector3F velocity;               // 4..6   - NED earth frame, m/s
    Vector3F position;               // 7..9   - NED earth frame, m
    Vector3F gyro_bias;              // 10..12 - body frame delta angle IMU bias, rad
    Vector3F accel_bias;             // 13..15 - body frame delta velocity IMU bias, m/s
    Vector3F earth_magfield;         // 16..18 - PHASE 1: zero-initialized and carried, never fused (see file banner)
    Vector3F body_magfield;          // 19..21 - PHASE 1: zero-initialized and carried, never fused
    Vector2F wind_vel;                // 22..23 - PHASE 1: zero-initialized and carried, never fused
};

// upstream: Matrix24, AP_NavEKF3_core.h:558 (`typedef ftype
// Matrix24[24][24]`) - a plain fixed 24x24, not this port's own VectorN.
using Matrix24 = std::array<std::array<ftype, 24>, 24>;

// EkfCore: the strapdown-INS-plus-covariance-prediction core described in
// this file's banner. NOT a drop-in AhrsDcm replacement (see banner's
// "explicitly out of scope" list) - a standalone estimator with no fusion,
// health/mode logic, or Plane wiring.
class EkfCore {
public:
    StateVector state{};
    Matrix24 P{};  // upstream: P (member of NavEKF3_core), covariance matrix

    // upstream: earthRateNED (member), set once via calcEarthRateNED() when
    // a home location becomes known - see file banner's simplification 7.
    // Zero (no earth-rotation compensation) until the caller sets it.
    Vector3F earth_rate_ned{};

    // upstream: prevTnb (member) - nav-to-body DCM from the previous
    // strapdown step, used both for earth-rate compensation and for
    // rotating the current step's body-frame delta-velocity into nav
    // frame. Starts as the identity (matches a freshly-constructed
    // identity-quaternion state).
    Matrix3F prev_tnb{Vector3F(ftype(1), ftype(0), ftype(0)), Vector3F(ftype(0), ftype(1), ftype(0)),
                       Vector3F(ftype(0), ftype(0), ftype(1))};

    // upstream: velDotNED/velDotNEDfilt (members) - computed by
    // UpdateStrapdownEquationsNED() for launch-detection/GPS-variance
    // consumers, neither of which exists in this phase. Exposed for
    // inspection/testing but not otherwise consumed here.
    Vector3F vel_dot_ned{};
    Vector3F vel_dot_ned_filt{};

    // upstream: PV_AidingMode != AID_NONE - collapsed to one bool since
    // this phase has exactly two states (aiding is always false: no
    // fusion of any kind exists to ever set it true). Exposed (not
    // hardcoded away) so phase 2 has an obvious place to start driving it
    // from real fusion-health logic.
    bool aiding = false;

    // upstream: vertVelVarClipCounter (member), ConstrainVariances()'s
    // persistent counter for the "vertical velocity variance has
    // collapsed" reset.
    std::uint32_t vert_vel_var_clip_counter = 0;

    // CPP-065 phase 11. upstream: hgtRate (member, AP_NavEKF3_core.cpp:266
    // zero-init), the filtered height-rate quantity CovariancePrediction()
    // itself maintains and consumes for wind-state process noise scaling
    // (~line 1038-1040, real comment: "use filtered height rate to increase
    // wind process noise when climbing or descending. Filter height rate
    // using a 10 second time constant filter"; real formula: alpha =
    // 0.1*dt; hgtRate = hgtRate*(1-alpha) - stateStruct.velocity.z*alpha).
    // VERIFIED DIRECTLY this is NOT a raw alias for state.velocity.z (down-
    // velocity) - it is a real, persistent, exponentially-filtered member
    // updated once per covariance_prediction() call, in NED down-velocity
    // units (m/s). Ported as a real member (not approximated as an
    // instantaneous velocity read) since the filter itself is simple,
    // well-specified, and directly affects the new windVelVar formula
    // below.
    ftype hgt_rate = 0;  // upstream: hgtRate

    // CPP-056 phase 2: real inhibitDelAngBiasStates/inhibitDelVelBiasStates/
    // inhibitMagStates/inhibitWindStates fields (AP_NavEKF3_core.h members,
    // set by AP_NavEKF3_Control.cpp - out of scope). Defaults reproduce
    // phase 1's existing hardcoded behavior exactly - see this file's
    // banner "CORRECTION TO THE CPP-056 TICKET'S OWN PREMISE" for why
    // these did not already exist and why constrain_variances()/
    // covariance_prediction() are not (yet) wired to read them.
    bool inhibit_del_ang_bias_states = false;  // upstream: inhibitDelAngBiasStates
    bool inhibit_del_vel_bias_states = false;  // upstream: inhibitDelVelBiasStates
    bool inhibit_mag_states = true;            // upstream: inhibitMagStates
    bool inhibit_wind_states = true;           // upstream: inhibitWindStates

    // CPP-058 phase 4: upstream lastVelPassTime_ms/lastGpsPosPassTime_ms
    // (AP_NavEKF3_core.h:1137-1138), elapsed-time bookkeeping used to
    // detect a sustained GPS outage (see this file's "CPP-058, PHASE 4"
    // banner). Seconds, per this port's own dt_ekf_avg-style
    // caller-supplied-time convention (ADR-0012) - stamped from
    // fuse_gps_velocity()'s/fuse_gps_position()'s `now_s` parameter,
    // never a real-time-clock read. Zero-initialized, matching upstream's
    // own AP_NavEKF3_core.cpp ~line 205-206 reset-on-init behavior
    // (`lastVelPassTime_ms = 0; lastGpsPosPassTime_ms = 0;`).
    ftype last_vel_pass_time_s = 0;  // upstream: lastVelPassTime_ms
    ftype last_pos_pass_time_s = 0;  // upstream: lastGpsPosPassTime_ms

    // CPP-067 phase 13. upstream: NavEKF3_core::storedGPS
    // (EKF_obs_buffer_t<gps_elements>, AP_NavEKF3_core.h), sized at
    // RUNTIME in setup_core() - see ekf_buffer.hpp's own "CORRECTION TO
    // THIS TICKET'S OWN PREMISE" for why a runtime size is not
    // reproduced here (compile-time N, ADR-0012 decision 4 - the same
    // no-dynamic-allocation adaptation ekf_buffer.hpp itself already
    // discloses).
    //
    // BUFFER SIZE REASONING (not an arbitrary round number): this port's
    // own established closed-loop-test precedent
    // (ekf_closed_loop_test.cpp: kGpsPeriodTicks) models GPS at 5Hz
    // (200ms period) against a 50Hz IMU tick (20ms); the ticket's own
    // stated realistic range is 5-10Hz (100-200ms period). A
    // well-behaved caller calling push_gps_sample() once per real GPS
    // fix and calling recall_gps_sample() once per EKF tick (far faster
    // than either GPS rate) will, under normal operation, never have
    // more than ONE unconsumed sample resident at a time - recall() is
    // destructive (ekf_buffer.hpp) and, if attempted every tick, a
    // freshly-pushed sample is consumed on the very next call that
    // reaches it. kGpsBufferCapacity=4 is sized to comfortably absorb
    // realistic deviations from that ideal case without silently losing
    // data:
    //   - up to 2 samples pushed back-to-back before an intervening
    //     successful recall (e.g. a GPS driver emitting two fixes in
    //     quick succession, or the caller skipping a tick or two before
    //     calling recall_gps_sample() again) at the FASTER end of the
    //     stated realistic range (10Hz = 100ms period - the worst case
    //     for "how fast can samples pile up"),
    //   - PLUS a 2x safety margin on top of that, so a transient hiccup
    //     does not silently start discarding valid GPS fixes via push()'s
    //     own overwrite-oldest-at-capacity behavior (ekf_buffer.hpp).
    // This is intentionally generous relative to the "<=1 resident
    // sample" steady-state case above; N is a compile-time std::array
    // size (no dynamic allocation, ADR-0012), so a few extra
    // GpsSample-sized ring slots cost negligible memory.
    static constexpr std::size_t kGpsBufferCapacity = 4;

    // The actual buffer - see push_gps_sample()/recall_gps_sample() below
    // for how it's filled/drained. Public, matching this file's own
    // established convention of exposing internal state for direct test
    // verification (same treatment as P/state/last_vel_pass_time_s
    // above) - a test can inspect gps_buffer.size()/.empty() directly
    // rather than only observing recall's effect indirectly through
    // fusion counts.
    ObsBuffer<GpsSample, kGpsBufferCapacity> gps_buffer;

    // ========================================================================
    // CPP-068, PHASE 14 (this ticket): extends phase 13 (CPP-067)'s
    // buffered, time-correct recall pattern to magnetometer fusion. Read
    // directly before writing any code (per the ticket's own instruction):
    // AP_NavEKF3_MagFusion.cpp SelectMagFusion()'s `magDataToFuse =
    // storedMag.recall(magDataDelayed,imuDataDelayed.time_ms);` (~line
    // 411, verified directly - matches the ticket's own ~395-420 estimate)
    // and AP_NavEKF3_Measurements.cpp's `storedMag.push(magDataNew);`
    // (~line 377, the last statement before `lastMagRead_ms = ...;` inside
    // the compass-read block - verified directly, matches the ticket's own
    // ~370-395 estimate).
    //
    // THE SECOND storedMag.recall() CALL SITE - VERIFIED, CONFIRMED MOOT:
    // storedMag.recall() is called in exactly TWO places in the real
    // upstream source (grepped directly across both files named above,
    // not assumed): SelectMagFusion() (~line 411, the one ported here) and
    // AP_NavEKF3_MagFusion.cpp's learnMagBiasFromGPS() (~line 1376: `if
    // (!storedMag.recall(mag_data, imuDataDelayed.time_ms)) { ... }`,
    // read directly - a completely separate function that learns
    // magnetometer hard-iron bias from GPS-derived yaw, gated on
    // have_table_earth_field/inFlight, feeding a World-Magnetic-Model
    // earth-field table lookup this port does not have). This port's own
    // CPP-059 phase 5 banner ALREADY names learnMagBiasFromGPS() as
    // excluded ("a separate, unrelated mag-bias learning mechanism
    // (verified: a distinct function, not called from within
    // FuseMagnetometer()), not part of this ticket") - confirmed again
    // here, directly, rather than assumed: this port has no
    // learnMagBiasFromGPS() equivalent at all, so recall_mag_sample()
    // below (SelectMagFusion()'s ported call) is the ONLY consumer of
    // mag_buffer. CPP-067's own central insight - ObsBuffer::recall() is
    // DESTRUCTIVE, so two independent callers draining the same queue
    // would be a real correctness bug, not a style question - does NOT
    // apply here: there is exactly one real consumer, matching CPP-067's
    // own GPS precedent (storedGPS.recall() also has exactly one real
    // call site, SelectVelPosFusion()).
    //
    // BUFFER SIZE REASONING - INDEPENDENTLY RE-DERIVED FOR MAGNETOMETER,
    // NOT COPIED FROM CPP-067's GPS-derived N=4: CPP-067's own
    // kGpsBufferCapacity=4 was derived from GPS's ticket-stated 5-10Hz
    // realistic range, using the FASTER end (10Hz = 100ms period, the
    // worst case for "how fast can unconsumed samples pile up") as the
    // base case. Magnetometer fusion has no such stated range in this
    // ticket - this port's own real, established precedent instead is a
    // single fixed rate: ekf_closed_loop_test.cpp's kMagPeriodTicks =
    // kTicksPerSecond/10 = 10Hz (100ms period), the ONLY magnetometer
    // fusion rate this port has ever exercised in a closed-loop
    // integration test (CPP-059/CPP-061's own established precedent,
    // cited directly per the ticket's own instruction). Applying the
    // SAME reasoning CPP-067 used (worst-case back-to-back pushes before
    // an intervening successful recall, PLUS a 2x safety margin against a
    // transient hiccup silently discarding valid readings via push()'s
    // overwrite-oldest-at-capacity behavior) to mag's own real 10Hz rate:
    //   - up to 2 samples pushed back-to-back before an intervening
    //     successful recall (recall_mag_sample() is attempted every EKF
    //     tick, far faster than 10Hz, so this covers a caller skipping a
    //     tick or two, or a compass driver emitting two readings in quick
    //     succession) at mag's own 100ms period,
    //   - PLUS the same 2x safety margin.
    // This independently lands on kMagBufferCapacity=4 - the SAME numeral
    // as CPP-067's kGpsBufferCapacity, but arrived at via mag's own real
    // 10Hz rate, not by copying GPS's reasoning unexamined. This is not a
    // coincidence to be embarrassed by: it is the DIRECT, disclosed
    // consequence of mag's own established 100ms period being
    // numerically identical to the fastest-end 100ms period that already
    // drove GPS's own derivation (10Hz in both cases) - had this port's
    // own magnetometer closed-loop precedent instead been, say, 25Hz
    // (40ms period), the same worst-case-plus-margin reasoning would have
    // produced a different N. A compile-time std::array size (no dynamic
    // allocation, ADR-0012 decision 4), same as gps_buffer.
    static constexpr std::size_t kMagBufferCapacity = 4;

    // The actual buffer - see push_mag_sample()/recall_mag_sample() below.
    // Public, matching gps_buffer's own established convention (direct
    // test inspection via mag_buffer.size()/.empty()).
    ObsBuffer<MagSample, kMagBufferCapacity> mag_buffer;
    // ========================================================================

    // CPP-059 phase 5. upstream: innovMag/varInnovMag (NavEKF3_core
    // members) - see this file's "CPP-059, PHASE 5" banner. Public so
    // tests can verify the dense per-axis formulas independently, per the
    // ticket's own verification standard (same treatment as gps_vel_test_
    // ratio()/gps_pos_test_ratio() above).
    Vector3F innov_mag{};      // upstream: innovMag
    Vector3F var_innov_mag{};  // upstream: varInnovMag

    // CPP-063 phase 9. upstream: innovVtas/varInnovVtas (NavEKF3_core
    // members) - see this file's "CPP-063, PHASE 9" banner ("tas_test_
    // ratio() IS MAG-SHAPED"). Scalars, not Vector3F, since airspeed fusion
    // has exactly one obsIndex.
    ftype innov_vtas{};      // upstream: innovVtas
    ftype var_innov_vtas{};  // upstream: varInnovVtas

    // --- Noise/limit parameters. Defaults are upstream's real
    // Plane-4.7.0 AP_NavEKF3.cpp APM_BUILD_ArduPlane parameter-default
    // block (traced directly, not invented) - "AP_Param not wired in
    // yet", same treatment as AhrsDcm's kp_yaw. ---
    ftype gyr_noise = static_cast<ftype>(1.5e-02f);          // GYRO_P_NSE_DEFAULT
    ftype acc_noise = static_cast<ftype>(3.5e-01f);          // ACC_P_NSE_DEFAULT
    ftype gyro_bias_process_noise = static_cast<ftype>(1.0e-03f);   // GBIAS_P_NSE_DEFAULT
    ftype accel_bias_process_noise = static_cast<ftype>(2.0e-02f);  // ABIAS_P_NSE_DEFAULT
    ftype gps_horiz_vel_noise = static_cast<ftype>(0.5f);    // VELNE_M_NSE_DEFAULT
    ftype gps_vert_vel_noise = static_cast<ftype>(0.7f);     // VELD_M_NSE_DEFAULT
    ftype gps_horiz_pos_noise = static_cast<ftype>(0.5f);    // POSNE_M_NSE_DEFAULT
    ftype baro_alt_noise = static_cast<ftype>(3.0f);         // ALT_M_NSE_DEFAULT
    ftype mag_noise = static_cast<ftype>(0.05f);             // MAG_M_NSE_DEFAULT (init value only - see banner simplification 1)
    ftype acc_bias_lim = static_cast<ftype>(1.0f);           // AP_GROUPINFO("ACC_BIAS_LIM", ..., 1.0f)

    // CPP-065 phase 11. upstream: AP_NavEKF3.cpp APM_BUILD_ArduPlane
    // #elif block (verified directly, lines ~71/97/71 depending on the
    // pinned tag's exact line numbering - real value confirmed identical
    // across builds): MAGE_P_NSE_DEFAULT = 1.0E-03f, MAGB_P_NSE_DEFAULT =
    // 1.0E-04f. These back AP_NavEKF3.h's _magEarthProcessNoise/
    // _magBodyProcessNoise (AP_GROUPINFO "MAGE_P_NSE"/"MAGB_P_NSE") - REAL,
    // DISTINCT parameters from mag_noise/MAG_M_NSE above (that one seeds
    // covariance_init()'s P[16..21] diagonal and R_MAG in mag fusion; these
    // two instead drive covariance_prediction()'s per-tick mag process
    // noise) - verified NOT to be the same upstream field before adding
    // these as new, separate members.
    ftype mag_earth_process_noise = static_cast<ftype>(1.0e-03f);  // MAGE_P_NSE_DEFAULT
    ftype mag_body_process_noise = static_cast<ftype>(1.0e-04f);   // MAGB_P_NSE_DEFAULT

    // CPP-065 phase 11. upstream: AP_NavEKF3.cpp APM_BUILD_ArduPlane
    // #elif block, WIND_P_NSE_DEFAULT = 0.1 (verified directly - NOT the
    // Copter-only 0.2 value that appears in a different #if branch of the
    // same file); backs AP_NavEKF3.h's _windVelProcessNoise (AP_GROUPINFO
    // "WIND_P_NSE"). wind_var_hgt_rate_scale below backs
    // _wndVarHgtRateScale (AP_GROUPINFO "WIND_PSCALE", a literal 1.0f
    // argument, NOT vehicle-dependent - verified directly, same value in
    // every APM_BUILD_TYPE).
    ftype wind_vel_process_noise = static_cast<ftype>(0.1f);        // WIND_P_NSE_DEFAULT
    ftype wind_var_hgt_rate_scale = static_cast<ftype>(1.0f);       // AP_GROUPINFO("WIND_PSCALE", ..., 1.0f)

    // CPP-057 phase 3: real VEL_I_GATE_DEFAULT/POS_I_GATE_DEFAULT gate
    // parameters (AP_NavEKF3.cpp _gpsVelInnovGate/_gpsPosInnovGate,
    // AP_Int16 "Percentage number of standard deviations" - hence the
    // _pct suffix), verified identical (500/500) across every
    // APM_BUILD_TYPE #elif block including ArduPlane's own (~line 34-39,
    // 60-65, 86-91, 112-117). Same "AP_Param not wired in yet" treatment
    // as this file's other noise/limit parameters above.
    ftype gps_vel_innov_gate_pct = static_cast<ftype>(500);  // VEL_I_GATE_DEFAULT
    ftype gps_pos_innov_gate_pct = static_cast<ftype>(500);  // POS_I_GATE_DEFAULT

    // CPP-060 phase 6: real MAG_I_GATE_DEFAULT gate parameter
    // (AP_NavEKF3.cpp _magInnovGate, AP_Int16 "Percentage number of
    // standard deviations applied to magnetometer innovation consistency
    // check" - hence the _pct suffix, same convention as the GPS gates
    // above), verified identical (300) across every APM_BUILD_TYPE #elif
    // block including ArduPlane's own (~line 37, 63, 89, 115). Same
    // "AP_Param not wired in yet" treatment as this file's other noise/
    // limit parameters.
    ftype mag_innov_gate_pct = static_cast<ftype>(300);  // MAG_I_GATE_DEFAULT

    // CPP-062 phase 8: real HGT_I_GATE_DEFAULT gate parameter (AP_NavEKF3.cpp
    // _hgtInnovGate, AP_Int16 "Percentage number of standard deviations" -
    // same _pct convention as the GPS/mag gates above), verified identical
    // (500) across every APM_BUILD_TYPE #elif block including ArduPlane's
    // own (~line 39, 65, 91, 117) - matching VEL_I_GATE_DEFAULT/
    // POS_I_GATE_DEFAULT's own 500, DIFFERENT from MAG_I_GATE_DEFAULT's 300.
    // Same "AP_Param not wired in yet" treatment as this file's other gate
    // parameters. See this file's "CPP-062, PHASE 8" banner.
    ftype hgt_innov_gate_pct = static_cast<ftype>(500);  // HGT_I_GATE_DEFAULT

    // CPP-063 phase 9: real EAS_M_NSE_DEFAULT/EAS_I_GATE_DEFAULT parameters
    // (AP_NavEKF3.cpp _easNoise/_tasInnovGate, ~line 274/282) - see this
    // file's "CPP-063, PHASE 9" banner "A REAL, VERIFIED CONSTANT-
    // DEFINITION-STYLE DISTINCTION" for why, unlike every other gate default
    // in this file, these are LITERAL AP_GROUPINFO default arguments, not
    // #define macros repeated across APM_BUILD_TYPE blocks. eas_noise is
    // used directly as true-airspeed noise (EAS2TAS assumed 1.0 - see
    // banner "EAS2TAS - NOT MODELED").
    ftype eas_noise = static_cast<ftype>(1.4f);          // EAS_M_NSE_DEFAULT
    ftype tas_innov_gate_pct = static_cast<ftype>(400);  // EAS_I_GATE_DEFAULT

    // CPP-062 phase 8: upstream lastHgtPassTime_ms (AP_NavEKF3_core.h - the
    // same bookkeeping family as lastVelPassTime_ms/lastGpsPosPassTime_ms
    // above), elapsed-time bookkeeping used to detect a sustained baro
    // outage (see this file's "CPP-062, PHASE 8" banner). Seconds, same
    // caller-supplied-time convention as last_vel_pass_time_s/
    // last_pos_pass_time_s - stamped from fuse_baro_height()'s own `now_s`
    // parameter, never a real-time-clock read.
    ftype last_hgt_pass_time_s = 0;  // upstream: lastHgtPassTime_ms

    // upstream: NavEKF3_core::InitialGyroBiasUncertainty(),
    // AP_NavEKF3_GyroBias.cpp - a fixed 2.5 deg/sec, not vehicle-specific
    // despite its own doc comment claiming otherwise (read directly:
    // `return 2.5f;`, unconditional).
    ftype initial_gyro_bias_uncertainty_deg_s = static_cast<ftype>(2.5f);

    // upstream: NavEKF3_core::CovarianceInit(), AP_NavEKF3_core.cpp ~line
    // 573. Requires state.quat already set (e.g. from an initial attitude
    // estimate) - matches upstream reading stateStruct.quat directly, not
    // a parameter to CovarianceInit() there either. dt_ekf_avg: see file
    // banner's simplification 6.
    void covariance_init(ftype dt_ekf_avg);

    // upstream: NavEKF3_core::UpdateStrapdownEquationsNED(),
    // AP_NavEKF3_core.cpp ~line 743, PLUS the ConstrainStates() call at
    // its end (~line 793 upstream: `ConstrainStates();`). dt_ekf_avg is
    // needed here too, for ConstrainStates()'s gyro/accel bias limits
    // (`GYRO_BIAS_LIMIT*dtEkfAvg`, `_accBiasLim*dtEkfAvg`).
    void update_strapdown_equations_ned(const GyroSample& gyro, const AccelSample& accel, ftype dt_ekf_avg);

    // upstream: NavEKF3_core::CovariancePrediction(Vector3F*
    // rotVarVecPtr), AP_NavEKF3_core.cpp ~line 1008, PLUS the
    // ConstrainVariances() call at its end (~line 783). `rot_var_vec`
    // non-null reproduces upstream's quaternion-covariance-RESET path
    // (upstream's only caller: CovarianceInit()); null is the normal
    // per-tick prediction path (upstream's only caller: UpdateFilter()).
    void covariance_prediction(const GyroSample& gyro, const AccelSample& accel, ftype dt_ekf_avg,
                                const Vector3F* rot_var_vec = nullptr);

    // CPP-056 phase 2. upstream: FuseVelPosNED()'s obsIndex loop body,
    // AP_NavEKF3_PosVelFusion.cpp ~line 1024-1163 - see this file's
    // banner for exactly what is/isn't reproduced. `state_index` is the
    // observed state's own index (upstream: `stateIndex = 4 + obsIndex`,
    // computed by the caller here rather than derived from an obsIndex,
    // since this primitive is meant to be reusable beyond GPS obsIndex
    // 0-4 too - e.g. a future height-fusion phase's obsIndex==5, state
    // index 9). `innovation` is upstream's innovVelPos[obsIndex]
    // (observed convention: state MINUS observation, matching upstream's
    // own `stateStruct.velocity[i] - velPosObs[i]` - NOT the more common
    // observation-minus-prediction sign). `obs_variance` is upstream's
    // R_OBS[obsIndex]. `dt_ekf_avg` is required only because this
    // primitive calls the existing constrain_variances(ftype) - upstream
    // doesn't need to pass it because ConstrainVariances() reads its own
    // dtEkfAvg member directly; this port's explicit-input convention
    // (ADR-0012) has no such member to read, hence the extra parameter
    // versus the ticket's own suggested 3-argument signature.
    //
    // Returns whether the update was applied (upstream's local
    // `healthyFusion` bool) - false means the negative-variance guard
    // fired and P/state were left untouched, matching upstream's own
    // "skip the update" branch exactly (see banner: this is the ONLY
    // fusion-health check in scope for this phase, not full innovation-
    // consistency gating).
    bool fuse_direct_state_observation(int state_index, ftype innovation, ftype obs_variance, ftype dt_ekf_avg);

    // CPP-057 phase 3. upstream: velTestRatio/posTestRatio, the real
    // innovation-consistency test-ratio formulas FuseVelPosNED() uses to
    // decide fuseVelData/fusePosData (AP_NavEKF3_PosVelFusion.cpp ~line
    // 875-901 velocity, ~line 806-816 position) - see this file's
    // "CPP-057, PHASE 3" banner for the full formula derivation and what
    // is/isn't reproduced. Public (like gps_*_obs_variance()) so tests
    // can verify the exact formula independently. Pass if the returned
    // ratio is < 1.0 (upstream: `if (velTestRatio < 1.0)` / `if
    // (posTestRatio < 1.0f || ...)`, the AID_NONE disjunct of which does
    // not apply here - see banner).
    [[nodiscard]] ftype gps_vel_test_ratio(const GpsSample& gps) const;
    [[nodiscard]] ftype gps_pos_test_ratio(const GpsSample& gps) const;

    // CPP-058 phase 4. upstream: NavEKF3_core::ResetVelocity()/
    // ResetPosition(), AP_NavEKF3_PosVelFusion.cpp lines 14-89/94-186 -
    // reduced to the state+covariance portion of the real
    // AID_ABSOLUTE/GPS-source branch. See this file's "CPP-058, PHASE 4"
    // banner for the full scope reduction, the verified variance-formula
    // degeneration, and a real correction found in reset_position()'s
    // case (the ticket's stated formula matches ResetPosition() itself,
    // but NOT the further caller-side override upstream's real
    // timeout-triggered call site applies - deliberately not followed,
    // see banner). Unconditionally OVERWRITES state.velocity.x/y (resp.
    // position.x/y) from the given GpsSample and re-seeds P accordingly -
    // NOT a Kalman blend, a direct reset. Also stamps
    // last_vel_pass_time_s/last_pos_pass_time_s = now_s, matching
    // upstream's own ResetVelocity()/ResetPosition() doing the same at
    // their own end (~line 74, ~line 184). Public (like
    // fuse_direct_state_observation()) so tests can verify the reset
    // behavior directly.
    void reset_velocity(const GpsSample& gps, ftype now_s);
    void reset_position(const GpsSample& gps, ftype now_s);

    // CPP-056 phase 2. upstream: FuseVelPosNED()'s obsIndex 0-2 path
    // (innovation ~line 1011-1014; R_OBS "no reported accuracy" branch
    // ~line 741-742) - fuses the 3 GPS velocity axes sequentially against
    // `gps.velocity_ned`, using the CURRENT state for each axis's
    // innovation (matching upstream: obsIndex 1's innovation is computed
    // AFTER obsIndex 0's state correction has already been applied,
    // since correlated off-diagonal terms mean fusing velN can shift the
    // velE/velD state estimate too). Returns the number of axes (0-3)
    // whose healthyFusion guard passed - phase 1/this phase has no
    // faultStatus.bad_nvel/bad_evel/bad_dvel bookkeeping to report this
    // through instead.
    //
    // CPP-057 phase 3 ADDENDUM: now gated by gps_vel_test_ratio() BEFORE
    // any axis is fused - if the combined test ratio is >= 1.0, this
    // returns 0 immediately and P/state are left completely untouched
    // (upstream: `else { fuseVelData = false; }`, ~line 928). The
    // per-axis sequential-fusion behavior described above is otherwise
    // unchanged and only runs at all once the gate has passed.
    //
    // CPP-058 phase 4 ADDENDUM: `now_s` is this port's caller-supplied
    // "current time" (seconds, ADR-0012 - see this file's "CPP-058,
    // PHASE 4" banner), defaulted to 0 so existing phase 2/3 call sites
    // keep compiling and behaving identically. On a gate failure, the
    // elapsed time since last_vel_pass_time_s is now checked against the
    // real 10.0s posRetryTimeUseVel_ms threshold BEFORE returning 0 - if
    // timed out, reset_velocity() is called instead (an unconditional
    // state/covariance overwrite, NOT a fusion - matches upstream's own
    // "don't fuse the same data used to reset states") and this still
    // returns 0 (no axis was fused this call). If not timed out, this is
    // byte-for-byte the phase-3 behavior: state/P untouched, returns 0.
    // On a gate PASS, last_vel_pass_time_s is stamped to now_s only if
    // at least one axis actually fused (n_fused > 0) - see banner's "A
    // REAL, NAMED DIVERGENCE" note for why this is slightly stricter
    // than upstream's own gate-pass-only condition.
    int fuse_gps_velocity(const GpsSample& gps, ftype dt_ekf_avg, ftype now_s = ftype(0));

    // CPP-056 phase 2. upstream: FuseVelPosNED()'s obsIndex 3-4 path
    // (innovation ~line 1016; R_OBS "no reported accuracy" branch ~line
    // 750-756). Fuses the 2 GPS horizontal position axes sequentially
    // against `gps.position_ne`. Returns the number of axes (0-2) fused.
    //
    // CPP-057 phase 3 ADDENDUM: now gated by gps_pos_test_ratio() BEFORE
    // any axis is fused - if the combined test ratio is >= 1.0, this
    // returns 0 immediately and P/state are left completely untouched
    // (upstream: `else { fusePosData = false; }`, ~line 859).
    //
    // CPP-058 phase 4 ADDENDUM: same `now_s`/timeout/reset wiring as
    // fuse_gps_velocity() above, using reset_position() and
    // last_pos_pass_time_s instead - see that function's doc comment and
    // this file's "CPP-058, PHASE 4" banner for the full detail.
    int fuse_gps_position(const GpsSample& gps, ftype dt_ekf_avg, ftype now_s = ftype(0));

    // CPP-067 phase 13. upstream: NavEKF3_core::readGpsData()'s own
    // `storedGPS.push(gpsDataNew)` call (AP_NavEKF3_Measurements.cpp
    // ~line 735, verified directly) - the real, disclosed new capability
    // this ticket adds: callers push GPS samples into gps_buffer AS THEY
    // ARRIVE, decoupled from the EKF's own per-tick fusion cadence,
    // instead of having to hand a fusion call exactly the right sample
    // synchronously every time (today's only option, still available
    // unchanged via the direct-sample fuse_gps_velocity()/
    // fuse_gps_position() overloads above - see this file's "CPP-067,
    // PHASE 13" banner for why both old and new paths are kept). The
    // caller must stamp `sample`'s timestamp via GpsSample::set_time_s()
    // before calling this (matching upstream's own `gpsDataNew.time_ms =
    // ...; ... storedGPS.push(gpsDataNew)` sequence - assign the field,
    // then push); this function does not stamp it itself.
    void push_gps_sample(const GpsSample& sample);

    // CPP-067 phase 13. upstream: NavEKF3_core::SelectVelPosFusion()'s own
    // `gpsDataToFuse = storedGPS.recall(gpsDataDelayed,
    // imuDataDelayed.time_ms)` (AP_NavEKF3_PosVelFusion.cpp ~line 537) -
    // recalls the correct buffered sample by timestamp using `now_s`,
    // THIS PORT'S OWN CALLER-SUPPLIED CURRENT TIME (ADR-0012/CPP-058
    // convention), NOT upstream's delayed IMU time horizon
    // (imuDataDelayed.time_ms), which does not exist in this port (the
    // full delayed-state redesign is deliberately out of scope - see
    // this file's "CPP-067, PHASE 13" banner). This is a real, disclosed,
    // narrower simplification relative to upstream's true delayed-horizon
    // recall, and still a genuine improvement over the fully-synchronous,
    // hand-fed-only behavior that exists without it.
    //
    // WHY THIS IS ONE COMBINED RECALL PRIMITIVE, NOT TWO INDEPENDENT
    // "fuse_gps_velocity_buffered()"/"fuse_gps_position_buffered()"
    // METHODS (a shape considered and rejected): ObsBuffer::recall() is
    // DESTRUCTIVE (ekf_buffer.hpp) - once a sample is recalled it is
    // gone. Upstream itself calls storedGPS.recall() EXACTLY ONCE per
    // tick and feeds that ONE recalled gpsDataDelayed to BOTH the
    // velocity (obsIndex 0-2) and position (obsIndex 3-4) paths inside
    // FuseVelPosNED() - never two separate recalls. Two independently-
    // recalling wrapper methods would each try to drain the SAME shared
    // queue: calling both in one tick could see the second call find the
    // buffer already emptied by the first, or - worse - silently recall
    // two DIFFERENT samples for what should be one consistent
    // observation. Instead, recall_gps_sample() is called ONCE per tick
    // and its result fed to the existing, UNCHANGED
    // fuse_gps_velocity()/fuse_gps_position() overloads above for both
    // axis groups - matching upstream's real recall-once-fuse-twice
    // structure exactly, and meaning zero of this port's existing
    // fuse_gps_velocity()/fuse_gps_position() call sites (~20 across
    // ekf_fusion_test.cpp/ekf_closed_loop_test.cpp) need to change.
    //
    // Returns false (leaving `out` untouched) if gps_buffer has no
    // sample within its hardcoded 100ms window of `now_s`
    // (ekf_buffer.hpp's own upstream-derived recall() window) - matching
    // upstream's own control flow exactly: SelectVelPosFusion()'s
    // `gpsDataToFuse` false skips the ENTIRE fuseVelData/fusePosData
    // decision for the tick, not merely "the gate failed". A sustained
    // total GPS outage (buffer perpetually empty) therefore will NOT
    // reach - and will not spuriously trigger - the now_s/timeout-driven
    // reset_velocity()/reset_position() wiring inside
    // fuse_gps_velocity()/fuse_gps_position() (CPP-058): that wiring only
    // fires when a sample IS recalled but then fails the innovation
    // gate. A real, sustained outage is handled upstream by
    // gpsCheckStatus/dead-reckoning aiding-mode-switch machinery this
    // port does not model at all (an already-established exclusion, not
    // a new gap this ticket introduces - see this file's own "OUT OF
    // SCOPE" precedent).
    bool recall_gps_sample(GpsSample& out, ftype now_s);

    // CPP-056 phase 2. upstream: FuseVelPosNED()'s R_OBS[0]/[1]/[2] "no
    // reported accuracy" formula, AP_NavEKF3_PosVelFusion.cpp ~line
    // 740-742 - `sq(constrain_ftype(noise, 0.05f, 5.0f)) +
    // sq(accScale*accNavMag)`, accNavMag = velDotNEDfilt.length()
    // (AP_NavEKF3_core.cpp ~line 772; this port's own vel_dot_ned_filt,
    // populated by update_strapdown_equations_ned()). Public (not a
    // fuse_gps_velocity() implementation detail) so tests can verify the
    // exact formula independently, per the ticket's verification
    // standard.
    [[nodiscard]] ftype gps_horiz_vel_obs_variance() const;
    [[nodiscard]] ftype gps_vert_vel_obs_variance() const;

    // CPP-056 phase 2. upstream: FuseVelPosNED()'s R_OBS[3]/[4] "no
    // reported accuracy" formula, ~line 750-756 -
    // `sq(constrain_ftype(_gpsHorizPosNoise, 0.1f, 10.0f)) +
    // sq(gpsPosVarAccScale*accNavMag)`.
    [[nodiscard]] ftype gps_horiz_pos_obs_variance() const;

    // CPP-060 phase 6. upstream: magTestRatio/magHealth, AP_NavEKF3_
    // MagFusion.cpp ~line 571-577 - `magTestRatio[i] = sq(innovMag[i]) /
    // (sq(MAX(0.01*_magInnovGate, 1.0)) * varInnovMag[i])` per axis,
    // `magHealth = (magTestRatio[0]<1.0 && magTestRatio[1]<1.0 &&
    // magTestRatio[2]<1.0)` - ALL THREE axes individually, NOT a single
    // combined sum-of-squares ratio the way gps_vel_test_ratio()/gps_pos_
    // test_ratio() work (see this file's "CPP-060, PHASE 6" banner).
    // Reads this object's own stored innov_mag/var_innov_mag (set once
    // per fuse_magnetometer() call, before this gate runs) and
    // mag_innov_gate_pct - unlike gps_vel_test_ratio()/gps_pos_test_ratio(),
    // which take a fresh sample and recompute innovations themselves,
    // this phase's innovations are already members by the time the gate
    // needs them (see fuse_magnetometer()'s own body). Public so tests
    // can verify the exact per-axis formula independently, same
    // treatment as the GPS gates above.
    [[nodiscard]] Vector3F mag_test_ratio() const;

    // CPP-059 phase 5 (CPP-060 phase 6 ADDENDUM below). upstream:
    // NavEKF3_core::FuseMagnetometer(), AP_NavEKF3_MagFusion.cpp ~line
    // 473-843 - see this file's "CPP-059, PHASE 5" banner for the full
    // scope, the verbatim-transcription rationale, and the
    // CovarianceInit()-on-failure behavior this reproduces. `gyro`
    // supplies R_MAG's angular-rate-scaling term (upstream:
    // imuDataDelayed.delAng/delAngDT - this port's existing GyroSample
    // already carries exactly those two fields, no new type needed).
    //
    // Returns false on any of THREE distinct outcomes (see this file's
    // "CPP-060, PHASE 6" banner for the third):
    //   1. A badly-conditioned axis (varInnovMag[i] < R_MAG) - P has
    //      ALREADY been reset via covariance_init() by the time this
    //      returns.
    //   2. A failed healthyFusion guard - P has ALREADY been reset via
    //      covariance_init() by the time this returns.
    //   3. (CPP-060) A failed mag_test_ratio()/magHealth gate - P and
    //      state are COMPLETELY UNTOUCHED (upstream's own bare `return;`,
    //      no CovarianceInit()) - do NOT treat this the same as 1/2.
    // Returns true only if the gate passed AND all 3 axes (X, Y, Z, fused
    // sequentially) completed without either covariance-reset abort path
    // firing.
    bool fuse_magnetometer(const MagSample& mag, const GyroSample& gyro, ftype dt_ekf_avg);

    // CPP-068 phase 14. upstream: NavEKF3_core::readMagData()'s own
    // `storedMag.push(magDataNew);` (AP_NavEKF3_Measurements.cpp ~line
    // 377, verified directly) - the SAME new capability CPP-067 (phase
    // 13) added for GPS, now for magnetometer: callers push MagSample
    // readings into mag_buffer AS THEY ARRIVE, decoupled from the EKF's
    // own per-tick fusion cadence, instead of having to hand
    // fuse_magnetometer() exactly the right sample synchronously every
    // time (today's only option, still available unchanged - see this
    // file's "CPP-068, PHASE 14" banner above mag_buffer for why both old
    // and new paths are kept). The caller must stamp `sample`'s
    // timestamp via MagSample::set_time_s() before calling this
    // (matching upstream's own `magDataNew.time_ms = ...;
    // storedMag.push(magDataNew)` sequence); this function does not
    // stamp it itself.
    void push_mag_sample(const MagSample& sample);

    // CPP-068 phase 14. upstream: NavEKF3_core::SelectMagFusion()'s own
    // `magDataToFuse = storedMag.recall(magDataDelayed,
    // imuDataDelayed.time_ms);` (AP_NavEKF3_MagFusion.cpp ~line 411,
    // verified directly) - recalls the correct buffered sample by
    // timestamp using `now_s`, THIS PORT'S OWN CALLER-SUPPLIED CURRENT
    // TIME (ADR-0012/CPP-058 convention, identical to
    // recall_gps_sample()'s own now_s-vs-imuDataDelayed.time_ms
    // simplification - see this file's "CPP-068, PHASE 14" banner).
    //
    // Unlike recall_gps_sample() (which feeds ONE recalled sample to BOTH
    // fuse_gps_velocity()/fuse_gps_position()), magnetometer fusion has
    // only ONE consumer of the recalled sample (fuse_magnetometer()
    // itself) - so there is no "two independent callers draining the
    // same queue" hazard to design around here the way CPP-067 had to for
    // GPS. This is still deliberately a SEPARATE, ADDITIVE primitive
    // rather than a change to fuse_magnetometer()'s own signature/body,
    // matching CPP-067's own kept-vs-replace reasoning exactly: zero of
    // this port's existing fuse_magnetometer() call sites (ekf_mag_
    // fusion_test.cpp, ekf_closed_loop_test.cpp) need to change.
    //
    // Returns false (leaving `out` untouched) if mag_buffer has no
    // sample within its hardcoded 100ms window of `now_s` (ekf_buffer.hpp's
    // own upstream-derived recall() window) - matching upstream's own
    // control flow: SelectMagFusion()'s `magDataToFuse` false means
    // `dataReady` is false, so the entire per-tick magnetometer fusion
    // decision (including the FUSE_YAW/FUSE_MAG3D branch selection) is
    // skipped for the tick, not merely "the gate failed inside
    // fuse_magnetometer()".
    bool recall_mag_sample(MagSample& out, ftype now_s);


    // CPP-062 phase 8. upstream: R_OBS[5]'s baro branch,
    // selectHeightForFusion(), AP_NavEKF3_PosVelFusion.cpp ~line 1376-1377 -
    // `posDownObsNoise = sq(constrain_ftype(frontend->_baroAltNoise, 0.1f,
    // 100.0f))`. Reuses the already-existing baro_alt_noise field (phase 1,
    // ALT_M_NSE_DEFAULT=3.0 for ArduPlane) rather than adding a second noise
    // field - see this file's "CPP-062, PHASE 8" banner. Public (like the
    // GPS/mag obs_variance() functions) so tests can verify it
    // independently.
    [[nodiscard]] ftype baro_hgt_obs_variance() const;

    // CPP-062 phase 8. upstream: hgtTestRatio, AP_NavEKF3_PosVelFusion.cpp
    // ~line 934, `sq(innovVelPos[5]) / (sq(MAX(0.01*_hgtInnovGate,1.0)) *
    // varInnovVelPos[5])` - see this file's "CPP-062, PHASE 8" banner for
    // the full sign-convention derivation of innovVelPos[5] from
    // `baro_altitude_m` (upstream's positive-up hgtMea convention) and for
    // why this recomputes fresh from the given reading (GPS-gate shape)
    // rather than reading stored members (mag-gate shape). Pass if the
    // returned ratio is < 1.0 (upstream: `if (hgtTestRatio < maxTestRatio)`
    // with maxTestRatio unconditionally 1.0 here - the onGroundNotNavigating
    // 3.0 relaxation does not apply, see banner).
    [[nodiscard]] ftype hgt_test_ratio(ftype baro_altitude_m) const;

    // CPP-062 phase 8. upstream: NavEKF3_core::ResetHeight(),
    // AP_NavEKF3_PosVelFusion.cpp lines 287-355 - reduced to ONLY
    // state.position.z + P[9][9], matching reset_position()'s/
    // reset_velocity()'s own established per-axis reduction (CPP-058). See
    // this file's "CPP-062, PHASE 8" banner ("A REAL DIVERGENCE FOUND IN
    // ResetHeight()'S OWN BODY") for a real, verified divergence found and
    // deliberately NOT followed here: upstream's real ResetHeight() ALSO
    // unconditionally resets P[6][6] (vertical velocity covariance) and
    // conditionally overwrites state.velocity.z, gated on state
    // (inFlight/gpsIsInUse/onGround) this port does not model. Directly
    // overwrites state.position.z from the given `baro_altitude_m`
    // (upstream: `stateStruct.position.z = -hgtMea;`) and re-seeds P[9][9]
    // to baro_hgt_obs_variance() (upstream: `P[9][9] = posDownObsNoise;`).
    // Stamps last_hgt_pass_time_s = now_s, matching upstream's own
    // `lastHgtPassTime_ms = imuSampleTime_ms;` at ResetHeight()'s own
    // timeout-clearing line (~line 316-317) - a reset counts as "just
    // passed", so the timeout clock restarts from here, same reasoning as
    // reset_velocity()/reset_position().
    void reset_height(ftype baro_altitude_m, ftype now_s);

    // CPP-062 phase 8. upstream: FuseVelPosNED()'s obsIndex==5 path, wired
    // exactly like fuse_gps_velocity()/fuse_gps_position() (CPP-057/
    // CPP-058): gated by hgt_test_ratio() BEFORE any state/P is touched - a
    // failing gate (ratio >= 1.0) leaves P/state completely untouched
    // (upstream: `else { fuseHgtData = false; }`, ~line 979), UNLESS the
    // elapsed time since last_hgt_pass_time_s has reached the real 10.0s
    // hgtRetryTimeMode0_ms timeout (a NEW, separate constant from
    // fuse_gps_velocity()/fuse_gps_position()'s own kGpsFusionTimeoutS - see
    // this file's "CPP-062, PHASE 8" banner "A REAL CONSTANT-IDENTITY
    // CHECK" for why these are not the same constant despite sharing a
    // value), in which case reset_height() is called instead (not a fusion -
    // matches upstream's own `ResetHeight(); fuseHgtData = false;`, ~line
    // 975-977). On a gate pass, fuses state_index=9 via
    // fuse_direct_state_observation() and stamps last_hgt_pass_time_s only
    // if the fusion actually applied - same "gate passed AND healthyFusion
    // passed" stricter convention as fuse_gps_velocity()/fuse_gps_position()
    // (CPP-058's own named divergence from upstream's gate-pass-only
    // condition). Returns a plain bool, NOT an axes-fused count or a
    // three-outcome collapse - see banner's "RETURN TYPE" note for why
    // that's the honest shape for a single-obsIndex primitive with exactly
    // one failure mode.
    bool fuse_baro_height(ftype baro_altitude_m, ftype dt_ekf_avg, ftype now_s = ftype(0));

    // CPP-063 phase 9. upstream: tasTestRatio, AP_NavEKF3_AirDataFusion.cpp
    // ~line 108-110 - `sq(innovVtas) / (sq(MAX(0.01*_tasInnovGate,1.0)) *
    // varInnovVtas)`. See this file's "CPP-063, PHASE 9" banner ("tas_test_
    // ratio() IS MAG-SHAPED, NOT GPS/BARO-SHAPED") for why this reads this
    // object's own already-populated innov_vtas/var_innov_vtas (set by
    // fuse_airspeed() itself before this gate needs them), matching
    // mag_test_ratio()'s convention, not gps_vel_test_ratio()'s/
    // hgt_test_ratio()'s recompute-fresh convention. Public so tests can
    // verify the exact formula independently, same treatment as this file's
    // other gate formulas.
    [[nodiscard]] ftype tas_test_ratio() const;

    // CPP-063 phase 9. upstream: NavEKF3_core::FuseAirspeed(),
    // AP_NavEKF3_AirDataFusion.cpp lines ~20-156 - see this file's "CPP-063,
    // PHASE 9" banner for the full scope, the verbatim-transcription
    // rationale, the airDataFusionWindOnly-provably-always-false finding,
    // and "THE REAL, THREE-WAY OUTCOME SHAPE" for exactly what each of the
    // three false-returning paths does and does not touch.
    //
    // `true_airspeed_m_s`: a bare scalar sensor reading, matching
    // baro_altitude_m's own established convention (see banner "tas_
    // reading: BARE ftype, NOT A NEW STRUCT") - taken as already being TRUE
    // airspeed (EAS2TAS assumed 1.0, see banner "EAS2TAS - NOT MODELED",
    // reusing TECS's own already-disclosed identical simplification).
    //
    // Returns true only if the gate passed AND the correction was actually
    // applied (VtasPred > 1.0 AND not badly-conditioned AND tasTestRatio <
    // 1.0) - false on any of the three distinct outcomes documented in the
    // banner. Unlike fuse_gps_velocity()/fuse_gps_position()/
    // fuse_baro_height(), there is NO now_s/timeout/reset parameter or
    // mechanism at all here - verified directly that FuseAirspeed() contains
    // no Reset*() call anywhere (see banner) - so this signature has no
    // `now_s` parameter and this class has no last_tas_pass_time_s field;
    // adding either would invent a mechanism that does not exist upstream.
    bool fuse_airspeed(ftype true_airspeed_m_s, ftype dt_ekf_avg);

private:
    void constrain_states(ftype dt_ekf_avg);   // upstream: NavEKF3_core::ConstrainStates()
    void constrain_variances(ftype dt_ekf_avg); // upstream: NavEKF3_core::ConstrainVariances()

    // CPP-056 phase 2. upstream: NavEKF3_core::updateStateIndexLim(),
    // AP_NavEKF3_Control.cpp ~line 190-208 - the same nested inhibit-flag
    // logic, transcribed directly, reused here to bound
    // fuse_direct_state_observation()'s KHP/healthyFusion/P-update loops
    // exactly as upstream bounds them (not the full 0..23 every call).
    [[nodiscard]] int state_index_lim() const;

    // upstream: NavEKF3_core::ForceSymmetry(), AP_NavEKF3_core.cpp ~line
    // 1862 - averages P[i][j]/P[j][i] for i in 1..lim. Did not exist in
    // phase 1 (nothing needed it before fusion existed).
    void force_symmetry(int lim);

    // CPP-056 phase 2 helper (no direct upstream counterpart - upstream's
    // statesArray is a flat float[24] alias over the same memory as
    // stateStruct, so `statesArray[i] -= Kfusion[i]*innovation` IS its
    // state update; this port's StateVector is a structured type, so the
    // flat-index correction has to be unpacked field-by-field here
    // instead). Applies `state -= kfusion*innovation` across all 24
    // conceptual slots unconditionally, then re-normalizes the
    // quaternion (upstream: `stateStruct.quat.normalize()`,
    // unconditional, right after the state update loop). Safe to apply
    // unconditionally to inhibited slots (16..23 when mag/wind
    // inhibited) because kfusion[i] is exactly 0.0 there by construction
    // (fuse_direct_state_observation() only ever writes a nonzero value
    // into an index it has confirmed is not inhibited) - not a shortcut,
    // provably identical to bounding this loop at state_index_lim() too.
    void apply_state_correction(const std::array<ftype, 24>& kfusion, ftype innovation);
};

} // namespace fwcpp::ekf
