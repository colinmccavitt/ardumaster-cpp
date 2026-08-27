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

#include <array>
#include <cmath>
#include <cstdint>

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
struct GpsSample {
    Vector3F velocity_ned;  // upstream: gpsDataDelayed.vel, NED m/s
    Vector2F position_ne;   // upstream: velPosObs[3]/[4] source value, local NE metres
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
struct MagSample {
    Vector3F mag;  // upstream: magDataDelayed.mag, body-frame gauss
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

    // CPP-059 phase 5. upstream: innovMag/varInnovMag (NavEKF3_core
    // members) - see this file's "CPP-059, PHASE 5" banner. Public so
    // tests can verify the dense per-axis formulas independently, per the
    // ticket's own verification standard (same treatment as gps_vel_test_
    // ratio()/gps_pos_test_ratio() above).
    Vector3F innov_mag{};      // upstream: innovMag
    Vector3F var_innov_mag{};  // upstream: varInnovMag

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

    // CPP-057 phase 3: real VEL_I_GATE_DEFAULT/POS_I_GATE_DEFAULT gate
    // parameters (AP_NavEKF3.cpp _gpsVelInnovGate/_gpsPosInnovGate,
    // AP_Int16 "Percentage number of standard deviations" - hence the
    // _pct suffix), verified identical (500/500) across every
    // APM_BUILD_TYPE #elif block including ArduPlane's own (~line 34-39,
    // 60-65, 86-91, 112-117). Same "AP_Param not wired in yet" treatment
    // as this file's other noise/limit parameters above.
    ftype gps_vel_innov_gate_pct = static_cast<ftype>(500);  // VEL_I_GATE_DEFAULT
    ftype gps_pos_innov_gate_pct = static_cast<ftype>(500);  // POS_I_GATE_DEFAULT

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

    // CPP-059 phase 5. upstream: NavEKF3_core::FuseMagnetometer(),
    // AP_NavEKF3_MagFusion.cpp ~line 473-843 - see this file's "CPP-059,
    // PHASE 5" banner for the full scope, the verbatim-transcription
    // rationale, and the CovarianceInit()-on-failure behavior this
    // reproduces. `gyro` supplies R_MAG's angular-rate-scaling term
    // (upstream: imuDataDelayed.delAng/delAngDT - this port's existing
    // GyroSample already carries exactly those two fields, no new type
    // needed). Returns false if EITHER upstream abort path fired (a
    // badly-conditioned axis, or a failed healthyFusion guard) - P has
    // already been reset via covariance_init() by the time this returns
    // false, matching upstream's real CovarianceInit()-then-return
    // behavior exactly. Returns true only if all 3 axes (X, Y, Z, fused
    // sequentially) completed without either abort path firing.
    bool fuse_magnetometer(const MagSample& mag, const GyroSample& gyro, ftype dt_ekf_avg);

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
